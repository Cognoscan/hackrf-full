use crate::{Buffer, Error, HackRf, consts::TransceiverMode, error::StateChangeError};

/// A HackRF operating in transmit mode.
///
/// To send data, first take a HackRF peripheral and call
/// [`HackRf::start_tx`], or use [`Transmit::new`] with it. Provide the maximum
/// block transfer size, in samples, that will be used for the duration of the
/// transmit.
///
/// Next, call [`get_buffer`][Transmit::get_buffer] to allocate a buffer, and
/// then fill that buffer with the next block of samples to transmit, up to the
/// maximum block transfer size chosen. Set up around half a million samples
/// worth of buffers before submitting any, ideally, or gaps in the transmit
/// sequence may occur.
///
/// Next, start calling [`submit`][Transmit::submit] to submit the blocks.
/// Continue the buffer filling and submitting process as long as needed,
/// preferrably pausing with [`next_complete`][Transmit::next_complete] when
/// there a more than half a million samples worth of buffers queued up. The
/// number of queued buffers can be checked with [`pending`][Transmit::pending].
/// Once buffers are queued up, the process of completing, grabbing a buffer,
/// and submitting it in a loop until finished.
///
/// When all buffers have been submitted, call [`flush`][Transmit::flush] to
/// queue up a final set of zero-filled buffers to flush out any remaining data
/// in the HackRF's internal buffers. Then continue calling
/// [`next_complete`][Transmit::next_complete] until
/// [`pending`][Transmit::pending] returns 0. Finally, exit transmit mode with
/// [`stop`][Transmit::stop]. Transmit mode can be exited without doing this
/// sequence, but not all samples queued up will necessarily be sent otherwise.
///
/// Putting it all together, here's an example program that transmits all the
/// data in a file:
///
/// ```no_run
/// use anyhow::Result;
/// use tokio::io::AsyncReadExt;
/// use waverave_hackrf::Buffer;
/// #[tokio::main]
/// async fn main() -> Result<()> {
///     let hackrf = waverave_hackrf::open_hackrf()?;
///
///     // Configure: 20MHz sample rate, turn on RF amp, set TX IF gain to 16 dB,
///     // and tune to 915 MHz.
///     hackrf.set_sample_rate(20e6).await?;
///     hackrf.set_txvga_gain(16).await?;
///     hackrf.set_freq(915_000_000).await?;
///     hackrf.set_amp_enable(true).await?;
///
///     // Open up a file for buffered reading.
///     let mut args = std::env::args();
///     args.next();
///     let file_name = args.next().unwrap_or_else(|| String::from("./tx.bin"));
///     let mut file = tokio::fs::File::open(&file_name).await?;
///
///     // Start transmitting, in bursts of 8192 samples
///     let mut hackrf_tx = hackrf.start_tx(8192).await.map_err(|e| e.err)?;
///
///     // Set up an asynchronous process that fills buffers and sends them on to
///     // the transmitter.
///     let (buf_send, mut buf_recv) = tokio::sync::mpsc::channel::<Buffer>(4);
///     let (data_send, mut data_recv) = tokio::sync::mpsc::channel::<Buffer>(4);
///     tokio::spawn(async move {
///         loop {
///             let Some(mut buf) = buf_recv.recv().await else {
///                 break;
///             };
///             buf.extend_zeros(buf.remaining_capacity());
///             file.read_exact(buf.bytes_mut()).await?;
///             let Ok(_) = data_send.send(buf).await else {
///                 break;
///             };
///         }
///         Ok::<(), anyhow::Error>(())
///     });
///
///     // Start filling up queue
///     let mut start = Vec::with_capacity(64);
///     while start.len() < 64 {
///         while buf_send.try_send(hackrf_tx.get_buffer()).is_ok() {}
///         let Some(buf) = data_recv.recv().await else {
///             break;
///         };
///         start.push(buf);
///     }
///
///     // Submit the whole starting queue in one go
///     for buf in start {
///         hackrf_tx.submit(buf);
///     }
///
///     // Continue filling the queue and submitting samples as often as possible
///     loop {
///         while buf_send.try_send(hackrf_tx.get_buffer()).is_ok() {}
///         hackrf_tx.next_complete().await?;
///         let Some(buf) = data_recv.recv().await else {
///             break;
///         };
///         hackrf_tx.submit(buf);
///     }
///
///     // Flush the remainder
///     hackrf_tx.flush();
///     while hackrf_tx.pending() > 0 {
///         hackrf_tx.next_complete().await?;
///     }
///
///     // Stop transmitting
///     hackrf_tx.stop().await?;
///
///     Ok(())
/// }
/// ```
pub struct Transmit {
    rf: HackRf,
    max_transfer_size: usize,
}

impl Transmit {
    /// Switch a HackRF into transmit mode, with a set maximum number of samples
    /// per buffer block.
    ///
    /// Buffers are reused across transmit operations, provided that the
    /// `max_transfer_size` is always the same.
    pub async fn new(rf: HackRf, max_transfer_size: usize) -> Result<Self, StateChangeError> {
        if let Err(err) = rf.set_transceiver_mode(TransceiverMode::Transmit).await {
            return Err(StateChangeError { err, rf });
        }
        // Round up to nearest 256-sample increment
        let max_transfer_size = (max_transfer_size.max(1) + 0xFF) & !0xFF;
        Ok(Self {
            rf,
            max_transfer_size,
        })
    }

    /// Get a buffer for holding transmit data.
    ///
    /// All buffers have the same capacity, set when transmit is initialized.
    /// Actual transmitted data is rounded up to the nearest 256 samples,
    /// zero-filling as needed.
    pub fn get_buffer(&self) -> Buffer {
        if let Ok(mut buf) = self.rf.tx.buf_pool.try_recv() {
            buf.clear();
            buf.reserve_exact(self.max_transfer_size * 2);
            Buffer::new(buf, self.rf.tx.buf_pool_send.clone())
        } else {
            Buffer::new(
                Vec::with_capacity(self.max_transfer_size * 2),
                self.rf.tx.buf_pool_send.clone(),
            )
        }
    }

    /// The maximum number of samples that can be queued within a single buffer.
    pub fn max_transfer_size(&self) -> usize {
        self.max_transfer_size
    }

    /// Queue up a transmit transfer.
    ///
    /// This will pull from a reusable buffer pool first, and allocate a new
    /// buffer if none are available in the pool.
    ///
    /// The buffer pool will grow so long as completed buffers aren't dropped.
    pub fn submit(&mut self, tx: Buffer) {
        let mut tx = tx.into_vec();
        // Round up to nearest 512-byte block and zero-fill remaining
        let new_len = (tx.len() + 0x1ff) & !0x1ff;
        tx.resize(new_len, 0);
        self.rf.tx.queue.submit(tx);
    }

    /// Flush whatever remaining samples are in the HackRF internal buffer.
    ///
    /// This will generate additional pending operations.
    ///
    /// Additional pending operations go up by `8192.div_ceil(max_transfer_size)`.
    pub fn flush(&mut self) {
        // HackRF buffer depth, in samples (0x8000 bytes)
        const BUFFER_DEPTH: usize = 0x4000;
        let mut total_size = 0;
        while total_size < BUFFER_DEPTH {
            let buf_size = self.max_transfer_size.min(BUFFER_DEPTH - total_size);
            let mut buf = self.get_buffer();
            buf.extend_zeros(buf_size);
            self.submit(buf);
            total_size += buf_size;
        }
    }

    /// Wait for a transmit operation to complete.
    ///
    /// This future is cancel-safe, so feel free to use it alongside a timeout
    /// or a `select!`-type pattern.
    pub async fn next_complete(&mut self) -> Result<(), Error> {
        let result = self.rf.tx.queue.next_complete().await;
        let _ = self.rf.tx.buf_pool_send.send(result.data.reuse());
        Ok(result.status?)
    }

    /// Get the number of pending requests.
    pub fn pending(&self) -> usize {
        self.rf.tx.queue.pending()
    }

    /// Halt receiving and return to idle mode.
    ///
    /// This attempts to cancel all transfers and then complete whatever is
    /// left. Transfer errors are ignored.
    pub async fn stop(mut self) -> Result<HackRf, StateChangeError> {
        self.rf.tx.queue.cancel_all();
        while self.pending() > 0 {
            let _ = self.next_complete().await;
        }
        match self.rf.set_transceiver_mode(TransceiverMode::Off).await {
            Ok(_) => Ok(self.rf),
            Err(err) => Err(StateChangeError { err, rf: self.rf }),
        }
    }
}
