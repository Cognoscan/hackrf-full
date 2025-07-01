use nusb::transfer::RequestBuffer;

use crate::{Buffer, Error, HackRf, consts::TransceiverMode, error::StateChangeError};

/// A HackRF operating in receive mode.
///
/// To receive data, first take a HackRF peripheral and call
/// [`HackRf::start_rx`], or use [`Receive::new`] with it.
///
/// Next, call [`submit`][Receive::submit] to queue up requests, stopping when
/// there are enough pending requests. `libhackrf` queues up 1 MiB of data, or
/// 524288 samples. You'll probably want something similar, with the number of
/// pending requests informed by your chosen transfer block size.
///
/// Actual reception is done with [`next_complete`][Receive::next_complete],
/// which will panic if there are no pending requests. The number of pending
/// requests can always be checked with [`pending`][Receive::pending].
///
/// When finished receiving, call [`stop`][Receive::stop] to cancel all
/// remaining transactions and switch the HackRF off again.
///
/// Putting it all together, here's an example receive program that writes all
/// data to a file:
///
///
/// ```no_run
/// use std::sync::{Arc, atomic};
///
/// use anyhow::Result;
/// use tokio::io::AsyncWriteExt;
/// use waverave_hackrf::Buffer;
/// #[tokio::main]
/// async fn main() -> Result<()> {
///     // Set up the ctrl-c handler
///     let ctrlc_rx = Arc::new(atomic::AtomicBool::new(false));
///     let ctrlc_tx = ctrlc_rx.clone();
///     tokio::spawn(async move {
///         tokio::signal::ctrl_c().await.unwrap();
///         ctrlc_tx.store(true, atomic::Ordering::Release);
///     });
///
///     // Open up a file for buffered writing.
///     let mut args = std::env::args();
///     args.next();
///     let file_name = args.next().unwrap_or_else(|| String::from("./rx.bin"));
///     let mut file = tokio::fs::File::create(&file_name).await?;
///
///     // Open up the HackRF
///     let hackrf = waverave_hackrf::open_hackrf()?;
///
///     // Configure: 20MHz sample rate, turn on the RF amp, set IF & BB gains to 16 dB,
///     // and tune to 915 MHz.
///     hackrf.set_amp_enable(true).await?;
///     hackrf.set_sample_rate(20e6).await?;
///     hackrf.set_lna_gain(16).await?;
///     hackrf.set_vga_gain(16).await?;
///     hackrf.set_freq(915_000_000).await?;
///
///     // Start receiving, in bursts of 8192 samples
///     let mut hackrf_rx = hackrf.start_rx(8192).await.map_err(|e| e.err)?;
///
///     // Separate the file writer from the sample reader with a separate task
///     let (data_send, mut data_recv) = tokio::sync::mpsc::unbounded_channel::<Buffer>();
///     let file_writer = tokio::spawn(async move {
///         loop {
///             let Some(buf) = data_recv.recv().await else {
///                 break;
///             };
///             file.write_all_buf(&mut buf.bytes()).await?;
///         }
///         file.flush().await?;
///         Ok::<(), anyhow::Error>(())
///     });
///
///     // Queue up 64 transfers immediately, then start retrieving them until we
///     // get a ctrl-c.
///     for _ in 0..64 {
///         hackrf_rx.submit();
///     }
///     loop {
///         if ctrlc_rx.load(atomic::Ordering::Acquire) {
///             break;
///         }
///         if data_send.send(hackrf_rx.next_complete().await?).is_err() {
///             break;
///         }
///         hackrf_rx.submit();
///     }
///
///     // Stop receiving
///     hackrf_rx.stop().await?;
///     drop(data_send);
///
///     // Wait for file writer task to close up shop
///     file_writer.await??;
///
///     Ok(())
/// }
/// ```
pub struct Receive {
    rf: HackRf,
    transfer_size: usize,
}

impl Receive {
    /// Switch a HackRF into receive mode, getting `transfer_size` samples at a
    /// time. The transfer size is always rounded up to the nearest 256-sample
    /// block increment; it's recommended to be 8192 samples but can be smaller
    /// or larger as needed.
    pub async fn new(rf: HackRf, transfer_size: usize) -> Result<Self, StateChangeError> {
        if let Err(err) = rf.set_transceiver_mode(TransceiverMode::Receive).await {
            return Err(StateChangeError { err, rf });
        }
        // Go from samples to bytes, and round up to nearest 256-sample increment
        let transfer_size = (transfer_size.max(1) + 0xFF) & !0xFF;
        Ok(Self { rf, transfer_size })
    }

    /// Get the chosen transfer size, in samples.
    pub fn transfer_size(&self) -> usize {
        self.transfer_size
    }

    /// Queue up a receive transfer.
    ///
    /// This will pull from a reusable buffer pool first, and allocate a new
    /// buffer if none are available in the pool.
    ///
    /// The buffer pool will grow so long as completed buffers aren't dropped.
    pub fn submit(&mut self) {
        let req = if let Ok(buf) = self.rf.rx.buf_pool.try_recv() {
            RequestBuffer::reuse(buf, self.transfer_size * 2)
        } else {
            RequestBuffer::new(self.transfer_size * 2)
        };
        self.rf.rx.queue.submit(req);
    }

    /// Retrieve the next chunk of receive data.
    ///
    /// This future is cancel-safe, so feel free to use it alongside a timeout
    /// or a `select!`-type pattern.
    pub async fn next_complete(&mut self) -> Result<Buffer, Error> {
        let result = self.rf.rx.queue.next_complete().await;
        match result.status {
            Ok(_) => Ok(Buffer::new(result.data, self.rf.rx.buf_pool_send.clone())),
            Err(e) => {
                // Reuse the buffer even in the event of an error.
                let _ = self.rf.rx.buf_pool_send.send(result.data);
                Err(e.into())
            }
        }
    }

    /// Get the number of pending requests.
    pub fn pending(&self) -> usize {
        self.rf.rx.queue.pending()
    }

    /// Halt receiving and return to idle mode.
    ///
    /// This attempts to cancel all transfers and then complete whatever is
    /// left. Transfer errors are ignored.
    pub async fn stop(mut self) -> Result<HackRf, Error> {
        self.rf.rx.queue.cancel_all();
        while self.pending() > 0 {
            let _ = self.next_complete().await;
        }
        self.rf.set_transceiver_mode(TransceiverMode::Off).await?;
        Ok(self.rf)
    }
}
