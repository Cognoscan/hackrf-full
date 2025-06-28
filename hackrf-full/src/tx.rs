use std::sync::mpsc;

use crate::{Buffer, Error, HackRf, consts::TransceiverMode, error::StateChangeError};

/// A HackRF operating in transmit mode.
pub struct Transmit {
    rf: HackRf,
    queue: nusb::transfer::Queue<Vec<u8>>,
    max_transfer_size: usize,
    buf_pool: mpsc::Receiver<Vec<u8>>,
    buf_pool_send: mpsc::Sender<Vec<u8>>,
}

impl Transmit {
    /// Switch a HackRF into transmit mode.
    pub async fn new(rf: HackRf, max_transfer_size: usize) -> Result<Self, StateChangeError> {
        // Round up to nearest 512-byte increment
        if let Err(err) = rf.set_transceiver_mode(TransceiverMode::Transmit).await {
            return Err(StateChangeError { err, rf });
        }
        let max_transfer_size = (max_transfer_size.max(1) + 0x1FF) & !0x1FF;
        let queue = rf.interface.bulk_out_queue(0x02);
        let (buf_pool_send, buf_pool) = mpsc::channel();
        Ok(Self {
            rf,
            queue,
            max_transfer_size,
            buf_pool,
            buf_pool_send,
        })
    }

    /// Get a buffer for holding transmit data.
    ///
    /// All buffers have the same capacity, set when transmit is initialized.
    /// Actual transmitted data is rounded up to the nearest 256 samples,
    /// zero-filling as needed.
    pub fn get_buffer(&self) -> Buffer {
        if let Ok(mut buf) = self.buf_pool.try_recv() {
            buf.clear();
            Buffer::new(buf, self.buf_pool_send.clone())
        } else {
            Buffer::new(
                Vec::with_capacity(self.max_transfer_size),
                self.buf_pool_send.clone(),
            )
        }
    }

    /// Queue up a transmit transfer.
    ///
    /// This will pull from a reusable buffer pool first, and allocate a new
    /// buffer if none are available in the pool.
    ///
    /// The buffer pool will grow so long as completed buffers aren't dropped.
    pub fn submit(&mut self, tx: Buffer) {
        self.queue.submit(tx.into_vec());
    }

    /// Wait for a transmit operation to complete
    pub async fn next_complete(&mut self) -> Result<(), Error> {
        let result = self.queue.next_complete().await;
        let _ = self.buf_pool_send.send(result.data.reuse());
        Ok(result.status?)
    }

    /// Get the number of pending requests.
    pub fn pending(&self) -> usize {
        self.queue.pending()
    }

    /// Halt receiving and return to idle mode.
    pub async fn stop(self) -> Result<HackRf, Error> {
        self.rf.set_transceiver_mode(TransceiverMode::Off).await?;
        Ok(self.rf)
    }
}
