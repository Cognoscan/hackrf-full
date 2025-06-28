use std::sync::mpsc;

use nusb::transfer::RequestBuffer;

use crate::{Buffer, Error, HackRf, consts::TransceiverMode, error::StateChangeError};

/// A HackRF operating in receive mode.
pub struct Receive {
    rf: HackRf,
    queue: nusb::transfer::Queue<nusb::transfer::RequestBuffer>,
    transfer_size: usize,
    buf_pool: mpsc::Receiver<Vec<u8>>,
    buf_pool_send: mpsc::Sender<Vec<u8>>,
}

impl Receive {
    /// Switch a HackRF into receive mode.
    pub async fn new(rf: HackRf, transfer_size: usize) -> Result<Self, StateChangeError> {
        // Round up to nearest 512-byte increment
        if let Err(err) = rf.set_transceiver_mode(TransceiverMode::Receive).await {
            return Err(StateChangeError { err, rf });
        }
        let transfer_size = (transfer_size.max(1) + 0x1FF) & !0x1FF;
        let queue = rf.interface.bulk_in_queue(0x81);
        let (buf_pool_send, buf_pool) = mpsc::channel();
        Ok(Self {
            rf,
            queue,
            transfer_size,
            buf_pool,
            buf_pool_send,
        })
    }

    /// Queue up a receive transfer.
    ///
    /// This will pull from a reusable buffer pool first, and allocate a new
    /// buffer if none are available in the pool.
    ///
    /// The buffer pool will grow so long as completed buffers aren't dropped.
    pub fn submit(&mut self) {
        let req = if let Ok(buf) = self.buf_pool.try_recv() {
            RequestBuffer::reuse(buf, self.transfer_size)
        } else {
            RequestBuffer::new(self.transfer_size)
        };
        self.queue.submit(req);
    }

    /// Retrieve the next chunk of receive data.
    pub async fn next_complete(&mut self) -> Result<Buffer, Error> {
        let buf = self.queue.next_complete().await.into_result()?;
        Ok(Buffer::new(buf, self.buf_pool_send.clone()))
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
