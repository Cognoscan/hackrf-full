use nusb::transfer::{ControlOut, ControlType, Recipient, RequestBuffer, TransferError};

use crate::{consts::{ControlRequest, TransceiverMode}, Error, HackRf};

/// A HackRF operating in receive mode.
pub struct Receive {
    rf: HackRf,
    queue: nusb::transfer::Queue<nusb::transfer::RequestBuffer>,
    transfer_size: usize,
}

impl Receive {
    /// Switch a HackRF into receive mode.
    pub async fn new(rf: HackRf, transfer_size: usize) -> Result<Self, (HackRf, Error)> {
        // Round up to nearest 512-byte increment
        let transfer_size = (transfer_size.max(1) + 0x1FF) & !0x1FF;
        if let Err(e) = rf.set_transceiver_mode(TransceiverMode::Receive).await {
            return Err((rf, e));
        }
        let queue = rf.interface.bulk_in_queue(0x81);
        Ok(Self {
            rf,
            queue,
            transfer_size,
        })
    }

    /// Queue up a receive transfer. Up to 4 can be in flight at a time.
    pub fn submit(&mut self, buf: Option<Vec<u8>>) -> Result<(), Error> {
        if self.pending() >= 4 {
            return Err(Error::TransferBusy);
        }
        let req = if let Some(buf) = buf {
            RequestBuffer::reuse(buf, self.transfer_size)
        } else {
            RequestBuffer::new(self.transfer_size)
        };
        self.queue.submit(req);
        Ok(())
    }

    /// Retrieve the next chunk of receive data.
    pub async fn next_complete(&mut self) -> Result<Vec<u8>, Error> {
        let result = self.queue.next_complete().await;
        // Accept partial completions
        match result.status {
            Ok(_) => Ok(result.data),
            Err(TransferError::Fault) => Ok(result.data),
            Err(e) => Err(e.into()),
        }
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

/// Configuration settings for a receive sweep across multiple frequencies.
pub struct SweepParams {
    /// List of frequency pairs to sweep over, in MHz. There can be up to 10.
    freq_mhz: Vec<(u16, u16)>,
    /// Number of blocks to capture per tuning. Each block is 16384 bytes, or
    /// 8192 samples.
    blocks_per_tuning: u16,
    /// Width of each tuning step, in Hz
    step_width_hz: u32,
    /// Frequency offset added to tuned frequencies. `Sample_rate/2` is a good
    /// value.
    offset_hz: u32,
    /// Sweep mode. In
    mode: SweepMode,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SweepMode {
    /// `step_width` is added to the current frequency at each step.
    Linear,
    /// Each step is divided into two interleaved sub-steps, allowing the host
    /// to select the best portions of the FFT of each sub-step and discard the
    /// rest.
    Interleaved,
}

/// A HackRF operating in sweep mode.
pub struct Sweep {
    rf: HackRf,
    queue: nusb::transfer::Queue<nusb::transfer::RequestBuffer>,
    transfer_size: usize,
}

impl Sweep {
    pub async fn new(
        rf: HackRf,
        params: &SweepParams,
        transfer_size: usize,
    ) -> Result<Self, (HackRf, Error)> {
        const MAX_SWEEP_RANGES: usize = 10;
        const TUNING_BLOCK_BYTES: usize = 16384;
        if params.freq_mhz.is_empty()
            || params.freq_mhz.len() > MAX_SWEEP_RANGES
            || params.blocks_per_tuning < 1
            || params.step_width_hz < 1
        {
            return Err((rf, Error::InvalidParameter));
        }

        // Build up the packed struct that we'll send to the HackRF
        let mut data = Vec::with_capacity(params.freq_mhz.len() * 4 + 9);
        data.extend_from_slice(&params.step_width_hz.to_le_bytes());
        data.extend_from_slice(&params.offset_hz.to_be_bytes());
        data.push(match params.mode {
            SweepMode::Linear => 0,
            SweepMode::Interleaved => 0,
        });
        for (lo, hi) in params.freq_mhz.iter().copied() {
            data.extend_from_slice(&lo.to_le_bytes());
            data.extend_from_slice(&hi.to_le_bytes());
        }

        let num_bytes = (params.blocks_per_tuning as u32) * (TUNING_BLOCK_BYTES as u32);

        // Set up the HackRF
        if let Err(e) = rf.interface.control_out(ControlOut {
            control_type: ControlType::Vendor,
            recipient: Recipient::Device,
            request: ControlRequest::InitSweep as u8,
            value: (num_bytes & 0xffff) as u16,
            index: (num_bytes >> 16) as u16,
            data: &data,
        }).await.into_result() {
            return Err((rf, e.into()));
        }

        // Round up to nearest 512-byte increment
        let transfer_size = (transfer_size.max(1) + 0x1FF) & !0x1FF;
        if let Err(e) = rf.set_transceiver_mode(TransceiverMode::RxSweep).await {
            return Err((rf, e));
        }
        let queue = rf.interface.bulk_in_queue(0x81);
        Ok(Self {
            rf,
            queue,
            transfer_size,
        })
    }

    /// Queue up a receive transfer. Up to 4 can be in flight at a time.
    pub fn submit(&mut self, buf: Option<Vec<u8>>) -> Result<(), Error> {
        if self.pending() >= 4 {
            return Err(Error::TransferBusy);
        }
        let req = if let Some(buf) = buf {
            RequestBuffer::reuse(buf, self.transfer_size)
        } else {
            RequestBuffer::new(self.transfer_size)
        };
        self.queue.submit(req);
        Ok(())
    }

    /// Retrieve the next chunk of receive data.
    pub async fn next_complete(&mut self) -> Result<Vec<u8>, Error> {
        let result = self.queue.next_complete().await;
        // Accept partial completions
        match result.status {
            Ok(_) => Ok(result.data),
            Err(TransferError::Fault) => Ok(result.data),
            Err(e) => Err(e.into()),
        }
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
