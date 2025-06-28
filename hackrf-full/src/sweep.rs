use std::sync::mpsc;

use nusb::transfer::{ControlOut, ControlType, Recipient, RequestBuffer};

use crate::{
    consts::{ControlRequest, TransceiverMode}, error::StateChangeError, Buffer, Error, HackRf
};

/// Configuration settings for a receive sweep across multiple frequencies.
pub struct SweepParams {
    /// Sample rate to operate at.
    pub sample_rate_hz: u32,
    /// List of frequency pairs to sweep over, in MHz. There can be up to 10.
    pub freq_mhz: Vec<(u16, u16)>,
    /// Number of blocks to capture per tuning. Each block is 16384 bytes, or
    /// 8192 samples.
    pub blocks_per_tuning: u16,
    /// Width of each tuning step, in Hz. `sample_rate` is a good value, in
    /// general.
    pub step_width_hz: u32,
    /// Frequency offset added to tuned frequencies. `Sample_rate*3/8` is a good
    /// value for Interleaved sweep mode.
    pub offset_hz: u32,
    /// Sweep mode.
    pub mode: SweepMode,
}

impl SweepParams {
    pub fn initialize_from_sample_rate(sample_rate_hz: u32) -> Self {
        Self {
            sample_rate_hz,
            freq_mhz: Vec::new(),
            blocks_per_tuning: 1,
            step_width_hz: sample_rate_hz,
            offset_hz: sample_rate_hz * 3 / 8,
            mode: SweepMode::Interleaved,
        }
    }
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

const SWEEP_BUF_SIZE: usize = 16384;
const SWEEP_SAMPLE_SIZE: usize = (SWEEP_BUF_SIZE - 10) / 2;

pub struct SweepBuf {
    freq_hz: u64,
    buf: Buffer,
}

impl SweepBuf {
    pub fn freq_hz(&self) -> u64 {
        self.freq_hz
    }

    pub fn samples(&self) -> &[num_complex::Complex<i8>] {
        // We checked this range would be valid when we made SweepBuf.
        unsafe { self.buf.samples().get_unchecked(5..) }
    }

    pub fn samples_mut(&mut self) -> &mut [num_complex::Complex<i8>] {
        // We checked this range would be valid when we made SweepBuf.
        unsafe { self.buf.samples_mut().get_unchecked_mut(5..) }
    }

    fn parse(buf: Buffer) -> Result<Self, Error> {
        let bytes = buf.bytes();
        if bytes.len() != SWEEP_BUF_SIZE {
            return Err(Error::ReturnData);
        }
        // SAFETY: We literally just checked the buffer is 16384 bytes. The
        // first 10 are there for sure.
        let header: &[u8; 2] = unsafe { &*(bytes.as_ptr() as *const [u8; 2]) };
        let freq: [u8; 8] = unsafe { *(bytes.as_ptr().add(2) as *const [u8; 8]) };
        if header != &[0x7f, 0x7f] {
            return Err(Error::ReturnData);
        }
        let freq_hz = u64::from_le_bytes(freq);
        if !(100_000..=7_100_000_000).contains(&freq_hz) {
            return Err(Error::ReturnData);
        }
        Ok(Self { freq_hz, buf })
    }
}

/// A HackRF operating in sweep mode.
pub struct Sweep {
    rf: HackRf,
    queue: nusb::transfer::Queue<nusb::transfer::RequestBuffer>,
    buf_pool: mpsc::Receiver<Vec<u8>>,
    buf_pool_send: mpsc::Sender<Vec<u8>>,
}

impl Sweep {
    /// Start a new RX sweep using the provided parameters.
    ///
    /// The size of each internal USB block transfer can be set,
    pub async fn new(rf: HackRf, params: &SweepParams) -> Result<Self, StateChangeError> {
        if let Err(err) = rf.set_sample_rate(params.sample_rate_hz as f64).await {
            return Err(StateChangeError { err, rf });
        }

        Self::new_with_custom_sample_rate(rf, params).await
    }

    /// Start a new RX sweep without configuring the sample rate or baseband filter.
    pub async fn new_with_custom_sample_rate(
        rf: HackRf,
        params: &SweepParams,
    ) -> Result<Self, StateChangeError> {
        const MAX_SWEEP_RANGES: usize = 10;
        const TUNING_BLOCK_BYTES: usize = 16384;
        if params.freq_mhz.is_empty()
            || params.freq_mhz.len() > MAX_SWEEP_RANGES
            || params.blocks_per_tuning < 1
            || params.step_width_hz < 1
        {
            return Err(StateChangeError {
                rf,
                err: Error::InvalidParameter("Invalid sweep parameters"),
            });
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
        if let Err(e) = rf
            .interface
            .control_out(ControlOut {
                control_type: ControlType::Vendor,
                recipient: Recipient::Device,
                request: ControlRequest::InitSweep as u8,
                value: (num_bytes & 0xffff) as u16,
                index: (num_bytes >> 16) as u16,
                data: &data,
            })
            .await
            .into_result()
        {
            return Err(StateChangeError { rf, err: e.into() });
        }

        // Round up to nearest 512-byte increment
        if let Err(err) = rf.set_transceiver_mode(TransceiverMode::RxSweep).await {
            return Err(StateChangeError { rf, err });
        }
        let queue = rf.interface.bulk_in_queue(0x81);
        let (buf_pool_send, buf_pool) = mpsc::channel();
        Ok(Self {
            rf,
            queue,
            buf_pool,
            buf_pool_send,
        })
    }

    /// Queue up a sweep transfer.
    ///
    /// This will pull from a reusable buffer pool first, and allocate a new
    /// buffer if none are available in the pool.
    ///
    /// The buffer pool will grow so long as completed buffers aren't dropped.
    pub fn submit(&mut self) {
        let req = if let Ok(buf) = self.buf_pool.try_recv() {
            RequestBuffer::reuse(buf, SWEEP_BUF_SIZE)
        } else {
            RequestBuffer::new(SWEEP_BUF_SIZE)
        };
        self.queue.submit(req);
    }

    /// Retrieve the next chunk of receive data.
    pub async fn next_complete(&mut self) -> Result<SweepBuf, Error> {
        let buf = self.queue.next_complete().await.into_result()?;
        let buf = Buffer::new(buf, self.buf_pool_send.clone());
        SweepBuf::parse(buf)
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