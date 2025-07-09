use nusb::transfer::{ControlOut, ControlType, Recipient, RequestBuffer};

use crate::{
    Buffer, Error, HackRf, baseband_filter_bw,
    consts::{ControlRequest, TransceiverMode},
    error::StateChangeError,
};

/// Configuration settings for a receive sweep across multiple frequencies.
///
/// The easiest way to configure this is to call
/// [`SweepParams::init_sample_rate`], then to add the desired
/// frequency pairs to sweep over. There shouldn't be more than 10 pairs.
///
/// The recommended usage adds an offset to the center frequency, such that the
/// lower edge of the baseband filter aligns with the lower limit of the sweep.
/// The step width is then 4/3 of the baseband filter.
///
/// The `blocks_per_tuning` parameter determines how many [`SweepBuf`] blocks at
/// a tuned frequency come out in a row. The blocks are *not* consecutive
/// samples; the HackRF briefly turns off between sample blocks.
///
/// It's really best to think of this is a tool for spectrum sensing, not active
/// demodulation.
///
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
    /// Initialize the sweep parameters with some sane defaults, given a sample
    /// rate.
    ///
    /// See the [main `SweepParams` documentation][SweepParams] for more info.
    pub fn init_sample_rate(sample_rate_hz: u32) -> Self {
        let filter_bw = baseband_filter_bw(sample_rate_hz * 3 / 4);
        let offset_hz = filter_bw / 2;
        let step_width_hz = filter_bw * 4 / 3;
        Self {
            sample_rate_hz,
            freq_mhz: Vec::new(),
            blocks_per_tuning: 1,
            step_width_hz,
            offset_hz,
            mode: SweepMode::Interleaved,
        }
    }
}

/// A chosen sweep mode.
///
/// While linear mode is the easiest to understand, the interleaved mode can
/// make it easy to discard the portion of the spectrum with the DC mixing spur.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SweepMode {
    /// `step_width` is added to the current frequency at each step.
    Linear,
    /// Each step is divided into two interleaved sub-steps, allowing the host
    /// to select the best portions of the FFT of each sub-step and discard the
    /// rest. The first step adds 1/4 of the step size, and the second step adds
    /// the remaining 3/4 of the step size. This makes it relatively easy to
    /// discard the center of the band, where mixer IQ imbalance can create a
    /// spike in the FFT.
    Interleaved,
}

const SWEEP_BUF_SIZE: usize = 16384;

/// A block of samples retrieved for some tuning frequency within the sweep.
pub struct SweepBuf {
    freq_hz: u64,
    buf: Buffer,
}

impl SweepBuf {
    /// The frequency tuned to, without the offset added in.
    pub fn freq_hz(&self) -> u64 {
        self.freq_hz
    }

    /// Access the retrieved samples in this tuning block.
    pub fn samples(&self) -> &[num_complex::Complex<i8>] {
        // We checked this range would be valid when we made SweepBuf.
        unsafe { self.buf.samples().get_unchecked(5..) }
    }

    /// Mutably access the retrieved samples in this tuning block.
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
///
/// A sweep continually retunes the HackRF, grabbing a block of 8187 samples
/// (8192, but the first 5 are overwritten with an internal header) at each
/// tuning. The process is:
///
/// 1. Get the lower frequency in a range.
/// 2. Tune to that frequency after adding the frequency offset, then grab the
///    samples.
/// 3. If multiple blocks are requested, grab another (non-sequential) block of
///    samples. Repeat until all requested blocks have been retrieved.
/// 4. Add the step (ignoring any offset). If using interleaved mode, the first
///    sub-step is 1/4 of the step size, and the second sub-step is 3/4 of the
///    step size.
/// 5. If the new frequency is greater or equal to the upper frequency in the
///    range, go to the next range and repeat from step 1. Otherwise go to step
///    2 with the frequency from step 4.
///
/// To receive sweeps, first take a HackRF peripheral and call
/// [`HackRf::start_rx_sweep`], or use [`Sweep::new`] with it.
///
/// Next, call [`submit`][Sweep::submit] to queue up requests, stopping when
/// there are enough pending requests. `libhackrf` queues up 1 MiB of data, or
/// 64 sweep blocks. You'll probably want something similar.
///
/// Actual reception is done with [`next_complete`][Sweep::next_complete],
/// which will panic if there are no pending requests. The number of pending
/// requests can always be checked with [`pending`][Sweep::pending].
///
/// A sweep requires configuration using [`SweepParams`]. The recommended way to
/// set this up is with [`SweepParams::init_sample_rate`], which does the following:
///
/// 1. Configures for [interleaved][SweepMode::Interleaved] mode.
/// 2. Finds the actual baseband filter bandwidth for a given sample rate.
/// 3. Sets the offset to 1/2 of the filter bandwidth, aligning the lower end of
///    the baseband to the lower frequency.
/// 4. Sets the step width to be 4/3 of the filter bandwidth.
///
/// When processing the retrieved data, if we mark the full sample band as
/// spanning from -4 to 4:
///
/// - -4 to -3: lower band edge, filtered out by baseband filter
/// - -3 to -1: in-band, low side
/// - -1 to 1: Too close to DC spur, discard from FFT
/// - 1 to 3: in-band, upper side
/// - 3 to 4: upper band edge, filtered out by baseband filter
///
/// Note that, in a normal FFT where at least two of the prime factors are 2,
/// the transition points are also centered on FFT bins. Using an Offset DFT can
/// fix this, putting the FFT bin transitions at the transition points instead.
///
pub struct Sweep {
    rf: HackRf,
}

impl Sweep {
    /// Start a new RX sweep using the provided parameters.
    ///
    /// Buffers are reused across sweep operations, provided that
    /// [`HackRf::start_rx`] isn't used, or is used with a 8192 sample buffer
    /// size.
    pub async fn new(rf: HackRf, params: &SweepParams) -> Result<Self, StateChangeError> {
        if let Err(err) = rf.set_sample_rate(params.sample_rate_hz as f64).await {
            return Err(StateChangeError { err, rf });
        }

        Self::new_with_custom_sample_rate(rf, params).await
    }

    /// Start a new RX sweep without configuring the sample rate or baseband filter.
    ///
    /// Buffers are reused across sweep operations, provided that
    /// [`HackRf::start_rx`] isn't used, or is used with a 8192 sample buffer
    /// size.
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
            SweepMode::Interleaved => 1,
        });
        for (lo, hi) in params.freq_mhz.iter().copied() {
            if lo >= hi
                || lo > (crate::consts::FREQ_MAX_MHZ as u16)
                || hi > (crate::consts::FREQ_MAX_MHZ as u16)
            {
                return Err(StateChangeError {
                    rf,
                    err: Error::InvalidParameter("Invalid frequency range"),
                });
            }
            // Force the upper ends of each tuning range to align with the step
            // size, pushing them upwards if necessary.
            let lo_hz = lo as u32 * 1_000_000;
            let hi_hz = hi as u32 * 1_000_000;
            let steps = (hi_hz - lo_hz).div_ceil(params.step_width_hz);
            let full_hi = (steps * params.step_width_hz).div_ceil(1_000_000);

            data.extend_from_slice(&lo.to_le_bytes());
            data.extend_from_slice(&full_hi.to_le_bytes());
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

        if let Err(err) = rf.set_transceiver_mode(TransceiverMode::RxSweep).await {
            return Err(StateChangeError { rf, err });
        }
        Ok(Self { rf })
    }

    /// Queue up a sweep transfer.
    ///
    /// This will pull from a reusable buffer pool first, and allocate a new
    /// buffer if none are available in the pool.
    ///
    /// The buffer pool will grow so long as completed buffers aren't dropped.
    pub fn submit(&mut self) {
        let req = if let Ok(buf) = self.rf.rx.buf_pool.try_recv() {
            RequestBuffer::reuse(buf, SWEEP_BUF_SIZE)
        } else {
            RequestBuffer::new(SWEEP_BUF_SIZE)
        };
        self.rf.rx.queue.submit(req);
    }

    /// Retrieve the next chunk of receive data.
    ///
    /// This future is cancel-safe, so feel free to use it alongside a timeout
    /// or a `select!`-type pattern.
    pub async fn next_complete(&mut self) -> Result<SweepBuf, Error> {
        let result = self.rf.rx.queue.next_complete().await;
        match result.status {
            Ok(_) => {
                let buf = Buffer::new(result.data, self.rf.rx.buf_pool_send.clone());
                SweepBuf::parse(buf)
            }
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
    pub async fn stop(mut self) -> Result<HackRf, StateChangeError> {
        self.rf.rx.queue.cancel_all();
        while self.pending() > 0 {
            let _ = self.next_complete().await;
        }
        match self.rf.set_transceiver_mode(TransceiverMode::Off).await {
            Ok(_) => Ok(self.rf),
            Err(err) => Err(StateChangeError { err, rf: self.rf }),
        }
    }
}
