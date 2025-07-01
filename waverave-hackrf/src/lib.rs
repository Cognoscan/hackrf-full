/*!

This is a complete, strongly-asynchronous host crate for the [HackRF][hackrf],
made using the pure-rust [`nusb`] crate for USB interfacing. It reproduces *all* 
the functionality of the original [`libhackrf`][libhackrf] library.

[hackrf]: https://greatscottgadgets.com/hackrf/one/
[libhackrf]: https://github.com/greatscottgadgets/hackrf/tree/master/host

The standard entry point for this library is [`open_hackrf()`], which will open
the first available HackRF device.

Getting started is easy: open up a HackRF peripheral, configure it as needed,
and enter into transmit, receive, or RX sweep mode. Changing the operating mode
also changes the struct used, i.e. it uses the typestate pattern. The different
states and their corresponding structs are:

- [`HackRf`] - The default, off, state.
- [`Receive`] - Receiving RF signals.
- [`Transmit`] - Transmitting RF signals.
- [`Sweep`] - Running a receive sweep through multiple tuning frequencies.

If a mode change error occurs, the [`HackRf`] struct is returned alongside the
error, and it can potentially be reset back to the off state by running
[`HackRf::turn_off`].

As for what using this library looks like in practice, here's an example program
that configures the system, enters receive mode, and processes samples to
estimate the average received power relative to full scale:

```no_run
use anyhow::Result;
#[tokio::main]
async fn main() -> Result<()> {
    let hackrf = waverave_hackrf::open_hackrf()?;

    // Configure: 20MHz sample rate, turn on the RF amp, set IF & BB gains to 16 dB,
    // and tune to 915 MHz.
    hackrf.set_sample_rate(20e6).await?;
    hackrf.set_amp_enable(true).await?;
    hackrf.set_lna_gain(16).await?;
    hackrf.set_vga_gain(16).await?;
    hackrf.set_freq(915_000_000).await?;

    // Start receiving, in bursts of 16384 samples
    let mut hackrf_rx = hackrf.start_rx(16384).await.map_err(|e| e.err)?;

    // Queue up 64 transfers, retrieve them, and measure average power.
    for _ in 0..64 {
        hackrf_rx.submit();
    }
    let mut count = 0;
    let mut pow_sum = 0.0;
    while hackrf_rx.pending() > 0 {
        let buf = hackrf_rx.next_complete().await?;
        for x in buf.samples() {
            let re = x.re as f64;
            let im = x.im as f64;
            pow_sum += re * re + im * im;
        }
        count += buf.len();
    }

    // Stop receiving
    hackrf_rx.stop().await?;

    // Print out our measurement
    let average_power = (pow_sum / (count as f64 * 127.0 * 127.0)).log10() * 10.;
    println!("Average Power = {average_power} dbFS");
    Ok(())
}

```

*/

#![warn(missing_docs)]

mod consts;
pub mod debug;
mod error;
pub mod info;
mod rx;
mod sweep;
mod tx;
use std::ops::Range;

use bytemuck::Pod;
use core::mem::size_of;
use nusb::transfer::{ControlIn, ControlOut, ControlType, Recipient};
use std::sync::mpsc;

use crate::consts::*;
use crate::debug::Debug;
use crate::info::Info;

pub use crate::error::{Error, StateChangeError};
pub use crate::rx::Receive;
pub use crate::sweep::{Sweep, SweepBuf, SweepMode, SweepParams};
pub use crate::tx::Transmit;

/// Complex 8-bit signed data, as used by the HackRF.
pub type ComplexI8 = num_complex::Complex<i8>;

/// Operacake port A1
pub const PORT_A1: u8 = 0;
/// Operacake port A2
pub const PORT_A2: u8 = 1;
/// Operacake port A3
pub const PORT_A3: u8 = 2;
/// Operacake port A4
pub const PORT_A4: u8 = 3;
/// Operacake port B1
pub const PORT_B1: u8 = 4;
/// Operacake port B2
pub const PORT_B2: u8 = 5;
/// Operacake port B3
pub const PORT_B3: u8 = 6;
/// Operacake port B4
pub const PORT_B4: u8 = 7;

/// A Buffer holding HackRF transfer data.
///
/// Samples can be directly accessed as slices, and can be extended up to the
/// length of the fixed-size underlying buffer.
///
/// When dropped, this buffer returns to the internal buffer pool it came from.
/// It can either be backed by an allocation from the system allocator, or by
/// some platform-specific way of allocating memory for zero-copy USB transfers.
pub struct Buffer {
    buf: Vec<u8>,
    pool: mpsc::Sender<Vec<u8>>,
}

impl Buffer {
    pub(crate) fn new(buf: Vec<u8>, pool: mpsc::Sender<Vec<u8>>) -> Self {
        assert!(buf.len() & 0x1FF == 0);
        Self { buf, pool }
    }

    pub(crate) fn into_vec(mut self) -> Vec<u8> {
        core::mem::take(&mut self.buf)
    }

    /// Get how many samples this buffer can hold.
    pub fn capacity(&self) -> usize {
        // Force down to the nearest 512-byte boundary, which is the transfer
        // size the HackRF requires.
        (self.buf.capacity() & !0x1FF) / size_of::<ComplexI8>()
    }

    /// Clear out the buffer's samples.
    pub fn clear(&mut self) {
        self.buf.clear();
    }

    /// Size of the buffer, in samples.
    pub fn len(&self) -> usize {
        self.buf.len() / size_of::<ComplexI8>()
    }

    /// Returns true if there are no samples in the buffer.
    pub fn is_empty(&self) -> bool {
        self.buf.is_empty()
    }

    /// Remaining capacity in the buffer, in samples.
    pub fn remaining_capacity(&self) -> usize {
        self.capacity() - self.len()
    }

    /// Extend the buffer with some number of samples set to 0, and get a
    /// mutable slice to the newly initialized samples.
    ///
    /// # Panics
    /// - If there is not enough space left for the added samples.
    pub fn extend_zeros(&mut self, len: usize) -> &mut [ComplexI8] {
        assert!(self.remaining_capacity() >= len);
        let old_len = self.buf.len();
        let new_len = old_len + len * size_of::<ComplexI8>();
        self.buf.resize(new_len, 0);
        let buf: &mut [u8] = &mut self.buf;
        // SAFETY: We only ever resize according to the size of a ComplexI8,
        // the buffer always holds ComplexI8 internally, and ComplexI8 has an
        // alignment of 1.
        unsafe {
            core::slice::from_raw_parts_mut(
                buf.as_mut_ptr().add(old_len) as *mut ComplexI8,
                len / size_of::<ComplexI8>(),
            )
        }
    }

    /// Extend the buffer with a slice of samples.
    ///
    /// # Panics
    /// - If there is no space left in the buffer for the slice.
    pub fn extend_from_slice(&mut self, slice: &[ComplexI8]) {
        assert!(self.remaining_capacity() >= slice.len());
        // SAFETY: We can always cast a ComplexI8 to bytes, as it meets all the
        // "plain old data" requirements.
        let slice = unsafe {
            core::slice::from_raw_parts(slice.as_ptr() as *const u8, core::mem::size_of_val(slice))
        };
        self.buf.extend_from_slice(slice);
    }

    /// Push a value onto the buffer.
    ///
    /// # Panics
    /// - If there is no space left in the buffer.
    pub fn push(&mut self, val: ComplexI8) {
        assert!(self.remaining_capacity() > 0);
        let slice: &[u8; 2] = unsafe { &*((&val) as *const ComplexI8 as *const [u8; 2]) };
        self.buf.extend_from_slice(slice);
    }

    /// Get the sample sequence as a slice of bytes instead of complex values.
    pub fn bytes(&self) -> &[u8] {
        &self.buf
    }

    /// Get the sample sequence as a mutable slice of bytes instead of complex values.
    pub fn bytes_mut(&mut self) -> &mut [u8] {
        &mut self.buf
    }

    /// Get the samples in the buffer.
    pub fn samples(&self) -> &[ComplexI8] {
        let buf: &[u8] = &self.buf;
        // SAFETY: the buffer is aligned because `ComplexI8` has an alignment of
        // 1, same as a byte buffer, the data is valid, and we truncate to only
        // valid populated pairs. Also we shouldn't ever have a byte buffer that
        // isn't an even number of bytes anyway...
        unsafe {
            core::slice::from_raw_parts(
                buf.as_ptr() as *const ComplexI8,
                self.buf.len() / size_of::<ComplexI8>(),
            )
        }
    }

    /// Mutably get the samples in the buffer.
    pub fn samples_mut(&mut self) -> &mut [ComplexI8] {
        let buf: &mut [u8] = &mut self.buf;
        // SAFETY: the buffer is aligned because `ComplexI8` has an alignment of
        // 1, same as a byte buffer, the data is valid, and we truncate to only
        // valid populated pairs. Also we shouldn't ever have a byte buffer that
        // isn't an even number of bytes anyway...
        unsafe {
            core::slice::from_raw_parts_mut(
                buf.as_mut_ptr() as *mut ComplexI8,
                self.buf.len() / size_of::<ComplexI8>(),
            )
        }
    }
}

impl Drop for Buffer {
    fn drop(&mut self) {
        let inner = core::mem::take(&mut self.buf);
        if inner.capacity() > 0 {
            let _ = self.pool.send(inner);
        }
    }
}

/// Configuration settings for the Bias-T switch.
///
/// Used when calling [`HackRf::set_user_bias_t_opts`].
#[derive(Clone, Debug)]
pub struct BiasTSetting {
    /// What mode change to apply when switching to transmit.
    pub tx: BiasTMode,
    /// What mode change to apply when switching to receive.
    pub rx: BiasTMode,
    /// What mode change to apply when switching off.
    pub off: BiasTMode,
}

/// A Bias-T setting change to apply on a mode change.
///
/// See [`BiasTSetting`] for where to use this.
#[allow(missing_docs)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BiasTMode {
    NoChange,
    Enable,
    Disable,
}

impl BiasTMode {
    fn as_u16(self) -> u16 {
        match self {
            Self::NoChange => 0x0,
            Self::Disable => 0x2,
            Self::Enable => 0x3,
        }
    }
}

/// RF Filter Setting Option.
///
/// Use when calling [`HackRf::set_freq_explicit`].
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RfPathFilter {
    /// No filter selected - mixer bypassed.
    Bypass = 0,
    /// Low pass filter, `f_c = f_IF - f_LO`
    LowPass = 1,
    /// High pass filter, `f_c = f_IF + f_LO`
    HighPass = 2,
}

impl std::fmt::Display for RfPathFilter {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Bypass => f.write_str("mixer bypass"),
            Self::LowPass => f.write_str("low pass filter"),
            Self::HighPass => f.write_str("high pass filter"),
        }
    }
}

/// Configuration for an Operacake board.
///
/// An Operacake board has three different operating modes:
///
/// - Manual: the switches are manually set and don't change until the next
///   configuration operation.
/// - Frequency: the switches change depending on the center frequency the board
///   is tuned to.
/// - Time: the switches change after some number of samples have been
///   sent/received.
///
/// Use when calling [`HackRf::operacake_set_mode`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u16)]
#[allow(missing_docs)]
pub enum OperacakeMode {
    Manual = 0,
    Frequency = 1,
    Time = 2,
}

/// A Frequency band allocated to a specific port for all Operacakes operating
/// in frequency mode.
///
/// This is used in [`HackRf::operacake_config_freq`].
///
/// Ports are zero-indexed, but can also be referred to with the top-level
/// constants:
/// - PORT_A1 = 0
/// - PORT_A2 = 1
/// - PORT_A3 = 2
/// - PORT_A4 = 3
/// - PORT_B1 = 4
/// - PORT_B2 = 5
/// - PORT_B3 = 6
/// - PORT_B4 = 7
#[derive(Clone, Copy, Debug)]
pub struct OperacakeFreq {
    /// Start frequency, in MHz.
    pub min: u16,
    /// Stop frequency, in MHz.
    pub max: u16,
    /// Port for A0 to use for the range. B0 will use the mirror image.
    pub port: u8,
}

/// A dwell time allocated to a specific port for all Operacakes operating in
/// dwell time mode.
///
/// This is used in [`HackRf::operacake_config_time`].
///
/// Ports are zero-indexed, but can also be referred to with the top-level
/// constants:
/// - PORT_A1 = 0
/// - PORT_A2 = 1
/// - PORT_A3 = 2
/// - PORT_A4 = 3
/// - PORT_B1 = 4
/// - PORT_B2 = 5
/// - PORT_B3 = 6
/// - PORT_B4 = 7
#[derive(Clone, Copy, Debug)]
pub struct OperacakeDwell {
    /// Dwell time, in number of samples
    pub dwell: u32,
    /// Port for A0 to use for the range. B0 will use the mirror image.
    pub port: u8,
}

/// A HackRF device descriptor, which can be opened.
///
/// These are mostly returned from calling [`list_hackrf_devices`], but can also
/// be formed by trying to convert a [`nusb::DeviceInfo`] into one.
pub struct HackRfDescriptor {
    info: nusb::DeviceInfo,
}

/// The type of HackRF device that was detected.
#[allow(missing_docs)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HackRfType {
    Jawbreaker,
    One,
    Rad1o,
}

impl std::fmt::Display for HackRfType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Jawbreaker => f.write_str("Jawbreaker"),
            Self::One => f.write_str("HackRF One"),
            Self::Rad1o => f.write_str("rad1o"),
        }
    }
}

impl HackRfDescriptor {
    /// Get the serial number of this HackRF, as a string.
    pub fn serial(&self) -> Option<&str> {
        self.info.serial_number()
    }

    /// Get the [type][HackRfType] of HackRF radio this is.
    pub fn radio_type(&self) -> HackRfType {
        match self.info.product_id() {
            HACKRF_JAWBREAKER_USB_PID => HackRfType::Jawbreaker,
            HACKRF_ONE_USB_PID => HackRfType::One,
            RAD1O_USB_PID => HackRfType::Rad1o,
            _ => panic!("Created a HackRfDescriptor without using a known product ID"),
        }
    }

    /// Try and open this HackRf device descriptor.
    pub fn open(self) -> Result<HackRf, std::io::Error> {
        let version = self.info.device_version();
        let ty = self.radio_type();
        let device = self.info.open()?;
        #[cfg(not(target_os = "windows"))]
        {
            if device.active_configuration()?.configuration_value() != 1 {
                device.detach_kernel_driver(0)?;
                device.set_configuration(1)?;
            }
        }
        let interface = device.detach_and_claim_interface(0)?;

        let (buf_pool_send, buf_pool) = mpsc::channel();
        let tx = TxEndpoint {
            queue: interface.bulk_out_queue(TX_ENDPOINT_ADDRESS),
            buf_pool,
            buf_pool_send,
        };
        let (buf_pool_send, buf_pool) = mpsc::channel();
        let rx = RxEndpoint {
            queue: interface.bulk_in_queue(RX_ENDPOINT_ADDRESS),
            buf_pool,
            buf_pool_send,
        };

        Ok(HackRf {
            interface,
            version,
            ty,
            rx,
            tx,
        })
    }
}

/// Try and turn any [`nusb::DeviceInfo`] descriptor into a HackRF, failing if
/// the VID and PID don't match any known devices.
impl TryFrom<nusb::DeviceInfo> for HackRfDescriptor {
    type Error = &'static str;
    fn try_from(value: nusb::DeviceInfo) -> Result<Self, Self::Error> {
        if value.vendor_id() == HACKRF_USB_VID {
            if matches!(
                value.product_id(),
                HACKRF_JAWBREAKER_USB_PID | HACKRF_ONE_USB_PID | RAD1O_USB_PID
            ) {
                Ok(HackRfDescriptor { info: value })
            } else {
                Err("VID recognized, PID not recognized")
            }
        } else {
            Err("VID doesn't match for HackRF")
        }
    }
}

/// List all available HackRF devices.
pub fn list_hackrf_devices() -> Result<Vec<HackRfDescriptor>, std::io::Error> {
    Ok(nusb::list_devices()?
        .filter(|d| {
            d.vendor_id() == HACKRF_USB_VID
                && matches!(
                    d.product_id(),
                    HACKRF_JAWBREAKER_USB_PID | HACKRF_ONE_USB_PID | RAD1O_USB_PID
                )
        })
        .map(|d| HackRfDescriptor { info: d })
        .collect::<Vec<HackRfDescriptor>>())
}

/// Open the first detected HackRF device in the system.
///
/// This is a shortcut for calling [`list_hackrf_devices`] and opening the first one.
pub fn open_hackrf() -> Result<HackRf, std::io::Error> {
    list_hackrf_devices()?
        .into_iter()
        .next()
        .ok_or_else(|| std::io::Error::other("No HackRF devices"))?
        .open()
}

/// A HackRF device. This is the main struct for talking to the HackRF.
///
/// This provides all the settings to actively configure the HackRF while it is
/// off, as well as the ability to use debug or info fetching operations with
/// the [`HackRf::info`] and [`HackRf::debug`] functions. Some of these
/// operations are also exposed while receiving & transmitting, if it makes
/// sense to do so.
pub struct HackRf {
    pub(crate) interface: nusb::Interface,
    pub(crate) version: u16,
    pub(crate) ty: HackRfType,
    pub(crate) rx: RxEndpoint,
    pub(crate) tx: TxEndpoint,
}

struct RxEndpoint {
    queue: nusb::transfer::Queue<nusb::transfer::RequestBuffer>,
    buf_pool: mpsc::Receiver<Vec<u8>>,
    buf_pool_send: mpsc::Sender<Vec<u8>>,
}

struct TxEndpoint {
    queue: nusb::transfer::Queue<Vec<u8>>,
    buf_pool: mpsc::Receiver<Vec<u8>>,
    buf_pool_send: mpsc::Sender<Vec<u8>>,
}

impl HackRf {
    fn api_check(&self, needed: u16) -> Result<(), Error> {
        if self.version < needed {
            Err(Error::ApiVersion {
                needed,
                actual: self.version,
            })
        } else {
            Ok(())
        }
    }

    async fn write_u32(&self, req: ControlRequest, val: u32) -> Result<(), Error> {
        Ok(self
            .interface
            .control_out(ControlOut {
                control_type: ControlType::Vendor,
                recipient: Recipient::Device,
                request: req as u8,
                value: (val & 0xffff) as u16,
                index: (val >> 16) as u16,
                data: &[],
            })
            .await
            .status?)
    }

    async fn write_u16(&self, req: ControlRequest, idx: u16, val: u16) -> Result<(), Error> {
        Ok(self
            .interface
            .control_out(ControlOut {
                control_type: ControlType::Vendor,
                recipient: Recipient::Device,
                request: req as u8,
                value: val,
                index: idx,
                data: &[],
            })
            .await
            .status?)
    }

    async fn read_u16(&self, req: ControlRequest, idx: u16) -> Result<u16, Error> {
        let ret = self
            .interface
            .control_in(ControlIn {
                control_type: ControlType::Vendor,
                recipient: Recipient::Device,
                request: req as u8,
                value: 0,
                index: idx,
                length: 2,
            })
            .await
            .into_result()?;
        let ret: [u8; 2] = ret.as_slice().try_into().map_err(|_| Error::ReturnData)?;
        Ok(u16::from_le_bytes(ret))
    }

    async fn write_u8(&self, req: ControlRequest, idx: u16, val: u8) -> Result<(), Error> {
        self.write_u16(req, idx, val as u16).await?;
        Ok(())
    }

    async fn read_u8(&self, req: ControlRequest, idx: u16) -> Result<u8, Error> {
        let ret = self
            .interface
            .control_in(ControlIn {
                control_type: ControlType::Vendor,
                recipient: Recipient::Device,
                request: req as u8,
                value: 0,
                index: idx,
                length: 1,
            })
            .await
            .into_result()?;
        ret.first().copied().ok_or(Error::ReturnData)
    }

    async fn write_bytes(&self, req: ControlRequest, data: &[u8]) -> Result<(), Error> {
        self.interface
            .control_out(ControlOut {
                control_type: ControlType::Vendor,
                recipient: Recipient::Device,
                request: req as u8,
                value: 0,
                index: 0,
                data,
            })
            .await
            .into_result()?;
        Ok(())
    }

    async fn read_bytes(&self, req: ControlRequest, len: usize) -> Result<Vec<u8>, Error> {
        assert!(len < u16::MAX as usize);
        Ok(self
            .interface
            .control_in(ControlIn {
                control_type: ControlType::Vendor,
                recipient: Recipient::Device,
                request: req as u8,
                value: 0,
                index: 0,
                length: len as u16,
            })
            .await
            .into_result()?)
    }

    async fn read_struct<T>(&self, req: ControlRequest) -> Result<T, Error>
    where
        T: Pod,
    {
        let size = size_of::<T>();
        let mut resp = self.read_bytes(req, size).await?;
        if resp.len() < size {
            return Err(Error::ReturnData);
        }
        resp.truncate(size);
        Ok(bytemuck::pod_read_unaligned(&resp))
    }

    async fn set_transceiver_mode(&self, mode: TransceiverMode) -> Result<(), Error> {
        self.write_u16(ControlRequest::SetTransceiverMode, 0, mode as u16)
            .await
    }

    /// Set the baseband filter bandwidth.
    ///
    /// The possible settings are: 1.75, 2.5, 3.5, 5, 5.5, 6, 7, 8, 9, 10, 12,
    /// 14, 15, 20, 24, and 28 MHz. This function will choose the nearest,
    /// rounded down.
    ///
    /// The default is to set this to 3/4 of the sample rate, rounded down to
    /// the nearest setting.
    ///
    /// Setting the sample rate with [`set_sample_rate`][Self::set_sample_rate]
    /// will modify this setting.
    pub async fn set_baseband_filter_bandwidth(&self, bandwidth_hz: u32) -> Result<(), Error> {
        let bandwidth_hz = baseband_filter_bw(bandwidth_hz);
        self.write_u32(ControlRequest::BasebandFilterBandwidthSet, bandwidth_hz)
            .await
    }

    /// Set the transmit underrun limit. This will cause the HackRF to stop
    /// operation if transmit runs out of samples to send. Set to 0 to disable.
    ///
    /// This will also cause all outstanding transmits to stall forever, so some
    /// timeout will need to be added to the transmit completion futures.
    pub async fn set_tx_underrun_limit(&self, val: u32) -> Result<(), Error> {
        self.api_check(0x0106)?;
        self.write_u32(ControlRequest::SetTxUnderrunLimit, val)
            .await
    }

    /// Set the receive overrun limit. This will cause the HackRF to stop
    /// operation if more than the specified amount of samples get lost. Set to
    /// 0 to disable.
    ///
    /// This will also cause all outstanding receives to stall forever, so some
    /// timeout will need to be added to the receive completion futures.
    pub async fn set_rx_overrun_limit(&self, val: u32) -> Result<(), Error> {
        self.api_check(0x0106)?;
        self.write_u32(ControlRequest::SetRxOverrunLimit, val).await
    }

    /// Access the debug/programming commands for the HackRF.
    pub fn debug(&mut self) -> Debug<'_> {
        Debug::new(self)
    }

    /// Access the info commands for the HackRF.
    pub fn info(&self) -> Info<'_> {
        Info::new(self)
    }

    /// Set the operating frequency (recommended method).
    ///
    /// This uses the internal frequency tuning code onboard the HackRF, which
    /// can differ between boards. It automatically sets the LO and IF
    /// frequencies, as well as the RF path filter.
    pub async fn set_freq(&self, freq_hz: u64) -> Result<(), Error> {
        const ONE_MHZ: u64 = 1_000_000;
        #[repr(C)]
        #[derive(Clone, Copy, bytemuck::Zeroable, bytemuck::Pod)]
        struct FreqParams {
            mhz: u32,
            hz: u32,
        }
        let mhz = freq_hz / ONE_MHZ;
        let hz = freq_hz % ONE_MHZ;
        let params = FreqParams {
            mhz: (mhz as u32).to_le(),
            hz: (hz as u32).to_le(),
        };

        self.write_bytes(ControlRequest::SetFreq, bytemuck::bytes_of(&params))
            .await
    }

    /// Set the IF & LO tuning frequencies, and the RF path filter.
    ///
    /// You may be looking for [`set_freq`][HackRf::set_freq] instead.
    ///
    /// This sets the center frequency to `f_c = f_IF + k * f_LO`, where k is
    /// -1, 0, or 1 depending on the filter selected.
    ///
    /// IF frequency *must* be between 2-3 GHz, and it's strongly recommended to
    /// be between 2170-2740 MHz.
    ///
    /// LO frequency must be between 84.375-5400 MHz. No effect if the filter is
    /// set to bypass mode.
    pub async fn set_freq_explicit(
        &self,
        if_freq_hz: u64,
        lo_freq_hz: u64,
        path: RfPathFilter,
    ) -> Result<(), Error> {
        #[repr(C)]
        #[derive(Clone, Copy, bytemuck::Zeroable, bytemuck::Pod)]
        struct FreqParams {
            if_freq_hz: u64,
            lo_freq_hz: u64,
            path: u8,
            reserved: [u8; 7],
        }

        const IF_RANGE: Range<u64> = Range {
            start: 2_000_000_000,
            end: 3_000_000_001,
        };
        const LO_RANGE: Range<u64> = Range {
            start: 84_375_000,
            end: 5_400_000_001,
        };

        if !IF_RANGE.contains(&if_freq_hz) {
            return Err(Error::TuningRange {
                range: IF_RANGE,
                val: if_freq_hz,
            });
        }
        if path != RfPathFilter::Bypass && !LO_RANGE.contains(&lo_freq_hz) {
            return Err(Error::TuningRange {
                range: LO_RANGE,
                val: lo_freq_hz,
            });
        }

        let params = FreqParams {
            if_freq_hz: if_freq_hz.to_le(),
            lo_freq_hz: lo_freq_hz.to_le(),
            path: path as u8,
            reserved: [0u8; 7],
        };

        self.write_bytes(ControlRequest::SetFreqExplicit, bytemuck::bytes_of(&params))
            .await
    }

    /// Set the sample rate using a clock frequency in Hz and a divider value.
    ///
    /// The resulting sample rate is `freq_hz/divider`. Divider value can be
    /// 1-31, and the rate range should be 2-20MHz. Lower & higher values are
    /// technically possible, but not recommended.
    ///
    /// This function will always call
    /// [`set_baseband_filter_bandwidth`][Self::set_baseband_filter_bandwidth],
    /// so any changes to the filter should be done *after* this function.
    ///
    /// You may want to just use [`set_sample_rate`][Self::set_sample_rate]
    /// instead.
    ///
    pub async fn set_sample_rate_manual(&self, freq_hz: u32, divider: u32) -> Result<(), Error> {
        #[repr(C)]
        #[derive(Clone, Copy, bytemuck::Zeroable, bytemuck::Pod)]
        struct FracRateParams {
            freq_hz: u32,
            divider: u32,
        }

        const DIV_RANGE: Range<u32> = Range { start: 1, end: 32 };
        if !DIV_RANGE.contains(&divider) {
            return Err(Error::ValueRange {
                range: DIV_RANGE,
                val: divider,
            });
        }

        let params = FracRateParams {
            freq_hz: freq_hz.to_le(),
            divider: divider.to_le(),
        };

        self.write_bytes(ControlRequest::SampleRateSet, bytemuck::bytes_of(&params))
            .await?;

        let filter_bw = baseband_filter_bw(freq_hz * 3 / (divider * 4));
        self.set_baseband_filter_bandwidth(filter_bw).await?;
        Ok(())
    }

    /// Set the sample rate, which should be between 2-20 MHz.
    ///
    /// Lower & higher rates are possible, but not recommended.
    ///
    /// This function will always call
    /// [`set_baseband_filter_bandwidth`][Self::set_baseband_filter_bandwidth],
    /// so any changes to the filter should be done *after* this function.
    ///
    /// This function is a convenience wrapper around
    /// [`set_sample_rate_manual`][Self::set_sample_rate_manual].
    ///
    pub async fn set_sample_rate(&self, freq: f64) -> Result<(), Error> {
        let freq = freq.clamp(2e6, 20e6);

        let mut freq_hz = 0;
        let mut divider = 1;
        let mut diff = f64::MAX;

        // Just blindly check the closest of all possible divider values,
        // preferring the smaller divider value on ties
        for i in 1u32..32 {
            let new_freq_hz = (freq * (i as f64)).round() as u32;
            let new_diff = ((freq_hz as f64) / (i as f64) - freq).abs();
            if new_diff < diff {
                freq_hz = new_freq_hz;
                divider = i;
                diff = new_diff;
            }
        }

        self.set_sample_rate_manual(freq_hz, divider).await
    }

    /// Enable/disable the 14dB RF amplifiers.
    ///
    /// Enable/disable the RX/TX amplifiers U13/U25 via the controlling switches
    /// U9 and U14.
    pub async fn set_amp_enable(&self, enable: bool) -> Result<(), Error> {
        self.write_u16(ControlRequest::AmpEnable, 0, enable as u16)
            .await
    }

    /// Set the LNA gain.
    ///
    /// Sets the RF RX gain of the MAX2837 transceiver IC. Must be in the range
    /// of 0-40 dB, and is forced to 8 dB steps. Intermediate values are rounded
    /// down.
    pub async fn set_lna_gain(&self, value: u16) -> Result<(), Error> {
        if value > 40 {
            return Err(Error::ValueRange {
                range: Range { start: 0, end: 41 },
                val: value as u32,
            });
        }

        let ret = self
            .read_u8(ControlRequest::SetLnaGain, value & (!0x07))
            .await?;
        if ret == 0 {
            return Err(Error::ReturnData);
        }
        Ok(())
    }

    /// Set the VGA gain.
    ///
    /// Sets the baseband RX gain of the MAX2837 transceiver IC. Must be in the range
    /// of 0-62 dB, and is forced to 2 dB steps. Intermediate values are rounded
    /// down.
    pub async fn set_vga_gain(&self, value: u16) -> Result<(), Error> {
        if value > 62 {
            return Err(Error::ValueRange {
                range: Range { start: 0, end: 63 },
                val: value as u32,
            });
        }

        let ret = self
            .read_u8(ControlRequest::SetVgaGain, value & (!0x01))
            .await?;
        if ret == 0 {
            return Err(Error::ReturnData);
        }
        Ok(())
    }

    /// Set the RF TX gain.
    ///
    /// Sets the RF TX gain of the MAX2837 transceiver IC. Must be in the range
    /// of 0-47 dB.
    pub async fn set_txvga_gain(&self, value: u16) -> Result<(), Error> {
        if value > 47 {
            return Err(Error::ValueRange {
                range: Range { start: 0, end: 48 },
                val: value as u32,
            });
        }

        let ret = self.read_u8(ControlRequest::SetTxvgaGain, value).await?;
        if ret == 0 {
            return Err(Error::ReturnData);
        }
        Ok(())
    }

    /// Temporarily enable/disable the bias-tee (antenna port power).
    ///
    /// Enable or disable the **3.3v (max 50 mA)** bias-tee. Defaults to
    /// disabled on power-up.
    ///
    /// The firmware auto-disables this after returning to IDLE mode. Consider
    /// using [`set_user_bias_t_opts`][Self::set_user_bias_t_opts] instead to
    /// configure the bias to work exactly the way you want it to.
    pub async fn set_antenna_enable(&self, enable: bool) -> Result<(), Error> {
        self.write_u16(ControlRequest::AntennaEnable, 0, enable as u16)
            .await
    }

    /// Set hardware sync mode (hardware triggering).
    ///
    /// See the documentation
    /// [here](https://hackrf.readthedocs.io/en/latest/hardware_triggering.html).
    ///
    /// When enabled, the next operating mode (RX, TX, or Sweep) will not start
    /// until the input hardware trigger occurs.
    ///
    /// Requires API version 0x0102 or higher.
    pub async fn set_hw_sync_mode(&self, enable: bool) -> Result<(), Error> {
        self.api_check(0x0102)?;
        self.write_u16(ControlRequest::SetHwSyncMode, 0, enable as u16)
            .await
    }

    /// Get a list of what operacake boards are attached (up to 8).
    ///
    /// Requires API version 0x0105 or higher.
    pub async fn operacake_boards(&self) -> Result<Vec<u8>, Error> {
        self.api_check(0x0105)?;
        let mut resp = self
            .read_bytes(ControlRequest::OperacakeGetBoards, 8)
            .await?;
        resp.retain(|&x| x != 0xFF);
        Ok(resp)
    }

    /// Set an Operacake board to a specific operating mode.
    ///
    /// When set to frequency or dwell time mode, the settings are shared
    /// between all operacakes in that operating mode.
    ///
    /// Requires API version 0x0105 or higher.
    pub async fn operacake_set_mode(
        &self,
        address: u8,
        setting: OperacakeMode,
    ) -> Result<(), Error> {
        self.api_check(0x0105)?;
        if address > 7 {
            return Err(Error::InvalidParameter("Operacake address is out of range"));
        }
        self.write_u8(ControlRequest::OperacakeSetMode, setting as u16, address)
            .await
    }

    /// Get the operating mode of an operacake board.
    ///
    /// Requires API version 0x0105 or higher.
    pub async fn operacake_get_mode(&self, address: u8) -> Result<OperacakeMode, Error> {
        self.api_check(0x0105)?;
        if address > 7 {
            return Err(Error::InvalidParameter("Operacake address is out of range"));
        }
        let ret = self
            .interface
            .control_in(ControlIn {
                control_type: ControlType::Vendor,
                recipient: Recipient::Device,
                request: ControlRequest::OperacakeGetMode as u8,
                value: address as u16,
                index: 0,
                length: 1,
            })
            .await
            .into_result()?;
        let ret = ret.first().ok_or(Error::ReturnData)?;
        match ret {
            0 => Ok(OperacakeMode::Manual),
            1 => Ok(OperacakeMode::Frequency),
            2 => Ok(OperacakeMode::Time),
            _ => Err(Error::ReturnData),
        }
    }

    /// Set an operacake's switches manually.
    ///
    /// Should be called after setting manual mode with
    /// [`operacake_set_mode`][Self::operacake_set_mode].
    ///
    /// Requires API version 0x0102 or higher.
    pub async fn operacake_config_manual(&self, address: u8, a: u8, b: u8) -> Result<(), Error> {
        self.api_check(0x0102)?;
        if address > 7 {
            return Err(Error::InvalidParameter("Operacake address is out of range"));
        }

        if a > 7 || b > 7 {
            return Err(Error::InvalidParameter(
                "One or more port numbers is out of range (0-7)",
            ));
        }
        if (a < 4 && b < 4) || (a >= 4 && b >= 4) {
            return Err(Error::InvalidParameter(
                "A0 & B0 ports are using same quad of multiplexed ports",
            ));
        }

        let a = a as u16;
        let b = b as u16;
        self.write_u8(ControlRequest::OperacakeSetPorts, a | (b << 8), address)
            .await
    }

    /// Match frequency bands to operacake ports.
    ///
    /// These frequency settings are used by any operacake operating in
    /// frequency mode.
    ///
    /// Requires API version 0x0103 or higher.
    pub async fn operacake_config_freq(&self, freqs: &[OperacakeFreq]) -> Result<(), Error> {
        self.api_check(0x0103)?;
        if freqs.len() > 8 {
            return Err(Error::InvalidParameter(
                "Operacake can only support 8 frequency bands max",
            ));
        }
        let mut data = Vec::with_capacity(5 * freqs.len());
        for f in freqs {
            if f.port > 7 {
                return Err(Error::InvalidParameter(
                    "Operacake frequency band port selection is out of range",
                ));
            }
            data.push((f.min >> 8) as u8);
            data.push((f.min & 0xFF) as u8);
            data.push((f.max >> 8) as u8);
            data.push((f.max & 0xFF) as u8);
            data.push(f.port);
        }

        self.write_bytes(ControlRequest::OperacakeSetRanges, &data)
            .await
    }

    /// Match dwell times to operacake ports.
    ///
    /// These dwell time settings are used by any operacake operating in
    /// time mode.
    ///
    /// Requires API version 0x0105 or higher.
    pub async fn operacake_config_time(&self, times: &[OperacakeDwell]) -> Result<(), Error> {
        self.api_check(0x0105)?;
        if times.len() > 16 {
            return Err(Error::InvalidParameter(
                "Operacake can only support 16 time slices max",
            ));
        }
        let mut data = Vec::with_capacity(5 * times.len());
        for t in times {
            if t.port > 7 {
                return Err(Error::InvalidParameter(
                    "Operacake time slice port selection is out of range",
                ));
            }
            data.extend_from_slice(&t.dwell.to_le_bytes());
            data.push(t.port);
        }
        self.write_bytes(ControlRequest::OperacakeSetDwellTimes, &data)
            .await
    }

    /// Reset the HackRF.
    ///
    /// Requires API version 0x0102 or higher.
    pub async fn reset(&self) -> Result<(), Error> {
        self.api_check(0x0102)?;
        self.write_u16(ControlRequest::Reset, 0, 0).await
    }

    /// Turn on the CLKOUT port.
    ///
    /// Requires API version 0x0103 or higher.
    pub async fn clkout_enable(&self, enable: bool) -> Result<(), Error> {
        self.api_check(0x0103)?;
        self.write_u16(ControlRequest::ClkoutEnable, 0, enable as u16)
            .await
    }

    /// Check the CLKIN port status.
    ///
    /// Set to true if the CLKIN port is used as the reference clock.
    ///
    /// Requires API version 0x0106 or higher.
    pub async fn clkin_status(&self) -> Result<bool, Error> {
        self.api_check(0x0106)?;
        Ok(self.read_u8(ControlRequest::GetClkinStatus, 0).await? != 0)
    }

    /// Perform a GPIO test of an Operacake board.
    ///
    /// Value 0xFFFF means "GPIO mode disabled" - remove additional add-on
    /// boards and retry.
    ///
    /// Value 0 means all tests passed.
    ///
    /// In any other values, a 1 bit signals an error. Bits are grouped in
    /// groups of 3. Encoding:
    ///
    /// ```text
    /// 0 - u1ctrl - u3ctrl0 - u3ctrl1 - u2ctrl0 - u2ctrl1
    /// ```
    ///
    /// Requires API version 0x0103 or higher.
    pub async fn operacake_gpio_test(&self, address: u8) -> Result<u16, Error> {
        self.api_check(0x0103)?;
        if address > 7 {
            return Err(Error::InvalidParameter("Operacake address is out of range"));
        }
        let ret = self
            .interface
            .control_in(ControlIn {
                control_type: ControlType::Vendor,
                recipient: Recipient::Device,
                request: ControlRequest::OperacakeGpioTest as u8,
                value: address as u16,
                index: 0,
                length: 2,
            })
            .await
            .into_result()?;
        let ret: [u8; 2] = ret.as_slice().try_into().map_err(|_| Error::ReturnData)?;
        Ok(u16::from_le_bytes(ret))
    }

    /// Enable/disable the UI display on devices with one (Rad1o, PortaPack).
    ///
    /// Requires API version 0x0104 or higher.
    pub async fn set_ui_enable(&self, val: u8) -> Result<(), Error> {
        self.api_check(0x0104)?;
        self.write_u8(ControlRequest::UiEnable, 0, val).await
    }

    /// Turn the LEDs on or off, overriding the default.
    ///
    /// There are normally 3 controllable LEDs: USB, RX, and TX. The Rad1o board
    /// has 4.  After setting them individually, they may get overridden later
    /// by other functions.
    ///
    /// | Bit | LED  |
    /// | --  | --   |
    /// | 0   | USB  |
    /// | 1   | RX   |
    /// | 2   | TX   |
    /// | 3   | User |
    ///
    /// Requires API version 0x0107 or higher.
    pub async fn set_leds(&self, state: u8) -> Result<(), Error> {
        self.api_check(0x0107)?;
        self.write_u8(ControlRequest::SetLeds, 0, state).await
    }

    /// Set the Bias-Tee behavior.
    ///
    /// This function will configure what change, if any, to apply to the
    /// bias-tee circuit on a mode change. The default is for it to always be
    /// off, but with a custom config, it can turn on when switching to RX, TX,
    /// or even to always be on. The settings in `opts` are always applied when
    /// first changing to that mode, with [`BiasTMode::NoChange`] not changing
    /// from whatever it is set to before the transition.
    ///
    /// Requires API version 0x0108 or higher.
    pub async fn set_user_bias_t_opts(&self, opts: BiasTSetting) -> Result<(), Error> {
        self.api_check(0x0108)?;
        let state: u16 =
            0x124 | opts.off.as_u16() | (opts.rx.as_u16() << 3) | (opts.tx.as_u16() << 6);
        self.write_u16(ControlRequest::SetUserBiasTOpts, 0, state)
            .await
    }

    /// Switch a HackRF into receive mode, getting `transfer_size` samples at a
    /// time. The transfer size is always rounded up to the nearest 256-sample
    /// block increment; it's recommended to be 8192 samples but can be smaller
    /// or larger as needed. If the same size is used repeatedly with
    /// `start_rx`, buffers won't need to be reallocated.
    pub async fn start_rx(self, transfer_size: usize) -> Result<Receive, StateChangeError> {
        Receive::new(self, transfer_size).await
    }

    /// Start a RX sweep, which will also set the sample rate and baseband filter.
    ///
    /// Buffers are reused across sweep operations, provided that
    /// [`HackRf::start_rx`] isn't used, or is used with a 8192 sample buffer
    /// size.
    ///
    /// See [`Sweep`] for more details on the RX sweep mode.
    pub async fn start_rx_sweep(self, params: &SweepParams) -> Result<Sweep, StateChangeError> {
        Sweep::new(self, params).await
    }

    /// Start a RX sweep, but don't set up the sample rate and baseband filter before starting.
    ///
    /// Buffers are reused across sweep operations, provided that
    /// [`HackRf::start_rx`] isn't used, or is used with a 8192 sample buffer
    /// size.
    ///
    /// See [`Sweep`] for more details on the RX sweep mode.
    pub async fn start_rx_sweep_custom_sample_rate(
        self,
        params: &SweepParams,
    ) -> Result<Sweep, StateChangeError> {
        Sweep::new_with_custom_sample_rate(self, params).await
    }

    /// Switch a HackRF into transmit mode, with a set maximum number of samples
    /// per buffer block.
    ///
    /// Buffers are reused across transmit operations, provided that the
    /// `max_transfer_size` is always the same.
    pub async fn start_tx(self, max_transfer_size: usize) -> Result<Transmit, StateChangeError> {
        Transmit::new(self, max_transfer_size).await
    }

    /// Try and turn the HackRF to the off state, regardless of what mode it is currently in.
    pub async fn turn_off(&self) -> Result<(), Error> {
        self.set_transceiver_mode(TransceiverMode::Off).await
    }
}

fn baseband_filter_bw(freq: u32) -> u32 {
    const MAX2837_FT: &[u32] = &[
        1750000, 2500000, 3500000, 5000000, 5500000, 6000000, 7000000, 8000000, 9000000, 10000000,
        12000000, 14000000, 15000000, 20000000, 24000000, 28000000,
    ];

    MAX2837_FT
        .iter()
        .rev()
        .find(|f| freq >= **f)
        .copied()
        .unwrap_or(MAX2837_FT[0])
}

#[cfg(test)]
mod tests {
    use crate::baseband_filter_bw;

    #[test]
    fn baseband_filter() {
        assert_eq!(baseband_filter_bw(1000), 1750000);
        assert_eq!(baseband_filter_bw(30_000_000), 28_000_000);
        assert_eq!(baseband_filter_bw(3_000_000), 2_500_000);
    }
}
