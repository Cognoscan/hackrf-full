#![allow(dead_code)]

mod consts;
pub mod debug;
mod error;
pub mod info;
mod rx;
mod sweep;
mod tx;
use std::ops::Range;

use bytemuck::Pod;
use consts::*;
pub use debug::Debug;
pub use error::{Error, StateChangeError};
pub use info::Info;
use nusb::transfer::{ControlIn, ControlOut, ControlType, Recipient};
pub use rx::Receive;
use std::sync::mpsc;
pub use sweep::{Sweep, SweepBuf, SweepParams};

pub type ComplexI8 = num_complex::Complex<i8>;

/// A Buffer holding USB transfer data.
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

    pub fn capacity(&self) -> usize {
        // Force down to the nearest 512-byte boundary, which is the transfer
        // size the HackRF requires.
        (self.buf.capacity() & !0x1FF) / core::mem::size_of::<ComplexI8>()
    }

    //// Clear out the buffer's samples.
    pub fn clear(&mut self) {
        self.buf.clear();
    }

    /// Size of the buffer, in samples.
    pub fn len(&self) -> usize {
        self.buf.len() / core::mem::size_of::<ComplexI8>()
    }

    /// Returns true if there are no samples in the buffer.
    pub fn is_empty(&self) -> bool {
        self.buf.is_empty()
    }

    /// Remaining capacity in the buffer, in samples.
    pub fn remaining_capacity(&self) -> usize {
        self.capacity() - self.len()
    }

    /// Extend the buffer with a slice of samples.
    ///
    /// # Panics
    /// - if there is no space left in the buffer for the slice.
    pub fn extend_from_slice(&mut self, slice: &[ComplexI8]) {
        assert!(self.remaining_capacity() >= slice.len());
        let slice =
            unsafe { core::slice::from_raw_parts(slice.as_ptr() as *const u8, slice.len() * 2) };
        self.buf.extend_from_slice(slice);
    }

    /// Push a value onto the buffer.
    ///
    /// # Panics
    /// - if there is no space left in the buffer.
    pub fn push(&mut self, val: ComplexI8) {
        assert!(self.remaining_capacity() > 0);
        let slice: &[u8; 2] = unsafe { &*((&val) as *const ComplexI8 as *const [u8; 2]) };
        self.buf.extend_from_slice(slice);
    }

    /// Get the sample sequence as a slice of bytes instead of complex values.
    pub fn bytes(&self) -> &[u8] {
        &self.buf
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
                self.buf.len() / core::mem::size_of::<ComplexI8>(),
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
                self.buf.len() / core::mem::size_of::<ComplexI8>(),
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

#[derive(Clone, Debug)]
pub struct BiasTSetting {
    tx: BiasTMode,
    rx: BiasTMode,
    off: BiasTMode,
}

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
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RfPathFilter {
    /// No filter selected
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

#[derive(Clone, Debug)]
pub enum OperacakeMode {
    /// Switched manually using `set_ports`.
    Manual { a: u8, b: u8 },
    /// Switched automatically when frequency is changed.
    Frequency(Vec<OperacakeFreq>),
    /// Dwell Time mode.
    Time(Vec<OpercakeDwell>),
}

#[derive(Clone, Copy, Debug)]
pub struct OperacakeFreq {
    /// Start frequency, in MHz.
    min: u16,
    /// Stop frequency, in MHz.
    max: u16,
    /// Port for A0 to use for the range. B0 will use the mirror image.
    port: u8,
}

#[derive(Clone, Copy, Debug)]
pub struct OpercakeDwell {
    /// Dwell time, in number of samples
    dwell: u32,
    /// Port for A0 to use for the range. B0 will use the mirror image.
    port: u8,
}

/// A HackRF device. This is the main struct for talking to the HackRF.
pub struct HackRf {
    pub(crate) interface: nusb::Interface,
    pub(crate) version: u16,
    pub(crate) ty: HackRfType,
}

/// A HackRF device descriptor, which can be opened.
pub struct HackRfDescriptor {
    info: nusb::DeviceInfo,
}

/// The type of HackRF device that was detected.
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
    pub fn serial(&self) -> Option<&str> {
        self.info.serial_number()
    }

    pub fn radio_type(&self) -> HackRfType {
        match self.info.product_id() {
            HACKRF_JAWBREAKER_USB_PID => HackRfType::Jawbreaker,
            HACKRF_ONE_USB_PID => HackRfType::One,
            RAD1O_USB_PID => HackRfType::Rad1o,
            _ => panic!("Created a HackRfDescriptor without using a known product ID"),
        }
    }

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

        Ok(HackRf {
            interface,
            version,
            ty,
        })
    }
}

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
pub fn open_hackrf() -> Result<HackRf, std::io::Error> {
    list_hackrf_devices()?
        .into_iter()
        .next()
        .ok_or_else(|| std::io::Error::other("No HackRF devices"))?
        .open()
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
        let size = core::mem::size_of::<T>();
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

    pub async fn set_baseband_filter_bandwidth(&self, bandwidth_hz: u32) -> Result<(), Error> {
        self.write_u32(ControlRequest::BasebandFilterBandwidthSet, bandwidth_hz)
            .await
    }

    pub async fn set_tx_underrun_limit(&self, val: u32) -> Result<(), Error> {
        self.api_check(0x0106)?;
        self.write_u32(ControlRequest::SetTxUnderrunLimit, val)
            .await
    }

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

    pub async fn set_amp_enable(&self, enable: bool) -> Result<(), Error> {
        self.write_u16(ControlRequest::AmpEnable, 0, enable as u16)
            .await
    }

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
        if ret != 0 {
            return Err(Error::ReturnData);
        }
        Ok(())
    }

    pub async fn set_vga_gain(&self, value: u16) -> Result<(), Error> {
        if value > 62 {
            return Err(Error::ValueRange {
                range: Range { start: 0, end: 63 },
                val: value as u32,
            });
        }

        let ret = self
            .read_u8(ControlRequest::SetLnaGain, value & (!0x01))
            .await?;
        if ret != 0 {
            return Err(Error::ReturnData);
        }
        Ok(())
    }

    pub async fn set_txvga_gain(&self, value: u16) -> Result<(), Error> {
        if value > 47 {
            return Err(Error::ValueRange {
                range: Range { start: 0, end: 48 },
                val: value as u32,
            });
        }

        let ret = self.read_u8(ControlRequest::SetLnaGain, value).await?;
        if ret != 0 {
            return Err(Error::ReturnData);
        }
        Ok(())
    }

    pub async fn set_antenna_enable(&self, enable: bool) -> Result<(), Error> {
        self.write_u16(ControlRequest::AntennaEnable, 0, enable as u16)
            .await
    }

    /// Set hardware sync mode (hardware triggering).
    ///
    /// Requires API version 0x0102 or higher.
    pub async fn set_hw_sync_mode(&self, enable: bool) -> Result<(), Error> {
        self.api_check(0x0102)?;
        self.write_u16(ControlRequest::SetHwSyncMode, 0, enable as u16)
            .await
    }

    pub async fn operacake_boards(&self) -> Result<Vec<u8>, Error> {
        self.api_check(0x0105)?;
        let mut resp = self
            .read_bytes(ControlRequest::OperacakeGetBoards, 8)
            .await?;
        resp.retain(|&x| x != 0xFF);
        Ok(resp)
    }

    pub async fn operacake_config(
        &self,
        address: u8,
        setting: &OperacakeMode,
    ) -> Result<(), Error> {
        self.api_check(0x0105)?;
        if address > 7 {
            return Err(Error::InvalidParameter("Operacake address is out of range"));
        }
        match setting {
            OperacakeMode::Manual { a, b } => {
                let a = *a as u16;
                let b = *b as u16;
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
                self.write_u8(ControlRequest::OperacakeSetMode, 0, address)
                    .await?;
                self.write_u8(ControlRequest::OperacakeSetPorts, a | (b << 8), address)
                    .await
            }
            OperacakeMode::Frequency(f) => {
                let freqs = f.as_slice();
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

                self.write_u8(ControlRequest::OperacakeSetMode, 1, address)
                    .await?;
                self.write_bytes(ControlRequest::OperacakeSetRanges, &data)
                    .await
            }
            OperacakeMode::Time(t) => {
                let times = t.as_slice();
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
                self.write_u8(ControlRequest::OperacakeSetMode, 2, address)
                    .await?;
                self.write_bytes(ControlRequest::OperacakeSetRanges, &data)
                    .await
            }
        }
    }

    pub async fn reset(&self) -> Result<(), Error> {
        self.api_check(0x0102)?;
        self.write_u16(ControlRequest::Reset, 0, 0).await
    }

    pub async fn clkout_enable(&self, enable: bool) -> Result<(), Error> {
        self.api_check(0x0103)?;
        self.write_u16(ControlRequest::ClkoutEnable, 0, enable as u16)
            .await
    }

    pub async fn clkin_status(&self) -> Result<bool, Error> {
        self.api_check(0x0106)?;
        Ok(self.read_u8(ControlRequest::GetClkinStatus, 0).await? != 0)
    }

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

    pub async fn set_ui_enable(&self, val: u8) -> Result<(), Error> {
        self.api_check(0x0104)?;
        self.write_u8(ControlRequest::UiEnable, 0, val).await
    }

    pub async fn set_leds(&self, state: u8) -> Result<(), Error> {
        self.api_check(0x0107)?;
        self.write_u8(ControlRequest::SetLeds, 0, state).await
    }

    pub async fn set_user_bias_t_opts(&self, opts: BiasTSetting) -> Result<(), Error> {
        self.api_check(0x0108)?;
        let state: u16 =
            0x124 | opts.off.as_u16() | (opts.rx.as_u16() << 3) | (opts.tx.as_u16() << 6);
        self.write_u16(ControlRequest::SetUserBiasTOpts, 0, state)
            .await
    }

    /// Start receiving data.
    pub async fn start_rx(self, transfer_size: usize) -> Result<Receive, StateChangeError> {
        Receive::new(self, transfer_size).await
    }

    /// Start a RX sweep, which will also set the sample rate and baseband filter.
    pub async fn start_rx_sweep(self, params: &SweepParams) -> Result<Sweep, StateChangeError> {
        Sweep::new(self, params).await
    }

    /// Start a RX sweep, but don't set up the sample rate and baseband filter before starting.
    pub async fn start_rx_sweep_custom_sample_rate(
        self,
        params: &SweepParams,
    ) -> Result<Sweep, StateChangeError> {
        Sweep::new_with_custom_sample_rate(self, params).await
    }

    pub async fn start_tx(&self) -> Result<(), Error> {
        todo!("Start TX")
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
