#![allow(dead_code)]

mod hackrf_consts;
use std::{io::Write, ops::Range};

use bytemuck::Pod;
pub use hackrf_consts::*;
use nusb::transfer::{ControlIn, ControlOut, ControlType, Recipient};

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

#[repr(C)]
#[derive(Clone, Copy, Debug, bytemuck::Zeroable, bytemuck::Pod)]
pub struct M0State {
    /// Requested Mode. Possible values are: 0(IDLE), 1(WAIT), 2(RX),
    /// 3(TX_START), 4(TX_RUN)
    pub requested_mode: u16,
    /// Request flag, 0 means request is completed, any other value means
    /// request is pending
    pub request_flag: u16,
    /// Active mode. Possible values are: 0(IDLE), 1(WAIT), 2(RX), 3(TX_START),
    /// 4(TX_RUN)
    pub active_mode: u32,
    /// Number of bytes transferred by M0
    pub m0_count: u32,
    /// Number of bytes transferred by M4
    pub m4_count: u32,
    /// Number of shortfalls
    pub num_shortfalls: u32,
    /// Longest shortfall in bytes
    pub longest_shortfall: u32,
    /// Shortfall limit in bytes
    pub shortfall_limit: u32,
    /// Threshold `m0_count` value in bytes for next mode change
    pub threshold: u32,
    /// Mode which will be switched to when threshold is reached.
    pub next_mode: u32,
    /// Error, if any, that caused M0 to revert to IDLE mode. Possible values
    /// are: 0 (NONE), 1 (RX_TIMEOUT), 2 (TX_TIMEOUT), or 3 (MISSED_DEADLINE)
    pub error: u32,
}

impl M0State {
    fn le_convert(&mut self) {
        self.requested_mode = self.requested_mode.to_le();
        self.request_flag = self.request_flag.to_le();
        self.active_mode = self.active_mode.to_le();
        self.m0_count = self.m0_count.to_le();
        self.m4_count = self.m4_count.to_le();
        self.num_shortfalls = self.num_shortfalls.to_le();
        self.longest_shortfall = self.longest_shortfall.to_le();
        self.shortfall_limit = self.shortfall_limit.to_le();
        self.threshold = self.threshold.to_le();
        self.next_mode = self.next_mode.to_le();
        self.error = self.error.to_le();
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

#[repr(C)]
#[derive(Clone, Copy, Debug, bytemuck::Zeroable, bytemuck::Pod)]
pub struct SerialNumber {
    part_id: [u32; 2],
    serial_no: [u32; 4],
}

impl SerialNumber {
    fn le_convert(&mut self) {
        for x in self.part_id.iter_mut() {
            *x = x.to_le();
        }
        for x in self.serial_no.iter_mut() {
            *x = x.to_le();
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub enum BoardRev {
    Old,
    R6,
    R7,
    R8,
    R9,
    R10,
    GsgR6,
    GsgR7,
    GsgR8,
    GsgR9,
    GsgR10,
    Unknown(u8),
}

impl std::fmt::Display for BoardRev {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Old => f.write_str("Older than r6"),
            Self::R6 => f.write_str("r6"),
            Self::R7 => f.write_str("r7"),
            Self::R8 => f.write_str("r8"),
            Self::R9 => f.write_str("r9"),
            Self::R10 => f.write_str("r10"),
            Self::GsgR6 => f.write_str("Great Scott Gadgets r6"),
            Self::GsgR7 => f.write_str("Great Scott Gadgets r7"),
            Self::GsgR8 => f.write_str("Great Scott Gadgets r8"),
            Self::GsgR9 => f.write_str("Great Scott Gadgets r9"),
            Self::GsgR10 => f.write_str("Great Scott Gadgets r10"),
            Self::Unknown(v) => write!(f, "unknown (0x{:x})", v),
        }
    }
}

impl BoardRev {
    fn from_u8(v: u8) -> Self {
        use BoardRev::*;
        match v {
            0 => Old,
            1 => R6,
            2 => R7,
            3 => R8,
            4 => R9,
            5 => R10,
            0x81 => GsgR6,
            0x82 => GsgR7,
            0x83 => GsgR8,
            0x84 => GsgR9,
            0x85 => GsgR10,
            v => Unknown(v),
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub enum BoardId {
    Jellybean,
    Jawbreaker,
    HackRf1Og,
    Rad1o,
    HackRf1R9,
    Unknown(u8),
}

impl std::fmt::Display for BoardId {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Jellybean => f.write_str("Jellybean"),
            Self::Jawbreaker => f.write_str("Jawbreaker"),
            Self::HackRf1Og => f.write_str("HackRF One"),
            Self::Rad1o => f.write_str("rad1o"),
            Self::HackRf1R9 => f.write_str("HackRF One Rev9"),
            Self::Unknown(v) => write!(f, "Unknown (0x{:x})", v),
        }
    }
}

impl BoardId {
    fn from_u8(v: u8) -> Self {
        use BoardId::*;
        match v {
            0 => Jellybean,
            1 => Jawbreaker,
            2 => HackRf1Og,
            3 => Rad1o,
            4 => HackRf1R9,
            v => Unknown(v),
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct SupportedPlatform {
    pub jawbreaker: bool,
    pub hackrf1_og: bool,
    pub rad1o: bool,
    pub hackrf1_r9: bool,
}

impl SupportedPlatform {
    fn from_u32(v: u32) -> Self {
        Self {
            jawbreaker: v & 1 != 0,
            hackrf1_og: v & 2 != 0,
            rad1o: v & 4 != 0,
            hackrf1_r9: v & 8 != 0,
        }
    }
}

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("I/O error")]
    Io(#[from] std::io::Error),
    #[error("USB transfer error")]
    Transfer(#[from] nusb::transfer::TransferError),
    #[error("Address (0x{addr:x}) out of range (0x{}..0x{})", .range.start, .range.end)]
    AddressRange { range: Range<u32>, addr: u32 },
    #[error("Value (0x{val:x}) out of range (0x{}..0x{})", .range.start, .range.end)]
    ValueRange { range: Range<u32>, val: u32 },
    #[error("Tuning Value ({val:x} Hz) out of range ({}..{} Hz)", .range.start, .range.end)]
    TuningRange { range: Range<u64>, val: u64 },
    #[error("Invalid return data")]
    ReturnData,
    #[error("Requires API >= 0x{:x}, but device has API 0x{:x}", needed, actual)]
    ApiVersion { needed: u16, actual: u16 },
    #[error("Invalid Parameter")]
    InvalidParameter,
}

pub struct HackRf {
    interface: nusb::Interface,
    version: u16,
    ty: HackRfType,
}

pub struct HackRfDescriptor {
    info: nusb::DeviceInfo,
}

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

pub fn open_hackrf() -> Result<HackRf, std::io::Error> {
    list_hackrf_devices()?
        .into_iter()
        .next()
        .ok_or_else(|| std::io::Error::other("No HackRF devices"))?
        .open()
}

pub struct SpiFlash<'a> {
    inner: &'a HackRf,
}

impl SpiFlash<'_> {
    pub async fn erase(&self) -> Result<(), Error> {
        self.inner
            .write_u8(ControlRequest::SpiflashErase, 0, 0)
            .await
    }

    pub async fn write(&self, addr: u32, data: &[u8]) -> Result<(), Error> {
        const END_ADDR: u32 = 0x100000;
        if addr >= END_ADDR {
            return Err(Error::AddressRange {
                range: Range {
                    start: 0,
                    end: END_ADDR,
                },
                addr,
            });
        }

        if (data.len() + addr as usize) > (END_ADDR as usize) {
            let end = END_ADDR - addr;
            return Err(Error::ValueRange {
                range: Range { start: 0, end },
                val: data.len() as u32,
            });
        }

        if data.len() >= 0x10000 {
            return Err(Error::ValueRange {
                range: Range {
                    start: 0,
                    end: 0x10000,
                },
                val: data.len() as u32,
            });
        }

        self.inner
            .interface
            .control_out(ControlOut {
                control_type: ControlType::Vendor,
                recipient: Recipient::Device,
                request: ControlRequest::SpiflashWrite as u8,
                value: (addr >> 16) as u16,
                index: (addr & 0xFFFF) as u16,
                data,
            })
            .await
            .into_result()?;
        Ok(())
    }

    pub async fn read(&self, addr: u32, len: usize) -> Result<Vec<u8>, Error> {
        const END_ADDR: u32 = 0x100000;
        if addr >= END_ADDR {
            return Err(Error::AddressRange {
                range: Range {
                    start: 0,
                    end: END_ADDR,
                },
                addr,
            });
        }

        if (len + addr as usize) > (END_ADDR as usize) {
            let end = END_ADDR - addr;
            return Err(Error::ValueRange {
                range: Range { start: 0, end },
                val: len as u32,
            });
        }

        if len >= 0x10000 {
            return Err(Error::ValueRange {
                range: Range {
                    start: 0,
                    end: 0x10000,
                },
                val: len as u32,
            });
        }

        let resp = self
            .inner
            .interface
            .control_in(ControlIn {
                control_type: ControlType::Vendor,
                recipient: Recipient::Device,
                request: ControlRequest::SpiflashRead as u8,
                value: (addr >> 16) as u16,
                index: (addr & 0xFFFF) as u16,
                length: len as u16,
            })
            .await
            .into_result()?;
        Ok(resp)
    }

    pub async fn status(&self) -> Result<[u8; 2], Error> {
        self.inner.api_check(0x0103)?;
        let val = self
            .inner
            .read_u16(ControlRequest::SpiflashStatus, 0)
            .await?;
        Ok(val.to_le_bytes())
    }

    pub async fn clear_status(&self) -> Result<(), Error> {
        self.inner.api_check(0x0103)?;
        self.inner
            .write_u16(ControlRequest::SpiflashClearStatus, 0, 0)
            .await
    }
}

impl HackRf {
    /// Get the device's implemented API version, as a binary-coded decimal
    /// (BCD) value.
    pub fn api_version(&self) -> u16 {
        self.version
    }

    /// Get the type of HackRF radio.
    pub fn radio_type(&self) -> HackRfType {
        self.ty
    }

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

    pub async fn set_transceiver_mode(&self, mode: TransceiverMode) -> Result<(), Error> {
        self.write_u16(ControlRequest::SetTransceiverMode, 0, mode as u16)
            .await
    }

    pub async fn max2837_read(&self, register: u8) -> Result<u16, Error> {
        if register >= 32 {
            return Err(Error::AddressRange {
                range: Range { start: 0, end: 32 },
                addr: register as u32,
            });
        }

        self.read_u16(ControlRequest::Max2837Read, register as u16)
            .await
    }

    pub async fn max2837_write(&self, register: u8, value: u16) -> Result<(), Error> {
        if register >= 32 {
            return Err(Error::AddressRange {
                range: Range { start: 0, end: 32 },
                addr: register as u32,
            });
        }

        if value >= 0x400 {
            return Err(Error::ValueRange {
                range: Range {
                    start: 0,
                    end: 0x400,
                },
                val: value as u32,
            });
        }

        self.write_u16(ControlRequest::Max2837Write, register as u16, value)
            .await
    }

    pub async fn si5351c_read(&self, register: u8) -> Result<u8, Error> {
        self.read_u8(ControlRequest::Si5351cRead, register as u16)
            .await
    }

    pub async fn si5351c_write(&self, register: u8, value: u8) -> Result<(), Error> {
        self.write_u8(ControlRequest::Si5351cWrite, register as u16, value)
            .await
    }

    pub async fn set_baseband_filter_bandwidth(&self, bandwidth_hz: u32) -> Result<(), Error> {
        self.write_u32(ControlRequest::BasebandFilterBandwidthSet, bandwidth_hz)
            .await
    }

    pub async fn rffc5071_read(&self, register: u8) -> Result<u16, Error> {
        if register >= 31 {
            return Err(Error::AddressRange {
                range: Range { start: 0, end: 31 },
                addr: register as u32,
            });
        }

        self.read_u16(ControlRequest::Rffc5071Read, register as u16)
            .await
    }

    pub async fn rffc5071_write(&self, register: u8, value: u16) -> Result<(), Error> {
        if register >= 31 {
            return Err(Error::AddressRange {
                range: Range { start: 0, end: 31 },
                addr: register as u32,
            });
        }

        self.write_u16(ControlRequest::Rffc5071Write, register as u16, value)
            .await
    }

    pub async fn get_m0_state(&self) -> Result<M0State, Error> {
        self.api_check(0x0106)?;
        let mut v: M0State = self.read_struct(ControlRequest::GetM0State).await?;
        v.le_convert();
        Ok(v)
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

    /// Access the attached SPI flash.
    pub fn spi_flash(&self) -> SpiFlash<'_> {
        SpiFlash { inner: self }
    }

    /// Update the CPLD with a new bitstream.
    ///
    /// Modifies the transceiver mode in order to do the update. Don't do this
    /// if actively using the RF chain.
    ///
    /// After every transfer completes, an optional callback will be invoked
    /// with the number of bytes transferred as the first argument, and the
    /// total number of bytes to be transferred as the second argument.
    pub async fn cpld_write<F>(&self, data: &[u8], mut callback: Option<F>) -> Result<(), Error>
    where
        F: FnMut(usize, usize),
    {
        const CHUNK_SIZE: usize = 512;
        self.set_transceiver_mode(TransceiverMode::CpldUpdate)
            .await?;
        let mut queue = self.interface.bulk_out_queue(TX_ENDPOINT_ADDRESS);
        let mut sent = 0;
        let total = data.len();
        for chunk in data.chunks(CHUNK_SIZE) {
            let mut buf = if queue.pending() != 0 {
                let resp = queue.next_complete().await.into_result()?;
                sent += resp.actual_length();
                if let Some(ref mut c) = callback {
                    c(sent, total);
                }
                resp.reuse()
            } else {
                Vec::with_capacity(CHUNK_SIZE)
            };
            buf.copy_from_slice(chunk);
            queue.submit(buf);
        }
        while queue.pending() != 0 {
            let resp = queue.next_complete().await.into_result()?;
            sent += resp.actual_length();
            if let Some(ref mut c) = callback {
                c(sent, total);
            }
        }
        Ok(())
    }

    pub async fn board_id_read(&self) -> Result<BoardId, Error> {
        let ret = self.read_u8(ControlRequest::BoardIdRead, 0).await?;
        Ok(BoardId::from_u8(ret))
    }

    pub async fn version_string_read(&self) -> Result<String, Error> {
        let resp = self
            .read_bytes(ControlRequest::VersionStringRead, 255)
            .await?;
        String::from_utf8(resp).map_err(|_| Error::ReturnData)
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

    pub async fn read_serial(&self) -> Result<SerialNumber, Error> {
        let mut v: SerialNumber = self
            .read_struct(ControlRequest::BoardPartidSerialnoRead)
            .await?;
        v.le_convert();
        Ok(v)
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
            return Err(Error::InvalidParameter);
        }
        match setting {
            OperacakeMode::Manual { a, b } => {
                let a = *a as u16;
                let b = *b as u16;
                if a > 7 || b > 7 {
                    return Err(Error::InvalidParameter);
                }
                if (a < 4 && b < 4) || (a >= 4 && b >= 4) {
                    return Err(Error::InvalidParameter);
                }
                self.write_u8(ControlRequest::OperacakeSetMode, 0, address)
                    .await?;
                self.write_u8(ControlRequest::OperacakeSetPorts, a | (b << 8), address)
                    .await
            }
            OperacakeMode::Frequency(f) => {
                let freqs = f.as_slice();
                if freqs.len() > 8 {
                    return Err(Error::InvalidParameter);
                }
                let mut data = Vec::with_capacity(5 * freqs.len());
                for f in freqs {
                    if f.port > 7 {
                        return Err(Error::InvalidParameter);
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
                    return Err(Error::InvalidParameter);
                }
                let mut data = Vec::with_capacity(5 * times.len());
                for t in times {
                    if t.port > 7 {
                        return Err(Error::InvalidParameter);
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
            return Err(Error::InvalidParameter);
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

    pub async fn cpld_checksum(&self) -> Result<u32, Error> {
        self.api_check(0x0103)?;
        let ret = self.read_bytes(ControlRequest::CpldChecksum, 4).await?;
        let ret: [u8; 4] = ret.as_slice().try_into().map_err(|_| Error::ReturnData)?;
        Ok(u32::from_le_bytes(ret))
    }

    pub async fn set_ui_enable(&self, val: u8) -> Result<(), Error> {
        self.api_check(0x0104)?;
        self.write_u8(ControlRequest::UiEnable, 0, val).await
    }

    pub async fn rev_read(&self) -> Result<BoardRev, Error> {
        self.api_check(0x0106)?;
        let rev = self.read_u8(ControlRequest::BoardRevRead, 0).await?;
        Ok(BoardRev::from_u8(rev))
    }

    pub async fn supported_platform_read(&self) -> Result<SupportedPlatform, Error> {
        self.api_check(0x0106)?;
        let ret = self
            .read_bytes(ControlRequest::SupportedPlatformRead, 4)
            .await?;
        let ret: [u8; 4] = ret.as_slice().try_into().map_err(|_| Error::ReturnData)?;
        let val = u32::from_le_bytes(ret);
        Ok(SupportedPlatform::from_u32(val))
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

    pub async fn start_rx(&self) -> Result<(), Error> {
        todo!("Start RX")
    }

    pub async fn start_rx_sweep(&self) -> Result<(), Error> {
        todo!("Start RX Sweep")
    }

    pub async fn start_tx(&self) -> Result<(), Error> {
        todo!("Start TX Sweep")
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
