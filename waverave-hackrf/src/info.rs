//! Get information about a HackRF board.
//!
//! This module contains the [`Info`] struct for accessing information from the
//! HackRF, which can be used to get:
//!
//! - The MCU's [serial number][SerialNumber] with [Info::serial].
//! - The ["compatible" platforms][SupportedPlatform] for a given board, with [Info::supported_platform]
//! - The [board identifier][BoardId], with [Info::board_id]
//! - The [board revision][BoardRev], with [Info::board_rev]
//!
//! The general way to do this with a HackRF is:
//!
//! ```no_run
//!
//! # use anyhow::Result;
//! # #[tokio::main]
//! # async fn main() -> Result<()> {
//!
//! use waverave_hackrf::info::*;
//!
//! let hackrf = waverave_hackrf::open_hackrf()?;
//! let info = hackrf.info();
//!
//! let serial: SerialNumber = info.serial().await?;
//! let compatible: SupportedPlatform = info.supported_platform().await?;
//! let board_id: BoardId = info.board_id().await?;
//! let board_rev: BoardRev = info.board_rev().await?;
//!
//! # Ok(())
//! # }
//! ```
use crate::{Error, HackRf, HackRfType, consts::ControlRequest};

/// The MCU serial number.
///
/// The Part ID identifies the exact LPC43xx part that was populated. See the
/// user manual for the exact decoding, but you're likely to find `0xa000cb3c`
/// for `part_id[0]`.
///
/// The "serial number" is referred to as the device unique ID in the user
/// manual for the LPC43x. It seems that only the last two 32-bit words are
/// nonzero, though this isn't guaranteed.
///
/// See the LPC43xx documentation for full details.
#[repr(C)]
#[derive(Clone, Copy, Debug, bytemuck::Zeroable, bytemuck::Pod)]
pub struct SerialNumber {
    pub part_id: [u32; 2],
    pub serial_no: [u32; 4],
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

/// The board revision.
///
/// The Great Scott Gadgets official board revisions are prefixed with "Gsg".
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
    /// Check if this is marked as an official Great Scott Gadgets board.
    pub fn is_official(&self) -> bool {
        use BoardRev::*;
        matches!(self, GsgR6 | GsgR7 | GsgR8 | GsgR9 | GsgR10)
    }

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

/// The physical board's identifier. These differentiate between board hardware
/// that's actually different.
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

/// Compatible platforms for this board.
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

/// Info-gathering operations for the HackRF.
///
/// Borrows the interface while doing operations.
pub struct Info<'a> {
    inner: &'a HackRf,
}

impl<'a> Info<'a> {
    pub(crate) fn new(inner: &'a HackRf) -> Info<'a> {
        Self { inner }
    }

    /// Get the device's implemented API version, as a binary-coded decimal
    /// (BCD) value.
    pub fn api_version(&self) -> u16 {
        self.inner.version
    }

    /// Get the [type][HackRfType] of HackRF radio.
    pub fn radio_type(&self) -> HackRfType {
        self.inner.ty
    }

    /// Get the [board hardware ID][BoardId].
    pub async fn board_id(&self) -> Result<BoardId, Error> {
        let ret = self.inner.read_u8(ControlRequest::BoardIdRead, 0).await?;
        Ok(BoardId::from_u8(ret))
    }

    /// Get the firmware version as a string.
    pub async fn version_string(&self) -> Result<String, Error> {
        let resp = self
            .inner
            .read_bytes(ControlRequest::VersionStringRead, 255)
            .await?;
        String::from_utf8(resp).map_err(|_| Error::ReturnData)
    }

    /// Get the MCU's serial numbers.
    ///
    /// In the LP43xx documentation, this refers to the device unique ID and the
    /// part identification number.
    ///
    /// See [`SerialNumber`] for more info.
    pub async fn serial(&self) -> Result<SerialNumber, Error> {
        let mut v: SerialNumber = self
            .inner
            .read_struct(ControlRequest::BoardPartidSerialnoRead)
            .await?;
        v.le_convert();
        Ok(v)
    }

    /// Read the board's [revision number][BoardRev].
    ///
    /// Requires API version 0x0106 or higher.
    pub async fn board_rev(&self) -> Result<BoardRev, Error> {
        self.inner.api_check(0x0106)?;
        let rev = self.inner.read_u8(ControlRequest::BoardRevRead, 0).await?;
        Ok(BoardRev::from_u8(rev))
    }

    /// Read the platforms [compatible][SupportedPlatform] with this board.
    ///
    /// Requires API version 0x0106 or higher.
    pub async fn supported_platform(&self) -> Result<SupportedPlatform, Error> {
        self.inner.api_check(0x0106)?;
        let ret = self
            .inner
            .read_bytes(ControlRequest::SupportedPlatformRead, 4)
            .await?;
        let ret: [u8; 4] = ret.as_slice().try_into().map_err(|_| Error::ReturnData)?;
        let val = u32::from_be_bytes(ret);
        Ok(SupportedPlatform::from_u32(val))
    }
}
