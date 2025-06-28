use std::ops::Range;

use crate::HackRf;

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
    #[error("Invalid Parameter: {0}")]
    InvalidParameter(&'static str),
    #[error("USB transfer pipe is full of pending transfers")]
    TransferBusy,
}

/// An error from trying to change HackRF states and failing.
/// 
/// Always contains the HackRF instance, but in an unknown state. To try and
/// return to a known state, try calling [`HackRf::turn_off`].
pub struct StateChangeError {
    /// The error that occurred while trying to change state.
    pub err: Error,
    /// The HackRF instance.
    pub rf: HackRf,
}

impl core::error::Error for StateChangeError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        Some(&self.err)
    }
}

impl core::fmt::Display for StateChangeError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("Failed to change state")
    }
}

impl core::fmt::Debug for StateChangeError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("StateChangeError")
            .field("err", &self.err)
            .finish_non_exhaustive()
    }
}