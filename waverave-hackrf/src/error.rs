use std::ops::Range;

use crate::HackRf;

/// An error from operating the HackRF.
///
/// Some errors are recoverable:
///
/// - `Io` & `Transfer` may just be a failed packet operation on the USB cable,
///   and can potentially be recovered from without giving up on the HackRF.
/// - `AddressRange`, `ValueRange`, `TuningRange`, and `InvalidParameter` all
///   mean the arguments to a function were out of range, and may even provide a
///   hint of how to fix them.
/// - `ReturnData` means the HackRF replied to a USB transaction with something
///   unintelligible. If this is for a bulk transfer during Sweep mode, it
///   *might* be possible to recover by stopping and re-entering sweep mode, but
///   most of the time this means something is seriously wrong and
///   nonrecoverable.
/// - `ApiVersion` indicates the firmware on the HackRF is too old to support
///   this function. Bail out, and advise the user to update their HackRF's
///   firmware.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum Error {
    /// Underlying OS I/O error.
    #[error("I/O error")]
    Io(#[from] std::io::Error),

    /// Transfer error from `nusb`.
    #[error("USB transfer error")]
    Transfer(#[from] nusb::transfer::TransferError),

    /// The provided address (usually for register I/O) is out of range.
    #[error("Address (0x{addr:x}) out of range (0x{}..0x{})", .range.start, .range.end)]
    #[allow(missing_docs)]
    AddressRange { range: Range<u32>, addr: u32 },

    /// The provided argument value is out of range.
    #[error("Value (0x{val:x}) out of range (0x{}..0x{})", .range.start, .range.end)]
    #[allow(missing_docs)]
    ValueRange { range: Range<u32>, val: u32 },

    /// The provided tuning frequency is out of range.
    #[error("Tuning Value ({val:x} Hz) out of range ({}..{} Hz)", .range.start, .range.end)]
    #[allow(missing_docs)]
    TuningRange { range: Range<u64>, val: u64 },

    /// Some argument to a function is invalid in a way not easily expressed as
    /// a range.
    #[error("Invalid Parameter: {0}")]
    InvalidParameter(&'static str),

    /// The API version of the opened HackRF is too old for this function.
    #[error(
        "Requires API >= 0x{:x}, but device has API 0x{:x}. Consider updating the firmware",
        needed,
        actual
    )]
    #[allow(missing_docs)]
    ApiVersion { needed: u16, actual: u16 },

    /// Returned data from a HackRF didn't make any sense.
    #[error("Invalid return data")]
    ReturnData,
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
