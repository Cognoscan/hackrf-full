use std::ops::Range;


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
    #[error("USB transfer pipe is full of pending transfers")]
    TransferBusy,
}
