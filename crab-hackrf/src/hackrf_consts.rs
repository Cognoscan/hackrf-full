pub const HACKRF_USB_VID: u16 = 0x1d50;
pub const HACKRF_JAWBREAKER_USB_PID: u16 = 0x604b;
pub const HACKRF_ONE_USB_PID: u16 = 0x6089;
pub const RAD1O_USB_PID: u16 = 0xcc15;

#[repr(u8)]
pub enum ControlRequest {
    SetTransceiverMode = 1,
    Max2837Write = 2,
    Max2837Read = 3,
    Si5351cWrite = 4,
    Si5351cRead = 5,
    SampleRateSet = 6,
    BasebandFilterBandwidthSet = 7,
    Rffc5071Write = 8,
    Rffc5071Read = 9,
    SpiflashErase = 10,
    SpiflashWrite = 11,
    SpiflashRead = 12,
    BoardIdRead = 14,
    VersionStringRead = 15,
    SetFreq = 16,
    AmpEnable = 17,
    BoardPartidSerialnoRead = 18,
    SetLnaGain = 19,
    SetVgaGain = 20,
    SetTxvgaGain = 21,
    AntennaEnable = 23,
    SetFreqExplicit = 24,
    UsbWcidVendorReq = 25,
    InitSweep = 26,
    OperacakeGetBoards = 27,
    OperacakeSetPorts = 28,
    SetHwSyncMode = 29,
    Reset = 30,
    OperacakeSetRanges = 31,
    ClkoutEnable = 32,
    SpiflashStatus = 33,
    SpiflashClearStatus = 34,
    OperacakeGpioTest = 35,
    CpldChecksum = 36,
    UiEnable = 37,
    OperacakeSetMode = 38,
    OperacakeGetMode = 39,
    OperacakeSetDwellTimes = 40,
    GetM0State = 41,
    SetTxUnderrunLimit = 42,
    SetRxOverrunLimit = 43,
    GetClkinStatus = 44,
    BoardRevRead = 45,
    SupportedPlatformRead = 46,
    SetLeds = 47,
    SetUserBiasTOpts = 48,
}

pub const RX_ENDPOINT_ADDRESS: u8 = 0x81;
pub const TX_ENDPOINT_ADDRESS: u8 = 0x02;

#[repr(u16)]
pub enum TransceiverMode {
    Off = 0,
    Receive = 1,
    Transmit = 2,
    Ss = 3,
    CpldUpdate = 4,
    RxSweep = 5,
}

#[repr(u8)]
pub enum SyncMode {
    Off = 0,
    On = 1,
}

pub const TRANSFER_COUNT: u8 = 4;
pub const TRANSFER_BUFFER_SIZE: u32 = 262144;
pub const DEVICE_BUFFER_SIZE: u32 = 32768;
pub const USB_MAX_SERIAL_LENGTH: u32 = 32;
