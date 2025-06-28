use std::ops::Range;

use nusb::transfer::{ControlIn, ControlOut, ControlType, Recipient};

use crate::{ControlRequest, Error, HackRf, TransceiverMode};

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

/// Debug operations for the HackRF, including programming operations.
///
/// Borrows the interface while doing operations.
pub struct Debug<'a> {
    inner: &'a mut HackRf,
}

impl<'a> Debug<'a> {
    pub(crate) fn new(inner: &'a mut HackRf) -> Debug<'a> {
        Self { inner }
    }

    /// Get the internal state of the M0 code of the LPC43xx MCU.
    ///
    /// Requires API 0x0106.
    pub async fn get_m0_state(&self) -> Result<M0State, Error> {
        self.inner.api_check(0x0106)?;
        let mut v: M0State = self.inner.read_struct(ControlRequest::GetM0State).await?;
        v.le_convert();
        Ok(v)
    }

    /// Access the attached SPI flash.
    pub fn spi_flash(&self) -> SpiFlash<'_> {
        SpiFlash { inner: self.inner }
    }

    /// Update the XC2C64A-7VQ100C CPLD with a new bitstream.
    ///
    /// After every transfer completes, an optional callback will be invoked
    /// with the number of bytes transferred as the first argument, and the
    /// total number of bytes to be transferred as the second argument.
    pub async fn cpld_write<F>(&mut self, data: &[u8], mut callback: Option<F>) -> Result<(), Error>
    where
        F: FnMut(usize, usize),
    {
        const CHUNK_SIZE: usize = 512;
        self.inner
            .set_transceiver_mode(TransceiverMode::CpldUpdate)
            .await?;
        let mut queue = self
            .inner
            .interface
            .bulk_out_queue(crate::consts::TX_ENDPOINT_ADDRESS);
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
        self.inner
            .set_transceiver_mode(TransceiverMode::Off)
            .await?;
        Ok(())
    }

    /// Get the checksum of the CPLD bitstream.
    pub async fn cpld_checksum(&self) -> Result<u32, Error> {
        self.inner.api_check(0x0103)?;
        let ret = self
            .inner
            .read_bytes(ControlRequest::CpldChecksum, 4)
            .await?;
        let ret: [u8; 4] = ret.as_slice().try_into().map_err(|_| Error::ReturnData)?;
        Ok(u32::from_le_bytes(ret))
    }

    /// Read a register from the SI5351C.
    pub async fn si5351c_read(&self, register: u8) -> Result<u8, Error> {
        self.inner
            .read_u8(ControlRequest::Si5351cRead, register as u16)
            .await
    }

    /// Write a register to the SI5351C.
    pub async fn si5351c_write(&self, register: u8, value: u8) -> Result<(), Error> {
        self.inner
            .write_u8(ControlRequest::Si5351cWrite, register as u16, value)
            .await
    }

    /// Read a register from the RFFC5071.
    pub async fn rffc5071_read(&self, register: u8) -> Result<u16, Error> {
        if register >= 31 {
            return Err(Error::AddressRange {
                range: Range { start: 0, end: 31 },
                addr: register as u32,
            });
        }

        self.inner
            .read_u16(ControlRequest::Rffc5071Read, register as u16)
            .await
    }

    /// Write a register to the RFFC5071.
    pub async fn rffc5071_write(&self, register: u8, value: u16) -> Result<(), Error> {
        if register >= 31 {
            return Err(Error::AddressRange {
                range: Range { start: 0, end: 31 },
                addr: register as u32,
            });
        }

        self.inner
            .write_u16(ControlRequest::Rffc5071Write, register as u16, value)
            .await
    }

    /// Read a register from the MAX2837.
    pub async fn max2837_read(&self, register: u8) -> Result<u16, Error> {
        if register >= 32 {
            return Err(Error::AddressRange {
                range: Range { start: 0, end: 32 },
                addr: register as u32,
            });
        }

        self.inner
            .read_u16(ControlRequest::Max2837Read, register as u16)
            .await
    }

    /// Write a register to the MAX2837.
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

        self.inner
            .write_u16(ControlRequest::Max2837Write, register as u16, value)
            .await
    }
}

/// Accessor for the W25Q80BV SPI Flash in the HackRF.
pub struct SpiFlash<'a> {
    inner: &'a HackRf,
}

impl SpiFlash<'_> {
    /// Erase the entire flash memory.
    ///
    /// Should be immediately followed by writing a new image, or the HackRF
    /// will be soft-bricked (but recoverable by DFU).
    pub async fn erase(&self) -> Result<(), Error> {
        self.inner
            .write_u8(ControlRequest::SpiflashErase, 0, 0)
            .await
    }

    /// Write firmware to the flash memory.
    ///
    /// Should only be used for firmware image writing, and likely needs to be
    /// preceeded by an erase command before doing a write sequence.
    ///
    /// Writes can be up to the max size of the memory; this command will split
    /// them into sub-commands if needed.
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

        let mut addr = addr;
        let mut data = data;
        let mut chunk: &[u8];
        while !data.is_empty() {
            // Split so that all writes are within a 256-byte page.
            let len = (0x100 - ((addr & 0xff) as usize)).min(data.len());
            (chunk, data) = data.split_at(len);
            self.inner
                .interface
                .control_out(ControlOut {
                    control_type: ControlType::Vendor,
                    recipient: Recipient::Device,
                    request: ControlRequest::SpiflashWrite as u8,
                    value: (addr >> 16) as u16,
                    index: (addr & 0xFFFF) as u16,
                    data: chunk,
                })
                .await
                .into_result()?;
            addr += len as u32;
        }
        Ok(())
    }

    /// Read from the flash memory.
    ///
    /// This should only be used for firmware verification.
    ///
    /// Reads can be up to the max size of the memory; this command will split
    /// them into sub-commands if needed.
    pub async fn read(&self, addr: u32, len: usize) -> Result<Vec<u8>, Error> {
        const END_ADDR: u32 = 0x10_0000;
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

        let mut addr = addr;
        let mut data = Vec::with_capacity(len);
        while data.len() < len {
            // Read from one 256-byte page at a time, dividing up as needed.
            let block_len = (0x100 - ((addr & 0xff) as usize)).min(len - data.len());
            let resp = self
                .inner
                .interface
                .control_in(ControlIn {
                    control_type: ControlType::Vendor,
                    recipient: Recipient::Device,
                    request: ControlRequest::SpiflashRead as u8,
                    value: (addr >> 16) as u16,
                    index: (addr & 0xFFFF) as u16,
                    length: block_len as u16,
                })
                .await
                .into_result()?;
            data.extend_from_slice(&resp);
            addr += resp.len() as u32;
        }
        Ok(data)
    }

    /// Get the status registers of the W25Q80BV flash memory.
    pub async fn status(&self) -> Result<[u8; 2], Error> {
        self.inner.api_check(0x0103)?;
        let val = self
            .inner
            .read_u16(ControlRequest::SpiflashStatus, 0)
            .await?;
        Ok(val.to_le_bytes())
    }

    /// Clear the status registers of the W25Q80BV flash memory.
    pub async fn clear_status(&self) -> Result<(), Error> {
        self.inner.api_check(0x0103)?;
        self.inner
            .write_u16(ControlRequest::SpiflashClearStatus, 0, 0)
            .await
    }
}
