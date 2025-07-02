use clap::Args;
use color_eyre::eyre::Context;
use waverave_hackrf::{HackRf, HackRfType, info::*, list_hackrf_devices};

/// Retrieve as much info as possible from each attached HackRF.
#[derive(Args, Debug)]
pub struct Cmd {}

#[derive(Clone, Debug)]
struct AllInfo {
    api: u16,
    radio_type: HackRfType,
    id: BoardId,
    rev: BoardRev,
    serial: SerialNumber,
    version_string: String,
    platform: SupportedPlatform,
    operacakes: Vec<u8>,
}

impl std::fmt::Display for AllInfo {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(
            f,
            "Serial number: {:08x}{:08x}{:08x}{:08x}",
            self.serial.serial_no[0],
            self.serial.serial_no[1],
            self.serial.serial_no[2],
            self.serial.serial_no[3],
        )?;
        writeln!(f, "Board ID: {}", self.id)?;
        writeln!(
            f,
            "Firmware Version: {} (API:{}{}.{}{})",
            self.version_string,
            (self.api >> 12) & 0xF,
            (self.api >> 8) & 0xF,
            (self.api >> 4) & 0xF,
            self.api & 0xF
        )?;
        writeln!(
            f,
            "Part ID Number: 0x{:08x} 0x{:08x}",
            self.serial.part_id[0], self.serial.part_id[1],
        )?;
        writeln!(f, "Hardware revision: {}", self.rev)?;
        if self.rev.is_official() {
            writeln!(
                f,
                "Hardware appears to have been manufactured by Great Scott Gadgets"
            )?;
        }
        writeln!(f, "Hardware supported by installed firmware:")?;
        if self.platform.jawbreaker {
            writeln!(f, "    Jawbreaker")?;
        }
        if self.platform.rad1o {
            writeln!(f, "    rad1o")?;
        }
        if self.platform.hackrf1_og {
            writeln!(f, "    HackRF One")?;
        }
        if self.platform.hackrf1_r9 {
            writeln!(f, "    HackRF One R9")?;
        }
        for addr in self.operacakes.iter() {
            writeln!(f, "Opera Cake found, address: {addr}")?;
        }
        Ok(())
    }
}

impl AllInfo {
    async fn load(rf: &HackRf) -> color_eyre::Result<Self> {
        let operacakes = rf.operacake_boards().await?;
        let info = rf.info();

        Ok(AllInfo {
            api: info.api_version(),
            radio_type: info.radio_type(),
            id: info.board_id().await.wrap_err("Failed getting board ID")?,
            rev: info
                .board_rev()
                .await
                .wrap_err("Failed getting board Rev")?,
            serial: info
                .serial()
                .await
                .wrap_err("Failed getting serial number")?,
            version_string: info
                .version_string()
                .await
                .wrap_err("Failed getting version string")?,
            platform: info
                .supported_platform()
                .await
                .wrap_err("Failed getting supported platform")?,
            operacakes,
        })
    }
}

impl Cmd {
    pub async fn cmd(&self, filter_serial: Option<String>) -> color_eyre::Result<()> {
        println!("Binary release {}", env!("CARGO_PKG_VERSION"));

        let mut found_hackrf = false;
        for dev in list_hackrf_devices().wrap_err("Couldn't enumerate HackRF devices")? {
            let serial = dev
                .serial()
                .map(|s| s.to_owned())
                .unwrap_or_else(|| String::from("Unknown"));

            if let Some(s) = filter_serial.as_ref() {
                if s != &serial {
                    continue;
                }
            }
            found_hackrf = true;

            let rf = match dev.open() {
                Ok(rf) => rf,
                Err(e) => {
                    println!("Couldn't open HackRF, serial {serial}: {e}");
                    continue;
                }
            };

            println!();
            println!("Found HackRF, USB serial {serial}");

            let info = rf.info();

            let api = info.api_version();
            let id = info.board_id().await.ok();
            let rev = info.board_rev().await.ok();
            let serial = info.serial().await.ok();
            let version_string = info.version_string().await.ok();
            let platform = info.supported_platform().await.ok();
            if let Some(serial) = serial.as_ref() {
                println!(
                    "Serial number: {:08x}{:08x}{:08x}{:08x}",
                    serial.serial_no[0],
                    serial.serial_no[1],
                    serial.serial_no[2],
                    serial.serial_no[3],
                );
            } else {
                println!("Serial number: ❌ Failed to retrieve");
            }

            if let Some(id) = id {
                println!("Board ID: {id}");
            } else {
                println!("Board ID: ❌ Failed to retrieve");
            }
            let version_string =
                version_string.unwrap_or_else(|| String::from("❌ Failed to retrieve"));
            println!(
                "Firmware Version: {} (API:{}{}.{}{})",
                version_string,
                (api >> 12) & 0xF,
                (api >> 8) & 0xF,
                (api >> 4) & 0xF,
                api & 0xF
            );
            if let Some(serial) = serial {
                println!(
                    "Part ID Number: 0x{:08x} 0x{:08x}",
                    serial.part_id[0], serial.part_id[1],
                );
            } else {
                println!("Part ID Number: ❌ Failed to retrieve");
            }
            if let Some(rev) = rev {
                println!("Hardware revision: {rev}");
                if rev.is_official() {
                    println!("Hardware appears to have been manufactured by Great Scott Gadgets");
                }
            } else {
                println!("Hardware revision: ❌ Failed to retrieve");
            }
            if let Some(platform) = platform {
                println!("Hardware supported by installed firmware:");
                if platform.jawbreaker {
                    println!("    Jawbreaker");
                }
                if platform.rad1o {
                    println!("    rad1o");
                }
                if platform.hackrf1_og {
                    println!("    HackRF One");
                }
                if platform.hackrf1_r9 {
                    println!("    HackRF One R9");
                }
            } else {
                println!("Hardware supported: ❌ Failed to retrieve");
            }
            if let Ok(operacakes) = rf.operacake_boards().await {
                for addr in operacakes.iter() {
                    println!("Opera Cake found, address: {addr}");
                }
            } else {
                println!("Opera Cakes: ❌ Failed to retrieve");
            }
        }

        if !found_hackrf {
            if let Some(serial) = filter_serial {
                println!("Unable to locate HackRF with serial number {serial}");
            } else {
                println!("Couldn't find any HackRF modules");
            }
        }
        Ok(())
    }
}
