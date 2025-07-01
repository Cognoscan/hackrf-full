#![allow(dead_code)]

use std::{
    collections::BTreeMap,
    str::FromStr,
    sync::{Arc, atomic},
};

use clap::{Args, Parser, Subcommand};
use color_eyre::eyre::Context;
use tokio::io::AsyncWriteExt;
use waverave_hackrf::{Buffer, HackRf, SweepBuf, SweepParams};

#[derive(Clone, Debug)]
struct AllInfo {
    api: u16,
    radio_type: waverave_hackrf::HackRfType,
    id: waverave_hackrf::info::BoardId,
    rev: waverave_hackrf::info::BoardRev,
    serial: waverave_hackrf::info::SerialNumber,
    version_string: String,
    platform: waverave_hackrf::info::SupportedPlatform,
}

#[derive(Parser, Debug)]
#[command(version, about)]
struct Cli {
    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand, Debug)]
enum Commands {
    Info(InfoCmd),
    Sweep(SweepCmd),
    Rx(RxCmd),
}

#[derive(Args, Debug)]
struct InfoCmd {}

impl InfoCmd {
    async fn cmd(&self, rf: HackRf) -> color_eyre::Result<()> {
        let info = rf.info();
        let api = info.api_version();
        let radio_type = info.radio_type();
        let id = info.board_id().await?;
        let serial = info.serial().await?;
        let rev = info.board_rev().await?;
        let version_string = info.version_string().await?;
        let platform = info.supported_platform().await?;

        let info = AllInfo {
            api,
            radio_type,
            id,
            rev,
            serial,
            version_string,
            platform,
        };
        println!("Info: {:#x?}", info);
        Ok(())
    }
}

#[derive(Clone, Copy, Debug)]
struct FreqPair {
    lo: u16,
    hi: u16,
}

impl FromStr for FreqPair {
    type Err = &'static str;
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let Some((lo, hi)) = s.split_once(":") else {
            return Err("Missing colon");
        };
        let lo: u16 = u16::from_str(lo).map_err(|_| "Low frequency isn't valid")?;
        let hi: u16 = u16::from_str(hi).map_err(|_| "High frequency isn't valid")?;
        if !(1..=6000).contains(&lo) {
            return Err("Low frequency out of range (1-6000)");
        }
        if !(1..=6000).contains(&hi) {
            return Err("High frequency out of range (1-6000)");
        }
        if hi <= lo {
            return Err("High frequency is less than low frequency");
        }
        Ok(Self { lo, hi })
    }
}

#[derive(Args, Debug)]
struct SweepCmd {
    /// Number of in-flight sweep buffers allowed. Defaults to 64.
    #[arg(short, long, default_value_t = 64)]
    queue_depth: usize,
    /// Minimum & maximum frequency, in MHz, as a colon-separated pair (ex.
    /// 90:110 for 90-110 MHz)
    #[arg(short, long)]
    freq: Vec<FreqPair>,
    /// Sample rate in Hz
    #[arg(short, long, default_value_t = 20000000)]
    sample_rate: u32,
}

impl SweepCmd {
    async fn cmd(&self, rf: HackRf) -> color_eyre::Result<()> {
        let ctrlc_rx = Arc::new(atomic::AtomicBool::new(false));
        let ctrlc_tx = ctrlc_rx.clone();
        tokio::spawn(async move {
            tokio::signal::ctrl_c().await.unwrap();
            ctrlc_tx.store(true, atomic::Ordering::Release);
        });

        let mut params = SweepParams::init_sample_rate(self.sample_rate);
        for f in self.freq.iter() {
            params.freq_mhz.push((f.lo, f.hi));
        }
        let mut sweep = rf.start_rx_sweep(&params).await.map_err(|e| e.err)?;

        let mut bufs: u64 = 0;
        let mut freq: BTreeMap<u64, u64> = BTreeMap::new();
        loop {
            if ctrlc_rx.load(atomic::Ordering::Acquire) {
                break;
            }
            if sweep.pending() > 0 {
                let buf = sweep
                    .next_complete()
                    .await
                    .wrap_err("Failed trying to get next sweep")?;
                let f = freq.entry(buf.freq_hz()).or_default();
                *f += 1;
                bufs += 1;
                if bufs % 256 == 0 {
                    println!("Received bufs: {}", bufs);
                }
            }
            while sweep.pending() < self.queue_depth {
                sweep.submit();
            }
        }

        let mut rf = sweep.stop().await?;

        println!("Frequencies:");
        for (f, cnt) in freq.iter() {
            println!("{:>7} kHz: {}", (*f) / 1000, cnt);
        }
        println!();
        let m0_state_after = rf.debug().get_m0_state().await?;
        println!("M0 state after = {:#x?}", m0_state_after);

        Ok(())
    }
}

#[derive(Args, Debug)]
struct RxCmd {
    /// Size of transfer buffers. Defaults to 4096.
    #[arg(short, long, default_value_t = 64)]
    buf_size: usize,
    /// Number of in-flight sweep buffers allowed. Defaults to 64.
    #[arg(short, long, default_value_t = 64)]
    queue_depth: usize,
}

impl RxCmd {
    async fn cmd(&self, rf: HackRf) -> color_eyre::Result<()> {
        let ctrlc_rx = Arc::new(atomic::AtomicBool::new(false));
        let ctrlc_tx = ctrlc_rx.clone();
        tokio::spawn(async move {
            tokio::signal::ctrl_c().await.unwrap();
            ctrlc_tx.store(true, atomic::Ordering::Release);
        });

        let mut rx = rf
            .start_rx(self.buf_size)
            .await
            .map_err(|e| e.err)
            .wrap_err("Failed to start RX")?;

        let mut total: u64 = 0;
        let mut errors: u64 = 0;
        let mut shortfalls: u64 = 0;

        let dest = tokio::fs::File::create("./test.bin").await?;
        let mut dest = tokio::io::BufWriter::new(dest);
        let (file_tx, mut file_rx) = tokio::sync::mpsc::unbounded_channel::<Buffer>();

        tokio::spawn(async move {
            loop {
                let Some(buf) = file_rx.recv().await else {
                    break;
                };
                //let Ok(_) = dest.write(&buf).await else { break };
            }
            let _ = dest.flush().await;
        });

        let start = std::time::Instant::now();
        let mut recv_cnt = 0;
        let mut buffers: u64 = 0;
        loop {
            if rx.pending() > 0 {
                let buf = rx
                    .next_complete()
                    .await
                    .wrap_err("Failed to get RX completion");
                let buf = match buf {
                    Ok(b) => b,
                    Err(e) => {
                        println!("{}", e);
                        errors += 1;
                        continue;
                    }
                };
                if buf.len() < self.buf_size {
                    shortfalls += 1;
                }
                let t = buf.len() as u64;
                total += t;
                recv_cnt += t;
                buffers += 1;
                if recv_cnt >= 10_000_000 {
                    println!("Received {}, total {}", recv_cnt, total);
                    recv_cnt = 0;
                }
                if ctrlc_rx.load(atomic::Ordering::Acquire) {
                    break;
                }
                if file_tx.send(buf).is_err() {
                    println!("File writer closed, stopping");
                    break;
                }
            }
            while rx.pending() < self.queue_depth {
                rx.submit();
            }
        }
        let mut rf = rx.stop().await?;
        let duration = start.elapsed().as_secs_f64();
        let rate = (total as f64) / duration / 1e6;

        let m0_state_after = rf.debug().get_m0_state().await?;

        println!("M0 state after = {:#x?}", m0_state_after);
        println!("Total received: {}", total);
        println!("Total errors: {}", errors);
        println!("Total shortfalls: {}", shortfalls);
        println!("Total buffers: {}", buffers);
        println!("MBps: {}", rate);
        Ok(())
    }
}

#[tokio::main]
async fn main() -> color_eyre::Result<()> {
    color_eyre::install()?;
    let args = Cli::parse();
    let rf =
        waverave_hackrf::open_hackrf().wrap_err_with(|| String::from("Failed to open HackRF"))?;
    match args.command {
        Commands::Info(c) => c.cmd(rf).await,
        Commands::Sweep(c) => c.cmd(rf).await,
        Commands::Rx(c) => c.cmd(rf).await,
    }
}
