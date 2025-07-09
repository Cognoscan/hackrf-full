#![allow(dead_code)]

mod config;
mod info;
mod os_signal;
mod rx;
mod sweep;
mod tx;

use clap::{Parser, Subcommand};
use color_eyre::eyre::{Context, eyre};

#[derive(Parser, Debug)]
#[command(version, about)]
struct Cli {
    #[command(subcommand)]
    command: Commands,
    /// Open one specific HackRF, chosen by serial number
    #[arg(short = 'd', long)]
    serial: Option<String>,
}

#[derive(Subcommand, Debug)]
enum Commands {
    Info(info::Cmd),
    Sweep(sweep::Cmd),
    Rx(rx::Cmd),
    Config(config::Cmd),
}

#[tokio::main]
async fn main() -> color_eyre::Result<()> {
    color_eyre::install()?;
    tracing_subscriber::fmt()
        .with_writer(std::io::stderr)
        .init();
    let args = Cli::parse();

    if let Commands::Info(c) = args.command {
        return c.cmd(args.serial).await;
    }

    let rf = if let Some(serial) = args.serial.as_ref() {
        let devices =
            waverave_hackrf::list_hackrf_devices().wrap_err("Couldn't list HackRF devices")?;
        let mut chosen = None;
        for dev in devices {
            if let Some(s) = dev.serial() {
                if s == serial.as_str() {
                    chosen = Some(dev);
                    break;
                }
            }
        }
        let Some(chosen) = chosen else {
            return Err(eyre!("Couldn't locate HackRF serial {}", serial));
        };
        chosen.open().wrap_err("Couldn't open selected HackRF")?
    } else {
        waverave_hackrf::open_hackrf().wrap_err("Failed to open HackRF")?
    };

    match args.command {
        Commands::Info(_) => unreachable!("Should've executed the Info command earlier"),
        Commands::Sweep(c) => c.cmd(rf).await,
        Commands::Rx(c) => c.cmd(rf).await,
        Commands::Config(c) => c.configure(&rf).await,
    }
}
