use std::{
    collections::BTreeMap,
    str::FromStr,
    sync::{Arc, atomic},
};

use clap::Args;
use color_eyre::eyre::Context;
use waverave_hackrf::{HackRf, SweepParams};

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
pub struct Cmd {
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

impl Cmd {
    pub async fn cmd(&self, rf: HackRf) -> color_eyre::Result<()> {
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
                    println!("Received bufs: {bufs}");
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
        println!("M0 state after = {m0_state_after:#x?}");

        Ok(())
    }
}
