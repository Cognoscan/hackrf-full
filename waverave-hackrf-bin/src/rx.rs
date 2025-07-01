use std::sync::{Arc, atomic};

use clap::Args;
use color_eyre::eyre::Context;
use tokio::io::AsyncWriteExt;
use waverave_hackrf::{Buffer, HackRf};

#[derive(Args, Debug)]
pub struct Cmd {
    /// Size of transfer buffers. Defaults to 4096.
    #[arg(short, long, default_value_t = 64)]
    buf_size: usize,
    /// Number of in-flight sweep buffers allowed. Defaults to 64.
    #[arg(short, long, default_value_t = 64)]
    queue_depth: usize,
}

impl Cmd {
    pub async fn cmd(&self, rf: HackRf) -> color_eyre::Result<()> {
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
