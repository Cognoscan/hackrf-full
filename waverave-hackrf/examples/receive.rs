use std::sync::{Arc, atomic};

use anyhow::Result;
use tokio::io::AsyncWriteExt;
use waverave_hackrf::Buffer;
#[tokio::main]
async fn main() -> Result<()> {
    // Set up the ctrl-c handler
    let ctrlc_rx = Arc::new(atomic::AtomicBool::new(false));
    let ctrlc_tx = ctrlc_rx.clone();
    tokio::spawn(async move {
        tokio::signal::ctrl_c().await.unwrap();
        ctrlc_tx.store(true, atomic::Ordering::Release);
    });

    // Open up a file for buffered writing.
    let mut args = std::env::args();
    args.next();
    let file_name = args.next().unwrap_or_else(|| String::from("./rx.bin"));
    let mut file = tokio::fs::File::create(&file_name).await?;

    // Open up the HackRF
    let hackrf = waverave_hackrf::open_hackrf()?;

    // Configure: 20MHz sample rate, turn on the RF amp, set IF & BB gains to 16 dB,
    // and tune to 915 MHz.
    hackrf.set_amp_enable(true).await?;
    hackrf.set_sample_rate(20e6).await?;
    hackrf.set_lna_gain(16).await?;
    hackrf.set_vga_gain(16).await?;
    hackrf.set_freq(915_000_000).await?;

    // Start receiving, in bursts of 8192 samples
    let mut hackrf_rx = hackrf.start_rx(8192).await.map_err(|e| e.err)?;

    // Separate the file writer from the sample reader with a separate task
    let (data_send, mut data_recv) = tokio::sync::mpsc::unbounded_channel::<Buffer>();
    let file_writer = tokio::spawn(async move {
        loop {
            let Some(buf) = data_recv.recv().await else {
                break;
            };
            file.write_all_buf(&mut buf.bytes()).await?;
        }
        file.flush().await?;
        Ok::<(), anyhow::Error>(())
    });

    // Queue up 64 transfers immediately, then start retrieving them until we
    // get a ctrl-c.
    for _ in 0..64 {
        hackrf_rx.submit();
    }
    loop {
        if ctrlc_rx.load(atomic::Ordering::Acquire) {
            break;
        }
        if data_send.send(hackrf_rx.next_complete().await?).is_err() {
            break;
        }
        hackrf_rx.submit();
    }

    // Stop receiving
    hackrf_rx.stop().await?;
    drop(data_send);

    // Wait for file writer task to close up shop
    file_writer.await??;

    Ok(())
}
