use anyhow::Result;
use tokio::io::AsyncReadExt;
use waverave_hackrf::Buffer;
#[tokio::main]
async fn main() -> Result<()> {
    let hackrf = waverave_hackrf::open_hackrf()?;

    // Configure: 20MHz sample rate, turn on RF amp, set TX IF gain to 16 dB,
    // and tune to 915 MHz.
    hackrf.set_sample_rate(20e6).await?;
    hackrf.set_txvga_gain(16).await?;
    hackrf.set_freq(915_000_000).await?;
    hackrf.set_amp_enable(true).await?;

    // Open up a file for buffered reading.
    let mut args = std::env::args();
    args.next();
    let file_name = args.next().unwrap_or_else(|| String::from("./tx.bin"));
    let mut file = tokio::fs::File::open(&file_name).await?;

    // Start transmitting, in bursts of 8192 samples
    let mut hackrf_tx = hackrf.start_tx(8192).await.map_err(|e| e.err)?;

    // Set up an asynchronous process that fills buffers and sends them on to
    // the transmitter.
    let (buf_send, mut buf_recv) = tokio::sync::mpsc::channel::<Buffer>(4);
    let (data_send, mut data_recv) = tokio::sync::mpsc::channel::<Buffer>(4);
    tokio::spawn(async move {
        loop {
            let Some(mut buf) = buf_recv.recv().await else {
                break;
            };
            buf.extend_zeros(buf.remaining_capacity());
            file.read_exact(buf.bytes_mut()).await?;
            let Ok(_) = data_send.send(buf).await else {
                break;
            };
        }
        Ok::<(), anyhow::Error>(())
    });

    // Start filling up queue
    let mut start = Vec::with_capacity(64);
    while start.len() < 64 {
        while buf_send.try_send(hackrf_tx.get_buffer()).is_ok() {}
        let Some(buf) = data_recv.recv().await else {
            break;
        };
        start.push(buf);
    }

    // Submit the whole starting queue in one go
    for buf in start {
        hackrf_tx.submit(buf);
    }

    // Continue filling the queue and submitting samples as often as possible
    loop {
        while buf_send.try_send(hackrf_tx.get_buffer()).is_ok() {}
        hackrf_tx.next_complete().await?;
        let Some(buf) = data_recv.recv().await else {
            break;
        };
        hackrf_tx.submit(buf);
    }

    // Flush the remainder
    hackrf_tx.flush();
    while hackrf_tx.pending() > 0 {
        hackrf_tx.next_complete().await?;
    }

    // Stop transmitting
    hackrf_tx.stop().await?;

    Ok(())
}
