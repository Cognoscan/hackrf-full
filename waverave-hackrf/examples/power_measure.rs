use anyhow::Result;
#[tokio::main]
async fn main() -> Result<()> {
    let hackrf = waverave_hackrf::open_hackrf()?;

    // Configure: 20MHz sample rate, turn on the RF amp, set IF & BB gains to 16 dB,
    // and tune to 915 MHz.
    hackrf.set_sample_rate(20e6).await?;
    hackrf.set_amp_enable(true).await?;
    hackrf.set_lna_gain(16).await?;
    hackrf.set_vga_gain(16).await?;
    hackrf.set_freq(915_000_000).await?;

    // Start receiving, in bursts of 16384 samples
    let mut hackrf_rx = hackrf.start_rx(16384).await?;

    // Queue up 64 transfers, retrieve them, and measure average power.
    for _ in 0..64 {
        hackrf_rx.submit();
    }
    let mut count = 0;
    let mut pow_sum = 0.0;
    while hackrf_rx.pending() > 0 {
        let buf = hackrf_rx.next_complete().await?;
        for x in buf.samples() {
            let re = x.re as f64;
            let im = x.im as f64;
            pow_sum += re * re + im * im;
        }
        count += buf.len();
    }

    // Stop receiving
    hackrf_rx.stop().await?;

    // Print out our measurement
    let average_power = (pow_sum / (count as f64 * 127.0 * 127.0)).log10() * 10.;
    println!("Average Power = {average_power} dbFS");
    Ok(())
}
