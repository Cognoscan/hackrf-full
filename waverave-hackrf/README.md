# `waverave-hackrf`

This is a complete, strongly-asynchronous host crate for the [HackRF][hackrf],
made using the pure-rust [`nusb`] crate for USB interfacing. It reproduces *all* 
the functionality of the original [`libhackrf`][libhackrf] library.

[hackrf]: https://greatscottgadgets.com/hackrf/one/
[libhackrf]: https://github.com/greatscottgadgets/hackrf/tree/master/host

The standard entry point for this library is `open_hackrf()`, which will open
the first available HackRF device.

Getting started is easy: open up a HackRF peripheral, configure it as needed,
and enter into transmit, receive, or RX sweep mode. Changing the oeprating mode
also changes the struct used, i.e. it uses the typestate pattern. The different
states and their corresponding structs are:

- `HackRf` - The default, off, state.
- `Receive` - Receiving RF signals.
- `Transmit` - Transmitting RF signals.
- `Sweep` - Running a receive sweep through multiple tuning frequencies.

If a mode change error occurs, the `HackRf` struct is returned alongside the
error, and it can potentially be reset back to the off state by running
`HackRf::turn_off`.

As for what using this library looks like in practice, here's an example program
that configures the system, enters receive mode, and processes samples to
estimate the average received power relative to full scale:

```rust
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
    let mut hackrf_rx = hackrf.start_rx(16384).await.map_err(|e| e.err)?;

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

```
