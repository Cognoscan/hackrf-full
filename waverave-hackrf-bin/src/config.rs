use clap::Args;
use color_eyre::eyre::{Context, eyre};
use waverave_hackrf::{HackRf, RfPathFilter};

#[derive(Args, Debug)]
pub struct Cmd {
    /// Frequency in Hz. Preferred range is 1MHz-6000MHz, valid range is 0MHz to
    /// 7250MHz.
    #[arg(short, long)]
    freq_hz: Option<u64>,

    /// Intermediate frequency (IF) in Hz. Preferred range is 2170-2740 MHz,
    /// 2000-3000 MHz is valid.
    #[arg(short, long = "if_freq")]
    if_freq_hz: Option<u64>,

    /// Front-end Local Oscillator (LO) frequency in Hz. Valid range is 84-540
    /// MHz.
    #[arg(short = 'o', long = "lo_freq")]
    lo_freq_hz: Option<u64>,

    /// Image Rejection filter selection. 0=bypass, 1=low-pass, 2=high-pass.
    #[arg(short = 'm', long)]
    image_reject: Option<u8>,

    /// RX/TX RF Amplifier
    #[arg(short, long = "amp_en")]
    amp_enable: Option<bool>,

    /// Antenna port power
    #[arg(short = 'p', long = "ant_en")]
    antenna_enable: Option<bool>,

    /// RX LNA (IF) gain, 0-40dB, 8dB steps
    #[arg(short = 'l', long = "lna")]
    lna_gain: Option<u16>,

    /// RX VGA (baseband) gain, 0-62dB, 2dB steps
    #[arg(short = 'g', long = "vga")]
    vga_gain: Option<u16>,

    /// TX VGA (IF) gain, 0-47, 1dB steps
    #[arg(short = 'x', long = "txvga")]
    txvga_gain: Option<u16>,

    /// Sample rate in Hz (2-20 MHz)
    #[arg(short, long = "rate", default_value_t = 10e6)]
    sample_rate_hz: f64,

    /// Set baseband filter bandwidth in Hz. Chooses nearest available value.
    /// Default value is `0.75 * sample_rate_hz`. Possible actual values: 1.75,
    /// 2.5, 3.5, 5, 5.5, 6, 7, 8, 9, 10, 12, 14, 15, 20, 24, 28 MHz.
    #[arg(short, long = "baseband")]
    baseband_filter_bw_hz: Option<u32>,

    /// Set internal crystal clock error in ppm.
    #[arg(short = 'C', long)]
    ppm: Option<i32>,

    /// Synchronize to external trigger input
    #[arg(short = 'H', long = "ext_trig")]
    sync: Option<bool>,
}

impl Cmd {
    pub async fn configure(&self, rf: &HackRf) -> color_eyre::Result<()> {
        if let Some(freq_hz) = self.freq_hz {
            let freq_hz = self
                .ppm
                .map(|ppm| freq_hz * (1_000_000 - (ppm as u64)) / 1_000_000)
                .unwrap_or(freq_hz);

            rf.set_freq(freq_hz)
                .await
                .wrap_err("Failed setting frequency")?;
        }

        // Explicit tuning - we don't seem to adjust the frequencies based on
        // the PPM correction. Should we?
        match (self.if_freq_hz, self.lo_freq_hz, self.image_reject) {
            (Some(if_freq_hz), Some(lo_freq_hz), Some(image_reject)) => {
                if self.freq_hz.is_some() {
                    return Err(eyre!(
                        "freq_hz cannot be set while the explicit frequency tuning values are also set"
                    ));
                }
                let filter = match image_reject {
                    0 => RfPathFilter::Bypass,
                    1 => RfPathFilter::LowPass,
                    2 => RfPathFilter::HighPass,
                    i => return Err(eyre!("image_reject valid range is 0-2, but got {i}")),
                };
                rf.set_freq_explicit(if_freq_hz, lo_freq_hz, filter)
                    .await
                    .wrap_err("Failed setting explicit frequency tuning")?;
            }
            (None, None, None) => (),
            _ => {
                return Err(eyre!(
                    "If `if_freq_hz`, `lo_freq_hz`, or `image_reject` are set, they all must be set"
                ));
            }
        }

        if let Some(amp_enable) = self.amp_enable {
            rf.set_amp_enable(amp_enable)
                .await
                .wrap_err("Failed setting amplifier enable on/off")?;
        }

        if let Some(gain) = self.lna_gain {
            rf.set_lna_gain(gain)
                .await
                .wrap_err("Failed setting LNA gain")?;
        }

        if let Some(gain) = self.vga_gain {
            rf.set_vga_gain(gain)
                .await
                .wrap_err("Failed setting VGA gain")?;
        }

        if let Some(gain) = self.txvga_gain {
            rf.set_txvga_gain(gain)
                .await
                .wrap_err("Failed setting TXVGA gain")?;
        }

        let sample_rate_hz = self
            .ppm
            .map(|ppm| self.sample_rate_hz * (1e6 - (ppm as f64)) / 1e6)
            .unwrap_or(self.sample_rate_hz);
        rf.set_sample_rate(sample_rate_hz)
            .await
            .wrap_err("Failed setting sample rate")?;

        // Correct for the baseband filter - the sample rate setting would've
        // used a smaller baseband filter than is actually required.
        if self.ppm.is_some() && self.baseband_filter_bw_hz.is_none() {
            let bb = (self.sample_rate_hz * 3.0 / 4.0).round() as u32;
            rf.set_baseband_filter_bandwidth(bb)
                .await
                .wrap_err("Failed setting baseband filter bandwidth")?;
        }

        if let Some(bb) = self.baseband_filter_bw_hz {
            rf.set_baseband_filter_bandwidth(bb)
                .await
                .wrap_err("Failed setting baseband filter bandwidth")?;
        }

        if let Some(sync) = self.sync {
            rf.set_hw_sync_mode(sync)
                .await
                .wrap_err("Failed setting HW sync mode")?;
        }

        Ok(())
    }
}
