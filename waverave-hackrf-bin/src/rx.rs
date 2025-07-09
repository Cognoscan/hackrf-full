use clap::Args;
use color_eyre::{Section, eyre::Context};
use tokio::{io::AsyncWriteExt, sync::mpsc};
use tracing::{error, warn};
use waverave_hackrf::{Buffer, HackRf};

#[derive(Args, Debug)]
pub struct Cmd {
    #[command(flatten)]
    params: crate::config::Cmd,
    /// Size of transfer buffers, in samples. Defaults to 8192, clamped between
    /// 256 and 2^17.
    #[arg(short = 'S', long, default_value_t = 8192)]
    buf_size: usize,

    /// Number of samples to transfer (default is unlimited).
    #[arg(short, long = "samples")]
    num_samples: Option<u64>,

    /// Record data to a file. Omit for stdout.
    #[arg(default_value_t)]
    filename: String,
}

impl Cmd {
    pub async fn cmd(&self, rf: HackRf) -> color_eyre::Result<()> {
        // Configure
        self.params
            .configure(&rf)
            .await
            .wrap_err("Failed configuring the HackRF")?;

        // Set up a task manager
        let mut tracker = tokio::task::JoinSet::new();

        // Run the receiver in a separate task
        let (buf_tx, buf_rx) = mpsc::channel::<Buffer>(20);
        let buf_params = BufParams::new(self.buf_size);
        tracker.spawn(run_rf(rf, buf_tx, buf_params));

        // Run the file writer in a separate task
        tracker.spawn(write_file(buf_rx, self.filename.clone(), self.num_samples));

        let quit = crate::os_signal::quit_watch().await;
        quit.quit().await;
        let results =
            tokio::time::timeout(tokio::time::Duration::from_secs(1), tracker.join_all()).await?;

        // Check for errors and compile them together if needed
        let err_count = results.iter().fold(0, |cnt, res| res.is_err() as u32 + cnt);
        if err_count == 0 {
            return Ok(());
        }
        if err_count == 1 {
            for r in results {
                r?;
            }
            return Ok(());
        }

        let err = results
            .into_iter()
            .filter(Result::is_err)
            .map(Result::unwrap_err)
            .fold(
                color_eyre::eyre::eyre!("encountered multiple errors"),
                |report, e| report.section(e),
            );
        Err(err)
    }
}

#[derive(Clone, Copy, Debug)]
struct BufParams {
    size: usize,
    depth: usize,
}

impl BufParams {
    fn new(buf_size: usize) -> Self {
        // Limit buffer size and queue depth to reasonable values. This should
        // give us 2 MiB of data, or about 50 ms of buffering.
        let size = buf_size.clamp(256, 1 << 17);
        let depth = ((1 << 20) / size).max(4);
        Self { size, depth }
    }
}

async fn run_rf(
    rf: HackRf,
    buf_tx: mpsc::Sender<Buffer>,
    buf_params: BufParams,
) -> color_eyre::Result<()> {
    // Set up a signal catcher
    let quit = crate::os_signal::quit_watch().await;

    // Start the receive operation
    let mut rf = match rf.start_rx(buf_params.size).await {
        Ok(rf) => rf,
        Err(e) => {
            // Couldn't transition to RX, shutdown
            if e.rf.turn_off().await.is_err() {
                error!("Failed switching to RX, and failed switching it off too");
            }
            return Err(e.err.into());
        }
    };

    // Main receive loop. cancel out of the loop when we're told to, or when the
    // output stream's receiver is closed
    quit.run_until_cancelled_owned(async {
        loop {
            while rf.pending() < buf_params.depth {
                rf.submit();
            }
            let buf = match rf.next_complete().await {
                Ok(buf) => buf,
                Err(e) => {
                    warn!("RF receive error: {}", e);
                    continue;
                }
            };
            match buf_tx.try_send(buf) {
                Ok(()) => (),
                Err(mpsc::error::TrySendError::Full(_)) => {
                    error!("RF to File buffer overflow");
                }
                _ => break,
            }
        }
    })
    .await;

    // Try to shut down and gather some stats after we do
    let mut rf = match rf.stop().await {
        Ok(rf) => rf,
        Err(e) => {
            return Err(e.into());
        }
    };
    match rf.debug().get_m0_state().await {
        Ok(m0) => {
            if m0.num_shortfalls > 0 {
                warn!("Shortfalls: {}", m0.num_shortfalls);
            }
            Ok(())
        }
        Err(e) => Err(e.into()),
    }
}

async fn write_file(
    mut buf_rx: mpsc::Receiver<Buffer>,
    filename: String,
    max_samples: Option<u64>,
) -> color_eyre::Result<()> {
    // Set up a signal catcher
    let quit = crate::os_signal::quit_watch().await;

    // Open the writer
    if filename.is_empty() {
        let mut writer = tokio::io::stdout();
        let mut sample_count = 0;
        if let Some(result) = quit
            .run_until_cancelled(async {
                loop {
                    let Some(buf) = buf_rx.recv().await else {
                        break;
                    };
                    writer.write_all(buf.bytes()).await?;
                    sample_count += buf.len() as u64;
                    if let Some(max) = max_samples {
                        if sample_count >= max {
                            quit.cancel();
                            break;
                        }
                    }
                }
                Ok::<(), std::io::Error>(())
            })
            .await
        {
            result?;
        }

        writer.flush().await?;

        Ok(())
    } else {
        let writer = tokio::fs::File::create(&filename).await?;
        let mut writer = tokio::io::BufWriter::new(writer);
        let mut sample_count = 0;
        if let Some(result) = quit
            .run_until_cancelled(async {
                loop {
                    let Some(buf) = buf_rx.recv().await else {
                        break;
                    };
                    writer.write_all(buf.bytes()).await?;
                    sample_count += buf.len() as u64;
                    if let Some(max) = max_samples {
                        if sample_count >= max {
                            quit.cancel();
                            break;
                        }
                    }
                }
                Ok::<(), std::io::Error>(())
            })
            .await
        {
            result?;
        }

        writer.flush().await?;

        Ok(())
    }
}
