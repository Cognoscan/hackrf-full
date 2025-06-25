#![allow(dead_code)]

use std::sync::{atomic, Arc};

use color_eyre::eyre::Context;
use tokio::io::AsyncWriteExt;

#[derive(Clone, Debug)]
struct AllInfo {
    api: u16,
    radio_type: hackrf_full::HackRfType,
    id: hackrf_full::info::BoardId,
    rev: hackrf_full::info::BoardRev,
    serial: hackrf_full::info::SerialNumber,
    version_string: String,
    platform: hackrf_full::info::SupportedPlatform,
}

#[tokio::main]
async fn main() -> color_eyre::Result<()> {
    color_eyre::install()?;
    let mut hackrf =
        hackrf_full::open_hackrf().wrap_err_with(|| String::from("Failed to open HackRF"))?;
    let info = hackrf.info();
    let api = info.api_version();
    let radio_type = info.radio_type();
    let id = info.board_id_read().await?;
    let serial = info.read_serial().await?;
    let rev = info.rev_read().await?;
    let version_string = info.version_string_read().await?;
    let platform = info.supported_platform_read().await?;
    let m0_state_before = hackrf.debug().get_m0_state().await?;

    let info = AllInfo {
        api,
        radio_type,
        id,
        rev,
        serial,
        version_string,
        platform,
    };
    
    let ctrlc_rx = Arc::new(atomic::AtomicBool::new(false));
    let ctrlc_tx = ctrlc_rx.clone();
    tokio::spawn(async move {
        tokio::signal::ctrl_c().await.unwrap();
        ctrlc_tx.store(true, atomic::Ordering::Release);
    });

    let mut rx = match hackrf.start_rx(4096).await {
        Ok(rx) => rx,
        Err((_, e)) => Err(e).wrap_err("Failed to start RX")?,
    };

    let mut total: u64 = 0;
    let mut errors: u64 = 0;

    let dest = tokio::fs::File::create("./test.bin").await?;
    let mut dest = tokio::io::BufWriter::new(dest);
    let (file_tx, mut file_rx) = tokio::sync::mpsc::unbounded_channel::<Vec<u8>>();

    let (bufloop_tx, mut bufloop_rx) =tokio::sync::mpsc::unbounded_channel::<Vec<u8>>();

    tokio::spawn(async move {
        loop {
            let Some(buf) = file_rx.recv().await else { break };
            let Ok(_) = dest.write(&buf).await else { break };
            let Ok(()) = bufloop_tx.send(buf) else { break };
        }
        let _ = dest.flush().await;
    });

    let start = std::time::Instant::now();
    let mut recv_cnt = 0;
    let mut recv_print = 0;
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
            let t = buf.len() as u64;
            total += t;
            recv_cnt += t;
            recv_print += 1;
            buffers += 1;
            if recv_print >= 100 {
                println!("Received {}, total {}", recv_cnt, total);
                recv_print = 0;
                recv_cnt = 0;
            }
            if ctrlc_rx.load(atomic::Ordering::Acquire) {
                break;
            }
            //if file_tx.send(buf).is_err() {
            //    println!("File writer closed, stopping");
            //    break;
            //}
            loop {
                if let Ok(buf) = bufloop_rx.try_recv() {
                    let Ok(()) = rx.submit(Some(buf)) else { break };
                }
                else {
                    let Ok(()) = rx.submit(None) else { break };
                }
            }
        } else {
            while let Ok(()) = rx.submit(None) {}
        }
    }
    let mut rf = rx.stop().await?;
    let duration = start.elapsed().as_secs_f64();
    let rate = (total as f64) / duration / 1e6;

    let m0_state_after = rf.debug().get_m0_state().await?;

    println!("Info: {:#x?}", info);
    println!("M0 state before = {:#x?}", m0_state_before);
    println!("M0 state after = {:#x?}", m0_state_after);
    println!("Total received: {}", total);
    println!("Total errors: {}", errors);
    println!("Total buffers: {}", buffers);
    println!("MBps: {}", rate);

    Ok(())
}
