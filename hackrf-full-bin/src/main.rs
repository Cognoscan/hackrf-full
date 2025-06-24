#![allow(dead_code)]

use color_eyre::eyre::Context;

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

#[tokio::main(flavor = "current_thread")]
async fn main() -> color_eyre::Result<()> {
    color_eyre::install()?;
    let hackrf =
        hackrf_full::open_hackrf().wrap_err_with(|| String::from("Failed to open HackRF"))?;
    let info = hackrf.info();
    let api = info.api_version();
    let radio_type = info.radio_type();
    let id = info.board_id_read().await?;
    let serial = info.read_serial().await?;
    let rev = info.rev_read().await?;
    let version_string = info.version_string_read().await?;
    let platform = info.supported_platform_read().await?;

    let info = AllInfo {
        api,
        radio_type,
        id,
        rev,
        serial,
        version_string,
        platform,
    };

    println!("{:#x?}", info);

    Ok(())
}
