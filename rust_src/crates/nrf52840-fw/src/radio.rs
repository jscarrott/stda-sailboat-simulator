//! SX1262 LoRa supervisory link (optional, `lora` feature).
//!
//! This is the *slow, long-range* channel, separate from the fast USB control
//! loop. On a schedule it transmits the latest [`PositionReport`] (boat → shore)
//! and listens briefly for a [`WaypointCmd`] (shore → boat). Received waypoints
//! are handed to the control loop via the [`crate::WAYPOINT`] signal, which then
//! steers toward them; positions come in via [`crate::POSITION`].
//!
//! Wiring is point-to-point LoRa (no LoRaWAN): a matching SX1262 radio at the
//! shore station talks to this one. Each LoRa packet carries one bare postcard
//! message (no COBS — a LoRa frame is already delimited and CRC-checked).
//!
//! ## Default pins (nRF52840-DK Arduino header, SPI3)
//! | Signal | Pin   | Arduino |
//! |--------|-------|---------|
//! | SCK    | P1.15 | D13     |
//! | MISO   | P1.14 | D12     |
//! | MOSI   | P1.13 | D11     |
//! | NSS    | P1.12 | D10     |
//! | RESET  | P1.11 | D9      |
//! | BUSY   | P1.10 | D8      |
//! | DIO1   | P1.08 | D7      |
//!
//! (SPI3 is used so it doesn't clash with the OLED, which is on the I2C/SPI
//! instance-0 block as TWISPI0.)

use defmt::{info, warn};
use embassy_nrf::gpio::{Input, Level, Output, OutputDrive, Pull};
use embassy_nrf::peripherals;
use embassy_nrf::spim::{self, Spim};
use embassy_time::{with_timeout, Delay, Duration, Ticker, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;
use lora_phy::iv::GenericSx126xInterfaceVariant;
use lora_phy::mod_params::{Bandwidth, CodingRate, SpreadingFactor};
use lora_phy::sx126x::{self, Sx1262, Sx126x, TcxoCtrlVoltage};
use lora_phy::{LoRa, RxMode};

use boat_control::proto::{decode_unframed, encode_unframed, WaypointCmd, MAX_FRAME};

use crate::{Irqs, POSITION, WAYPOINT};

/// Operating frequency. **Default is EU 868 MHz** — change to a 915 MHz channel
/// (e.g. `903_900_000`) for US/Canada, and make sure your module and local ISM
/// regulations match.
const LORA_FREQUENCY_HZ: u32 = 868_100_000;
/// TX power (dBm). 14 dBm is the EU868 limit on most channels; the SX1262 can do
/// up to ~22 dBm where regulations permit.
const TX_POWER_DBM: i32 = 14;
/// How often to transmit a position report.
const REPORT_PERIOD: Duration = Duration::from_secs(10);
/// How long to listen for a waypoint after each report.
const LISTEN_WINDOW: Duration = Duration::from_secs(2);

#[allow(clippy::too_many_arguments)]
pub async fn run(
    spi3: peripherals::SPI3,
    sck: peripherals::P1_15,
    miso: peripherals::P1_14,
    mosi: peripherals::P1_13,
    nss: peripherals::P1_12,
    reset: peripherals::P1_11,
    busy: peripherals::P1_10,
    dio1: peripherals::P1_08,
) -> ! {
    let mut spi_config = spim::Config::default();
    spi_config.frequency = spim::Frequency::M8;
    let spi_bus = Spim::new(spi3, Irqs, sck, miso, mosi, spi_config);

    let nss = Output::new(nss, Level::High, OutputDrive::Standard);
    let reset = Output::new(reset, Level::High, OutputDrive::Standard);
    let busy = Input::new(busy, Pull::Down);
    let dio1 = Input::new(dio1, Pull::Down);

    let spi_dev = ExclusiveDevice::new(spi_bus, nss, Delay).expect("spi device");

    let config = sx126x::Config {
        chip: Sx1262,
        tcxo_ctrl: Some(TcxoCtrlVoltage::Ctrl1V7),
        use_dcdc: true,
        rx_boost: false,
    };
    let iv = GenericSx126xInterfaceVariant::new(reset, dio1, busy, None, None)
        .expect("sx126x interface");

    let mut lora = match LoRa::new(Sx126x::new(spi_dev, iv, config), false, Delay).await {
        Ok(lora) => lora,
        Err(_) => {
            warn!("LoRa init failed; supervisory link disabled");
            loop {
                Timer::after(Duration::from_secs(60)).await;
            }
        }
    };

    let mod_params = lora
        .create_modulation_params(
            SpreadingFactor::_10,
            Bandwidth::_125KHz,
            CodingRate::_4_8,
            LORA_FREQUENCY_HZ,
        )
        .expect("modulation params");

    let mut tx_buf = [0u8; MAX_FRAME];
    let mut rx_buf = [0u8; MAX_FRAME];
    let mut ticker = Ticker::every(REPORT_PERIOD);

    info!("LoRa supervisory link up @ {} Hz", LORA_FREQUENCY_HZ);

    loop {
        ticker.next().await;

        // 1) Transmit the freshest position the control loop has produced.
        if let Some(report) = POSITION.try_take() {
            if let Ok(payload) = encode_unframed(&report, &mut tx_buf) {
                let len = payload.len();
                if let Ok(mut tx_params) =
                    lora.create_tx_packet_params(8, false, true, false, &mod_params)
                {
                    if lora
                        .prepare_for_tx(&mod_params, &mut tx_params, TX_POWER_DBM, &tx_buf[..len])
                        .await
                        .is_ok()
                    {
                        let _ = lora.tx().await;
                    }
                }
            }
        }

        // 2) Listen briefly for a waypoint command from shore.
        if let Ok(rx_params) =
            lora.create_rx_packet_params(8, false, MAX_FRAME as u8, true, false, &mod_params)
        {
            if lora
                .prepare_for_rx(RxMode::Continuous, &mod_params, &rx_params)
                .await
                .is_ok()
            {
                if let Ok(Ok((len, _status))) =
                    with_timeout(LISTEN_WINDOW, lora.rx(&rx_params, &mut rx_buf)).await
                {
                    if let Ok(wp) = decode_unframed::<WaypointCmd>(&rx_buf[..len as usize]) {
                        info!("LoRa waypoint received");
                        WAYPOINT.signal(wp);
                    }
                }
            }
        }
    }
}
