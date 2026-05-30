//! SX1262 LoRa link (optional, `lora` feature).
//!
//! Two roles, selected at build time:
//!
//! - **Boat** (`lora`, default role): [`run_node`] transmits the latest
//!   [`PositionReport`] periodically and listens for a [`WaypointCmd`], handing
//!   it to the control loop via [`crate::WAYPOINT`]; positions arrive via
//!   [`crate::POSITION`].
//! - **Shore bridge** (`bridge` feature): [`run_bridge`] relays between the USB
//!   serial link (COBS-framed, to the host `shore` tool) and the LoRa air
//!   interface — boat position reports go USB-out, host waypoints go LoRa-out.
//!
//! Point-to-point LoRa (no LoRaWAN). Each LoRa packet carries one bare postcard
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
//! (SPI3 is used so it doesn't clash with the OLED on the I2C instance-0 block.)

use defmt::{info, warn};
use embassy_nrf::gpio::{Input, Level, Output, OutputDrive, Pull};
use embassy_nrf::peripherals;
use embassy_nrf::spim::{self, Spim};
use embassy_time::{with_timeout, Delay, Duration, Timer};
#[cfg(not(feature = "bridge"))]
use embassy_time::Ticker;
use embedded_hal_async::delay::DelayNs;
use embedded_hal_bus::spi::ExclusiveDevice;
use lora_phy::iv::GenericSx126xInterfaceVariant;
use lora_phy::mod_params::{Bandwidth, CodingRate, ModulationParams, SpreadingFactor};
use lora_phy::mod_traits::RadioKind;
use lora_phy::sx126x::{self, Sx1262, Sx126x, TcxoCtrlVoltage};
use lora_phy::{LoRa, RxMode};

use boat_control::proto::{decode_unframed, encode_unframed, WaypointCmd, MAX_FRAME};

#[cfg(feature = "bridge")]
use boat_control::proto::{decode, encode, PositionReport};
#[cfg(feature = "bridge")]
use embassy_usb::class::cdc_acm::CdcAcmClass;

use crate::Irqs;
#[cfg(not(feature = "bridge"))]
use crate::{POSITION, WAYPOINT};

/// Operating frequency. **Default is EU 868 MHz** — change to a 915 MHz channel
/// (e.g. `903_900_000`) for US/Canada, and make sure your module and local ISM
/// regulations match.
const LORA_FREQUENCY_HZ: u32 = 868_100_000;
/// TX power (dBm). 14 dBm is the EU868 limit on most channels; the SX1262 can do
/// up to ~22 dBm where regulations permit.
const TX_POWER_DBM: i32 = 14;
/// How long to listen for an inbound LoRa packet before yielding.
const RX_WINDOW: Duration = Duration::from_secs(1);

/// Boat role: report position, accept waypoints.
#[cfg(not(feature = "bridge"))]
#[allow(clippy::too_many_arguments)]
pub async fn run_node(
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
    let (mut lora, mod_params) = match radio_init(Sx126x::new(spi_dev, iv, config), Delay).await {
        Some(r) => r,
        None => park("LoRa init failed").await,
    };

    let mut tx_buf = [0u8; MAX_FRAME];
    let mut rx_buf = [0u8; MAX_FRAME];
    let mut ticker = Ticker::every(Duration::from_secs(10));
    info!("LoRa node up @ {} Hz", LORA_FREQUENCY_HZ);

    loop {
        ticker.next().await;

        // Transmit the freshest position the control loop produced.
        if let Some(report) = POSITION.try_take() {
            let len = match encode_unframed(&report, &mut tx_buf) {
                Ok(p) => p.len(),
                Err(_) => continue,
            };
            lora_send(&mut lora, &mod_params, &tx_buf[..len]).await;
        }

        // Listen briefly for a waypoint from shore.
        if let Some(len) = lora_recv(&mut lora, &mod_params, &mut rx_buf, RX_WINDOW).await {
            if let Ok(wp) = decode_unframed::<WaypointCmd>(&rx_buf[..len]) {
                info!("LoRa waypoint received");
                WAYPOINT.signal(wp);
            }
        }
    }
}

/// Shore-bridge role: relay USB serial <-> LoRa for the host `shore` tool.
#[cfg(feature = "bridge")]
#[allow(clippy::too_many_arguments)]
pub async fn run_bridge<'d, D: embassy_usb::driver::Driver<'d>>(
    class: &mut CdcAcmClass<'d, D>,
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
    let (mut lora, mod_params) = match radio_init(Sx126x::new(spi_dev, iv, config), Delay).await {
        Some(r) => r,
        None => park("LoRa init failed").await,
    };

    class.wait_connection().await;
    info!("bridge up: USB <-> LoRa @ {} Hz", LORA_FREQUENCY_HZ);

    let mut acc = crate::heapless_acc::FrameAcc::new();
    let mut usb_in = [0u8; 64];
    let mut lora_buf = [0u8; MAX_FRAME];
    let mut cobs = [0u8; MAX_FRAME];

    loop {
        // Boat position (LoRa) -> host (USB, COBS-framed).
        if let Some(len) = lora_recv(&mut lora, &mod_params, &mut lora_buf, RX_WINDOW).await {
            if let Ok(report) = decode_unframed::<PositionReport>(&lora_buf[..len]) {
                if let Ok(out) = encode(&report, &mut cobs) {
                    let _ = class.write_packet(out).await;
                }
            }
        }

        // Host waypoint (USB) -> boat (LoRa, bare). Quick poll so we spend most
        // of the time listening.
        if let Ok(Ok(n)) =
            with_timeout(Duration::from_millis(50), class.read_packet(&mut usb_in)).await
        {
            for &b in &usb_in[..n] {
                if let Some(frame) = acc.push(b) {
                    if let Ok(wp) = decode::<WaypointCmd>(frame) {
                        let len = match encode_unframed(&wp, &mut lora_buf) {
                            Ok(p) => p.len(),
                            Err(_) => continue,
                        };
                        lora_send(&mut lora, &mod_params, &lora_buf[..len]).await;
                        info!("relayed waypoint to LoRa");
                    }
                }
            }
        }
    }
}

/// Bring up the radio and build the shared modulation params. Returns `None` on
/// any init failure.
async fn radio_init<RK, DLY>(radio: RK, delay: DLY) -> Option<(LoRa<RK, DLY>, ModulationParams)>
where
    RK: RadioKind,
    DLY: DelayNs,
{
    let mut lora = LoRa::new(radio, false, delay).await.ok()?;
    let mod_params = lora
        .create_modulation_params(
            SpreadingFactor::_10,
            Bandwidth::_125KHz,
            CodingRate::_4_8,
            LORA_FREQUENCY_HZ,
        )
        .ok()?;
    Some((lora, mod_params))
}

async fn lora_send<RK, DLY>(lora: &mut LoRa<RK, DLY>, mp: &ModulationParams, payload: &[u8])
where
    RK: RadioKind,
    DLY: DelayNs,
{
    if let Ok(mut pp) = lora.create_tx_packet_params(8, false, true, false, mp) {
        if lora
            .prepare_for_tx(mp, &mut pp, TX_POWER_DBM, payload)
            .await
            .is_ok()
        {
            let _ = lora.tx().await;
        }
    }
}

async fn lora_recv<RK, DLY>(
    lora: &mut LoRa<RK, DLY>,
    mp: &ModulationParams,
    buf: &mut [u8],
    window: Duration,
) -> Option<usize>
where
    RK: RadioKind,
    DLY: DelayNs,
{
    let pp = lora
        .create_rx_packet_params(8, false, MAX_FRAME as u8, true, false, mp)
        .ok()?;
    lora.prepare_for_rx(RxMode::Continuous, mp, &pp).await.ok()?;
    match with_timeout(window, lora.rx(&pp, buf)).await {
        Ok(Ok((len, _status))) => Some(len as usize),
        _ => None,
    }
}

/// No usable radio: log once and idle forever (never returns).
async fn park(reason: &str) -> ! {
    warn!("{}; LoRa link disabled", reason);
    loop {
        Timer::after(Duration::from_secs(60)).await;
    }
}
