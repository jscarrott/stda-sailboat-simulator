//! nRF52840 hardware-in-the-loop controller firmware.
//!
//! Exposes a USB CDC-ACM virtual serial port. The host simulator sends a
//! [`ConfigPacket`] once (controller gains + boat `factor` + sail stretching),
//! then a [`SensorPacket`] every control tick. For each sensor packet the device
//! runs the *same* control logic as the in-process simulator —
//! [`boat_control::apparent_wind`] → [`boat_control::sail_angle`] →
//! [`boat_control::HeadingController`] — at `f32` on the Cortex-M4F FPU, and
//! replies with a [`CommandPacket`] (rudder + sail angle).
//!
//! This is the embedded half of the HIL rig; the host half is
//! `sailboat_sim`'s `--scenario hil`.

#![no_std]
#![no_main]
// In bridge builds the controller path (Autopilot, control_loop) is compiled but
// unused — the board only relays. Silence the dead-code warnings for that build.
#![cfg_attr(feature = "bridge", allow(dead_code))]

use defmt::{info, warn};
use embassy_executor::Spawner;
#[cfg(any(not(feature = "lora"), feature = "bridge"))]
use embassy_futures::join::join;
#[cfg(all(feature = "lora", not(feature = "bridge")))]
use embassy_futures::join::join3;
use embassy_nrf::usb::vbus_detect::HardwareVbusDetect;
use embassy_nrf::usb::Driver;
use embassy_nrf::{bind_interrupts, peripherals, spim, twim, usb};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::{Builder, Config};
use {defmt_rtt as _, panic_probe as _};

use boat_control::proto::{
    self, CommandPacket, ConfigPacket, HostMsg, SensorPacket, WaypointCmd, MAX_FRAME,
};
use boat_control::{apparent_wind, sail_angle, HeadingController};
use num_traits::Float;

mod display;
use display::Screen;

#[cfg(feature = "lora")]
mod radio;

// Channels between the fast control loop and the slow LoRa task. The control
// loop publishes the latest position; the LoRa task publishes received
// waypoints. `Signal` coalesces to the most recent value, which is exactly what
// a periodic telemetry/command link wants.
#[cfg(feature = "lora")]
use boat_control::proto::PositionReport;
#[cfg(feature = "lora")]
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
#[cfg(feature = "lora")]
static POSITION: Signal<CriticalSectionRawMutex, PositionReport> = Signal::new();
#[cfg(feature = "lora")]
static WAYPOINT: Signal<CriticalSectionRawMutex, WaypointCmd> = Signal::new();

bind_interrupts!(struct Irqs {
    USBD => usb::InterruptHandler<peripherals::USBD>;
    POWER_CLOCK => usb::vbus_detect::InterruptHandler;
    // I2C for the optional OLED status screen, SPI for the optional LoRa radio.
    // Both are bound even when their feature is off — the peripheral is simply
    // never enabled then.
    SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0 => twim::InterruptHandler<peripherals::TWISPI0>;
    SPIM3 => spim::InterruptHandler<peripherals::SPI3>;
});

/// Holds the controller state between sensor ticks. `None` until the host's
/// config packet arrives.
struct Autopilot {
    controller: HeadingController<f32>,
    sail_stretching: f32,
    last_sail_mag: f32,
}

impl Autopilot {
    fn from_config(cfg: &ConfigPacket) -> Self {
        Self {
            controller: HeadingController::from_params(
                cfg.factor,
                cfg.kp,
                cfg.ki,
                cfg.kd,
                cfg.sample_time,
                cfg.speed_adaption,
                cfg.max_rudder_angle,
            ),
            sail_stretching: cfg.sail_stretching,
            last_sail_mag: 0.0,
        }
    }

    /// One control tick — the f32 twin of `FixedHeadingAutopilot::step`.
    ///
    /// When `waypoint` is set (delivered over LoRa) the target heading is the
    /// bearing from the current position to the waypoint; otherwise the host's
    /// commanded heading is used.
    fn step(&mut self, s: &SensorPacket, waypoint: Option<WaypointCmd>) -> CommandPacket {
        let target_heading = match waypoint {
            Some(wp) => (wp.y - s.pos_y).atan2(wp.x - s.pos_x),
            None => s.target_heading,
        };

        let (app_angle, app_speed) = apparent_wind(
            s.heading,
            s.vel_x_body,
            s.vel_y_body,
            s.true_wind_dir,
            s.true_wind_speed,
        );
        self.last_sail_mag =
            sail_angle(app_angle, app_speed, self.sail_stretching, self.last_sail_mag);
        let sail_angle_cmd = app_angle.signum() * self.last_sail_mag;

        let speed = (s.vel_x_body * s.vel_x_body + s.vel_y_body * s.vel_y_body).sqrt();
        let drift = s.vel_y_body.atan2(s.vel_x_body);
        let rudder = self.controller.control(
            target_heading,
            s.heading,
            s.yaw_rate,
            speed,
            s.roll,
            drift,
        );

        CommandPacket { rudder_angle: rudder, sail_angle: sail_angle_cmd }
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());
    info!("nrf52840 HIL controller starting");

    // The USB peripheral needs the HF clock and VBUS detection; embassy's
    // HardwareVbusDetect drives that from the POWER/CLOCK peripheral.
    let driver = Driver::new(p.USBD, Irqs, HardwareVbusDetect::new(Irqs));

    let mut config = Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("stda-sailboat");
    config.product = Some("HIL controller");
    config.serial_number = Some("0001");
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    // USB descriptor / control buffers.
    let mut config_descriptor = [0u8; 256];
    let mut bos_descriptor = [0u8; 256];
    let mut msos_descriptor = [0u8; 128];
    let mut control_buf = [0u8; 64];
    let mut state = State::new();

    let mut builder = Builder::new(
        driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut msos_descriptor,
        &mut control_buf,
    );

    let mut class = CdcAcmClass::new(&mut builder, &mut state, 64);
    let mut usb = builder.build();

    let usb_fut = usb.run();

    // --- Shore-bridge role: relay USB <-> LoRa, no controller ---------------
    #[cfg(feature = "bridge")]
    {
        info!("running as LoRa <-> USB bridge");
        let bridge_fut = radio::run_bridge(
            &mut class, p.SPI3, p.P1_15, p.P1_14, p.P1_13, p.P1_12, p.P1_11, p.P1_10, p.P1_08,
        );
        join(usb_fut, bridge_fut).await;
    }

    // --- Boat role: run the controller (and optional LoRa node) -------------
    #[cfg(not(feature = "bridge"))]
    {
        // Optional OLED status screen on TWISPI0 (I2C). Default pins: P0.26 =
        // SDA, P0.27 = SCL (DK Arduino header). Without `--features display`
        // this is a no-op `()`.
        #[cfg(feature = "display")]
        let mut screen = {
            let mut tcfg = twim::Config::default();
            tcfg.frequency = twim::Frequency::K400;
            let twi = twim::Twim::new(p.TWISPI0, Irqs, p.P0_26, p.P0_27, tcfg);
            let iface = ssd1306::I2CDisplayInterface::new(twi);
            let disp = ssd1306::Ssd1306::new(
                iface,
                ssd1306::size::DisplaySize128x64,
                ssd1306::rotation::DisplayRotation::Rotate0,
            )
            .into_buffered_graphics_mode();
            display::OledScreen::new(disp)
        };
        #[cfg(not(feature = "display"))]
        let mut screen = ();

        let control_fut = async {
            let mut autopilot: Option<Autopilot> = None;
            loop {
                class.wait_connection().await;
                info!("host connected");
                if control_loop(&mut class, &mut autopilot, &mut screen).await.is_err() {
                    warn!("host disconnected");
                }
            }
        };

        // Optional SX1262 LoRa supervisory link on SPI3 (DK Arduino header).
        #[cfg(feature = "lora")]
        {
            let lora_fut = radio::run_node(
                p.SPI3, p.P1_15, p.P1_14, p.P1_13, p.P1_12, p.P1_11, p.P1_10, p.P1_08,
            );
            join3(usb_fut, control_fut, lora_fut).await;
        }
        #[cfg(not(feature = "lora"))]
        join(usb_fut, control_fut).await;
    }
}

struct Disconnected;

impl From<EndpointError> for Disconnected {
    fn from(e: EndpointError) -> Self {
        match e {
            EndpointError::BufferOverflow => defmt::panic!("USB buffer overflow"),
            EndpointError::Disabled => Disconnected,
        }
    }
}

/// Read COBS-framed host messages, run the controller, write command replies.
async fn control_loop<'d, D: embassy_usb::driver::Driver<'d>>(
    class: &mut CdcAcmClass<'d, D>,
    autopilot: &mut Option<Autopilot>,
    screen: &mut impl Screen,
) -> Result<(), Disconnected> {
    // RX accumulator: bytes up to a `0x00` COBS delimiter form one frame.
    let mut rx: heapless_acc::FrameAcc = heapless_acc::FrameAcc::new();
    let mut packet = [0u8; 64];
    let mut tx = [0u8; MAX_FRAME];
    // Latest waypoint delivered over LoRa (latched across ticks). Always `None`
    // without the `lora` feature.
    #[allow(unused_mut)]
    let mut waypoint: Option<WaypointCmd> = None;

    loop {
        let n = class.read_packet(&mut packet).await?;
        for &byte in &packet[..n] {
            let Some(frame) = rx.push(byte) else { continue };
            // `frame` is one complete COBS frame (incl. the 0x00 delimiter).
            match proto::decode::<HostMsg>(frame) {
                Ok(HostMsg::Config(cfg)) => {
                    *autopilot = Some(Autopilot::from_config(&cfg));
                    info!("configured");
                }
                Ok(HostMsg::Sensor(sensor)) => {
                    // Pick up a waypoint the LoRa task may have received.
                    #[cfg(feature = "lora")]
                    if let Some(wp) = WAYPOINT.try_take() {
                        info!("steering to LoRa waypoint");
                        waypoint = Some(wp);
                    }

                    if let Some(ap) = autopilot.as_mut() {
                        let cmd = ap.step(&sensor, waypoint);
                        if let Ok(out) = proto::encode(&cmd, &mut tx) {
                            class.write_packet(out).await?;
                        }
                        screen.show(&sensor, &cmd);

                        // Publish position for the LoRa telemetry task.
                        #[cfg(feature = "lora")]
                        {
                            let speed = (sensor.vel_x_body * sensor.vel_x_body
                                + sensor.vel_y_body * sensor.vel_y_body)
                                .sqrt();
                            POSITION.signal(PositionReport {
                                pos_x: sensor.pos_x,
                                pos_y: sensor.pos_y,
                                heading: sensor.heading,
                                speed,
                            });
                        }
                    } else {
                        warn!("sensor before config; ignoring");
                    }
                }
                Err(_) => warn!("bad frame"),
            }
        }
    }
}

/// A tiny fixed-capacity byte accumulator that yields a complete COBS frame
/// (terminated by `0x00`) without heap allocation.
mod heapless_acc {
    use boat_control::proto::MAX_FRAME;

    pub struct FrameAcc {
        buf: [u8; MAX_FRAME],
        len: usize,
    }

    impl FrameAcc {
        pub fn new() -> Self {
            Self { buf: [0u8; MAX_FRAME], len: 0 }
        }

        /// Push one byte. Returns the complete frame (including the trailing
        /// `0x00`) when a delimiter is seen, then resets. Overlong frames are
        /// dropped (resync on the next delimiter).
        pub fn push(&mut self, byte: u8) -> Option<&mut [u8]> {
            if self.len < self.buf.len() {
                self.buf[self.len] = byte;
                self.len += 1;
            } else {
                // Overflow: drop and wait for the next delimiter.
                if byte == 0 {
                    self.len = 0;
                }
                return None;
            }
            if byte == 0 {
                let end = self.len;
                self.len = 0;
                Some(&mut self.buf[..end])
            } else {
                None
            }
        }
    }
}
