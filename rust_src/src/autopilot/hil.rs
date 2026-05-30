//! Hardware-in-the-loop autopilot: the control law runs on an nRF52840 over
//! USB, not in this process.
//!
//! Each control tick this adapter packs the simulator's [`Observation`] into a
//! [`proto::SensorPacket`], writes it over the device's USB CDC-ACM virtual
//! serial port, blocks for the [`proto::CommandPacket`] reply, and hands the
//! rudder/sail angles back to the simulator as a [`Command`]. The simulator's
//! physics, wind and the boat itself are unchanged — only the controller has
//! moved across the wire. Mirrors what `FixedHeadingAutopilot` does in-process.
//!
//! Enabled by the `hil` cargo feature (which pulls in `serialport`).

use std::io::{Read, Write};
use std::time::Duration;

use anyhow::{Context, Result};
use boat_control::proto::{self, CommandPacket, ConfigPacket, HostMsg, SensorPacket, MAX_FRAME};
use serialport::SerialPort;

use crate::autopilot::{Autopilot, Command, Observation};
use crate::config::Config;

pub struct HilAutopilot {
    port: Box<dyn SerialPort>,
    target_heading: f64,
    /// Accumulates received bytes until a COBS frame delimiter (`0x00`).
    rx: Vec<u8>,
    scratch: [u8; MAX_FRAME],
}

impl HilAutopilot {
    /// Open `port_path` (e.g. `/dev/ttyACM0`), send the controller config the
    /// device needs, and hold `target_heading` (rad). `sample_time` is the
    /// control period the device integrates the PID with.
    pub fn new(
        cfg: &Config,
        target_heading: f64,
        sample_time: f64,
        port_path: &str,
    ) -> Result<Self> {
        // CDC-ACM ignores the baud rate, but the API requires one.
        let port = serialport::new(port_path, 115_200)
            .timeout(Duration::from_secs(2))
            .open()
            .with_context(|| format!("opening HIL serial port {port_path}"))?;

        // factor mirrors controller::new_from_config; the device builds its
        // HeadingController<f32> from exactly these values.
        let b = &cfg.boat;
        let e = &cfg.environment;
        let factor =
            b.distance_cog_rudder * b.rudder.area * std::f64::consts::PI * e.water_density / b.moi_z;
        let (kp, ki, kd) = match cfg.controller_gains {
            Some(g) => (g.kp, g.ki, g.kd),
            None => (0.5, 0.1, 0.9),
        };
        let config = HostMsg::Config(ConfigPacket {
            factor: factor as f32,
            kp: kp as f32,
            ki: ki as f32,
            kd: kd as f32,
            sample_time: sample_time as f32,
            speed_adaption: 0.3,
            max_rudder_angle: 15.0_f32.to_radians(),
            sail_stretching: cfg.boat.sail.stretching as f32,
        });

        let mut hil = Self {
            port,
            target_heading,
            rx: Vec::with_capacity(2 * MAX_FRAME),
            scratch: [0u8; MAX_FRAME],
        };
        hil.send(&config).context("sending HIL config packet")?;
        Ok(hil)
    }

    fn send<T: serde::Serialize>(&mut self, msg: &T) -> Result<()> {
        let frame = proto::encode(msg, &mut self.scratch)
            .map_err(|e| anyhow::anyhow!("encoding HIL frame: {e}"))?;
        self.port.write_all(frame).context("writing HIL frame")?;
        self.port.flush().context("flushing HIL frame")?;
        Ok(())
    }

    /// Block until a full COBS frame (terminated by `0x00`) has arrived, then
    /// decode it as a `CommandPacket`.
    fn recv_command(&mut self) -> Result<CommandPacket> {
        loop {
            if let Some(end) = self.rx.iter().position(|&b| b == 0) {
                // Drain one frame (including the delimiter) and decode it.
                let mut frame: Vec<u8> = self.rx.drain(..=end).collect();
                let cmd: CommandPacket = proto::decode(&mut frame)
                    .map_err(|e| anyhow::anyhow!("decoding HIL command frame: {e}"))?;
                return Ok(cmd);
            }
            let mut buf = [0u8; MAX_FRAME];
            let n = self.port.read(&mut buf).context("reading HIL reply")?;
            if n == 0 {
                anyhow::bail!("HIL serial closed before a full command frame arrived");
            }
            self.rx.extend_from_slice(&buf[..n]);
        }
    }
}

impl Autopilot for HilAutopilot {
    fn step(&mut self, obs: &Observation) -> Command {
        let sensor = HostMsg::Sensor(SensorPacket {
            target_heading: self.target_heading as f32,
            heading: obs.heading as f32,
            yaw_rate: obs.yaw_rate as f32,
            roll: obs.roll as f32,
            vel_x_body: obs.vel_x_body as f32,
            vel_y_body: obs.vel_y_body as f32,
            true_wind_dir: obs.true_wind.direction as f32,
            true_wind_speed: obs.true_wind.speed as f32,
            pos_x: obs.pos_x as f32,
            pos_y: obs.pos_y as f32,
        });
        // A dropped frame mid-run is unrecoverable for the sim, so surface it
        // as a panic rather than silently freezing the actuators.
        self.send(&sensor).expect("HIL: send sensor packet");
        let cmd = self.recv_command().expect("HIL: receive command packet");
        Command {
            rudder_angle: cmd.rudder_angle as f64,
            sail_angle: cmd.sail_angle as f64,
            mission_complete: false,
        }
    }
}
