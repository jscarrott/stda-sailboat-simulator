//! Wire protocol shared by the host simulator and the nRF52840 firmware.
//!
//! Messages are serde structs encoded with [`postcard`] and COBS-framed (each
//! frame is terminated by a `0x00` byte), so the receiver can resync on the
//! delimiter. Encoding is allocation-free on both ends — callers pass a fixed
//! `[u8; MAX_FRAME]` scratch buffer.
//!
//! Flow:
//! 1. Host → device: one [`ConfigPacket`] at startup (controller gains + boat
//!    `factor` + sail stretching).
//! 2. Host → device: a [`SensorPacket`] every control tick (raw simulated
//!    sensors — the device runs the full autopilot step).
//! 3. Device → host: a [`CommandPacket`] in reply (rudder + sail angle).
//!
//! Host→device messages share the [`HostMsg`] envelope so a single decode path
//! distinguishes config from sensor frames.

use serde::{Deserialize, Serialize};

/// Upper bound on an encoded + COBS-framed message. The largest payload is
/// `ConfigPacket` (8 × f32 = 32 bytes); postcard adds varint tagging and COBS a
/// little overhead. 64 bytes is comfortable headroom.
pub const MAX_FRAME: usize = 64;

/// Sent once at startup: everything the device needs to build its controller.
#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq)]
pub struct ConfigPacket {
    pub factor: f32,
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub sample_time: f32,
    pub speed_adaption: f32,
    pub max_rudder_angle: f32,
    pub sail_stretching: f32,
}

/// Sent every control tick: the simulated sensor snapshot. The device computes
/// apparent wind → sail trim → heading PID from these raw values.
#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq)]
pub struct SensorPacket {
    pub target_heading: f32,
    pub heading: f32,
    pub yaw_rate: f32,
    pub roll: f32,
    pub vel_x_body: f32,
    pub vel_y_body: f32,
    pub true_wind_dir: f32,
    pub true_wind_speed: f32,
}

/// Device → host reply: the actuator commands for this tick.
#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq)]
pub struct CommandPacket {
    pub rudder_angle: f32,
    pub sail_angle: f32,
}

/// Host → device envelope.
#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq)]
pub enum HostMsg {
    Config(ConfigPacket),
    Sensor(SensorPacket),
}

/// Encode `msg` into `buf` as a COBS-framed postcard message, returning the
/// written slice (including the trailing `0x00` delimiter).
pub fn encode<'a, T: Serialize>(
    msg: &T,
    buf: &'a mut [u8],
) -> Result<&'a mut [u8], postcard::Error> {
    postcard::to_slice_cobs(msg, buf)
}

/// Decode a single COBS-framed message from `frame` (the bytes up to and
/// including the `0x00` delimiter, which postcard's COBS reader consumes
/// in place).
pub fn decode<T: for<'de> Deserialize<'de>>(frame: &mut [u8]) -> Result<T, postcard::Error> {
    postcard::from_bytes_cobs(frame)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn host_msg_round_trips() {
        let mut buf = [0u8; MAX_FRAME];
        let cfg = HostMsg::Config(ConfigPacket {
            factor: 0.477,
            kp: 0.5,
            ki: 0.1,
            kd: 0.9,
            sample_time: 0.3,
            speed_adaption: 0.3,
            max_rudder_angle: 0.2618,
            sail_stretching: 0.961,
        });
        let n = encode(&cfg, &mut buf).unwrap().len();
        let got: HostMsg = decode(&mut buf[..n]).unwrap();
        assert_eq!(got, cfg);

        let sensor = HostMsg::Sensor(SensorPacket {
            target_heading: 0.5,
            heading: 0.1,
            yaw_rate: -0.02,
            roll: 0.05,
            vel_x_body: 1.5,
            vel_y_body: 0.1,
            true_wind_dir: 0.785,
            true_wind_speed: 5.0,
        });
        let mut buf2 = [0u8; MAX_FRAME];
        let n2 = encode(&sensor, &mut buf2).unwrap().len();
        let got2: HostMsg = decode(&mut buf2[..n2]).unwrap();
        assert_eq!(got2, sensor);
    }

    #[test]
    fn command_round_trips() {
        let mut buf = [0u8; MAX_FRAME];
        let cmd = CommandPacket { rudder_angle: -0.12, sail_angle: 0.34 };
        let n = encode(&cmd, &mut buf).unwrap().len();
        let got: CommandPacket = decode(&mut buf[..n]).unwrap();
        assert_eq!(got, cmd);
    }
}
