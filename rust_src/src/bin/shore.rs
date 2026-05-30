//! Shore-station LoRa tool.
//!
//! Talks to a **LoRa↔USB bridge** — a second nRF52840 running this firmware
//! built with `--features bridge` — over its USB CDC-ACM serial port. The bridge
//! relays between that serial link (COBS-framed postcard, same as the HIL link)
//! and the LoRa air interface, so this tool:
//!
//! - prints [`PositionReport`]s the boat broadcasts (boat → LoRa → bridge → USB), and
//! - sends [`WaypointCmd`]s to the boat (USB → bridge → LoRa → boat).
//!
//! ```text
//!  boat (--features lora) ⇄ LoRa ⇄ bridge (--features bridge) ⇄ USB ⇄ this tool
//! ```
//!
//! Usage:
//! ```bash
//! cargo run --features hil --bin shore -- --port /dev/ttyACM0 --waypoint 800,950
//! # then type "X,Y" lines on stdin to send more waypoints; Ctrl-D to quit.
//! cargo run --features hil --bin shore -- --loopback   # offline self-test
//! ```

use std::io::{BufRead, Read};
use std::time::{Duration, Instant};

use anyhow::{Context, Result};
use boat_control::proto::{self, PositionReport, WaypointCmd, MAX_FRAME};
use clap::Parser;

#[derive(Parser, Debug)]
#[command(name = "shore", about = "Shore-station LoRa tracker / waypoint sender")]
struct Cli {
    /// Serial port of the LoRa↔USB bridge (a second nRF52840 flashed with
    /// `--features bridge`).
    #[arg(long, default_value = "/dev/ttyACM0")]
    port: String,

    /// Waypoint to send on connect, as `X,Y` in metres (repeatable).
    #[arg(long = "waypoint", value_parser = parse_xy)]
    waypoints: Vec<(f64, f64)>,

    /// Don't open a port; run an offline loopback self-test of the framing and
    /// formatting (no hardware needed).
    #[arg(long)]
    loopback: bool,
}

fn parse_xy(s: &str) -> Result<(f64, f64), String> {
    let s = s.trim();
    // Accept `X,Y` or whitespace-separated `X Y`.
    let (a, b) = match s.split_once(',') {
        Some(pair) => pair,
        None => s
            .split_once(char::is_whitespace)
            .ok_or_else(|| "expected `X,Y`".to_string())?,
    };
    let x = a.trim().parse().map_err(|_| format!("bad X in {s:?}"))?;
    let y = b.trim().parse().map_err(|_| format!("bad Y in {s:?}"))?;
    Ok((x, y))
}

fn format_report(r: &PositionReport, t: Duration) -> String {
    format!(
        "[{:6.1}s] pos=({:8.1}, {:8.1}) m  hdg={:5.1}°  speed={:4.2} m/s",
        t.as_secs_f64(),
        r.pos_x,
        r.pos_y,
        r.heading.to_degrees().rem_euclid(360.0),
        r.speed,
    )
}

fn main() -> Result<()> {
    let cli = Cli::parse();

    if cli.loopback {
        return loopback();
    }

    let port = serialport::new(&cli.port, 115_200)
        .timeout(Duration::from_millis(200))
        .open()
        .with_context(|| format!("opening bridge serial port {}", cli.port))?;
    println!("connected to bridge on {}", cli.port);

    // Reader thread: decode incoming position reports and print them.
    let mut reader = port.try_clone().context("cloning serial port for reader")?;
    let start = Instant::now();
    std::thread::spawn(move || {
        let mut acc: Vec<u8> = Vec::with_capacity(2 * MAX_FRAME);
        let mut buf = [0u8; 64];
        loop {
            match reader.read(&mut buf) {
                Ok(0) => break,
                Ok(n) => acc.extend_from_slice(&buf[..n]),
                // Timeouts are normal (no traffic) — keep waiting.
                Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => continue,
                Err(e) => {
                    eprintln!("serial read error: {e}");
                    break;
                }
            }
            while let Some(end) = acc.iter().position(|&b| b == 0) {
                let mut frame: Vec<u8> = acc.drain(..=end).collect();
                match proto::decode::<PositionReport>(&mut frame) {
                    Ok(report) => println!("{}", format_report(&report, start.elapsed())),
                    Err(_) => eprintln!("(dropped malformed frame)"),
                }
            }
        }
    });

    // Writer: send the waypoints given on the command line, then any typed on
    // stdin as `X,Y` lines.
    let mut writer = port;
    for &(x, y) in &cli.waypoints {
        send_waypoint(&mut *writer, x, y)?;
    }
    if !cli.waypoints.is_empty() {
        println!("sent {} waypoint(s); type more as `X,Y`, Ctrl-D to quit", cli.waypoints.len());
    } else {
        println!("type waypoints as `X,Y` (metres), Ctrl-D to quit");
    }

    let stdin = std::io::stdin();
    for line in stdin.lock().lines() {
        let line = line?;
        let line = line.trim();
        if line.is_empty() {
            continue;
        }
        match parse_xy(line) {
            Ok((x, y)) => send_waypoint(&mut *writer, x, y)?,
            Err(e) => eprintln!("ignored: {e}"),
        }
    }
    Ok(())
}

fn send_waypoint(port: &mut dyn serialport::SerialPort, x: f64, y: f64) -> Result<()> {
    let mut buf = [0u8; MAX_FRAME];
    let wp = WaypointCmd { x: x as f32, y: y as f32 };
    let frame = proto::encode(&wp, &mut buf).map_err(|e| anyhow::anyhow!("encoding waypoint: {e}"))?;
    port.write_all(frame).context("writing waypoint")?;
    port.flush().context("flushing waypoint")?;
    println!("-> waypoint ({x:.1}, {y:.1})");
    Ok(())
}

/// Offline self-test: round-trip both message types through the on-wire framing
/// and exercise the formatting, with no serial port.
fn loopback() -> Result<()> {
    println!("loopback self-test (no hardware)");

    let mut buf = [0u8; MAX_FRAME];
    let wp = WaypointCmd { x: 800.0, y: 950.0 };
    let n = proto::encode(&wp, &mut buf).map_err(|e| anyhow::anyhow!("encode: {e}"))?.len();
    let got: WaypointCmd = proto::decode(&mut buf[..n]).map_err(|e| anyhow::anyhow!("decode: {e}"))?;
    anyhow::ensure!(got == wp, "waypoint round-trip mismatch");
    println!("  waypoint round-trip ok ({n} bytes on the wire)");

    let report = PositionReport { pos_x: 1500.0, pos_y: -200.0, heading: 1.5708, speed: 1.42 };
    let n = proto::encode(&report, &mut buf).map_err(|e| anyhow::anyhow!("encode: {e}"))?.len();
    let got: PositionReport = proto::decode(&mut buf[..n]).map_err(|e| anyhow::anyhow!("decode: {e}"))?;
    anyhow::ensure!(got == report, "position round-trip mismatch");
    println!("  position round-trip ok ({n} bytes on the wire)");
    println!("  would print: {}", format_report(&report, Duration::from_secs_f64(12.3)));
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_waypoint_forms() {
        assert_eq!(parse_xy("800,950").unwrap(), (800.0, 950.0));
        assert_eq!(parse_xy(" -12.5 , 7 ").unwrap(), (-12.5, 7.0));
        assert_eq!(parse_xy("3 4").unwrap(), (3.0, 4.0));
        assert!(parse_xy("nope").is_err());
        assert!(parse_xy("1,").is_err());
    }

    #[test]
    fn loopback_runs() {
        loopback().unwrap();
    }
}
