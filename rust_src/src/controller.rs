//! Host-side adapter over the shared [`boat_control`] controller.
//!
//! The control logic itself now lives in the `boat-control` crate (generic over
//! the float type, `no_std`, so it also runs on the nRF52840 firmware). The
//! simulator uses the `f64` instantiation — that keeps the bit-exact
//! Python-trace test below passing — and this module just supplies the
//! `Config`-derived constructor the rest of the simulator already calls.

use std::f64::consts::PI;

use crate::config::Config;

/// The simulator's heading controller: the shared generic controller pinned to
/// `f64`. Autopilots store this type directly.
pub type HeadingController = boat_control::HeadingController<f64>;

/// Build a [`HeadingController`] from the boat/environment config.
///
/// Computes `factor = distance_cog_rudder · rudder_area · π · water_density /
/// moi_z` and prefers LQR-derived gains from the YAML, falling back to the 4 m
/// hull's hand-tuned values `(0.5, 0.1, 0.9)` (which match what
/// `heading_controller.py:33-35` runs with — the Python-trace test depends on
/// it). `±15°` rudder limit, `0.3 m/s` low-speed clamp.
pub fn new_from_config(cfg: &Config, sample_time: f64) -> HeadingController {
    let b = &cfg.boat;
    let e = &cfg.environment;
    let factor = b.distance_cog_rudder * b.rudder.area * PI * e.water_density / b.moi_z;
    let (kp, ki, kd) = match cfg.controller_gains {
        Some(g) => (g.kp, g.ki, g.kd),
        None => (0.5, 0.1, 0.9),
    };
    HeadingController::from_params(factor, kp, ki, kd, sample_time, 0.3, 15.0_f64.to_radians())
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde::Deserialize;
    use std::path::PathBuf;

    fn manifest_dir() -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR"))
    }

    #[derive(Deserialize)]
    struct TracesFile {
        traces: Vec<Trace>,
    }

    #[derive(Deserialize)]
    struct Trace {
        name: String,
        sample_time: f64,
        inputs: Vec<Input>,
        outputs: Vec<f64>,
    }

    #[derive(Deserialize)]
    struct Input {
        desired_heading: f64,
        heading: f64,
        yaw_rate: f64,
        speed: f64,
        roll: f64,
        drift_angle: f64,
    }

    #[test]
    fn matches_python_controller_traces() {
        let cfg = Config::load(&manifest_dir().join("sim_params_config.yaml")).unwrap();
        let raw = std::fs::read_to_string(manifest_dir().join("tests/fixtures/controller_traces.json"))
            .expect("run scripts/dump_fixtures.py first");
        let file: TracesFile = serde_json::from_str(&raw).unwrap();
        for trace in &file.traces {
            let mut ctrl = new_from_config(&cfg, trace.sample_time);
            for (i, (input, &expected)) in trace.inputs.iter().zip(trace.outputs.iter()).enumerate() {
                let got = ctrl.control(
                    input.desired_heading,
                    input.heading,
                    input.yaw_rate,
                    input.speed,
                    input.roll,
                    input.drift_angle,
                );
                let diff = (got - expected).abs();
                assert!(
                    diff < 1e-12,
                    "trace {} step {} diff {} (rust {} vs py {})",
                    trace.name,
                    i,
                    diff,
                    got,
                    expected
                );
            }
        }
    }
}
