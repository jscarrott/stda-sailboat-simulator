use std::f64::consts::PI;

use crate::config::Config;
use crate::physics::util::sign;

/// PID heading controller with anti-windup and low-speed gain shaping.
/// Port of `heading_controller.py:17`.
///
/// Default gains (0.5, 0.1, 0.9) are the ones Python actually runs
/// with. They are close to but not identical to the LQR-derived values
/// — re-derive via `scripts/compute_lqr_gains.py` if `Q`, `r`, or
/// `yaw_timeconstant` change. The LQR design inputs are
/// `Q = diag([0.1, 1, 0.3])` and `r = 30`; see the script for the
/// 3-state linearisation `[heading_error, yaw_rate, integrated_error]`.
pub struct HeadingController {
    pub sample_time: f64,
    pub speed_adaption: f64,
    pub max_rudder_angle: f64,
    pub factor: f64,
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
    summed_error: f64,
}

impl HeadingController {
    pub fn new(cfg: &Config, sample_time: f64) -> Self {
        let b = &cfg.boat;
        let e = &cfg.environment;
        let factor = b.distance_cog_rudder * b.rudder.area * PI * e.water_density / b.moi_z;
        Self {
            sample_time,
            speed_adaption: 0.3,
            max_rudder_angle: 15.0_f64.to_radians(),
            factor,
            kp: 0.5,
            ki: 0.1,
            kd: 0.9,
            summed_error: 0.0,
        }
    }

    /// Port of `heading_controller.controll()`.
    pub fn control(
        &mut self,
        desired_heading: f64,
        heading: f64,
        yaw_rate: f64,
        speed: f64,
        roll: f64,
        drift_angle: f64,
    ) -> f64 {
        let mut heading_error = desired_heading - heading;
        while heading_error > PI {
            heading_error -= 2.0 * PI;
        }
        while heading_error < -PI {
            heading_error += 2.0 * PI;
        }

        self.summed_error += self.sample_time * (heading_error - drift_angle);

        let effective_speed = if speed < self.speed_adaption {
            self.speed_adaption
        } else {
            speed
        };
        let factor2 = -1.0 / self.factor / (effective_speed * effective_speed) / roll.cos();

        let mut rudder_angle =
            factor2 * (self.kp * heading_error + self.ki * self.summed_error - self.kd * yaw_rate);

        if rudder_angle.abs() > self.max_rudder_angle {
            rudder_angle = sign(rudder_angle) * self.max_rudder_angle;
            // Anti-windup: back-calculate the integrator so it doesn't
            // wind past saturation. Mirrors heading_controller.py:122.
            self.summed_error = (rudder_angle / factor2 - (self.kp * heading_error - self.kd * yaw_rate)) / self.ki;
        }

        rudder_angle
    }

    pub fn reset(&mut self) {
        self.summed_error = 0.0;
    }
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
            let mut ctrl = HeadingController::new(&cfg, trace.sample_time);
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
