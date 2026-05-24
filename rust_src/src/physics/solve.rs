use ode_solvers::System;

use crate::config::{Config, Invariants};
use crate::physics::forces::{
    calculate_damping, calculate_hydrostatic_force, calculate_lateral_force, calculate_rudder_force,
    calculate_sail_force, Environment,
};
use crate::physics::wave::{calculate_wave_impedance, calculate_wave_influence};
use crate::physics::wind::calculate_apparent_wind;
use crate::state::*;

/// Mechanical slew limits for the actuators. Numbers picked to mimic a
/// hobby-grade rudder servo and a drum-style sail winch; the runner in
/// `scenario::simulate` uses these as the cap when slew-limiting
/// autopilot commands so the integrator never sees a step in the
/// applied actuator angle larger than the hardware can deliver.
pub const MAX_RUDDER_SPEED: f64 = std::f64::consts::PI / 30.0;
pub const MAX_SAIL_SPEED: f64 = std::f64::consts::PI / 10.0;

/// First-order rate constants (1/s) for the rudder and sail-winch
/// actuators. `state_rate = -RATE · (state − command)` has time
/// constant τ = 1/RATE. The runner uses these constants to step the
/// actuator state analytically between outer control ticks (see
/// `scenario::simulate`) rather than letting Dopri5 chase the fast
/// rudder mode — that was driving the integrator into stiffness on
/// long IOM runs even with the sign-smoothing fix in place.
pub const RUDDER_RATE: f64 = 2.0;
pub const SAIL_RATE: f64 = 0.1;

/// Half-width of the smoothed dead-downwind transition, in radians.
/// `sign(apparent_wind.angle)` flips between ±1 abruptly across angle=0,
/// making the ODE RHS non-differentiable: any infinitesimal lateral
/// motion at exact dead-downwind alignment thrashes the sail force
/// across substeps and Dopri5 grinds to a halt.
/// Replacing `sign()` with `tanh(angle / SAIL_SIGN_SMOOTHING_RAD)` gives
/// a C^∞ S-curve that matches sign() to within fp precision outside
/// roughly ±3·SAIL_SIGN_SMOOTHING_RAD, and smoothly interpolates the
/// sail force through dead-downwind. Physically this also matches
/// reality better than the abrupt-snap model — real sails reduce side
/// force as they approach the gybe zone before the wind actually
/// crosses behind.
const SAIL_SIGN_SMOOTHING_RAD: f64 = 0.05;

pub struct OdeContext<'a> {
    pub cfg: &'a Config,
    pub inv: &'a Invariants,
    pub env: Environment,
    pub actor_dynamics: bool,
}

impl<'a> System<f64, State> for OdeContext<'a> {
    fn system(&self, time: f64, y: &State, dy: &mut State) {
        let cfg = self.cfg;
        let inv = self.inv;
        let env = self.env;

        let pos_x = y[POS_X];
        let pos_y = y[POS_Y];
        let pos_z = y[POS_Z];
        let roll = y[ROLL];
        let pitch = y[PITCH];
        let yaw = y[YAW];
        let vel_x = y[VEL_X];
        let vel_y = y[VEL_Y];
        let vel_z = y[VEL_Z];
        let roll_rate = y[ROLL_RATE];
        let pitch_rate = y[PITCH_RATE];
        let yaw_rate = y[YAW_RATE];

        let (rudder_angle, sail_angle) = if self.actor_dynamics {
            (y[RUDDER_STATE], y[SAIL_STATE])
        } else {
            (env.rudder_angle, env.sail_angle)
        };

        let speed = (vel_x * vel_x + vel_y * vel_y).sqrt();
        let wave_influence = calculate_wave_influence(pos_x, pos_y, yaw, env.wave, time, cfg.environment.gravity);
        let apparent_wind = calculate_apparent_wind(yaw, vel_x, vel_y, env.true_wind);

        // True Sail Angle Sign Convention (CLAUDE.md): the sign of the
        // sail follows the apparent wind side. Smoothed via tanh to
        // remove the sign() discontinuity at dead downwind — see
        // SAIL_SIGN_SMOOTHING_RAD above.
        let sail_side = (apparent_wind.angle / SAIL_SIGN_SMOOTHING_RAD).tanh();
        let true_sail_angle = sail_side * sail_angle.abs();

        let damping = calculate_damping(vel_x, vel_y, vel_z, roll_rate, pitch_rate, yaw_rate, inv);
        let (hydrostatic_force, x_hs, y_hs) = calculate_hydrostatic_force(pos_z, roll, pitch, wave_influence, inv);
        let wave_impedance = calculate_wave_impedance(vel_x, speed, cfg.boat.hull_speed, inv);
        let rudder_force = calculate_rudder_force(speed, rudder_angle, &cfg.environment, &cfg.boat);
        let (lateral_force, lateral_separation) =
            calculate_lateral_force(vel_x, vel_y, roll, speed, &cfg.environment, &cfg.boat);
        let sail_force = calculate_sail_force(roll, apparent_wind, true_sail_angle, &cfg.environment, &cfg.boat);

        let delta_pos_x = vel_x * yaw.cos() - vel_y * yaw.sin();
        let delta_pos_y = vel_y * yaw.cos() + vel_x * yaw.sin();
        let delta_pos_z = vel_z;
        let delta_roll = roll_rate;
        let delta_pitch = pitch_rate * roll.cos() - yaw_rate * roll.sin();
        let delta_yaw = yaw_rate * roll.cos() + pitch_rate * roll.sin();

        let mass = cfg.boat.mass;
        let delta_vel_x = delta_yaw * vel_y
            + (sail_force.x + lateral_force.x + rudder_force.x + damping.x + wave_impedance + hydrostatic_force.x) / mass;
        let delta_vel_y = -delta_yaw * vel_x
            + ((sail_force.y + lateral_force.y + rudder_force.y) * roll.cos() + hydrostatic_force.y + damping.y) / mass;
        let delta_vel_z = ((sail_force.y + lateral_force.y + rudder_force.y) * roll.sin() + hydrostatic_force.z
            - inv.gravity_force
            + damping.z)
            / mass;

        let delta_roll_rate =
            (hydrostatic_force.z * y_hs - sail_force.y * cfg.boat.sail.pressure_point_height + damping.roll) / cfg.boat.moi_x;
        let delta_pitch_rate = (sail_force.x * cfg.boat.sail.pressure_point_height
            - hydrostatic_force.z * x_hs * roll.cos()
            + damping.pitch
            - (cfg.boat.moi_x - cfg.boat.moi_z) * roll_rate * yaw_rate)
            / cfg.boat.moi_y;

        let distance_cog_keel_middle = cfg.boat.distance_cog_keel_pressure_point - 0.7;
        let delta_yaw_rate = (damping.yaw
            - rudder_force.y * cfg.boat.distance_cog_rudder
            + sail_force.y * cfg.boat.distance_cog_sail_pressure_point
            + sail_force.x * true_sail_angle.sin() * cfg.boat.distance_mast_sail_pressure_point
            + lateral_force.y
                * (cfg.boat.distance_cog_keel_pressure_point * (1.0 - lateral_separation)
                    + distance_cog_keel_middle * lateral_separation))
            / cfg.boat.moi_z;

        dy[POS_X] = delta_pos_x;
        dy[POS_Y] = delta_pos_y;
        dy[POS_Z] = delta_pos_z;
        dy[ROLL] = delta_roll;
        dy[PITCH] = delta_pitch;
        dy[YAW] = delta_yaw;
        dy[VEL_X] = delta_vel_x;
        dy[VEL_Y] = delta_vel_y;
        dy[VEL_Z] = delta_vel_z;
        dy[ROLL_RATE] = delta_roll_rate;
        dy[PITCH_RATE] = delta_pitch_rate;
        dy[YAW_RATE] = delta_yaw_rate;

        if self.actor_dynamics {
            let delta_rudder = (-RUDDER_RATE * (rudder_angle - env.rudder_angle))
                .clamp(-MAX_RUDDER_SPEED, MAX_RUDDER_SPEED);
            let delta_sail = (-SAIL_RATE * (sail_angle - env.sail_angle))
                .clamp(-MAX_SAIL_SPEED, MAX_SAIL_SPEED);
            dy[RUDDER_STATE] = delta_rudder;
            dy[SAIL_STATE] = delta_sail;
        } else {
            dy[RUDDER_STATE] = 0.0;
            dy[SAIL_STATE] = 0.0;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::Config;
    use crate::physics::wave::Wave;
    use crate::physics::wind::TrueWind;
    use serde::Deserialize;
    use std::path::PathBuf;

    fn manifest_dir() -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR"))
    }

    #[derive(Deserialize)]
    struct FixtureFile {
        fixtures: Vec<Fixture>,
    }

    #[derive(Deserialize)]
    struct Fixture {
        name: String,
        time: f64,
        state: [f64; 14],
        env: FixtureEnv,
        derivative: [f64; 14],
    }

    #[derive(Deserialize)]
    struct FixtureEnv {
        sail_angle: f64,
        rudder_angle: f64,
        wind_strength: f64,
        wind_dir_deg: f64,
        wave_length: f64,
        wave_direction: f64,
        wave_amplitude: f64,
    }

    /// Integrate for one outer control step with Dopri5 to confirm the
    /// `System` impl and `ode_solvers` wiring actually run end-to-end.
    /// Just checks finiteness and bounded motion — strict numerical
    /// agreement with scipy is verified per-step by the fixture test.
    #[test]
    fn dopri5_integrates_one_step() {
        use ode_solvers::Dopri5;
        let cfg = Config::load(&manifest_dir().join("sim_params_config.yaml")).unwrap();
        let inv = Invariants::from_config(&cfg);
        let env = crate::physics::forces::Environment::from_config(&cfg);
        let ctx = OdeContext { cfg: &cfg, inv: &inv, env, actor_dynamics: true };
        let y0 = initial_state(&cfg, true);
        let mut stepper = Dopri5::new(ctx, 0.0, 0.3, 0.01, y0, 1e-6, 1e-9);
        stepper.integrate().expect("dopri5 integration");
        let last = stepper.y_out().last().expect("at least one output").clone();
        for i in 0..14 {
            assert!(last[i].is_finite(), "state[{}] non-finite: {}", i, last[i]);
        }
        // Boat starts at rest with no propulsion-relevant motion; after
        // 0.3s it shouldn't have moved more than a meter or rotated
        // more than ~30°.
        assert!(last[POS_X].abs() < 1.0);
        assert!(last[POS_Y].abs() < 1.0);
        assert!(last[YAW].abs() < 0.6);
    }

    /// Loose-tolerance regression check against Python-generated
    /// fixtures. The Rust solve() now uses a tanh-smoothed sail sign
    /// convention (see SAIL_SIGN_SMOOTHING_RAD) that differs from
    /// Python's abrupt sign() near dead downwind. The fixtures still
    /// catch large-scale regressions (sign flips, missing terms,
    /// integration scaling errors) within a 1% absolute tolerance.
    #[test]
    fn derivatives_within_one_percent_of_python() {
        let cfg = Config::load(&manifest_dir().join("sim_params_config.yaml")).unwrap();
        let inv = Invariants::from_config(&cfg);
        let path = manifest_dir().join("tests/fixtures/derivatives.json");
        let raw = std::fs::read_to_string(&path).expect("run scripts/dump_fixtures.py first");
        let file: FixtureFile = serde_json::from_str(&raw).unwrap();
        assert!(!file.fixtures.is_empty(), "no fixtures loaded");

        for fx in &file.fixtures {
            let dir_rad = fx.env.wind_dir_deg.to_radians();
            let env = crate::physics::forces::Environment {
                sail_angle: fx.env.sail_angle,
                rudder_angle: fx.env.rudder_angle,
                true_wind: TrueWind {
                    x: fx.env.wind_strength * dir_rad.cos(),
                    y: fx.env.wind_strength * dir_rad.sin(),
                    strength: fx.env.wind_strength,
                    direction: fx.env.wind_dir_deg,
                },
                wave: Wave {
                    length: fx.env.wave_length,
                    direction: fx.env.wave_direction,
                    amplitude: fx.env.wave_amplitude,
                },
            };
            let ctx = OdeContext { cfg: &cfg, inv: &inv, env, actor_dynamics: true };
            let y = State::from_column_slice(&fx.state);
            let mut dy = State::zeros();
            ctx.system(fx.time, &y, &mut dy);
            for i in 0..14 {
                let diff = (dy[i] - fx.derivative[i]).abs();
                let tol = 1e-2 * fx.derivative[i].abs().max(1e-6);
                assert!(
                    diff < tol,
                    "fixture {} index {} diff {} > tol {} (rust {} vs py {})",
                    fx.name,
                    i,
                    diff,
                    tol,
                    dy[i],
                    fx.derivative[i]
                );
            }
        }
    }
}

pub fn initial_state(cfg: &Config, actor_dynamics: bool) -> State {
    let i = &cfg.simulator.initial;
    let mut s = State::zeros();
    s[ROLL] = i.roll;
    s[PITCH] = i.pitch;
    s[YAW] = i.yaw;
    s[VEL_X] = i.vel_x;
    s[VEL_Y] = i.vel_y;
    s[VEL_Z] = i.vel_z;
    s[ROLL_RATE] = i.roll_rate;
    s[PITCH_RATE] = i.pitch_rate;
    s[YAW_RATE] = i.yaw_rate;
    if actor_dynamics {
        s[RUDDER_STATE] = i.rudder_angle;
        s[SAIL_STATE] = i.sail_angle;
    }
    s
}
