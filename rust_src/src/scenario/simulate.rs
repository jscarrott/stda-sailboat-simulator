use anyhow::Result;
use ode_solvers::{Dopri5, Rk4};

use crate::autopilot::{Autopilot, Observation, WindReading};
use crate::config::{Config, Invariants};
use crate::current_model::CurrentModel;
use crate::physics::forces::Environment;
use crate::physics::solve::{
    OdeContext, MAX_RUDDER_SPEED, MAX_SAIL_SPEED, RUDDER_RATE, SAIL_RATE,
};
use crate::state::*;
use crate::wind_model::WindModel;

/// Which inner ODE method to use per outer control step.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Solver {
    /// Adaptive Dormand-Prince 5(4). Higher accuracy per step, but
    /// fails out with `StiffnessDetected` on aggressive IOM dynamics.
    Dopri5,
    /// Fixed-step classical RK4. Lower accuracy per step, no stiffness
    /// check — useful when Dopri5 bails on long runs with heavy gusts.
    /// Uses substeps of `RK4_SUBSTEP` seconds within each outer step.
    Rk4,
}

const RK4_SUBSTEP: f64 = 0.01;

#[derive(Debug)]
pub struct SimResult {
    pub t: Vec<f64>,
    pub x: Vec<[f64; N_STATES_ACTUATED]>,
    pub rudder: Vec<f64>,
    pub sail: Vec<f64>,
}

/// Move `current` toward `target` by at most `max_delta`.
fn slew(current: f64, target: f64, max_delta: f64) -> f64 {
    let delta = target - current;
    if delta.abs() <= max_delta {
        target
    } else {
        current + delta.signum() * max_delta
    }
}

/// Run the outer control loop and inner ODE integration.
///
/// Per outer step at `sampletime`:
///   1. Refresh the wind from the `WindModel`.
///   2. Build an `Observation` and ask the `Autopilot` for a `Command`.
///   3. **Slew-limit** the commanded rudder/sail angles to the
///      actuators' mechanical max-rates so the controller can never
///      ask for a step the hardware couldn't deliver.
///   4. **Step the actuator state analytically** as a first-order lag
///      toward the slewed command, using the closed-form
///      `x ← target + (x − target)·exp(−rate·dt)`. This used to live
///      inside the ODE state vector but the fast rudder mode (τ=0.5 s)
///      kept tripping Dopri5's stiffness detector on long runs. Pulling
///      it out makes the remaining 12-DOF physics system non-stiff.
///   5. Integrate the physics for one step with `actor_dynamics=false`
///      so the ODE reads the current actuator state from `env`.
///
/// State slots 12 (RUDDER_STATE) and 13 (SAIL_STATE) are kept in the
/// SimResult record so downstream plotters / fixtures see the same
/// schema, but they are now written by the analytic update rather than
/// by the ODE itself.
pub fn simulate(
    cfg: &Config,
    inv: &Invariants,
    mut env: Environment,
    autopilot: &mut dyn Autopilot,
    wind: &mut dyn WindModel,
    current: &mut dyn CurrentModel,
    sampletime: f64,
    n_steps: usize,
    x0: State,
    _actor_dynamics: bool,
    solver: Solver,
) -> Result<SimResult> {
    let mut result = SimResult {
        t: Vec::with_capacity(n_steps + 1),
        x: Vec::with_capacity(n_steps + 1),
        rudder: Vec::with_capacity(n_steps),
        sail: Vec::with_capacity(n_steps),
    };
    let mut x = x0;
    let mut t = 0.0;
    let to_array = |s: &State| {
        let mut a = [0.0_f64; N_STATES_ACTUATED];
        for i in 0..N_STATES_ACTUATED {
            a[i] = s[i];
        }
        a
    };
    result.t.push(t);
    result.x.push(to_array(&x));

    // Actuator state, tracked externally to the ODE so the integrator
    // never sees the fast first-order rudder mode.
    let mut actuator_rudder = x[RUDDER_STATE];
    let mut actuator_sail = x[SAIL_STATE];
    // Slew-limited command target carried between ticks.
    let mut cmd_rudder = env.rudder_angle;
    let mut cmd_sail = env.sail_angle;

    let max_d_rudder = MAX_RUDDER_SPEED * sampletime;
    let max_d_sail = MAX_SAIL_SPEED * sampletime;
    let alpha_rudder = (-RUDDER_RATE * sampletime).exp();
    let alpha_sail = (-SAIL_RATE * sampletime).exp();

    for _ in 0..n_steps {
        env.true_wind = wind.sample(t, sampletime);
        env.water_current = current.sample(t);

        let obs = Observation {
            t,
            pos_x: x[POS_X],
            pos_y: x[POS_Y],
            heading: x[YAW],
            yaw_rate: x[YAW_RATE],
            roll: x[ROLL],
            vel_x_body: x[VEL_X],
            vel_y_body: x[VEL_Y],
            true_wind: WindReading {
                direction: env.true_wind.y.atan2(env.true_wind.x),
                speed: env.true_wind.strength,
            },
        };

        let raw_cmd = autopilot.step(&obs);
        if raw_cmd.mission_complete {
            break;
        }

        // (1) Slew-limit the new commands.
        cmd_rudder = slew(cmd_rudder, raw_cmd.rudder_angle, max_d_rudder);
        cmd_sail = slew(cmd_sail, raw_cmd.sail_angle, max_d_sail);

        // (2) Analytic first-order actuator step toward the slewed cmd.
        actuator_rudder = cmd_rudder + (actuator_rudder - cmd_rudder) * alpha_rudder;
        actuator_sail = cmd_sail + (actuator_sail - cmd_sail) * alpha_sail;

        // Hand the actuator state to the ODE through env.
        env.rudder_angle = actuator_rudder;
        env.sail_angle = actuator_sail;

        result.rudder.push(actuator_rudder);
        result.sail.push(actuator_sail);

        let ctx = OdeContext { cfg, inv, env, actor_dynamics: false };
        x = match solver {
            Solver::Dopri5 => {
                // Looser tolerances than the 1e-6/1e-9 used during
                // Python parity: the IOM has fast pitch and roll modes
                // (T ~0.25 s, 1.3 s) that Dopri5 can step through but
                // its stiffness detector would flag at tight tolerances.
                let mut stepper = Dopri5::new(ctx, t, t + sampletime, 0.01, x, 1e-4, 1e-7);
                stepper
                    .integrate()
                    .map_err(|e| anyhow::anyhow!("dopri5 step at t={}: {:?}", t, e))?;
                *stepper.y_out().last().expect("dopri5 produces at least one output")
            }
            Solver::Rk4 => {
                // No adaptive step, no stiffness check — just grind
                // through with 30 substeps per outer step. Trades
                // accuracy for robustness on stiff transients.
                let mut stepper = Rk4::new(ctx, t, x, t + sampletime, RK4_SUBSTEP);
                stepper
                    .integrate()
                    .map_err(|e| anyhow::anyhow!("rk4 step at t={}: {:?}", t, e))?;
                *stepper.y_out().last().expect("rk4 produces at least one output")
            }
        };
        t += sampletime;

        // Slots 12/13 are unused with actor_dynamics=false; overwrite
        // them so the SimResult reflects the analytic actuator state.
        x[RUDDER_STATE] = actuator_rudder;
        x[SAIL_STATE] = actuator_sail;

        result.t.push(t);
        result.x.push(to_array(&x));
    }
    Ok(result)
}
