use anyhow::Result;
use ode_solvers::Dopri5;

use crate::autopilot::{Autopilot, Observation, WindReading};
use crate::config::{Config, Invariants};
use crate::physics::forces::Environment;
use crate::physics::solve::OdeContext;
use crate::state::*;
use crate::wind_model::WindModel;

#[derive(Debug)]
pub struct SimResult {
    pub t: Vec<f64>,
    pub x: Vec<[f64; N_STATES_ACTUATED]>,
    pub rudder: Vec<f64>,
    pub sail: Vec<f64>,
}

/// Run the outer control loop and inner ODE integration.
///
/// Each outer step at `sampletime` the runner:
///   1. Builds an `Observation` from the current state + environment.
///   2. Calls `autopilot.step(obs)` to get actuator commands.
///   3. Writes the commands into the environment and integrates one
///      step with Dopri5.
///
/// The sim has no knowledge of what kind of autopilot it is — same
/// trait will be driven from real hardware sensors / servos.
pub fn simulate(
    cfg: &Config,
    inv: &Invariants,
    mut env: Environment,
    autopilot: &mut dyn Autopilot,
    wind: &mut dyn WindModel,
    sampletime: f64,
    n_steps: usize,
    x0: State,
    actor_dynamics: bool,
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

    for _ in 0..n_steps {
        // Refresh the wind once per outer step. For constant wind this
        // is a no-op; for Ornstein–Uhlenbeck it integrates a gust step.
        env.true_wind = wind.sample(t, sampletime);

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

        let cmd = autopilot.step(&obs);
        if cmd.mission_complete {
            break;
        }
        env.rudder_angle = cmd.rudder_angle;
        env.sail_angle = cmd.sail_angle;

        result.rudder.push(env.rudder_angle);
        result.sail.push(env.sail_angle);

        let ctx = OdeContext { cfg, inv, env, actor_dynamics };
        let mut stepper = Dopri5::new(ctx, t, t + sampletime, 0.01, x, 1e-6, 1e-9);
        stepper
            .integrate()
            .map_err(|e| anyhow::anyhow!("dopri5 step at t={}: {:?}", t, e))?;
        x = *stepper.y_out().last().expect("dopri5 produces at least one output");
        t += sampletime;

        result.t.push(t);
        result.x.push(to_array(&x));
    }
    Ok(result)
}
