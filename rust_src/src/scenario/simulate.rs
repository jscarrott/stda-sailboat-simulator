use anyhow::Result;
use ode_solvers::Dopri5;

use crate::config::{Config, Invariants};
use crate::controller::HeadingController;
use crate::physics::forces::Environment;
use crate::physics::solve::OdeContext;
use crate::physics::wind::calculate_apparent_wind;
use crate::sail::sail_angle;
use crate::state::*;

#[derive(Debug)]
pub struct SimResult {
    pub t: Vec<f64>,
    pub x: Vec<[f64; N_STATES_ACTUATED]>,
    pub rudder: Vec<f64>,
    pub sail: Vec<f64>,
    pub ref_heading: Vec<f64>,
}

/// Run the outer control loop and inner ODE integration. Mirrors the
/// Python `simulate()` in `run.py:333`. The `heading_provider` closure
/// receives `(t, state)` each control step and returns the desired
/// heading, or `None` to stop early (used by the route follower when
/// the final waypoint is captured).
pub fn simulate(
    cfg: &Config,
    inv: &Invariants,
    mut env: Environment,
    controller: &mut HeadingController,
    sail_sampletime: f64,
    sampletime: f64,
    n_steps: usize,
    x0: State,
    actor_dynamics: bool,
    mut heading_provider: impl FnMut(f64, &State) -> Option<f64>,
) -> Result<SimResult> {
    let mut result = SimResult {
        t: Vec::with_capacity(n_steps + 1),
        x: Vec::with_capacity(n_steps + 1),
        rudder: Vec::with_capacity(n_steps),
        sail: Vec::with_capacity(n_steps),
        ref_heading: Vec::with_capacity(n_steps),
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

    let sail_every = ((sail_sampletime / sampletime).round() as usize).max(1);
    let mut last_sail = env.sail_angle;

    for i in 0..n_steps {
        let speed = (x[VEL_X].powi(2) + x[VEL_Y].powi(2)).sqrt();
        let drift = x[VEL_Y].atan2(x[VEL_X]);

        let desired = match heading_provider(t, &x) {
            Some(h) => h,
            None => break,
        };

        let rudder = controller.control(desired, x[YAW], x[YAW_RATE], speed, x[ROLL], drift);
        env.rudder_angle = rudder;

        if i % sail_every == 0 {
            let apparent = calculate_apparent_wind(x[YAW], x[VEL_X], x[VEL_Y], env.true_wind);
            let new_sail = sail_angle(apparent.angle, apparent.speed, cfg.boat.sail.stretching);
            env.sail_angle = new_sail;
            last_sail = new_sail;
        } else {
            env.sail_angle = last_sail;
        }

        result.rudder.push(rudder);
        result.sail.push(env.sail_angle);
        result.ref_heading.push(desired);

        let ctx = OdeContext { cfg, inv, env, actor_dynamics };
        let mut stepper = Dopri5::new(ctx, t, t + sampletime, 0.01, x, 1e-6, 1e-9);
        stepper.integrate().map_err(|e| anyhow::anyhow!("dopri5 step at t={}: {:?}", t, e))?;
        x = *stepper.y_out().last().expect("dopri5 produces at least one output");
        t += sampletime;

        result.t.push(t);
        result.x.push(to_array(&x));
    }

    Ok(result)
}
