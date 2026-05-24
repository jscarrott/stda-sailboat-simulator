pub mod simulate;

use anyhow::Result;
use std::path::Path;

use crate::config::{Config, Invariants};
use crate::controller::HeadingController;
use crate::physics::forces::Environment;
use crate::physics::solve::initial_state;
use crate::route::{Route, RouteFollower};
use crate::state::{POS_X, POS_Y};

pub use simulate::{simulate, SimResult};

const SAMPLE_TIME: f64 = 0.3;
const SAIL_SAMPLE_TIME: f64 = 2.0;
const MAX_RUN_TIME: f64 = 900.0;

pub struct RouteRun {
    pub result: SimResult,
    pub route: Route,
}

/// Load a route from `route_path`, run the simulator with a
/// `RouteFollower` driving the heading reference, and return both the
/// trajectory and the loaded route (used by the plotter).
pub fn scenario_route(cfg: &Config, route_path: &Path) -> Result<RouteRun> {
    let route = Route::load(route_path)?;
    let inv = Invariants::from_config(cfg);
    let mut env = Environment::from_config(cfg);
    if let Some(w) = route.wind {
        env.true_wind = w.to_true_wind();
    }
    let true_wind = env.true_wind; // route wind is constant for now

    let mut follower = RouteFollower::new(route.clone());
    let mut controller = HeadingController::new(cfg, SAMPLE_TIME);
    let x0 = initial_state(cfg, true);
    let n_steps = (MAX_RUN_TIME / SAMPLE_TIME) as usize;

    let result = simulate(
        cfg,
        &inv,
        env,
        &mut controller,
        SAIL_SAMPLE_TIME,
        SAMPLE_TIME,
        n_steps,
        x0,
        true,
        |t, state| follower.update(t, state[POS_X], state[POS_Y], true_wind),
    )?;
    Ok(RouteRun { result, route })
}
