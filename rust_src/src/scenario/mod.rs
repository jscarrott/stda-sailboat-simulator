pub mod simulate;

use anyhow::Result;
use std::path::Path;

use crate::chart::Chart;
use crate::config::{Config, Invariants};
use crate::controller::HeadingController;
use crate::physics::forces::Environment;
use crate::physics::solve::initial_state;
use crate::route::{Route, RouteFollower};
use crate::state::{POS_X, POS_Y};

pub use simulate::{simulate, SimResult};

const SAMPLE_TIME: f64 = 0.3;
const SAIL_SAMPLE_TIME: f64 = 2.0;

pub struct RouteRun {
    pub result: SimResult,
    pub route: Route,
    pub chart: Option<Chart>,
}

/// Load a route from `route_path`, optionally load a chart for overlay,
/// run the simulator with a `RouteFollower` driving the heading
/// reference, and return the trajectory + route + chart for the plotter.
///
/// `max_run_time_s` caps wall-clock-equivalent simulation length.
/// Increase for large routes (Lundy circumnavigation is ~16 km).
pub fn scenario_route(
    cfg: &Config,
    route_path: &Path,
    chart_path: Option<&Path>,
    max_run_time_s: f64,
) -> Result<RouteRun> {
    let route = Route::load(route_path)?;
    let chart = chart_path.map(Chart::load).transpose()?;

    let inv = Invariants::from_config(cfg);
    let mut env = Environment::from_config(cfg);
    if let Some(w) = route.wind {
        env.true_wind = w.to_true_wind();
    }
    let true_wind = env.true_wind;

    let mut follower = RouteFollower::new(route.clone());
    let mut controller = HeadingController::new(cfg, SAMPLE_TIME);
    let mut x0 = initial_state(cfg, true);
    if let Some(start) = route.waypoints.first() {
        x0[POS_X] = start.x;
        x0[POS_Y] = start.y;
    }
    let n_steps = (max_run_time_s / SAMPLE_TIME) as usize;

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
    Ok(RouteRun { result, route, chart })
}
