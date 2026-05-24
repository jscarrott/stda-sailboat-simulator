pub mod simulate;

use anyhow::Result;
use std::path::Path;

use crate::autopilot::RouteAutopilot;
use crate::chart::Chart;
use crate::config::{Config, Invariants};
use crate::physics::forces::Environment;
use crate::physics::solve::initial_state;
use crate::route::{Route, WindOverride};
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
/// run the simulator driven by a `RouteAutopilot`, and return the
/// trajectory + route + chart for the plotter.
///
/// `wind_override` (CLI flag) takes precedence over the route YAML's
/// `wind` block; useful for wind-condition sweeps over a single route.
///
/// `max_run_time_s` caps wall-clock-equivalent simulation length.
pub fn scenario_route(
    cfg: &Config,
    route_path: &Path,
    chart_path: Option<&Path>,
    max_run_time_s: f64,
    wind_override: Option<WindOverride>,
) -> Result<RouteRun> {
    let route = Route::load(route_path)?;
    let chart = chart_path.map(Chart::load).transpose()?;

    let inv = Invariants::from_config(cfg);
    let mut env = Environment::from_config(cfg);
    if let Some(w) = wind_override.or(route.wind) {
        env.true_wind = w.to_true_wind();
    }

    let mut autopilot =
        RouteAutopilot::new(cfg, route.clone(), SAMPLE_TIME, SAIL_SAMPLE_TIME);
    let mut x0 = initial_state(cfg, true);
    if let Some(start) = route.waypoints.first() {
        x0[POS_X] = start.x;
        x0[POS_Y] = start.y;
    }
    let n_steps = (max_run_time_s / SAMPLE_TIME) as usize;

    let result = simulate(cfg, &inv, env, &mut autopilot, SAMPLE_TIME, n_steps, x0, true)?;
    Ok(RouteRun { result, route, chart })
}
