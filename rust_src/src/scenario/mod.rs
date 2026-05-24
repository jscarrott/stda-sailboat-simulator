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
use crate::wind_model::{ConstantWind, OrnsteinUhlenbeckWind, WindModel};

pub use simulate::{simulate, SimResult};

/// CLI knobs for adding Ornstein–Uhlenbeck variance to the route's
/// constant wind. All zero → pass through `ConstantWind`.
#[derive(Debug, Clone, Copy)]
pub struct WindVariance {
    pub speed_sigma: f64,        // m/s
    pub direction_sigma_rad: f64,
    pub correlation_time_s: f64,
    pub seed: u64,
}

impl WindVariance {
    pub fn is_disabled(&self) -> bool {
        self.speed_sigma == 0.0 && self.direction_sigma_rad == 0.0
    }
}

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
    variance: Option<WindVariance>,
) -> Result<RouteRun> {
    let route = Route::load(route_path)?;
    let chart = chart_path.map(Chart::load).transpose()?;

    let inv = Invariants::from_config(cfg);
    let mut env = Environment::from_config(cfg);
    if let Some(w) = wind_override.or(route.wind) {
        env.true_wind = w.to_true_wind();
    }

    // Build the wind model. If variance is disabled (the default), use
    // the constant wind already stored in env; otherwise wrap it in an
    // OU process whose mean matches.
    let mean_speed = env.true_wind.strength;
    let mean_dir = env.true_wind.y.atan2(env.true_wind.x);
    let mut wind_model: Box<dyn WindModel> = match variance {
        Some(v) if !v.is_disabled() => Box::new(OrnsteinUhlenbeckWind::new(
            mean_speed,
            mean_dir,
            v.speed_sigma,
            v.direction_sigma_rad,
            v.correlation_time_s,
            v.seed,
        )),
        _ => Box::new(ConstantWind::new(env.true_wind)),
    };

    let mut autopilot =
        RouteAutopilot::new(cfg, route.clone(), SAMPLE_TIME, SAIL_SAMPLE_TIME);
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
        &mut autopilot,
        wind_model.as_mut(),
        SAMPLE_TIME,
        n_steps,
        x0,
        true,
    )?;
    Ok(RouteRun { result, route, chart })
}
