pub mod simulate;

use anyhow::Result;
use std::path::Path;

use crate::autopilot::RouteAutopilot;
use crate::chart::Chart;
use crate::config::{Config, Invariants};
use crate::current_model::{CurrentModel, NoCurrent, TidalStream};
use crate::physics::forces::Environment;
use crate::physics::solve::initial_state;
use crate::route::{Route, WindOverride};
use crate::state::{POS_X, POS_Y};
use crate::wind_model::{ConstantWind, OrnsteinUhlenbeckWind, WindModel};

pub use simulate::{simulate, SimResult, Solver};

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

/// CLI knobs for a uniform reversing tidal stream. `peak_speed == 0` →
/// still water (`NoCurrent`).
#[derive(Debug, Clone, Copy)]
pub struct TidalParams {
    pub peak_speed: f64,    // m/s
    pub axis_rad: f64,      // flood direction (math angle)
    pub period_s: f64,
    pub phase_rad: f64,
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
    tide: Option<TidalParams>,
    crab: bool,
    solver: Solver,
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

    // Build the tidal-current model. peak_speed == 0 → still water.
    let mut current_model: Box<dyn CurrentModel> = match tide {
        Some(p) if p.peak_speed != 0.0 => {
            Box::new(TidalStream::new(p.peak_speed, p.axis_rad, p.period_s, p.phase_rad))
        }
        _ => Box::new(NoCurrent),
    };

    let mut autopilot =
        RouteAutopilot::new(cfg, route.clone(), SAMPLE_TIME, SAIL_SAMPLE_TIME, crab);
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
        current_model.as_mut(),
        SAMPLE_TIME,
        n_steps,
        x0,
        true,
        solver,
    )?;
    Ok(RouteRun { result, route, chart })
}

/// Measure the boat's steady-state speed polar: for each true wind
/// angle (TWA, the angle between heading and the wind), hold that
/// heading with a `FixedHeadingAutopilot` in a constant wind and record
/// the mean speed over the last third of the run (once transients have
/// settled). Returns `(twa_deg, speed_mps)` pairs. The calibration
/// instrument for the hull parameters.
pub fn scenario_polar(
    cfg: &Config,
    wind_speed: f64,
    solver: Solver,
) -> Result<Vec<(f64, f64)>> {
    use crate::autopilot::FixedHeadingAutopilot;
    use crate::current_model::NoCurrent;
    use crate::physics::wind::TrueWind;
    use crate::state::{VEL_X, VEL_Y, YAW};
    use crate::wind_model::ConstantWind;
    use std::f64::consts::PI;

    let inv = Invariants::from_config(cfg);
    // Wind blows toward +y (north); it comes FROM the south (270° math).
    let wind_from = 1.5 * PI;
    let true_wind = TrueWind {
        x: 0.0,
        y: wind_speed,
        strength: wind_speed,
        direction: 90.0,
    };
    // Long enough to reach steady state on the slow points of sail.
    let run_time = 600.0;
    let n_steps = (run_time / SAMPLE_TIME) as usize;

    let mut polar = Vec::new();
    let mut twa_deg: f64 = 30.0;
    while twa_deg <= 180.0 + 1e-6 {
        let twa = twa_deg.to_radians();
        let heading = wind_from - twa; // starboard tack
        let mut env = Environment::from_config(cfg);
        env.true_wind = true_wind;

        let mut ap = FixedHeadingAutopilot::new(cfg, heading, SAMPLE_TIME);
        let mut wind = ConstantWind::new(true_wind);
        let mut current = NoCurrent;
        let mut x0 = initial_state(cfg, true);
        // Start pointed the right way with a little way on so the boat
        // doesn't sit in irons while the controller spins it up.
        x0[YAW] = heading;
        x0[VEL_X] = 0.3;

        let result = simulate(
            cfg, &inv, env, &mut ap, &mut wind, &mut current,
            SAMPLE_TIME, n_steps, x0, true, solver,
        )?;

        // Mean speed over the last third (steady state).
        let n = result.x.len();
        let start = n - n / 3;
        let mut sum = 0.0;
        for s in &result.x[start..] {
            sum += (s[VEL_X] * s[VEL_X] + s[VEL_Y] * s[VEL_Y]).sqrt();
        }
        let speed = sum / (n - start) as f64;
        if std::env::var("POLAR_DEBUG").is_ok() {
            let last = &result.x[n - 1];
            let final_yaw = last[YAW].to_degrees().rem_euclid(360.0);
            eprintln!(
                "  [dbg] twa={:.0} target_hdg={:.0} final_hdg={:.0} final_vx={:.2} final_vy={:.2}",
                twa_deg,
                heading.to_degrees().rem_euclid(360.0),
                final_yaw,
                last[VEL_X],
                last[VEL_Y],
            );
        }
        polar.push((twa_deg, speed));
        twa_deg += 15.0;
    }
    Ok(polar)
}
