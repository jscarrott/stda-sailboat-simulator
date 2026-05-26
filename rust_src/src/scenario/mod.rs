pub mod simulate;

use anyhow::{Context, Result};
use std::path::Path;

use std::f64::consts::PI;

use crate::autopilot::{Obstacle, ReplanContext, RouteAutopilot};
use crate::chart::Chart;
use crate::config::{Config, Invariants};
use crate::current_model::{CurrentModel, NoCurrent, TabulatedCurrent, TideForecast, TidalStream};
use crate::physics::forces::Environment;
use crate::physics::solve::initial_state;
use crate::planner::{plan, simplify, PlanConfig, Polar};
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
    tide_data: Option<&Path>,
    crab: bool,
    solver: Solver,
    replan_after_wait_s: Option<f64>,
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

    // Build the tidal-current model. A --tide-data cache (real CMEMS
    // time series) wins; else the analytic stream if peak_speed != 0;
    // else still water.
    let mut current_model: Box<dyn CurrentModel> = if let Some(path) = tide_data {
        Box::new(TabulatedCurrent::load(path)?)
    } else {
        match tide {
            Some(p) if p.peak_speed != 0.0 => {
                Box::new(TidalStream::new(p.peak_speed, p.axis_rad, p.period_s, p.phase_rad))
            }
            _ => Box::new(NoCurrent),
        }
    };

    let forecast = current_model.forecaster();
    // Hand the follower a coarse outline of the coastline so it can deflect
    // away from a lee shore the offline route didn't account for (tidal set,
    // wind shifts). Dense chart polylines (Lundy has 3000+ raw vertices) are
    // simplified so the per-tick lookahead test stays cheap.
    let obstacles = chart
        .as_ref()
        .map(build_obstacles)
        .unwrap_or_default();
    let mut autopilot = RouteAutopilot::new(
        cfg,
        route.clone(),
        SAMPLE_TIME,
        SAIL_SAMPLE_TIME,
        crab,
        forecast,
        obstacles,
    );

    // Opt-in mid-mission re-routing on a held tidal gate. Measuring the
    // polar costs a handful of short sims, so it's done only when enabled.
    if let Some(after_wait_s) = replan_after_wait_s {
        let wind_speed = env.true_wind.strength;
        let wind_from =
            (env.true_wind.y.atan2(env.true_wind.x) + PI).rem_euclid(2.0 * PI) - PI;
        let polar = Polar::new(scenario_polar(cfg, wind_speed, solver)?);
        let dest_radius = route.acceptance_radius.max(400.0);
        autopilot.enable_replanning(ReplanContext::new(
            polar,
            chart.clone(),
            wind_from,
            after_wait_s,
            dest_radius,
        ));
    }

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

/// Turn chart polygons into simplified obstacles for the follower's
/// reactive land avoidance. Each polyline is decimated with RDP (50 m
/// tolerance — coarse enough to keep the per-tick lookahead cheap, fine
/// enough to preserve headlands and islets), and degenerate results are
/// dropped.
fn build_obstacles(chart: &Chart) -> Vec<Obstacle> {
    chart
        .polygons
        .iter()
        .filter_map(|poly| {
            let pts: Vec<(f64, f64)> = poly.points.iter().map(|p| (p[0], p[1])).collect();
            if pts.len() < 2 {
                return None;
            }
            let simple = simplify(&pts, 50.0);
            if simple.len() < 2 {
                return None;
            }
            Some(Obstacle::new(simple, poly.closed))
        })
        .collect()
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

/// Build a `TideForecast` for planning from the analytic tide params or
/// a tabulated cache (matching how `scenario_route` builds its current).
fn build_forecast(tide: Option<TidalParams>, tide_data: Option<&Path>) -> Result<TideForecast> {
    use crate::current_model::CurrentModel;
    if let Some(path) = tide_data {
        return Ok(TabulatedCurrent::load(path)?.forecaster());
    }
    Ok(match tide {
        Some(p) if p.peak_speed != 0.0 => {
            TidalStream::new(p.peak_speed, p.axis_rad, p.period_s, p.phase_rad).forecaster()
        }
        _ => TideForecast::None,
    })
}

/// Offline isochrone planning: read start (first waypoint) and
/// destination (last waypoint) from `route_path`, sample the boat polar
/// from `cfg` at the wind speed, build the tide forecast, plan a
/// time-optimal path through wind+tide avoiding chart land, and write
/// it to `out_route` as a fly-by route the follower can sail.
#[allow(clippy::too_many_arguments)]
pub fn scenario_plan(
    cfg: &Config,
    route_path: &Path,
    chart_path: Option<&Path>,
    wind_override: Option<WindOverride>,
    tide: Option<TidalParams>,
    tide_data: Option<&Path>,
    solver: Solver,
    out_route: &Path,
) -> Result<()> {
    let route = Route::load(route_path)?;
    let chart = chart_path.map(Chart::load).transpose()?;
    let wind = wind_override
        .or(route.wind)
        .ok_or_else(|| anyhow::anyhow!("planner needs a wind (route `wind:` block or --wind-*)"))?;
    let tw = wind.to_true_wind();
    let wind_from = (tw.y.atan2(tw.x) + PI).rem_euclid(2.0 * PI) - PI;

    // Sample the model's polar at the planning wind speed.
    let polar = Polar::new(scenario_polar(cfg, wind.speed, solver)?);
    let forecast = build_forecast(tide, tide_data)?;

    let start = (route.waypoints[0].x, route.waypoints[0].y);
    let last = route.waypoints.last().unwrap();
    let dest = (last.x, last.y);

    let pc = PlanConfig {
        start,
        dest,
        wind_from,
        start_time: 0.0,
        dt: 600.0,
        heading_step_deg: 5.0,
        cross_track_bucket_m: 500.0,
        max_steps: 800,
        dest_radius: route.acceptance_radius.max(400.0),
    };
    let result = plan(&pc, &polar, &forecast, chart.as_ref())
        .ok_or_else(|| anyhow::anyhow!("planner could not reach the destination within max_steps"))?;

    // Keep tack apexes (RDP); each emitted leg is then a clean tack the
    // follower sails without adding its own tacking.
    let pts = simplify(&result.path, 250.0);
    write_planned_route(out_route, &route, &wind, &pts)?;
    println!(
        "planned {} -> {}: {:.0} km, ETA {:.1} h ({} path pts, {} waypoints written)",
        out_route.display(),
        route.name,
        result.length_m / 1000.0,
        result.eta_s / 3600.0,
        result.path.len(),
        pts.len(),
    );
    Ok(())
}

fn write_planned_route(
    out: &Path,
    base: &Route,
    wind: &WindOverride,
    pts: &[(f64, f64)],
) -> Result<()> {
    use std::fmt::Write as _;
    let mut s = String::new();
    writeln!(s, "name: {}_planned", base.name).ok();
    writeln!(s, "acceptance_radius: {}", base.acceptance_radius).ok();
    writeln!(s, "close_hauled_angle_deg: {}", base.close_hauled_angle_deg).ok();
    writeln!(s, "xte_lookahead: {}", base.xte_lookahead).ok();
    writeln!(s, "min_tack_duration_s: {}", base.min_tack_duration_s).ok();
    writeln!(s, "fly_by_radius: 400.0").ok();
    writeln!(s, "# Generated by `--scenario plan` (isochrone planner).").ok();
    writeln!(s, "wind: {{ direction_deg: {}, speed: {} }}", wind.direction_deg, wind.speed).ok();
    writeln!(s, "waypoints:").ok();
    let n = pts.len();
    for (i, p) in pts.iter().enumerate() {
        // First and last are hard (start/destination); the rest fly-by.
        let soft = i != 0 && i != n - 1;
        if soft {
            writeln!(s, "  - {{ x: {:.1}, y: {:.1}, soft: true }}", p.0, p.1).ok();
        } else {
            writeln!(s, "  - {{ x: {:.1}, y: {:.1} }}", p.0, p.1).ok();
        }
    }
    writeln!(s, "loop: false").ok();
    std::fs::write(out, s).with_context(|| format!("writing planned route {}", out.display()))?;
    Ok(())
}
