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
use crate::planner::{plan, simplify, simplify_idx, PlanConfig, Polar};
use crate::route::{Route, WindOverride};
use crate::state::{POS_X, POS_Y};
use crate::wind_model::{ConstantWind, OrnsteinUhlenbeckWind, TabulatedWind, WindModel};

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
    wind_data: Option<&Path>,
    crab: bool,
    solver: Solver,
    replan_after_wait_s: Option<f64>,
    polar_derate: f64,
) -> Result<RouteRun> {
    let route = Route::load(route_path)?;
    let chart = chart_path.map(Chart::load).transpose()?;

    let inv = Invariants::from_config(cfg);
    let mut env = Environment::from_config(cfg);
    if let Some(w) = wind_override.or(route.wind) {
        env.true_wind = w.to_true_wind();
    }

    // Build the wind model. Precedence: a --wind-data cache (real Open-Meteo
    // forecast) wins, since real wind beats any stochastic placeholder; else
    // OU if variance is configured; else the constant already stored in env.
    let mut wind_model: Box<dyn WindModel> = if let Some(path) = wind_data {
        let mut tw = TabulatedWind::load(path)?;
        // Seed env.true_wind from the forecast at t=0 so the first step has
        // the right wind even before simulate() ticks the model.
        env.true_wind = tw.sample(0.0, 0.0);
        Box::new(tw)
    } else {
        let mean_speed = env.true_wind.strength;
        let mean_dir = env.true_wind.y.atan2(env.true_wind.x);
        match variance {
            Some(v) if !v.is_disabled() => Box::new(OrnsteinUhlenbeckWind::new(
                mean_speed,
                mean_dir,
                v.speed_sigma,
                v.direction_sigma_rad,
                v.correlation_time_s,
                v.seed,
            )),
            _ => Box::new(ConstantWind::new(env.true_wind)),
        }
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
        let polar = Polar::new(scenario_polar(cfg, wind_speed, solver)?).derate(polar_derate);
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
    polar_derate: f64,
    tidal_gates: bool,
) -> Result<()> {
    let route = Route::load(route_path)?;
    let chart = chart_path.map(Chart::load).transpose()?;
    let wind = wind_override
        .or(route.wind)
        .ok_or_else(|| anyhow::anyhow!("planner needs a wind (route `wind:` block or --wind-*)"))?;
    let tw = wind.to_true_wind();
    let wind_from = (tw.y.atan2(tw.x) + PI).rem_euclid(2.0 * PI) - PI;

    // Sample the model's polar at the planning wind speed, derated for margin.
    let polar = Polar::new(scenario_polar(cfg, wind.speed, solver)?).derate(polar_derate);
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
    // follower sails without adding its own tacking. Carry the planner's
    // arrival times through the simplification so we can decide gates.
    let idxs = simplify_idx(&result.path, 250.0);
    let mut pts: Vec<(f64, f64)> = idxs.iter().map(|&i| result.path[i]).collect();
    let mut times: Vec<f64> = idxs.iter().map(|&i| result.times[i]).collect();

    // When gating, split any leg longer than a single fair-tide window can
    // sail into sub-legs, with a gate between. A long leg committed in one
    // go turns foul mid-way and sets the boat back; short sub-legs each fit
    // a fair phase, so the boat rides through several gates while the tide
    // is fair and only holds when it turns foul. `GATE_MAX_LEG_M` is sized
    // so a sub-leg sails well inside a semidiurnal fair phase.
    const GATE_MAX_LEG_M: f64 = 5000.0;
    const GATE_MIN_LEG_M: f64 = 500.0;
    // Hold only when the stream is actively foul; open as soon as it's
    // non-foul, so cross-tide legs (along-current ~0) don't lock up.
    const GATE_OPEN_ALONG: f64 = 0.0;
    // Only gate a leg when the foul stream is strong enough to stop or
    // reverse the boat. As long as the foul is weaker than the boat's own
    // speed it still makes net headway, so sailing through beats holding
    // (which parks it for the whole foul phase and exposes it to any
    // cross-set) — confirmed by ungated-vs-gated A/B runs. Scale the
    // threshold off the planner's mean speed (length / ETA) so it tracks
    // the boat and conditions rather than a fixed number; gate just below
    // the speed where along-track progress would collapse.
    let nominal_speed = if result.eta_s > 0.0 {
        result.length_m / result.eta_s
    } else {
        0.6
    };
    let gate_foul_margin = (0.9 * nominal_speed).clamp(0.25, 1.2);

    // Raise the minimum tack duration for long legs so the boat commits to
    // one or two long tacks on a close-hauled passage instead of chattering
    // tacks at the no-go boundary (which stalls it). Short routes keep their
    // base value via the lower clamp. The XTE lookahead is *not* scaled
    // here: the follower adapts it per-leg, so the long open-water legs foot
    // for speed while the short approach legs still track the line and clear
    // nearby land (a route-level value loose enough for the crossing would
    // plow the boat through the islands on the final approach).
    let longest_leg = pts
        .windows(2)
        .map(|w| (w[1].0 - w[0].0).hypot(w[1].1 - w[0].1))
        .fold(0.0_f64, f64::max);
    let min_tack_s = (0.05 * longest_leg / nominal_speed.max(0.1))
        .clamp(route.min_tack_duration_s, route.min_tack_duration_s.max(1800.0));

    if tidal_gates {
        let (dp, dt) = densify_legs(&pts, &times, GATE_MAX_LEG_M);
        pts = dp;
        times = dt;
    }

    let gates = if tidal_gates {
        crate::planner::tidal_gate_flags(&pts, &times, &forecast, gate_foul_margin, GATE_MIN_LEG_M)
    } else {
        vec![false; pts.len()]
    };
    let n_gates = gates.iter().filter(|&&g| g).count();

    // A gate must see a fair window long enough to sail its leg before the
    // tide turns; size it from the longest gated leg at a conservative
    // through-water + fair-tide speed.
    let longest_gated_leg = pts
        .windows(2)
        .zip(&gates)
        .filter(|(_, &g)| g)
        .map(|(w, _)| (w[1].0 - w[0].0).hypot(w[1].1 - w[0].1))
        .fold(0.0_f64, f64::max);
    let gate_window_s = (longest_gated_leg / 0.6).clamp(600.0, 9000.0);

    write_planned_route(
        out_route,
        &route,
        &wind,
        &pts,
        &gates,
        GATE_OPEN_ALONG,
        gate_window_s,
        min_tack_s,
    )?;
    println!(
        "planned {} -> {}: {:.0} km, ETA {:.1} h ({} path pts, {} waypoints, {} tidal gates)",
        out_route.display(),
        route.name,
        result.length_m / 1000.0,
        result.eta_s / 3600.0,
        result.path.len(),
        pts.len(),
        n_gates,
    );
    if tidal_gates && n_gates == 0 {
        println!(
            "  (no gates: along-leg foul never exceeds {:.2} m/s, the threshold for this \
             boat's ~{:.2} m/s speed — the tide is too weak or too cross-track to be worth \
             holding for; sailing straight through is faster)",
            gate_foul_margin, nominal_speed,
        );
    }
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn write_planned_route(
    out: &Path,
    base: &Route,
    wind: &WindOverride,
    pts: &[(f64, f64)],
    gates: &[bool],
    gate_threshold: f64,
    gate_window_s: f64,
    min_tack_s: f64,
) -> Result<()> {
    use std::fmt::Write as _;
    let mut s = String::new();
    writeln!(s, "name: {}_planned", base.name).ok();
    writeln!(s, "acceptance_radius: {}", base.acceptance_radius).ok();
    writeln!(s, "close_hauled_angle_deg: {}", base.close_hauled_angle_deg).ok();
    writeln!(s, "xte_lookahead: {}", base.xte_lookahead).ok();
    writeln!(s, "min_tack_duration_s: {:.0}", min_tack_s).ok();
    writeln!(s, "fly_by_radius: 400.0").ok();
    writeln!(s, "# Generated by `--scenario plan` (isochrone planner).").ok();
    if min_tack_s > base.min_tack_duration_s {
        writeln!(
            s,
            "# Minimum tack raised for long legs: commit to long tacks. The follower also \
             loosens XTE lookahead per-leg so long legs foot for speed."
        )
        .ok();
    }
    if gates.iter().any(|&g| g) {
        writeln!(s, "# Tidal gates inserted where a leg would be sailed against a foul stream.").ok();
        writeln!(s, "gate_open_along_current: {}", gate_threshold).ok();
        writeln!(s, "gate_min_fair_window_s: {:.0}", gate_window_s).ok();
    }
    writeln!(s, "wind: {{ direction_deg: {}, speed: {} }}", wind.direction_deg, wind.speed).ok();
    writeln!(s, "waypoints:").ok();
    let n = pts.len();
    for (i, p) in pts.iter().enumerate() {
        // First and last are hard (start/destination); the rest fly-by.
        let soft = i != 0 && i != n - 1;
        let gate = gates.get(i).copied().unwrap_or(false);
        let mut fields = format!("x: {:.1}, y: {:.1}", p.0, p.1);
        if soft {
            fields.push_str(", soft: true");
        }
        if gate {
            fields.push_str(", gate: true");
        }
        writeln!(s, "  - {{ {} }}", fields).ok();
    }
    writeln!(s, "loop: false").ok();
    std::fs::write(out, s).with_context(|| format!("writing planned route {}", out.display()))?;
    Ok(())
}

/// Split any leg longer than `max_len` into equal sub-legs, linearly
/// interpolating the planner arrival time across the inserted points.
/// Keeps every original point; only adds points on over-long legs.
fn densify_legs(pts: &[(f64, f64)], times: &[f64], max_len: f64) -> (Vec<(f64, f64)>, Vec<f64>) {
    let mut dp: Vec<(f64, f64)> = Vec::with_capacity(pts.len());
    let mut dt: Vec<f64> = Vec::with_capacity(pts.len());
    if pts.is_empty() {
        return (dp, dt);
    }
    for i in 0..pts.len() - 1 {
        let (ax, ay) = pts[i];
        let (bx, by) = pts[i + 1];
        dp.push(pts[i]);
        dt.push(times[i]);
        let len = (bx - ax).hypot(by - ay);
        if len > max_len {
            let n = (len / max_len).ceil() as usize;
            for k in 1..n {
                let f = k as f64 / n as f64;
                dp.push((ax + (bx - ax) * f, ay + (by - ay) * f));
                dt.push(times[i] + (times[i + 1] - times[i]) * f);
            }
        }
    }
    dp.push(*pts.last().unwrap());
    dt.push(*times.last().unwrap());
    (dp, dt)
}
