// Several YAML fields (latitude/longitude, stepper.{stepsize,clockrate},
// sail/keel/rudder geometry not in the active force model) and a few
// public methods on RouteFollower are kept for future scenarios and
// for debugging — silence the unused warnings rather than littering
// the modules with per-item allows.
#![allow(dead_code)]

mod autopilot;
mod chart;
mod config;
mod controller;
mod current_model;
mod physics;
mod planner;
mod plot;
mod route;
mod sail;
mod scenario;
mod state;
mod wind_model;

use anyhow::{bail, Result};
use clap::Parser;
use std::path::PathBuf;

use config::Config;
use route::WindOverride;
use scenario::{scenario_plan, scenario_polar, scenario_route, Solver, TidalParams, WindVariance};

#[derive(Parser, Debug)]
#[command(name = "sailboat_sim", version, about = "6-DOF sailboat simulator")]
struct Cli {
    /// Which scenario to run.
    #[arg(long, default_value = "route")]
    scenario: String,

    /// Simulator parameter config.
    #[arg(long, default_value = "sim_params_config.yaml")]
    config: PathBuf,

    /// Route YAML (required when --scenario route).
    #[arg(long)]
    route: Option<PathBuf>,

    /// Optional chart JSON for plot overlay (e.g. charts/lundy_coastline.json).
    #[arg(long)]
    chart: Option<PathBuf>,

    /// Maximum simulated seconds before bailing out. Bump for long
    /// routes (Lundy circumnavigation ≈ 16 km → ~8000 s at hull speed).
    #[arg(long, default_value_t = 900.0)]
    max_run_time_s: f64,

    /// Override the wind direction (math convention: angle of the wind
    /// velocity vector in deg — i.e. direction the wind is blowing
    /// TOWARD). Used together with --wind-speed; if either is set the
    /// route YAML's `wind` block is ignored for this run.
    #[arg(long)]
    wind_deg: Option<f64>,

    /// Override the wind speed (m/s).
    #[arg(long)]
    wind_speed: Option<f64>,

    /// Wind speed std-dev for the Ornstein-Uhlenbeck gust model (m/s).
    /// Set together with --wind-time-constant; both default to 0 (constant wind).
    #[arg(long, default_value_t = 0.0)]
    wind_speed_sigma: f64,

    /// Wind direction std-dev for the gust model (degrees).
    #[arg(long, default_value_t = 0.0)]
    wind_dir_sigma_deg: f64,

    /// Correlation time τ for the OU gust process (seconds). Typical
    /// real-wind values: 5-15 s for short gusts, 60-300 s for shifts.
    #[arg(long, default_value_t = 30.0)]
    wind_time_constant: f64,

    /// Seed for the gust RNG (reproducibility).
    #[arg(long, default_value_t = 0xC0FFEE_u64)]
    wind_seed: u64,

    /// Real tidal-current time series (JSON from scripts/fetch_tides.py).
    /// Takes precedence over the analytic --tide-* stream.
    #[arg(long)]
    tide_data: Option<PathBuf>,

    /// Real wind-forecast time series (JSON from scripts/fetch_wind.py —
    /// Open-Meteo / ECMWF IFS). Drives the boat with the cached forecast
    /// instead of constant or Ornstein–Uhlenbeck wind, taking precedence
    /// over the route YAML's `wind:` block and any --wind-deg/--wind-speed.
    #[arg(long)]
    wind_data: Option<PathBuf>,

    /// Inner ODE solver: `dopri5` (adaptive, accurate, may fail with
    /// StiffnessDetected on aggressive IOM dynamics) or `rk4`
    /// (fixed-step, no stiffness check, trades accuracy for robustness).
    #[arg(long, default_value = "dopri5")]
    solver: String,

    /// Peak tidal-stream speed (m/s) for a uniform reversing current.
    /// 0 (default) = still water. Bristol Channel springs near Lundy
    /// run roughly 1.5-2 m/s.
    #[arg(long, default_value_t = 0.0)]
    tide_peak: f64,

    /// Flood direction (compass degrees, the way the flood flows TOWARD).
    /// Only used when --tide-peak > 0.
    #[arg(long, default_value_t = 70.0)]
    tide_flood_deg: f64,

    /// Tidal period in hours (default 12.42 = semidiurnal M2).
    #[arg(long, default_value_t = 12.42)]
    tide_period_h: f64,

    /// Tidal phase at t=0, in degrees of the cycle. 0 = slack water
    /// going to flood; 90 = peak flood; 180 = slack to ebb.
    #[arg(long, default_value_t = 0.0)]
    tide_phase_deg: f64,

    /// Enable experimental tide crab-angle feed-forward (off by
    /// default). The route follower's cross-track term already holds
    /// the rhumb line well for moderate set; crab adds a feed-forward
    /// heading offset to counter the current directly. Note it ignores
    /// the polar, so crabbing toward the wind can cost drive — see the
    /// note in autopilot::route.
    #[arg(long)]
    crab: bool,

    /// Override the output PNG path. Defaults to `figs/route_<name>.png`.
    #[arg(long)]
    out: Option<PathBuf>,

    /// Experimental: re-route on a held tidal gate. If the boat waits at a
    /// gate longer than `--replan-after-wait-s`, re-run the isochrone
    /// planner from the current position+time (tide-aware) and sail the
    /// new path instead of waiting out the missed window. Off by default;
    /// enabling it measures the boat polar up front (a few short sims).
    #[arg(long)]
    replan_on_gate: bool,

    /// Hold-time (s) at a tidal gate before re-routing kicks in (only used
    /// with --replan-on-gate). Default ~ one semidiurnal half-cycle.
    #[arg(long, default_value_t = 3600.0)]
    replan_after_wait_s: f64,

    /// Planning-polar derate factor (`--scenario plan` and the gate
    /// re-router). The measured steady-state polar overstates what the
    /// boat holds through tacks/gusts; scaling it < 1 makes the planner's
    /// ETA realistic and leans it less on outrunning a foul tide. 1.0 =
    /// no change; ~0.8 is a reasonable starting margin.
    #[arg(long, default_value_t = 1.0)]
    polar_derate: f64,

    /// `--scenario plan`: insert tidal gates where the planned path would
    /// otherwise sail a leg against a foul stream, so the written route is
    /// sailed with wait-for-fair discipline instead of being set off. Needs
    /// a tide (--tide-* or --tide-data); off by default.
    #[arg(long)]
    plan_tidal_gates: bool,

    /// `--scenario fleet`: comma-separated config paths to run on the same
    /// route, each as its own track overlaid on one PNG (labelled by boat
    /// length, with finish time in days). Generate sizes with
    /// scripts/scale_hull.py.
    #[arg(long, value_delimiter = ',')]
    fleet_configs: Vec<PathBuf>,

    /// `--scenario hil`: serial port of the nRF52840 running the controller
    /// firmware (e.g. /dev/ttyACM0). The host streams simulated sensors to the
    /// device and applies the rudder/sail commands it replies with. Requires a
    /// build with `--features hil`.
    #[arg(long, default_value = "/dev/ttyACM0")]
    hil_port: String,

    /// `--scenario hil`: true wind angle (deg) to hold during the run.
    #[arg(long, default_value_t = 90.0)]
    hil_twa_deg: f64,
}

fn main() -> Result<()> {
    let cli = Cli::parse();
    let cfg = Config::load(&cli.config)?;

    match cli.scenario.as_str() {
        "route" => {
            let route_path = cli
                .route
                .as_deref()
                .ok_or_else(|| anyhow::anyhow!("--route <path> is required for --scenario route"))?;
            let wind_override = match (cli.wind_deg, cli.wind_speed) {
                (Some(d), Some(s)) => Some(WindOverride { direction_deg: d, speed: s }),
                (None, None) => None,
                _ => bail!("--wind-deg and --wind-speed must be set together"),
            };
            let variance = WindVariance {
                speed_sigma: cli.wind_speed_sigma,
                direction_sigma_rad: cli.wind_dir_sigma_deg.to_radians(),
                correlation_time_s: cli.wind_time_constant,
                seed: cli.wind_seed,
            };
            let solver = match cli.solver.as_str() {
                "dopri5" => Solver::Dopri5,
                "rk4" => Solver::Rk4,
                other => bail!("unknown --solver {other:?}; supported: dopri5, rk4"),
            };
            // Compass bearing (flow TOWARD) → math angle in our east/north
            // frame: theta = 90° − bearing.
            let tide = TidalParams {
                peak_speed: cli.tide_peak,
                axis_rad: (90.0 - cli.tide_flood_deg).to_radians(),
                period_s: cli.tide_period_h * 3600.0,
                phase_rad: cli.tide_phase_deg.to_radians(),
            };
            let run = scenario_route(
                &cfg,
                route_path,
                cli.chart.as_deref(),
                cli.max_run_time_s,
                wind_override,
                Some(variance),
                Some(tide),
                cli.tide_data.as_deref(),
                cli.wind_data.as_deref(),
                cli.crab,
                solver,
                cli.replan_on_gate.then_some(cli.replan_after_wait_s),
                cli.polar_derate,
            )?;
            let out = cli.out.unwrap_or_else(|| {
                PathBuf::from(format!("figs/route_{}.png", run.route.name))
            });
            println!(
                "{}: simulated {:.1} s across {} steps ({} states recorded){}",
                run.route.name,
                run.result.t.last().copied().unwrap_or(0.0),
                run.result.rudder.len(),
                run.result.x.len(),
                if run.chart.is_some() { " + chart overlay" } else { "" },
            );
            plot::plot_trajectory(&run.result, Some(&run.route), run.chart.as_ref(), &out)?;
            println!("wrote {}", out.display());
            report_route_progress(&run);
        }
        "polar" => {
            let wind_speed = cli.wind_speed.unwrap_or(4.0);
            let solver = match cli.solver.as_str() {
                "dopri5" => Solver::Dopri5,
                "rk4" => Solver::Rk4,
                other => bail!("unknown --solver {other:?}; supported: dopri5, rk4"),
            };
            let polar = scenario_polar(&cfg, wind_speed, solver)?;
            println!("speed polar at {:.1} m/s true wind ({} config):", wind_speed, cli.config.display());
            println!("  TWA(deg)  speed(m/s)  speed(kn)  point of sail");
            for (twa, sp) in &polar {
                let pos = match *twa as i32 {
                    t if t <= 50 => "close hauled",
                    t if t <= 80 => "close reach",
                    t if t <= 100 => "beam reach",
                    t if t <= 150 => "broad reach",
                    _ => "run",
                };
                println!("  {:>7.0}  {:>9.2}  {:>8.2}  {}", twa, sp, sp * 1.94384, pos);
            }
            let out = cli.out.unwrap_or_else(|| {
                let stem = cli
                    .config
                    .file_stem()
                    .and_then(|s| s.to_str())
                    .unwrap_or("config");
                PathBuf::from(format!("figs/polar_{}_{:.0}ms.png", stem, wind_speed))
            });
            plot::plot_polar(&polar, wind_speed, &out)?;
            println!("wrote {}", out.display());
        }
        "plan" => {
            let route_path = cli
                .route
                .as_deref()
                .ok_or_else(|| anyhow::anyhow!("--route <path> (start+dest+wind) is required for --scenario plan"))?;
            let wind_override = match (cli.wind_deg, cli.wind_speed) {
                (Some(d), Some(s)) => Some(WindOverride { direction_deg: d, speed: s }),
                (None, None) => None,
                _ => bail!("--wind-deg and --wind-speed must be set together"),
            };
            let tide = TidalParams {
                peak_speed: cli.tide_peak,
                axis_rad: (90.0 - cli.tide_flood_deg).to_radians(),
                period_s: cli.tide_period_h * 3600.0,
                phase_rad: cli.tide_phase_deg.to_radians(),
            };
            let solver = match cli.solver.as_str() {
                "dopri5" => Solver::Dopri5,
                "rk4" => Solver::Rk4,
                other => bail!("unknown --solver {other:?}; supported: dopri5, rk4"),
            };
            let out = cli.out.unwrap_or_else(|| {
                let stem = route_path.file_stem().and_then(|s| s.to_str()).unwrap_or("route");
                PathBuf::from(format!("routes/{}_planned.yaml", stem))
            });
            scenario_plan(
                &cfg,
                route_path,
                cli.chart.as_deref(),
                wind_override,
                Some(tide),
                cli.tide_data.as_deref(),
                solver,
                &out,
                cli.polar_derate,
                cli.plan_tidal_gates,
            )?;
        }
        "fleet" => {
            if cli.fleet_configs.is_empty() {
                bail!("--fleet-configs <a.yaml,b.yaml,...> is required for --scenario fleet");
            }
            let route_path = cli
                .route
                .as_deref()
                .ok_or_else(|| anyhow::anyhow!("--route <path> is required for --scenario fleet"))?;
            let wind_override = match (cli.wind_deg, cli.wind_speed) {
                (Some(d), Some(s)) => Some(WindOverride { direction_deg: d, speed: s }),
                (None, None) => None,
                _ => bail!("--wind-deg and --wind-speed must be set together"),
            };
            let variance = WindVariance {
                speed_sigma: cli.wind_speed_sigma,
                direction_sigma_rad: cli.wind_dir_sigma_deg.to_radians(),
                correlation_time_s: cli.wind_time_constant,
                seed: cli.wind_seed,
            };
            let solver = match cli.solver.as_str() {
                "dopri5" => Solver::Dopri5,
                "rk4" => Solver::Rk4,
                other => bail!("unknown --solver {other:?}; supported: dopri5, rk4"),
            };
            let tide = TidalParams {
                peak_speed: cli.tide_peak,
                axis_rad: (90.0 - cli.tide_flood_deg).to_radians(),
                period_s: cli.tide_period_h * 3600.0,
                phase_rad: cli.tide_phase_deg.to_radians(),
            };

            let mut tracks: Vec<(String, scenario::SimResult)> = Vec::new();
            let mut shared: Option<(route::Route, Option<chart::Chart>)> = None;
            println!("fleet on {} ({} boats):", route_path.display(), cli.fleet_configs.len());
            for cfg_path in &cli.fleet_configs {
                let cfg = Config::load(cfg_path)?;
                let length = cfg.boat.length;
                // Name the track by the config file stem (e.g. "aclass", "2.5m"),
                // dropping a "sim_params_" prefix, so named boats are distinct.
                let stem = cfg_path.file_stem().and_then(|s| s.to_str()).unwrap_or("boat");
                let name = stem.strip_prefix("sim_params_").unwrap_or(stem);
                let run = scenario_route(
                    &cfg,
                    route_path,
                    cli.chart.as_deref(),
                    cli.max_run_time_s,
                    wind_override,
                    Some(variance),
                    Some(tide),
                    cli.tide_data.as_deref(),
                    cli.wind_data.as_deref(),
                    cli.crab,
                    solver,
                    cli.replan_on_gate.then_some(cli.replan_after_wait_s),
                    cli.polar_derate,
                )?;
                let finish_s = finish_time_s(&run.route, &run.result);
                let label = match finish_s {
                    Some(s) => {
                        println!("  {name} ({length:.2} m): finished in {:.2} days ({:.0} s)", s / 86400.0, s);
                        format!("{name} — {:.2} d", s / 86400.0)
                    }
                    None => {
                        println!(
                            "  {name} ({length:.2} m): did not finish within {:.0} s ({:.1} days)",
                            cli.max_run_time_s, cli.max_run_time_s / 86400.0
                        );
                        format!("{name} — DNF")
                    }
                };
                if shared.is_none() {
                    shared = Some((run.route.clone(), run.chart.clone()));
                }
                tracks.push((label, run.result));
            }
            let (route, chart) = shared.expect("at least one fleet config");
            let out = cli.out.unwrap_or_else(|| PathBuf::from("figs/fleet.png"));
            let refs: Vec<(String, &scenario::SimResult)> =
                tracks.iter().map(|(l, r)| (l.clone(), r)).collect();
            plot::plot_fleet(&refs, &route, chart.as_ref(), &out)?;
            println!("wrote {}", out.display());
            // Also emit a self-contained interactive HTML next to the PNG —
            // pan/zoom into Lundy detail, hover any track for time/speed/trim.
            // If a wind forecast was used, include it as an inset so loops
            // can be correlated with the wind series.
            let html_out = out.with_extension("html");
            let title = match chart.as_ref() {
                Some(c) => format!("Fleet: {} on {}", route.name, c.name),
                None => format!("Fleet: {}", route.name),
            };
            let wind_series = cli.wind_data.as_deref().map(plot::load_wind_series).transpose()?;
            let tide_series = cli.tide_data.as_deref().map(plot::load_tide_series).transpose()?;
            plot::write_fleet_html(
                &refs, &route, chart.as_ref(),
                wind_series.as_ref(), tide_series.as_ref(),
                &title, &html_out,
            )?;
            println!("wrote {}", html_out.display());
        }
        "hil" => {
            #[cfg(feature = "hil")]
            {
                let wind_speed = cli.wind_speed.unwrap_or(4.0);
                let solver = match cli.solver.as_str() {
                    "dopri5" => Solver::Dopri5,
                    "rk4" => Solver::Rk4,
                    other => bail!("unknown --solver {other:?}; supported: dopri5, rk4"),
                };
                let run_time = cli.max_run_time_s.min(120.0);
                println!(
                    "HIL: holding TWA {:.0}° in {:.1} m/s wind via controller on {} for {:.0} s",
                    cli.hil_twa_deg, wind_speed, cli.hil_port, run_time
                );
                let (result, heading) = scenario::scenario_hil(
                    &cfg, wind_speed, cli.hil_twa_deg, run_time, solver, &cli.hil_port,
                )?;
                report_hil_run(&result, heading);
            }
            #[cfg(not(feature = "hil"))]
            {
                let _ = (&cli.hil_port, cli.hil_twa_deg);
                bail!("the `hil` scenario needs the serial link; rebuild with `cargo run --features hil -- --scenario hil ...`");
            }
        }
        other => bail!("unknown scenario {other:?}; supported: route, polar, plan, fleet, hil"),
    }
    Ok(())
}

/// Summarise a hardware-in-the-loop run: steady-state speed (mean over the last
/// third) plus a short downsampled trace, so the device-driven track can be
/// eyeballed against an in-process `polar`/fixed-heading run.
#[cfg(feature = "hil")]
fn report_hil_run(result: &scenario::SimResult, heading: f64) {
    use state::{VEL_X, VEL_Y, YAW};
    let n = result.x.len();
    if n < 2 {
        println!("HIL: no trajectory recorded (device did not respond?)");
        return;
    }
    let start = n - n / 3;
    let mut sum = 0.0;
    for s in &result.x[start..] {
        sum += (s[VEL_X] * s[VEL_X] + s[VEL_Y] * s[VEL_Y]).sqrt();
    }
    let speed = sum / (n - start) as f64;
    println!(
        "HIL: target heading {:.0}°, steady speed {:.2} m/s ({:.2} kn) over {} steps",
        heading.to_degrees().rem_euclid(360.0),
        speed,
        speed * 1.94384,
        result.rudder.len(),
    );
    println!("  t,heading_deg,speed,sail_deg,rudder_deg");
    for k in 0..=10 {
        let i = (k * (n - 1)) / 10;
        let s = &result.x[i];
        let sp = (s[VEL_X] * s[VEL_X] + s[VEL_Y] * s[VEL_Y]).sqrt();
        let si = i.min(result.sail.len().saturating_sub(1));
        println!(
            "  {:.0},{:.0},{:.2},{:.0},{:.0}",
            result.t[i],
            s[YAW].to_degrees().rem_euclid(360.0),
            sp,
            result.sail[si].to_degrees(),
            result.rudder[si].to_degrees(),
        );
    }
}

/// Time (s) the boat captures the final waypoint, or `None` if it never
/// reaches the destination within the run. The simulator breaks the loop on
/// mission-complete, so when finished the last track point sits inside the
/// destination's acceptance radius and the final timestamp is the finish time.
fn finish_time_s(route: &route::Route, result: &scenario::SimResult) -> Option<f64> {
    use state::{POS_X, POS_Y};
    let dest = route.waypoints.last()?;
    let last = result.x.last()?;
    let d = ((last[POS_X] - dest.x).powi(2) + (last[POS_Y] - dest.y).powi(2)).sqrt();
    (d <= route.acceptance_radius).then(|| result.t.last().copied()).flatten()
}

/// Post-run report: how far along the route the boat actually got, how
/// fast it sailed, and the closest approach to each waypoint. Helps
/// distinguish "ran out of time", "couldn't point upwind", and "missed
/// the acceptance radius" failure modes.
fn report_route_progress(run: &scenario::RouteRun) {
    use state::{POS_X, POS_Y, VEL_X, VEL_Y};
    let track = &run.result.x;
    if track.len() < 2 {
        return;
    }

    // Path length + speed stats.
    let mut path_len = 0.0;
    let mut max_speed: f64 = 0.0;
    let mut speed_sum = 0.0;
    for i in 0..track.len() {
        let s = &track[i];
        let speed = (s[VEL_X] * s[VEL_X] + s[VEL_Y] * s[VEL_Y]).sqrt();
        max_speed = max_speed.max(speed);
        speed_sum += speed;
        if i > 0 {
            let p = &track[i - 1];
            path_len += ((s[POS_X] - p[POS_X]).powi(2) + (s[POS_Y] - p[POS_Y]).powi(2)).sqrt();
        }
    }
    let mean_speed = speed_sum / track.len() as f64;

    // Closest approach to each waypoint (in sequence, so a later
    // waypoint's scan starts from where the previous one was captured).
    let accept = run.route.acceptance_radius;
    let fly_by = run.route.fly_by_radius;
    let mut captured = 0;
    let mut scan_from = 0usize;
    println!("route progress (acceptance radius {:.0} m):", accept);
    for (wi, wp) in run.route.waypoints.iter().enumerate().skip(1) {
        // Soft waypoints are "captured" by passing within the larger
        // fly-by radius, not the tight acceptance circle.
        let r = if wp.soft { accept.max(fly_by) } else { accept };
        let mut closest = f64::INFINITY;
        let mut capture_idx = None;
        for i in scan_from..track.len() {
            let s = &track[i];
            let d = ((s[POS_X] - wp.x).powi(2) + (s[POS_Y] - wp.y).powi(2)).sqrt();
            closest = closest.min(d);
            if d < r && capture_idx.is_none() {
                capture_idx = Some(i);
            }
        }
        match capture_idx {
            Some(i) => {
                captured += 1;
                scan_from = i;
                println!(
                    "  wp{} ({:.0},{:.0}): captured at t={:.0}s (closest {:.1} m)",
                    wi, wp.x, wp.y, run.result.t[i], closest
                );
            }
            None => {
                println!(
                    "  wp{} ({:.0},{:.0}): MISSED (closest {:.1} m)",
                    wi, wp.x, wp.y, closest
                );
            }
        }
    }
    println!(
        "  {}/{} waypoints, path {:.0} m, mean {:.2} m/s, max {:.2} m/s",
        captured,
        run.route.waypoints.len() - 1,
        path_len,
        mean_speed,
        max_speed
    );

    // Land-incursion check: a verification counter for the follower's
    // reactive land avoidance — flags any track points that fall inside a
    // closed chart polygon (land). Split into the Lundy region (the lee
    // shore the avoidance targets) and everything else.
    if let Some(chart) = &run.chart {
        let mut near_lundy = 0usize; // x < 2000 (the island region)
        let mut elsewhere = 0usize;
        for s in track.iter() {
            for poly in &chart.polygons {
                if poly.closed && point_in_polygon(s[POS_X], s[POS_Y], &poly.points) {
                    if s[POS_X] < 2000.0 {
                        near_lundy += 1;
                    } else {
                        elsewhere += 1;
                    }
                    break;
                }
            }
        }
        let total = near_lundy + elsewhere;
        if total > 0 {
            println!(
                "  !! LAND INCURSION: {} track pts inside land ({} near Lundy, {} channel/islets)",
                total, near_lundy, elsewhere
            );
        } else {
            println!("  no land incursions (closed polygons; mainland is an open polyline, not checked)");
        }
    }

    if std::env::var("DUMP_TRACK").is_ok() {
        use state::YAW;
        let n = track.len();
        println!("  t,x,y,yaw_deg,speed,sail_deg,rudder_deg");
        for k in 0..=20 {
            let i = (k * (n - 1)) / 20;
            let s = &track[i];
            let speed = (s[VEL_X] * s[VEL_X] + s[VEL_Y] * s[VEL_Y]).sqrt();
            // result.sail/rudder have one entry per step (n-1 total).
            let si = i.min(run.result.sail.len().saturating_sub(1));
            println!(
                "  {:.0},{:.1},{:.1},{:.0},{:.2},{:.0},{:.0}",
                run.result.t[i],
                s[POS_X],
                s[POS_Y],
                s[YAW].to_degrees().rem_euclid(360.0),
                speed,
                run.result.sail[si].to_degrees(),
                run.result.rudder[si].to_degrees(),
            );
        }
    }
}

/// Even-odd ray-casting point-in-polygon test (polygon as [x,y] vertices).
fn point_in_polygon(px: f64, py: f64, pts: &[[f64; 2]]) -> bool {
    let n = pts.len();
    if n < 3 {
        return false;
    }
    let mut inside = false;
    let mut j = n - 1;
    for i in 0..n {
        let (xi, yi) = (pts[i][0], pts[i][1]);
        let (xj, yj) = (pts[j][0], pts[j][1]);
        if ((yi > py) != (yj > py)) && (px < (xj - xi) * (py - yi) / (yj - yi) + xi) {
            inside = !inside;
        }
        j = i;
    }
    inside
}
