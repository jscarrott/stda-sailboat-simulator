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
use scenario::{scenario_route, Solver, TidalParams, WindVariance};

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

    /// Override the output PNG path. Defaults to `figs/route_<name>.png`.
    #[arg(long)]
    out: Option<PathBuf>,
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
                solver,
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
        other => bail!("unknown scenario {other:?}; supported: route"),
    }
    Ok(())
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
    let r = run.route.acceptance_radius;
    let mut captured = 0;
    let mut scan_from = 0usize;
    println!("route progress (acceptance radius {:.0} m):", r);
    for (wi, wp) in run.route.waypoints.iter().enumerate().skip(1) {
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
