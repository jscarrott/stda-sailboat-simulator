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
mod physics;
mod plot;
mod route;
mod sail;
mod scenario;
mod state;

use anyhow::{bail, Result};
use clap::Parser;
use std::path::PathBuf;

use config::Config;
use route::WindOverride;
use scenario::scenario_route;

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
            let run = scenario_route(
                &cfg,
                route_path,
                cli.chart.as_deref(),
                cli.max_run_time_s,
                wind_override,
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
        }
        other => bail!("unknown scenario {other:?}; supported: route"),
    }
    Ok(())
}
