// Phase 1 scaffolding: many items below are not yet wired up. Remove this
// allow as soon as Phase 2/3 connect them.
#![allow(dead_code)]

mod config;
mod physics;
mod state;

use anyhow::Result;
use clap::Parser;
use std::path::PathBuf;

use config::{Config, Invariants};

#[derive(Parser, Debug)]
#[command(name = "sailboat_sim", version, about = "6-DOF sailboat simulator")]
struct Cli {
    #[arg(long, default_value = "scenario_1")]
    scenario: String,

    #[arg(long, default_value = "sim_params_config.yaml")]
    config: PathBuf,

    #[arg(long)]
    route: Option<PathBuf>,
}

fn main() -> Result<()> {
    let cli = Cli::parse();
    let cfg = Config::load(&cli.config)?;
    let inv = Invariants::from_config(&cfg);

    println!("loaded {}", cli.config.display());
    println!(
        "  boat: mass={} kg, length={} m, sail_area={} m^2",
        cfg.boat.mass, cfg.boat.length, cfg.boat.sail.area
    );
    println!(
        "  env:  water_density={} kg/m^3, gravity={} m/s^2",
        cfg.environment.water_density, cfg.environment.gravity
    );
    println!(
        "  init: wind {} m/s from {} deg",
        cfg.simulator.initial.wind_strength, cfg.simulator.initial.wind_direction
    );
    println!(
        "  invariants: gravity_force={:.4} N, wave_impedance={:.4}",
        inv.gravity_force, inv.wave_impedance
    );
    println!("scenario={} route={:?}", cli.scenario, cli.route);
    Ok(())
}
