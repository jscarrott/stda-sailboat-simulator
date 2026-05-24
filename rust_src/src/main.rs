// Phase 1 scaffolding: many items below are not yet wired up. Remove this
// allow as soon as Phase 2/3 connect them.
#![allow(dead_code)]

mod config;
mod physics;
mod state;

use anyhow::Result;
use clap::Parser;
use std::path::PathBuf;

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
    println!(
        "scenario={} config={} route={:?}",
        cli.scenario,
        cli.config.display(),
        cli.route
    );
    Ok(())
}
