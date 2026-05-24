use anyhow::{Context, Result};
use serde::Deserialize;
use std::fs::File;
use std::path::Path;

#[derive(Deserialize, Debug)]
pub struct Config {
    pub boat: BoatCfg,
    pub environment: EnvCfg,
    pub simulator: SimCfg,
    /// LQR-derived heading controller gains. Optional; if absent the
    /// controller falls back to the 4 m hull's hand-tuned defaults
    /// (kp=0.5, ki=0.1, kd=0.9). Regenerate with
    /// `scripts/compute_controller_gains.py <this-yaml>` whenever
    /// yaw_timeconstant or the Q/r weights change.
    #[serde(default)]
    pub controller_gains: Option<ControllerGains>,
}

#[derive(Deserialize, Debug, Clone, Copy)]
pub struct ControllerGains {
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
}

#[derive(Deserialize, Debug)]
pub struct BoatCfg {
    pub sail: SailCfg,
    pub rudder: RudderCfg,
    pub keel: KeelCfg,
    pub length: f64,
    pub mass: f64,
    pub height_bouyancy: f64,
    pub lateral_area: f64,
    pub waterline_area: f64,
    pub distance_cog_sail_pressure_point: f64,
    pub distance_cog_keel_pressure_point: f64,
    pub distance_cog_rudder: f64,
    pub distance_mast_sail_pressure_point: f64,
    pub geometrical_moi_x: f64,
    pub geometrical_moi_y: f64,
    pub moi_x: f64,
    pub moi_y: f64,
    pub moi_z: f64,
    pub roll_damping: f64,
    pub pitch_damping: f64,
    pub damping_z: f64,
    pub yaw_timeconstant: f64,
    pub along_damping: f64,
    pub transverse_damping: f64,
    pub hull_speed: f64,
}

#[derive(Deserialize, Debug)]
pub struct SailCfg {
    pub pressure_point_height: f64,
    pub height: f64,
    pub area: f64,
    pub length: f64,
    pub stretching: f64,
}

#[derive(Deserialize, Debug)]
pub struct RudderCfg {
    pub stretching: f64,
    pub area: f64,
}

#[derive(Deserialize, Debug)]
pub struct KeelCfg {
    pub height: f64,
    pub length: f64,
    pub stretching: f64,
}

#[derive(Deserialize, Debug)]
pub struct EnvCfg {
    pub water_viscosity: f64,
    pub air_viscosity: f64,
    pub water_density: f64,
    pub air_density: f64,
    pub gravity: f64,
}

#[derive(Deserialize, Debug)]
pub struct SimCfg {
    pub stepper: StepperCfg,
    pub initial: InitialCfg,
}

#[derive(Deserialize, Debug)]
pub struct StepperCfg {
    pub stepsize: f64,
    pub clockrate: f64,
}

#[derive(Deserialize, Debug)]
pub struct InitialCfg {
    pub vel_x: f64,
    pub vel_y: f64,
    pub vel_z: f64,
    pub yaw: f64,
    pub pitch: f64,
    pub roll: f64,
    pub roll_rate: f64,
    pub pitch_rate: f64,
    pub yaw_rate: f64,
    pub latitude: f64,
    pub longitude: f64,
    pub wind_strength: f64,
    pub wind_direction: f64,
    pub wave_direction: f64,
    pub wave_length: f64,
    pub wave_amplitude: f64,
    pub sail_angle: f64,
    pub rudder_angle: f64,
}

impl Config {
    pub fn load(path: &Path) -> Result<Self> {
        let file = File::open(path)
            .with_context(|| format!("opening config file {}", path.display()))?;
        let cfg: Config = serde_yaml::from_reader(file)
            .with_context(|| format!("parsing config file {}", path.display()))?;
        Ok(cfg)
    }
}

/// Pre-computed scalars that depend only on `Config`. Mirrors
/// `simulation.py:80-97`. Computed once and passed by reference into the
/// ODE RHS to keep `solve()` cheap.
#[derive(Debug, Clone, Copy)]
pub struct Invariants {
    pub wave_impedance: f64,
    pub hydrostatic_eff_x: f64,
    pub hydrostatic_eff_y: f64,
    pub hydrostatic_invariant_z: f64,
    pub gravity_force: f64,
    pub damping_invariant_x: f64,
    pub damping_invariant_y: f64,
    pub damping_invariant_z: f64,
    pub damping_invariant_yaw: f64,
    pub damping_invariant_pitch: f64,
    pub damping_invariant_roll: f64,
}

impl Invariants {
    pub fn from_config(cfg: &Config) -> Self {
        let b = &cfg.boat;
        let e = &cfg.environment;
        let wave_impedance = (e.water_density / 2.0) * b.lateral_area;
        let hydrostatic_eff_x =
            b.height_bouyancy + (e.water_density / b.mass) * b.geometrical_moi_x;
        let hydrostatic_eff_y =
            b.height_bouyancy + (e.water_density / b.mass) * b.geometrical_moi_y;
        let hydrostatic_invariant_z = -e.water_density * b.waterline_area * e.gravity;
        let gravity_force = b.mass * e.gravity;
        let damping_invariant_x = -b.mass / b.along_damping;
        let damping_invariant_y = -b.mass / b.transverse_damping;
        let damping_invariant_z = -0.5
            * b.damping_z
            * (e.water_density * b.waterline_area * e.gravity * b.mass).sqrt();
        let damping_invariant_yaw = -(b.moi_z / b.yaw_timeconstant);
        let damping_invariant_pitch =
            -2.0 * b.pitch_damping * (b.moi_y * b.mass * e.gravity * hydrostatic_eff_y).sqrt();
        let damping_invariant_roll =
            -2.0 * b.roll_damping * (b.moi_x * b.mass * e.gravity * hydrostatic_eff_x).sqrt();
        Self {
            wave_impedance,
            hydrostatic_eff_x,
            hydrostatic_eff_y,
            hydrostatic_invariant_z,
            gravity_force,
            damping_invariant_x,
            damping_invariant_y,
            damping_invariant_z,
            damping_invariant_yaw,
            damping_invariant_pitch,
            damping_invariant_roll,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::PathBuf;

    fn yaml_path() -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("sim_params_config.yaml")
    }

    #[test]
    fn loads_yaml() {
        let cfg = Config::load(&yaml_path()).expect("config should load");
        assert_eq!(cfg.boat.mass, 350.0);
        assert_eq!(cfg.boat.length, 4.0);
        assert_eq!(cfg.boat.sail.area, 6.4);
        assert_eq!(cfg.boat.rudder.area, 0.13);
        assert_eq!(cfg.boat.keel.length, 2.0);
        assert_eq!(cfg.environment.water_density, 1000.0);
        assert_eq!(cfg.environment.gravity, 9.81);
        assert_eq!(cfg.simulator.stepper.stepsize, 0.1);
        assert_eq!(cfg.simulator.initial.wind_direction, 45.0);
        assert_eq!(cfg.simulator.initial.sail_angle, 1.0);
    }

    #[test]
    fn invariants_match_python() {
        let cfg = Config::load(&yaml_path()).expect("config should load");
        let inv = Invariants::from_config(&cfg);
        // Values reproduced by running simulation.py module-level
        // expressions with the shipped YAML.
        assert!((inv.wave_impedance - 1250.0).abs() < 1e-9);
        assert!((inv.gravity_force - 3433.5).abs() < 1e-9);
        assert!((inv.damping_invariant_x - (-350.0 / 15.0)).abs() < 1e-9);
        assert!((inv.damping_invariant_yaw - (-1066.0 / 5.0)).abs() < 1e-9);
        // Spot-check the sqrt-bearing invariants stay finite and signed.
        assert!(inv.damping_invariant_z < 0.0 && inv.damping_invariant_z.is_finite());
        assert!(inv.damping_invariant_pitch < 0.0 && inv.damping_invariant_pitch.is_finite());
        assert!(inv.damping_invariant_roll < 0.0 && inv.damping_invariant_roll.is_finite());
        assert!(inv.hydrostatic_eff_x > 0.0);
        assert!(inv.hydrostatic_eff_y > 0.0);
    }
}
