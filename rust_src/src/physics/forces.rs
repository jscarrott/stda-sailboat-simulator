use std::f64::consts::PI;

use crate::config::{BoatCfg, Config, EnvCfg, Invariants};
use crate::physics::util::sign;
use crate::physics::wave::WaveInfluence;
use crate::physics::wind::ApparentWind;

#[derive(Clone, Copy, Debug, Default)]
pub struct SailForce {
    pub x: f64,
    pub y: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct LateralForce {
    pub x: f64,
    pub y: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct RudderForce {
    pub x: f64,
    pub y: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct HydrostaticForce {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Damping {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub roll: f64,
    pub pitch: f64,
    pub yaw: f64,
}

/// Port of `calculate_damping` from `simulation.py:343`.
pub fn calculate_damping(
    vel_x: f64,
    vel_y: f64,
    vel_z: f64,
    roll_rate: f64,
    pitch_rate: f64,
    yaw_rate: f64,
    inv: &Invariants,
) -> Damping {
    Damping {
        x: inv.damping_invariant_x * vel_x,
        y: inv.damping_invariant_y * vel_y,
        z: inv.damping_invariant_z * vel_z,
        roll: inv.damping_invariant_roll * roll_rate,
        pitch: inv.damping_invariant_pitch * pitch_rate,
        yaw: inv.damping_invariant_yaw * yaw_rate,
    }
}

/// Port of `calculate_hydrostatic_force` from `simulation.py:322`.
/// Returns `(force, x_hs, y_hs)` — the two scalars are moment arms that
/// the pitch/roll moment assembly inside `solve()` reuses. Preserve the
/// 3-tuple shape (CLAUDE.md "Common Pitfalls").
pub fn calculate_hydrostatic_force(
    pos_z: f64,
    roll: f64,
    pitch: f64,
    wave_influence: WaveInfluence,
    inv: &Invariants,
) -> (HydrostaticForce, f64, f64) {
    let force_z = inv.hydrostatic_invariant_z * (pos_z - wave_influence.height) + inv.gravity_force;
    let force = HydrostaticForce {
        x: force_z * wave_influence.gradient_x,
        y: force_z * wave_influence.gradient_y,
        z: force_z,
    };
    let x_hs = inv.hydrostatic_eff_y * (pitch + wave_influence.gradient_x.atan()).sin();
    let y_hs = inv.hydrostatic_eff_x * -(roll - wave_influence.gradient_y.atan()).sin();
    (force, x_hs, y_hs)
}

/// Port of `calculate_rudder_force` from `simulation.py:268`.
pub fn calculate_rudder_force(speed: f64, rudder_angle: f64, env: &EnvCfg, boat: &BoatCfg) -> RudderForce {
    let pressure = (env.water_density / 2.0) * speed * speed;
    RudderForce {
        x: -((4.0 * PI) / boat.rudder.stretching) * rudder_angle * rudder_angle * pressure * boat.rudder.area,
        y: 2.0 * PI * pressure * boat.rudder.area * rudder_angle,
    }
}

/// Port of `calculate_lateral_force` from `simulation.py:230`.
/// Returns `(force, separation)`. The separation factor feeds back into
/// the yaw-moment assembly in `solve()` — never discard it.
///
/// FIXME(per CLAUDE.md pitfall #7): `boat.sail.area` is reused inside
/// the keel separated-flow branch — likely a copy-paste bug from
/// `calculate_sail_force`. Ported verbatim.
pub fn calculate_lateral_force(
    vel_x: f64,
    vel_y: f64,
    roll: f64,
    speed: f64,
    env: &EnvCfg,
    boat: &BoatCfg,
) -> (LateralForce, f64) {
    let pressure = (env.water_density / 2.0) * speed * speed * roll.cos().powi(2);
    let friction = if speed != 0.0 {
        2.66 * (env.water_viscosity / (speed * boat.keel.length)).sqrt()
    } else {
        0.0
    };
    let aoa = vel_y.atan2(vel_x);
    let eff_aoa = if aoa < -PI / 2.0 {
        PI + aoa
    } else if aoa > PI / 2.0 {
        -PI + aoa
    } else {
        aoa
    };
    let separation = 1.0 - (-((eff_aoa.abs() / (PI / 180.0 * 25.0)).powi(2))).exp();
    let tmp = -(friction + (4.0 * PI * eff_aoa * eff_aoa * separation) / boat.keel.stretching);
    let separated_transverse_force = -sign(aoa) * pressure * boat.sail.area * aoa.sin().powi(2);
    let lateral_area = boat.lateral_area;
    let force = LateralForce {
        x: (1.0 - separation) * (tmp * aoa.cos() + 2.0 * PI * eff_aoa * aoa.sin()) * pressure * lateral_area,
        y: (1.0 - separation) * (tmp * aoa.sin() - 2.0 * PI * eff_aoa * aoa.cos()) * pressure * lateral_area
            + separation * separated_transverse_force,
    };
    (force, separation)
}

/// Port of `calculate_sail_force` from `simulation.py:183`.
/// `sail_angle` here is the signed sail angle (sign applied at the call
/// site in `solve()` — never pre-applied per CLAUDE.md "True Sail Angle
/// Sign Convention").
pub fn calculate_sail_force(roll: f64, wind: ApparentWind, sail_angle: f64, env: &EnvCfg, boat: &BoatCfg) -> SailForce {
    let mut aoa = wind.angle - sail_angle;
    if aoa * sail_angle < 0.0 {
        aoa = 0.0;
    }
    let eff_aoa = if aoa < -PI / 2.0 {
        PI + aoa
    } else if aoa > PI / 2.0 {
        -PI + aoa
    } else {
        aoa
    };
    let pressure = (env.air_density / 2.0) * wind.speed * wind.speed * (roll * sail_angle.cos()).cos().powi(2);
    let friction = if wind.speed != 0.0 {
        3.55 * (env.air_viscosity / (wind.speed * boat.sail.length)).sqrt()
    } else {
        0.0
    };
    let separation = 1.0 - (-((eff_aoa.abs() / (PI / 180.0 * 25.0)).powi(2))).exp();
    let propulsion = (2.0 * PI * eff_aoa * wind.angle.sin()
        - (friction + (4.0 * PI * eff_aoa * eff_aoa * separation) / boat.sail.stretching) * wind.angle.cos())
        * boat.sail.area
        * pressure;
    let transverse_force = (-2.0 * PI * eff_aoa * wind.angle.cos()
        - (friction + (4.0 * PI * eff_aoa * eff_aoa * separation) / boat.sail.stretching) * wind.angle.sin())
        * boat.sail.area
        * pressure;
    let separated_propulsion = sign(aoa) * pressure * boat.sail.area * aoa.sin().powi(2) * sail_angle.sin();
    let separated_transverse_force = -sign(aoa) * pressure * boat.sail.area * aoa.sin().powi(2) * sail_angle.cos();
    SailForce {
        x: (1.0 - separation) * propulsion + separation * separated_propulsion,
        y: (1.0 - separation) * transverse_force + separation * separated_transverse_force,
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Environment {
    pub sail_angle: f64,
    pub rudder_angle: f64,
    pub true_wind: crate::physics::wind::TrueWind,
    pub wave: crate::physics::wave::Wave,
    /// Water (tidal) current in the global frame, m/s (east, north).
    /// Hydrodynamic forces act on the boat's velocity *relative to the
    /// water*, so this is subtracted from the boat's ground velocity
    /// before computing keel / rudder / hull-drag forces, and the
    /// position derivative carries the boat over ground = through-water
    /// velocity + current. Zero for still water.
    pub water_current: (f64, f64),
}

impl Environment {
    pub fn from_config(cfg: &Config) -> Self {
        let i = &cfg.simulator.initial;
        let dir_rad = i.wind_direction.to_radians();
        Self {
            sail_angle: i.sail_angle,
            rudder_angle: i.rudder_angle,
            true_wind: crate::physics::wind::TrueWind {
                x: i.wind_strength * dir_rad.cos(),
                y: i.wind_strength * dir_rad.sin(),
                strength: i.wind_strength,
                direction: i.wind_direction,
            },
            wave: crate::physics::wave::Wave {
                length: i.wave_length,
                direction: i.wave_direction,
                amplitude: i.wave_amplitude,
            },
            water_current: (0.0, 0.0),
        }
    }
}
