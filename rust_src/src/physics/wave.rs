use crate::config::Invariants;
use std::f64::consts::PI;

use crate::physics::util::sign;

#[derive(Clone, Copy, Debug)]
pub struct Wave {
    pub length: f64,
    pub direction: f64,
    pub amplitude: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct WaveInfluence {
    pub height: f64,
    pub gradient_x: f64,
    pub gradient_y: f64,
}

/// Port of `calculate_wave_influence` from `simulation.py:294`.
///
/// Note: the Python version treats `wave.direction` as radians inside
/// `cos`/`sin`, even though the YAML stores it in degrees. Ported
/// verbatim because the shipped scenarios all have `amplitude = 0`,
/// making this branch dormant.
pub fn calculate_wave_influence(pos_x: f64, pos_y: f64, yaw: f64, wave: Wave, time: f64, gravity: f64) -> WaveInfluence {
    let frequency = ((2.0 * PI * gravity) / wave.length).sqrt();
    let k_x = 2.0 * PI / wave.length * wave.direction.cos();
    let k_y = 2.0 * PI / wave.length * wave.direction.sin();
    let phase = frequency * time - k_x * pos_x - k_y * pos_y;
    let factor = -wave.amplitude * phase.cos();
    let gradient_x = k_x * factor;
    let gradient_y = k_y * factor;
    WaveInfluence {
        height: wave.amplitude * phase.sin(),
        gradient_x: gradient_x * yaw.cos() + gradient_y * yaw.sin(),
        gradient_y: gradient_y * yaw.cos() - gradient_x * yaw.sin(),
    }
}

/// Port of `calculate_wave_impedance` from `simulation.py:283`.
pub fn calculate_wave_impedance(vel_x: f64, speed: f64, hull_speed: f64, inv: &Invariants) -> f64 {
    -sign(vel_x) * speed.powi(2) * (speed / hull_speed).powi(2) * inv.wave_impedance
}
