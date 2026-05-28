//! Wind models for the simulator's environment. Three impls so far:
//! [`ConstantWind`] (used when a route has no variance configured),
//! [`OrnsteinUhlenbeckWind`] (mean-reverting gust + shift noise), and
//! [`TabulatedWind`] (a cached real-world forecast series).
//!
//! The model is ticked once per outer control step in `simulate()`,
//! and the resulting `TrueWind` is what the physics ODE sees plus what
//! the autopilot reads (after rotation into body frame).

use std::f64::consts::PI;
use std::path::Path;

use anyhow::{Context, Result};
use serde::Deserialize;

use crate::physics::wind::TrueWind;

/// Trait implemented by all wind models. Stateful — each call advances
/// the model by `dt` seconds.
pub trait WindModel {
    /// Sample the wind at simulation time `t`, advancing internal state
    /// by `dt` seconds.
    fn sample(&mut self, t: f64, dt: f64) -> TrueWind;
}

/// A wind that never changes. The default if no variance is requested.
pub struct ConstantWind {
    wind: TrueWind,
}

impl ConstantWind {
    pub fn new(wind: TrueWind) -> Self {
        Self { wind }
    }
}

impl WindModel for ConstantWind {
    fn sample(&mut self, _t: f64, _dt: f64) -> TrueWind {
        self.wind
    }
}

/// Mean-reverting Ornstein–Uhlenbeck process on both wind speed and
/// wind direction, applied as separate, independent OU processes.
///
/// The discrete Euler–Maruyama update for an OU process with mean `m`,
/// stationary std-dev `σ`, and correlation time `τ` is:
///   x ← x − (x − m)·dt/τ + σ·√(2·dt/τ)·N(0,1)
/// which has stationary variance σ² as dt → 0.
pub struct OrnsteinUhlenbeckWind {
    pub mean_speed: f64,
    pub mean_direction_rad: f64,
    pub speed_sigma: f64,        // m/s, std-dev of speed about the mean
    pub direction_sigma: f64,    // rad, std-dev of direction about the mean
    pub correlation_time_s: f64, // τ, seconds
    speed: f64,
    direction: f64,
    rng: Prng,
}

impl OrnsteinUhlenbeckWind {
    pub fn new(
        mean_speed: f64,
        mean_direction_rad: f64,
        speed_sigma: f64,
        direction_sigma: f64,
        correlation_time_s: f64,
        seed: u64,
    ) -> Self {
        Self {
            mean_speed,
            mean_direction_rad,
            speed_sigma,
            direction_sigma,
            correlation_time_s,
            speed: mean_speed,
            direction: mean_direction_rad,
            rng: Prng::new(seed),
        }
    }
}

impl WindModel for OrnsteinUhlenbeckWind {
    fn sample(&mut self, _t: f64, dt: f64) -> TrueWind {
        let tau = self.correlation_time_s.max(1e-3);
        let noise_scale = (2.0 * dt / tau).sqrt();

        self.speed += -(self.speed - self.mean_speed) * (dt / tau)
            + self.speed_sigma * noise_scale * self.rng.gauss();
        // Clamp speed to non-negative; a negative wind speed has no
        // physical meaning and would invert the direction.
        if self.speed < 0.0 {
            self.speed = 0.0;
        }

        self.direction += -(self.direction - self.mean_direction_rad) * (dt / tau)
            + self.direction_sigma * noise_scale * self.rng.gauss();

        TrueWind {
            x: self.speed * self.direction.cos(),
            y: self.speed * self.direction.sin(),
            strength: self.speed,
            direction: self.direction.to_degrees(),
        }
    }
}

/// A wind series cached from a real-world forecast (e.g. Open-Meteo via
/// `scripts/fetch_wind.py`): uniform in space, linearly interpolated in time.
/// `t = 0` maps to the first sample; before/after the series the endpoints
/// are held. Mirrors `current_model::TabulatedCurrent`.
#[derive(Deserialize)]
pub struct TabulatedWind {
    dt_s: f64,
    u_east: Vec<f64>,
    v_north: Vec<f64>,
}

impl TabulatedWind {
    pub fn load(path: &Path) -> Result<Self> {
        let file = std::fs::File::open(path)
            .with_context(|| format!("opening wind data {}", path.display()))?;
        let w: TabulatedWind = serde_json::from_reader(file)
            .with_context(|| format!("parsing wind data {}", path.display()))?;
        anyhow::ensure!(
            !w.u_east.is_empty() && w.u_east.len() == w.v_north.len() && w.dt_s > 0.0,
            "wind data {} must have matching non-empty u/v and dt_s>0",
            path.display()
        );
        Ok(w)
    }
}

impl WindModel for TabulatedWind {
    fn sample(&mut self, t: f64, _dt: f64) -> TrueWind {
        let n = self.u_east.len();
        let f = (t / self.dt_s).clamp(0.0, (n - 1) as f64);
        let i = f.floor() as usize;
        let (u, v) = if i + 1 >= n {
            (self.u_east[n - 1], self.v_north[n - 1])
        } else {
            let frac = f - i as f64;
            (
                self.u_east[i] + (self.u_east[i + 1] - self.u_east[i]) * frac,
                self.v_north[i] + (self.v_north[i + 1] - self.v_north[i]) * frac,
            )
        };
        let strength = (u * u + v * v).sqrt();
        // TrueWind.direction is the math angle of the velocity vector (deg).
        let direction = v.atan2(u).to_degrees();
        TrueWind { x: u, y: v, strength, direction }
    }
}

/// Tiny xorshift64* PRNG + Box–Muller for Gaussian samples. Inlined so
/// the simulator has zero RNG dependencies; reproducible given a seed.
struct Prng {
    state: u64,
    cached: Option<f64>,
}

impl Prng {
    fn new(seed: u64) -> Self {
        Self { state: seed | 1, cached: None }
    }
    fn next_u64(&mut self) -> u64 {
        let mut x = self.state;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.state = x;
        x.wrapping_mul(2685821657736338717)
    }
    fn next_f64(&mut self) -> f64 {
        // Top 53 bits → [0, 1).
        (self.next_u64() >> 11) as f64 * (1.0 / ((1u64 << 53) as f64))
    }
    fn gauss(&mut self) -> f64 {
        if let Some(z) = self.cached.take() {
            return z;
        }
        let mut u1: f64;
        loop {
            u1 = self.next_f64();
            if u1 > 1e-300 {
                break;
            }
        }
        let u2 = self.next_f64();
        let r = (-2.0 * u1.ln()).sqrt();
        let theta = 2.0 * PI * u2;
        self.cached = Some(r * theta.sin());
        r * theta.cos()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn constant_wind_does_not_change() {
        let tw = TrueWind { x: 3.0, y: 4.0, strength: 5.0, direction: 53.13 };
        let mut m = ConstantWind::new(tw);
        let a = m.sample(0.0, 0.3);
        let b = m.sample(100.0, 0.3);
        assert_eq!(a.strength, b.strength);
        assert_eq!(a.x, b.x);
    }

    #[test]
    fn ou_wind_stays_near_mean_long_term() {
        // Sample for ~10 correlation times and check the running mean
        // is close to the configured mean (within a few standard errors).
        let mut m = OrnsteinUhlenbeckWind::new(
            5.0,                            // mean speed
            0.5,                            // mean dir
            0.5,                            // speed σ
            5.0_f64.to_radians(),           // dir σ
            30.0,                           // τ
            0xC0FFEE,
        );
        let dt = 0.3;
        let n = (300.0 / dt) as usize;
        let mut s_sum = 0.0;
        let mut d_sum = 0.0;
        for _ in 0..n {
            let w = m.sample(0.0, dt);
            s_sum += w.strength;
            d_sum += w.direction.to_radians();
        }
        let s_avg = s_sum / n as f64;
        let d_avg = d_sum / n as f64;
        // Sample mean SE ≈ σ * sqrt(2τ / (N·dt)) ≈ σ * sqrt(τ/T_total).
        // For τ=30, T=300, σ_s=0.5 → SE ≈ 0.16. 3·SE = 0.5.
        assert!((s_avg - 5.0).abs() < 1.0, "speed avg {} drifted from 5.0", s_avg);
        assert!((d_avg - 0.5).abs() < 0.3, "dir avg {} drifted from 0.5", d_avg);
    }

    #[test]
    fn ou_wind_reproducible_under_same_seed() {
        let mut a = OrnsteinUhlenbeckWind::new(5.0, 0.0, 1.0, 0.1, 10.0, 42);
        let mut b = OrnsteinUhlenbeckWind::new(5.0, 0.0, 1.0, 0.1, 10.0, 42);
        for i in 0..50 {
            let x = a.sample(i as f64 * 0.3, 0.3);
            let y = b.sample(i as f64 * 0.3, 0.3);
            assert_eq!(x.x, y.x);
            assert_eq!(x.y, y.y);
        }
    }
}
