//! Water (tidal) current models for the simulator's environment, the
//! analogue of `wind_model`. Ticked once per outer control step; the
//! returned global-frame current vector is subtracted from the boat's
//! ground velocity before hydrodynamic forces are computed, and added
//! back in the position derivative (ground track = through-water track
//! + current).
//!
//! Two impls: [`NoCurrent`] (still water, the default) and
//! [`TidalStream`] (a uniform sinusoidal flood/ebb stream).

use std::f64::consts::PI;

use anyhow::{Context, Result};
use serde::Deserialize;
use std::path::Path;

/// Semidiurnal lunar tidal period, seconds (12 h 25.2 min). The
/// dominant constituent (M2) in the Bristol Channel.
pub const SEMIDIURNAL_PERIOD_S: f64 = 12.42 * 3600.0;

pub trait CurrentModel {
    /// Water current velocity in the global frame `(east, north)`, m/s,
    /// at simulation time `t`.
    fn sample(&mut self, t: f64) -> (f64, f64);
}

/// Still water.
pub struct NoCurrent;

impl CurrentModel for NoCurrent {
    fn sample(&mut self, _t: f64) -> (f64, f64) {
        (0.0, 0.0)
    }
}

/// A spatially-uniform reversing tidal stream: speed oscillates
/// sinusoidally along a fixed flood/ebb axis.
///
///   speed(t) = peak · sin(2π·t/period + phase)
///
/// `speed > 0` flows toward `axis_rad` (flood); `speed < 0` flows the
/// opposite way (ebb). `axis_rad` is a math-convention angle (0 = +x
/// east, π/2 = +y north). Real tidal streams are not uniform — they
/// accelerate through races and reverse at different times in
/// different places — so this is a first-order approximation good for
/// passage-timing experiments, not navigation.
pub struct TidalStream {
    pub peak_speed: f64,
    pub axis_rad: f64,
    pub period_s: f64,
    pub phase_rad: f64,
}

impl TidalStream {
    pub fn new(peak_speed: f64, axis_rad: f64, period_s: f64, phase_rad: f64) -> Self {
        Self { peak_speed, axis_rad, period_s, phase_rad }
    }
}

impl CurrentModel for TidalStream {
    fn sample(&mut self, t: f64) -> (f64, f64) {
        let speed = self.peak_speed * (2.0 * PI * t / self.period_s + self.phase_rad).sin();
        (speed * self.axis_rad.cos(), speed * self.axis_rad.sin())
    }
}

/// A measured/forecast current time series (e.g. from CMEMS via
/// scripts/fetch_tides.py): uniform in space, linearly interpolated in
/// time. `t = 0` maps to the first sample; before/after the series the
/// endpoints are held.
#[derive(Deserialize)]
pub struct TabulatedCurrent {
    dt_s: f64,
    u_east: Vec<f64>,
    v_north: Vec<f64>,
}

impl TabulatedCurrent {
    pub fn load(path: &Path) -> Result<Self> {
        let file = std::fs::File::open(path)
            .with_context(|| format!("opening tide data {}", path.display()))?;
        let t: TabulatedCurrent = serde_json::from_reader(file)
            .with_context(|| format!("parsing tide data {}", path.display()))?;
        anyhow::ensure!(
            !t.u_east.is_empty() && t.u_east.len() == t.v_north.len() && t.dt_s > 0.0,
            "tide data {} must have matching non-empty u/v and dt_s>0",
            path.display()
        );
        Ok(t)
    }
}

impl CurrentModel for TabulatedCurrent {
    fn sample(&mut self, t: f64) -> (f64, f64) {
        let n = self.u_east.len();
        let f = (t / self.dt_s).clamp(0.0, (n - 1) as f64);
        let i = f.floor() as usize;
        if i + 1 >= n {
            return (self.u_east[n - 1], self.v_north[n - 1]);
        }
        let frac = f - i as f64;
        let lerp = |a: f64, b: f64| a + (b - a) * frac;
        (
            lerp(self.u_east[i], self.u_east[i + 1]),
            lerp(self.v_north[i], self.v_north[i + 1]),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn no_current_is_zero() {
        let mut m = NoCurrent;
        assert_eq!(m.sample(0.0), (0.0, 0.0));
        assert_eq!(m.sample(1234.5), (0.0, 0.0));
    }

    #[test]
    fn tidal_stream_peaks_and_reverses() {
        // Flood along +x (east), peak 2 m/s, phase 0.
        let p = SEMIDIURNAL_PERIOD_S;
        let mut m = TidalStream::new(2.0, 0.0, p, 0.0);
        // t=0: sin(0)=0 → slack.
        let (x0, y0) = m.sample(0.0);
        assert!(x0.abs() < 1e-9 && y0.abs() < 1e-9);
        // Quarter period: peak flood toward +x.
        let (xq, yq) = m.sample(p / 4.0);
        assert!((xq - 2.0).abs() < 1e-9, "flood peak {}", xq);
        assert!(yq.abs() < 1e-9);
        // Three-quarter period: peak ebb toward -x.
        let (xe, _) = m.sample(3.0 * p / 4.0);
        assert!((xe + 2.0).abs() < 1e-9, "ebb peak {}", xe);
    }

    #[test]
    fn tabulated_current_interpolates_and_clamps() {
        let json = r#"{"dt_s":100.0,"u_east":[0.0,1.0,2.0],"v_north":[0.0,0.0,0.0]}"#;
        let mut t: TabulatedCurrent = serde_json::from_str(json).unwrap();
        assert_eq!(t.sample(0.0), (0.0, 0.0));
        assert_eq!(t.sample(50.0), (0.5, 0.0)); // halfway between sample 0 and 1
        assert_eq!(t.sample(100.0), (1.0, 0.0));
        assert_eq!(t.sample(150.0), (1.5, 0.0));
        // Past the end → hold last sample.
        assert_eq!(t.sample(999.0), (2.0, 0.0));
        // Before the start → hold first.
        assert_eq!(t.sample(-10.0), (0.0, 0.0));
    }

    #[test]
    fn tidal_stream_axis_rotates() {
        // Flood toward north (+y).
        let p = SEMIDIURNAL_PERIOD_S;
        let mut m = TidalStream::new(1.0, PI / 2.0, p, 0.0);
        let (x, y) = m.sample(p / 4.0);
        assert!(x.abs() < 1e-9, "x {}", x);
        assert!((y - 1.0).abs() < 1e-9, "y {}", y);
    }
}
