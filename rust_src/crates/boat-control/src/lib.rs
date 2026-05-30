//! Float-generic, `no_std` sailboat control logic.
//!
//! This crate holds the *single* copy of the heading controller, the sail-trim
//! optimiser and the apparent-wind transform. It is generic over the float type
//! so the host simulator can instantiate it at `f64` (preserving the bit-exact
//! Python-trace regression tests) while the nRF52840 firmware instantiates it at
//! `f32` (using the Cortex-M4F hardware FPU).
//!
//! Ports:
//! - [`HeadingController`] ← `sailboat_sim/src/controller.rs` (`heading_controller.py`)
//! - [`sail_angle`]        ← `sailboat_sim/src/sail.rs` (`sail_angle.py`)
//! - [`apparent_wind`]     ← `sailboat_sim/src/physics/wind.rs`
//!
//! All math goes through `num_traits::Float`, which routes to the system libm
//! under the `std` feature (host) and to the `libm` crate under `no_std`
//! (device). No allocation, no I/O.

#![cfg_attr(not(feature = "std"), no_std)]

use num_traits::{Float, FloatConst};

pub mod proto;

/// Sign function matching Python's `copysign(1, v) if v != 0 else 0`.
///
/// Differs from `Float::signum`, which returns ±1 even for ±0.0. This is the
/// generic twin of `sailboat_sim`'s `physics::util::sign`; kept in sync by hand
/// (the simulator's physics code keeps its own f64 copy to avoid depending on
/// this crate).
#[inline]
pub fn sign<T: Float>(v: T) -> T {
    if v == T::zero() {
        T::zero()
    } else if v > T::zero() {
        T::one()
    } else {
        -T::one()
    }
}

/// Helper: build a `T` from an `f64` literal. Panics only if the target type
/// cannot represent the value, which never happens for the small constants used
/// here.
#[inline]
fn c<T: Float>(v: f64) -> T {
    T::from(v).expect("constant representable in target float type")
}

/// PID heading controller with anti-windup and low-speed gain shaping.
///
/// Port of `heading_controller.controll()`. Carries only the integrator state
/// (`summed_error`); everything else is recomputed each call. All parameters are
/// precomputed scalars — this type has no dependency on the simulator's `Config`
/// (the host builds them in `controller::new_from_config`, the firmware in its
/// init handshake).
#[derive(Clone, Copy, Debug)]
pub struct HeadingController<T> {
    pub sample_time: T,
    pub speed_adaption: T,
    pub max_rudder_angle: T,
    pub factor: T,
    pub kp: T,
    pub ki: T,
    pub kd: T,
    summed_error: T,
}

impl<T: Float + FloatConst> HeadingController<T> {
    /// Construct from fully precomputed parameters.
    #[allow(clippy::too_many_arguments)]
    pub fn from_params(
        factor: T,
        kp: T,
        ki: T,
        kd: T,
        sample_time: T,
        speed_adaption: T,
        max_rudder_angle: T,
    ) -> Self {
        Self {
            sample_time,
            speed_adaption,
            max_rudder_angle,
            factor,
            kp,
            ki,
            kd,
            summed_error: T::zero(),
        }
    }

    /// Convenience constructor mirroring the simulator's defaults: the 4 m
    /// hull's hand-tuned gains `(0.5, 0.1, 0.9)`, `speed_adaption = 0.3 m/s`,
    /// and a `±15°` rudder limit. `factor` is boat-specific
    /// (`distance_cog_rudder · rudder_area · π · water_density / moi_z`).
    pub fn with_default_gains(factor: T, sample_time: T) -> Self {
        let max_rudder_angle = c::<T>(15.0) * T::PI() / c::<T>(180.0);
        Self::from_params(
            factor,
            c(0.5),
            c(0.1),
            c(0.9),
            sample_time,
            c(0.3),
            max_rudder_angle,
        )
    }

    /// One control tick. Returns the commanded rudder angle (rad).
    ///
    /// Port of `controll()`: wrap heading error to `[-π, π]`, accumulate the
    /// integrator (drift-compensated), clamp speed to avoid the low-speed
    /// singularity, apply the speed/roll-scaled PID law, then saturate with
    /// back-calculated anti-windup.
    pub fn control(
        &mut self,
        desired_heading: T,
        heading: T,
        yaw_rate: T,
        speed: T,
        roll: T,
        drift_angle: T,
    ) -> T {
        let pi = T::PI();
        let two_pi = pi + pi;

        let mut heading_error = desired_heading - heading;
        while heading_error > pi {
            heading_error = heading_error - two_pi;
        }
        while heading_error < -pi {
            heading_error = heading_error + two_pi;
        }

        self.summed_error = self.summed_error + self.sample_time * (heading_error - drift_angle);

        let effective_speed = if speed < self.speed_adaption {
            self.speed_adaption
        } else {
            speed
        };
        let factor2 = -T::one() / self.factor / (effective_speed * effective_speed) / roll.cos();

        let mut rudder_angle = factor2
            * (self.kp * heading_error + self.ki * self.summed_error - self.kd * yaw_rate);

        if rudder_angle.abs() > self.max_rudder_angle {
            rudder_angle = sign(rudder_angle) * self.max_rudder_angle;
            // Anti-windup: back-calculate the integrator so it doesn't wind past
            // saturation. Mirrors heading_controller.py:122.
            self.summed_error =
                (rudder_angle / factor2 - (self.kp * heading_error - self.kd * yaw_rate)) / self.ki;
        }

        rudder_angle
    }

    pub fn reset(&mut self) {
        self.summed_error = T::zero();
    }
}

/// Apparent-wind speed below which `sail_angle` holds the previous trim, because
/// `atan2` of the (vx, vy) components becomes numerically unreliable. Matches
/// `sailboat_sim/src/sail.rs`.
const MIN_WIND_SPEED_FOR_TRIM: f64 = 0.2;
const LIMIT_WIND_SPEED: f64 = 6.0;
const STALL_DEG: f64 = 14.0;

/// Optimal sail trim angle from apparent wind. Port of `sail_angle.py:3` plus
/// the low-apparent-wind guard. Returns the absolute sail angle — the sign is
/// applied by the caller per the "True Sail Angle Sign Convention".
pub fn sail_angle<T: Float + FloatConst>(
    wind_angle: T,
    wind_speed: T,
    sail_stretching: T,
    previous_sail: T,
) -> T {
    if wind_speed < c(MIN_WIND_SPEED_FOR_TRIM) {
        return previous_sail;
    }
    let cos_wa = wind_angle.cos();
    let mut opt_aoa =
        wind_angle.sin() / (cos_wa + c::<T>(0.4) * cos_wa.powi(2)) * sail_stretching / c(4.0);
    let stall_rad = c::<T>(STALL_DEG) * T::PI() / c::<T>(180.0);
    if opt_aoa.abs() > stall_rad {
        opt_aoa = sign(wind_angle) * stall_rad;
    }
    if wind_speed > c(LIMIT_WIND_SPEED) {
        opt_aoa = opt_aoa * (c::<T>(LIMIT_WIND_SPEED) / wind_speed).powi(2);
    }
    let half_pi = T::PI() / c(2.0);
    (wind_angle - opt_aoa).max(-half_pi).min(half_pi).abs()
}

/// Apparent wind in the body frame from true wind and boat velocity.
///
/// Port of `calculate_apparent_wind` (`physics/wind.rs`). `true_wind_dir` is the
/// world-frame direction the wind blows *toward* (rad), matching the sim's math
/// convention. Returns `(angle, speed)`: the apparent-wind angle relative to the
/// hull (rad) and its magnitude (m/s).
pub fn apparent_wind<T: Float>(
    heading: T,
    vel_x_body: T,
    vel_y_body: T,
    true_wind_dir: T,
    true_wind_speed: T,
) -> (T, T) {
    let tw_x = true_wind_speed * true_wind_dir.cos();
    let tw_y = true_wind_speed * true_wind_dir.sin();
    let transformed_x = tw_x * heading.cos() + tw_y * heading.sin();
    let transformed_y = tw_x * -heading.sin() + tw_y * heading.cos();
    let apparent_x = transformed_x - vel_x_body;
    let apparent_y = transformed_y - vel_y_body;
    let angle = (-apparent_y).atan2(-apparent_x);
    let speed = (apparent_x * apparent_x + apparent_y * apparent_y).sqrt();
    (angle, speed)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sign_matches_python_semantics() {
        assert_eq!(sign(0.0_f64), 0.0);
        assert_eq!(sign(3.2_f64), 1.0);
        assert_eq!(sign(-0.1_f64), -1.0);
    }

    #[test]
    fn f32_control_tracks_f64() {
        // The shared logic must give the same answer (to f32 precision) whether
        // instantiated at f64 (host) or f32 (device).
        let factor = 0.477_f64;
        let mut c64 = HeadingController::<f64>::from_params(
            factor, 0.5, 0.1, 0.9, 0.3, 0.3, 15.0_f64.to_radians(),
        );
        let mut c32 = HeadingController::<f32>::from_params(
            factor as f32,
            0.5,
            0.1,
            0.9,
            0.3,
            0.3,
            15.0_f32.to_radians(),
        );
        // A few ticks with a standing heading error.
        for _ in 0..5 {
            let r64 = c64.control(0.5, 0.0, 0.0, 1.5, 0.1, 0.0);
            let r32 = c32.control(0.5, 0.0, 0.0, 1.5, 0.1, 0.0);
            assert!((r64 as f32 - r32).abs() < 1e-4, "f64 {} vs f32 {}", r64, r32);
        }
    }

    #[test]
    fn sail_angle_holds_previous_below_threshold() {
        let prev = 0.42_f64;
        assert_eq!(sail_angle(0.0, 0.1, 0.961, prev), prev);
        assert_ne!(sail_angle(0.0, 0.5, 0.961, prev), prev);
    }
}
