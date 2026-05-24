//! Control / world boundary.
//!
//! Everything that produces actuator commands from sensed state lives
//! behind the `Autopilot` trait. The simulator builds `Observation`s
//! from its state vector and applies `Command`s to its environment; a
//! future hardware adapter will do the same from real sensors and to
//! real servos. Neither side knows which the other is.

pub mod fixed_heading;
pub mod route;

pub use fixed_heading::FixedHeadingAutopilot;
pub use route::RouteAutopilot;

/// World-frame wind sensed by the boat.
///
/// `direction` follows the sim's math convention (the angle of the
/// wind's velocity vector — i.e. the way it is blowing **toward**, not
/// where it's blowing from). On real hardware this is computed from
/// apparent wind + boat course, or read from a dedicated true-wind
/// instrument.
#[derive(Debug, Clone, Copy)]
pub struct WindReading {
    pub direction: f64, // rad
    pub speed: f64,     // m/s
}

/// Snapshot the runner hands to the autopilot once per control tick.
/// All angles in radians, velocities in m/s.
///
/// On a real boat: `pos_x`/`pos_y` come from a projected GPS fix
/// (faked or real); `heading` from a fluxgate compass; `yaw_rate` /
/// `roll` from an IMU; `vel_*_body` from GPS velocity rotated into the
/// hull frame.
#[derive(Debug, Clone, Copy)]
pub struct Observation {
    pub t: f64,
    pub pos_x: f64,
    pub pos_y: f64,
    pub heading: f64,
    pub yaw_rate: f64,
    pub roll: f64,
    pub vel_x_body: f64,
    pub vel_y_body: f64,
    pub true_wind: WindReading,
    /// Estimated water (tidal) current in the global frame, m/s
    /// (east, north). On hardware this comes from a tidal-stream atlas
    /// or is inferred from GPS course/speed vs heading/log; in the sim
    /// it's the true current. `(0, 0)` when unknown or still water —
    /// crab compensation then does nothing.
    pub current: (f64, f64),
}

/// What the autopilot wants the actuators set to this tick.
#[derive(Debug, Clone, Copy)]
pub struct Command {
    pub rudder_angle: f64,
    pub sail_angle: f64,
    /// Runner should terminate the run cleanly (mission accomplished).
    pub mission_complete: bool,
}

/// The single seam between control and world.
///
/// Object-safe on purpose: runners hold `&mut dyn Autopilot`, so any
/// new method must keep this dyn-compatible (no generics, no `Self`
/// by value).
pub trait Autopilot {
    fn step(&mut self, obs: &Observation) -> Command;

    fn reset(&mut self) {}
}
