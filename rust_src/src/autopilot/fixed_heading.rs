//! A minimal autopilot that holds a constant compass heading and trims
//! the sail to the apparent wind. Used by the `polar` scenario to
//! measure steady-state boat speed at each point of sail — the
//! instrument for calibrating the hull parameters against a known
//! speed polar.

use crate::autopilot::{Autopilot, Command, Observation};
use crate::config::Config;
use crate::controller::{new_from_config, HeadingController};
use crate::physics::wind::{calculate_apparent_wind, TrueWind};
use crate::sail::sail_angle;

pub struct FixedHeadingAutopilot {
    target_heading: f64,
    heading_controller: HeadingController,
    sail_stretching: f64,
    last_sail_mag: f64,
}

impl FixedHeadingAutopilot {
    pub fn new(cfg: &Config, target_heading: f64, control_period_s: f64) -> Self {
        Self {
            target_heading,
            heading_controller: new_from_config(cfg, control_period_s),
            sail_stretching: cfg.boat.sail.stretching,
            last_sail_mag: 0.0,
        }
    }
}

impl Autopilot for FixedHeadingAutopilot {
    fn step(&mut self, obs: &Observation) -> Command {
        let tw = TrueWind {
            x: obs.true_wind.speed * obs.true_wind.direction.cos(),
            y: obs.true_wind.speed * obs.true_wind.direction.sin(),
            strength: obs.true_wind.speed,
            direction: obs.true_wind.direction.to_degrees(),
        };
        let apparent = calculate_apparent_wind(obs.heading, obs.vel_x_body, obs.vel_y_body, tw);
        self.last_sail_mag =
            sail_angle(apparent.angle, apparent.speed, self.sail_stretching, self.last_sail_mag);
        let sail = apparent.angle.signum() * self.last_sail_mag;

        let speed = obs.vel_x_body.hypot(obs.vel_y_body);
        let drift = obs.vel_y_body.atan2(obs.vel_x_body);
        let rudder = self.heading_controller.control(
            self.target_heading,
            obs.heading,
            obs.yaw_rate,
            speed,
            obs.roll,
            drift,
        );

        Command { rudder_angle: rudder, sail_angle: sail, mission_complete: false }
    }
}
