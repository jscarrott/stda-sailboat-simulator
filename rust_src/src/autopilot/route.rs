//! The currently-shipping autopilot: route follower → heading
//! controller → sail-trim optimiser, all wired together behind the
//! `Autopilot` trait.

use std::f64::consts::PI;

use crate::autopilot::{Autopilot, Command, Observation};
use crate::config::Config;
use crate::controller::HeadingController;
use crate::physics::wind::{calculate_apparent_wind, TrueWind};
use crate::route::{Route, RouteFollower};
use crate::sail::sail_angle;

/// Hysteresis half-band (rad) around the tack (apparent angle 0) and
/// gybe (apparent angle ±π) transitions. Inside the band the sail side
/// is held rather than flipped, so a boat oscillating across
/// dead-downwind doesn't gybe-slam the rig side-to-side. ~25°.
const SAIL_SIDE_DEADBAND: f64 = 0.44;

pub struct RouteAutopilot {
    follower: RouteFollower,
    heading_controller: HeadingController,
    sail_stretching: f64,
    sail_resample_period_s: f64,
    /// Sail trim *magnitude* (always ≥ 0), resampled periodically.
    last_sail_mag: f64,
    last_sail_t: Option<f64>,
    /// Which side the sail is sheeted (+1 / −1), updated every tick with
    /// hysteresis. The signed command is `sail_side * last_sail_mag`.
    sail_side: f64,
}

impl RouteAutopilot {
    pub fn new(
        cfg: &Config,
        route: Route,
        control_period_s: f64,
        sail_resample_period_s: f64,
    ) -> Self {
        Self {
            follower: RouteFollower::new(route),
            heading_controller: HeadingController::new(cfg, control_period_s),
            sail_stretching: cfg.boat.sail.stretching,
            sail_resample_period_s,
            last_sail_mag: 0.0,
            last_sail_t: None,
            sail_side: 1.0,
        }
    }

    pub fn route(&self) -> &Route {
        self.follower.route()
    }
}

/// Pick the sail side from the apparent wind angle, holding the previous
/// side through the ambiguous bands around dead-ahead (tack) and
/// dead-astern (gybe). Away from those bands the side follows the wind:
/// positive apparent angle → +1, negative → −1 (matching the old
/// `sign(apparent_angle)` convention).
fn next_sail_side(apparent_angle: f64, prev: f64) -> f64 {
    let a = apparent_angle.abs();
    if a <= SAIL_SIDE_DEADBAND || a >= PI - SAIL_SIDE_DEADBAND {
        prev
    } else if apparent_angle >= 0.0 {
        1.0
    } else {
        -1.0
    }
}

impl Autopilot for RouteAutopilot {
    fn step(&mut self, obs: &Observation) -> Command {
        // Adapt the sensor WindReading into the physics `TrueWind`
        // struct the existing follower + apparent-wind helper expect.
        // This conversion is the only place autopilot/route depends on
        // physics types — the trait itself doesn't.
        let tw = TrueWind {
            x: obs.true_wind.speed * obs.true_wind.direction.cos(),
            y: obs.true_wind.speed * obs.true_wind.direction.sin(),
            strength: obs.true_wind.speed,
            direction: obs.true_wind.direction.to_degrees(),
        };

        let desired = match self
            .follower
            .update(obs.t, obs.pos_x, obs.pos_y, tw)
        {
            Some(h) => h,
            None => {
                return Command {
                    rudder_angle: 0.0,
                    sail_angle: self.sail_side * self.last_sail_mag,
                    mission_complete: true,
                };
            }
        };

        let speed = obs.vel_x_body.hypot(obs.vel_y_body);
        let drift = obs.vel_y_body.atan2(obs.vel_x_body);
        let rudder = self.heading_controller.control(
            desired,
            obs.heading,
            obs.yaw_rate,
            speed,
            obs.roll,
            drift,
        );

        let apparent =
            calculate_apparent_wind(obs.heading, obs.vel_x_body, obs.vel_y_body, tw);

        // Sail trim *magnitude* is re-optimised at `sail_resample_period_s`;
        // between resamples we hold it. sail_angle() itself holds the
        // previous magnitude when apparent wind is too weak to trust.
        let due = match self.last_sail_t {
            None => true,
            Some(prev) => obs.t - prev >= self.sail_resample_period_s,
        };
        if due {
            self.last_sail_mag = sail_angle(
                apparent.angle,
                apparent.speed,
                self.sail_stretching,
                self.last_sail_mag,
            );
            self.last_sail_t = Some(obs.t);
        }

        // Sail *side* is updated every tick with hysteresis, so the rig
        // doesn't gybe-slam when the boat wanders across dead-downwind.
        // The signed command, slew-limited by the runner, carries the
        // sail smoothly across centreline during a real tack or gybe.
        self.sail_side = next_sail_side(apparent.angle, self.sail_side);

        Command {
            rudder_angle: rudder,
            sail_angle: self.sail_side * self.last_sail_mag,
            mission_complete: false,
        }
    }

    fn reset(&mut self) {
        self.heading_controller.reset();
        self.last_sail_mag = 0.0;
        self.last_sail_t = None;
        self.sail_side = 1.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::autopilot::WindReading;
    use crate::route::Waypoint;
    use std::path::PathBuf;

    fn cfg() -> Config {
        Config::load(&PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("sim_params_config.yaml")).unwrap()
    }

    fn straight_north_route() -> Route {
        Route {
            name: "test_north".into(),
            acceptance_radius: 5.0,
            close_hauled_angle_deg: 45.0,
            xte_lookahead: 15.0,
            min_tack_duration_s: 3.0,
            wind: None,
            waypoints: vec![Waypoint { x: 0.0, y: 0.0 }, Waypoint { x: 0.0, y: 100.0 }],
            loop_route: false,
        }
    }

    fn obs(t: f64, x: f64, y: f64) -> Observation {
        Observation {
            t,
            pos_x: x,
            pos_y: y,
            heading: 0.0,
            yaw_rate: 0.0,
            roll: 0.0,
            vel_x_body: 1.0,
            vel_y_body: 0.0,
            true_wind: WindReading { direction: -std::f64::consts::PI / 2.0, speed: 5.0 },
        }
    }

    #[test]
    fn autopilot_signals_mission_complete_when_route_done() {
        let mut ap = RouteAutopilot::new(&cfg(), straight_north_route(), 0.3, 2.0);
        // Step once well inside acceptance radius of the only target.
        let cmd = ap.step(&obs(0.0, 0.0, 99.0));
        assert!(cmd.mission_complete, "captured final waypoint should end mission");
    }

    #[test]
    fn autopilot_emits_finite_rudder_and_sail() {
        let mut ap = RouteAutopilot::new(&cfg(), straight_north_route(), 0.3, 2.0);
        let cmd = ap.step(&obs(0.0, 0.0, 0.0));
        assert!(cmd.rudder_angle.is_finite());
        assert!(cmd.sail_angle.is_finite());
        assert!(!cmd.mission_complete);
    }
}
