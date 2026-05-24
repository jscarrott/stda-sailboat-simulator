//! The currently-shipping autopilot: route follower → heading
//! controller → sail-trim optimiser, all wired together behind the
//! `Autopilot` trait.

use crate::autopilot::{Autopilot, Command, Observation};
use crate::config::Config;
use crate::controller::HeadingController;
use crate::physics::wind::{calculate_apparent_wind, TrueWind};
use crate::route::{Route, RouteFollower};
use crate::sail::sail_angle;

pub struct RouteAutopilot {
    follower: RouteFollower,
    heading_controller: HeadingController,
    sail_stretching: f64,
    sail_resample_period_s: f64,
    last_sail: f64,
    last_sail_t: Option<f64>,
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
            last_sail: 0.0,
            last_sail_t: None,
        }
    }

    pub fn route(&self) -> &Route {
        self.follower.route()
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
                    sail_angle: self.last_sail,
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

        // Sail trim is re-optimised at `sail_resample_period_s`; in
        // between, hold the last command so winch motion is smooth.
        let due = match self.last_sail_t {
            None => true,
            Some(prev) => obs.t - prev >= self.sail_resample_period_s,
        };
        if due {
            let apparent =
                calculate_apparent_wind(obs.heading, obs.vel_x_body, obs.vel_y_body, tw);
            // sail_angle() holds `self.last_sail` itself when apparent
            // wind is below its reliability threshold, so we always
            // call it and let the guard decide.
            self.last_sail = sail_angle(
                apparent.angle,
                apparent.speed,
                self.sail_stretching,
                self.last_sail,
            );
            self.last_sail_t = Some(obs.t);
        }

        Command {
            rudder_angle: rudder,
            sail_angle: self.last_sail,
            mission_complete: false,
        }
    }

    fn reset(&mut self) {
        self.heading_controller.reset();
        self.last_sail = 0.0;
        self.last_sail_t = None;
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
