//! The currently-shipping autopilot: route follower → heading
//! controller → sail-trim optimiser, all wired together behind the
//! `Autopilot` trait.

use std::f64::consts::PI;

use crate::autopilot::{Autopilot, Command, Observation};
use crate::config::Config;
use crate::controller::HeadingController;
use crate::current_model::TideForecast;
use crate::physics::wind::{calculate_apparent_wind, TrueWind};
use crate::route::{Route, RouteFollower, Tack};
use crate::sail::sail_angle;

/// Below this through-water speed (m/s) crab compensation is disabled:
/// the boat has too little way on for the current triangle to be
/// well-conditioned (it would demand huge crab angles), so we just
/// point at the target and let it build speed first.
const CRAB_MIN_WATER_SPEED: f64 = 0.5;

/// Cap on the crab angle. Even if the perpendicular current is a large
/// fraction of the boat's water speed, never offset the heading by more
/// than this — a near-90° crab means the boat can't hold the course
/// anyway, and a runaway offset just destabilises the heading loop.
const MAX_CRAB_RAD: f64 = 40.0 * PI / 180.0;

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
    /// When true, offset the commanded heading by a crab angle so the
    /// resulting course over ground tracks the desired course despite
    /// the tidal set (direct steering only — never while tacking).
    crab_enabled: bool,
    /// Tidal-current forecast for gate look-ahead (matches the run's
    /// current model). `TideForecast::None` → gates use the present
    /// current only.
    forecast: TideForecast,
}

impl RouteAutopilot {
    pub fn new(
        cfg: &Config,
        route: Route,
        control_period_s: f64,
        sail_resample_period_s: f64,
        crab_enabled: bool,
        forecast: TideForecast,
    ) -> Self {
        Self {
            follower: RouteFollower::new(route),
            heading_controller: HeadingController::new(cfg, control_period_s),
            sail_stretching: cfg.boat.sail.stretching,
            sail_resample_period_s,
            last_sail_mag: 0.0,
            last_sail_t: None,
            sail_side: 1.0,
            crab_enabled,
            forecast,
        }
    }

    pub fn route(&self) -> &Route {
        self.follower.route()
    }
}

/// Heading that makes good `desired_cog` (course over ground) given a
/// `current` (global east/north m/s) and the boat's through-water speed
/// `water_speed`. Solves the current triangle: the boat points upstream
/// of the track by `asin(-c_perp / V_w)` so the cross-track component of
/// the current is cancelled, capped at ±`MAX_CRAB_RAD` and disabled
/// below `CRAB_MIN_WATER_SPEED`.
///
/// Caveat (why crab is opt-in): this is a pure kinematic triangle that
/// ignores the sail polar. Crabbing toward the wind moves the boat to a
/// finer point of sail and *reduces* `water_speed`, which then demands
/// even more crab — a coupling that can stall a marginally-powered boat
/// and destabilise the heading loop. The follower's reactive cross-track
/// term holds moderate set well without this; doing crab properly needs
/// polar-aware speed/heading planning.
fn crab_heading(desired_cog: f64, current: (f64, f64), water_speed: f64) -> f64 {
    if water_speed < CRAB_MIN_WATER_SPEED {
        return desired_cog;
    }
    let (cx, cy) = current;
    let c_perp = -cx * desired_cog.sin() + cy * desired_cog.cos();
    let lim = MAX_CRAB_RAD.sin();
    desired_cog + (-c_perp / water_speed).clamp(-lim, lim).asin()
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
            .update(obs.t, obs.pos_x, obs.pos_y, tw, obs.current, &self.forecast)
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

        // Crab compensation: treat the follower's output as the desired
        // course over ground and offset the commanded heading so the
        // tide doesn't set the boat off track. Only when steering
        // directly — while tacking the heading is wind-relative and
        // already as high as the boat can point, so crabbing it would be
        // wrong; the cross-track term handles set over successive tacks.
        let has_current = obs.current.0 != 0.0 || obs.current.1 != 0.0;
        let heading_ref = if self.crab_enabled
            && has_current
            && self.follower.current_tack() == Tack::None
        {
            // through-water speed = |ground velocity − current|
            let vgx = obs.vel_x_body * obs.heading.cos() - obs.vel_y_body * obs.heading.sin();
            let vgy = obs.vel_x_body * obs.heading.sin() + obs.vel_y_body * obs.heading.cos();
            let water_speed = (vgx - obs.current.0).hypot(vgy - obs.current.1);
            crab_heading(desired, obs.current, water_speed)
        } else {
            desired
        };

        let speed = obs.vel_x_body.hypot(obs.vel_y_body);
        let drift = obs.vel_y_body.atan2(obs.vel_x_body);
        let rudder = self.heading_controller.control(
            heading_ref,
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
            waypoints: vec![Waypoint { x: 0.0, y: 0.0, gate: false }, Waypoint { x: 0.0, y: 100.0, gate: false }],
            gate_open_along_current: 0.0,
            gate_lead_time_s: 0.0,
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
            current: (0.0, 0.0),
        }
    }

    #[test]
    fn crab_heading_cancels_cross_track_current() {
        // Desired course due east (0 rad). Current sets north at 0.5 m/s,
        // boat makes 1 m/s through the water. The boat must point south
        // of east by asin(0.5/1) = 30° to hold an easterly ground track.
        let h = crab_heading(0.0, (0.0, 0.5), 1.0);
        assert!((h - (-(0.5_f64).asin())).abs() < 1e-9, "got {}", h);
        // No cross-track component (current along the track) → no crab.
        let h2 = crab_heading(0.0, (0.7, 0.0), 1.0);
        assert!(h2.abs() < 1e-9, "got {}", h2);
        // Perpendicular current exceeds water speed → saturates at the
        // -MAX_CRAB_RAD cap rather than running to -90°.
        let h3 = crab_heading(0.0, (0.0, 5.0), 1.0);
        assert!((h3 + MAX_CRAB_RAD).abs() < 1e-9, "got {}", h3);
        // Too slow for crab → returns the desired course unchanged.
        let h4 = crab_heading(0.0, (0.0, 0.5), 0.3);
        assert!(h4.abs() < 1e-9, "got {}", h4);
    }

    #[test]
    fn autopilot_signals_mission_complete_when_route_done() {
        let mut ap = RouteAutopilot::new(&cfg(), straight_north_route(), 0.3, 2.0, true, TideForecast::None);
        // Step once well inside acceptance radius of the only target.
        let cmd = ap.step(&obs(0.0, 0.0, 99.0));
        assert!(cmd.mission_complete, "captured final waypoint should end mission");
    }

    #[test]
    fn autopilot_emits_finite_rudder_and_sail() {
        let mut ap = RouteAutopilot::new(&cfg(), straight_north_route(), 0.3, 2.0, true, TideForecast::None);
        let cmd = ap.step(&obs(0.0, 0.0, 0.0));
        assert!(cmd.rudder_angle.is_finite());
        assert!(cmd.sail_angle.is_finite());
        assert!(!cmd.mission_complete);
    }
}
