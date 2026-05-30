//! The currently-shipping autopilot: route follower → heading
//! controller → sail-trim optimiser, all wired together behind the
//! `Autopilot` trait.

use std::f64::consts::PI;

use crate::autopilot::{Autopilot, Command, Observation};
use crate::chart::Chart;
use crate::config::Config;
use crate::controller::{new_from_config, HeadingController};
use crate::current_model::TideForecast;
use crate::physics::wind::{calculate_apparent_wind, TrueWind};
use crate::planner::{plan, simplify, PlanConfig, Polar};
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

/// How far ahead (m) the follower looks for land. If the commanded
/// heading would put the boat into land within this distance, it
/// deflects to the nearest clear heading. Chosen smaller than typical
/// off-the-rocks waypoint clearances so legitimate close approaches
/// aren't fought, but large enough to react to tidal set.
const LAND_LOOKAHEAD_M: f64 = 300.0;

/// A coastline obstacle for reactive avoidance: a simplified polyline
/// (closed for islands, open for the mainland) plus its bounding box
/// for a cheap reject.
#[derive(Clone)]
pub struct Obstacle {
    pub pts: Vec<(f64, f64)>,
    pub closed: bool,
    pub bbox: (f64, f64, f64, f64), // xmin, xmax, ymin, ymax
}

impl Obstacle {
    pub fn new(pts: Vec<(f64, f64)>, closed: bool) -> Self {
        let mut bbox = (f64::INFINITY, f64::NEG_INFINITY, f64::INFINITY, f64::NEG_INFINITY);
        for &(x, y) in &pts {
            bbox.0 = bbox.0.min(x);
            bbox.1 = bbox.1.max(x);
            bbox.2 = bbox.2.min(y);
            bbox.3 = bbox.3.max(y);
        }
        Obstacle { pts, closed, bbox }
    }
}

/// Everything the autopilot needs to re-run the offline isochrone planner
/// from the boat's current position and time. Built once (the polar
/// measurement is expensive) and handed to the autopilot via
/// [`RouteAutopilot::enable_replanning`].
pub struct ReplanContext {
    polar: Polar,
    chart: Option<Chart>,
    /// Direction the wind blows FROM (math angle, rad) used for planning.
    /// The nominal/mean wind, not the gusting instantaneous value.
    wind_from: f64,
    /// Re-route once a tidal gate has been held continuously this long (s).
    after_wait_s: f64,
    dest_radius: f64,
    /// When the boat first started waiting at the current gate. Reset when
    /// it stops waiting or after a replan fires.
    wait_started_t: Option<f64>,
}

impl ReplanContext {
    pub fn new(
        polar: Polar,
        chart: Option<Chart>,
        wind_from: f64,
        after_wait_s: f64,
        dest_radius: f64,
    ) -> Self {
        Self { polar, chart, wind_from, after_wait_s, dest_radius, wait_started_t: None }
    }
}

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
    /// Simplified coastline obstacles for reactive land avoidance. Empty
    /// → no avoidance (open-water-only runs, all unit tests).
    obstacles: Vec<Obstacle>,
    /// When present, re-route from the current position if a tidal gate is
    /// held past its window instead of waiting it out. `None` → gates
    /// behave as before (wait for the fair stream).
    replan: Option<ReplanContext>,
}

impl RouteAutopilot {
    pub fn new(
        cfg: &Config,
        route: Route,
        control_period_s: f64,
        sail_resample_period_s: f64,
        crab_enabled: bool,
        forecast: TideForecast,
        obstacles: Vec<Obstacle>,
    ) -> Self {
        Self {
            follower: RouteFollower::new(route),
            heading_controller: new_from_config(cfg, control_period_s),
            sail_stretching: cfg.boat.sail.stretching,
            sail_resample_period_s,
            last_sail_mag: 0.0,
            last_sail_t: None,
            sail_side: 1.0,
            crab_enabled,
            forecast,
            obstacles,
            replan: None,
        }
    }

    /// Enable mid-mission re-routing on a held tidal gate (off by default).
    pub fn enable_replanning(&mut self, ctx: ReplanContext) {
        self.replan = Some(ctx);
    }

    /// If the follower is parked at a tidal gate and has been for longer
    /// than `after_wait_s`, re-run the planner from the current position
    /// and time and swap in the result. Returns `true` if the route was
    /// replaced (the caller should re-query the follower for this tick's
    /// heading). No-op when replanning is disabled or the boat isn't
    /// waiting.
    fn maybe_replan(&mut self, t: f64, pos: (f64, f64)) -> bool {
        let Some(ctx) = self.replan.as_mut() else { return false };
        if !self.follower.waiting_at_gate() {
            ctx.wait_started_t = None;
            return false;
        }
        let started = *ctx.wait_started_t.get_or_insert(t);
        if t - started < ctx.after_wait_s {
            return false;
        }
        // Plan from here to the route's destination, starting at the
        // current wall-clock time so the tide forecast phase lines up.
        let dest = self.follower.destination();
        let pc = PlanConfig {
            start: pos,
            dest,
            wind_from: ctx.wind_from,
            start_time: t,
            dt: 600.0,
            heading_step_deg: 5.0,
            cross_track_bucket_m: 500.0,
            max_steps: 800,
            dest_radius: ctx.dest_radius,
        };
        match plan(&pc, &ctx.polar, &self.forecast, ctx.chart.as_ref()) {
            Some(result) => {
                let pts = simplify(&result.path, 250.0);
                if pts.len() >= 2 {
                    self.follower.replace_remaining(&pts);
                }
                // Whether or not the path was usable, stand down the timer
                // so we don't hammer the planner every tick.
                ctx.wait_started_t = None;
                pts.len() >= 2
            }
            None => {
                // Couldn't reach: keep waiting, but don't retry until the
                // window has elapsed again.
                ctx.wait_started_t = Some(t);
                false
            }
        }
    }

    /// If steering `heading` from `pos` would run the boat into a coastline
    /// obstacle within `LAND_LOOKAHEAD_M`, return the nearest clear heading
    /// (smallest symmetric deflection that clears). Otherwise return
    /// `heading` unchanged. No obstacles → no-op.
    fn avoid_land(&self, pos: (f64, f64), heading: f64) -> f64 {
        if self.obstacles.is_empty() || !self.heading_hits_land(pos, heading) {
            return heading;
        }
        let step = 10.0_f64.to_radians();
        let max = 100.0_f64.to_radians();
        let mut delta = step;
        while delta <= max + 1e-9 {
            // Prefer the smaller-magnitude deflection; try both sides at
            // each step so we turn the least amount that clears.
            for &sign in &[1.0_f64, -1.0] {
                let h = heading + sign * delta;
                if !self.heading_hits_land(pos, h) {
                    return h;
                }
            }
            delta += step;
        }
        // Boxed in within ±100°: hold the commanded heading rather than
        // spin; the controller's other terms still apply.
        heading
    }

    /// True if the lookahead segment from `pos` along `heading` crosses any
    /// obstacle polyline. Bounding-box pre-filter keeps open water cheap.
    fn heading_hits_land(&self, pos: (f64, f64), heading: f64) -> bool {
        let (x0, y0) = pos;
        let x1 = x0 + LAND_LOOKAHEAD_M * heading.cos();
        let y1 = y0 + LAND_LOOKAHEAD_M * heading.sin();
        let (sxmin, sxmax) = (x0.min(x1), x0.max(x1));
        let (symin, symax) = (y0.min(y1), y0.max(y1));
        for ob in &self.obstacles {
            let (bxmin, bxmax, bymin, bymax) = ob.bbox;
            if sxmax < bxmin || sxmin > bxmax || symax < bymin || symin > bymax {
                continue;
            }
            let n = ob.pts.len();
            if n < 2 {
                continue;
            }
            let edges = if ob.closed { n } else { n - 1 };
            for i in 0..edges {
                let a = ob.pts[i];
                let b = ob.pts[(i + 1) % n];
                if segments_intersect(x0, y0, x1, y1, a.0, a.1, b.0, b.1) {
                    return true;
                }
            }
        }
        false
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

/// Proper segment-segment intersection test (excludes collinear touching).
fn segments_intersect(
    ax: f64, ay: f64, bx: f64, by: f64,
    cx: f64, cy: f64, dx: f64, dy: f64,
) -> bool {
    let d1 = cross3(cx, cy, dx, dy, ax, ay);
    let d2 = cross3(cx, cy, dx, dy, bx, by);
    let d3 = cross3(ax, ay, bx, by, cx, cy);
    let d4 = cross3(ax, ay, bx, by, dx, dy);
    ((d1 > 0.0 && d2 < 0.0) || (d1 < 0.0 && d2 > 0.0))
        && ((d3 > 0.0 && d4 < 0.0) || (d3 < 0.0 && d4 > 0.0))
}

/// Cross product (p2-p1) x (p3-p1).
fn cross3(p1x: f64, p1y: f64, p2x: f64, p2y: f64, p3x: f64, p3y: f64) -> f64 {
    (p2x - p1x) * (p3y - p1y) - (p2y - p1y) * (p3x - p1x)
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

        let mut desired = match self
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

        // If we're parked at a tidal gate past its window, re-route from
        // here rather than wait it out (opt-in). A successful replan swaps
        // the follower's route, so re-query it for this tick's heading.
        if self.maybe_replan(obs.t, (obs.pos_x, obs.pos_y)) {
            if let Some(h) =
                self.follower
                    .update(obs.t, obs.pos_x, obs.pos_y, tw, obs.current, &self.forecast)
            {
                desired = h;
            }
        }

        // Reactive land avoidance: deflect the desired course over ground
        // away from any coastline within the lookahead before it's handed
        // to crab/controller. The follower has no chart awareness; a fast
        // (e.g. tide-assisted) crossing can arrive at a mark with the set
        // pushing it onto the lee shore, and the planner's land avoidance
        // only shapes the offline route, not the closed loop.
        let desired = self.avoid_land((obs.pos_x, obs.pos_y), desired);

        // Crab compensation: treat the follower's output as the desired
        // course over ground and offset the commanded heading so the
        // tide doesn't set the boat off track. Only when steering
        // directly — while tacking the heading is wind-relative and
        // already as high as the boat can point, so crabbing it would be
        // wrong; the cross-track term handles set over successive tacks.
        // While station-keeping at a gate the follower already returns a
        // water-frame heading (it stems the current itself), so crab would
        // double-compensate — skip it.
        let has_current = obs.current.0 != 0.0 || obs.current.1 != 0.0;
        let heading_ref = if self.crab_enabled
            && has_current
            && !self.follower.waiting_at_gate()
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
            waypoints: vec![Waypoint { x: 0.0, y: 0.0, gate: false, soft: false }, Waypoint { x: 0.0, y: 100.0, gate: false, soft: false }],
            gate_open_along_current: 0.0,
            gate_lead_time_s: 0.0,
            gate_min_fair_window_s: 0.0,
            fly_by_radius: 0.0,
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
    fn held_gate_triggers_replan_and_swaps_route() {
        use crate::planner::Polar;
        // North-bound route whose start is a tidal gate; destination is
        // far enough that the planner emits a real (tacking) path.
        let mut route = straight_north_route();
        route.waypoints = vec![
            Waypoint { x: 0.0, y: 0.0, gate: true, soft: false },
            Waypoint { x: 0.0, y: 2000.0, gate: false, soft: false },
        ];
        let mut ap = RouteAutopilot::new(
            &cfg(),
            route,
            0.3,
            2.0,
            false,
            TideForecast::None,
            vec![],
        );
        // Flat polar: can't point below 45° but sails 1 m/s otherwise.
        let polar = Polar::new(vec![(45.0, 1.0), (90.0, 1.0), (135.0, 1.0), (180.0, 1.0)]);
        // Wind from the north (+y) → destination is dead upwind, forcing a
        // tacked plan. Re-route the instant the gate is found held. The
        // 400 m capture radius matches the planner's 600 m step granularity
        // (a tighter radius the coarse front would overshoot).
        ap.enable_replanning(ReplanContext::new(polar, None, PI / 2.0, 0.0, 400.0));

        // Foul (southward) tide at the gate → the follower would normally
        // park here; replanning should instead swap in a fresh route.
        let o = Observation {
            t: 10.0,
            pos_x: 0.0,
            pos_y: 0.0,
            heading: PI / 2.0,
            yaw_rate: 0.0,
            roll: 0.0,
            vel_x_body: 0.5,
            vel_y_body: 0.0,
            true_wind: WindReading { direction: -PI / 2.0, speed: 5.0 },
            current: (0.0, -0.6),
        };
        let cmd = ap.step(&o);
        let wps = &ap.route().waypoints;
        assert!(!wps[0].gate, "the gate should be gone after re-routing");
        assert!(wps.len() >= 3, "tacked replan emits intermediate waypoints, got {}", wps.len());
        let last = wps.last().unwrap();
        assert_eq!((last.x, last.y), (0.0, 2000.0), "destination preserved");
        assert!(!cmd.mission_complete);
        assert!(cmd.rudder_angle.is_finite());
    }

    #[test]
    fn autopilot_signals_mission_complete_when_route_done() {
        let mut ap = RouteAutopilot::new(&cfg(), straight_north_route(), 0.3, 2.0, true, TideForecast::None, vec![]);
        // Step once well inside acceptance radius of the only target.
        let cmd = ap.step(&obs(0.0, 0.0, 99.0));
        assert!(cmd.mission_complete, "captured final waypoint should end mission");
    }

    #[test]
    fn avoid_land_deflects_around_obstacle_and_passes_clear_water() {
        // A wall straight ahead (east) of the boat at x = 100, spanning
        // y ∈ [-200, 200] — wide enough that the 300 m lookahead pointed
        // due east hits it.
        let wall = Obstacle::new(
            vec![(100.0, -200.0), (100.0, 200.0)],
            false,
        );
        let ap = RouteAutopilot::new(
            &cfg(),
            straight_north_route(),
            0.3,
            2.0,
            true,
            TideForecast::None,
            vec![wall],
        );
        // Heading due east (0 rad) from the origin runs into the wall.
        assert!(ap.heading_hits_land((0.0, 0.0), 0.0));
        let deflected = ap.avoid_land((0.0, 0.0), 0.0);
        assert!((deflected - 0.0).abs() > 1e-6, "should deflect off the wall");
        assert!(!ap.heading_hits_land((0.0, 0.0), deflected), "deflected heading must clear");
        // Heading due north (π/2) runs parallel to the wall and never hits
        // it → left unchanged.
        let clear = std::f64::consts::PI / 2.0;
        assert!(!ap.heading_hits_land((0.0, 0.0), clear));
        assert_eq!(ap.avoid_land((0.0, 0.0), clear), clear);
    }

    #[test]
    fn avoid_land_is_noop_without_obstacles() {
        let ap = RouteAutopilot::new(
            &cfg(),
            straight_north_route(),
            0.3,
            2.0,
            true,
            TideForecast::None,
            vec![],
        );
        assert_eq!(ap.avoid_land((0.0, 0.0), 1.234), 1.234);
        assert!(!ap.heading_hits_land((0.0, 0.0), 0.0));
    }

    #[test]
    fn autopilot_emits_finite_rudder_and_sail() {
        let mut ap = RouteAutopilot::new(&cfg(), straight_north_route(), 0.3, 2.0, true, TideForecast::None, vec![]);
        let cmd = ap.step(&obs(0.0, 0.0, 0.0));
        assert!(cmd.rudder_angle.is_finite());
        assert!(cmd.sail_angle.is_finite());
        assert!(!cmd.mission_complete);
    }
}
