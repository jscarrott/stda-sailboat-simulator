use anyhow::{Context, Result};
use serde::Deserialize;
use std::f64::consts::PI;
use std::fs::File;
use std::path::Path;

use crate::current_model::TideForecast;
use crate::physics::wind::TrueWind;

#[derive(Deserialize, Clone, Copy, Debug)]
pub struct Waypoint {
    pub x: f64,
    pub y: f64,
    /// Tidal gate: when the boat reaches this waypoint it loiters here
    /// until the tidal stream turns fair for the *next* leg (see
    /// `Route::gate_open_along_current`), then proceeds. Ignored on the
    /// final waypoint (no next leg). Defaults to false.
    #[serde(default)]
    pub gate: bool,
    /// Fly-by (soft) waypoint: the boat doesn't have to hit it, only
    /// pass it. The leg advances as soon as the boat crosses the
    /// waypoint's perpendicular line (along-track progress past it), so
    /// it never doubles back to nail a point the tide pushed it past and
    /// it cuts the corner onto the next leg. Steering stays LOS+XTE
    /// (robust against a foul set); actively exploiting a favourable
    /// tide is a separate routing step. Hard waypoints (default) need
    /// the acceptance circle.
    #[serde(default)]
    pub soft: bool,
}

/// Wind override matching the Python `sim_params_config.yaml`
/// convention: `direction_deg` is the *math-coords* angle of the wind
/// **velocity vector**, i.e. the direction the wind is blowing TOWARD
/// (0° = +x, 90° = +y). The reciprocal — the direction the wind comes
/// from — is `direction_deg + 180`. To produce a leg directly upwind,
/// set `direction_deg` to the bearing of the leg + 180°.
#[derive(Deserialize, Clone, Copy, Debug)]
pub struct WindOverride {
    pub direction_deg: f64,
    pub speed: f64,
}

impl WindOverride {
    pub fn to_true_wind(self) -> TrueWind {
        let dir_rad = self.direction_deg.to_radians();
        TrueWind {
            x: self.speed * dir_rad.cos(),
            y: self.speed * dir_rad.sin(),
            strength: self.speed,
            direction: self.direction_deg,
        }
    }
}

fn default_close_hauled() -> f64 {
    45.0
}
fn default_xte_lookahead() -> f64 {
    15.0
}
fn default_min_tack_duration() -> f64 {
    3.0
}
fn default_loop() -> bool {
    false
}

#[derive(Deserialize, Clone, Debug)]
pub struct Route {
    pub name: String,
    pub acceptance_radius: f64,
    #[serde(default = "default_close_hauled")]
    pub close_hauled_angle_deg: f64,
    #[serde(default = "default_xte_lookahead")]
    pub xte_lookahead: f64,
    #[serde(default = "default_min_tack_duration")]
    pub min_tack_duration_s: f64,
    #[serde(default)]
    pub wind: Option<WindOverride>,
    /// Minimum along-next-leg tidal-current component (m/s) for a tidal
    /// gate to open. 0.0 = wait until the stream is neutral-to-fair; a
    /// positive value demands a fair tide of at least that strength.
    #[serde(default)]
    pub gate_open_along_current: f64,
    /// Forecast lead time (s) for tidal gates. The gate opens when the
    /// *forecast* stream `gate_lead_time_s` from now will be fair, so
    /// the boat starts moving early and is up to speed as the fair tide
    /// arrives. 0 = react to the present current (needs a forecastable
    /// current model — see `CurrentModel::forecaster`).
    #[serde(default)]
    pub gate_lead_time_s: f64,
    /// Required fair-tide window (s) for a tidal gate to open: the
    /// forecast must stay fair (along-leg current >= threshold) for at
    /// least this long from the lead-adjusted open time, so the boat
    /// isn't released onto a leg the tide will turn foul on mid-way.
    /// 0 = only check the single lead-adjusted instant (no window).
    /// Needs a forecastable current model to have any effect.
    #[serde(default)]
    pub gate_min_fair_window_s: f64,
    /// Turn-anticipation radius (m) for fly-by (soft) waypoints: the leg
    /// advances as soon as the boat is within this distance of a soft
    /// waypoint, so it cuts the corner onto the next leg instead of
    /// sailing all the way to the point. 0 = no anticipation (advance
    /// only on along-track pass or the acceptance circle). Ignored for
    /// hard waypoints.
    #[serde(default)]
    pub fly_by_radius: f64,
    pub waypoints: Vec<Waypoint>,
    #[serde(rename = "loop", default = "default_loop")]
    pub loop_route: bool,
}

impl Route {
    pub fn load(path: &Path) -> Result<Self> {
        let file = File::open(path).with_context(|| format!("opening route {}", path.display()))?;
        let r: Route =
            serde_yaml::from_reader(file).with_context(|| format!("parsing route {}", path.display()))?;
        anyhow::ensure!(r.waypoints.len() >= 2, "route {} needs >= 2 waypoints", r.name);
        Ok(r)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Tack {
    None,
    Port,
    Starboard,
}

/// Stateful route follower.
///
/// Per-step decisions:
/// - Capture: when within `acceptance_radius` of the next waypoint, advance.
/// - Direct steering: when the bearing TO the next waypoint sits outside the
///   no-go cone (`±close_hauled_angle` around wind-from), return LOS+XTE
///   `chi_path + atan2(-xte, xte_lookahead)`.
/// - Tacking: when the bearing is inside the no-go cone, pick the tack with
///   better velocity-made-good to the target. Switch tacks only when
///   `min_tack_duration_s` has elapsed since the last change.
///
/// **Tack switching deliberately uses bearing-to-target VMG rather than the
/// LOS heading.** A directly-upwind leg with the boat exactly on the rhumb
/// line gives chi_los == wind_from, which leaves both close-hauled
/// candidates equidistant — VMG-on-bearing breaks the symmetry as soon as
/// the boat drifts laterally, producing a natural zigzag.
pub struct RouteFollower {
    route: Route,
    leg_index: usize,
    tack: Tack,
    last_tack_change_t: f64,
    finished: bool,
    waiting_at_gate: bool,
    departed: bool,
}

impl RouteFollower {
    pub fn new(route: Route) -> Self {
        Self {
            route,
            leg_index: 0,
            tack: Tack::None,
            last_tack_change_t: f64::NEG_INFINITY,
            finished: false,
            waiting_at_gate: false,
            departed: false,
        }
    }

    pub fn finished(&self) -> bool {
        self.finished
    }

    /// True while loitering at a tidal gate waiting for a fair stream.
    pub fn waiting_at_gate(&self) -> bool {
        self.waiting_at_gate
    }

    /// Along-next-leg component of `current` at a gate waypoint
    /// `gate_idx` (m/s, positive = fair). The "next leg" runs from the
    /// gate to the waypoint after it.
    fn gate_along_current(&self, gate_idx: usize, current: (f64, f64)) -> f64 {
        let a = self.route.waypoints[gate_idx];
        let b = self.route.waypoints[gate_idx + 1];
        let dx = b.x - a.x;
        let dy = b.y - a.y;
        let len = dx.hypot(dy).max(1e-9);
        (current.0 * dx + current.1 * dy) / len
    }

    /// Whether the tidal gate before the leg `from_idx -> from_idx+1`
    /// should open at time `t`. The forecast must show the along-leg
    /// current staying >= `gate_open_along_current` for the whole
    /// `gate_min_fair_window_s`, sampled from `t + gate_lead_time_s`.
    /// With no forecast it falls back to the present `current`; with a
    /// zero window it's a single-instant check.
    fn gate_open_for_leg(
        &self,
        from_idx: usize,
        t: f64,
        current: (f64, f64),
        forecast: &TideForecast,
    ) -> bool {
        let lead = self.route.gate_lead_time_s;
        let window = self.route.gate_min_fair_window_s.max(0.0);
        let threshold = self.route.gate_open_along_current;
        const N: usize = 12;
        for k in 0..=N {
            let ts = t + lead + window * (k as f64) / (N as f64);
            let cur = forecast.at(ts).unwrap_or(current);
            if self.gate_along_current(from_idx, cur) < threshold {
                return false;
            }
        }
        true
    }

    pub fn route(&self) -> &Route {
        &self.route
    }

    pub fn current_leg(&self) -> Option<(Waypoint, Waypoint)> {
        if self.finished || self.leg_index + 1 >= self.route.waypoints.len() {
            None
        } else {
            Some((self.route.waypoints[self.leg_index], self.route.waypoints[self.leg_index + 1]))
        }
    }

    pub fn current_tack(&self) -> Tack {
        self.tack
    }

    /// Returns the desired heading (rad) for this step, or `None` when the
    /// route is complete (and not looping). `current` is the tidal stream
    /// (global east/north m/s), used only for tidal-gate decisions.
    pub fn update(
        &mut self,
        t: f64,
        pos_x: f64,
        pos_y: f64,
        true_wind: TrueWind,
        current: (f64, f64),
        forecast: &TideForecast,
    ) -> Option<f64> {
        if self.finished {
            return None;
        }

        // ----- Departure gate -----
        // If the first waypoint is a tidal gate, hold near the start
        // until the first leg's fair-tide window opens — forecast-aware
        // departure planning. Loiter by steering back at the start
        // waypoint (so the boat station-keeps rather than being swept
        // off, as far as it can out-sail the stream).
        if !self.departed {
            let start = self.route.waypoints[0];
            if start.gate
                && self.route.waypoints.len() >= 2
                && !self.gate_open_for_leg(0, t, current, forecast)
            {
                self.waiting_at_gate = true;
                self.tack = Tack::None;
                return Some((start.y - pos_y).atan2(start.x - pos_x));
            }
            self.departed = true;
            self.waiting_at_gate = false;
        }

        // ----- Capture -----
        let prev_wp = self.route.waypoints[self.leg_index];
        let target = self.route.waypoints[self.leg_index + 1];
        let dist_to_target = ((target.x - pos_x).powi(2) + (target.y - pos_y).powi(2)).sqrt();
        // Fly-by: a soft waypoint is "captured" the moment the boat
        // crosses its perpendicular line (along-track progress past it),
        // so it never doubles back to nail a point the tide pushed it
        // past. Hard waypoints need the acceptance circle.
        let passed_soft = target.soft && {
            let lvx = target.x - prev_wp.x;
            let lvy = target.y - prev_wp.y;
            let leg_len = lvx.hypot(lvy).max(1e-9);
            ((pos_x - prev_wp.x) * lvx + (pos_y - prev_wp.y) * lvy) / leg_len >= leg_len
        };
        // Soft waypoints also advance within the larger fly-by radius
        // (turn anticipation → corner cut); hard ones need the
        // acceptance circle.
        let capture_radius = if target.soft {
            self.route.acceptance_radius.max(self.route.fly_by_radius)
        } else {
            self.route.acceptance_radius
        };
        if dist_to_target < capture_radius || passed_soft {
            // Tidal gate: hold here until the stream turns fair for the
            // next leg. Loiter by steering back at the gate waypoint;
            // do not advance the leg until the gate opens. With a
            // forecast we probe the stream `gate_lead_time_s` ahead so
            // the gate opens early and the boat is moving as the fair
            // tide arrives; without one we fall back to the present.
            let target_idx = self.leg_index + 1;
            let is_gate = target.gate && target_idx + 1 < self.route.waypoints.len();
            if is_gate && !self.gate_open_for_leg(target_idx, t, current, forecast) {
                self.waiting_at_gate = true;
                self.tack = Tack::None;
                return Some((target.y - pos_y).atan2(target.x - pos_x));
            }
            self.waiting_at_gate = false;

            self.leg_index += 1;
            if self.leg_index + 1 >= self.route.waypoints.len() {
                if self.route.loop_route {
                    self.leg_index = 0;
                } else {
                    self.finished = true;
                    return None;
                }
            }
            // A captured waypoint usually means we want to re-evaluate
            // tack from scratch on the new leg.
            self.tack = Tack::None;
        }

        let prev = self.route.waypoints[self.leg_index];
        let next = self.route.waypoints[self.leg_index + 1];

        // ----- LOS + XTE direct heading -----
        let dx = next.x - prev.x;
        let dy = next.y - prev.y;
        let leg_len = dx.hypot(dy);
        let chi_path = dy.atan2(dx);
        let xte = ((pos_x - prev.x) * (-dy) + (pos_y - prev.y) * dx) / leg_len;
        let chi_los = wrap_pi(chi_path + (-xte).atan2(self.route.xte_lookahead));

        // ----- No-go decision based on bearing-to-target -----
        let bearing_to_target = (next.y - pos_y).atan2(next.x - pos_x);
        let wind_from = wrap_pi(true_wind.y.atan2(true_wind.x) + PI);
        let close_hauled_rad = self.route.close_hauled_angle_deg.to_radians();
        let off_wind = wrap_pi(bearing_to_target - wind_from);
        let in_no_go = off_wind.abs() < close_hauled_rad;

        if !in_no_go {
            // Goal is outside the no-go cone — direct steering (LOS+XTE).
            self.tack = Tack::None;
            return Some(chi_los);
        }

        // ----- In no-go: pick tack by VMG-to-target -----
        let cand_port = wrap_pi(wind_from + close_hauled_rad);
        let cand_starboard = wrap_pi(wind_from - close_hauled_rad);
        let vmg_port = wrap_pi(cand_port - bearing_to_target).cos();
        let vmg_starboard = wrap_pi(cand_starboard - bearing_to_target).cos();
        let preferred = if vmg_port >= vmg_starboard {
            Tack::Port
        } else {
            Tack::Starboard
        };

        let elapsed = t - self.last_tack_change_t;
        if self.tack == Tack::None {
            self.tack = preferred;
            self.last_tack_change_t = t;
        } else if self.tack != preferred && elapsed >= self.route.min_tack_duration_s {
            self.tack = preferred;
            self.last_tack_change_t = t;
        }

        let heading = match self.tack {
            Tack::Port => cand_port,
            Tack::Starboard => cand_starboard,
            Tack::None => unreachable!("just latched above"),
        };
        Some(heading)
    }
}

fn wrap_pi(x: f64) -> f64 {
    let two_pi = 2.0 * PI;
    let y = (x + PI).rem_euclid(two_pi) - PI;
    if y == -PI {
        PI
    } else {
        y
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn wind_from_deg(speed: f64, from_deg: f64) -> TrueWind {
        // Velocity vector points OPPOSITE to "from" direction.
        let from_rad = from_deg.to_radians();
        TrueWind {
            x: -speed * from_rad.cos(),
            y: -speed * from_rad.sin(),
            strength: speed,
            direction: from_deg,
        }
    }

    fn straight_route(name: &str, ax: f64, ay: f64, bx: f64, by: f64) -> Route {
        Route {
            name: name.into(),
            acceptance_radius: 5.0,
            close_hauled_angle_deg: 45.0,
            xte_lookahead: 15.0,
            min_tack_duration_s: 3.0,
            wind: None,
            waypoints: vec![Waypoint { x: ax, y: ay, gate: false, soft: false }, Waypoint { x: bx, y: by, gate: false, soft: false }],
            gate_open_along_current: 0.0,
            gate_lead_time_s: 0.0,
            gate_min_fair_window_s: 0.0,
            fly_by_radius: 0.0,
            loop_route: false,
        }
    }

    #[test]
    fn tidal_gate_holds_until_stream_turns_fair() {
        // 3 waypoints west->east: leg2 (wp1->wp2) runs due east (+x).
        // wp1 is a tidal gate; it should hold until the current has a
        // positive eastward (along-leg2) component.
        let mut route = straight_route("gated", 0.0, 0.0, 100.0, 0.0);
        route.waypoints.push(Waypoint { x: 200.0, y: 0.0, gate: false, soft: false });
        route.waypoints[1].gate = true; // gate at (100,0); next leg is +x
        let wind = wind_from_deg(5.0, 90.0); // crosswind, no tacking
        let mut rf = RouteFollower::new(route);

        // Arrive at the gate with a foul (westward) current → must hold,
        // not advance: still steering toward wp1, flagged waiting.
        let h = rf
            .update(0.0, 98.0, 0.0, wind, (-0.6, 0.0), &TideForecast::None)
            .expect("not finished");
        assert!(rf.waiting_at_gate(), "should wait at gate in foul tide");
        assert_eq!(rf.leg_index, 0, "must not advance past the gate");
        assert!(h.abs() < 1e-9, "holds by pointing at the gate (due east)");

        // Tide turns fair (eastward) → gate opens, leg advances.
        rf.update(100.0, 98.0, 0.0, wind, (0.6, 0.0), &TideForecast::None);
        assert!(!rf.waiting_at_gate(), "gate should open on fair tide");
        assert_eq!(rf.leg_index, 1, "advanced onto the next leg");
    }

    #[test]
    fn tidal_gate_forecast_lead_opens_early() {
        // Same gate (next leg due east, +x). Reversing E/W stream with a
        // 100 s period: at t=0 it's foul, but it turns fair ~25 s later.
        let mut route = straight_route("gated_fc", 0.0, 0.0, 100.0, 0.0);
        route.waypoints.push(Waypoint { x: 200.0, y: 0.0, gate: false, soft: false });
        route.waypoints[1].gate = true;
        route.gate_lead_time_s = 30.0; // look 30 s ahead
        let wind = wind_from_deg(5.0, 90.0);
        // Stream along +x: speed = 0.6·sin(2π t/100). Foul (negative) for
        // t in (50,100), fair for t in (0,50). At t=60 it's foul now but
        // forecast at t+30=90 is still foul → hold.
        let fc = TideForecast::Stream {
            peak_speed: 0.6,
            axis_rad: 0.0,
            period_s: 100.0,
            phase_rad: 0.0,
        };
        // present current at t=60 (foul) — but we pass the forecast.
        let mut rf = RouteFollower::new(route);
        let present = (0.6 * (2.0 * PI * 60.0 / 100.0).sin(), 0.0); // foul
        rf.update(60.0, 98.0, 0.0, wind, present, &fc);
        assert!(rf.waiting_at_gate(), "t=60: foul now and at t+30=90 → hold");

        // At t=80, forecast at t+30=110≡10 in next cycle is fair → open
        // early, before the present stream (still foul at t=80) turns.
        let present80 = (0.6 * (2.0 * PI * 80.0 / 100.0).sin(), 0.0); // foul
        assert!(present80.0 < 0.0, "sanity: present still foul at t=80");
        rf.update(80.0, 98.0, 0.0, wind, present80, &fc);
        assert!(!rf.waiting_at_gate(), "forecast lead opens the gate early");
        assert_eq!(rf.leg_index, 1);
    }

    #[test]
    fn tidal_gate_requires_long_enough_fair_window() {
        // Next leg due east. Stream along +x with a 100 s period: fair
        // (sin>0) only for t in (0,50) each cycle. Demand a 40 s fair
        // window. At t=40 it's fair NOW but turns foul at t=50 (only 10 s
        // left) → must hold. At t=2 the whole 0..40 window is fair → open.
        let mut route = straight_route("gated_win", 0.0, 0.0, 100.0, 0.0);
        route.waypoints.push(Waypoint { x: 200.0, y: 0.0, gate: false, soft: false });
        route.waypoints[1].gate = true;
        route.gate_min_fair_window_s = 40.0;
        let wind = wind_from_deg(5.0, 90.0);
        let fc = TideForecast::Stream { peak_speed: 0.6, axis_rad: 0.0, period_s: 100.0, phase_rad: 0.0 };

        let mut rf = RouteFollower::new(route);
        // t=40: fair now but window 40..80 goes foul → hold.
        rf.update(40.0, 98.0, 0.0, wind, fc.at(40.0).unwrap(), &fc);
        assert!(rf.waiting_at_gate(), "short remaining fair window → hold");
        // t=2: window 2..42 is (almost) all fair → open.
        rf.update(2.0, 98.0, 0.0, wind, fc.at(2.0).unwrap(), &fc);
        assert!(!rf.waiting_at_gate(), "full fair window ahead → open");
        assert_eq!(rf.leg_index, 1);
    }

    #[test]
    fn departure_gate_holds_at_start_until_fair() {
        // First waypoint is a gate; first leg runs due east (+x). Hold at
        // the start in a foul (westward) stream, release when it's fair.
        let mut route = straight_route("dep", 0.0, 0.0, 100.0, 0.0);
        route.waypoints[0].gate = true;
        let wind = wind_from_deg(5.0, 90.0);

        let mut rf = RouteFollower::new(route);
        rf.update(0.0, 0.0, 0.0, wind, (-0.5, 0.0), &TideForecast::None);
        assert!(rf.waiting_at_gate(), "foul tide at start → hold departure");
        assert_eq!(rf.leg_index, 0);

        rf.update(10.0, 0.0, 0.0, wind, (0.5, 0.0), &TideForecast::None);
        assert!(!rf.waiting_at_gate(), "fair tide → depart");
        // Once departed it follows the leg normally on subsequent ticks.
        rf.update(20.0, 1.0, 0.0, wind, (0.5, 0.0), &TideForecast::None);
        assert_eq!(rf.leg_index, 0); // still on the first leg, just sailing it
    }

    #[test]
    fn fly_by_advances_on_pass_not_circle() {
        // A(0,0) -> B(100,0) [soft] -> C(200,0). Crosswind (beam reach).
        let mut route = straight_route("flyby", 0.0, 0.0, 100.0, 0.0);
        route.waypoints.push(Waypoint { x: 200.0, y: 0.0, gate: false, soft: false });
        route.waypoints[1].soft = true;
        let wind = wind_from_deg(5.0, 90.0);
        let mut rf = RouteFollower::new(route);

        // Boat at (105, 30): 30 m off B (well outside the 5 m acceptance)
        // but past B along-track → a fly-by advances to the B->C leg
        // rather than doubling back to nail B.
        rf.update(0.0, 105.0, 30.0, wind, (0.0, 0.0), &TideForecast::None);
        assert_eq!(rf.leg_index, 1, "soft waypoint advances on along-track pass");

        // A hard waypoint in the same spot would NOT advance (still > 5 m
        // away, not within acceptance).
        let mut route2 = straight_route("hard", 0.0, 0.0, 100.0, 0.0);
        route2.waypoints.push(Waypoint { x: 200.0, y: 0.0, gate: false, soft: false });
        let mut rf2 = RouteFollower::new(route2);
        rf2.update(0.0, 105.0, 30.0, wind, (0.0, 0.0), &TideForecast::None);
        assert_eq!(rf2.leg_index, 0, "hard waypoint needs the acceptance circle");
    }

    #[test]
    fn wrap_pi_handles_edges() {
        assert!((wrap_pi(0.0) - 0.0).abs() < 1e-12);
        assert!((wrap_pi(PI) - PI).abs() < 1e-12);
        assert!((wrap_pi(-PI) - PI).abs() < 1e-12);
        assert!((wrap_pi(3.0 * PI) - PI).abs() < 1e-12);
        assert!((wrap_pi(-3.0 * PI) - PI).abs() < 1e-12);
        assert!((wrap_pi(0.5) - 0.5).abs() < 1e-12);
    }

    #[test]
    fn crosswind_uses_los_plus_xte() {
        // Route east, wind from north — beam reach, no tacking.
        let route = straight_route("crosswind", 0.0, 0.0, 100.0, 0.0);
        let mut rf = RouteFollower::new(route);
        let wind = wind_from_deg(5.0, 90.0);
        // Boat 5m south of the rhumb. chi_path=0, xte = -5.
        // chi_los = atan2(5, 15) ≈ 0.3217 rad ≈ +18° (turn north to recover).
        let h = rf.update(0.0, 50.0, -5.0, wind, (0.0, 0.0), &TideForecast::None).expect("not finished");
        let expected = (5.0_f64).atan2(15.0);
        assert!((h - expected).abs() < 1e-9, "got {} expected {}", h, expected);
        assert_eq!(rf.current_tack(), Tack::None);
    }

    #[test]
    fn upwind_latches_a_tack_without_chatter() {
        // Route due north, wind from north. Goal is directly upwind.
        let route = straight_route("upwind", 0.0, 0.0, 0.0, 100.0);
        let mut rf = RouteFollower::new(route);
        let wind = wind_from_deg(5.0, 90.0);
        // wind_from = atan2(velocity.y, velocity.x)+π
        //   velocity = (-5*cos90, -5*sin90) = (0, -5). atan2(-5, 0) = -π/2. +π = π/2. ✓
        // bearing_to_target from (0,0) to (0,100) = π/2.
        // off_wind = wrap_pi(π/2 - π/2) = 0 → in no-go.
        let h0 = rf.update(0.0, 0.0, 0.0, wind, (0.0, 0.0), &TideForecast::None).expect("not finished");
        // Both VMGs equal (cos ±π/4) → preferred=Port; cand_port = π/2 + π/4 = 3π/4.
        assert!((h0 - 3.0 * PI / 4.0).abs() < 1e-9, "got {}", h0);
        assert_eq!(rf.current_tack(), Tack::Port);
        // Holding the same position should hold the same tack (no chatter).
        for step in 1..10 {
            let t = step as f64 * 0.3;
            let h = rf.update(t, 0.0, 0.0, wind, (0.0, 0.0), &TideForecast::None).expect("not finished");
            assert!((h - 3.0 * PI / 4.0).abs() < 1e-9);
            assert_eq!(rf.current_tack(), Tack::Port);
        }
    }

    #[test]
    fn upwind_switches_tack_after_lateral_drift() {
        // Same upwind route, wind from north. Boat heads NW on Port tack,
        // ends up far left of rhumb — VMG argues for Starboard.
        let route = straight_route("upwind_switch", 0.0, 0.0, 0.0, 200.0);
        let mut rf = RouteFollower::new(route);
        let wind = wind_from_deg(5.0, 90.0);
        // Latch a tack at origin.
        rf.update(0.0, 0.0, 0.0, wind, (0.0, 0.0), &TideForecast::None);
        assert_eq!(rf.current_tack(), Tack::Port);
        // Now boat is at (-40, 60). Bearing to (0,200) = atan2(140, 40) ≈ 1.292 rad.
        // off_wind = wrap_pi(1.292 - π/2) ≈ -0.279 rad → still in no-go (|.|<π/4).
        // VMG: cand_port (3π/4=2.356) vs bearing 1.292 → cos(1.064) ≈ 0.487.
        //      cand_starboard (π/4=0.785) vs bearing 1.292 → cos(-0.507) ≈ 0.874.
        // Preferred = Starboard. Should switch since min_tack_duration_s=3 elapsed at t=10.
        let h = rf.update(10.0, -40.0, 60.0, wind, (0.0, 0.0), &TideForecast::None).expect("not finished");
        assert_eq!(rf.current_tack(), Tack::Starboard);
        assert!((h - PI / 4.0).abs() < 1e-9, "got {}", h);
    }

    #[test]
    fn capture_advances_leg_and_then_finishes() {
        let route = Route {
            name: "triangle".into(),
            acceptance_radius: 5.0,
            close_hauled_angle_deg: 45.0,
            xte_lookahead: 15.0,
            min_tack_duration_s: 3.0,
            wind: None,
            waypoints: vec![
                Waypoint { x: 0.0, y: 0.0, gate: false, soft: false },
                Waypoint { x: 50.0, y: 0.0, gate: false, soft: false },
                Waypoint { x: 100.0, y: 0.0, gate: false, soft: false },
            ],
            gate_open_along_current: 0.0,
            gate_lead_time_s: 0.0,
            gate_min_fair_window_s: 0.0,
            fly_by_radius: 0.0,
            loop_route: false,
        };
        let mut rf = RouteFollower::new(route);
        let wind = wind_from_deg(5.0, 90.0); // crosswind, no tacking
        // Before capture: leg 0→1.
        rf.update(0.0, 0.0, 0.0, wind, (0.0, 0.0), &TideForecast::None);
        assert_eq!(rf.leg_index, 0);
        // Step into the acceptance radius of waypoint 1.
        rf.update(1.0, 48.0, 0.0, wind, (0.0, 0.0), &TideForecast::None);
        assert_eq!(rf.leg_index, 1);
        assert!(!rf.finished());
        // Capture the final waypoint.
        let h = rf.update(2.0, 98.0, 0.0, wind, (0.0, 0.0), &TideForecast::None);
        assert!(rf.finished());
        assert!(h.is_none());
    }
}
