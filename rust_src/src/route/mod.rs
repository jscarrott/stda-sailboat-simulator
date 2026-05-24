use anyhow::{Context, Result};
use serde::Deserialize;
use std::f64::consts::PI;
use std::fs::File;
use std::path::Path;

use crate::physics::wind::TrueWind;

#[derive(Deserialize, Clone, Copy, Debug)]
pub struct Waypoint {
    pub x: f64,
    pub y: f64,
}

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
}

impl RouteFollower {
    pub fn new(route: Route) -> Self {
        Self {
            route,
            leg_index: 0,
            tack: Tack::None,
            last_tack_change_t: f64::NEG_INFINITY,
            finished: false,
        }
    }

    pub fn finished(&self) -> bool {
        self.finished
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
    /// route is complete (and not looping).
    pub fn update(&mut self, t: f64, pos_x: f64, pos_y: f64, true_wind: TrueWind) -> Option<f64> {
        if self.finished {
            return None;
        }

        // ----- Capture -----
        let target = self.route.waypoints[self.leg_index + 1];
        let dist_to_target = ((target.x - pos_x).powi(2) + (target.y - pos_y).powi(2)).sqrt();
        if dist_to_target < self.route.acceptance_radius {
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
            // Goal is outside the no-go cone — direct steering.
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
            waypoints: vec![Waypoint { x: ax, y: ay }, Waypoint { x: bx, y: by }],
            loop_route: false,
        }
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
        let h = rf.update(0.0, 50.0, -5.0, wind).expect("not finished");
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
        let h0 = rf.update(0.0, 0.0, 0.0, wind).expect("not finished");
        // Both VMGs equal (cos ±π/4) → preferred=Port; cand_port = π/2 + π/4 = 3π/4.
        assert!((h0 - 3.0 * PI / 4.0).abs() < 1e-9, "got {}", h0);
        assert_eq!(rf.current_tack(), Tack::Port);
        // Holding the same position should hold the same tack (no chatter).
        for step in 1..10 {
            let t = step as f64 * 0.3;
            let h = rf.update(t, 0.0, 0.0, wind).expect("not finished");
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
        rf.update(0.0, 0.0, 0.0, wind);
        assert_eq!(rf.current_tack(), Tack::Port);
        // Now boat is at (-40, 60). Bearing to (0,200) = atan2(140, 40) ≈ 1.292 rad.
        // off_wind = wrap_pi(1.292 - π/2) ≈ -0.279 rad → still in no-go (|.|<π/4).
        // VMG: cand_port (3π/4=2.356) vs bearing 1.292 → cos(1.064) ≈ 0.487.
        //      cand_starboard (π/4=0.785) vs bearing 1.292 → cos(-0.507) ≈ 0.874.
        // Preferred = Starboard. Should switch since min_tack_duration_s=3 elapsed at t=10.
        let h = rf.update(10.0, -40.0, 60.0, wind).expect("not finished");
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
                Waypoint { x: 0.0, y: 0.0 },
                Waypoint { x: 50.0, y: 0.0 },
                Waypoint { x: 100.0, y: 0.0 },
            ],
            loop_route: false,
        };
        let mut rf = RouteFollower::new(route);
        let wind = wind_from_deg(5.0, 90.0); // crosswind, no tacking
        // Before capture: leg 0→1.
        rf.update(0.0, 0.0, 0.0, wind);
        assert_eq!(rf.leg_index, 0);
        // Step into the acceptance radius of waypoint 1.
        rf.update(1.0, 48.0, 0.0, wind);
        assert_eq!(rf.leg_index, 1);
        assert!(!rf.finished());
        // Capture the final waypoint.
        let h = rf.update(2.0, 98.0, 0.0, wind);
        assert!(rf.finished());
        assert!(h.is_none());
    }
}
