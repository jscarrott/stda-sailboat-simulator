//! Offline isochrone route planner.
//!
//! Given a start and destination, the boat's speed polar, a (uniform)
//! wind, a tidal-current forecast and the coastline chart, this finds a
//! near time-optimal path through the time-varying wind+tide field and
//! emits it as a fly-by route the existing follower can sail.
//!
//! Method (classic isochrone): expand a front of reachable points by
//! Δt at a time. From each front point fan over headings; the boat's
//! through-water speed comes from the polar at that true-wind angle and
//! the tide (forecast at that point's time) advects it. Segments that
//! cross land are rejected. The candidate cloud is pruned to an
//! isochrone by bucketing on cross-track offset from the start→dest
//! line and keeping the furthest-along point per bucket. Stop when the
//! destination is within a step's reach; backtrack the parent chain.
//!
//! The plan is only as good as the (uncalibrated) polar — it optimises
//! the model, not reality. But it routes *through* the field rather
//! than at fixed marks, so it avoids the dead-upwind-mark capture
//! problem by construction and exploits a fair tide / good pressure.

use std::f64::consts::PI;

use crate::chart::Chart;
use crate::current_model::TideForecast;

/// Boat speed polar: through-water speed (m/s) vs true wind angle
/// (degrees, 0 = head to wind, 180 = dead run). Below the lowest
/// sampled TWA the boat can't point, so speed is 0.
pub struct Polar {
    twa_deg: Vec<f64>,
    speed: Vec<f64>,
}

impl Polar {
    pub fn new(mut samples: Vec<(f64, f64)>) -> Self {
        samples.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
        Polar {
            twa_deg: samples.iter().map(|s| s.0).collect(),
            speed: samples.iter().map(|s| s.1).collect(),
        }
    }

    /// Interpolated speed at a true wind angle (degrees, folded to
    /// [0,180]). Below the lowest sampled angle → 0 (in the no-go zone).
    pub fn speed_at(&self, twa_deg: f64) -> f64 {
        let a = twa_deg.abs().min(180.0);
        if self.twa_deg.is_empty() || a < self.twa_deg[0] {
            return 0.0;
        }
        let n = self.twa_deg.len();
        if a >= self.twa_deg[n - 1] {
            return self.speed[n - 1];
        }
        let i = self.twa_deg.partition_point(|&x| x <= a) - 1;
        let frac = (a - self.twa_deg[i]) / (self.twa_deg[i + 1] - self.twa_deg[i]);
        self.speed[i] + (self.speed[i + 1] - self.speed[i]) * frac
    }

    /// Best through-water speed available on any point of sail (used to
    /// size the reach radius).
    fn max_speed(&self) -> f64 {
        self.speed.iter().cloned().fold(0.0, f64::max)
    }

    /// Scale every speed by `factor`. A value below 1 derates the polar
    /// for planning margin: the measured steady-state polar is optimistic
    /// versus the speed the boat actually holds through tacks, gusts and
    /// accelerations, so an un-derated polar makes the planner's ETA (and any
    /// gate-arrival timing built on it) run early. Derating also raises the
    /// tide-to-boatspeed ratio the planner sees — correctly, since a slower
    /// boat is more set by the current — so it leans less on outrunning a
    /// foul stream. `factor = 1.0` is a no-op.
    pub fn derate(mut self, factor: f64) -> Self {
        for s in &mut self.speed {
            *s *= factor;
        }
        self
    }
}

/// Planner configuration.
pub struct PlanConfig {
    pub start: (f64, f64),
    pub dest: (f64, f64),
    /// Direction the wind blows FROM (math angle, rad).
    pub wind_from: f64,
    /// Wall-clock time (s) the plan starts at. The tide forecast is
    /// queried at `start_time + node_elapsed`, so a mid-mission replan
    /// sees the same tide phase the boat will actually meet. 0 for a
    /// from-scratch plan at the start of the run.
    pub start_time: f64,
    pub dt: f64,
    pub heading_step_deg: f64,
    pub cross_track_bucket_m: f64,
    pub max_steps: usize,
    /// Capture the destination when within this distance (m).
    pub dest_radius: f64,
}

struct Node {
    x: f64,
    y: f64,
    t: f64,
    parent: usize,
}

pub struct PlanResult {
    /// Path waypoints from start to destination (inclusive).
    pub path: Vec<(f64, f64)>,
    /// Planner arrival time (s, relative to `start_time`) at each `path`
    /// point — the continuous-sail schedule, used to decide which legs
    /// would be sailed against a foul tide and so want a gate.
    pub times: Vec<f64>,
    /// Estimated time to sail it (s).
    pub eta_s: f64,
    /// Path length (m).
    pub length_m: f64,
}

/// Run the isochrone planner. Returns `None` if the destination can't be
/// reached within `max_steps`.
pub fn plan(
    cfg: &PlanConfig,
    polar: &Polar,
    forecast: &TideForecast,
    chart: Option<&Chart>,
) -> Option<PlanResult> {
    let axis = unit(sub(cfg.dest, cfg.start));
    let perp = (-axis.1, axis.0);
    let n_head = (360.0 / cfg.heading_step_deg).round() as usize;

    let mut arena: Vec<Node> = vec![Node { x: cfg.start.0, y: cfg.start.1, t: 0.0, parent: usize::MAX }];
    let mut front: Vec<usize> = vec![0];

    for _ in 0..cfg.max_steps {
        // Reached?
        for &fi in &front {
            let n = &arena[fi];
            if dist((n.x, n.y), cfg.dest) <= cfg.dest_radius {
                return Some(backtrack(&arena, fi, cfg.dest));
            }
        }

        // Expand.
        let mut candidates: Vec<usize> = Vec::new();
        for &fi in &front {
            let (px, py, pt) = (arena[fi].x, arena[fi].y, arena[fi].t);
            let (cx, cy) = forecast.at(cfg.start_time + pt).unwrap_or((0.0, 0.0));
            for k in 0..n_head {
                let h = (k as f64) * cfg.heading_step_deg.to_radians();
                let twa = wrap_pi(h - cfg.wind_from).abs().to_degrees();
                let v = polar.speed_at(twa);
                if v <= 1e-6 {
                    continue;
                }
                let nx = px + (v * h.cos() + cx) * cfg.dt;
                let ny = py + (v * h.sin() + cy) * cfg.dt;
                if let Some(ch) = chart {
                    if segment_hits_land(px, py, nx, ny, ch) {
                        continue;
                    }
                }
                arena.push(Node { x: nx, y: ny, t: pt + cfg.dt, parent: fi });
                candidates.push(arena.len() - 1);
            }
        }
        if candidates.is_empty() {
            return None;
        }

        // Prune to the isochrone: per cross-track bucket, keep the point
        // with the greatest along-track progress toward the destination.
        let mut best: std::collections::HashMap<i64, (f64, usize)> = std::collections::HashMap::new();
        for &ci in &candidates {
            let d = sub((arena[ci].x, arena[ci].y), cfg.start);
            let along = dot(d, axis);
            let cross = dot(d, perp);
            let key = (cross / cfg.cross_track_bucket_m).round() as i64;
            let e = best.entry(key).or_insert((f64::NEG_INFINITY, ci));
            if along > e.0 {
                *e = (along, ci);
            }
        }
        front = best.values().map(|&(_, ci)| ci).collect();
    }
    None
}

fn backtrack(arena: &[Node], end: usize, dest: (f64, f64)) -> PlanResult {
    let mut idx = end;
    let eta = arena[end].t;
    let mut pts: Vec<(f64, f64)> = vec![dest];
    let mut times: Vec<f64> = vec![eta]; // dest reached ~at the final node's time
    loop {
        let n = &arena[idx];
        pts.push((n.x, n.y));
        times.push(n.t);
        if n.parent == usize::MAX {
            break;
        }
        idx = n.parent;
    }
    pts.reverse();
    times.reverse();
    let mut length = 0.0;
    for w in pts.windows(2) {
        length += dist(w[0], w[1]);
    }
    PlanResult { path: pts, times, eta_s: eta, length_m: length }
}

// --- geometry helpers ---

fn sub(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 - b.0, a.1 - b.1)
}
fn dot(a: (f64, f64), b: (f64, f64)) -> f64 {
    a.0 * b.0 + a.1 * b.1
}
fn dist(a: (f64, f64), b: (f64, f64)) -> f64 {
    (a.0 - b.0).hypot(a.1 - b.1)
}
fn unit(a: (f64, f64)) -> (f64, f64) {
    let m = a.0.hypot(a.1).max(1e-9);
    (a.0 / m, a.1 / m)
}
fn wrap_pi(x: f64) -> f64 {
    (x + PI).rem_euclid(2.0 * PI) - PI
}

/// True if the segment (x0,y0)-(x1,y1) crosses any chart polygon edge.
/// A bounding-box pre-filter skips polygons the segment can't touch, so
/// open-water segments are rejected cheaply.
fn segment_hits_land(x0: f64, y0: f64, x1: f64, y1: f64, chart: &Chart) -> bool {
    let (sxmin, sxmax) = (x0.min(x1), x0.max(x1));
    let (symin, symax) = (y0.min(y1), y0.max(y1));
    for poly in &chart.polygons {
        let pts = &poly.points;
        if pts.len() < 2 {
            continue;
        }
        if let Some((bxmin, bxmax, bymin, bymax)) = poly_bbox(pts) {
            if sxmax < bxmin || sxmin > bxmax || symax < bymin || symin > bymax {
                continue;
            }
        }
        let edges = if poly.closed { pts.len() } else { pts.len() - 1 };
        for i in 0..edges {
            let a = pts[i];
            let b = pts[(i + 1) % pts.len()];
            if segments_intersect(x0, y0, x1, y1, a[0], a[1], b[0], b[1]) {
                return true;
            }
        }
    }
    false
}

fn poly_bbox(pts: &[[f64; 2]]) -> Option<(f64, f64, f64, f64)> {
    let mut it = pts.iter();
    let first = it.next()?;
    let (mut xmin, mut xmax, mut ymin, mut ymax) = (first[0], first[0], first[1], first[1]);
    for p in it {
        xmin = xmin.min(p[0]);
        xmax = xmax.max(p[0]);
        ymin = ymin.min(p[1]);
        ymax = ymax.max(p[1]);
    }
    Some((xmin, xmax, ymin, ymax))
}

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

/// Cross product of (p2-p1) x (p3-p1).
fn cross3(p1x: f64, p1y: f64, p2x: f64, p2y: f64, p3x: f64, p3y: f64) -> f64 {
    (p2x - p1x) * (p3y - p1y) - (p2y - p1y) * (p3x - p1x)
}

/// Simplify a dense path to its shape-defining points (Ramer–Douglas–
/// Peucker): keep the tack apexes and corners, drop points that lie
/// within `epsilon` of the line between kept points. Uniform sampling
/// would slice across the zigzag tacks and make the follower add its
/// own tacking between waypoints; RDP keeps each emitted leg a clean
/// single tack.
pub fn simplify(path: &[(f64, f64)], epsilon: f64) -> Vec<(f64, f64)> {
    simplify_idx(path, epsilon).into_iter().map(|i| path[i]).collect()
}

/// RDP simplification returning the *indices* of the kept points (in
/// ascending order). Lets a caller carry per-point side data (e.g. the
/// planner's arrival times) through the simplification.
pub fn simplify_idx(path: &[(f64, f64)], epsilon: f64) -> Vec<usize> {
    let n = path.len();
    if n < 3 {
        return (0..n).collect();
    }
    let mut keep = vec![false; n];
    keep[0] = true;
    keep[n - 1] = true;
    rdp(path, 0, n - 1, epsilon, &mut keep);
    (0..n).filter(|&i| keep[i]).collect()
}

fn rdp(path: &[(f64, f64)], lo: usize, hi: usize, epsilon: f64, keep: &mut [bool]) {
    if hi <= lo + 1 {
        return;
    }
    let (s, e) = (path[lo], path[hi]);
    let mut idx = lo;
    let mut dmax = 0.0;
    for i in (lo + 1)..hi {
        let d = point_seg_dist(path[i], s, e);
        if d > dmax {
            dmax = d;
            idx = i;
        }
    }
    if dmax > epsilon {
        keep[idx] = true;
        rdp(path, lo, idx, epsilon, keep);
        rdp(path, idx, hi, epsilon, keep);
    }
}

/// Decide which planned waypoints should be tidal gates. Waypoint `i` is
/// gated when the leg `i -> i+1` would be sailed against a foul (or merely
/// insufficient) stream at its planned arrival time — i.e. the along-leg
/// current is below `threshold` (m/s) — so the follower holds there until
/// the tide turns fair instead of committing and being set off. Legs
/// shorter than `min_leg_m` are never gated (a hold isn't worth it), and
/// the final waypoint (no next leg) is never a gate. Returns one flag per
/// point in `pts`.
pub fn tidal_gate_flags(
    pts: &[(f64, f64)],
    times: &[f64],
    forecast: &TideForecast,
    threshold: f64,
    min_leg_m: f64,
) -> Vec<bool> {
    let mut gates = vec![false; pts.len()];
    for i in 0..pts.len().saturating_sub(1) {
        let (dx, dy) = (pts[i + 1].0 - pts[i].0, pts[i + 1].1 - pts[i].1);
        let len = dx.hypot(dy);
        if len < min_leg_m {
            continue;
        }
        let (cx, cy) = forecast.at(times[i]).unwrap_or((0.0, 0.0));
        let along = (cx * dx + cy * dy) / len;
        if along < threshold {
            gates[i] = true;
        }
    }
    gates
}

/// Perpendicular distance from point `p` to the segment `a`-`b`.
fn point_seg_dist(p: (f64, f64), a: (f64, f64), b: (f64, f64)) -> f64 {
    let ab = sub(b, a);
    let len2 = dot(ab, ab);
    if len2 < 1e-12 {
        return dist(p, a);
    }
    let t = (dot(sub(p, a), ab) / len2).clamp(0.0, 1.0);
    dist(p, (a.0 + ab.0 * t, a.1 + ab.1 * t))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn flat_polar() -> Polar {
        // 1 m/s everywhere from 45°..180°, nothing below 45° (no-go).
        Polar::new(vec![(45.0, 1.0), (90.0, 1.0), (135.0, 1.0), (180.0, 1.0)])
    }

    #[test]
    fn tidal_gate_flags_marks_foul_legs_only() {
        // Three points, both legs run due +x. Reversing stream along +x
        // (period 400 s, phase π/2): fair (+0.6) at t=0, foul (−0.6) at
        // t=200. So leg0 (sailed at t=0) is fine, leg1 (at t=200) is foul.
        let pts = vec![(0.0, 0.0), (100.0, 0.0), (200.0, 0.0)];
        let times = vec![0.0, 200.0, 400.0];
        let fc = TideForecast::Stream {
            peak_speed: 0.6,
            axis_rad: 0.0,
            period_s: 400.0,
            phase_rad: PI / 2.0,
        };
        let gates = tidal_gate_flags(&pts, &times, &fc, 0.05, 10.0);
        assert_eq!(gates, vec![false, true, false]);

        // Same foul leg but below the minimum gated length → not gated.
        let short = vec![(0.0, 0.0), (5.0, 0.0), (10.0, 0.0)];
        let g2 = tidal_gate_flags(&short, &times, &fc, 0.05, 50.0);
        assert_eq!(g2, vec![false, false, false]);
    }

    #[test]
    fn derate_scales_all_speeds_and_preserves_no_go() {
        let p = flat_polar().derate(0.8);
        assert!((p.speed_at(90.0) - 0.8).abs() < 1e-9);
        assert!((p.speed_at(135.0) - 0.8).abs() < 1e-9);
        // Below the no-go angle stays zero (scaling 0 is still 0).
        assert_eq!(p.speed_at(30.0), 0.0);
        // A factor of 1.0 is a no-op.
        let q = flat_polar().derate(1.0);
        assert!((q.speed_at(90.0) - 1.0).abs() < 1e-9);
    }

    #[test]
    fn polar_interpolates_and_no_go() {
        let p = flat_polar();
        assert_eq!(p.speed_at(30.0), 0.0); // below no-go
        assert!((p.speed_at(90.0) - 1.0).abs() < 1e-9);
        assert!((p.speed_at(60.0) - 1.0).abs() < 1e-9);
    }

    #[test]
    fn plans_straight_downwind_no_obstacles() {
        // Wind comes FROM +x (wind_from = 0). Destination due west (-x)
        // is dead downwind → a straight run, no land, no tide.
        let cfg = PlanConfig {
            start: (0.0, 0.0),
            dest: (-1000.0, 0.0),
            wind_from: 0.0,
            start_time: 0.0,
            dt: 100.0,
            heading_step_deg: 10.0,
            cross_track_bucket_m: 100.0,
            max_steps: 50,
            dest_radius: 120.0,
        };
        let r = plan(&cfg, &flat_polar(), &TideForecast::None, None).expect("reachable");
        // At 1 m/s straight downwind, ~1000 m takes ~1000 s.
        assert!(r.eta_s <= 1200.0, "eta {}", r.eta_s);
        assert!(r.length_m < 1300.0, "length {}", r.length_m);
        assert!(r.path.len() >= 2);
    }

    #[test]
    fn plans_upwind_by_tacking() {
        // Destination dead upwind (wind from +x, dest at +x means dest is
        // upwind). The boat can't sail < 45° off, so it must tack; the
        // planner should still reach via angled headings.
        let cfg = PlanConfig {
            start: (0.0, 0.0),
            dest: (2000.0, 0.0),
            wind_from: 0.0, // wind from +x; dest is at +x = straight upwind
            start_time: 0.0,
            dt: 100.0,
            heading_step_deg: 5.0,
            cross_track_bucket_m: 200.0,
            max_steps: 200,
            dest_radius: 150.0,
        };
        // Wind from +x means heading toward +x is TWA 0 (no-go). To go
        // upwind toward +x the boat tacks at ±45°.
        let r = plan(&cfg, &flat_polar(), &TideForecast::None, None);
        assert!(r.is_some(), "should reach an upwind destination by tacking");
        let r = r.unwrap();
        // Tacking is longer than the straight-line 2000 m.
        assert!(r.length_m > 2000.0, "tacked length {}", r.length_m);
    }
}
