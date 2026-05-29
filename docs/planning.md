# Route planning

This doc explains how `--scenario plan` produces a sailable route, what
decisions the planner makes vs. what the route follower makes at runtime,
and the principles behind the choices that matter most.

It covers the code in `rust_src/src/planner.rs` (search + simplify + gate
decisions) and `rust_src/src/scenario/mod.rs` (orchestration + post-
processing + YAML output), plus the parts of `rust_src/src/route/mod.rs`
that the planner output relies on (gate holds, per-leg XTE).

## What the planner produces

A YAML route file containing:

- An ordered list of waypoints from start to destination, each marked as
  `hard` (default, must capture inside `acceptance_radius`) or
  `soft` (fly-by, captured on along-track pass or fly-by radius).
- Following params: `xte_lookahead`, `min_tack_duration_s`,
  `close_hauled_angle_deg`, etc.
- Optional tidal-gate annotations on waypoints (`gate: true`) plus the
  route-level gate-open rules (`gate_open_along_current`,
  `gate_min_fair_window_s`) that govern when each held gate releases.
- The wind override the plan was computed for, so a re-sail uses the
  same conditions.

The boat is sailed by the route follower (`RouteFollower::update` in
`route/mod.rs`); the planner does not run the simulation. The plan is
a recommendation the follower tracks at runtime, adapting locally for
tides and gusts the planner only knew about as a forecast.

## Step 1 — Isochrone search

`planner::plan` (`planner.rs:123`) explores reachable space outward in
time from the start.

**State:** each `Node` holds a position `(x, y)` and a planner time `t`
(seconds since `start_time`). The arena grows; a parent index threads
back to the start so any frontier node can be back-tracked to a path.

**Expansion:** at each step, every frontier node spawns one candidate
per heading in `0..360` stepped by `heading_step_deg` (planner.rs:150).
For each heading, the boat's through-water speed comes from the polar
(`Polar::speed_at`, derated by `--polar-derate`); the tidal current at
the *node's* time is added to give a ground velocity; one `dt` of motion
gives the candidate position. Headings where the polar returns ~0 speed
(inside the no-go cone) are dropped.

**Tide-aware timing.** The forecast is queried at `start_time + node.t`
— each node sees the tide phase the boat will *actually* meet at that
point in the plan, not the tide at planner-launch time. This is what
lets the gate logic later compare tide phase across legs that the boat
hits hours apart.

**Land avoidance.** If a chart is provided, each candidate segment is
tested against chart polygons (`segment_hits_land`, `planner.rs:235`,
with a bounding-box pre-filter). Candidates that cross land are dropped
during expansion, so the search never plans through a coastline.

**Isochrone pruning.** Per expansion, candidates are bucketed by their
cross-track offset from the start→dest axis (bucket size
`cross_track_bucket_m`); within each bucket only the candidate with the
greatest along-track progress is kept (`planner.rs:172-188`). This is
the classic isochrone trick: at any given moment in time, only the most
forward point in each lateral lane can lead to the optimal path. It
keeps the frontier bounded as time advances.

The search terminates when any frontier node lands inside `dest_radius`
of the destination, then back-tracks via `parent` to build the path
(`backtrack`, `planner.rs:190`).

## Step 2 — Simplify (RDP)

The raw path has one point per `dt` step — far too dense to feed the
follower as waypoints. `simplify_idx` (`planner.rs:303`) runs Ramer-
Douglas-Peucker with `epsilon = 250 m` (`scenario/mod.rs:345`) and
returns indices into the dense path. Per-point side data (here: the
planner's arrival times) ride through the simplification by indexing.

RDP keeps **shape-defining points** — tack apexes and corners — and
drops collinear interior points. Uniform sub-sampling would slice
across the zigzag tacks and force the follower to add its own tacking
between waypoints; RDP keeps each emitted leg a clean single tack the
follower steers directly.

## Step 3 — Densify long legs (only when gating)

When `--plan-tidal-gates` is on, `densify_legs` (`scenario/mod.rs:468`)
splits any leg longer than `GATE_MAX_LEG_M = 5000 m` into equal
sub-legs, linearly interpolating the planner's arrival times across the
inserted points.

A single long gated leg is a trap: the boat commits to it during a fair
phase, the tide turns foul mid-way, and the boat gets set back before
it clears. Short sub-legs each fit comfortably inside one fair-tide
window, so the boat **rides through several sub-gates during a fair
phase and only holds when the tide turns foul.** The sub-points become
soft fly-by waypoints; the boat doesn't pause at them when the tide
stays fair through the next sub-leg.

This step is skipped without gating, so a plain `--scenario plan` keeps
its RDP-simplified legs unchanged.

## Step 4 — Decide tidal gates

`tidal_gate_flags` (`planner.rs:346`) flags waypoint `i` as a gate if
the leg `i → i+1` is **tide-exposed**: its along-leg current goes foul
(`< -foul_margin`) at some point over a 12.42 h (one M2 semidiurnal
cycle) sampled from the planned arrival time at `i`.

Two principles drive this:

1. **Worst-case-over-cycle, not planned-time.** After waiting at earlier
   gates the boat reaches this leg at an unpredictable phase, so
   checking only the planner's arrival instant under-gates. Sampling
   the cycle correctly gates any leg that *can* turn foul, leaving the
   actual timing decision to the follower's gate-open check.

2. **Relative threshold (`foul_margin`).** A leg is only worth gating
   when sailing through the foul would actually stop or reverse the
   boat. As long as the foul is weaker than the boat's own speed, the
   boat still makes net headway; holding (which parks it for the whole
   foul phase and exposes it to cross-track set) loses time. The
   margin is scaled off the planner's mean speed:

       gate_foul_margin = clamp(0.9 · length/ETA, 0.25, 1.2)

   so it tracks the boat and conditions (a 0.66 m/s boat gates at
   ~0.6 m/s foul; a 1.0 m/s boat at ~0.9 m/s) rather than firing on
   any current.

Cross-tide legs (along-current ~0 at every phase) and legs shorter than
`GATE_MIN_LEG_M = 500 m` are never gated. The final waypoint is never
a gate (no next leg to time).

A/B-validated regime boundaries:

- Weak foul (foul < boat speed) → 0 gates emitted, sail straight
  through, faster (printed: *"no gates: along-leg foul never exceeds
  X m/s..."*).
- Strong foul (foul > boat speed) → gates fire, holding through the
  foul beats getting pushed backward; the gated route reaches further
  than the same route ungated.

## Step 5 — Scale `min_tack_duration_s`

The Lundy crossing sits ~44.5° off the wind — just inside the close-
hauled no-go cone — so on the long open-water leg the follower flip-
flops tacks at the boundary every time the tide nudges the bearing
across 45°. A 30 s minimum tack lets it chatter; the boat never gets
across.

`scenario/mod.rs` raises the minimum tack with leg length:

    min_tack_s = clamp(0.05 · longest_leg / nominal_speed, base, 1800)

— roughly 5% of the time it would take to sail the longest leg —
floored at the input route's value and capped at 30 minutes. Short
routes keep their base value (no scaling kicks in); long passages get
a long-enough minimum tack to commit to one or two long tacks.

The `xte_lookahead` is **not** scaled at the route level — see step 7.

## Step 6 — Emit the YAML

`write_planned_route` (`scenario/mod.rs:420`) writes the file. First
and last points are hard waypoints (start anchor and destination);
interior points are soft (fly-by). Gated waypoints carry `gate: true`.
A header comment notes when params were scaled.

## Step 7 — Per-leg XTE in the follower

The XTE lookahead controls how aggressively the follower corrects when
pushed off the rhumb line:

    chi_los = chi_path + atan2(-xte, eff_lookahead)

A tight lookahead (100 m) on a long open leg turns any tidal offset
into a near-90° "claw back to the line" command. On a close-hauled
course that means *pinching* — the boat points above close-hauled,
loses drive, and stalls. A loose lookahead lets it foot for speed.

But a route-level loose lookahead is wrong: applied to the *short*
final-approach legs it makes the boat ignore the line and cut the
corner straight through whatever's between waypoints (in the Lundy
case, the island itself — 13,972 land-incursion points).

The follower scales the lookahead per-leg (`route/mod.rs:380` ish):

    eff_lookahead = clamp(0.10 · leg_len, route.xte_lookahead, 3000)

`route.xte_lookahead` is the floor (so a route is never made tighter
than configured), the cap prevents arbitrarily large lookaheads. Long
open legs foot; short approach legs track; the planner only has to
emit the floor.

## Step 8 — Holds station-keep, not orbit

When a gate is held, the follower commands a heading that **stems the
tide** (with a gentle pull back toward the gate point) rather than
steering *at* the gate (`station_keep_heading` in `route/mod.rs`).

Steering at a fixed point with way-on in a current produces orbits —
the boat overshoots, turns around, overshoots again. Stemming the
current is true station-keeping: ground velocity is cancelled to zero
(plus a small recovery). If the stem direction falls inside the no-go
cone it snaps to the nearest close-hauled limit so the boat keeps
drive and loses the least ground. The autopilot skips its crab
compensation while a hold is active because the stem heading is
already a water-frame command, not a course over ground.

## What the planner does *not* do

- It doesn't run the physics simulation (the follower does, with full
  6-DOF dynamics and stochastic wind/tide).
- It uses a *polar* — derated for slow steady wind, no gust or stall
  modelling. The follower's actual speed differs.
- It picks gates from a single tide cycle's worst case; it doesn't
  model multi-cycle tide curves or springs/neaps.
- It plans a path; it doesn't pick a *departure time*. To depart at
  the start of a fair tide, gate the first waypoint (the follower's
  departure-gate logic holds at the start until the first leg's
  fair-tide window opens, ferry-gliding meanwhile).
- It doesn't replan in flight on its own. The autopilot can be told
  (opt-in) to replan from the current position when a held gate has
  blown past its window (`RouteAutopilot::with_held_gate_replan`,
  using `RouteFollower::replace_remaining`).

## Tuning summary

| Lever | Where | Default | When to change |
|---|---|---|---|
| `heading_step_deg` | `PlanConfig` | (CLI) | Finer = better path, slower search. |
| `cross_track_bucket_m` | `PlanConfig` | (CLI) | Smaller = wider isochrone, slower. |
| RDP epsilon | `scenario/mod.rs:345` | 250 m | Bigger = fewer waypoints, more deviation. |
| `GATE_MAX_LEG_M` | `scenario/mod.rs` | 5000 m | Tied to fair-tide window length. |
| `gate_foul_margin` factor | `scenario/mod.rs` | 0.9 × nominal | Lower → more gates fire. |
| `min_tack_s` factor | `scenario/mod.rs` | 0.05 × leg sail time | Higher → fewer, longer tacks. |
| `XTE_LEG_FRACTION` | `route/mod.rs` | 0.10 | Higher → looser line, more footing. |
| `XTE_LOOKAHEAD_CAP` | `route/mod.rs` | 3000 m | Cap on adaptive loosening. |
