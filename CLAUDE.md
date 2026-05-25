# CLAUDE.md — stda-sailboat-simulator

A 6-degree-of-freedom (6-DOF) sailboat physics simulator. Originally authored by Simon
Kohaut (MIT License, 2018) in Python; currently being ported to Rust.

---

## Branch Overview

| Branch | Language | Source root | Status |
|---|---|---|---|
| `master` | Python 2 | `src/` | Original implementation; not Python 3 compatible |
| `migrate-to-python3` | Python 3.11 | `sailboat_sim/` | Python 3 fixes applied; Poetry-managed |
| `rust_implementation` | Rust + Python 3.11 | `rust_src/` + `sailboat_sim/` | Active development branch; Rust skeleton + Python reference |

**Primary development branch: `rust_implementation`.**
The `sailboat_sim/` package on that branch is the authoritative Python 3 reference
implementation to port from. The `master` branch is Python 2 and should not be the
basis for new work.

---

## Source Paper

The model is taken from:

> M. C. Buehler, C. Heinz, S. Kohaut, *"Dynamic Simulation Model for an Autonomous
> Sailboat"*, Sailing Team Darmstadt e.V., Proc. International Robotic Sailing
> Conference 2018, Southampton. Original code: `github.com/simko96/stda-sailboat-simulator`.

`docs/` is the place for a copy of the PDF. The equation numbers below refer to that paper.

### Scope and validation status (read this first)

The model deliberately targets **dynamic behaviour, not speed accuracy**. Direct quotes:

- *"In contrast to standard velocity prediction programs (VPP) … the precise prediction
  of the actual reachable velocities are of minor interest."* (Abstract)
- *"so far we do not have empirical data to validate its performance. Instead we
  qualitatively examine for plausibility."* (§3)
- *"we still need to record real data, to identify parameters like damping."* (Conclusion)

**Implications.** Absolute boat speeds are uncalibrated. The published model produces
~0.8–1.0 m/s for the 4 m hull at 5 m/s wind, and the Rust port reproduces this
(≈0.72 m/s at 4 m/s). The free parameters that set absolute speed — the **damping
coefficients `d`** and the **wave-resistance weight `c_wr`** — are explicitly rough
estimates the authors flag for real-data identification. Treat any speed match (e.g. the
IOM `wave_resistance_weight = 0.06` curve-fit) as provisional until fitted against a
measured polar. Use `--scenario polar` to measure; see `docs/project_plan.html` phase 2b/6.

### Reference frames (§2.1)

1. **Global / navigational** — position `(xg, yg, zg)`, heading `ψ`; `z = 0` at the
   undisturbed surface. Position kinematics (eq. 1).
2. **Heading frame** — at the CoG, parallel to the water surface, x-axis along heading.
   Buoyancy `Fhs`, translation dynamics `(vx, vy, vz)` and wave resistance live here.
3. **Body frame** — fixed to the hull, reached from the heading frame by roll `φ`
   (pitch `θ` assumed small). Rotational dynamics and the sail/rudder/keel forces live here.

Relative flow on a foil: `v_rel = v_flow − v − Ω × r_foil` (the `Ω × r_foil` rotational
term is **not** in the current code — forces use `v_flow − v` only).

### Foil lift & drag (§2.2, eqs. 5–7)

Both sails and lateral foils (keel, rudder) are modelled as thin foils in a stream:

- Lift coefficient `cL = cL,0 + cL,α·α`, with `cL,0 = 0` (symmetric) and slope
  `cL,α = 2π` (thin-airfoil theory).
- Drag `cD = cf + cD,i`. Induced drag `cD,i = cL² / (π·Λ)` where **`Λ` is the geometric
  stretching = aspect ratio** (the YAML `stretching` fields). Friction
  `cf = 2.66 / √Re`, laminar flat plate, `Re = v·l / ν` (`l` = chord, `ν` = kinematic
  viscosity). → higher `Λ` = more lift per AoA + less induced drag = points/drives better.
- Force (eq. 7): `F = q·A·[(sinβ·cL − cosβ·cD)·e1 − (sinβ·cD + cosβ·cL)·e2]`,
  dynamic pressure `q = ½·ρ·v_rel²`, `β` the relative-flow angle.
- Point of application: ¼-chord (symmetric) shifting toward ⅓-chord (asymmetric,
  AoA-dependent).

### Flow separation (§2.2.2)

At large AoA the flow separates: lift drops, pressure drag grows. Separated drag
`cD = sin²(α)`; at 90° there's no lift, `cD ≈ 1`, and the load acts at the foil centre.
Attached↔separated is blended by `s = 1 − exp(−(α / α_sep)²)` with `α_sep = 25°`, giving
a ~15° stall angle. (See "Separation Factor" below — matches the code exactly.)

### Wave resistance (§2.3, eq. 8)

Displacement-hull wave-making drag, the dominant speed limiter. Hull speed
`v_hull = 0.4·√(g·l_wl)` → ~2.5 m/s for a 4 m waterline, ~1.25 m/s for a 1 m IOM.
Paper form: `Fwr = −sign(vx)·c_wr·q·A_LK·(v/v_hull)⁴` (≈ a 6th-order speed polynomial),
with tunable weight `c_wr` and lateral area `A_LK`.

**Code differs:** `calculate_wave_impedance` uses `−sign(vx)·v²·(v/v_hull)²·(c_wr·ρ/2·A_LK)`
— i.e. `(v/v_hull)²`, a 4th-order total, not the paper's 6th-order. `c_wr` is
`boat.wave_resistance_weight` (default 1.0).

### Hydrostatic buoyancy & waves (§2.4, eqs. 9–10)

`Fhs ≈ m·g + ρ·g·A_w·(η − zg)` (`A_w` = waterplane area, `η` = wave elevation). The load
point depends on buoyancy height above CoG and the waterplane second moments `I_L`
(roll) and `I_T` (pitch); `I_T ≫ I_L` keeps pitch small. Effective roll/pitch are taken
relative to the wave surface (`φ_eff`, `θ_eff`). Assumes wavelength ≫ hull length.

### Damping (§2.5)

Linear, decoupled: `[F_D; T_D] = −d·[v; Ω]`. The paper calls these **"rough estimates"**
and notes a quadratic term would be better. These are the `*_damping` / `yaw_timeconstant`
YAML fields and the prime candidates (with `c_wr`) for real-data tuning.

### Actuators & inputs (§2.6)

First-order lag with time constant `τ`; the sail's *sign* follows the relative wind while
its *magnitude* is set by the controller (rope length). Paper values: rudder `τ ≈ 0.1 s`,
sail `τ ≈ 3 s`; rudder limit `±35°`, sail `±90°`; control sample time `100 ms`.

**Code differs:** the runner steps actuators analytically (`RUDDER_RATE = 2.0` → τ = 0.5 s,
`SAIL_RATE = 0.1` → τ = 10 s), uses a `±15°` rudder limit (`controller.rs`) and a `0.3 s`
control period (`SAMPLE_TIME`). Revisit these against the paper if matching its traces.

### Not modelled

- **Added mass** — the paper flags it as planned-but-absent and important for roll
  (Korotkin 2009). Still not implemented; expect roll dynamics to be too lively.
- The `Ω × r_foil` rotational inflow term (see frames, above).

### Integration

Dormand–Prince adaptive Runge–Kutta (the `Dopri5` path; the port adds a fixed-step
`Rk4` option for the stiffer IOM dynamics).

---

## Running the Code

### Rust (primary target)

```bash
cd rust_src
cargo build
cargo run
```

**Dependencies** (declared in `rust_src/Cargo.toml`):
- `polars = { version = "0.35.4", features = ["ndarray"] }` — DataFrame library for result storage
- Rust edition 2021

No ODE solver crate is currently declared; one will need to be added (see
[Rust Implementation Status](#rust-implementation-status)).

### Python reference

The Python code lives in `sailboat_sim/` and is managed with Poetry from the repo root.

```bash
# Install dependencies (once)
poetry install

# Run from sailboat_sim/ — YAML loading requires this CWD
cd sailboat_sim
poetry run python run.py
```

`simulation.py` and `heading_controller.py` both open `sim_params_config.yaml` with a
bare filename at module import time. The working directory **must** be `sailboat_sim/`
or the import will raise `FileNotFoundError`.

**Python dependencies** (from `pyproject.toml`):
- `python = "^3.11"`
- `numpy ^1.26.2`, `scipy ^1.11.4`, `matplotlib ^3.8.2`, `pyyaml ^6.0.1`

---

## File Structure

```
stda-sailboat-simulator/
├── pyproject.toml           Poetry config (Python + polars deps)
├── poetry.lock
├── .python-version          pyenv target: 3.11
├── LICENSE                  MIT, 2018 Simon Kohaut
├── rust_src/                Rust port (active development)
│   ├── Cargo.toml
│   ├── Cargo.lock
│   └── src/
│       └── main.rs          Only Rust file; skeleton implementation
└── sailboat_sim/            Python 3 reference implementation
    ├── __init__.py
    ├── sim_params_config.yaml   All boat/environment/simulator parameters
    ├── simulation.py            Core 6-DOF physics engine + ODE definition
    ├── heading_controller.py    PID+LQR rudder controller
    ├── sail_angle.py            Optimal sail trim angle calculator
    └── run.py                   Scenarios, simulation loop, matplotlib plots
```

### Role of each Python file

**`simulation.py`** — the physics core. Do not run directly. Imported with
`from simulation import *` in `run.py`. At module import it:
1. Parses `sim_params_config.yaml` and declares ~50 module-level constants
2. Pre-computes derived invariants (e.g. `WAVE_IMPEDANCE_INVARIANT`) to avoid
   repeated computation inside the ODE hot path
3. Defines all `namedtuple` structured types
4. Declares state-index integer constants (`POS_X`, `ROLL`, `YAW_RATE`, etc.)
5. Instantiates two global mutable objects: `state` (numpy array) and `environment` (list)
6. Defines `solve(time, boat)` — the ODE right-hand-side consumed by scipy

**`run.py`** — the simulation driver. Contains:
- `scenario_0()`: 3-second leeway/separation test
- `scenario_1()`: 150-second maneuver with heading changes and sail trimming
- `simulate()`: main time-stepping loop (updates environment, calls controller and sail
  optimizer, advances the scipy integrator, records history)
- `simple_integrator`: Euler-step fallback integrator for comparison
- All plotting helpers (saves EPS to a `figs/` subdirectory)

**`heading_controller.py`** — `heading_controller` class. Also opens the YAML
independently at import time. Key methods:
- `calculate_controller_params()`: solves the continuous Riccati equation (CARE) for LQR gains
- `controll()` (one `l`): computes rudder command with anti-windup and saturation

**`sail_angle.py`** — single function `sail_angle()`. Computes optimal sail trim from
apparent wind using a sine/cosine lift formula, clamped to 14° stall angle, with
high-wind derating.

**`sim_params_config.yaml`** — authoritative source for all numerical parameters. Units
are annotated inline. Initial angles are in radians (= 0); wind direction is in degrees.

---

## Physics Architecture

This section is the specification for the Rust port.

### State Vector

12 elements by default; 14 when `actor_dynamics = True` (adds actuator states).

| Index | Python constant | Description | Unit |
|---|---|---|---|
| 0 | `POS_X` | Global x position | m |
| 1 | `POS_Y` | Global y position | m |
| 2 | `POS_Z` | Vertical position (heave) | m |
| 3 | `ROLL` | Roll angle (heel) | rad |
| 4 | `PITCH` | Pitch angle (trim) | rad |
| 5 | `YAW` | Heading | rad |
| 6 | `VEL_X` | Surge velocity (body-frame forward) | m/s |
| 7 | `VEL_Y` | Sway/leeway velocity (body-frame lateral) | m/s |
| 8 | `VEL_Z` | Heave velocity | m/s |
| 9 | `ROLL_RATE` | Roll angular rate | rad/s |
| 10 | `PITCH_RATE` | Pitch angular rate | rad/s |
| 11 | `YAW_RATE` | Yaw angular rate | rad/s |
| 12 | `RUDDER_STATE` | Actual rudder angle (actuator model) | rad |
| 13 | `SAIL_STATE` | Actual sail angle (actuator model) | rad |

When `actor_dynamics` is enabled, `environment[SAIL_ANGLE]` and
`environment[RUDDER_ANGLE]` become reference commands rather than direct values, and
the actuator states evolve as first-order dynamics in the ODE.

### Global Environment (Python) → `Environment` Struct (Rust)

| Index | Python constant | Rust field | Type |
|---|---|---|---|
| 0 | `SAIL_ANGLE` | `sail_angle` | f32 (rad) |
| 1 | `RUDDER_ANGLE` | `rudder_angle` | f32 (rad) |
| 2 | `TRUE_WIND` | `true_wind` | `Truewind` struct |
| 3 | `WAVE` | `wave` | `Option<Wave>` struct |

### Namedtuple → Struct Mapping

| Python namedtuple | Fields | Notes |
|---|---|---|
| `TrueWind` | x, y, strength, direction | Already `Truewind` struct in Rust |
| `ApparentWind` | x, y, angle, speed | Already `ApparentWind` struct in Rust |
| `Wave` | length, direction, amplitude | Currently `Option<f32>` in Rust — needs a proper struct |
| `SailForce` | x, y | Needs a struct |
| `LateralForce` | x, y | `calculate_lateral_force` returns `(LateralForce, separation_factor)` |
| `RudderForce` | x, y | Needs a struct |
| `HydrostaticForce` | x, y, z | Needs a struct |
| `Damping` | x, y, z, yaw, pitch, roll | Needs a struct |

### Force Model Call Order in `solve()`

The `solve(time, boat)` function assembles all forces and returns the state derivative:

1. Unpack state vector by index; unpack environment (sail/rudder angles, wind, wave)
2. `calculate_wave_influence()` — wave height and surface gradients at boat position
3. `calculate_apparent_wind()` — true wind rotated to body frame minus boat velocity
4. `calculate_damping()` — linear/angular rate damping
5. `calculate_hydrostatic_force()` — buoyancy + wave-elevation; returns force x/y/z
   **plus** two moment-arm scalars (`x_hs`, `y_hs`) used for pitch/roll moments
6. `calculate_wave_impedance()` — hull wave-making drag (cubic speed scaling)
7. `calculate_rudder_force()` — lift/drag on rudder blade
8. `calculate_lateral_force()` — keel hydrodynamic lift/drag; returns
   `(LateralForce, separation)` — the separation factor is reused in yaw moment assembly
9. `calculate_sail_force()` — aerodynamic lift/drag with flow-separation blending
10. Assemble linear and angular accelerations from Newton/Euler equations
11. (If `actor_dynamics`) append first-order actuator derivatives for rudder and sail

The ODE is integrated with `scipy.integrate.ode` using **dopri5** (Dormand-Prince RK45).
For the Rust port, use a crate such as `ode_solvers` or `diffeq` — add the dependency to
`rust_src/Cargo.toml`.

### Separation Factor

Both sail and keel force functions use a Gaussian-based flow separation model:

```python
separation = 1 - exp(-((abs(eff_aoa)) / (pi/180*25))**2)
```

`separation = 0` → attached flow (linear lift); `separation = 1` → separated (drag-dominated).
`calculate_lateral_force` returns `(LateralForce, separation)` as a tuple — both values
are used at the call site. Do not discard the separation return value.

### True Sail Angle Sign Convention

`environment[SAIL_ANGLE]` (and `sail_angle()`) always returns a non-negative magnitude.
Inside `solve()`, the sign is applied dynamically:

```python
true_sail_angle = np.sign(apparent_wind.angle) * abs(sail_angle)
```

This ensures the sail is always on the correct tack regardless of which side the wind comes from.

### Heading Controller Design

`heading_controller.controll()` computes:

```
rudder = (1 / factor / speed² / cos(roll)) × (KP × error + KI × integral − KD × yaw_rate)
```

where `factor = DISTANCE_COG_RUDDER × RUDDER_BLADE_AREA × π × WATER_DENSITY / MOI_Z`.

`calculate_controller_params()` uses `scipy.linalg.solve_continuous_are` to solve the
CARE on a 3-state linearization of yaw dynamics (heading error, yaw rate, integrated
error). The LQR gains map directly to PID gains KP, KD, KI. Anti-windup is applied by
back-calculating the integrator when the rudder saturates.

---

## Rust Implementation Status

### Done

- `Truewind` struct (x, y, angle, speed — all f32)
- `ApparentWind` struct
- `Scenario` struct (n_states, actor_dynamics, wind)
- `Environment` struct (sail_angle, rudder_angle, true_wind, wave as `Option<f32>`)
- `run_scenario()` skeleton: timing parameters, state vector initialization via `Series::new`
- `init_dataframe()`: builds a DataFrame of zero-filled `Series` columns

### Not Yet Implemented

- All 8 force-calculation functions (sail, lateral, rudder, hydrostatic, wave
  impedance, wave influence, apparent wind, damping)
- ODE integration loop (the simulation time-stepping)
- YAML config loading (currently all parameters are hardcoded or zero)
- `Wave` struct (currently `Option<f32>` placeholder)
- Remaining force-output structs (`SailForce`, `LateralForce`, etc.)
- Heading controller (equivalent of `heading_controller.py`)
- Sail angle optimizer (equivalent of `sail_angle.py`)
- Result output / plotting

### Known Issues in Current Rust Code

`run_scenario()` contains `x.select_physical() = x0.clone()` which is not valid Rust
(assignment to a function-call expression). This must be fixed before the crate compiles.
The intent appears to be initializing the first column/row of the DataFrame with the
initial state — the correct Polars API call would be different (e.g. using `lazy()` or
rebuilding the DataFrame with the initial state as the first entry).

Consider whether Polars DataFrames are the right data structure for the state history.
A simpler approach for ODE integration is a `Vec<[f32; 14]>` (one array per timestep),
converting to a DataFrame only for output. Polars is optimized for columnar analytics,
not row-by-row mutation during integration.

---

## Key Conventions

### Naming

- **Physical constants**: `UPPERCASE_WITH_UNDERSCORES` in Python; use Rust constants
  (`const BOAT_LENGTH: f32 = 4.0;`) or a config struct loaded from YAML
- **Functions**: `snake_case` in both Python and Rust
- **Structs/Classes**: `PascalCase` in Rust; the Python `heading_controller` class
  violates this (lowercase) — use PascalCase for any new Rust types
- **State index constants**: short uppercase, e.g. `POS_X`, `VEL_Y`, `YAW_RATE`

### Units and Angles

- **All angles are radians** in code. Convert only at boundaries (YAML input, print output).
- **Velocities**: m/s in body frame (surge = forward, sway = lateral/leeway)
- **Positions**: meters in a global Cartesian frame (NED-like)
- **Forces**: Newtons; **Moments**: N·m (divided by MOI to get angular acceleration)
- **Dynamic pressure**: `(density/2) × speed²` — standard aero/hydro formula
- **Aspect ratio ("stretching")**: dimensionless; appears in drag-due-to-lift denominator
  `4π × eff_aoa² / stretching`

### All Angles Are Radians

The YAML file mixes units: initial wind direction is in **degrees**; initial sail and
rudder angles are in **radians** (= 1 rad ≈ 57°). Check the YAML comments carefully
before changing initial conditions.

---

## Domain Knowledge

### 6-DOF Rigid Body Dynamics

6 DOF = surge (x), sway (y), heave (z), roll (about x), pitch (about y), yaw (about z).
Velocities are in body frame; positions are in global frame, connected via
`cos(yaw)/sin(yaw)` rotation terms in the position derivatives.

### Sailboat Physics Glossary

- **Leeway**: sway velocity `VEL_Y`; the keel's lateral force opposes it
- **Heel**: roll angle; reduces effective sail area via `cos(roll)` factors
- **Hull speed**: theoretical maximum displacement-hull speed (~2.5 m/s for this boat);
  wave-making drag grows as the cube of speed beyond hull speed
- **Apparent wind**: vector sum of true wind and negative boat velocity, transformed to
  body frame; upwind sailing produces faster, more forward apparent wind than true wind
- **Angle of attack (AoA)**: for sails, the angle between apparent wind and sail chord;
  for keel, the leeway angle between boat velocity and longitudinal axis
- **Effective AoA**: folded back when `|aoa| > π/2` to avoid discontinuity when running
  downwind (`eff_aoa = π + aoa` or `-π + aoa`)

### Wave Model

Sinusoidal waves with configurable wavelength, amplitude, and direction. In all current
scenarios `amplitude = 0`, so waves are present in the model but inactive. Wave
influence provides a surface height and two gradient components that tilt the buoyancy
vector.

---

## Guidance for AI Assistants

### Patterns to Follow

1. **Use index constants, never magic numbers** for state/environment access: `state[YAW]`
   not `state[5]`.
2. **All angles in radians** in code. Add explicit conversion at any new boundary.
3. **Add new Python forces as named functions** following the `calculate_*` pattern,
   returning a namedtuple. In Rust, return a named struct.
4. **Pre-compute invariants** at module level (Python) or as constants/lazy statics (Rust)
   for any expression inside `solve()` that depends only on configuration parameters.
5. **Do not modify `environment` inside `solve()`**. It is a pure ODE RHS; side effects
   will be called multiple times per timestep by the adaptive integrator.
6. **Preserve the 2-tuple return from `calculate_lateral_force()`**. The separation
   factor feeds back into the yaw moment. Refactoring this signature requires updating
   both the call site and moment assembly.

### Common Pitfalls

1. **CWD dependency**: `open('sim_params_config.yaml')` resolves relative to CWD, not
   the script's location. Any file importing `simulation` or `heading_controller` must
   be run from `sailboat_sim/`, or the YAML load must use `pathlib.Path(__file__).parent`.
2. **`from simulation import *` side effects**: importing triggers YAML parsing,
   constant initialization, and global object creation immediately. Tests sharing a
   process share global `state` and `environment` — reset them explicitly between cases.
3. **`actor_dynamics` flag**: hardcoded boolean in `simulation.py` (not in YAML).
   Changing it changes the state vector length from 12 to 14. All array allocations
   using `n_states` must be consistent.
4. **`calculate_lateral_force` returns a 2-tuple**: `(LateralForce, separation)`.
   Never unpack as a single value.
5. **Sail angle sign**: `environment[SAIL_ANGLE]` is always non-negative. The sign is
   applied inside `solve()` based on apparent wind direction. Do not pre-apply it.
6. **Heading angle wrapping**: `state[YAW]` is not automatically wrapped to `[-π, π]`
   in the integration loop. The controller wraps heading errors via while-loops. Add
   explicit wrapping for any new heading comparisons.
7. **`SAIL_AREA` in `calculate_lateral_force`**: line ~248 of `simulation.py` uses the
   sail area constant (6.4 m²) inside the keel force separated-flow term — likely a
   copy-paste bug from `calculate_sail_force`. Be aware when modifying the keel model.
8. **Commented-out code**: many alternative parameter sets and development notes are
   commented out throughout. Treat as engineering notes; do not remove without understanding.
9. **Polars DataFrame mutation**: in the current Rust skeleton, `x.select_physical() =
   x0.clone()` is invalid Rust. Fix before attempting to compile. Consider using
   `Vec<Vec<f32>>` or `Vec<[f32; N]>` for the integration history and converting to
   DataFrame only for output.
10. **No ODE solver in Rust yet**: `Cargo.toml` has no ODE crate. Add one (e.g.
    `ode_solvers = "0.3"`) before implementing `solve()`.

### Extending the Rust Port

- **Implementing a force function**: write a pure `fn calculate_*(...) -> ForceStruct`
  with no side effects. Precompute any parameter-only invariants as `const` or in a
  `Config` struct loaded once from YAML.
- **Adding YAML config loading**: add `serde = { version = "1", features = ["derive"] }`
  and `serde_yaml = "0.9"` to `Cargo.toml`; define a `Config` struct mirroring the
  YAML layout; deserialize in `main()` and pass into `run_scenario()`.
- **ODE integration**: the Python `simulate()` loop pattern is:
  1. Update `environment` (controller output, sail trim)
  2. Call `integrator.integrate(t + dt)` — this calls `solve()` internally
  3. Record state history
  Replicate this pattern with the chosen Rust ODE crate.

---

## Quick Reference

```
rust_src/src/main.rs         Rust entry point + all structs + skeleton functions
sailboat_sim/simulation.py   Python physics constants, force functions, solve(), global state
sailboat_sim/run.py          Python scenarios, simulate() loop, plots
sailboat_sim/heading_controller.py   PID+LQR rudder controller
sailboat_sim/sail_angle.py   Optimal sail trim angle
sailboat_sim/sim_params_config.yaml  All tunable parameters (run from sailboat_sim/)
```

State indices (Python constants, Rust to implement equivalently):
```
POS_X=0  POS_Y=1  POS_Z=2  ROLL=3  PITCH=4  YAW=5
VEL_X=6  VEL_Y=7  VEL_Z=8  ROLL_RATE=9  PITCH_RATE=10  YAW_RATE=11
[RUDDER_STATE=12  SAIL_STATE=13]  ← only when actor_dynamics=true
```

Environment indices:
```
SAIL_ANGLE=0  RUDDER_ANGLE=1  TRUE_WIND=2  WAVE=3
```
