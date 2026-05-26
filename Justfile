# Justfile — common sailboat-sim runs.
#
# Recipes execute from rust_src/ (where the config, routes/, charts/ and
# scripts/ paths are rooted). Run `just` with no args to list everything.
#
#   just lundy                 # headline Ilfracombe -> Lundy round trip
#   just pond 270 3.5          # IOM pond triangle, wind from the N at 3.5 m/s
#   just fetch-tides 2026-05-25 3   # refresh the CMEMS tide series
#
# Wind direction is the math convention used by the CLI: degrees the wind
# blows TOWARD (45° = blowing NE = a SW wind). Route YAMLs that carry their
# own `wind:` block don't need --wind-* here.

set working-directory := 'rust_src'

config     := 'sim_params_config.yaml'   # 4 m hull (default scenarios)
iom_config := 'sim_params_iom.yaml'      # 1 m IOM pond boat
chart      := 'charts/north_devon.json'  # Lundy + N. Devon coastline overlay
tide_data  := 'charts/lundy_tides.json'  # produced by `just fetch-tides`
run        := 'cargo run --release -q --'

# List available recipes.
default:
    @just --list

# Build the release binary (used by the script-based recipes).
build:
    cargo build --release

# --- Common routes -------------------------------------------------------
# Each writes a PNG under figs/ (route_<name>.png unless --out is set) and
# prints a waypoint-by-waypoint report. `plan` and `polar` write PNGs too.

# Ilfracombe -> Lundy round trip + circumnavigation, with coastline overlay.
# ~72 km round trip; bump the time budget if it runs out, e.g. `just lundy 60000`.
[doc('Ilfracombe -> Lundy round trip + circumnavigation, with chart overlay')]
lundy time='40000':
    {{run}} --scenario route --route routes/ilfracombe_to_lundy.yaml \
        --chart {{chart}} --max-run-time-s {{time}}

# Same passage with fly-by (soft) waypoints — smoother turns at the marks.
lundy-flyby time='40000':
    {{run}} --scenario route --route routes/ilfracombe_to_lundy_flyby.yaml \
        --chart {{chart}} --max-run-time-s {{time}}

# Counter-clockwise circumnavigation of Lundy (~16 km) with overlay.
circumnavigate time='8000':
    {{run}} --scenario route --route routes/lundy_circumnavigate.yaml \
        --chart {{chart}} --max-run-time-s {{time}}

# IOM pond triangle. Wind from the CLI (compass deg the wind blows TOWARD):
# `just pond 270 3.5` = wind out of the north at 3.5 m/s. The PNG name encodes
# the wind so repeated runs don't overwrite each other.
[doc('IOM pond triangle; wind from the CLI, e.g. `just pond 270 3.5`')]
pond wind_deg='270' wind_speed='3.5' time='1200':
    {{run}} --config {{iom_config}} --scenario route --route routes/pond_triangle.yaml \
        --wind-deg {{wind_deg}} --wind-speed {{wind_speed}} --max-run-time-s {{time}} \
        --out figs/pond_{{wind_deg}}deg_{{wind_speed}}ms.png

# Sweep the pond triangle over N/E/S/W winds -> figs/iom_pond_*.png.
pond-sweep: build
    ./scripts/wind_sweep.sh

# Tidal-gate demo: the boat loiters at the gate until the stream sets fair.
gated:
    {{run}} --scenario route --route routes/gated_demo.yaml \
        --tide-peak 0.6 --tide-flood-deg 0 --tide-period-h 2 --tide-phase-deg 200

# Cross-current / crab-compensation demo (steady northerly set).
crosscurrent:
    {{run}} --scenario route --route routes/crosscurrent.yaml \
        --wind-deg 90 --wind-speed 6 \
        --tide-peak 0.4 --tide-flood-deg 0 --tide-period-h 500 --tide-phase-deg 90 \
        --crab

# Run any route by file stem in routes/, passing extra flags through, e.g.
# `just route triangle --wind-deg 0 --wind-speed 4` or `just route crossing_strong_gated`.
[doc('Run any route by name in routes/, passing extra flags through')]
route name *args:
    {{run}} --scenario route --route routes/{{name}}.yaml {{args}}

# --- Planning & analysis -------------------------------------------------

# Offline isochrone planner on the Ilfracombe->Lundy crossing input. Writes
# routes/crossing_plan_input_planned.yaml, then renders the planned route
# (sailed, with chart overlay) to figs/plan_crossing.png. Uses rk4 — dopri5
# stalls on the planner's internal polar measurement. Pass planner flags
# through, e.g. `just plan --plan-tidal-gates --tide-peak 1.5` (then render a
# gated plan yourself with the tide via `just route crossing_plan_input_planned ...`).
[doc('Plan the crossing -> routes/..._planned.yaml + figs/plan_crossing.png')]
plan *args:
    {{run}} --scenario plan --route routes/crossing_plan_input.yaml --solver rk4 {{args}}
    {{run}} --scenario route --route routes/crossing_plan_input_planned.yaml \
        --chart {{chart}} --solver rk4 --max-run-time-s 60000 --out figs/plan_crossing.png

# Speed polar at a given true-wind speed (default 4 m/s, 4 m hull) -> a radial
# polar diagram at figs/polar_<config>_<speed>ms.png. Uses the fixed-step rk4
# solver — dopri5 goes unstable sweeping low-speed upwind angles.
# For the IOM: `just polar 3.5 sim_params_iom.yaml`.
[doc('Speed polar (radial PNG) at a true-wind speed, e.g. `just polar 4`')]
polar wind_speed='4.0' cfg=config:
    {{run}} --config {{cfg}} --scenario polar --wind-speed {{wind_speed}} --solver rk4

# --- Tide / chart data ---------------------------------------------------

# Fetch a CMEMS tidal-current series -> charts/lundy_tides.json. Deps live in
# the uv `tides` group (auto-installed by --group). One-time CMEMS auth:
# `uv run --group tides copernicusmarine login`. Usage: `just fetch-tides 2026-05-25 3`.
[doc('Fetch a CMEMS tide series -> charts/lundy_tides.json (see comment for CMEMS auth)')]
fetch-tides start days='3':
    uv run --group tides python scripts/fetch_tides.py --start {{start}} --days {{days}}

# Re-fetch the North Devon + Lundy coastline overlay -> charts/north_devon.json.
fetch-coastline:
    uv run python scripts/fetch_coastline.py

# Ilfracombe -> Lundy under the real CMEMS tide, as a FLEET: the committed
# A-Class candidate (sim_params_aclass.yaml) plus each size in `sizes` (LWL m,
# generated with scale_hull.py), each sailed as its own track overlaid on
# figs/fleet_lundy.png; finish time per boat printed in days. Default sizes all
# round Lundy cleanly; <2 m gets pinned on the lee shore — add it to see the
# limit: `just lundy-tides "1.5 2.0"`. (run `just fetch-tides ...` first.)
[doc('Lundy fleet (A-Class + sizes) under real tide -> figs/fleet_lundy.png (finish in days)')]
lundy-tides sizes='2.5 3.0 4.0' time='400000':
    #!/usr/bin/env bash
    set -euo pipefail
    cfgs="sim_params_aclass.yaml"
    for L in {{sizes}}; do
        uv run python scripts/scale_hull.py --length "$L" --out "/tmp/${L}m.yaml" >/dev/null
        cfgs="$cfgs,/tmp/${L}m.yaml"
    done
    {{run}} --scenario fleet --fleet-configs "$cfgs" \
        --route routes/ilfracombe_to_lundy.yaml --chart {{chart}} \
        --tide-data {{tide_data}} --solver rk4 --max-run-time-s {{time}} \
        --out figs/fleet_lundy.png
