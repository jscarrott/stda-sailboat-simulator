#!/usr/bin/env bash
# Sweep the pond triangle under several wind conditions and produce
# one PNG per condition. Wind is specified in compass terms ("FROM N")
# but converted to the sim's math-direction-deg (angle of the wind
# *velocity vector* — i.e. where it's blowing TOWARD).
#
# Run from rust_src/ after `cargo build --release`.
set -euo pipefail

CONFIG=${CONFIG:-sim_params_iom.yaml}
ROUTE=${ROUTE:-routes/pond_triangle.yaml}
MAX_RUN_TIME=${MAX_RUN_TIME:-1200}
SPEEDS=${SPEEDS:-"2.0 3.5"}

# label: compass "from" → math direction the wind blows TOWARD
declare -A DIRS=( [N]=270 [E]=180 [S]=90 [W]=0 )

mkdir -p figs
for label in "${!DIRS[@]}"; do
    deg=${DIRS[$label]}
    for spd in $SPEEDS; do
        out="figs/iom_pond_from_${label}_${spd}ms.png"
        echo "--- wind FROM $label @ ${spd} m/s (math dir ${deg}°) -> $out"
        ./target/release/sailboat_sim \
            --config "$CONFIG" \
            --scenario route \
            --route "$ROUTE" \
            --wind-deg "$deg" \
            --wind-speed "$spd" \
            --max-run-time-s "$MAX_RUN_TIME" \
            --out "$out" 2>&1 | tail -2 || echo "  (integrator failed — see above)"
    done
done
