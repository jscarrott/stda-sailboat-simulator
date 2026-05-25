# stda-sailboat-simulator

A 6-degree-of-freedom (6-DOF) sailboat physics simulator. Originally authored by
Simon Kohaut (MIT License, 2018) in Python; being ported to Rust under `rust_src/`.

## Model source

The physics is the model from:

> M. C. Buehler, C. Heinz, S. Kohaut, *"Dynamic Simulation Model for an Autonomous
> Sailboat"*, Sailing Team Darmstadt e.V., Proc. International Robotic Sailing
> Conference 2018, Southampton.

**Important:** the paper's stated scope is dynamic *behaviour*, not speed accuracy
("the precise prediction of the actual reachable velocities are of minor interest"),
and it has no empirical validation. Absolute boat speeds are uncalibrated; the
wave-resistance weight and damping coefficients need fitting against real data. See
`CLAUDE.md` → *Source Paper* for the full force model, coefficients, and the places
where the code diverges from the paper.

## Quick start (Rust)

```bash
cd rust_src
cargo build --release
cargo run --release -- --scenario route \
    --route routes/ilfracombe_to_lundy.yaml \
    --chart charts/north_devon.json --max-run-time-s 200000
# Measure the speed polar (calibration instrument):
cargo run --release -- --scenario polar --config sim_params_iom.yaml --wind-speed 4
```

- `CLAUDE.md` — full technical reference (physics, conventions, port status, paper notes).
- `docs/project_plan.html` — hardware-retrofit project plan and calibration roadmap.
