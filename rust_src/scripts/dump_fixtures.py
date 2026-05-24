#!/usr/bin/env python3
"""Dump (state, env, derivative) fixtures from the Python reference
implementation so the Rust port can be diffed against it.

Usage (from rust_src/):
    python scripts/dump_fixtures.py

Writes tests/fixtures/derivatives.json.
"""
import json
import os
import sys
from math import cos, radians, sin

HERE = os.path.dirname(os.path.abspath(__file__))
RUST_SRC = os.path.dirname(HERE)
REPO = os.path.dirname(RUST_SRC)
SAILBOAT_DIR = os.path.join(REPO, "sailboat_sim")

# simulation.py opens 'sim_params_config.yaml' relative to CWD on import,
# so we have to chdir before importing it.
os.chdir(SAILBOAT_DIR)
sys.path.insert(0, SAILBOAT_DIR)

import simulation as sim  # noqa: E402

assert sim.actor_dynamics is True, "Rust port assumes actor_dynamics=True"


def make_env(sail_angle=1.0, rudder_angle=1.0, wind_strength=5.0, wind_dir_deg=45.0,
             wave_length=2.0, wave_direction=220.0, wave_amplitude=0.0):
    return [
        sail_angle,
        rudder_angle,
        sim.TrueWind(
            wind_strength * cos(radians(wind_dir_deg)),
            wind_strength * sin(radians(wind_dir_deg)),
            wind_strength,
            wind_dir_deg,
        ),
        sim.Wave(length=wave_length, direction=wave_direction, amplitude=wave_amplitude),
    ]


def init_state(**overrides):
    """Return a 14-element initial state with optional per-index overrides."""
    s = list(sim.initial_state()) + [1.0, 1.0]  # rudder, sail (matches YAML)
    for k, v in overrides.items():
        s[getattr(sim, k.upper())] = v
    return s


FIXTURES = [
    {
        "name": "initial_state",
        "time": 0.0,
        "state": init_state(),
        "env": {"sail_angle": 1.0, "rudder_angle": 1.0,
                "wind_strength": 5.0, "wind_dir_deg": 45.0,
                "wave_length": 2.0, "wave_direction": 220.0, "wave_amplitude": 0.0},
    },
    {
        "name": "moving_with_leeway",
        "time": 3.5,
        "state": init_state(vel_x=1.5, vel_y=0.1, yaw=0.7),
        "env": {"sail_angle": 0.6, "rudder_angle": 0.05,
                "wind_strength": 5.0, "wind_dir_deg": 45.0,
                "wave_length": 2.0, "wave_direction": 220.0, "wave_amplitude": 0.0},
    },
    {
        "name": "heeled_turning",
        "time": 12.0,
        "state": init_state(vel_x=2.0, vel_y=-0.2, roll=0.18, yaw=1.4, yaw_rate=0.08),
        "env": {"sail_angle": 0.4, "rudder_angle": -0.1,
                "wind_strength": 6.0, "wind_dir_deg": 90.0,
                "wave_length": 2.0, "wave_direction": 220.0, "wave_amplitude": 0.0},
    },
    {
        "name": "downwind_high_speed",
        "time": 30.0,
        "state": init_state(vel_x=3.2, vel_y=0.0, yaw=3.14159, pitch=0.02, pos_x=12.0, pos_y=5.0),
        "env": {"sail_angle": 1.4, "rudder_angle": 0.0,
                "wind_strength": 7.0, "wind_dir_deg": 180.0,
                "wave_length": 2.0, "wave_direction": 220.0, "wave_amplitude": 0.0},
    },
]


def main():
    rows = []
    for fx in FIXTURES:
        env = make_env(**fx["env"])
        sim.environment[:] = env
        deriv = sim.solve(fx["time"], list(fx["state"])).tolist()
        rows.append({
            "name": fx["name"],
            "time": float(fx["time"]),
            "state": [float(v) for v in fx["state"]],
            "env": fx["env"],
            "derivative": [float(v) for v in deriv],
        })
    out_path = os.path.join(RUST_SRC, "tests", "fixtures", "derivatives.json")
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, "w") as f:
        json.dump({"fixtures": rows}, f, indent=2)
    print(f"wrote {len(rows)} fixtures to {out_path}")


if __name__ == "__main__":
    main()
