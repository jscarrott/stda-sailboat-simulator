#!/usr/bin/env python3
"""Print the LQR-derived heading controller gains so they can be
hardcoded in src/controller.rs. Re-run this whenever Q, r, or
YAW_TIMECONSTANT changes.

Usage (from rust_src/):
    python scripts/compute_lqr_gains.py
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SAILBOAT_DIR = os.path.join(os.path.dirname(HERE), "..", "sailboat_sim")
os.chdir(SAILBOAT_DIR)
sys.path.insert(0, SAILBOAT_DIR)

from heading_controller import heading_controller  # noqa: E402

c = heading_controller(sample_time=0.3)
k = c.calculate_controller_params(store=False)
kp, kd, ki = float(k[0]), float(k[1]), -float(k[2])
print(f"LQR_KP = {kp!r};")
print(f"LQR_KD = {kd!r};")
print(f"LQR_KI = {ki!r};")
