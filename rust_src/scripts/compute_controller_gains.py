#!/usr/bin/env python3
"""Solve the continuous-time algebraic Riccati equation for the
3-state yaw linearisation used by heading_controller.py:69 and emit
the resulting LQR gains as a YAML block ready to paste into a sim
config.

State:  x = [heading_error, yaw_rate, integrated_error]
Input:  rudder
Plant:  A = [[0,        1,        0],
             [0, -1/yaw_tc,        0],
             [-1,       0,        0]]
        B = [0, 1, 1]ᵀ
Design: Q = diag(0.1, 1, 0.3),  r = 30   (Python defaults)

The optimal feedback K = R⁻¹ Bᵀ P (where A'P + PA - PBR⁻¹B'P + Q = 0)
maps to PID gains via KP = K[0], KD = K[1], KI = -K[2] — same
mapping as heading_controller.py:84-89.

Usage (from rust_src/):
    python scripts/compute_controller_gains.py sim_params_iom.yaml
"""
import argparse
import sys

import numpy as np
import yaml
from scipy.linalg import solve_continuous_are


def compute(yaw_tc: float, Q=None, r=None):
    if Q is None:
        Q = np.diag([1e-1, 1.0, 0.3])
    if r is None:
        r = np.array([[30.0]])
    A = np.array(
        [
            [0.0, 1.0, 0.0],
            [0.0, -1.0 / yaw_tc, 0.0],
            [-1.0, 0.0, 0.0],
        ]
    )
    B = np.array([[0.0], [1.0], [1.0]])
    P = solve_continuous_are(A, B, Q, r)
    K = (B.T @ P).ravel() / r[0, 0]
    return {"kp": float(K[0]), "kd": float(K[1]), "ki": float(-K[2])}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("config", help="path to sim_params_*.yaml")
    ap.add_argument("--q-heading", type=float, default=0.1)
    ap.add_argument("--q-yawrate", type=float, default=1.0)
    ap.add_argument("--q-integral", type=float, default=0.3)
    ap.add_argument("--r", type=float, default=30.0)
    args = ap.parse_args()

    with open(args.config) as f:
        cfg = yaml.safe_load(f)
    yaw_tc = cfg["boat"]["yaw_timeconstant"]
    Q = np.diag([args.q_heading, args.q_yawrate, args.q_integral])
    r = np.array([[args.r]])
    g = compute(yaw_tc, Q, r)

    print(
        f"# LQR gains derived from yaw_timeconstant={yaw_tc} with",
        file=sys.stderr,
    )
    print(
        f"# Q=diag({args.q_heading}, {args.q_yawrate}, {args.q_integral}), r={args.r}",
        file=sys.stderr,
    )
    print(
        f"# Source: scripts/compute_controller_gains.py {args.config}",
        file=sys.stderr,
    )
    print("controller_gains:", file=sys.stderr)
    print(f"  kp: {g['kp']:.6f}", file=sys.stderr)
    print(f"  ki: {g['ki']:.6f}", file=sys.stderr)
    print(f"  kd: {g['kd']:.6f}", file=sys.stderr)
    # Also print to stdout so caller can capture.
    print(yaml.safe_dump({"controller_gains": g}, default_flow_style=False))


if __name__ == "__main__":
    main()
