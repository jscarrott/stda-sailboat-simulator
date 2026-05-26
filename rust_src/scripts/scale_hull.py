#!/usr/bin/env python3
"""Generate a geometrically-scaled sim config from the calibrated 4 m
reference hull (sim_params_config.yaml), at an arbitrary waterline length.

Dimensions scale geometrically (lengths ×λ, areas ×λ², mass ×λ³, waterplane
2nd-moments ×λ⁴, mass-moments-of-inertia ×λ⁵, damping/yaw times ×√λ, hull_speed
×√λ, c_wr constant; λ = L/4). Foil aspect ratios do NOT — see foil_ar(): the
4 m reference's very low-AR appendages make a shrunk hull directionally
unstable, so ARs blend toward the (stable) 1 m IOM's deep high-AR fins as the
boat gets smaller. Areas still scale geometrically, so higher AR = deeper,
narrower foil — which is what real small craft actually use.

Usage (from rust_src/):
    uv run python scripts/scale_hull.py --length 2.5 --out sim_params_2_5m.yaml
    # then: ./target/release/sailboat_sim --config sim_params_2_5m.yaml ...
"""
import argparse
from math import sqrt

# Calibrated 4 m reference (sim_params_config.yaml). c_wr = 0.06.
BASE_L = 4.0
BASE = {
    "sail": {"pressure_point_height": 2.48, "height": 6.2, "area": 6.4,
             "length": 1.0, "stretching": 0.961},
    "rudder": {"stretching": 2.326923076923077, "area": 0.13},
    "keel": {"height": 0.55, "length": 2.0, "stretching": 0.605},
    "length": 4.0, "mass": 350.0, "height_bouyancy": 0.15,
    "lateral_area": 2.5, "waterline_area": 1.6, "wave_resistance_weight": 0.06,
    "distance_cog_sail_pressure_point": 0.43,
    "distance_cog_keel_pressure_point": 0.24,
    "distance_cog_rudder": 1.24,
    "distance_mast_sail_pressure_point": 0.68,
    "geometrical_moi_x": 0.256, "geometrical_moi_y": 25.6,
    "moi_x": 25.6, "moi_y": 1600.0, "moi_z": 1066.0,
    "roll_damping": 0.25, "pitch_damping": 0.25, "damping_z": 0.2,
    "yaw_timeconstant": 5.0, "along_damping": 15.0, "transverse_damping": 5.0,
    "hull_speed": 2.5,
}


# Foil aspect ratios ("stretching") are NOT geometrically scaled. The 4 m
# reference carries freakishly low-AR appendages (keel 0.6, rudder 2.3, rig
# 0.96); shrinking those onto a small hull gives a directionally-unstable boat
# that spins on every heading, and even at 4 m the AR-0.6 keel makes so much
# leeway the boat can't hold off a lee shore (it gets set onto Lundy). Real
# craft of every size use higher-AR fins/rigs — model yachts very high, full-
# size boats moderate. So blend the aspect ratios linearly by length between
# the (stable) 1 m IOM and realistic full-size values; areas/spans still scale
# geometrically, so a higher AR just means a deeper, narrower foil — which is
# what real fin keels, spade rudders and Bermudan rigs actually are.
IOM_AR = {"sail": 5.0, "rudder": 5.0, "keel": 6.0}      # ~1 m, from sim_params_iom.yaml
BIG_AR = {"sail": 3.0, "rudder": 3.0, "keel": 2.0}      # ~4 m fin-keel sloop (realistic)


def foil_ar(name: str, length: float) -> float:
    frac = min(1.0, max(0.0, (length - 1.0) / (BASE_L - 1.0)))  # 0 at 1 m, 1 at 4 m
    return IOM_AR[name] + frac * (BIG_AR[name] - IOM_AR[name])


def scale(lam: float) -> dict:
    L, A, M = lam, lam**2, lam**3          # length / area / mass ratios
    length = lam * BASE_L
    b = BASE
    return {
        "sail": {
            "pressure_point_height": b["sail"]["pressure_point_height"] * L,
            "height": b["sail"]["height"] * L,
            "area": b["sail"]["area"] * A,
            "length": b["sail"]["length"] * L,
            "stretching": foil_ar("sail", length),
        },
        "rudder": {"stretching": foil_ar("rudder", length),
                   "area": b["rudder"]["area"] * A},
        "keel": {"height": b["keel"]["height"] * L,
                 "length": b["keel"]["length"] * L,
                 "stretching": foil_ar("keel", length)},
        "length": b["length"] * L,
        "mass": b["mass"] * M,
        "height_bouyancy": b["height_bouyancy"] * L,
        "lateral_area": b["lateral_area"] * A,
        "waterline_area": b["waterline_area"] * A,
        "wave_resistance_weight": b["wave_resistance_weight"],
        "distance_cog_sail_pressure_point": b["distance_cog_sail_pressure_point"] * L,
        "distance_cog_keel_pressure_point": b["distance_cog_keel_pressure_point"] * L,
        "distance_cog_rudder": b["distance_cog_rudder"] * L,
        "distance_mast_sail_pressure_point": b["distance_mast_sail_pressure_point"] * L,
        "geometrical_moi_x": b["geometrical_moi_x"] * lam**4,
        "geometrical_moi_y": b["geometrical_moi_y"] * lam**4,
        "moi_x": b["moi_x"] * lam**5,
        "moi_y": b["moi_y"] * lam**5,
        "moi_z": b["moi_z"] * lam**5,
        "roll_damping": b["roll_damping"],
        "pitch_damping": b["pitch_damping"],
        "damping_z": b["damping_z"],
        "yaw_timeconstant": b["yaw_timeconstant"] * sqrt(lam),
        "along_damping": b["along_damping"] * sqrt(lam),
        "transverse_damping": b["transverse_damping"] * sqrt(lam),
        "hull_speed": b["hull_speed"] * sqrt(lam),
    }


def emit(boat: dict, length: float) -> str:
    def g(x):
        return f"{x:.6g}"
    s, r, k = boat["sail"], boat["rudder"], boat["keel"]
    return f"""## Geometrically-scaled hull, LWL {length:.2f} m (λ = {length / BASE_L:.4f}).
## Generated by scripts/scale_hull.py from the calibrated 4 m reference.
boat:
  sail:
    pressure_point_height: {g(s['pressure_point_height'])}
    height: {g(s['height'])}
    area: {g(s['area'])}
    length: {g(s['length'])}
    stretching: {g(s['stretching'])}
  rudder:
    stretching: {g(r['stretching'])}
    area: {g(r['area'])}
  keel:
    height: {g(k['height'])}
    length: {g(k['length'])}
    stretching: {g(k['stretching'])}
  length: {g(boat['length'])}
  mass: {g(boat['mass'])}
  height_bouyancy: {g(boat['height_bouyancy'])}
  lateral_area: {g(boat['lateral_area'])}
  waterline_area: {g(boat['waterline_area'])}
  wave_resistance_weight: {g(boat['wave_resistance_weight'])}
  distance_cog_sail_pressure_point: {g(boat['distance_cog_sail_pressure_point'])}
  distance_cog_keel_pressure_point: {g(boat['distance_cog_keel_pressure_point'])}
  distance_cog_rudder: {g(boat['distance_cog_rudder'])}
  distance_mast_sail_pressure_point: {g(boat['distance_mast_sail_pressure_point'])}
  geometrical_moi_x: {g(boat['geometrical_moi_x'])}
  geometrical_moi_y: {g(boat['geometrical_moi_y'])}
  moi_x: {g(boat['moi_x'])}
  moi_y: {g(boat['moi_y'])}
  moi_z: {g(boat['moi_z'])}
  roll_damping: {g(boat['roll_damping'])}
  pitch_damping: {g(boat['pitch_damping'])}
  damping_z: {g(boat['damping_z'])}
  yaw_timeconstant: {g(boat['yaw_timeconstant'])}
  along_damping: {g(boat['along_damping'])}
  transverse_damping: {g(boat['transverse_damping'])}
  hull_speed: {g(boat['hull_speed'])}

environment:
  water_viscosity: 0.0000001
  air_viscosity: 0.0000171
  water_density: 1000
  air_density: 1.3
  gravity: 9.81

simulator:
  stepper:
    stepsize: 0.1
    clockrate: 10
  initial:
    vel_x: 0
    vel_y: 0
    vel_z: 0
    yaw: 0
    pitch: 0
    roll: 0
    roll_rate: 0
    pitch_rate: 0
    yaw_rate: 0
    latitude: 51.207
    longitude: -4.110
    wind_strength: 5.0
    wind_direction: 45.0
    wave_direction: 220.0
    wave_length: 2.0
    wave_amplitude: 0.0
    sail_angle: 1
    rudder_angle: 1
"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--length", type=float, required=True, help="target LWL in metres")
    ap.add_argument("--out", required=True, help="output config path")
    ap.add_argument("--mass", type=float, default=None,
                    help="override displacement (kg); MOI rescales with it. Real "
                         "racing models are much lighter than the geometric ∝L³ mass.")
    ap.add_argument("--roll-damping", type=float, default=None,
                    help="override roll damping ratio ζ (default 0.25 from the 4 m "
                         "reference). A deep bulb keel realistically pushes ζ up; "
                         "the light A-Class needs ≥0.7 to stay numerically stable.")
    ap.add_argument("--pitch-damping", type=float, default=None,
                    help="override pitch damping ratio ζ (default 0.25).")
    args = ap.parse_args()
    boat = scale(args.length / BASE_L)
    if args.mass is not None:
        factor = args.mass / boat["mass"]      # rescale rotational inertia with mass
        boat["mass"] = args.mass
        for k in ("moi_x", "moi_y", "moi_z"):
            boat[k] *= factor
    if args.roll_damping is not None:
        boat["roll_damping"] = args.roll_damping
    if args.pitch_damping is not None:
        boat["pitch_damping"] = args.pitch_damping
    with open(args.out, "w") as f:
        f.write(emit(boat, args.length))
    print(f"wrote {args.out}: LWL {args.length} m, mass {boat['mass']:.1f} kg, "
          f"hull_speed {boat['hull_speed']:.2f} m/s")


if __name__ == "__main__":
    main()
