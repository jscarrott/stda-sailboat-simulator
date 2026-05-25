#!/usr/bin/env python3
"""Fetch a tidal-current time series from the Copernicus Marine Service
(CMEMS) and cache it for the simulator as a small JSON the Rust
`TabulatedCurrent` model can load.

CMEMS gives gridded eastward/northward sea-water velocity (uo, vo) that
includes the tidal signal in the hourly North-West-Shelf physics
products. We subset a small box around a representative point on the
Ilfracombe -> Lundy corridor, take the surface layer, area-average to a
single (u, v) time series, and write it out. The simulator's current
model is spatially uniform, so a point/area time series is the right
shape; graduate to a full grid later if you want the races resolved.

REQUIREMENTS (run on your own machine with your CMEMS account):
    pip install copernicusmarine xarray netCDF4
    copernicusmarine login        # once, stores credentials

VERIFY before trusting the output:
  - DATASET_ID below. Product/dataset ids change between CMEMS releases;
    browse https://data.marine.copernicus.eu and pick the North-West
    Shelf *hourly* physics analysis/forecast that carries uo/vo. The
    default here is the current NWS analysis-forecast hourly dataset at
    time of writing and MUST be re-checked.
  - That uo/vo are tidal-inclusive (the hourly products are; daily-mean
    products are not).

Usage (from rust_src/):
    python scripts/fetch_tides.py --start 2026-05-25 --days 3
"""
import argparse
import json
import os
import sys
from datetime import datetime, timezone

# Representative sampling box on the channel corridor (around mid-way
# Ilfracombe<->Lundy). Small box; we area-average inside it.
LAT_MIN, LAT_MAX = 51.15, 51.25
LON_MIN, LON_MAX = -4.55, -4.25

# Tangent-plane origin shared with the coastline cache (centre of Lundy).
ORIGIN = {"lat": 51.1735, "lon": -4.6680}

# CMEMS NW-Shelf hourly physics (uo, vo). VERIFY — see module docstring.
DATASET_ID = "cmems_mod_nws_phy_anfc_0.027deg-2D_PT1H-i"
VAR_U, VAR_V = "uo", "vo"

HERE = os.path.dirname(os.path.abspath(__file__))
RUST_SRC = os.path.dirname(HERE)
OUT_PATH = os.path.join(RUST_SRC, "charts", "lundy_tides.json")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--start", required=True, help="UTC start date YYYY-MM-DD")
    ap.add_argument("--days", type=int, default=3)
    ap.add_argument("--dataset", default=DATASET_ID)
    ap.add_argument("--out", default=OUT_PATH)
    args = ap.parse_args()

    try:
        import copernicusmarine
        import numpy as np
        import xarray as xr
    except ImportError as e:
        sys.exit(
            f"missing dependency ({e}). Install with:\n"
            "  pip install copernicusmarine xarray netCDF4\n"
            "  copernicusmarine login"
        )

    start = datetime.strptime(args.start, "%Y-%m-%d").replace(tzinfo=timezone.utc)
    end = start.replace(hour=23, minute=59)
    end = end.fromordinal(start.toordinal() + args.days - 1).replace(
        hour=23, minute=59, tzinfo=timezone.utc
    )

    tmp_nc = os.path.join(HERE, "_cmems_tmp.nc")
    print(f"subsetting {args.dataset} {args.start} +{args.days}d ...")
    copernicusmarine.subset(
        dataset_id=args.dataset,
        variables=[VAR_U, VAR_V],
        minimum_longitude=LON_MIN,
        maximum_longitude=LON_MAX,
        minimum_latitude=LAT_MIN,
        maximum_latitude=LAT_MAX,
        start_datetime=start.isoformat(),
        end_datetime=end.isoformat(),
        minimum_depth=0.0,
        maximum_depth=1.0,
        output_filename=tmp_nc,
        force_download=True,
    )

    ds = xr.open_dataset(tmp_nc)
    # Area-average over the box (and squeeze any depth dim) -> (time,) series.
    u = ds[VAR_U].mean(dim=[d for d in ds[VAR_U].dims if d != "time"]).values
    v = ds[VAR_V].mean(dim=[d for d in ds[VAR_V].dims if d != "time"]).values
    times = ds["time"].values
    dt_s = float((times[1] - times[0]) / np.timedelta64(1, "s"))

    # Replace NaNs (land/masked cells) with 0 so the series is clean.
    u = np.nan_to_num(np.asarray(u, dtype=float))
    v = np.nan_to_num(np.asarray(v, dtype=float))

    out = {
        "source": f"CMEMS {args.dataset}",
        "fetched_at": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "origin": ORIGIN,
        "point": {"lat": (LAT_MIN + LAT_MAX) / 2, "lon": (LON_MIN + LON_MAX) / 2},
        "t0_iso": str(times[0]),
        "dt_s": dt_s,
        "u_east": [round(x, 4) for x in u.tolist()],
        "v_north": [round(x, 4) for x in v.tolist()],
    }
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    with open(args.out, "w") as f:
        json.dump(out, f, separators=(",", ":"))
    os.remove(tmp_nc)
    speeds = (u**2 + v**2) ** 0.5
    print(
        f"wrote {args.out}: {len(u)} samples @ {dt_s:.0f}s, "
        f"|current| min/mean/max = {speeds.min():.2f}/{speeds.mean():.2f}/{speeds.max():.2f} m/s"
    )


if __name__ == "__main__":
    main()
