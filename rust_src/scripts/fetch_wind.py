#!/usr/bin/env python3
"""Fetch a wind forecast from Open-Meteo and cache it as a small JSON the
Rust `TabulatedWind` model can load. Mirrors fetch_tides.py.

Open-Meteo (https://open-meteo.com) is free and requires no account or API key
— a single HTTPS GET returns hourly ECMWF-IFS forecast wind at 10 m for any
lat/lon (up to ~16 days ahead). It re-distributes ECMWF data; for commercial
use check their licence.

Usage (from rust_src/):
    uv run python scripts/fetch_wind.py --start 2026-05-27 --days 7
    # or simply: just fetch-wind 2026-05-27 7
"""
import argparse
import json
import os
import sys
import urllib.parse
import urllib.request
from datetime import datetime, timezone
from math import cos, radians, sin

# Sampling point on the Ilfracombe<->Lundy corridor (centre of Lundy).
LAT = 51.1735
LON = -4.6680

# Tangent-plane origin shared with the coastline cache and the tide series.
ORIGIN = {"lat": 51.1735, "lon": -4.6680}

HERE = os.path.dirname(os.path.abspath(__file__))
RUST_SRC = os.path.dirname(HERE)
OUT_PATH = os.path.join(RUST_SRC, "charts", "lundy_wind.json")

API = "https://api.open-meteo.com/v1/forecast"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--start", required=True, help="UTC start date YYYY-MM-DD")
    ap.add_argument("--days", type=int, default=7,
                    help="forecast horizon in days (Open-Meteo allows up to ~16)")
    ap.add_argument("--lat", type=float, default=LAT)
    ap.add_argument("--lon", type=float, default=LON)
    ap.add_argument("--out", default=OUT_PATH)
    args = ap.parse_args()

    start = datetime.strptime(args.start, "%Y-%m-%d").replace(tzinfo=timezone.utc)
    end = start.fromordinal(start.toordinal() + args.days - 1)

    qs = urllib.parse.urlencode({
        "latitude": args.lat,
        "longitude": args.lon,
        "start_date": start.strftime("%Y-%m-%d"),
        "end_date": end.strftime("%Y-%m-%d"),
        "hourly": "wind_speed_10m,wind_direction_10m",
        "wind_speed_unit": "ms",
        "timezone": "UTC",
    })
    url = f"{API}?{qs}"
    print(f"GET {url}")
    try:
        with urllib.request.urlopen(url, timeout=30) as resp:
            data = json.load(resp)
    except urllib.error.HTTPError as e:
        sys.exit(f"Open-Meteo returned {e.code}: {e.reason}\n  {e.read().decode(errors='replace')[:400]}")

    times = data["hourly"]["time"]
    speeds = data["hourly"]["wind_speed_10m"]
    dirs = data["hourly"]["wind_direction_10m"]

    # Open-Meteo wind_direction_10m is meteorological: degrees the wind comes
    # FROM (0=N, 90=E). The simulator wants the velocity vector (where the
    # wind blows TO) so it matches TrueWind.{x,y} and the tide JSON convention.
    #   u_east  = −speed · sin(dir_from)
    #   v_north = −speed · cos(dir_from)
    u_east, v_north = [], []
    for s, d in zip(speeds, dirs):
        if s is None or d is None:
            u_east.append(0.0); v_north.append(0.0); continue
        r = radians(d)
        u_east.append(-s * sin(r))
        v_north.append(-s * cos(r))

    out = {
        "source": "Open-Meteo / ECMWF IFS 10 m winds",
        "fetched_at": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "origin": ORIGIN,
        "point": {"lat": args.lat, "lon": args.lon},
        "t0_iso": times[0],   # "YYYY-MM-DDTHH:MM" (UTC, hourly)
        "dt_s": 3600.0,
        "u_east": [round(x, 3) for x in u_east],
        "v_north": [round(x, 3) for x in v_north],
    }
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    with open(args.out, "w") as f:
        json.dump(out, f, separators=(",", ":"))
    mags = [(u * u + v * v) ** 0.5 for u, v in zip(u_east, v_north)]
    sp_min = min(mags) if mags else 0
    sp_max = max(mags) if mags else 0
    sp_mean = (sum(mags) / len(mags)) if mags else 0
    print(
        f"wrote {args.out}: {len(u_east)} hourly samples from {times[0]}, "
        f"|wind| min/mean/max = {sp_min:.1f}/{sp_mean:.1f}/{sp_max:.1f} m/s"
    )


if __name__ == "__main__":
    main()
