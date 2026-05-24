#!/usr/bin/env python3
"""Fetch Lundy Island's coastline from OpenStreetMap, chain coastline
ways into closed rings, project to a tangent-plane local meter frame
centered on the island, and write the result to
rust_src/charts/lundy_coastline.json. Re-run when OSM updates.

Usage (from rust_src/):
    python scripts/fetch_lundy.py
"""
import json
import os
import sys
import time
import urllib.parse
import urllib.request
from math import cos, radians

# Lundy Island — OSM relation 3067397.
# Tangent-plane origin: rough geographic centre of the island.
LAT0 = 51.1735
LON0 = -4.6680
RELATION_ID = 3067397
OVERPASS = "https://overpass-api.de/api/interpreter"
QUERY = f"[out:json][timeout:25];relation({RELATION_ID});out geom;"

HERE = os.path.dirname(os.path.abspath(__file__))
RUST_SRC = os.path.dirname(HERE)
OUT_PATH = os.path.join(RUST_SRC, "charts", "lundy_coastline.json")


def fetch():
    data = urllib.parse.urlencode({"data": QUERY}).encode()
    req = urllib.request.Request(
        OVERPASS,
        data=data,
        headers={"User-Agent": "stda-sailboat-sim/0.1", "Accept": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=60) as r:
        return json.loads(r.read())


def project(lat, lon):
    """Tangent-plane local meters at LAT0, LON0. East x, north y."""
    m_per_deg_lat = 111320.0
    m_per_deg_lon = 111320.0 * cos(radians(LAT0))
    x = (lon - LON0) * m_per_deg_lon
    y = (lat - LAT0) * m_per_deg_lat
    return [x, y]


def coords_key(pt):
    # Geometry comes back with full float precision; same node → same coords.
    return (round(pt["lat"], 9), round(pt["lon"], 9))


def chain_ways(ways):
    """Greedy ring assembly: pop a way, extend its head/tail by any way
    whose endpoint matches, repeat. Returns a list of point chains.
    """
    pending = [list(w["geometry"]) for w in ways]
    rings = []
    while pending:
        chain = pending.pop(0)
        extended = True
        while extended:
            extended = False
            head = coords_key(chain[0])
            tail = coords_key(chain[-1])
            for i, w in enumerate(pending):
                w_head = coords_key(w[0])
                w_tail = coords_key(w[-1])
                if w_head == tail:
                    chain.extend(w[1:])
                    pending.pop(i)
                    extended = True
                    break
                if w_tail == tail:
                    chain.extend(reversed(w[:-1]))
                    pending.pop(i)
                    extended = True
                    break
                if w_tail == head:
                    chain = list(w[:-1]) + chain
                    pending.pop(i)
                    extended = True
                    break
                if w_head == head:
                    chain = list(reversed(w[1:])) + chain
                    pending.pop(i)
                    extended = True
                    break
        rings.append(chain)
    return rings


def main():
    print(f"fetching relation {RELATION_ID} from {OVERPASS} ...")
    payload = fetch()
    rel = payload["elements"][0]
    members = [m for m in rel["members"] if m.get("role") == "outer"]
    print(f"  got {len(members)} outer ways")
    rings = chain_ways(members)
    print(f"  joined into {len(rings)} rings; sizes: {[len(r) for r in rings]}")

    polygons = []
    for i, ring in enumerate(rings):
        head = coords_key(ring[0])
        tail = coords_key(ring[-1])
        closed = head == tail
        points = [project(p["lat"], p["lon"]) for p in ring]
        polygons.append(
            {
                "name": "main_island" if i == 0 else f"feature_{i}",
                "closed": closed,
                "points": points,
            }
        )

    out = {
        "name": "Lundy",
        "origin": {"lat": LAT0, "lon": LON0},
        "projection": "tangent_plane_meters",
        "osm_relation_id": RELATION_ID,
        "fetched_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "polygons": polygons,
    }
    os.makedirs(os.path.dirname(OUT_PATH), exist_ok=True)
    with open(OUT_PATH, "w") as f:
        json.dump(out, f, separators=(",", ":"))
    total_pts = sum(len(p["points"]) for p in polygons)
    closed_count = sum(1 for p in polygons if p["closed"])
    print(f"wrote {OUT_PATH}: {len(polygons)} polygons ({closed_count} closed), {total_pts} points")


if __name__ == "__main__":
    main()
