#!/usr/bin/env python3
"""Fetch the North Devon coastline + Lundy Island from OpenStreetMap,
project everything to a tangent-plane local metre frame, and write to
rust_src/charts/north_devon.json. Re-run when OSM updates.

Two Overpass queries are issued:
  1. relation(3067397) — Lundy Island, returned as 28 outer ways that
     greedily chain into a single closed ring.
  2. way[natural=coastline] in the bounding box covering Lundy plus
     the Saunton Sands → Ilfracombe → Heddon's Mouth stretch of the
     North Devon coast. These are chained best-effort; mainland chains
     are left open (they exit the bbox at both ends).

Usage (from rust_src/):
    python scripts/fetch_coastline.py
"""
import json
import os
import time
import urllib.parse
import urllib.request
from math import cos, radians

# Tangent-plane origin: rough centre of Lundy Island. Chosen so the
# physics frame is roughly equidistant to the SW corner of the island
# and the harbour mouth at Ilfracombe (~39 km east).
LAT0 = 51.1735
LON0 = -4.6680

# Bounding box covering Lundy + the North Devon coast from Saunton
# Sands round to Heddon's Mouth. (south, west, north, east)
BBOX = (51.05, -4.72, 51.26, -4.00)

LUNDY_RELATION = 3067397
OVERPASS = "https://overpass-api.de/api/interpreter"

HERE = os.path.dirname(os.path.abspath(__file__))
RUST_SRC = os.path.dirname(HERE)
OUT_PATH = os.path.join(RUST_SRC, "charts", "north_devon.json")


def query(q: str):
    data = urllib.parse.urlencode({"data": q}).encode()
    req = urllib.request.Request(
        OVERPASS,
        data=data,
        headers={"User-Agent": "stda-sailboat-sim/0.1", "Accept": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=120) as r:
        return json.loads(r.read())


def project(lat, lon):
    """Tangent-plane local metres at LAT0, LON0. East x, north y."""
    m_per_deg_lat = 111320.0
    m_per_deg_lon = 111320.0 * cos(radians(LAT0))
    return [(lon - LON0) * m_per_deg_lon, (lat - LAT0) * m_per_deg_lat]


def coords_key(pt):
    return (round(pt["lat"], 9), round(pt["lon"], 9))


def chain_ways(ways):
    """Greedy ring assembly: extend a chain's head/tail by any way whose
    endpoint matches. Returns a list of point chains.
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
                    chain.extend(w[1:]); pending.pop(i); extended = True; break
                if w_tail == tail:
                    chain.extend(reversed(w[:-1])); pending.pop(i); extended = True; break
                if w_tail == head:
                    chain = list(w[:-1]) + chain; pending.pop(i); extended = True; break
                if w_head == head:
                    chain = list(reversed(w[1:])) + chain; pending.pop(i); extended = True; break
        rings.append(chain)
    return rings


def fetch_lundy():
    print(f"fetching Lundy (relation {LUNDY_RELATION}) ...")
    payload = query(f"[out:json][timeout:60];relation({LUNDY_RELATION});out geom;")
    rel = payload["elements"][0]
    ways = [m for m in rel["members"] if m.get("role") == "outer"]
    print(f"  {len(ways)} outer ways, {sum(len(w['geometry']) for w in ways)} nodes")
    rings = chain_ways(ways)
    print(f"  -> {len(rings)} ring(s), sizes {[len(r) for r in rings]}")
    return rings


def fetch_mainland(lundy_node_keys):
    s, w, n, e = BBOX
    print(f"fetching North Devon coastline in bbox {BBOX} ...")
    q = (
        f"[out:json][timeout:120];"
        f"way[\"natural\"=\"coastline\"]({s},{w},{n},{e});"
        f"out geom;"
    )
    payload = query(q)
    ways = [el for el in payload["elements"] if el["type"] == "way" and "geometry" in el]
    print(f"  {len(ways)} coastline ways, {sum(len(w['geometry']) for w in ways)} nodes")
    # Drop ways that belong to Lundy (already in ring above).
    mainland = []
    for w_ in ways:
        keys = {coords_key(p) for p in w_["geometry"]}
        if keys & lundy_node_keys:
            continue
        mainland.append(w_)
    print(f"  {len(mainland)} mainland ways after removing Lundy overlap")
    rings = chain_ways(mainland)
    print(f"  -> {len(rings)} chain(s), sizes {[len(r) for r in rings]}")
    return rings


def to_polygon(name, ring, closed_override=None):
    head = coords_key(ring[0])
    tail = coords_key(ring[-1])
    closed = head == tail if closed_override is None else closed_override
    return {
        "name": name,
        "closed": closed,
        "points": [project(p["lat"], p["lon"]) for p in ring],
    }


def main():
    lundy_rings = fetch_lundy()
    lundy_node_keys = {coords_key(p) for ring in lundy_rings for p in ring}
    mainland_rings = fetch_mainland(lundy_node_keys)

    polygons = []
    for i, ring in enumerate(lundy_rings):
        polygons.append(to_polygon("lundy" if i == 0 else f"lundy_{i}", ring))
    for i, ring in enumerate(mainland_rings):
        polygons.append(to_polygon(f"mainland_{i}", ring))

    out = {
        "name": "North Devon & Lundy",
        "origin": {"lat": LAT0, "lon": LON0},
        "projection": "tangent_plane_metres",
        "bbox_geographic": {"south": BBOX[0], "west": BBOX[1], "north": BBOX[2], "east": BBOX[3]},
        "osm_relation_id_lundy": LUNDY_RELATION,
        "fetched_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "polygons": polygons,
    }
    os.makedirs(os.path.dirname(OUT_PATH), exist_ok=True)
    with open(OUT_PATH, "w") as f:
        json.dump(out, f, separators=(",", ":"))
    total = sum(len(p["points"]) for p in polygons)
    closed = sum(1 for p in polygons if p["closed"])
    print(f"wrote {OUT_PATH}: {len(polygons)} polygons ({closed} closed), {total} points")


if __name__ == "__main__":
    main()
