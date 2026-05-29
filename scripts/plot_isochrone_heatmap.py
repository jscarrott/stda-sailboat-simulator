"""Render an illustrative isochrone heat map for docs/planning.md.

For each grid point, computes an approximate minimum time-to-reach from
the start using a representative dinghy polar plus a steady tide drift.
The result is a heat map (dark = fast, light = slow) with isochrone
contour lines overlaid. The shape is asymmetric because of the no-go
cone (upper-left, where the boat must tack) and the tide drift (which
biases the wavefront downstream).

Not a faithful simulation of the planner's isochrone search — the real
planner is iterative and discrete — but a clear visual analogue of the
reachability surface the search is exploring.

Re-run with:  uv run python scripts/plot_isochrone_heatmap.py
"""
from __future__ import annotations

import numpy as np
import matplotlib.pyplot as plt


def main() -> None:
    # Grid (kilometres in plot coords; everything else in m / m/s).
    x = np.linspace(-12, 50, 620)
    y = np.linspace(-22, 22, 440)
    X, Y = np.meshgrid(x, y)
    R_m = np.hypot(X, Y) * 1000.0           # distance from start (m)
    theta = np.arctan2(Y, X)                # math angle from start

    # Wind from upper-left (math angle 135°): no-go cone faces NW from start.
    wind_from = np.radians(135.0)

    # Steady tide drift, ground frame (m/s) — east-ish, modest.
    tide = (0.30, 0.05)

    # Representative dinghy polar: speed (m/s) vs TWA (deg).
    twa_table   = np.array([0,   30,  40,  45,  60,   90,   120,  150,  180])
    speed_table = np.array([0.0, 0.0, 0.05, 0.35, 0.55, 0.75, 0.85, 0.7,  0.45])

    def polar(twa_rad: np.ndarray) -> np.ndarray:
        t = np.abs(twa_rad)
        t = np.where(t > np.pi, 2 * np.pi - t, t)
        return np.interp(np.degrees(t), twa_table, speed_table)

    # Per-point sail-direct estimate: boat heads at the target, polar gives
    # through-water speed, tide projected along the target direction adds
    # to ground speed.
    twa = theta - wind_from
    twa_abs = np.abs(twa)
    twa_abs = np.where(twa_abs > np.pi, 2 * np.pi - twa_abs, twa_abs)

    water_speed = polar(twa)
    tide_along = tide[0] * np.cos(theta) + tide[1] * np.sin(theta)
    direct_ground = water_speed + tide_along

    # Inside the no-go cone the boat has to tack: VMG-to-target is the
    # close-hauled speed projected onto the target direction.
    no_go = np.radians(45.0)
    close_hauled_speed = float(np.interp(45.0, twa_table, speed_table))
    tack_vmg = close_hauled_speed * np.cos(no_go - twa_abs) + tide_along
    in_no_go = twa_abs < no_go
    effective = np.where(in_no_go, tack_vmg, direct_ground)
    effective = np.maximum(effective, 1e-3)

    time_h = R_m / effective / 3600.0
    time_clip = np.clip(time_h, 0, 24)
    time_clip[R_m < 300] = 0  # don't paint a hot spot right at start

    # Render.
    fig, ax = plt.subplots(figsize=(11, 6))
    cmap = plt.cm.viridis_r  # dark = fast, bright = slow
    im = ax.contourf(X, Y, time_clip, levels=np.linspace(0, 24, 25), cmap=cmap)

    iso_levels = [2, 4, 7, 10, 14, 18, 22]
    cs = ax.contour(X, Y, time_clip, levels=iso_levels,
                    colors='white', linewidths=1.4, alpha=0.85)
    ax.clabel(cs, inline=True, fontsize=9, fmt='%d h')

    # Start.
    ax.plot(0, 0, 'o', markersize=13, markerfacecolor='white',
            markeredgecolor='black', markeredgewidth=2)
    ax.annotate('start', (0, 0), xytext=(1.6, 1.0),
                fontsize=11, fontweight='bold', color='white')

    # Destination.
    ax.plot(42, -3, '^', markersize=14, markerfacecolor='white',
            markeredgecolor='black', markeredgewidth=2)
    ax.annotate('dest', (42, -3), xytext=(43, -1.2),
                fontsize=11, fontweight='bold', color='white')

    # Wind arrow (top-left), pointing into the canvas (the way the wind blows).
    ax.annotate('', xy=(-3, 14), xytext=(-9, 19),
                arrowprops=dict(arrowstyle='->', color='#ff944d', lw=2.5))
    ax.text(-11, 20, 'wind', color='#ff944d', fontsize=12, fontweight='bold')

    # Tide arrow (bottom-left), pointing east.
    ax.annotate('', xy=(-1, -17), xytext=(-9, -17),
                arrowprops=dict(arrowstyle='->', color='#f85149', lw=2.5))
    ax.text(-11, -15.5, 'tide drift', color='#f85149', fontsize=12, fontweight='bold')

    ax.set_aspect('equal')
    ax.set_xlabel('east (km)')
    ax.set_ylabel('north (km)')
    ax.set_title(
        'Isochrone heat map — minimum time to reach (hours)\n'
        'white lines = isochrones; dark = fast, bright = slow.\n'
        'No-go cone (upper-left of start, into the wind) takes longer because the boat must tack.\n'
        'Tide drift pushes the isochrones east — downwind targets are fastest of all.'
    )
    fig.colorbar(im, ax=ax, label='time to reach (h)', fraction=0.04, pad=0.02)

    fig.tight_layout()
    out = 'docs/diagrams/isochrone_heatmap.png'
    fig.savefig(out, dpi=130, bbox_inches='tight', facecolor='white')
    print(f'wrote {out}')


if __name__ == '__main__':
    main()
