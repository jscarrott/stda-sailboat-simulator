use anyhow::{Context, Result};
use plotters::prelude::*;
use std::path::Path;

use crate::chart::Chart;
use crate::planner::simplify;
use crate::route::Route;
use crate::scenario::SimResult;
use crate::state::{POS_X, POS_Y, VEL_X, VEL_Y};

const LAND_FILL: RGBColor = RGBColor(225, 215, 190);
const LAND_STROKE: RGBColor = RGBColor(120, 100, 70);
const ROUTE_LEG: RGBColor = RGBColor(50, 50, 50);
const ACCEPTANCE_RING: RGBColor = RGBColor(180, 180, 180);

/// Plot the boat's XY trajectory. Optional `chart` is rendered first as
/// filled land polygons, then the route legs, waypoint markers, and
/// boat track go on top. Saves a PNG at `out_path`.
pub fn plot_trajectory(
    result: &SimResult,
    route: Option<&Route>,
    chart: Option<&Chart>,
    out_path: &Path,
) -> Result<()> {
    if let Some(parent) = out_path.parent() {
        std::fs::create_dir_all(parent).context("creating figs directory")?;
    }

    // Bounding box covers track + waypoints + chart.
    let mut xs: Vec<f64> = result.x.iter().map(|s| s[POS_X]).collect();
    let mut ys: Vec<f64> = result.x.iter().map(|s| s[POS_Y]).collect();
    if let Some(r) = route {
        for w in &r.waypoints {
            xs.push(w.x);
            ys.push(w.y);
        }
    }
    if let Some(c) = chart {
        if let Some((xmin, xmax, ymin, ymax)) = c.bbox() {
            xs.push(xmin);
            xs.push(xmax);
            ys.push(ymin);
            ys.push(ymax);
        }
    }
    let (xmin, xmax, ymin, ymax) = bbox(&xs, &ys, 100.0);

    // Square aspect (we have square PNG dims) — pad the narrower axis.
    let span = (xmax - xmin).max(ymax - ymin);
    let cx = (xmin + xmax) / 2.0;
    let cy = (ymin + ymax) / 2.0;
    let half = span / 2.0;
    let (xmin, xmax, ymin, ymax) = (cx - half, cx + half, cy - half, cy + half);

    let img_size = 1024;
    let root = BitMapBackend::new(out_path, (img_size, img_size)).into_drawing_area();
    root.fill(&RGBColor(235, 245, 252))?; // sea-blue background

    let title = match (route, chart) {
        (Some(r), Some(c)) => format!("{} on {}", r.name, c.name),
        (Some(r), None) => format!("Route: {}", r.name),
        (None, Some(c)) => c.name.clone(),
        (None, None) => "Trajectory".to_string(),
    };

    let mut builder = ChartBuilder::on(&root);
    builder
        .caption(title, ("sans-serif", 28))
        .margin(20)
        .x_label_area_size(40)
        .y_label_area_size(50);
    let mut chart_area = builder.build_cartesian_2d(xmin..xmax, ymin..ymax)?;
    chart_area
        .configure_mesh()
        .x_desc("x east (m)")
        .y_desc("y north (m)")
        .axis_desc_style(("sans-serif", 16))
        .draw()?;

    // Land first, so route + track sit on top.
    if let Some(c) = chart {
        for poly in &c.polygons {
            let pts: Vec<(f64, f64)> = poly.points.iter().map(|p| (p[0], p[1])).collect();
            if poly.closed && pts.len() >= 3 {
                chart_area.draw_series(std::iter::once(Polygon::new(
                    pts.clone(),
                    ShapeStyle::from(&LAND_FILL).filled(),
                )))?;
            }
            chart_area.draw_series(std::iter::once(PathElement::new(
                pts,
                ShapeStyle::from(&LAND_STROKE).stroke_width(1),
            )))?;
        }
    }

    if let Some(r) = route {
        // Leg lines
        chart_area
            .draw_series(r.waypoints.windows(2).map(|w| {
                PathElement::new(
                    vec![(w[0].x, w[0].y), (w[1].x, w[1].y)],
                    ShapeStyle::from(&ROUTE_LEG).stroke_width(1),
                )
            }))?
            .label("Route legs")
            .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], ROUTE_LEG));

        // Acceptance rings
        chart_area.draw_series(r.waypoints.iter().map(|w| {
            Circle::new(
                (w.x, w.y),
                pixels_for_radius(r.acceptance_radius, xmin, xmax, img_size),
                ShapeStyle::from(&ACCEPTANCE_RING).stroke_width(1),
            )
        }))?;

        // Waypoint markers
        chart_area
            .draw_series(r.waypoints.iter().map(|w| {
                Circle::new((w.x, w.y), 5, ShapeStyle::from(&RED).filled())
            }))?
            .label("Waypoints")
            .legend(|(x, y)| Circle::new((x + 10, y), 5, ShapeStyle::from(&RED).filled()));
    }

    let track: Vec<(f64, f64)> = result.x.iter().map(|s| (s[POS_X], s[POS_Y])).collect();
    chart_area
        .draw_series(LineSeries::new(track.iter().copied(), BLUE.stroke_width(2)))?
        .label("Boat track")
        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], BLUE));

    if let Some(&(sx, sy)) = track.first() {
        chart_area.draw_series(std::iter::once(Cross::new(
            (sx, sy),
            8,
            ShapeStyle::from(&GREEN).stroke_width(2),
        )))?;
    }

    chart_area
        .configure_series_labels()
        .background_style(WHITE.mix(0.85))
        .border_style(BLACK)
        .draw()?;

    root.present().context("flushing PNG")?;
    Ok(())
}

/// Plot a speed polar as a radial diagram: TWA measured clockwise from
/// straight upwind (top of the plot), boat speed as the radius. Both tacks
/// are drawn (the model is port/starboard symmetric). Saves a PNG at `out_path`.
pub fn plot_polar(polar: &[(f64, f64)], wind_speed: f64, out_path: &Path) -> Result<()> {
    if let Some(parent) = out_path.parent() {
        std::fs::create_dir_all(parent).context("creating figs directory")?;
    }

    let max_speed = polar.iter().map(|&(_, s)| s).fold(0.0_f64, f64::max);
    let ring_step = if max_speed <= 1.0 {
        0.2
    } else if max_speed <= 3.0 {
        0.5
    } else {
        1.0
    };
    let r_max = ((max_speed / ring_step).ceil() * ring_step).max(ring_step);
    let lim = r_max * 1.15;

    let img = 900;
    let root = BitMapBackend::new(out_path, (img, img)).into_drawing_area();
    root.fill(&WHITE)?;

    let mut builder = ChartBuilder::on(&root);
    builder
        .caption(
            format!("Speed polar @ {wind_speed:.1} m/s true wind"),
            ("sans-serif", 26),
        )
        .margin(20);
    let mut area = builder.build_cartesian_2d(-lim..lim, -lim..lim)?;

    // (twa_deg, speed) -> (x, y): 0° at the top, increasing clockwise to starboard.
    let pt = |twa_deg: f64, sp: f64| {
        let th = twa_deg.to_radians();
        (sp * th.sin(), sp * th.cos())
    };
    let grid = RGBColor(205, 205, 205);
    let label = RGBColor(110, 110, 110);

    // Concentric speed rings, labelled up the upwind axis.
    let mut sr = ring_step;
    while sr <= r_max + 1e-9 {
        let ring: Vec<(f64, f64)> = (0..=360).step_by(4).map(|d| pt(d as f64, sr)).collect();
        area.draw_series(std::iter::once(PathElement::new(
            ring,
            ShapeStyle::from(&grid).stroke_width(1),
        )))?;
        area.draw_series(std::iter::once(Text::new(
            format!("{sr:.1}"),
            pt(0.0, sr),
            ("sans-serif", 13).into_font().color(&label),
        )))?;
        sr += ring_step;
    }

    // Radial spokes + TWA labels every 30°.
    for twa in (0..=180).step_by(30) {
        let t = twa as f64;
        for sign in [1.0_f64, -1.0] {
            if (twa == 0 || twa == 180) && sign < 0.0 {
                continue;
            }
            area.draw_series(std::iter::once(PathElement::new(
                vec![(0.0, 0.0), pt(sign * t, r_max)],
                ShapeStyle::from(&grid).stroke_width(1),
            )))?;
        }
        let (lx, ly) = pt(t, r_max * 1.07);
        area.draw_series(std::iter::once(Text::new(
            format!("{twa}°"),
            (lx, ly),
            ("sans-serif", 14).into_font().color(&BLACK),
        )))?;
    }

    // The polar curve: starboard (TWA +) then mirrored round to port (TWA -).
    let curve: Vec<(f64, f64)> = polar
        .iter()
        .map(|&(t, s)| pt(t, s))
        .chain(polar.iter().rev().map(|&(t, s)| pt(-t, s)))
        .collect();
    area.draw_series(std::iter::once(PathElement::new(curve, BLUE.stroke_width(2))))?;
    area.draw_series(
        polar
            .iter()
            .map(|&(t, s)| Circle::new(pt(t, s), 3, ShapeStyle::from(&RED).filled())),
    )?;

    root.present().context("flushing polar PNG")?;
    Ok(())
}

/// Overlay several boats' tracks (e.g. a hull-size sweep) on one chart, each
/// in its own colour with a labelled legend. Saves a PNG at `out_path`.
pub fn plot_fleet(
    tracks: &[(String, &SimResult)],
    route: &Route,
    chart: Option<&Chart>,
    out_path: &Path,
) -> Result<()> {
    if let Some(parent) = out_path.parent() {
        std::fs::create_dir_all(parent).context("creating figs directory")?;
    }

    // Bounding box over every track + route waypoints + chart.
    let mut xs: Vec<f64> = Vec::new();
    let mut ys: Vec<f64> = Vec::new();
    for (_, r) in tracks {
        for s in &r.x {
            xs.push(s[POS_X]);
            ys.push(s[POS_Y]);
        }
    }
    for w in &route.waypoints {
        xs.push(w.x);
        ys.push(w.y);
    }
    if let Some(c) = chart {
        if let Some((xmin, xmax, ymin, ymax)) = c.bbox() {
            xs.push(xmin);
            xs.push(xmax);
            ys.push(ymin);
            ys.push(ymax);
        }
    }
    let (xmin, xmax, ymin, ymax) = bbox(&xs, &ys, 200.0);
    let span = (xmax - xmin).max(ymax - ymin);
    let (cx, cy, half) = ((xmin + xmax) / 2.0, (ymin + ymax) / 2.0, span / 2.0);
    let (xmin, xmax, ymin, ymax) = (cx - half, cx + half, cy - half, cy + half);

    let img_size = 1024;
    let root = BitMapBackend::new(out_path, (img_size, img_size)).into_drawing_area();
    root.fill(&RGBColor(235, 245, 252))?;

    let title = match chart {
        Some(c) => format!("Fleet: {} on {}", route.name, c.name),
        None => format!("Fleet: {}", route.name),
    };
    let mut builder = ChartBuilder::on(&root);
    builder
        .caption(title, ("sans-serif", 28))
        .margin(20)
        .x_label_area_size(40)
        .y_label_area_size(50);
    let mut area = builder.build_cartesian_2d(xmin..xmax, ymin..ymax)?;
    area.configure_mesh()
        .x_desc("x east (m)")
        .y_desc("y north (m)")
        .axis_desc_style(("sans-serif", 16))
        .draw()?;

    if let Some(c) = chart {
        for poly in &c.polygons {
            let pts: Vec<(f64, f64)> = poly.points.iter().map(|p| (p[0], p[1])).collect();
            if poly.closed && pts.len() >= 3 {
                area.draw_series(std::iter::once(Polygon::new(
                    pts.clone(),
                    ShapeStyle::from(&LAND_FILL).filled(),
                )))?;
            }
            area.draw_series(std::iter::once(PathElement::new(
                pts,
                ShapeStyle::from(&LAND_STROKE).stroke_width(1),
            )))?;
        }
    }

    // Faint route legs for reference.
    area.draw_series(route.waypoints.windows(2).map(|w| {
        PathElement::new(
            vec![(w[0].x, w[0].y), (w[1].x, w[1].y)],
            ShapeStyle::from(&ACCEPTANCE_RING).stroke_width(1),
        )
    }))?;

    const PALETTE: [RGBColor; 6] = [
        RGBColor(0, 90, 200),   // blue
        RGBColor(210, 60, 30),  // red
        RGBColor(30, 150, 70),  // green
        RGBColor(200, 130, 0),  // amber
        RGBColor(140, 40, 160), // purple
        RGBColor(0, 150, 170),  // teal
    ];
    for (i, (label, r)) in tracks.iter().enumerate() {
        let color = PALETTE[i % PALETTE.len()];
        let track: Vec<(f64, f64)> = r.x.iter().map(|s| (s[POS_X], s[POS_Y])).collect();
        area.draw_series(LineSeries::new(track, color.stroke_width(2)))?
            .label(label.clone())
            .legend(move |(x, y)| {
                PathElement::new(vec![(x, y), (x + 22, y)], color.stroke_width(3))
            });
    }

    area.configure_series_labels()
        .background_style(WHITE.mix(0.9))
        .border_style(BLACK)
        .label_font(("sans-serif", 16))
        .draw()?;

    root.present().context("flushing fleet PNG")?;
    Ok(())
}

fn bbox(xs: &[f64], ys: &[f64], pad: f64) -> (f64, f64, f64, f64) {
    let xmin = xs.iter().cloned().fold(f64::INFINITY, f64::min) - pad;
    let xmax = xs.iter().cloned().fold(f64::NEG_INFINITY, f64::max) + pad;
    let ymin = ys.iter().cloned().fold(f64::INFINITY, f64::min) - pad;
    let ymax = ys.iter().cloned().fold(f64::NEG_INFINITY, f64::max) + pad;
    (xmin, xmax, ymin, ymax)
}

fn pixels_for_radius(radius: f64, xmin: f64, xmax: f64, img_size: u32) -> i32 {
    let world_per_pixel = (xmax - xmin) / img_size as f64;
    (radius / world_per_pixel).max(2.0) as i32
}

/// Write a self-contained interactive plotly.js HTML page alongside the fleet
/// PNG: each track is a colour-coded scatter line, the chart polygons are land
/// fills, and hovering a track point shows time, speed, and sail/rudder
/// angles. plotly.js is loaded from a CDN — no extra dependencies, just open
/// the file in a browser. Track points are decimated to keep the HTML small
/// and the hover interaction snappy.
/// Wind time series for the HTML inset: hourly (or whatever cadence the
/// forecast cache uses) speed in m/s and meteorological "FROM" direction in
/// compass degrees. Built from the same JSON the simulator loads.
pub struct WindSeries {
    pub dt_s: f64,
    pub speed: Vec<f64>,
    pub direction_deg_from: Vec<f64>,
}

/// Load a wind JSON (the one fetch_wind.py writes) and project it for the
/// inset plot. The simulator already consumes the file via TabulatedWind;
/// this is a cheap second parse so the HTML can show it without exposing the
/// model's internals.
pub fn load_wind_series(path: &Path) -> Result<WindSeries> {
    let (dt_s, u, v) = read_uv_series(path, "wind")?;
    let speed: Vec<f64> = u.iter().zip(&v).map(|(a, b)| (a * a + b * b).sqrt()).collect();
    // Compass "FROM" direction: u,v are velocity components (toward), so the
    // direction the wind comes FROM is atan2(-u, -v) in math terms.
    let direction_deg_from: Vec<f64> = u
        .iter()
        .zip(&v)
        .map(|(a, b)| (-a).atan2(-b).to_degrees().rem_euclid(360.0))
        .collect();
    Ok(WindSeries { dt_s, speed, direction_deg_from })
}

/// Tide-current time series for the HTML inset: hourly current speed (m/s)
/// and meteorological-style "TOWARD" direction (compass deg the stream flows
/// toward). Built from the same JSON the simulator's TabulatedCurrent loads.
pub struct TideSeries {
    pub dt_s: f64,
    pub speed: Vec<f64>,
    pub direction_deg_toward: Vec<f64>,
}

/// Load a tide JSON (the one fetch_tides.py writes) and project it for the
/// inset. Tides are described conventionally by the direction the stream
/// flows TOWARD (e.g. "flood sets NE"), the opposite of meteorological wind.
pub fn load_tide_series(path: &Path) -> Result<TideSeries> {
    let (dt_s, u, v) = read_uv_series(path, "tide")?;
    let speed: Vec<f64> = u.iter().zip(&v).map(|(a, b)| (a * a + b * b).sqrt()).collect();
    let direction_deg_toward: Vec<f64> = u
        .iter()
        .zip(&v)
        .map(|(a, b)| a.atan2(*b).to_degrees().rem_euclid(360.0))
        .collect();
    Ok(TideSeries { dt_s, speed, direction_deg_toward })
}

/// Shared loader for the (dt_s, u_east, v_north) JSON shape used by both
/// fetch_wind.py and fetch_tides.py. The label is just for error context.
fn read_uv_series(path: &Path, label: &str) -> Result<(f64, Vec<f64>, Vec<f64>)> {
    #[derive(serde::Deserialize)]
    struct UvFile {
        dt_s: f64,
        u_east: Vec<f64>,
        v_north: Vec<f64>,
    }
    let file = std::fs::File::open(path)
        .with_context(|| format!("opening {} data {}", label, path.display()))?;
    let f: UvFile = serde_json::from_reader(file)
        .with_context(|| format!("parsing {} data {}", label, path.display()))?;
    Ok((f.dt_s, f.u_east, f.v_north))
}

pub fn write_fleet_html(
    tracks: &[(String, &SimResult)],
    route: &Route,
    chart: Option<&Chart>,
    wind: Option<&WindSeries>,
    tide: Option<&TideSeries>,
    title: &str,
    out_path: &Path,
) -> Result<()> {
    use serde_json::{json, Value};
    if let Some(parent) = out_path.parent() {
        std::fs::create_dir_all(parent).context("creating output directory")?;
    }

    const MAX_POINTS_PER_TRACK: usize = 3000;

    let track_data: Vec<Value> = tracks
        .iter()
        .map(|(label, r)| {
            let n = r.x.len();
            let stride = (n / MAX_POINTS_PER_TRACK).max(1);
            let mut t = Vec::new();
            let mut xs = Vec::new();
            let mut ys = Vec::new();
            let mut speed = Vec::new();
            let mut sail_deg = Vec::new();
            let mut rud_deg = Vec::new();
            let mut i = 0;
            while i < n {
                let s = &r.x[i];
                t.push(r.t[i]);
                xs.push(s[POS_X]);
                ys.push(s[POS_Y]);
                speed.push((s[VEL_X] * s[VEL_X] + s[VEL_Y] * s[VEL_Y]).sqrt());
                // sail/rudder have one entry per step (n-1 total); fall back to
                // the last available entry for the final track point.
                let si = i.min(r.sail.len().saturating_sub(1));
                sail_deg.push(r.sail[si].to_degrees());
                rud_deg.push(r.rudder[si].to_degrees());
                i += stride;
            }
            // Sequential waypoint captures (mirrors main::report_route_progress):
            // for each wp in order, advance through the un-decimated track and
            // record the first index inside its acceptance ring. Soft waypoints
            // use the wider fly-by radius.
            let accept = route.acceptance_radius;
            let fly_by = route.fly_by_radius;
            let mut captures: Vec<Value> = Vec::new();
            let mut scan_from = 0usize;
            for (wi, wp) in route.waypoints.iter().enumerate().skip(1) {
                let r_acc = if wp.soft { accept.max(fly_by) } else { accept };
                let mut hit: Option<usize> = None;
                for j in scan_from..r.x.len() {
                    let s = &r.x[j];
                    let d = ((s[POS_X] - wp.x).powi(2) + (s[POS_Y] - wp.y).powi(2)).sqrt();
                    if d < r_acc {
                        hit = Some(j);
                        break;
                    }
                }
                if let Some(j) = hit {
                    let s = &r.x[j];
                    captures.push(json!({
                        "wp": wi, "t": r.t[j], "x": s[POS_X], "y": s[POS_Y]
                    }));
                    scan_from = j;
                }
            }
            json!({
                "label": label,
                "t": t, "x": xs, "y": ys,
                "speed": speed, "sail": sail_deg, "rudder": rud_deg,
                "captures": captures,
            })
        })
        .collect();

    // Decimate the coastline polygons before embedding — Lundy + the mainland
    // carry 10k+ vertices between them, and SVG path rendering chokes on that
    // during pan/zoom. RDP at 50 m matches the obstacle-avoidance simplification
    // and is well below screen-pixel resolution, so the visual is identical.
    let chart_polys: Vec<Value> = chart
        .map(|c| {
            c.polygons
                .iter()
                .map(|p| {
                    let pts: Vec<(f64, f64)> = p.points.iter().map(|q| (q[0], q[1])).collect();
                    let simple = if pts.len() > 30 { simplify(&pts, 50.0) } else { pts };
                    let arr: Vec<[f64; 2]> = simple.iter().map(|(x, y)| [*x, *y]).collect();
                    json!({ "closed": p.closed, "points": arr })
                })
                .collect()
        })
        .unwrap_or_default();

    let route_pts: Vec<Value> = route
        .waypoints
        .iter()
        .map(|w| json!({ "x": w.x, "y": w.y }))
        .collect();

    let wind_json = wind.map(|w| {
        let t_hours: Vec<f64> = (0..w.speed.len())
            .map(|i| (i as f64) * w.dt_s / 3600.0)
            .collect();
        json!({
            "t_hours": t_hours,
            "speed": w.speed,
            "dir_deg_from": w.direction_deg_from,
        })
    });
    let tide_json = tide.map(|t| {
        let t_hours: Vec<f64> = (0..t.speed.len())
            .map(|i| (i as f64) * t.dt_s / 3600.0)
            .collect();
        json!({
            "t_hours": t_hours,
            "speed": t.speed,
            "dir_deg_toward": t.direction_deg_toward,
        })
    });
    let data = json!({
        "title": title,
        "tracks": track_data,
        "chart": { "polygons": chart_polys },
        "route": { "name": route.name, "waypoints": route_pts },
        "wind": wind_json,
        "tide": tide_json,
    });
    let data_json = serde_json::to_string(&data).context("serializing fleet data")?;

    let html = HTML_TEMPLATE.replace("__DATA__", &data_json);
    std::fs::write(out_path, html).context("writing HTML")?;
    Ok(())
}

const HTML_TEMPLATE: &str = r##"<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>sailboat fleet</title>
<script src="https://cdn.plot.ly/plotly-2.35.2.min.js" charset="utf-8"></script>
<style>
  html, body { margin: 0; padding: 0; height: 100%; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif; }
  #plot { width: 100vw; height: 100vh; }
</style>
</head>
<body>
<div id="plot"></div>
<script>
const D = __DATA__;

const LAND_FILL = 'rgba(225,215,190,0.85)';
const LAND_STROKE = 'rgba(120,100,70,1)';
const traces = [];

// Land polygons (closed → filled), open coastlines (line only).
for (const poly of D.chart.polygons) {
  if (poly.closed && poly.points.length >= 3) {
    traces.push({
      x: poly.points.map(p => p[0]),
      y: poly.points.map(p => p[1]),
      fill: 'toself', fillcolor: LAND_FILL,
      line: {color: LAND_STROKE, width: 1},
      type: 'scatter', mode: 'lines',
      hoverinfo: 'skip', showlegend: false,
    });
  } else {
    traces.push({
      x: poly.points.map(p => p[0]),
      y: poly.points.map(p => p[1]),
      type: 'scatter', mode: 'lines',
      line: {color: LAND_STROKE, width: 1},
      hoverinfo: 'skip', showlegend: false,
    });
  }
}

// Faint route legs + red waypoint markers.
const wp = D.route.waypoints;
traces.push({
  x: wp.map(w => w.x), y: wp.map(w => w.y),
  type: 'scatter', mode: 'lines+markers',
  line: {color: 'rgba(80,80,80,0.55)', width: 1, dash: 'dot'},
  marker: {size: 6, color: 'rgba(220,40,40,0.9)'},
  name: 'route', hoverinfo: 'skip', showlegend: false,
});

// Boat tracks: line in the boat's palette colour (identity); markers
// coloured by speed (shared Viridis colourbar) so slow loops stand out.
// Capture stars per boat live in their own trace, grouped to the boat's
// legend item via `legendgroup` so one toggle hides everything for that
// boat (track + speed markers + captures).
const PALETTE = [
  'rgb(0,90,200)', 'rgb(210,60,30)', 'rgb(30,150,70)',
  'rgb(200,130,0)', 'rgb(140,40,160)', 'rgb(0,150,170)',
];
D.tracks.forEach((tr, i) => {
  const color = PALETTE[i % PALETTE.length];
  const text = tr.t.map((t, j) => {
    const h = t / 3600;
    const d = h / 24;
    return `t = ${h.toFixed(2)} h (${d.toFixed(2)} d)`
         + `<br>speed = ${tr.speed[j].toFixed(2)} m/s (${(tr.speed[j]*1.94384).toFixed(2)} kn)`
         + `<br>sail = ${tr.sail[j].toFixed(0)}°  rudder = ${tr.rudder[j].toFixed(0)}°`;
  });
  // scattergl: WebGL rendering of the boat tracks, ~10× faster pan/zoom
  // than SVG `scatter` when each track is a few thousand points.
  traces.push({
    x: tr.x, y: tr.y,
    type: 'scattergl', mode: 'lines+markers',
    line: {color, width: 1.5},
    marker: {
      size: 5,
      color: tr.speed,
      colorscale: 'Viridis',
      cmin: 0, cmax: 2.5,
      showscale: i === 0,
      colorbar: i === 0 ? {title: {text: 'speed (m/s)', side: 'right'}, len: 0.7, x: 1.02, y: 0.5} : undefined,
    },
    text, hovertemplate: '%{text}<extra>'+tr.label+'</extra>',
    name: tr.label,
    legendgroup: tr.label,
  });
  // Waypoint capture stars for this boat: hover shows wp # + capture time.
  if (tr.captures && tr.captures.length) {
    traces.push({
      x: tr.captures.map(c => c.x),
      y: tr.captures.map(c => c.y),
      text: tr.captures.map(c => `wp${c.wp} @ ${(c.t/3600).toFixed(2)} h`),
      type: 'scatter', mode: 'markers',
      marker: {
        symbol: 'star', size: 12,
        color: color,
        line: {color: 'rgba(0,0,0,0.7)', width: 1},
      },
      hovertemplate: '%{text}<extra>'+tr.label+'</extra>',
      name: tr.label + ' captures',
      legendgroup: tr.label,
      showlegend: false,
    });
  }
});

// Optional wind + tide time-series inset (top-right corner). Wind speed on
// the left axis, tide-current speed on the right (dual y-axis on a shared
// time axis). Hover either trace to see speed + direction at that hour, so
// you can correlate a loop's `t = X h` reading from the main plot with both
// the wind AND the tide at that moment.
const shapes = [];
const hasInset = !!(D.wind || D.tide);
if (hasInset) {
  shapes.push({
    type: 'rect', xref: 'paper', yref: 'paper',
    x0: 0.61, x1: 1.00, y0: 0.66, y1: 1.00,
    line: {color: 'rgba(80,80,80,0.4)', width: 1},
    fillcolor: 'rgba(255,255,255,0.9)',
    layer: 'below',
  });
}
if (D.wind) {
  traces.push({
    x: D.wind.t_hours, y: D.wind.speed,
    text: D.wind.t_hours.map((h, j) =>
      `t = ${h.toFixed(1)} h<br>wind = ${D.wind.speed[j].toFixed(1)} m/s FROM ${D.wind.dir_deg_from[j].toFixed(0)}°`),
    hovertemplate: '%{text}<extra>wind</extra>',
    type: 'scatter', mode: 'lines',
    line: {color: 'rgb(40,80,140)', width: 1.5},
    xaxis: 'x2', yaxis: 'y2',
    name: 'wind', showlegend: false,
  });
}
if (D.tide) {
  traces.push({
    x: D.tide.t_hours, y: D.tide.speed,
    text: D.tide.t_hours.map((h, j) =>
      `t = ${h.toFixed(1)} h<br>tide = ${D.tide.speed[j].toFixed(2)} m/s TOWARD ${D.tide.dir_deg_toward[j].toFixed(0)}°`),
    hovertemplate: '%{text}<extra>tide</extra>',
    type: 'scatter', mode: 'lines',
    line: {color: 'rgb(160,60,170)', width: 1.5, dash: 'dot'},
    xaxis: 'x2', yaxis: 'y3',
    name: 'tide', showlegend: false,
  });
}

const layout = {
  title: {text: D.title, font: {size: 18}},
  xaxis: {title: 'x east (m)', scaleanchor: 'y', scaleratio: 1},
  yaxis: {title: 'y north (m)'},
  xaxis2: hasInset ? {
    domain: [0.64, 0.99], anchor: 'y2',
    title: {text: 'time (h)', font: {size: 10}},
    tickfont: {size: 9}, showgrid: true, gridcolor: 'rgba(0,0,0,0.08)',
  } : undefined,
  yaxis2: hasInset ? {
    domain: [0.70, 0.97], anchor: 'x2',
    title: {text: 'wind (m/s)', font: {size: 10, color: 'rgb(40,80,140)'}},
    tickfont: {size: 9, color: 'rgb(40,80,140)'},
    showgrid: true, gridcolor: 'rgba(0,0,0,0.08)',
    rangemode: 'tozero',
  } : undefined,
  yaxis3: D.tide ? {
    domain: [0.70, 0.97], anchor: 'x2',
    overlaying: 'y2', side: 'right',
    title: {text: 'tide (m/s)', font: {size: 10, color: 'rgb(160,60,170)'}},
    tickfont: {size: 9, color: 'rgb(160,60,170)'},
    showgrid: false, rangemode: 'tozero',
  } : undefined,
  shapes,
  hovermode: 'closest',
  legend: {
    x: 0.01, y: 0.99, xanchor: 'left', yanchor: 'top',
    bgcolor: 'rgba(255,255,255,0.85)',
    title: {text: 'click to toggle · double-click to isolate', font: {size: 10}},
  },
  plot_bgcolor: 'rgb(235,245,252)',
  margin: {t: 60, l: 60, r: 90, b: 50},
};

Plotly.newPlot('plot', traces, layout, {
  responsive: true, scrollZoom: true, displaylogo: false,
  toImageButtonOptions: {filename: 'fleet'},
});
</script>
</body>
</html>
"##;
