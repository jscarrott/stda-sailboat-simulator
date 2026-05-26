use anyhow::{Context, Result};
use plotters::prelude::*;
use std::path::Path;

use crate::chart::Chart;
use crate::route::Route;
use crate::scenario::SimResult;
use crate::state::{POS_X, POS_Y};

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
