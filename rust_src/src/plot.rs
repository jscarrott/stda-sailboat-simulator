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
