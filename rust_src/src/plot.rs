use anyhow::{Context, Result};
use plotters::prelude::*;
use std::path::Path;

use crate::route::Route;
use crate::scenario::SimResult;
use crate::state::{POS_X, POS_Y};

/// Plot the boat's XY trajectory with the route overlaid: dashed leg
/// lines, waypoint markers with acceptance-radius circles, and a start
/// marker. Saves a PNG at `out_path`.
pub fn plot_trajectory(result: &SimResult, route: Option<&Route>, out_path: &Path) -> Result<()> {
    if let Some(parent) = out_path.parent() {
        std::fs::create_dir_all(parent).context("creating figs directory")?;
    }

    let mut xs: Vec<f64> = result.x.iter().map(|s| s[POS_X]).collect();
    let mut ys: Vec<f64> = result.x.iter().map(|s| s[POS_Y]).collect();
    if let Some(r) = route {
        for w in &r.waypoints {
            xs.push(w.x);
            ys.push(w.y);
        }
    }
    let (xmin, xmax, ymin, ymax) = bbox(&xs, &ys, 10.0);

    // Force square aspect so trajectories aren't visually distorted.
    let span = (xmax - xmin).max(ymax - ymin);
    let cx = (xmin + xmax) / 2.0;
    let cy = (ymin + ymax) / 2.0;
    let half = span / 2.0;
    let (xmin, xmax, ymin, ymax) = (cx - half, cx + half, cy - half, cy + half);

    let root = BitMapBackend::new(out_path, (1024, 1024)).into_drawing_area();
    root.fill(&WHITE)?;

    let title = route
        .map(|r| format!("Route: {}", r.name))
        .unwrap_or_else(|| "Trajectory".to_string());

    let mut chart = ChartBuilder::on(&root)
        .caption(title, ("sans-serif", 28))
        .margin(20)
        .x_label_area_size(40)
        .y_label_area_size(50)
        .build_cartesian_2d(xmin..xmax, ymin..ymax)?;

    chart
        .configure_mesh()
        .x_desc("x (m)")
        .y_desc("y (m)")
        .axis_desc_style(("sans-serif", 16))
        .draw()?;

    if let Some(r) = route {
        // Leg lines (dashed-ish by drawing alternating short segments).
        chart
            .draw_series(r.waypoints.windows(2).map(|w| {
                PathElement::new(
                    vec![(w[0].x, w[0].y), (w[1].x, w[1].y)],
                    ShapeStyle::from(&BLACK).stroke_width(1),
                )
            }))?
            .label("Route legs")
            .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], BLACK));

        // Acceptance-radius rings
        chart.draw_series(r.waypoints.iter().map(|w| {
            Circle::new((w.x, w.y), pixels_for_radius(r.acceptance_radius, xmin, xmax),
                        ShapeStyle::from(&RGBColor(200, 200, 200)).stroke_width(1))
        }))?;

        // Waypoint markers
        chart
            .draw_series(r.waypoints.iter().map(|w| {
                Circle::new((w.x, w.y), 5, ShapeStyle::from(&RED).filled())
            }))?
            .label("Waypoints")
            .legend(|(x, y)| Circle::new((x + 10, y), 5, ShapeStyle::from(&RED).filled()));
    }

    // Trajectory
    let track: Vec<(f64, f64)> = result.x.iter().map(|s| (s[POS_X], s[POS_Y])).collect();
    chart
        .draw_series(LineSeries::new(track.iter().copied(), &BLUE))?
        .label("Boat track")
        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], BLUE));

    // Start marker (an X)
    if let Some(&(sx, sy)) = track.first() {
        chart.draw_series(std::iter::once(Cross::new((sx, sy), 8, ShapeStyle::from(&GREEN).stroke_width(2))))?;
    }

    chart
        .configure_series_labels()
        .background_style(WHITE.mix(0.8))
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

/// Convert a world-units radius to backend pixels so the acceptance
/// rings stay correctly sized on the plot.
fn pixels_for_radius(radius: f64, xmin: f64, xmax: f64) -> i32 {
    let world_per_pixel = (xmax - xmin) / 1024.0;
    (radius / world_per_pixel).max(2.0) as i32
}
