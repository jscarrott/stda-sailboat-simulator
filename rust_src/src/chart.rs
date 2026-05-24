use anyhow::{Context, Result};
use serde::Deserialize;
use std::fs::File;
use std::path::Path;

/// A coastline / obstacle chart in the simulator's local meter frame.
/// Cached by scripts/fetch_lundy.py from OpenStreetMap; `origin` records
/// the geographic centre used for the tangent-plane projection so the
/// chart can be re-projected or registered against other geographic data
/// later.
#[derive(Deserialize, Debug, Clone)]
pub struct Chart {
    pub name: String,
    pub origin: GeoOrigin,
    pub projection: String,
    pub polygons: Vec<ChartPolygon>,
}

#[derive(Deserialize, Debug, Clone, Copy)]
pub struct GeoOrigin {
    pub lat: f64,
    pub lon: f64,
}

#[derive(Deserialize, Debug, Clone)]
pub struct ChartPolygon {
    pub name: String,
    pub closed: bool,
    pub points: Vec<[f64; 2]>,
}

impl Chart {
    pub fn load(path: &Path) -> Result<Self> {
        let file = File::open(path)
            .with_context(|| format!("opening chart {}", path.display()))?;
        let chart: Chart = serde_json::from_reader(file)
            .with_context(|| format!("parsing chart {}", path.display()))?;
        Ok(chart)
    }

    /// Bounding box `(xmin, xmax, ymin, ymax)` across all polygons.
    pub fn bbox(&self) -> Option<(f64, f64, f64, f64)> {
        let mut xmin = f64::INFINITY;
        let mut xmax = f64::NEG_INFINITY;
        let mut ymin = f64::INFINITY;
        let mut ymax = f64::NEG_INFINITY;
        let mut any = false;
        for poly in &self.polygons {
            for p in &poly.points {
                any = true;
                xmin = xmin.min(p[0]);
                xmax = xmax.max(p[0]);
                ymin = ymin.min(p[1]);
                ymax = ymax.max(p[1]);
            }
        }
        if any { Some((xmin, xmax, ymin, ymax)) } else { None }
    }
}
