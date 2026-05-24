#[derive(Clone, Copy, Debug)]
pub struct TrueWind {
    pub x: f64,
    pub y: f64,
    pub angle: f64,
    pub speed: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct ApparentWind {
    pub x: f64,
    pub y: f64,
    pub angle: f64,
    pub speed: f64,
}
