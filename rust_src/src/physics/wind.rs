#[derive(Clone, Copy, Debug)]
pub struct TrueWind {
    pub x: f64,
    pub y: f64,
    pub strength: f64,
    pub direction: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct ApparentWind {
    pub x: f64,
    pub y: f64,
    pub angle: f64,
    pub speed: f64,
}

/// Port of `calculate_apparent_wind` from `simulation.py:159`.
pub fn calculate_apparent_wind(yaw: f64, vel_x: f64, vel_y: f64, true_wind: TrueWind) -> ApparentWind {
    let transformed_x = true_wind.x * yaw.cos() + true_wind.y * yaw.sin();
    let transformed_y = true_wind.x * -yaw.sin() + true_wind.y * yaw.cos();
    let apparent_x = transformed_x - vel_x;
    let apparent_y = transformed_y - vel_y;
    ApparentWind {
        x: apparent_x,
        y: apparent_y,
        angle: (-apparent_y).atan2(-apparent_x),
        speed: (apparent_x * apparent_x + apparent_y * apparent_y).sqrt(),
    }
}
