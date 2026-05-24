use std::f64::consts::PI;

use crate::physics::util::sign;

const LIMIT_WIND_SPEED: f64 = 6.0;
const STALL_DEG: f64 = 14.0;

/// Optimal sail trim angle from apparent wind. Port of
/// `sail_angle.py:3`. Returns the absolute sail angle — sign is
/// applied inside `solve()` per CLAUDE.md's "True Sail Angle Sign
/// Convention".
pub fn sail_angle(wind_angle: f64, wind_speed: f64, sail_stretching: f64) -> f64 {
    let mut opt_aoa = wind_angle.sin()
        / (wind_angle.cos() + 0.4 * wind_angle.cos().powi(2))
        * sail_stretching
        / 4.0;
    let stall_rad = STALL_DEG.to_radians();
    if opt_aoa.abs() > stall_rad {
        opt_aoa = sign(wind_angle) * stall_rad;
    }
    if wind_speed > LIMIT_WIND_SPEED {
        opt_aoa *= (LIMIT_WIND_SPEED / wind_speed).powi(2);
    }
    (wind_angle - opt_aoa).clamp(-PI / 2.0, PI / 2.0).abs()
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde::Deserialize;
    use std::path::PathBuf;

    #[derive(Deserialize)]
    struct CasesFile {
        cases: Vec<Case>,
    }

    #[derive(Deserialize)]
    struct Case {
        wind_angle: f64,
        wind_speed: f64,
        sail_stretching: f64,
        expected: f64,
    }

    #[test]
    fn matches_python_sail_angle_table() {
        let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/sail_angles.json");
        let raw = std::fs::read_to_string(&path).expect("run scripts/dump_fixtures.py first");
        let file: CasesFile = serde_json::from_str(&raw).unwrap();
        assert!(!file.cases.is_empty());
        for c in &file.cases {
            let got = sail_angle(c.wind_angle, c.wind_speed, c.sail_stretching);
            let diff = (got - c.expected).abs();
            assert!(
                diff < 1e-12,
                "wa={} ws={} stretch={} diff {} (rust {} vs py {})",
                c.wind_angle,
                c.wind_speed,
                c.sail_stretching,
                diff,
                got,
                c.expected
            );
        }
    }
}
