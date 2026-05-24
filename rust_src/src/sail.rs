use std::f64::consts::PI;

use crate::physics::util::sign;

const LIMIT_WIND_SPEED: f64 = 6.0;
const STALL_DEG: f64 = 14.0;

/// Below this apparent wind speed the `atan2` of the (vx, vy)
/// components is numerically unreliable: small noise in either axis
/// flips `wind_angle` between +π and −π, so `sail_angle()` would
/// slam from one extreme to the other across consecutive ticks.
/// Holding the previous trim keeps the sail still until there's
/// enough flow to optimise against.
const MIN_WIND_SPEED_FOR_TRIM: f64 = 0.2;

/// Optimal sail trim angle from apparent wind. Port of
/// `sail_angle.py:3`, plus a low-apparent-wind guard not present in
/// the Python original. Returns the absolute sail angle — sign is
/// applied inside `solve()` per CLAUDE.md's "True Sail Angle Sign
/// Convention".
///
/// `previous_sail` is the angle commanded on the last tick; it is
/// returned unchanged when the apparent wind is below the
/// numerical-reliability threshold (e.g. boat momentarily matched to
/// the true wind, or starting from rest with wind dead astern).
pub fn sail_angle(
    wind_angle: f64,
    wind_speed: f64,
    sail_stretching: f64,
    previous_sail: f64,
) -> f64 {
    if wind_speed < MIN_WIND_SPEED_FOR_TRIM {
        return previous_sail;
    }
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
            // previous_sail placeholder: fixture cases all have
            // wind_speed >= 1.0 (well above MIN_WIND_SPEED_FOR_TRIM),
            // so the guard never fires and this value is unused.
            let got = sail_angle(c.wind_angle, c.wind_speed, c.sail_stretching, 0.0);
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

    #[test]
    fn low_apparent_wind_holds_previous_trim() {
        // Apparent wind below the guard threshold: any sail angle the
        // formula would compute is unreliable, so we expect the
        // function to return whatever was passed in as previous_sail.
        let prev = 0.42;
        let got = sail_angle(0.0, MIN_WIND_SPEED_FOR_TRIM - 0.05, 0.961, prev);
        assert_eq!(got, prev, "guard should hold previous trim at sub-threshold wind");

        // Even with a non-trivial wind_angle the guard should still hold.
        let got2 = sail_angle(2.5, 0.0, 0.961, prev);
        assert_eq!(got2, prev);

        // Above threshold the formula must run as before — sanity check
        // that the guard doesn't leak above its threshold.
        let got3 = sail_angle(0.0, MIN_WIND_SPEED_FOR_TRIM + 0.05, 0.961, prev);
        assert_ne!(got3, prev);
    }
}
