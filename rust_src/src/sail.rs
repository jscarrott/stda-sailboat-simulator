//! Sail-trim optimiser.
//!
//! The implementation now lives in the shared `boat-control` crate (generic over
//! the float type, `no_std`, so the same code runs on the nRF52840 firmware).
//! This re-export pins it to `f64` for the simulator; the bit-exact Python-trace
//! test below exercises that path.

/// Optimal sail trim angle from apparent wind. See
/// [`boat_control::sail_angle`]. Returns the absolute sail angle — the sign is
/// applied inside `solve()` per CLAUDE.md's "True Sail Angle Sign Convention".
pub use boat_control::sail_angle;

#[cfg(test)]
const MIN_WIND_SPEED_FOR_TRIM: f64 = 0.2;

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
