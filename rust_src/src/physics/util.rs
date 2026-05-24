/// Sign function matching Python's `copysign(1, v) if v != 0 else 0`.
/// Differs from `f64::signum`, which returns ±1 even for ±0.0.
#[inline]
pub fn sign(v: f64) -> f64 {
    if v == 0.0 {
        0.0
    } else if v > 0.0 {
        1.0
    } else {
        -1.0
    }
}
