/// Dot product of two equal-length slices.
pub fn dot_f64(a: &[f64], b: &[f64]) -> f64 {
    a.iter().zip(b.iter()).map(|(x, y)| x * y).sum()
}

/// Normalize i16 samples to f64 in [-1.0, 1.0].
pub fn normalize_i16(samples: &[i16]) -> Vec<f64> {
    let scale = i16::MAX as f64;
    samples.iter().map(|s| *s as f64 / scale).collect()
}
