#![allow(missing_docs)]

#[derive(Debug, Clone, Copy)]
pub struct InnovationConsistencyConfig {
    pub lower_tail_probability: f64,
    pub upper_tail_probability: f64,
}

impl Default for InnovationConsistencyConfig {
    fn default() -> Self {
        Self { lower_tail_probability: 0.01, upper_tail_probability: 0.99 }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct InnovationConsistencyBounds {
    pub lower_bound: f64,
    pub upper_bound: f64,
}

pub fn innovation_consistency_bounds(
    measurement_dimension: usize,
    config: InnovationConsistencyConfig,
) -> Option<InnovationConsistencyBounds> {
    if measurement_dimension == 0 {
        return None;
    }
    let lower_tail_probability = validate_probability(config.lower_tail_probability)?;
    let upper_tail_probability = validate_probability(config.upper_tail_probability)?;
    if lower_tail_probability >= upper_tail_probability {
        return None;
    }
    let dof = measurement_dimension as f64;
    Some(InnovationConsistencyBounds {
        lower_bound: chi_square_quantile_wilson_hilferty(dof, lower_tail_probability),
        upper_bound: chi_square_quantile_wilson_hilferty(dof, upper_tail_probability),
    })
}

fn validate_probability(probability: f64) -> Option<f64> {
    if probability.is_finite() && probability > 0.0 && probability < 1.0 {
        Some(probability)
    } else {
        None
    }
}

fn chi_square_quantile_wilson_hilferty(degrees_of_freedom: f64, probability: f64) -> f64 {
    if degrees_of_freedom <= 0.0 {
        return 0.0;
    }

    // Wilson-Hilferty gives a stable chi-square quantile approximation without adding a stats crate.
    let z = standard_normal_inverse_cdf(probability);
    let scale = 2.0 / (9.0 * degrees_of_freedom);
    let transformed = 1.0 - scale + z * scale.sqrt();
    if transformed <= 0.0 {
        0.0
    } else {
        degrees_of_freedom * transformed.powi(3)
    }
}

fn standard_normal_inverse_cdf(probability: f64) -> f64 {
    const A: [f64; 6] = [
        -3.969_683_028_665_376e1,
        2.209_460_984_245_205e2,
        -2.759_285_104_469_687e2,
        1.383_577_518_672_69e2,
        -3.066_479_806_614_716e1,
        2.506_628_277_459_239,
    ];
    const B: [f64; 5] = [
        -5.447_609_879_822_406e1,
        1.615_858_368_580_409e2,
        -1.556_989_798_598_866e2,
        6.680_131_188_771_972e1,
        -1.328_068_155_288_572e1,
    ];
    const C: [f64; 6] = [
        -7.784_894_002_430_293e-3,
        -3.223_964_580_411_365e-1,
        -2.400_758_277_161_838,
        -2.549_732_539_343_734,
        4.374_664_141_464_968,
        2.938_163_982_698_783,
    ];
    const D: [f64; 4] = [
        7.784_695_709_041_462e-3,
        3.224_671_290_700_398e-1,
        2.445_134_137_142_996,
        3.754_408_661_907_416,
    ];
    const LOW_REGION: f64 = 0.02425;
    const HIGH_REGION: f64 = 1.0 - LOW_REGION;

    if probability <= 0.0 {
        return f64::NEG_INFINITY;
    }
    if probability >= 1.0 {
        return f64::INFINITY;
    }

    if probability < LOW_REGION {
        let q = (-2.0 * probability.ln()).sqrt();
        return (((((C[0] * q + C[1]) * q + C[2]) * q + C[3]) * q + C[4]) * q + C[5])
            / ((((D[0] * q + D[1]) * q + D[2]) * q + D[3]) * q + 1.0);
    }

    if probability > HIGH_REGION {
        let q = (-2.0 * (1.0 - probability).ln()).sqrt();
        return -(((((C[0] * q + C[1]) * q + C[2]) * q + C[3]) * q + C[4]) * q + C[5])
            / ((((D[0] * q + D[1]) * q + D[2]) * q + D[3]) * q + 1.0);
    }

    let q = probability - 0.5;
    let r = q * q;
    (((((A[0] * r + A[1]) * r + A[2]) * r + A[3]) * r + A[4]) * r + A[5]) * q
        / (((((B[0] * r + B[1]) * r + B[2]) * r + B[3]) * r + B[4]) * r + 1.0)
}

#[cfg(test)]
mod tests {
    use super::{innovation_consistency_bounds, InnovationConsistencyConfig};

    #[test]
    fn innovation_consistency_bounds_are_ordered_for_scalar_measurements() {
        let bounds = innovation_consistency_bounds(1, InnovationConsistencyConfig::default())
            .expect("default scalar innovation probabilities must define finite bounds");

        assert!(bounds.lower_bound >= 0.0);
        assert!(bounds.lower_bound < bounds.upper_bound);
        assert!((bounds.lower_bound - 0.000_157).abs() < 0.001);
        assert!((bounds.upper_bound - 6.635).abs() < 0.2);
    }

    #[test]
    fn innovation_consistency_bounds_reject_invalid_probability_ranges() {
        assert!(innovation_consistency_bounds(
            1,
            InnovationConsistencyConfig {
                lower_tail_probability: 0.9,
                upper_tail_probability: 0.1,
            },
        )
        .is_none());
        assert!(innovation_consistency_bounds(
            1,
            InnovationConsistencyConfig {
                lower_tail_probability: 0.0,
                upper_tail_probability: 0.99,
            },
        )
        .is_none());
    }
}
