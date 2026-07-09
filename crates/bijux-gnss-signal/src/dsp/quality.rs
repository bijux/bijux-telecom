//! Signal quality metrics computed directly from complex I/Q samples.

use bijux_gnss_core::api::Sample;
use serde::{Deserialize, Serialize};

/// Front-end metrics measured from a complex I/Q sample window.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct IqFrontEndMetrics {
    /// Number of complex samples used for the measurement.
    pub sample_count: usize,
    /// Mean of the in-phase component.
    pub i_mean: f64,
    /// Mean of the quadrature component.
    pub q_mean: f64,
    /// Mean power of the in-phase component.
    pub i_power: f64,
    /// Mean power of the quadrature component.
    pub q_power: f64,
    /// I/Q branch power ratio as I power divided by Q power.
    pub iq_power_ratio: f64,
    /// Whether the branch powers differ enough to warrant an operator warning.
    pub power_imbalance_warning: bool,
    /// Estimated deviation from ideal 90 degree I/Q phase separation in degrees.
    ///
    /// This is `None` when the sample window does not provide enough centered I and Q variance
    /// to support a meaningful estimate.
    pub quadrature_error_deg: Option<f64>,
    /// Whether the estimated quadrature error exceeds the operator warning limit.
    pub quadrature_error_warning: bool,
    /// Complex RMS magnitude computed from I and Q power.
    pub rms: f64,
    /// Normalized DC vector magnitude relative to the complex RMS magnitude.
    pub dc_imbalance: f64,
}

const POWER_RATIO_WARNING_LIMIT: f64 = 1.5;
const POWER_RATIO_EPSILON: f64 = 1e-12;
const QUADRATURE_WARNING_LIMIT_DEG: f64 = 5.0;
const QUADRATURE_VARIANCE_EPSILON: f64 = 1e-12;

/// Incremental analyzer for front-end metrics over one or more I/Q frames.
#[derive(Debug, Clone, Default)]
pub struct IqFrontEndAnalyzer {
    sample_count: usize,
    sum_i: f64,
    sum_q: f64,
    sum_i_power: f64,
    sum_q_power: f64,
    sum_iq: f64,
    sum_power: f64,
}

impl IqFrontEndAnalyzer {
    /// Create a new analyzer.
    pub fn new() -> Self {
        Self::default()
    }

    /// Accumulate metrics from a complex I/Q slice.
    pub fn update(&mut self, samples: &[Sample]) {
        self.sample_count += samples.len();
        for sample in samples {
            let i = sample.re as f64;
            let q = sample.im as f64;
            self.sum_i += i;
            self.sum_q += q;
            self.sum_i_power += i * i;
            self.sum_q_power += q * q;
            self.sum_iq += i * q;
            self.sum_power += i * i + q * q;
        }
    }

    /// Finalize the current metrics snapshot.
    pub fn finish(&self) -> IqFrontEndMetrics {
        let count = self.sample_count as f64;
        let i_mean = if self.sample_count == 0 { 0.0 } else { self.sum_i / count };
        let q_mean = if self.sample_count == 0 { 0.0 } else { self.sum_q / count };
        let i_power = if self.sample_count == 0 {
            0.0
        } else {
            self.sum_i_power / count
        };
        let q_power = if self.sample_count == 0 {
            0.0
        } else {
            self.sum_q_power / count
        };
        let iq_power_ratio = iq_power_ratio(i_power, q_power);
        let power_imbalance_warning = power_imbalance_warning(i_power, q_power);
        let quadrature_error_deg =
            quadrature_error_deg(self.sample_count, i_mean, q_mean, i_power, q_power, self.sum_iq);
        let quadrature_error_warning = quadrature_error_deg
            .map(|error_deg| error_deg.abs() > QUADRATURE_WARNING_LIMIT_DEG)
            .unwrap_or(false);
        let rms = if self.sample_count == 0 {
            0.0
        } else {
            (self.sum_power / count).sqrt()
        };
        let dc_magnitude = (i_mean * i_mean + q_mean * q_mean).sqrt();
        let dc_imbalance = if rms > 0.0 { dc_magnitude / rms } else { 0.0 };
        IqFrontEndMetrics {
            sample_count: self.sample_count,
            i_mean,
            q_mean,
            i_power,
            q_power,
            iq_power_ratio,
            power_imbalance_warning,
            quadrature_error_deg,
            quadrature_error_warning,
            rms,
            dc_imbalance,
        }
    }
}

fn iq_power_ratio(i_power: f64, q_power: f64) -> f64 {
    if i_power <= POWER_RATIO_EPSILON && q_power <= POWER_RATIO_EPSILON {
        1.0
    } else {
        i_power / q_power.max(POWER_RATIO_EPSILON)
    }
}

fn power_imbalance_warning(i_power: f64, q_power: f64) -> bool {
    if i_power <= POWER_RATIO_EPSILON && q_power <= POWER_RATIO_EPSILON {
        return false;
    }
    let stronger = i_power.max(q_power);
    let weaker = i_power.min(q_power);
    weaker <= POWER_RATIO_EPSILON || stronger / weaker > POWER_RATIO_WARNING_LIMIT
}

fn quadrature_error_deg(
    sample_count: usize,
    i_mean: f64,
    q_mean: f64,
    i_power: f64,
    q_power: f64,
    sum_iq: f64,
) -> Option<f64> {
    if sample_count == 0 {
        return None;
    }
    let count = sample_count as f64;
    let iq_mean = sum_iq / count;
    let i_variance = (i_power - i_mean * i_mean).max(0.0);
    let q_variance = (q_power - q_mean * q_mean).max(0.0);
    if i_variance <= QUADRATURE_VARIANCE_EPSILON || q_variance <= QUADRATURE_VARIANCE_EPSILON {
        return None;
    }
    let covariance = iq_mean - i_mean * q_mean;
    let normalized = (covariance / (i_variance.sqrt() * q_variance.sqrt())).clamp(-1.0, 1.0);
    Some(normalized.asin().to_degrees())
}

/// Measure front-end metrics over a single I/Q slice.
pub fn measure_iq_front_end_metrics(samples: &[Sample]) -> IqFrontEndMetrics {
    let mut analyzer = IqFrontEndAnalyzer::new();
    analyzer.update(samples);
    analyzer.finish()
}

/// Remove the mean I/Q offset from a sample window in place.
///
/// Returns the measured front-end metrics before the correction is applied.
pub fn remove_dc_offset_in_place(samples: &mut [Sample]) -> IqFrontEndMetrics {
    let metrics = measure_iq_front_end_metrics(samples);
    let i_mean = metrics.i_mean as f32;
    let q_mean = metrics.q_mean as f32;
    for sample in samples {
        sample.re -= i_mean;
        sample.im -= q_mean;
    }
    metrics
}

#[cfg(test)]
mod tests {
    use super::{measure_iq_front_end_metrics, remove_dc_offset_in_place, IqFrontEndAnalyzer};
    use bijux_gnss_core::api::Sample;
    use std::f32::consts::TAU;

    const QUADRATURE_FIXTURE_TOLERANCE_DEG: f64 = 0.25;

    fn approx_eq(left: f64, right: f64) {
        assert!((left - right).abs() < 1e-9, "left={left} right={right}");
    }

    fn phase_skewed_carrier_samples(sample_count: usize, phase_error_deg: f32) -> Vec<Sample> {
        let phase_error_rad = phase_error_deg.to_radians();
        (0..sample_count)
            .map(|idx| {
                let phase = TAU * idx as f32 / sample_count as f32;
                Sample::new(phase.cos(), (phase + phase_error_rad).sin())
            })
            .collect()
    }

    #[test]
    fn measure_iq_front_end_metrics_reports_power_rms_and_dc_imbalance() {
        let samples = vec![
            Sample::new(0.5, 0.0),
            Sample::new(0.5, 0.0),
            Sample::new(-0.5, 0.0),
            Sample::new(0.5, 0.5),
        ];

        let metrics = measure_iq_front_end_metrics(&samples);

        assert_eq!(metrics.sample_count, 4);
        approx_eq(metrics.i_mean, 0.25);
        approx_eq(metrics.q_mean, 0.125);
        approx_eq(metrics.i_power, 0.25);
        approx_eq(metrics.q_power, 0.0625);
        approx_eq(metrics.iq_power_ratio, 4.0);
        assert!(metrics.power_imbalance_warning);
        assert!(metrics.quadrature_error_deg.is_some());
        assert!(metrics.quadrature_error_warning);
        approx_eq(metrics.rms, 0.5590169943749475);
        approx_eq(metrics.dc_imbalance, 0.5);
    }

    #[test]
    fn front_end_analyzer_combines_multiple_frames() {
        let mut analyzer = IqFrontEndAnalyzer::new();
        analyzer.update(&[Sample::new(0.5, 0.25), Sample::new(0.5, -0.25)]);
        analyzer.update(&[Sample::new(0.5, 0.25), Sample::new(0.5, -0.25)]);

        let metrics = analyzer.finish();

        assert_eq!(metrics.sample_count, 4);
        approx_eq(metrics.i_mean, 0.5);
        approx_eq(metrics.q_mean, 0.0);
        approx_eq(metrics.i_power, 0.25);
        approx_eq(metrics.q_power, 0.0625);
        approx_eq(metrics.iq_power_ratio, 4.0);
        assert!(metrics.power_imbalance_warning);
        assert_eq!(metrics.quadrature_error_deg, None);
        assert!(!metrics.quadrature_error_warning);
        approx_eq(metrics.rms, 0.5590169943749475);
        approx_eq(metrics.dc_imbalance, 0.8944271909999159);
    }

    #[test]
    fn remove_dc_offset_in_place_zero_centers_samples() {
        let mut samples = vec![
            Sample::new(0.75, 0.25),
            Sample::new(0.75, -0.25),
            Sample::new(0.25, 0.25),
            Sample::new(0.25, -0.25),
        ];

        let before = remove_dc_offset_in_place(&mut samples);
        let after = measure_iq_front_end_metrics(&samples);

        approx_eq(before.i_mean, 0.5);
        approx_eq(before.q_mean, 0.0);
        approx_eq(before.i_power, 0.3125);
        approx_eq(before.q_power, 0.0625);
        approx_eq(before.iq_power_ratio, 5.0);
        assert!(before.power_imbalance_warning);
        assert_eq!(before.quadrature_error_deg, Some(0.0));
        assert!(!before.quadrature_error_warning);
        approx_eq(after.i_mean, 0.0);
        approx_eq(after.q_mean, 0.0);
        approx_eq(after.i_power, 0.0625);
        approx_eq(after.q_power, 0.0625);
        approx_eq(after.iq_power_ratio, 1.0);
        assert!(!after.power_imbalance_warning);
        assert_eq!(after.quadrature_error_deg, Some(0.0));
        assert!(!after.quadrature_error_warning);
        approx_eq(after.rms, 0.3535533905932738);
        approx_eq(after.dc_imbalance, 0.0);
    }

    #[test]
    fn balanced_branch_powers_do_not_trigger_warning() {
        let samples = vec![
            Sample::new(0.5, 0.5),
            Sample::new(-0.5, -0.5),
            Sample::new(0.25, -0.25),
            Sample::new(-0.25, 0.25),
        ];

        let metrics = measure_iq_front_end_metrics(&samples);

        approx_eq(metrics.i_power, metrics.q_power);
        approx_eq(metrics.iq_power_ratio, 1.0);
        assert!(!metrics.power_imbalance_warning);
        assert!(metrics.quadrature_error_deg.is_some());
    }

    #[test]
    fn synthetic_phase_skew_fixture_detects_quadrature_error_within_tolerance() {
        let expected_error_deg = 12.5_f64;
        let samples = phase_skewed_carrier_samples(4096, expected_error_deg as f32);

        let metrics = measure_iq_front_end_metrics(&samples);
        let measured_error_deg =
            metrics.quadrature_error_deg.expect("quadrature error should be measurable");

        assert!(
            (measured_error_deg - expected_error_deg).abs() <= QUADRATURE_FIXTURE_TOLERANCE_DEG,
            "measured={measured_error_deg} expected={expected_error_deg}"
        );
        assert!(metrics.quadrature_error_warning);
    }
}
