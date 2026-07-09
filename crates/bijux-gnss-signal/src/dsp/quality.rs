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
    /// Complex RMS magnitude computed from I and Q power.
    pub rms: f64,
    /// Normalized DC vector magnitude relative to the complex RMS magnitude.
    pub dc_imbalance: f64,
}

/// Incremental analyzer for front-end metrics over one or more I/Q frames.
#[derive(Debug, Clone, Default)]
pub struct IqFrontEndAnalyzer {
    sample_count: usize,
    sum_i: f64,
    sum_q: f64,
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
            self.sum_power += i * i + q * q;
        }
    }

    /// Finalize the current metrics snapshot.
    pub fn finish(&self) -> IqFrontEndMetrics {
        let count = self.sample_count as f64;
        let i_mean = if self.sample_count == 0 { 0.0 } else { self.sum_i / count };
        let q_mean = if self.sample_count == 0 { 0.0 } else { self.sum_q / count };
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
            rms,
            dc_imbalance,
        }
    }
}

/// Measure front-end metrics over a single I/Q slice.
pub fn measure_iq_front_end_metrics(samples: &[Sample]) -> IqFrontEndMetrics {
    let mut analyzer = IqFrontEndAnalyzer::new();
    analyzer.update(samples);
    analyzer.finish()
}

#[cfg(test)]
mod tests {
    use super::{measure_iq_front_end_metrics, IqFrontEndAnalyzer};
    use bijux_gnss_core::api::Sample;

    fn approx_eq(left: f64, right: f64) {
        assert!((left - right).abs() < 1e-9, "left={left} right={right}");
    }

    #[test]
    fn measure_iq_front_end_metrics_reports_mean_rms_and_dc_imbalance() {
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
        approx_eq(metrics.rms, 0.5590169943749475);
        approx_eq(metrics.dc_imbalance, 0.8944271909999159);
    }
}
