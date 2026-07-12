#![allow(missing_docs)]

use bijux_gnss_core::api::{SatId, SigId, SignalBand};

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct BroadcastIonosphereResidualObservation {
    pub epoch_idx: u64,
    pub t_rx_s: f64,
    pub sat: SatId,
    pub signal_1: Option<SigId>,
    pub signal_2: Option<SigId>,
    pub band_1: SignalBand,
    pub band_2: SignalBand,
    pub broadcast_model: String,
    pub azimuth_deg: Option<f64>,
    pub elevation_deg: Option<f64>,
    pub measured_code_delay_band_1_m: Option<f64>,
    pub measured_code_delay_band_2_m: Option<f64>,
    pub measured_phase_delay_band_1_m: Option<f64>,
    pub measured_phase_delay_band_2_m: Option<f64>,
    pub broadcast_delay_band_1_m: Option<f64>,
    pub broadcast_delay_band_2_m: Option<f64>,
    pub code_residual_band_1_m: Option<f64>,
    pub code_residual_band_2_m: Option<f64>,
    pub phase_residual_band_1_m: Option<f64>,
    pub phase_residual_band_2_m: Option<f64>,
    pub measured_code_status: String,
    pub measured_code_reason: String,
    pub measured_phase_status: String,
    pub measured_phase_reason: String,
    pub broadcast_status: String,
    pub broadcast_reason: String,
}

#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct BroadcastIonosphereResidualStats {
    pub sample_count: usize,
    pub mean_residual_m: f64,
    pub mean_abs_residual_m: f64,
    pub rms_residual_m: f64,
    pub max_abs_residual_m: f64,
}

#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct BroadcastIonosphereResidualSummary {
    pub observation_count: usize,
    pub broadcast_valid_count: usize,
    pub code_band_1: Option<BroadcastIonosphereResidualStats>,
    pub code_band_2: Option<BroadcastIonosphereResidualStats>,
    pub phase_band_1: Option<BroadcastIonosphereResidualStats>,
    pub phase_band_2: Option<BroadcastIonosphereResidualStats>,
}

pub fn summarize_broadcast_ionosphere_residuals(
    observations: &[BroadcastIonosphereResidualObservation],
) -> BroadcastIonosphereResidualSummary {
    BroadcastIonosphereResidualSummary {
        observation_count: observations.len(),
        broadcast_valid_count: observations
            .iter()
            .filter(|observation| observation.broadcast_status == "ok")
            .count(),
        code_band_1: summarize_residual_component(
            observations.iter().filter_map(|observation| observation.code_residual_band_1_m),
        ),
        code_band_2: summarize_residual_component(
            observations.iter().filter_map(|observation| observation.code_residual_band_2_m),
        ),
        phase_band_1: summarize_residual_component(
            observations.iter().filter_map(|observation| observation.phase_residual_band_1_m),
        ),
        phase_band_2: summarize_residual_component(
            observations.iter().filter_map(|observation| observation.phase_residual_band_2_m),
        ),
    }
}

fn summarize_residual_component(
    values: impl Iterator<Item = f64>,
) -> Option<BroadcastIonosphereResidualStats> {
    let values = values.filter(|value| value.is_finite()).collect::<Vec<_>>();
    if values.is_empty() {
        return None;
    }

    let sample_count = values.len();
    let sum = values.iter().sum::<f64>();
    let mean_residual_m = sum / sample_count as f64;
    let mean_abs_residual_m = values.iter().map(|value| value.abs()).sum::<f64>() / sample_count as f64;
    let rms_residual_m =
        (values.iter().map(|value| value * value).sum::<f64>() / sample_count as f64).sqrt();
    let max_abs_residual_m =
        values.iter().map(|value| value.abs()).fold(0.0_f64, f64::max);

    Some(BroadcastIonosphereResidualStats {
        sample_count,
        mean_residual_m,
        mean_abs_residual_m,
        rms_residual_m,
        max_abs_residual_m,
    })
}

#[cfg(test)]
mod tests {
    use super::{
        summarize_broadcast_ionosphere_residuals, BroadcastIonosphereResidualObservation,
        BroadcastIonosphereResidualStats,
    };
    use bijux_gnss_core::api::{Constellation, SatId, SignalBand};

    fn sample_observation(
        code_band_1: Option<f64>,
        code_band_2: Option<f64>,
        phase_band_1: Option<f64>,
        phase_band_2: Option<f64>,
        broadcast_status: &str,
    ) -> BroadcastIonosphereResidualObservation {
        BroadcastIonosphereResidualObservation {
            epoch_idx: 0,
            t_rx_s: 0.0,
            sat: SatId { constellation: Constellation::Gps, prn: 11 },
            signal_1: None,
            signal_2: None,
            band_1: SignalBand::L1,
            band_2: SignalBand::L2,
            broadcast_model: "klobuchar".to_string(),
            azimuth_deg: Some(120.0),
            elevation_deg: Some(35.0),
            measured_code_delay_band_1_m: None,
            measured_code_delay_band_2_m: None,
            measured_phase_delay_band_1_m: None,
            measured_phase_delay_band_2_m: None,
            broadcast_delay_band_1_m: None,
            broadcast_delay_band_2_m: None,
            code_residual_band_1_m: code_band_1,
            code_residual_band_2_m: code_band_2,
            phase_residual_band_1_m: phase_band_1,
            phase_residual_band_2_m: phase_band_2,
            measured_code_status: "ok".to_string(),
            measured_code_reason: "ok".to_string(),
            measured_phase_status: "ok".to_string(),
            measured_phase_reason: "ok".to_string(),
            broadcast_status: broadcast_status.to_string(),
            broadcast_reason: broadcast_status.to_string(),
        }
    }

    fn assert_stats(
        stats: BroadcastIonosphereResidualStats,
        sample_count: usize,
        mean_residual_m: f64,
        mean_abs_residual_m: f64,
        rms_residual_m: f64,
        max_abs_residual_m: f64,
    ) {
        assert_eq!(stats.sample_count, sample_count);
        assert!((stats.mean_residual_m - mean_residual_m).abs() < 1.0e-12);
        assert!((stats.mean_abs_residual_m - mean_abs_residual_m).abs() < 1.0e-12);
        assert!((stats.rms_residual_m - rms_residual_m).abs() < 1.0e-12);
        assert!((stats.max_abs_residual_m - max_abs_residual_m).abs() < 1.0e-12);
    }

    #[test]
    fn broadcast_ionosphere_residual_summary_aggregates_each_component() {
        let summary = summarize_broadcast_ionosphere_residuals(&[
            sample_observation(Some(1.0), Some(-2.0), Some(0.5), None, "ok"),
            sample_observation(Some(-3.0), Some(4.0), None, Some(-1.5), "ok"),
            sample_observation(None, None, Some(1.5), Some(2.5), "invalid"),
        ]);

        assert_eq!(summary.observation_count, 3);
        assert_eq!(summary.broadcast_valid_count, 2);
        assert_stats(
            summary.code_band_1.expect("code L1 stats"),
            2,
            -1.0,
            2.0,
            (5.0_f64).sqrt(),
            3.0,
        );
        assert_stats(
            summary.code_band_2.expect("code L2 stats"),
            2,
            1.0,
            3.0,
            (10.0_f64).sqrt(),
            4.0,
        );
        assert_stats(
            summary.phase_band_1.expect("phase L1 stats"),
            2,
            1.0,
            1.0,
            (1.25_f64).sqrt(),
            1.5,
        );
        assert_stats(
            summary.phase_band_2.expect("phase L2 stats"),
            2,
            0.5,
            2.0,
            (4.25_f64).sqrt(),
            2.5,
        );
    }

    #[test]
    fn broadcast_ionosphere_residual_summary_ignores_missing_components() {
        let summary = summarize_broadcast_ionosphere_residuals(&[sample_observation(
            None,
            None,
            None,
            None,
            "invalid",
        )]);

        assert_eq!(summary.observation_count, 1);
        assert_eq!(summary.broadcast_valid_count, 0);
        assert!(summary.code_band_1.is_none());
        assert!(summary.code_band_2.is_none());
        assert!(summary.phase_band_1.is_none());
        assert!(summary.phase_band_2.is_none());
    }
}
