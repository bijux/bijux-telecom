#![allow(missing_docs)]

use std::collections::HashMap;

use bijux_gnss_core::api::{ObsEpoch, ObservationStatus, SigId};
use bijux_gnss_signal::api::signal_cycles_to_meters;
use serde::{Deserialize, Serialize};

use crate::pipeline::observations::{
    ObservationPipelineArtifacts, ObservationResidualEpochReport, ObservationResidualSatellite,
};

const RAW_CORRECTED_MATCH_TOLERANCE_M: f64 = 1.0e-6;
const STEADY_STATE_SMOOTHING_AGE: u32 = 5;

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct CarrierSmoothedCodeValidationReport {
    pub observations: usize,
    pub accepted_observations: usize,
    pub smoothed_observations: usize,
    pub reference_observations: usize,
    pub improvement_observations: usize,
    pub raw_mean_abs_residual_m: Option<f64>,
    pub smoothed_mean_abs_residual_m: Option<f64>,
    pub improvement_delta_m: Option<f64>,
    pub improvement_verified: Option<bool>,
    pub cycle_slip_observations: usize,
    pub slip_reset_observations: usize,
    pub slip_preserved_observations: usize,
    pub hidden_slip_observations: usize,
    pub slip_visibility_verified: Option<bool>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
struct ResidualKey {
    epoch_idx: u64,
    signal_id: SigId,
}

pub fn summarize_carrier_smoothed_code(
    observations: &[ObsEpoch],
) -> CarrierSmoothedCodeValidationReport {
    let mut report = CarrierSmoothedCodeValidationReport {
        observations: 0,
        accepted_observations: 0,
        smoothed_observations: 0,
        reference_observations: 0,
        improvement_observations: 0,
        raw_mean_abs_residual_m: None,
        smoothed_mean_abs_residual_m: None,
        improvement_delta_m: None,
        improvement_verified: None,
        cycle_slip_observations: 0,
        slip_reset_observations: 0,
        slip_preserved_observations: 0,
        hidden_slip_observations: 0,
        slip_visibility_verified: None,
    };

    for epoch in observations {
        for sat in &epoch.sats {
            report.observations += 1;
            if sat.observation_status == ObservationStatus::Accepted {
                report.accepted_observations += 1;
            }
            if sat.metadata.smoothing_age > 0 {
                report.smoothed_observations += 1;
            }
            if sat.lock_flags.cycle_slip {
                report.cycle_slip_observations += 1;
                if sat.metadata.smoothing_age == 1 {
                    report.slip_reset_observations += 1;
                } else {
                    report.hidden_slip_observations += 1;
                }
            }
        }
    }

    report.slip_visibility_verified =
        (report.cycle_slip_observations > 0).then_some(report.hidden_slip_observations == 0);
    report
}

pub fn validate_carrier_smoothed_code(
    observations: &[ObsEpoch],
    residuals: &[ObservationResidualEpochReport],
) -> CarrierSmoothedCodeValidationReport {
    let mut report = summarize_carrier_smoothed_code(observations);
    let residuals_by_signal = residuals_by_signal(residuals);
    let mut carrier_divergence_reference_by_signal = HashMap::new();
    let mut raw_abs_errors_m = Vec::new();
    let mut smoothed_abs_errors_m = Vec::new();

    for epoch in observations {
        for sat in &epoch.sats {
            let Some(residual) = residuals_by_signal
                .get(&ResidualKey { epoch_idx: epoch.epoch_idx, signal_id: sat.signal_id })
            else {
                continue;
            };

            if sat.lock_flags.cycle_slip
                && sat.metadata.smoothing_age == 1
                && residuals_preserve_raw_code(residual)
            {
                report.slip_preserved_observations += 1;
            }

            if sat.observation_status != ObservationStatus::Accepted {
                continue;
            }

            let carrier_reference_m =
                signal_cycles_to_meters(sat.carrier_phase_cycles, sat.metadata.signal).0;
            let raw_divergence_m = residual.pseudorange_m.raw - carrier_reference_m;

            if sat.metadata.smoothing_age == 1 {
                carrier_divergence_reference_by_signal.insert(sat.signal_id, raw_divergence_m);
                report.reference_observations += 1;
                continue;
            }

            let Some(reference_divergence_m) =
                carrier_divergence_reference_by_signal.get(&sat.signal_id).copied()
            else {
                carrier_divergence_reference_by_signal.insert(sat.signal_id, raw_divergence_m);
                report.reference_observations += 1;
                continue;
            };

            report.reference_observations += 1;

            if sat.lock_flags.cycle_slip || sat.metadata.smoothing_age < STEADY_STATE_SMOOTHING_AGE
            {
                continue;
            }

            let expected_pseudorange_m = carrier_reference_m + reference_divergence_m;

            report.improvement_observations += 1;
            raw_abs_errors_m.push((residual.pseudorange_m.raw - expected_pseudorange_m).abs());
            smoothed_abs_errors_m
                .push((residual.pseudorange_m.corrected - expected_pseudorange_m).abs());
        }
    }

    if !raw_abs_errors_m.is_empty() {
        let raw_mean_abs_residual_m = mean(&raw_abs_errors_m);
        let smoothed_mean_abs_residual_m = mean(&smoothed_abs_errors_m);
        report.raw_mean_abs_residual_m = Some(raw_mean_abs_residual_m);
        report.smoothed_mean_abs_residual_m = Some(smoothed_mean_abs_residual_m);
        report.improvement_delta_m = Some(raw_mean_abs_residual_m - smoothed_mean_abs_residual_m);
        report.improvement_verified = Some(smoothed_mean_abs_residual_m < raw_mean_abs_residual_m);
    }

    if report.cycle_slip_observations > 0 {
        report.slip_visibility_verified = Some(report.hidden_slip_observations == 0);
    }

    report
}

pub fn validate_carrier_smoothed_code_from_artifacts(
    artifacts: &ObservationPipelineArtifacts,
) -> CarrierSmoothedCodeValidationReport {
    validate_carrier_smoothed_code(&artifacts.epochs, &artifacts.residuals)
}

fn mean(values: &[f64]) -> f64 {
    values.iter().sum::<f64>() / values.len() as f64
}

fn residuals_by_signal(
    residuals: &[ObservationResidualEpochReport],
) -> HashMap<ResidualKey, ObservationResidualSatellite> {
    let mut by_signal = HashMap::new();
    for epoch in residuals {
        for sat in &epoch.sats {
            by_signal.insert(
                ResidualKey { epoch_idx: epoch.epoch_idx, signal_id: sat.signal_id },
                sat.clone(),
            );
        }
    }
    by_signal
}

fn residuals_preserve_raw_code(residual: &ObservationResidualSatellite) -> bool {
    (residual.pseudorange_m.corrected - residual.pseudorange_m.raw).abs()
        <= RAW_CORRECTED_MATCH_TOLERANCE_M
}

#[cfg(test)]
mod tests {
    use super::{summarize_carrier_smoothed_code, validate_carrier_smoothed_code_from_artifacts};
    use crate::api::ReceiverPipelineConfig;
    use crate::pipeline::doppler::carrier_hz_from_doppler_hz;
    use crate::pipeline::observations::observation_artifacts_from_tracking_results_with_gps_anchor;
    use crate::pipeline::tracking::TrackingResult;
    use bijux_gnss_core::api::{
        Chips, Constellation, Cycles, Epoch, GpsTime, Hertz, ReceiverSampleTrace, SatId,
        SignalBand, SignalDelayAlignment, TrackEpoch,
    };
    use std::f64::consts::PI;

    const OBSERVATION_CN0_DBHZ: f64 = 60.0;
    const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;
    const GPS_L1_HZ: f64 = 1_575_420_000.0;

    fn tracking_config() -> ReceiverPipelineConfig {
        ReceiverPipelineConfig {
            sampling_freq_hz: 1_023_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            channels: 4,
            early_late_spacing_chips: 0.5,
            dll_bw_hz: 2.0,
            pll_bw_hz: 15.0,
            fll_bw_hz: 10.0,
            ..ReceiverPipelineConfig::default()
        }
    }

    fn samples_per_epoch(config: &ReceiverPipelineConfig) -> u64 {
        ((config.sampling_freq_hz * config.code_length as f64) / config.code_freq_basis_hz).round()
            as u64
    }

    fn aligned_tracking_epoch(
        config: &ReceiverPipelineConfig,
        sat: SatId,
        epoch_idx: u64,
        carrier_hz: f64,
        carrier_phase_cycles: f64,
        whole_code_periods: u64,
        code_phase_samples: f64,
    ) -> TrackEpoch {
        let sample_index = epoch_idx * samples_per_epoch(config);
        TrackEpoch {
            epoch: Epoch { index: epoch_idx },
            sample_index,
            source_time: ReceiverSampleTrace::from_sample_index(
                sample_index,
                config.sampling_freq_hz,
            ),
            sat,
            signal_band: match sat.constellation {
                Constellation::Galileo => SignalBand::E1,
                _ => SignalBand::L1,
            },
            glonass_frequency_channel: None,
            prompt_i: 1.0,
            prompt_q: 0.0,
            early_i: 0.0,
            early_q: 0.0,
            late_i: 0.0,
            late_q: 0.0,
            carrier_hz: Hertz(carrier_hz),
            carrier_phase_cycles: Cycles(carrier_phase_cycles),
            code_rate_hz: Hertz(config.code_freq_basis_hz),
            code_phase_samples: Chips(test_tracking_code_phase_samples(config, code_phase_samples)),
            lock: true,
            cn0_dbhz: OBSERVATION_CN0_DBHZ,
            pll_lock: true,
            dll_lock: true,
            fll_lock: true,
            cycle_slip: false,
            nav_bit_lock: false,
            navigation_bit_sign: None,
            dll_err: 0.0,
            pll_err: 0.0,
            fll_err: 0.0,
            anti_false_lock: false,
            cycle_slip_reason: None,
            lock_state: "tracking".to_string(),
            lock_state_reason: Some("stable_tracking".to_string()),
            channel_id: Some(0),
            channel_uid: format!("Gps-{:02}-ch00", sat.prn),
            tracking_provenance: "carrier_smoothed_code_validation".to_string(),
            tracking_assumptions: None,
            signal_delay_alignment: Some(SignalDelayAlignment {
                whole_code_periods,
                source: "synthetic_truth".to_string(),
            }),
            tracking_uncertainty: None,
            processing_ms: None,
        }
    }

    fn test_tracking_code_phase_samples(
        config: &ReceiverPipelineConfig,
        aligned_code_phase_samples: f64,
    ) -> f64 {
        if !aligned_code_phase_samples.is_finite() || aligned_code_phase_samples < 0.0 {
            return aligned_code_phase_samples;
        }
        (samples_per_epoch(config) as f64 - aligned_code_phase_samples)
            .rem_euclid(samples_per_epoch(config) as f64)
    }

    fn observation_track(sat: SatId, epochs: Vec<TrackEpoch>) -> TrackingResult {
        TrackingResult {
            sat,
            carrier_hz: epochs.last().map(|epoch| epoch.carrier_hz.0).unwrap_or_default(),
            code_phase_samples: epochs
                .last()
                .map(|epoch| epoch.code_phase_samples.0)
                .unwrap_or_default(),
            acquisition_hypothesis: "accepted".to_string(),
            acquisition_score: 1.0,
            acquisition_code_phase_samples: 0,
            acquisition_carrier_hz: epochs
                .first()
                .map(|epoch| epoch.carrier_hz.0)
                .unwrap_or_default(),
            acq_to_track_state: "accepted".to_string(),
            epochs,
            transitions: Vec::new(),
        }
    }

    #[test]
    fn carrier_smoothed_code_validation_proves_noise_reduction() {
        let config = tracking_config();
        let sat = SatId { constellation: Constellation::Gps, prn: 30 };
        let carrier_hz = carrier_hz_from_doppler_hz(config.intermediate_freq_hz, 125.0);
        let lambda_m = SPEED_OF_LIGHT_MPS / GPS_L1_HZ;
        let meters_per_sample = SPEED_OF_LIGHT_MPS / config.sampling_freq_hz;
        let carrier_delta_cycles = 0.125;
        let carrier_delta_samples = carrier_delta_cycles * lambda_m / meters_per_sample;
        let epoch_count = 18usize;
        let truth_code_phase_samples = (0..epoch_count)
            .map(|offset| offset as f64 * carrier_delta_samples)
            .collect::<Vec<_>>();
        let noisy_code_phase_samples = truth_code_phase_samples
            .iter()
            .enumerate()
            .map(|(offset, truth_code_phase_samples)| {
                let noise_samples = 0.00045 * ((offset as f64) * PI / 6.0).sin();
                truth_code_phase_samples + noise_samples
            })
            .collect::<Vec<_>>();
        let artifacts = observation_artifacts_from_tracking_results_with_gps_anchor(
            &config,
            Some(GpsTime { week: 2200, tow_s: 345_600.0 }),
            &[observation_track(
                sat,
                noisy_code_phase_samples
                    .iter()
                    .enumerate()
                    .map(|(offset, code_phase_samples)| {
                        aligned_tracking_epoch(
                            &config,
                            sat,
                            70 + offset as u64,
                            carrier_hz,
                            10.0 + offset as f64 * carrier_delta_cycles,
                            68,
                            *code_phase_samples,
                        )
                    })
                    .collect(),
            )],
            8,
        )
        .output;
        let report = validate_carrier_smoothed_code_from_artifacts(&artifacts);

        assert!(report.reference_observations > 0);
        assert!(report.improvement_observations > 0);
        assert_eq!(report.improvement_verified, Some(true));
        assert!(
            report.smoothed_mean_abs_residual_m.expect("smoothed mean")
                < report.raw_mean_abs_residual_m.expect("raw mean")
        );
        assert_eq!(report.cycle_slip_observations, 0);
        assert_eq!(report.slip_visibility_verified, None);
    }

    #[test]
    fn carrier_smoothed_code_validation_preserves_cycle_slips() {
        let config = tracking_config();
        let sat = SatId { constellation: Constellation::Gps, prn: 20 };
        let carrier_hz = carrier_hz_from_doppler_hz(config.intermediate_freq_hz, 125.0);
        let artifacts = observation_artifacts_from_tracking_results_with_gps_anchor(
            &config,
            Some(GpsTime { week: 2200, tow_s: 345_600.0 }),
            &[observation_track(
                sat,
                vec![
                    aligned_tracking_epoch(&config, sat, 70, carrier_hz, 10.0, 68, 0.0),
                    aligned_tracking_epoch(&config, sat, 71, carrier_hz, 10.125, 68, 0.0),
                    aligned_tracking_epoch(&config, sat, 72, carrier_hz, 10.250, 68, 0.02),
                    aligned_tracking_epoch(&config, sat, 73, carrier_hz, 10.375, 68, 0.02),
                ],
            )],
            10,
        )
        .output;
        let report = validate_carrier_smoothed_code_from_artifacts(&artifacts);
        let slip_summary = summarize_carrier_smoothed_code(&artifacts.epochs);

        assert_eq!(report.cycle_slip_observations, 1);
        assert_eq!(report.slip_reset_observations, 1);
        assert_eq!(report.slip_preserved_observations, 1);
        assert_eq!(report.hidden_slip_observations, 0);
        assert_eq!(report.slip_visibility_verified, Some(true));
        assert_eq!(slip_summary.slip_visibility_verified, Some(true));
    }
}
