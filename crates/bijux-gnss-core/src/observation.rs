#![allow(missing_docs)]
#![allow(dead_code)]

use crate::api::{Cycles, Hertz, Meters, Seconds, SigId, SignalBand, SignalCode};
use serde::{Deserialize, Serialize};

pub(crate) mod acquisition;
pub(crate) mod epochs;
pub(crate) mod navigation;
pub(crate) mod tracking;

fn default_signal_band() -> SignalBand {
    SignalBand::L1
}

fn default_signal_code() -> SignalCode {
    SignalCode::Unknown
}

pub const TRACKING_STATE_MODEL_VERSION: u32 = 1;
pub const OBSERVATION_MODEL_VERSION: u32 = 3;
pub const OBSERVATION_DOWNSTREAM_PROFILE_VERSION: u32 = 1;
pub const OBSERVATION_DOPPLER_MODEL_TRACKED_CARRIER_IF_OFFSET: &str =
    "tracked_carrier_hz_minus_intermediate_freq";

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub struct SignalDelayAlignment {
    pub whole_code_periods: u64,
    #[serde(default)]
    pub sample_delay_samples: u64,
    pub source: String,
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct AmbiguityId {
    pub sig: SigId,
    pub signal: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum AmbiguityStatus {
    Unknown,
    Float,
    Fixed,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AmbiguityState {
    pub id: AmbiguityId,
    pub float_cycles: Cycles,
    pub variance: f64,
    pub status: AmbiguityStatus,
    pub last_update_epoch: u64,
    pub carrier_phase_arc_id: Option<String>,
    pub valid_for_carrier_phase_arc: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub(crate) struct CarrierPhaseTerms {
    pub range_m: Meters,
    pub receiver_clock_s: Seconds,
    pub sat_clock_s: Seconds,
    pub tropo_m: Meters,
    pub iono_m: Meters,
    pub wavelength_m: Meters,
    pub ambiguity_cycles: Cycles,
}

pub(crate) fn carrier_phase_cycles(terms: &CarrierPhaseTerms) -> f64 {
    let corrected_m = terms.range_m.0
        + 299_792_458.0 * (terms.receiver_clock_s.0 - terms.sat_clock_s.0)
        + terms.tropo_m.0
        - terms.iono_m.0;
    corrected_m / terms.wavelength_m.0 + terms.ambiguity_cycles.0
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SingleDifference {
    pub sig: SigId,
    pub code_m: Meters,
    pub phase_cycles: Cycles,
    pub doppler_hz: Hertz,
    pub ambiguity_rover: AmbiguityId,
    pub ambiguity_base: AmbiguityId,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DoubleDifference {
    pub ref_sig: SigId,
    pub sig: SigId,
    pub code_m: Meters,
    pub phase_cycles: Cycles,
    pub doppler_hz: Hertz,
    pub canceled: Vec<AmbiguityId>,
}

pub(crate) fn geometry_free_phase_m(
    phase1_cycles: f64,
    phase2_cycles: f64,
    lambda1_m: f64,
    lambda2_m: f64,
) -> f64 {
    phase1_cycles * lambda1_m - phase2_cycles * lambda2_m
}

pub(crate) fn melbourne_wubbena_m(
    code1_m: f64,
    code2_m: f64,
    phase1_cycles: f64,
    phase2_cycles: f64,
    lambda1_m: f64,
    lambda2_m: f64,
) -> f64 {
    let phi1 = phase1_cycles * lambda1_m;
    let phi2 = phase2_cycles * lambda2_m;
    (phi1 - phi2) - (code1_m - code2_m)
}

// Errors moved to crate::error.

#[cfg(test)]
mod tests {
    use super::{
        acquisition::acq_result_stability_key,
        epochs::{obs_epoch_stability_key, ObsEpoch},
        tracking::TrackingUncertainty,
        SignalDelayAlignment, OBSERVATION_DOPPLER_MODEL_TRACKED_CARRIER_IF_OFFSET,
    };
    use crate::api::{
        trackable_acq_tracking_seeds, AcqCodePhaseRefinement, AcqComponentCombinationMode,
        AcqComponentProvenance, AcqComponentStatistic, AcqEvidence, AcqHypothesis, AcqResult,
        AcqSearchSummary, AcqUncertainty, AcqUncertaintyCovariance, CarrierPhaseArc,
        CodeCarrierDivergence, Constellation, CycleSlipDecisionEvidence, CycleSlipDetector,
        CycleSlipDetectorEvidence, Cycles, GlonassFrequencyChannel, Hertz, LeapSeconds, LockFlags,
        MeasurementErrorModel, NavLifecycleState, NavQualityFlag, ObsMetadata,
        ObservationCovarianceStatus, ObservationEpochDecision, ObservationStatus, ReceiverRole,
        ReceiverSampleTrace, SatId, SignalBand, SignalCode, SignalComponentRole, SignalSpec,
        SolutionStatus, UtcTime, GPS_L1_CA_CARRIER_HZ,
    };
    use crate::observation_quality::ObsSatellite;
    use crate::time::utc_to_gps;

    #[test]
    fn leap_second_table_validates() {
        let table = LeapSeconds::default_table();
        table.validate().expect("leap second table valid");
        assert_eq!(table.latest_offset(), 18);
    }

    #[test]
    fn leap_second_offset_lookup() {
        let table = LeapSeconds::default_table();
        assert_eq!(table.offset_at_utc(0.0), 0);
        assert_eq!(table.offset_at_utc(362_793_600.0), 1);
        assert_eq!(table.offset_at_utc(1_483_228_800.0), 18);
    }

    #[test]
    fn utc_to_gps_uses_leap_offset() {
        let table = LeapSeconds::default_table();
        let utc = UtcTime { unix_s: 1_700_000_000.0 };
        let gps = utc_to_gps(utc, &table);
        let offset = table.offset_at_utc(utc.unix_s);
        let expected = utc.unix_s - 315_964_800.0 + offset as f64;
        assert!((gps.to_seconds() - expected).abs() < 1e-6);
    }

    #[test]
    fn utc_to_gps_anchors_week_zero_to_gps_epoch() {
        let table = LeapSeconds::default_table();
        let gps = utc_to_gps(UtcTime { unix_s: 315_964_800.0 }, &table);

        assert_eq!(gps.week, 0);
        assert_eq!(gps.tow_s, 0.0);
    }

    #[test]
    fn precise_solution_status_validity_matches_runtime_semantics() {
        assert!(!SolutionStatus::Unavailable.is_valid());
        assert!(!SolutionStatus::Refused.is_valid());
        assert!(SolutionStatus::Degraded.is_valid());
        assert!(!SolutionStatus::IntegrityFailed.is_valid());
        assert!(!SolutionStatus::Diverged.is_valid());
        assert!(SolutionStatus::CodeOnly.is_valid());
        assert!(SolutionStatus::Float.is_valid());
        assert!(SolutionStatus::Fixed.is_valid());
    }

    #[test]
    fn precise_solution_status_quality_flags_distinguish_no_fix_and_degraded_cases() {
        assert_eq!(SolutionStatus::Unavailable.quality_flag(), NavQualityFlag::NoFix);
        assert_eq!(SolutionStatus::Refused.quality_flag(), NavQualityFlag::NoFix);
        assert_eq!(SolutionStatus::Degraded.quality_flag(), NavQualityFlag::Degraded);
        assert_eq!(SolutionStatus::IntegrityFailed.quality_flag(), NavQualityFlag::Degraded);
        assert_eq!(SolutionStatus::Diverged.quality_flag(), NavQualityFlag::Degraded);
        assert_eq!(SolutionStatus::CodeOnly.quality_flag(), NavQualityFlag::Float);
        assert_eq!(SolutionStatus::Float.quality_flag(), NavQualityFlag::Float);
        assert_eq!(SolutionStatus::Fixed.quality_flag(), NavQualityFlag::Fix);
    }

    #[test]
    fn precise_solution_status_lifecycle_state_tracks_status_exactly() {
        assert_eq!(SolutionStatus::Unavailable.lifecycle_state(), NavLifecycleState::Unavailable);
        assert_eq!(SolutionStatus::Refused.lifecycle_state(), NavLifecycleState::Refused);
        assert_eq!(SolutionStatus::Degraded.lifecycle_state(), NavLifecycleState::Degraded);
        assert_eq!(
            SolutionStatus::IntegrityFailed.lifecycle_state(),
            NavLifecycleState::IntegrityFailed
        );
        assert_eq!(SolutionStatus::Diverged.lifecycle_state(), NavLifecycleState::Diverged);
        assert_eq!(SolutionStatus::CodeOnly.lifecycle_state(), NavLifecycleState::CodeOnly);
        assert_eq!(SolutionStatus::Float.lifecycle_state(), NavLifecycleState::Float);
        assert_eq!(SolutionStatus::Fixed.lifecycle_state(), NavLifecycleState::Fixed);
    }

    #[test]
    fn precise_solution_status_decision_labels_match_public_status_words() {
        assert_eq!(SolutionStatus::Unavailable.decision_label(), "unavailable");
        assert_eq!(SolutionStatus::Refused.decision_label(), "refused");
        assert_eq!(SolutionStatus::Degraded.decision_label(), "degraded");
        assert_eq!(SolutionStatus::IntegrityFailed.decision_label(), "integrity_failed");
        assert_eq!(SolutionStatus::Diverged.decision_label(), "diverged");
        assert_eq!(SolutionStatus::CodeOnly.decision_label(), "accepted");
        assert_eq!(SolutionStatus::Float.decision_label(), "accepted");
        assert_eq!(SolutionStatus::Fixed.decision_label(), "accepted");
    }

    #[test]
    fn acq_search_summary_counts_each_decision() {
        let sat = SatId { constellation: Constellation::Gps, prn: 1 };
        let results = vec![
            acq_result_for_summary(sat, AcqHypothesis::Accepted),
            acq_result_for_summary(sat, AcqHypothesis::Ambiguous),
            acq_result_for_summary(sat, AcqHypothesis::Rejected),
            acq_result_for_summary(sat, AcqHypothesis::Deferred),
        ];

        let summary = AcqSearchSummary::from_results(&results);

        assert_eq!(
            summary,
            AcqSearchSummary {
                searched_satellites: 4,
                accepted: 1,
                ambiguous: 1,
                rejected: 1,
                deferred: 1,
            }
        );
    }

    #[test]
    fn acq_tracking_seed_uses_explicit_tracking_start_fields() {
        let sat = SatId { constellation: Constellation::Gps, prn: 7 };
        let result = AcqResult {
            sat,
            signal_band: SignalBand::L1,
            signal_code: SignalCode::Unknown,
            glonass_frequency_channel: None,
            source_time: ReceiverSampleTrace::from_sample_index(8_184, 4_092_000.0),
            candidate_rank: 1,
            is_primary_candidate: true,
            doppler_hz: Hertz(750.0),
            doppler_rate_hz_per_s: 0.0,
            carrier_hz: Hertz(750.0),
            code_phase_samples: 100,
            peak: 0.0,
            second_peak: 0.0,
            mean: 0.0,
            peak_mean_ratio: 0.0,
            peak_second_ratio: 0.0,
            cn0_proxy: 0.0,
            score: 1.0,
            hypothesis: AcqHypothesis::Accepted,
            assumptions: None,
            evidence: Vec::new(),
            threshold_provenance: None,
            explain_selection_reason: None,
            doppler_refinement: None,
            code_phase_refinement: Some(AcqCodePhaseRefinement {
                method: "parabolic_code_peak".to_string(),
                offset_samples: 0.125,
                refined_code_phase_samples: 100.125,
                left_correlation_norm: 0.8,
                center_correlation_norm: 1.0,
                right_correlation_norm: 0.7,
            }),
            signal_delay_alignment: Some(SignalDelayAlignment {
                whole_code_periods: 68,
                sample_delay_samples: 0,
                source: "synthetic_truth".to_string(),
            }),
            uncertainty: Some(AcqUncertainty {
                doppler_hz: 125.0,
                code_phase_samples: 0.25,
                doppler_rate_hz_per_s: None,
                covariance: Some(AcqUncertaintyCovariance {
                    doppler_variance_hz2: 15_625.0,
                    doppler_code_phase_covariance_hz_samples: -3.0,
                    code_phase_variance_samples2: 0.0625,
                    doppler_rate_variance_hz2_per_s2: None,
                    doppler_doppler_rate_covariance_hz2_per_s: None,
                    code_phase_doppler_rate_covariance_samples_hz_per_s: None,
                }),
            }),
        };

        let seed = result.tracking_seed();

        assert_eq!(seed.sat, sat);
        assert_eq!(seed.signal_band, SignalBand::L1);
        assert_eq!(seed.glonass_frequency_channel, None);
        assert_eq!(seed.source_time, result.source_time);
        assert_eq!(seed.doppler_hz.0, 750.0);
        assert!((seed.code_phase_samples.0 - 100.125).abs() <= f64::EPSILON);
        assert_eq!(
            seed.signal_delay_alignment,
            Some(SignalDelayAlignment {
                whole_code_periods: 68,
                sample_delay_samples: 0,
                source: "synthetic_truth".to_string(),
            })
        );
        assert_eq!(seed.uncertainty.as_ref().map(|u| u.doppler_hz), Some(125.0));
    }

    #[test]
    fn acq_result_stability_key_includes_signal_delay_alignment() {
        let sat = SatId { constellation: Constellation::Gps, prn: 9 };
        let mut base = acq_result_for_summary(sat, AcqHypothesis::Accepted);
        base.signal_delay_alignment = Some(SignalDelayAlignment {
            whole_code_periods: 68,
            sample_delay_samples: 0,
            source: "synthetic_truth".to_string(),
        });
        let mut changed = base.clone();
        changed.signal_delay_alignment = Some(SignalDelayAlignment {
            whole_code_periods: 68,
            sample_delay_samples: 4,
            source: "synthetic_truth".to_string(),
        });

        assert_ne!(acq_result_stability_key(&base), acq_result_stability_key(&changed));
    }

    #[test]
    fn acq_tracking_seed_preserves_glonass_frequency_channel() {
        let sat = SatId { constellation: Constellation::Glonass, prn: 8 };
        let channel =
            GlonassFrequencyChannel::new(-4).expect("channel -4 must be valid for GLONASS");
        let mut result = acq_result_for_summary(sat, AcqHypothesis::Accepted);
        result.glonass_frequency_channel = Some(channel);

        let seed = result.tracking_seed();

        assert_eq!(seed.sat, sat);
        assert_eq!(seed.glonass_frequency_channel, Some(channel));
    }

    #[test]
    fn acq_result_stability_key_includes_glonass_frequency_channel() {
        let sat = SatId { constellation: Constellation::Glonass, prn: 8 };
        let lower = GlonassFrequencyChannel::new(-4).expect("channel -4 must be valid");
        let upper = GlonassFrequencyChannel::new(5).expect("channel 5 must be valid");
        let mut base = acq_result_for_summary(sat, AcqHypothesis::Accepted);
        base.glonass_frequency_channel = Some(lower);
        let mut changed = base.clone();
        changed.glonass_frequency_channel = Some(upper);

        assert_ne!(acq_result_stability_key(&base), acq_result_stability_key(&changed));
    }

    #[test]
    fn acq_result_component_provenance_reads_primary_evidence() {
        let sat = SatId { constellation: Constellation::Galileo, prn: 11 };
        let mut result = acq_result_for_summary(sat, AcqHypothesis::Accepted);
        result.evidence = vec![AcqEvidence {
            rank: 1,
            code_phase_samples: result.code_phase_samples,
            doppler_hz: result.carrier_hz.0,
            doppler_rate_hz_per_s: 0.0,
            peak: result.peak,
            second_peak: result.second_peak,
            peak_mean_ratio: result.peak_mean_ratio,
            peak_second_ratio: result.peak_second_ratio,
            mean: result.mean,
            component_provenance: Some(AcqComponentProvenance {
                combination_mode: AcqComponentCombinationMode::CoherentComponentSum,
                components: vec![
                    AcqComponentStatistic {
                        role: SignalComponentRole::Data,
                        peak: 12.0,
                        second_peak: 3.0,
                        mean: 1.5,
                        peak_mean_ratio: 8.0,
                        peak_second_ratio: 4.0,
                        secondary_code_phase_periods: None,
                    },
                    AcqComponentStatistic {
                        role: SignalComponentRole::Pilot,
                        peak: 11.0,
                        second_peak: 2.5,
                        mean: 1.4,
                        peak_mean_ratio: 7.857143,
                        peak_second_ratio: 4.4,
                        secondary_code_phase_periods: None,
                    },
                ],
            }),
        }];

        let provenance =
            result.component_provenance().expect("component provenance must be discoverable");

        assert_eq!(provenance.combination_mode, AcqComponentCombinationMode::CoherentComponentSum);
        assert_eq!(provenance.components.len(), 2);
        assert_eq!(provenance.components[0].role, SignalComponentRole::Data);
        assert_eq!(provenance.components[1].role, SignalComponentRole::Pilot);
    }

    #[test]
    fn acq_result_stability_key_includes_component_provenance() {
        let sat = SatId { constellation: Constellation::Galileo, prn: 11 };
        let mut base = acq_result_for_summary(sat, AcqHypothesis::Accepted);
        base.evidence = vec![AcqEvidence {
            rank: 1,
            code_phase_samples: base.code_phase_samples,
            doppler_hz: base.carrier_hz.0,
            doppler_rate_hz_per_s: 0.0,
            peak: base.peak,
            second_peak: base.second_peak,
            peak_mean_ratio: base.peak_mean_ratio,
            peak_second_ratio: base.peak_second_ratio,
            mean: base.mean,
            component_provenance: Some(AcqComponentProvenance {
                combination_mode: AcqComponentCombinationMode::NoncoherentComponentSum,
                components: vec![AcqComponentStatistic {
                    role: SignalComponentRole::Data,
                    peak: 12.0,
                    second_peak: 3.0,
                    mean: 1.5,
                    peak_mean_ratio: 8.0,
                    peak_second_ratio: 4.0,
                    secondary_code_phase_periods: None,
                }],
            }),
        }];
        let mut changed = base.clone();
        changed.evidence[0].component_provenance = Some(AcqComponentProvenance {
            combination_mode: AcqComponentCombinationMode::CoherentComponentSum,
            components: vec![AcqComponentStatistic {
                role: SignalComponentRole::Data,
                peak: 12.0,
                second_peak: 3.0,
                mean: 1.5,
                peak_mean_ratio: 8.0,
                peak_second_ratio: 4.0,
                secondary_code_phase_periods: None,
            }],
        });

        assert_ne!(acq_result_stability_key(&base), acq_result_stability_key(&changed));
    }

    #[test]
    fn acq_result_stability_key_includes_component_secondary_code_phase() {
        let sat = SatId { constellation: Constellation::Galileo, prn: 11 };
        let mut base = acq_result_for_summary(sat, AcqHypothesis::Accepted);
        base.evidence = vec![AcqEvidence {
            rank: 1,
            code_phase_samples: base.code_phase_samples,
            doppler_hz: base.carrier_hz.0,
            doppler_rate_hz_per_s: 0.0,
            peak: base.peak,
            second_peak: base.second_peak,
            peak_mean_ratio: base.peak_mean_ratio,
            peak_second_ratio: base.peak_second_ratio,
            mean: base.mean,
            component_provenance: Some(AcqComponentProvenance {
                combination_mode: AcqComponentCombinationMode::CoherentComponentSum,
                components: vec![AcqComponentStatistic {
                    role: SignalComponentRole::Pilot,
                    peak: 12.0,
                    second_peak: 3.0,
                    mean: 1.5,
                    peak_mean_ratio: 8.0,
                    peak_second_ratio: 4.0,
                    secondary_code_phase_periods: Some(17),
                }],
            }),
        }];
        let mut changed = base.clone();
        changed.evidence[0]
            .component_provenance
            .as_mut()
            .expect("component provenance must exist")
            .components[0]
            .secondary_code_phase_periods = Some(18);

        assert_ne!(acq_result_stability_key(&base), acq_result_stability_key(&changed));
    }

    #[test]
    fn obs_metadata_defaults_carrier_phase_contract() {
        let metadata = ObsMetadata::default();

        assert_eq!(metadata.carrier_phase_model, "tracked_carrier_cycles");
        assert_eq!(metadata.carrier_phase_continuity, "unusable");
        assert_eq!(metadata.carrier_phase_arc_start_epoch_idx, 0);
        assert_eq!(metadata.carrier_phase_arc_start_sample_index, 0);
        assert_eq!(metadata.carrier_phase_arc, None);
    }

    #[test]
    fn obs_metadata_defaults_doppler_contract() {
        let metadata = ObsMetadata::default();

        assert_eq!(metadata.doppler_model, OBSERVATION_DOPPLER_MODEL_TRACKED_CARRIER_IF_OFFSET);
    }

    #[test]
    fn obs_metadata_defaults_receiver_clock_contract() {
        let metadata = ObsMetadata::default();

        assert_eq!(metadata.receiver_clock_bias_s, crate::api::Seconds(0.0));
        assert_eq!(metadata.receiver_clock_frequency_bias_hz, 0.0);
        assert_eq!(metadata.receiver_clock_bias_sigma_s, crate::api::Seconds(0.0));
        assert_eq!(metadata.receiver_clock_source, "");
    }

    #[test]
    fn code_carrier_divergence_residual_tracks_unexplained_component() {
        let divergence = CodeCarrierDivergence::from_terms(130.0, 12.5, 7.0, 1.5, -0.25, 0.75, 2.0);

        assert_eq!(divergence.raw_m, 130.0);
        assert_eq!(divergence.jump_m, 12.5);
        assert_eq!(divergence.expected_ionosphere_m, 7.0);
        assert_eq!(divergence.receiver_clock_m, 1.5);
        assert_eq!(divergence.oscillator_m, -0.25);
        assert_eq!(divergence.smoothing_transient_m, 0.75);
        assert_eq!(divergence.multipath_m, 2.0);
        assert_eq!(divergence.unexplained_m, 1.5);
        assert_eq!(divergence.unexplained_abs_m(), 1.5);
    }

    #[test]
    fn obs_metadata_defaults_do_not_invent_code_carrier_divergence() {
        let metadata = ObsMetadata::default();

        assert_eq!(metadata.code_carrier_divergence, None);
    }

    #[test]
    fn cycle_slip_decision_records_triggered_detector_budget() {
        let mut decision = CycleSlipDecisionEvidence::from_contributors(
            vec![
                CycleSlipDetectorEvidence::new(
                    CycleSlipDetector::TrackingLock,
                    false,
                    None,
                    None,
                    "",
                    "tracking_lock_continuous",
                ),
                CycleSlipDetectorEvidence::new(
                    CycleSlipDetector::PhaseInnovation,
                    true,
                    Some(0.42),
                    Some(0.15),
                    "cycles",
                    "phase_residual",
                ),
            ],
            0.99,
            1.0e-3,
        );

        assert!(decision.detected);
        assert_eq!(decision.primary_reason.as_deref(), Some("phase_residual"));
        assert_eq!(decision.triggered_detectors(), vec![CycleSlipDetector::PhaseInnovation]);
        assert_eq!(decision.detection_probability_budget, 0.99);
        assert_eq!(decision.false_alarm_probability_budget, 1.0e-3);

        decision.upsert_contributor(CycleSlipDetectorEvidence::new(
            CycleSlipDetector::TrackingLock,
            true,
            None,
            None,
            "",
            "loss_of_lock",
        ));

        assert_eq!(
            decision.triggered_detectors(),
            vec![CycleSlipDetector::TrackingLock, CycleSlipDetector::PhaseInnovation]
        );
        assert_eq!(decision.primary_reason.as_deref(), Some("loss_of_lock"));
    }

    #[test]
    fn obs_metadata_defaults_do_not_invent_cycle_slip_evidence() {
        let metadata = ObsMetadata::default();

        assert_eq!(metadata.cycle_slip_evidence, None);
    }

    #[test]
    fn carrier_phase_arc_id_includes_signal_and_start_boundary() {
        let signal_id = crate::api::SigId {
            sat: SatId { constellation: Constellation::Gps, prn: 4 },
            band: SignalBand::L1,
            code: crate::api::SignalCode::Ca,
        };
        let arc = CarrierPhaseArc::new(signal_id, 70, 286_440, "arc_start");
        let invalid = CarrierPhaseArc::invalid_boundary(signal_id, "loss_of_lock");

        assert_eq!(arc.signal_id, signal_id);
        assert_eq!(arc.start_epoch_idx, 70);
        assert_eq!(arc.start_sample_index, 286_440);
        assert_eq!(arc.start_reason, "arc_start");
        assert!(arc.valid_for_smoothing);
        assert!(arc.valid_for_ambiguity);
        assert!(arc.id.contains("gps-04-l1-ca-e0000000070-s000000286440"));
        assert_eq!(invalid.start_reason, "loss_of_lock");
        assert!(!invalid.valid_for_smoothing);
        assert!(!invalid.valid_for_ambiguity);
    }

    #[test]
    fn obs_metadata_defaults_observation_lock_contract() {
        let metadata = ObsMetadata::default();

        assert_eq!(metadata.observation_lock_state, "inactive");
        assert_eq!(metadata.observation_lock_reason, None);
    }

    #[test]
    fn obs_epoch_stability_key_includes_carrier_phase_arc_metadata() {
        let sat_id = SatId { constellation: Constellation::Gps, prn: 4 };
        let base = ObsEpoch {
            t_rx_s: crate::api::Seconds(0.07),
            source_time: ReceiverSampleTrace::from_sample_index(286_440, 4_092_000.0),
            gps_week: None,
            tow_s: None,
            epoch_idx: 70,
            discontinuity: false,
            valid: true,
            processing_ms: None,
            role: ReceiverRole::Rover,
            sats: vec![ObsSatellite {
                signal_id: crate::api::SigId {
                    sat: sat_id,
                    band: SignalBand::L1,
                    code: crate::api::SignalCode::Ca,
                },
                pseudorange_m: crate::api::Meters(21_000_000.0),
                pseudorange_var_m2: 1.0,
                carrier_phase_cycles: Cycles(12.25),
                carrier_phase_var_cycles2: 1.0,
                doppler_hz: Hertz(125.0),
                doppler_var_hz2: 1.0,
                cn0_dbhz: 45.0,
                lock_flags: LockFlags {
                    code_lock: true,
                    carrier_lock: true,
                    bit_lock: false,
                    cycle_slip: false,
                },
                multipath_suspect: false,
                observation_status: ObservationStatus::Accepted,
                observation_reject_reasons: Vec::new(),
                elevation_deg: None,
                azimuth_deg: None,
                weight: None,
                timing: None,
                error_model: None,
                metadata: ObsMetadata {
                    carrier_phase_continuity: "continuous".to_string(),
                    carrier_phase_arc_start_epoch_idx: 70,
                    carrier_phase_arc_start_sample_index: 286_440,
                    carrier_phase_arc: Some(CarrierPhaseArc::new(
                        crate::api::SigId {
                            sat: sat_id,
                            band: SignalBand::L1,
                            code: crate::api::SignalCode::Ca,
                        },
                        70,
                        286_440,
                        "arc_start",
                    )),
                    ..ObsMetadata::default()
                },
            }],
            decision: ObservationEpochDecision::Accepted,
            decision_reason: None,
            manifest: None,
        };
        let mut changed = base.clone();
        changed.sats[0].metadata.carrier_phase_continuity = "reset_after_cycle_slip".to_string();
        changed.sats[0].metadata.carrier_phase_arc_start_epoch_idx = 73;
        changed.sats[0].metadata.carrier_phase_arc_start_sample_index = 298_716;
        changed.sats[0].metadata.carrier_phase_arc = Some(CarrierPhaseArc::new(
            crate::api::SigId {
                sat: sat_id,
                band: SignalBand::L1,
                code: crate::api::SignalCode::Ca,
            },
            73,
            298_716,
            "cycle_slip",
        ));

        assert_ne!(obs_epoch_stability_key(&base), obs_epoch_stability_key(&changed));

        let mut changed_arc_only = base.clone();
        changed_arc_only.sats[0].metadata.carrier_phase_arc = Some(CarrierPhaseArc::new(
            crate::api::SigId {
                sat: sat_id,
                band: SignalBand::L1,
                code: crate::api::SignalCode::Ca,
            },
            70,
            286_441,
            "arc_start",
        ));
        assert_ne!(obs_epoch_stability_key(&base), obs_epoch_stability_key(&changed_arc_only));
    }

    #[test]
    fn obs_epoch_stability_key_includes_doppler_model() {
        let sat_id = SatId { constellation: Constellation::Gps, prn: 4 };
        let base = ObsEpoch {
            t_rx_s: crate::api::Seconds(0.07),
            source_time: ReceiverSampleTrace::from_sample_index(286_440, 4_092_000.0),
            gps_week: None,
            tow_s: None,
            epoch_idx: 70,
            discontinuity: false,
            valid: true,
            processing_ms: None,
            role: ReceiverRole::Rover,
            sats: vec![ObsSatellite {
                signal_id: crate::api::SigId {
                    sat: sat_id,
                    band: SignalBand::L1,
                    code: crate::api::SignalCode::Ca,
                },
                pseudorange_m: crate::api::Meters(21_000_000.0),
                pseudorange_var_m2: 1.0,
                carrier_phase_cycles: Cycles(12.25),
                carrier_phase_var_cycles2: 1.0,
                doppler_hz: Hertz(125.0),
                doppler_var_hz2: 1.0,
                cn0_dbhz: 45.0,
                lock_flags: LockFlags {
                    code_lock: true,
                    carrier_lock: true,
                    bit_lock: false,
                    cycle_slip: false,
                },
                multipath_suspect: false,
                observation_status: ObservationStatus::Accepted,
                observation_reject_reasons: Vec::new(),
                elevation_deg: None,
                azimuth_deg: None,
                weight: None,
                timing: None,
                error_model: None,
                metadata: ObsMetadata::default(),
            }],
            decision: ObservationEpochDecision::Accepted,
            decision_reason: None,
            manifest: None,
        };
        let mut changed = base.clone();
        changed.sats[0].metadata.doppler_model = "legacy_absolute_doppler_hz".to_string();

        assert_ne!(obs_epoch_stability_key(&base), obs_epoch_stability_key(&changed));
    }

    fn covariance_satellite(
        pseudorange_var_m2: f64,
        carrier_phase_var_cycles2: f64,
        doppler_var_hz2: f64,
    ) -> ObsSatellite {
        ObsSatellite {
            signal_id: crate::api::SigId {
                sat: SatId { constellation: Constellation::Gps, prn: 4 },
                band: SignalBand::L1,
                code: crate::api::SignalCode::Ca,
            },
            pseudorange_m: crate::api::Meters(21_000_000.0),
            pseudorange_var_m2,
            carrier_phase_cycles: Cycles(12.25),
            carrier_phase_var_cycles2,
            doppler_hz: Hertz(125.0),
            doppler_var_hz2,
            cn0_dbhz: 45.0,
            lock_flags: LockFlags {
                code_lock: true,
                carrier_lock: true,
                bit_lock: false,
                cycle_slip: false,
            },
            multipath_suspect: false,
            observation_status: ObservationStatus::Accepted,
            observation_reject_reasons: Vec::new(),
            elevation_deg: None,
            azimuth_deg: None,
            weight: None,
            timing: None,
            error_model: Some(MeasurementErrorModel {
                thermal_noise_m: crate::api::Meters(0.0),
                tracking_jitter_m: crate::api::Meters(pseudorange_var_m2.sqrt()),
                multipath_proxy_m: crate::api::Meters(0.0),
                clock_error_m: crate::api::Meters(0.5),
            }),
            metadata: ObsMetadata {
                signal: SignalSpec {
                    constellation: Constellation::Gps,
                    band: SignalBand::L1,
                    code: SignalCode::Ca,
                    code_rate_hz: 1_023_000.0,
                    carrier_hz: GPS_L1_CA_CARRIER_HZ,
                },
                tracking_uncertainty: Some(TrackingUncertainty {
                    code_phase_samples: 0.01,
                    carrier_phase_cycles: carrier_phase_var_cycles2.sqrt(),
                    doppler_hz: doppler_var_hz2.sqrt(),
                    cn0_dbhz: 0.5,
                }),
                ..ObsMetadata::default()
            },
        }
    }

    #[test]
    fn observation_measurement_covariance_is_symmetric_positive_semidefinite() {
        let sat = covariance_satellite(4.0, 0.01, 9.0);

        let covariance = sat.measurement_covariance().expect("measurement covariance");
        let matrix = covariance.matrix();

        assert_eq!(covariance.status, ObservationCovarianceStatus::PositiveSemidefinite);
        assert_eq!(matrix[0][1], matrix[1][0]);
        assert_eq!(matrix[1][2], matrix[2][1]);
        assert!(matrix[0][0] > 0.0);
        assert!(matrix[1][1] > 0.0);
        assert!(matrix[2][2] > 0.0);
        let leading_minor = matrix[0][0] * matrix[1][1] - matrix[0][1].powi(2);
        let determinant = matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2].powi(2))
            - matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])
            + matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]);
        assert!(leading_minor >= 0.0);
        assert!(determinant >= -1.0e-12);
    }

    #[test]
    fn observation_measurement_covariance_uses_common_clock_error() {
        let sat = covariance_satellite(4.0, 10.0, 9.0);

        let covariance = sat.measurement_covariance().expect("measurement covariance");

        assert!((covariance.code_carrier_m2 - 0.25).abs() < 1.0e-12);
        assert_eq!(covariance.code_doppler_m_hz, 0.0);
    }

    #[test]
    fn observation_measurement_covariance_uses_carrier_loop_evidence() {
        let sat = covariance_satellite(4.0, 0.01, 9.0);

        let covariance = sat.measurement_covariance().expect("measurement covariance");

        assert!(covariance.carrier_doppler_m_hz > 0.0);
        assert_eq!(covariance.carrier_doppler_m_hz, covariance.doppler_carrier_hz_m);
    }

    #[test]
    fn observation_measurement_covariance_requires_positive_variances() {
        let sat = covariance_satellite(0.0, 0.01, 9.0);

        assert!(sat.measurement_covariance().is_none());
        assert!(sat.covariance_pseudorange_sigma_m().is_none());
    }

    #[test]
    fn trackable_acq_tracking_seeds_keep_only_trackable_results() {
        let sat = SatId { constellation: Constellation::Gps, prn: 9 };
        let accepted = acq_result_for_summary(sat, AcqHypothesis::Accepted);
        let ambiguous = acq_result_for_summary(sat, AcqHypothesis::Ambiguous);
        let rejected = acq_result_for_summary(sat, AcqHypothesis::Rejected);
        let deferred = acq_result_for_summary(sat, AcqHypothesis::Deferred);

        let seeds = trackable_acq_tracking_seeds(&[accepted, ambiguous, rejected, deferred]);

        assert_eq!(seeds.len(), 2);
        assert!(seeds.iter().all(|seed| seed.signal_band == SignalBand::L1));
    }

    fn acq_result_for_summary(sat: SatId, hypothesis: AcqHypothesis) -> AcqResult {
        AcqResult {
            sat,
            signal_band: SignalBand::L1,
            signal_code: SignalCode::Unknown,
            glonass_frequency_channel: None,
            source_time: ReceiverSampleTrace::default(),
            candidate_rank: 1,
            is_primary_candidate: true,
            doppler_hz: Hertz(0.0),
            doppler_rate_hz_per_s: 0.0,
            carrier_hz: Hertz(0.0),
            code_phase_samples: 0,
            peak: 0.0,
            second_peak: 0.0,
            mean: 0.0,
            peak_mean_ratio: 0.0,
            peak_second_ratio: 0.0,
            cn0_proxy: 0.0,
            score: 0.0,
            hypothesis,
            assumptions: None,
            evidence: Vec::new(),
            threshold_provenance: None,
            explain_selection_reason: None,
            doppler_refinement: None,
            code_phase_refinement: None,
            signal_delay_alignment: None,
            uncertainty: None,
        }
    }
}
