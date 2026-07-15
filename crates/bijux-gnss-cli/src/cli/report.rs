#[derive(Debug, Serialize)]
struct AcquisitionReport {
    sats: Vec<SatId>,
    search_summary: bijux_gnss_infra::api::core::AcqSearchSummary,
    doppler_search: DopplerSearchSettings,
    code_phase_search: CodePhaseSearchSettings,
    front_end_metrics: bijux_gnss_infra::api::signal::IqFrontEndMetrics,
    signal_quality: RawIqSignalQualityReport,
    reported_prns: Vec<ReportedPrn>,
    primary_results: Vec<AcquisitionRow>,
    results: Vec<AcquisitionRow>,
}

#[derive(Debug, Serialize, Clone)]
struct AcquisitionRow {
    sat: SatId,
    signal_band: bijux_gnss_infra::api::core::SignalBand,
    source_sample_index: u64,
    doppler_hz: f64,
    candidate_rank: u8,
    is_primary_candidate: bool,
    carrier_hz: f64,
    coarse_carrier_hz: Option<f64>,
    doppler_refinement_hz: Option<f64>,
    doppler_refinement_bins: Option<f64>,
    doppler_uncertainty_hz: Option<f64>,
    code_phase_samples: usize,
    refined_code_phase_samples: Option<f64>,
    code_phase_refinement_samples: Option<f64>,
    code_phase_uncertainty_samples: Option<f64>,
    peak: f32,
    peak_mean_ratio: f32,
    peak_second_ratio: f32,
    hypothesis: String,
    selection_reason: Option<String>,
}

#[derive(Debug, Serialize, Clone)]
struct ReportedPrn {
    sat: SatId,
    classification: String,
    carrier_hz: f64,
    peak_mean_ratio: f32,
    peak_second_ratio: f32,
}

fn summarize_reported_prns(rows: &[AcquisitionRow]) -> Vec<ReportedPrn> {
    let mut by_prn = std::collections::BTreeMap::<SatId, ReportedPrn>::new();
    for row in rows {
        if !row.is_primary_candidate {
            continue;
        }
        let classification = match row.hypothesis.as_str() {
            "accepted" => "accepted",
            "ambiguous" => "candidate",
            _ => continue,
        };
        let candidate = ReportedPrn {
            sat: row.sat,
            classification: classification.to_string(),
            carrier_hz: row.carrier_hz,
            peak_mean_ratio: row.peak_mean_ratio,
            peak_second_ratio: row.peak_second_ratio,
        };
        match by_prn.get(&row.sat) {
            Some(existing)
                if reported_prn_sort_key(existing) <= reported_prn_sort_key(&candidate) => {}
            _ => {
                by_prn.insert(row.sat, candidate);
            }
        }
    }
    let mut reported_prns: Vec<_> = by_prn.into_values().collect();
    reported_prns
        .sort_by(|left, right| reported_prn_sort_key(left).cmp(&reported_prn_sort_key(right)));
    reported_prns
}

fn reported_prn_sort_key(entry: &ReportedPrn) -> (u8, std::cmp::Reverse<u32>, u8) {
    let class_rank = if entry.classification == "accepted" { 0 } else { 1 };
    (class_rank, std::cmp::Reverse(entry.peak_mean_ratio.to_bits()), entry.sat.prn)
}

#[derive(Debug, Serialize, Clone, PartialEq, Eq)]
struct TrackedPrn {
    sat: SatId,
    epoch_count: usize,
    locked_epoch_count: usize,
    latest_lock_state: String,
}

fn summarize_tracked_prns(
    tracks: &[bijux_gnss_infra::api::receiver::TrackingResult],
) -> Vec<TrackedPrn> {
    let mut tracked_prns = tracks
        .iter()
        .filter(|track| !track.epochs.is_empty())
        .map(|track| TrackedPrn {
            sat: track.sat,
            epoch_count: track.epochs.len(),
            locked_epoch_count: track.epochs.iter().filter(|epoch| epoch.lock).count(),
            latest_lock_state: track
                .epochs
                .last()
                .map(|epoch| epoch.lock_state.clone())
                .unwrap_or_else(|| "idle".to_string()),
        })
        .collect::<Vec<_>>();
    tracked_prns.sort_by(|left, right| {
        right
            .locked_epoch_count
            .cmp(&left.locked_epoch_count)
            .then_with(|| right.epoch_count.cmp(&left.epoch_count))
            .then_with(|| left.sat.prn.cmp(&right.sat.prn))
    });
    tracked_prns
}

#[derive(Debug, Serialize, Clone, PartialEq, Eq)]
struct NavigationAttemptSummary {
    attempted_epochs: usize,
    valid_epochs: usize,
    stable_epochs: usize,
    refusal_counts: std::collections::BTreeMap<String, usize>,
}

fn summarize_navigation_attempts(
    solutions: &[bijux_gnss_infra::api::core::NavSolutionEpoch],
) -> NavigationAttemptSummary {
    let mut refusal_counts = std::collections::BTreeMap::new();
    for solution in solutions {
        if let Some(refusal) = solution.refusal_class {
            let key = format!("{refusal:?}");
            *refusal_counts.entry(key).or_insert(0usize) += 1;
        }
    }
    NavigationAttemptSummary {
        attempted_epochs: solutions.len(),
        valid_epochs: solutions.iter().filter(|solution| solution.valid).count(),
        stable_epochs: solutions
            .iter()
            .filter(|solution| {
                solution.valid
                    && matches!(
                        solution.validity,
                        bijux_gnss_infra::api::core::SolutionValidity::Stable
                    )
            })
            .count(),
        refusal_counts,
    }
}

#[derive(Debug, Serialize, Clone, PartialEq)]
struct PositionAttemptReport {
    epoch_idx: u64,
    valid: bool,
    status: bijux_gnss_infra::api::core::SolutionStatus,
    validity: bijux_gnss_infra::api::core::SolutionValidity,
    sat_count: usize,
    used_sat_count: usize,
    rejected_sat_count: usize,
    pdop: f64,
    rms_m: f64,
    refusal_class: Option<String>,
}

fn summarize_position_attempts(
    solutions: &[bijux_gnss_infra::api::core::NavSolutionEpoch],
) -> Vec<PositionAttemptReport> {
    solutions
        .iter()
        .map(|solution| PositionAttemptReport {
            epoch_idx: solution.epoch.index,
            valid: solution.valid,
            status: solution.status,
            validity: solution.validity,
            sat_count: solution.sat_count,
            used_sat_count: solution.used_sat_count,
            rejected_sat_count: solution.rejected_sat_count,
            pdop: solution.pdop,
            rms_m: solution.rms_m.0,
            refusal_class: solution.refusal_class.map(|refusal| format!("{refusal:?}")),
        })
        .collect()
}

#[derive(Debug, Serialize)]
struct CaptureValidationReport {
    acquisition: AcquisitionReport,
    tracked_prns: Vec<TrackedPrn>,
    tracking_epochs: usize,
    observation_epochs: usize,
    navigation_attempts: NavigationAttemptSummary,
    position_attempts: Vec<PositionAttemptReport>,
    validation: bijux_gnss_infra::api::receiver::ValidationReport,
}

#[derive(Debug, Serialize)]
struct InspectReport {
    format: String,
    sample_rate_hz: f64,
    intermediate_freq_hz: f64,
    capture_start_utc: String,
    total_samples: usize,
    usable_duration_s: f64,
    front_end_metrics: bijux_gnss_infra::api::signal::IqFrontEndMetrics,
    noise_floor_db: f64,
    power_histogram: Vec<u64>,
    signal_quality: RawIqSignalQualityReport,
}

#[derive(Debug, Serialize, Deserialize, Clone)]
struct RawIqSignalQualityReport {
    format: String,
    sample_rate_hz: f64,
    intermediate_freq_hz: f64,
    capture_start_utc: String,
    analyzed_samples: usize,
    usable_duration_s: f64,
    estimated_noise_floor_db: f64,
    front_end_metrics: bijux_gnss_infra::api::signal::IqFrontEndMetrics,
}

#[derive(Debug, Serialize)]
struct StreamingRunReport {
    epochs: u64,
    processed_input_samples: u64,
    acquisitions: usize,
    tracked_channels: usize,
    observation_epochs: usize,
    front_end_metrics: bijux_gnss_infra::api::signal::IqFrontEndMetrics,
    signal_quality: RawIqSignalQualityReport,
}

#[derive(Debug, Serialize, Clone)]
struct ExperimentRunResult {
    run_index: usize,
    config_hash: String,
    scenario_id: String,
    overrides: Vec<(String, String)>,
    lock_pct: f64,
    pvt_rms_m: f64,
    residual_rms_m: f64,
    rejected_count: usize,
    ms_per_epoch: f64,
}

#[derive(Debug, Serialize)]
struct ExperimentSummary {
    runs: Vec<ExperimentRunResult>,
}

#[derive(Debug, Serialize)]
struct SyntheticIqExportReport {
    scenario_id: String,
    seed: u64,
    sample_count: usize,
    sample_rate_hz: f64,
    output_iq: String,
    output_sidecar: String,
    output_truth: String,
    satellites: Vec<SatId>,
}

#[derive(Debug, Serialize)]
struct SyntheticIqValidationReport {
    input_iq: String,
    input_sidecar: String,
    input_truth: String,
    validation: bijux_gnss_infra::api::receiver::sim::SyntheticCn0ValidationReport,
    acquisition_code_phase_validation:
        bijux_gnss_infra::api::receiver::sim::SyntheticAcquisitionCodePhaseValidationReport,
    acquisition_code_phase_refinement_validation:
        bijux_gnss_infra::api::receiver::sim::SyntheticAcquisitionCodePhaseRefinementReport,
    acquisition_doppler_validation:
        bijux_gnss_infra::api::receiver::sim::SyntheticAcquisitionDopplerValidationReport,
    acquisition_receiver_clock_offset_validation: bijux_gnss_infra::api::receiver::sim::
        SyntheticAcquisitionReceiverClockOffsetValidationReport,
}

#[derive(Debug, Serialize)]
struct SyntheticNavigationValidationReport {
    scenario_id: String,
    scenario_path: String,
    output_artifact: String,
    pass: bool,
    truth_coverage_ready: bool,
    data_source: bijux_gnss_infra::api::receiver::sim::SyntheticGnssAccuracyDataSource,
    reference_truth: bijux_gnss_infra::api::receiver::sim::SyntheticGnssAccuracyReferenceTruth,
    acquisition: bijux_gnss_infra::api::receiver::sim::SyntheticGnssAcquisitionStageSummary,
    tracking: bijux_gnss_infra::api::receiver::sim::SyntheticGnssTrackingStageSummary,
    observation: bijux_gnss_infra::api::receiver::sim::SyntheticGnssObservationStageSummary,
    pvt: bijux_gnss_infra::api::receiver::sim::SyntheticGnssPvtStageSummary,
}

#[derive(Debug, Serialize)]
struct SyntheticQuantizationMeasurementReport {
    scenario_id: String,
    scenario_path: String,
    output_artifact: String,
    measured_quantizations: Vec<bijux_gnss_infra::api::signal::IqQuantization>,
    measurement: bijux_gnss_infra::api::receiver::sim::SyntheticQuantizationLossReport,
}

#[cfg(test)]
mod report_tests {
    use super::*;
    use bijux_gnss_infra::api::core::{
        Constellation, Epoch, Meters, NavLifecycleState, NavSolutionEpoch, NavUncertaintyClass,
        ReceiverSampleTrace, Seconds, SolutionStatus, SolutionValidity, TrackEpoch,
        NAV_OUTPUT_STABILITY_SIGNATURE_VERSION, NAV_SOLUTION_MODEL_VERSION,
    };

    fn gps_row(prn: u8, hypothesis: &str, peak_mean_ratio: f32) -> AcquisitionRow {
        AcquisitionRow {
            sat: SatId { constellation: Constellation::Gps, prn },
            signal_band: bijux_gnss_infra::api::core::SignalBand::L1,
            source_sample_index: 0,
            doppler_hz: 0.0,
            candidate_rank: 1,
            is_primary_candidate: true,
            carrier_hz: 500.0 * prn as f64,
            coarse_carrier_hz: None,
            doppler_refinement_hz: None,
            doppler_refinement_bins: None,
            doppler_uncertainty_hz: None,
            code_phase_samples: 42,
            refined_code_phase_samples: None,
            code_phase_refinement_samples: None,
            code_phase_uncertainty_samples: None,
            peak: peak_mean_ratio * 10.0,
            peak_mean_ratio,
            peak_second_ratio: 1.1,
            hypothesis: hypothesis.to_string(),
            selection_reason: None,
        }
    }

    fn sample_track(
        prn: u8,
        states: &[(&str, bool)],
    ) -> bijux_gnss_infra::api::receiver::TrackingResult {
        bijux_gnss_infra::api::receiver::TrackingResult {
            sat: SatId { constellation: Constellation::Gps, prn },
            carrier_hz: 0.0,
            code_phase_samples: 0.0,
            acquisition_hypothesis: "accepted".to_string(),
            acquisition_score: 1.0,
            acquisition_code_phase_samples: 0,
            acquisition_carrier_hz: 0.0,
            acq_to_track_state: "accepted".to_string(),
            epochs: states
                .iter()
                .enumerate()
                .map(|(epoch_idx, (lock_state, lock))| TrackEpoch {
                    epoch: Epoch { index: epoch_idx as u64 },
                    sample_index: epoch_idx as u64 * 4000,
                    source_time: ReceiverSampleTrace::from_sample_index(
                        epoch_idx as u64 * 4000,
                        4_000_000.0,
                    ),
                    sat: SatId { constellation: Constellation::Gps, prn },
                    signal_band: bijux_gnss_infra::api::core::SignalBand::L1,
                    signal_code: bijux_gnss_infra::api::core::SignalCode::Unknown,
                    glonass_frequency_channel: None,
                    prompt_i: 0.0,
                    prompt_q: 0.0,
                    early_i: 0.0,
                    early_q: 0.0,
                    late_i: 0.0,
                    late_q: 0.0,
                    carrier_hz: bijux_gnss_infra::api::core::Hertz(0.0),
                    carrier_phase_cycles: bijux_gnss_infra::api::core::Cycles(0.0),
                    code_rate_hz: bijux_gnss_infra::api::core::Hertz(1_023_000.0),
                    code_phase_samples: bijux_gnss_infra::api::core::Chips(0.0),
                    lock: *lock,
                    cn0_dbhz: 35.0,
                    pll_lock: *lock,
                    dll_lock: *lock,
                    fll_lock: *lock,
                    cycle_slip: false,
                    nav_bit_lock: false,
                    navigation_bit_sign: None,
                    transmit_time: None,
                    dll_err: 0.0,
                    pll_err: 0.0,
                    fll_err: 0.0,
                    anti_false_lock: false,
                    cycle_slip_reason: None,
                    lock_state: (*lock_state).to_string(),
                    lock_state_reason: None,
                    channel_id: Some(0),
                    channel_uid: format!("Gps-{prn}-00"),
                    tracking_provenance: "test".to_string(),
                    tracking_assumptions: None,
                    tracking_uncertainty: None,
                    signal_delay_alignment: None,
                    processing_ms: None,
                })
                .collect(),
            transitions: Vec::new(),
        }
    }

    fn sample_solution(
        epoch_idx: u64,
        valid: bool,
        validity: SolutionValidity,
        refusal_class: Option<bijux_gnss_infra::api::core::NavRefusalClass>,
    ) -> NavSolutionEpoch {
        NavSolutionEpoch {
            epoch: Epoch { index: epoch_idx },
            t_rx_s: Seconds(epoch_idx as f64),
            source_time: ReceiverSampleTrace::from_sample_index(epoch_idx * 4000, 4_000_000.0),
            ecef_x_m: Meters(0.0),
            ecef_y_m: Meters(0.0),
            ecef_z_m: Meters(0.0),
            position_covariance_ecef_m2: None,
            latitude_deg: 0.0,
            longitude_deg: 0.0,
            altitude_m: Meters(0.0),
            clock_bias_s: Seconds(0.0),
            clock_bias_m: Meters(0.0),
            clock_drift_s_per_s: 0.0,
            pdop: 1.0,
            rms_m: Meters(1.0),
            pre_fit_residual_rms_m: None,
            post_fit_residual_rms_m: None,
            status: SolutionStatus::CodeOnly,
            quality: SolutionStatus::CodeOnly.quality_flag(),
            validity,
            valid,
            processing_ms: None,
            residuals: Vec::new(),
            constellation_residual_rms: Vec::new(),
            health: Vec::new(),
            isb: Vec::new(),
            sigma_e_m: None,
            sigma_n_m: None,
            sigma_u_m: None,
            horizontal_error_ellipse_major_axis_m: None,
            horizontal_error_ellipse_minor_axis_m: None,
            horizontal_error_ellipse_azimuth_deg: None,
            sigma_h_m: None,
            sigma_v_m: None,
            innovation_rms_m: None,
            normalized_innovation_rms: None,
            normalized_innovation_max: None,
            ekf_innovation_rms: None,
            ekf_condition_number: None,
            wls_solver_rank: None,
            wls_condition_number: None,
            ekf_whiteness_ratio: None,
            ekf_predicted_variance: None,
            ekf_observed_variance: None,
            integrity_hpl_m: None,
            integrity_vpl_m: None,
            model_version: NAV_SOLUTION_MODEL_VERSION,
            lifecycle_state: NavLifecycleState::CodeOnly,
            uncertainty_class: NavUncertaintyClass::Low,
            assumptions: None,
            refusal_class,
            artifact_id: format!("nav-epoch-{epoch_idx:010}-report"),
            source_observation_epoch_id: format!("obs-epoch-{epoch_idx:010}-report"),
            explain_decision: "test".to_string(),
            explain_reasons: Vec::new(),
            provenance: None,
            sat_count: 4,
            used_sat_count: 4,
            rejected_sat_count: 0,
            hdop: Some(1.0),
            vdop: Some(1.0),
            gdop: Some(1.0),
            tdop: Some(1.0),
            stability_signature: format!(
                "navsig:v{}:epoch={epoch_idx}",
                NAV_OUTPUT_STABILITY_SIGNATURE_VERSION
            ),
            stability_signature_version: NAV_OUTPUT_STABILITY_SIGNATURE_VERSION,
        }
    }

    #[test]
    fn summarize_reported_prns_prefers_accepted_rows_and_filters_rejections() {
        let rows = vec![
            gps_row(31, "ambiguous", 8.0),
            gps_row(31, "accepted", 7.0),
            gps_row(12, "ambiguous", 6.0),
            gps_row(25, "deferred", 9.0),
            gps_row(32, "rejected", 10.0),
        ];

        let reported_prns = summarize_reported_prns(&rows);

        assert_eq!(reported_prns.len(), 2);
        assert_eq!(reported_prns[0].sat.prn, 31);
        assert_eq!(reported_prns[0].classification, "accepted");
        assert_eq!(reported_prns[1].sat.prn, 12);
        assert_eq!(reported_prns[1].classification, "candidate");
    }

    #[test]
    fn summarize_reported_prns_ignores_non_primary_candidates() {
        let mut alternative = gps_row(7, "accepted", 12.0);
        alternative.candidate_rank = 2;
        alternative.is_primary_candidate = false;
        let primary = gps_row(7, "ambiguous", 7.0);

        let reported_prns = summarize_reported_prns(&[alternative, primary.clone()]);

        assert_eq!(reported_prns.len(), 1);
        assert_eq!(reported_prns[0].sat.prn, primary.sat.prn);
        assert_eq!(reported_prns[0].classification, "candidate");
        assert_eq!(reported_prns[0].peak_mean_ratio, primary.peak_mean_ratio);
    }

    #[test]
    fn summarize_tracked_prns_orders_by_locked_epochs_then_total_epochs() {
        let tracks = vec![
            sample_track(12, &[("pull_in", false), ("tracking", true)]),
            sample_track(31, &[("tracking", true), ("tracking", true), ("lost", false)]),
            sample_track(25, &[]),
        ];

        let tracked_prns = summarize_tracked_prns(&tracks);

        assert_eq!(tracked_prns.len(), 2);
        assert_eq!(tracked_prns[0].sat.prn, 31);
        assert_eq!(tracked_prns[0].epoch_count, 3);
        assert_eq!(tracked_prns[0].locked_epoch_count, 2);
        assert_eq!(tracked_prns[0].latest_lock_state, "lost");
        assert_eq!(tracked_prns[1].sat.prn, 12);
    }

    #[test]
    fn summarize_navigation_attempts_counts_valid_stable_and_refused_epochs() {
        let solutions = vec![
            sample_solution(
                0,
                false,
                SolutionValidity::Invalid,
                Some(bijux_gnss_infra::api::core::NavRefusalClass::InconsistentObservations),
            ),
            sample_solution(1, true, SolutionValidity::Converging, None),
            sample_solution(2, true, SolutionValidity::Stable, None),
            sample_solution(
                3,
                false,
                SolutionValidity::Invalid,
                Some(bijux_gnss_infra::api::core::NavRefusalClass::InsufficientGeometry),
            ),
        ];

        let summary = summarize_navigation_attempts(&solutions);

        assert_eq!(summary.attempted_epochs, 4);
        assert_eq!(summary.valid_epochs, 2);
        assert_eq!(summary.stable_epochs, 1);
        assert_eq!(summary.refusal_counts.get("InconsistentObservations"), Some(&1usize));
        assert_eq!(summary.refusal_counts.get("InsufficientGeometry"), Some(&1usize));
    }

    #[test]
    fn summarize_position_attempts_preserves_refusal_and_geometry_counts() {
        let solutions = vec![
            sample_solution(
                10,
                false,
                SolutionValidity::Invalid,
                Some(bijux_gnss_infra::api::core::NavRefusalClass::InconsistentObservations),
            ),
            sample_solution(11, true, SolutionValidity::Stable, None),
        ];

        let attempts = summarize_position_attempts(&solutions);

        assert_eq!(attempts.len(), 2);
        assert_eq!(attempts[0].epoch_idx, 10);
        assert_eq!(attempts[0].valid, false);
        assert_eq!(attempts[0].sat_count, 4);
        assert_eq!(attempts[0].used_sat_count, 4);
        assert_eq!(attempts[0].rejected_sat_count, 0);
        assert_eq!(attempts[0].refusal_class.as_deref(), Some("InconsistentObservations"));
        assert_eq!(attempts[1].epoch_idx, 11);
        assert_eq!(attempts[1].validity, SolutionValidity::Stable);
        assert_eq!(attempts[1].refusal_class, None);
    }
}

#[derive(Debug, Serialize, Deserialize)]
struct TrackingReport {
    sats: Vec<SatId>,
    doppler_search: DopplerSearchSettings,
    front_end_metrics: bijux_gnss_infra::api::signal::IqFrontEndMetrics,
    signal_quality: RawIqSignalQualityReport,
    epochs: Vec<TrackingRow>,
}

#[derive(Debug, Serialize, Deserialize, Clone)]
struct TrackingRow {
    epoch_idx: u64,
    sample_index: u64,
    sat: SatId,
    carrier_hz: f64,
    #[serde(default)]
    carrier_phase_cycles: f64,
    code_rate_hz: f64,
    code_phase_samples: f64,
    prompt_i: f32,
    prompt_q: f32,
    #[serde(default)]
    early_i: f32,
    #[serde(default)]
    early_q: f32,
    #[serde(default)]
    late_i: f32,
    #[serde(default)]
    late_q: f32,
    lock: bool,
    cn0_dbhz: f64,
    pll_lock: bool,
    dll_lock: bool,
    fll_lock: bool,
    cycle_slip: bool,
    nav_bit_lock: bool,
    #[serde(default)]
    navigation_bit_sign: Option<i8>,
    dll_err: f32,
    pll_err: f32,
    fll_err: f32,
    #[serde(default)]
    anti_false_lock: bool,
    #[serde(default)]
    cycle_slip_reason: Option<String>,
    #[serde(default)]
    lock_state: String,
    #[serde(default)]
    lock_state_reason: Option<String>,
}

#[derive(Debug, Serialize)]
struct NavDecodeReport {
    sat: SatId,
    reference_week: Option<u32>,
    bit_start_ms: usize,
    bit_signs: Vec<i8>,
    aligned_subframes: Vec<bijux_gnss_infra::api::nav::GpsL1CaLnavSubframeAlignment>,
    decoded_subframes: Vec<bijux_gnss_infra::api::nav::GpsL1CaLnavDecodedSubframe>,
    ephemeris_rejections: Vec<bijux_gnss_infra::api::nav::GpsL1CaLnavEphemerisRejection>,
    parity_word_count: usize,
    parity_failed_words: usize,
    preamble_hits: usize,
    parity_pass_rate: f64,
    ephemerides: Vec<bijux_gnss_infra::api::nav::GpsEphemeris>,
}
