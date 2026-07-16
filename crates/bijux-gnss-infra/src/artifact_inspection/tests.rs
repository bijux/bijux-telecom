use super::{artifact_explain, artifact_validate};
use bijux_gnss_receiver::api::core::{
    AcqHypothesis, AcqResult, AcqResultV1, ArtifactHeaderV1, Constellation, Epoch, Hertz, Meters,
    NavLifecycleState, NavSolutionEpoch, NavSolutionEpochV1, NavUncertaintyClass,
    ReceiverSampleTrace, SatId, Seconds, SignalBand, SignalCode, SolutionStatus, SolutionValidity,
    TrackEpoch, TrackEpochV1, NAV_OUTPUT_STABILITY_SIGNATURE_VERSION, NAV_SOLUTION_MODEL_VERSION,
};
use std::fs;
use tempfile::tempdir;

fn header() -> ArtifactHeaderV1 {
    ArtifactHeaderV1 {
        schema_version: 1,
        producer: "bijux-gnss-infra-test".to_string(),
        producer_version: "0.1.0".to_string(),
        created_at_unix_ms: 1,
        git_sha: "unknown".to_string(),
        config_hash: "fixture".to_string(),
        dataset_id: None,
        toolchain: "rustc test".to_string(),
        features: Vec::new(),
        deterministic: true,
        git_dirty: false,
    }
}

#[test]
fn artifact_validate_accepts_acquisition_trace() {
    let dir = tempdir().expect("tempdir");
    let path = dir.path().join("acq.jsonl");
    let wrapped = AcqResultV1 {
        header: header(),
        payload: AcqResult {
            sat: SatId { constellation: Constellation::Gps, prn: 3 },
            signal_band: SignalBand::L1,
            signal_code: SignalCode::Unknown,
            glonass_frequency_channel: None,
            source_time: ReceiverSampleTrace::from_sample_index(4_092, 4_092_000.0),
            candidate_rank: 1,
            is_primary_candidate: true,
            doppler_hz: Hertz(500.0),
            doppler_rate_hz_per_s: 0.0,
            carrier_hz: Hertz(500.0),
            code_phase_samples: 17,
            peak: 10.0,
            second_peak: 2.0,
            mean: 1.0,
            peak_mean_ratio: 10.0,
            peak_second_ratio: 5.0,
            cn0_proxy: 42.0,
            score: 0.9,
            hypothesis: AcqHypothesis::Accepted,
            assumptions: None,
            evidence: Vec::new(),
            threshold_provenance: None,
            explain_selection_reason: Some("accepted_peak".to_string()),
            doppler_refinement: None,
            code_phase_refinement: None,
            signal_delay_alignment: None,
            uncertainty: None,
        },
    };
    fs::write(&path, serde_json::to_string(&wrapped).expect("serialize")).expect("write");

    let result = artifact_validate(&path, None, true).expect("validate artifact");
    assert_eq!(result.kind, "acq");
    assert!(result.diagnostics.is_empty(), "diagnostics={:?}", result.diagnostics);
}

#[test]
fn artifact_validate_rejects_non_monotonic_track_sample_index() {
    let dir = tempdir().expect("tempdir");
    let path = dir.path().join("track.jsonl");
    let sat = SatId { constellation: Constellation::Gps, prn: 7 };
    let rows = [32_u64, 16_u64]
        .into_iter()
        .map(|sample_index| TrackEpochV1 {
            header: header(),
            payload: TrackEpoch {
                epoch: Epoch { index: sample_index / 16 },
                sample_index,
                source_time: ReceiverSampleTrace::from_sample_index(sample_index, 16_000.0),
                sat,
                signal_band: SignalBand::L1,
                signal_code: SignalCode::Unknown,
                glonass_frequency_channel: None,
                prompt_i: 1.0,
                prompt_q: 0.0,
                early_i: 0.0,
                early_q: 0.0,
                late_i: 0.0,
                late_q: 0.0,
                carrier_hz: Hertz(500.0),
                carrier_phase_cycles: bijux_gnss_receiver::api::core::Cycles(0.0),
                code_rate_hz: Hertz(1_023_000.0),
                code_phase_samples: bijux_gnss_receiver::api::core::Chips(0.0),
                lock: true,
                cn0_dbhz: 45.0,
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
                lock_state_reason: None,
                channel_id: None,
                channel_uid: String::new(),
                tracking_provenance: String::new(),
                tracking_assumptions: None,
                signal_delay_alignment: None,
                transmit_time: None,
                tracking_uncertainty: None,
                processing_ms: None,
            },
        })
        .map(|wrapped| serde_json::to_string(&wrapped).expect("serialize"))
        .collect::<Vec<_>>();
    fs::write(&path, rows.join("\n")).expect("write");

    let result = artifact_validate(&path, None, true).expect("validate artifact");
    assert!(result.diagnostics.iter().any(|event| event.code == "GNSS_TRACK_SAMPLE_NON_MONOTONIC"));
}

#[test]
fn artifact_explain_reads_nav_trace_artifact() {
    let dir = tempdir().expect("tempdir");
    let path = dir.path().join("pvt.jsonl");
    let wrapped = NavSolutionEpochV1 {
        header: header(),
        payload: NavSolutionEpoch {
            epoch: Epoch { index: 9 },
            t_rx_s: Seconds(0.25),
            source_time: ReceiverSampleTrace::from_sample_index(1_023_000, 4_092_000.0),
            ecef_x_m: Meters(1.0),
            ecef_y_m: Meters(2.0),
            ecef_z_m: Meters(3.0),
            position_covariance_ecef_m2: None,
            latitude_deg: 60.0,
            longitude_deg: 18.0,
            altitude_m: Meters(4.0),
            clock_bias_s: Seconds(0.0),
            clock_bias_m: Meters(0.0),
            clock_drift_s_per_s: 0.0,
            pdop: 1.0,
            pre_fit_residual_rms_m: None,
            post_fit_residual_rms_m: None,
            rms_m: Meters(2.0),
            status: SolutionStatus::CodeOnly,
            quality: SolutionStatus::CodeOnly.quality_flag(),
            validity: SolutionValidity::Stable,
            valid: true,
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
            refusal_class: None,
            artifact_id: "nav-epoch-0000000009-test".to_string(),
            source_observation_epoch_id: "epoch-0000000009-sample-000001023000".to_string(),
            explain_decision: "accepted".to_string(),
            explain_reasons: vec!["navigation_solution_usable".to_string()],
            provenance: None,
            sat_count: 4,
            used_sat_count: 4,
            rejected_sat_count: 0,
            hdop: None,
            vdop: None,
            gdop: None,
            tdop: None,
            stability_signature: "navsig:v2:test".to_string(),
            stability_signature_version: NAV_OUTPUT_STABILITY_SIGNATURE_VERSION,
        },
    };
    fs::write(&path, serde_json::to_string(&wrapped).expect("serialize")).expect("write");

    let result = artifact_explain(&path).expect("explain artifact");
    assert_eq!(result.kind, "pvt");
    assert_eq!(result.entries, 1);
    assert_eq!(result.diagnostics_error, 0);
}
