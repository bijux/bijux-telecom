use super::*;
use crate::api::TrackingResult;
use bijux_gnss_core::api::{
    Chips, Constellation, Cycles, Epoch, Hertz, LockFlags, Meters, NavAssumptions, ObsMetadata,
    ObsSatellite, ObservationEpochDecision, ObservationStatus, ReceiverRole, ReceiverSampleTrace,
    SatId, SigId, SignalCode, TrackEpoch,
};
use bijux_gnss_signal::api::{carrier_wavelength_m, signal_registry};
use serde::Deserialize;

#[path = "tests/dual_frequency_readiness.rs"]
mod dual_frequency_readiness;
#[path = "tests/observation_diagnostics.rs"]
mod observation_diagnostics;
#[path = "tests/ppp_policy.rs"]
mod ppp_policy;
#[path = "tests/reference_position.rs"]
mod reference_position;

#[test]
fn validation_report_surfaces_innovation_consistency_anomalies() {
    let mut solution = fixture_solution(9, 1.0, 0.5, 4);
    solution.health.push(bijux_gnss_core::api::NavHealthEvent::InnovationConsistencyAnomaly {
        normalized_innovation_squared: 8.0,
        lower_bound: 0.1,
        upper_bound: 6.6,
        measurement_dimension: 1,
    });
    let reference = ValidationReferenceEpoch {
        epoch_idx: 9,
        t_rx_s: Some(9.0),
        latitude_deg: 0.0,
        longitude_deg: 0.0,
        altitude_m: 0.0,
        ecef_x_m: Some(0.0),
        ecef_y_m: Some(0.0),
        ecef_z_m: Some(0.0),
        vel_x_mps: None,
        vel_y_mps: None,
        vel_z_mps: None,
    };

    let report = build_validation_report(
        &[],
        &[],
        &[solution],
        &[reference],
        1.0,
        false,
        Vec::new(),
        ValidationSciencePolicy::default(),
    )
    .expect("validation report");

    assert!(report.consistency_warnings.iter().any(|warning| {
        warning.contains("innovation consistency anomalies") && warning.contains("peak NIS 8.000")
    }));
}

#[derive(Debug, Deserialize)]
struct ScienceFixtureCase {
    name: String,
    pdop: f64,
    rms_m: f64,
    used_sat_count: usize,
    lock_ratio: f64,
    expected_class: String,
}

fn fixture_solution(
    epoch_idx: u64,
    pdop: f64,
    rms_m: f64,
    used_sat_count: usize,
) -> NavSolutionEpoch {
    NavSolutionEpoch {
        epoch: bijux_gnss_core::api::Epoch { index: epoch_idx },
        t_rx_s: bijux_gnss_core::api::Seconds(epoch_idx as f64),
        source_time: bijux_gnss_core::api::ReceiverSampleTrace::from_sample_index(epoch_idx, 1.0),
        ecef_x_m: bijux_gnss_core::api::Meters(0.0),
        ecef_y_m: bijux_gnss_core::api::Meters(0.0),
        ecef_z_m: bijux_gnss_core::api::Meters(0.0),
        position_covariance_ecef_m2: None,
        latitude_deg: 0.0,
        longitude_deg: 0.0,
        altitude_m: bijux_gnss_core::api::Meters(0.0),
        clock_bias_s: bijux_gnss_core::api::Seconds(0.0),
        clock_bias_m: bijux_gnss_core::api::Meters(0.0),
        clock_drift_s_per_s: 0.0,
        pdop,
        pre_fit_residual_rms_m: Some(bijux_gnss_core::api::Meters(rms_m)),
        post_fit_residual_rms_m: Some(bijux_gnss_core::api::Meters(rms_m)),
        rms_m: bijux_gnss_core::api::Meters(rms_m),
        status: SolutionStatus::CodeOnly,
        quality: SolutionStatus::CodeOnly.quality_flag(),
        validity: bijux_gnss_core::api::SolutionValidity::Stable,
        valid: true,
        processing_ms: None,
        sigma_e_m: Some(bijux_gnss_core::api::Meters(1.0)),
        sigma_n_m: Some(bijux_gnss_core::api::Meters(1.0)),
        sigma_u_m: Some(bijux_gnss_core::api::Meters(1.0)),
        horizontal_error_ellipse_major_axis_m: Some(bijux_gnss_core::api::Meters(1.0)),
        horizontal_error_ellipse_minor_axis_m: Some(bijux_gnss_core::api::Meters(0.5)),
        horizontal_error_ellipse_azimuth_deg: Some(0.0),
        sigma_h_m: Some(bijux_gnss_core::api::Meters(1.0)),
        sigma_v_m: Some(bijux_gnss_core::api::Meters(1.0)),
        residuals: Vec::new(),
        constellation_residual_rms: Vec::new(),
        isb: Vec::new(),
        health: Vec::new(),
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
        model_version: bijux_gnss_core::api::NAV_SOLUTION_MODEL_VERSION,
        lifecycle_state: bijux_gnss_core::api::NavLifecycleState::CodeOnly,
        uncertainty_class: bijux_gnss_core::api::NavUncertaintyClass::Medium,
        assumptions: Some(NavAssumptions {
            time_system: "gps".to_string(),
            reference_frame: "ecef_wgs84".to_string(),
            clock_model: "receiver_clock_bias_drift_linear".to_string(),
            ephemeris_source: "broadcast_lnav".to_string(),
            frame_decode_mode: "lnav".to_string(),
            ephemeris_completeness: "sufficient".to_string(),
            ephemeris_count: used_sat_count,
        }),
        refusal_class: None,
        artifact_id: format!("nav-epoch-{epoch_idx:010}"),
        source_observation_epoch_id: format!("obs-epoch-{epoch_idx:010}"),
        explain_decision: "accepted".to_string(),
        explain_reasons: vec!["fixture".to_string()],
        provenance: None,
        sat_count: used_sat_count,
        used_sat_count,
        rejected_sat_count: 0,
        hdop: Some(pdop),
        vdop: Some(pdop),
        gdop: Some(pdop),
        tdop: Some(pdop / 2.0),
        stability_signature: format!("navsig:v2:fixture:{epoch_idx}"),
        stability_signature_version: bijux_gnss_core::api::NAV_OUTPUT_STABILITY_SIGNATURE_VERSION,
    }
}

fn with_integrity_support(mut solution: NavSolutionEpoch) -> NavSolutionEpoch {
    solution.integrity_hpl_m = Some(10.0);
    solution.integrity_vpl_m = Some(12.0);
    solution
}

fn dual_frequency_epoch(epoch_idx: u64, sats: Vec<ObsSatellite>) -> ObsEpoch {
    ObsEpoch {
        t_rx_s: bijux_gnss_core::api::Seconds(epoch_idx as f64),
        source_time: ReceiverSampleTrace::from_sample_index(epoch_idx, 1.0),
        gps_week: None,
        tow_s: None,
        epoch_idx,
        discontinuity: false,
        valid: true,
        processing_ms: None,
        role: ReceiverRole::Rover,
        sats,
        decision: ObservationEpochDecision::Accepted,
        decision_reason: Some("accepted_observables_present".to_string()),
        manifest: None,
    }
}

fn dual_frequency_satellite(
    band: SignalBand,
    code: SignalCode,
    code_lock: bool,
    carrier_lock: bool,
) -> ObsSatellite {
    dual_frequency_satellite_for_constellation(
        Constellation::Gps,
        band,
        code,
        code_lock,
        carrier_lock,
    )
}

fn dual_frequency_satellite_for_constellation(
    constellation: Constellation,
    band: SignalBand,
    code: SignalCode,
    code_lock: bool,
    carrier_lock: bool,
) -> ObsSatellite {
    ObsSatellite {
        signal_id: SigId { sat: SatId { constellation, prn: 12 }, band, code },
        pseudorange_m: Meters(22_000_000.0),
        pseudorange_var_m2: 1.0,
        carrier_phase_cycles: Cycles(200.0),
        carrier_phase_var_cycles2: 0.1,
        doppler_hz: Hertz(15.0),
        doppler_var_hz2: 1.0,
        cn0_dbhz: 45.0,
        lock_flags: LockFlags { code_lock, carrier_lock, bit_lock: true, cycle_slip: false },
        multipath_suspect: false,
        observation_status: ObservationStatus::Accepted,
        observation_reject_reasons: Vec::new(),
        elevation_deg: Some(45.0),
        azimuth_deg: Some(120.0),
        weight: Some(1.0),
        timing: None,
        error_model: None,
        metadata: ObsMetadata {
            tracking_mode: "scalar".to_string(),
            integration_ms: 1,
            lock_quality: 1.0,
            smoothing_window: 0,
            smoothing_age: 0,
            smoothing_resets: 0,
            signal: signal_registry(constellation, band, code)
                .expect("dual-frequency signal must exist")
                .spec,
            doppler_model: "tracked_carrier_hz_minus_intermediate_freq".to_string(),
            observation_lock_state: "locked".to_string(),
            ..ObsMetadata::default()
        },
    }
}

fn dual_frequency_satellite_with_phase(
    band: SignalBand,
    code: SignalCode,
    phase_m: f64,
) -> ObsSatellite {
    let mut satellite = dual_frequency_satellite(band, code, true, true);
    let wavelength_m = carrier_wavelength_m(satellite.metadata.signal.carrier_hz).0;
    satellite.carrier_phase_cycles = Cycles(phase_m / wavelength_m);
    satellite
}

fn carrier_smoothed_code_satellite(smoothing_age: u32, cycle_slip: bool) -> ObsSatellite {
    let mut satellite = dual_frequency_satellite(SignalBand::L1, SignalCode::Ca, true, true);
    satellite.lock_flags.cycle_slip = cycle_slip;
    satellite.metadata.smoothing_window = 8;
    satellite.metadata.smoothing_age = smoothing_age;
    satellite
}

fn fixture_track(epoch_idx: u64, lock_ratio: f64) -> TrackingResult {
    let sat = SatId { constellation: Constellation::Gps, prn: 1 };
    let total = 8u64;
    let locked = ((total as f64) * lock_ratio).round() as u64;
    let mut epochs = Vec::new();
    for idx in 0..total {
        epochs.push(TrackEpoch {
            epoch: Epoch { index: epoch_idx },
            sample_index: idx,
            sat,
            signal_band: bijux_gnss_core::api::SignalBand::L1,
            glonass_frequency_channel: None,
            prompt_i: 1.0,
            prompt_q: 0.0,
            carrier_hz: Hertz(0.0),
            code_rate_hz: Hertz(1_023_000.0),
            code_phase_samples: Chips(0.0),
            lock: idx < locked,
            cn0_dbhz: 35.0,
            pll_lock: idx < locked,
            dll_lock: idx < locked,
            fll_lock: idx < locked,
            cycle_slip: false,
            nav_bit_lock: false,
            dll_err: 0.0,
            pll_err: 0.0,
            fll_err: 0.0,
            anti_false_lock: false,
            cycle_slip_reason: None,
            lock_state: "tracking".to_string(),
            lock_state_reason: None,
            processing_ms: None,
            ..TrackEpoch::default()
        });
    }
    TrackingResult {
        sat,
        carrier_hz: 0.0,
        code_phase_samples: 0.0,
        acquisition_hypothesis: "accepted".to_string(),
        acquisition_score: 1.0,
        acquisition_code_phase_samples: 0,
        acquisition_carrier_hz: 0.0,
        acq_to_track_state: "tracking".to_string(),
        epochs,
        transitions: Vec::new(),
    }
}

#[test]
fn science_fixture_integrity_classes_are_deterministic() {
    let fixture_path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../bijux-gnss-receiver/tests/data/validation/science_integrity_classification.json");
    let cases: Vec<ScienceFixtureCase> =
        serde_json::from_str(&std::fs::read_to_string(fixture_path).expect("fixture"))
            .expect("cases");
    for (idx, case) in cases.iter().enumerate() {
        let solution = with_integrity_support(fixture_solution(
            idx as u64,
            case.pdop,
            case.rms_m,
            case.used_sat_count,
        ));
        let track = fixture_track(idx as u64, case.lock_ratio);
        let report = build_validation_report(
            &[track],
            &[],
            &[solution],
            &[],
            1.0,
            false,
            Vec::new(),
            ValidationSciencePolicy::default(),
        )
        .expect("report");
        let class = report.integrity.first().expect("integrity row");
        let expected = match case.expected_class.as_str() {
            "missing_integrity_evidence" => NavIntegrityClass::IntegrityEvidenceMissing,
            "weak_geometry" => NavIntegrityClass::WeakGeometry,
            "suspicious_residuals" => NavIntegrityClass::SuspiciousResiduals,
            "unstable_lock" => NavIntegrityClass::UnstableLock,
            other => panic!("unsupported expected class: {other}"),
        };
        assert_eq!(class.class, expected, "case {}", case.name);
    }
}

#[test]
fn validation_policy_marks_high_gdop_as_weak_geometry() {
    let solution = with_integrity_support(fixture_solution(0, 2.0, 1.0, 4));
    let report = build_validation_report(
        &[fixture_track(0, 1.0)],
        &[],
        &[NavSolutionEpoch { gdop: Some(20.0), ..solution }],
        &[],
        1.0,
        false,
        Vec::new(),
        ValidationSciencePolicy::default(),
    )
    .expect("report");

    assert_eq!(
        report.integrity.first().expect("integrity row").class,
        NavIntegrityClass::WeakGeometry
    );
}

#[test]
fn validation_policy_requires_integrity_evidence_for_nominal_classification() {
    let solution = fixture_solution(0, 2.0, 1.0, 4);
    let report = build_validation_report(
        &[fixture_track(0, 1.0)],
        &[],
        &[solution],
        &[],
        1.0,
        false,
        Vec::new(),
        ValidationSciencePolicy::default(),
    )
    .expect("report");

    let integrity = report.integrity.first().expect("integrity row");
    assert_eq!(integrity.class, NavIntegrityClass::IntegrityEvidenceMissing);
    assert_eq!(integrity.reasons, vec!["missing_integrity_evidence".to_string()]);
}

#[test]
fn validation_policy_allows_nominal_when_integrity_evidence_is_present() {
    let solution = with_integrity_support(fixture_solution(0, 2.0, 1.0, 4));
    let report = build_validation_report(
        &[fixture_track(0, 1.0)],
        &[],
        &[solution],
        &[],
        1.0,
        false,
        Vec::new(),
        ValidationSciencePolicy::default(),
    )
    .expect("report");

    let integrity = report.integrity.first().expect("integrity row");
    assert_eq!(integrity.class, NavIntegrityClass::Nominal);
    assert!(integrity.reasons.is_empty());
}
