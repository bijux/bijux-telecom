#![allow(missing_docs)]

use bijux_gnss_core::api::{
    ArtifactPayloadValidate, Constellation, SatId, SigId, SignalBand, SignalCode,
};
use bijux_gnss_nav::api::{
    rtk_float_baseline_from_double_differences,
    rtk_float_baseline_from_double_differences_with_rover_prior,
    rtk_switch_double_difference_reference, rtk_transform_float_baseline_reference,
    RtkDoubleDifferenceObservation, RtkFloatAmbiguityEstimate, RtkFloatBaselineSolution,
    RtkSingleDifferenceCovarianceEvidence,
};
use bijux_gnss_signal::api::signal_id_wavelength_m;
use bijux_gnss_testkit::rtk_baseline::clean_gps_l1_short_baseline_case;

fn enu_to_ecef(base_ecef_m: [f64; 3], enu_m: [f64; 3]) -> [f64; 3] {
    let (lat_deg, lon_deg, _alt_m) =
        bijux_gnss_nav::api::ecef_to_geodetic(base_ecef_m[0], base_ecef_m[1], base_ecef_m[2]);
    let (sin_lat, cos_lat) = lat_deg.to_radians().sin_cos();
    let (sin_lon, cos_lon) = lon_deg.to_radians().sin_cos();
    let east_m = enu_m[0];
    let north_m = enu_m[1];
    let up_m = enu_m[2];
    let dx = -sin_lon * east_m - sin_lat * cos_lon * north_m + cos_lat * cos_lon * up_m;
    let dy = cos_lon * east_m - sin_lat * sin_lon * north_m + cos_lat * sin_lon * up_m;
    let dz = cos_lat * north_m + sin_lat * up_m;
    [base_ecef_m[0] + dx, base_ecef_m[1] + dy, base_ecef_m[2] + dz]
}

fn retarget_double_differences_signal(
    observations: &[RtkDoubleDifferenceObservation],
    band: SignalBand,
    code: SignalCode,
) -> Vec<RtkDoubleDifferenceObservation> {
    observations
        .iter()
        .cloned()
        .map(|mut observation| {
            let source_wavelength_m =
                signal_id_wavelength_m(observation.sig).expect("fixture signal wavelength").0;
            observation.sig.band = band;
            observation.sig.code = code;
            observation.ref_sig.band = band;
            observation.ref_sig.code = code;
            let target_wavelength_m =
                signal_id_wavelength_m(observation.sig).expect("target signal wavelength").0;
            let phase_scale = source_wavelength_m / target_wavelength_m;
            observation.phase_cycles *= phase_scale;
            observation.phase_variance_cycles2 *= phase_scale * phase_scale;
            scale_phase_covariance(&mut observation.covariance_evidence.signal, phase_scale);
            scale_phase_covariance(&mut observation.covariance_evidence.reference, phase_scale);
            observation
        })
        .collect()
}

fn scale_phase_covariance(evidence: &mut RtkSingleDifferenceCovarianceEvidence, phase_scale: f64) {
    let variance_scale = phase_scale * phase_scale;
    evidence.rover.phase_cycles2 *= variance_scale;
    evidence.base.phase_cycles2 *= variance_scale;
    evidence.rover_base_phase_covariance_cycles2 *= variance_scale;
    evidence.shared_phase_covariance_cycles2 *= variance_scale;
}

fn assert_solution_matches_truth(solution: &RtkFloatBaselineSolution, truth_enu_m: [f64; 3]) {
    assert!((solution.enu_m[0] - truth_enu_m[0]).abs() < 0.05, "east mismatch");
    assert!((solution.enu_m[1] - truth_enu_m[1]).abs() < 0.05, "north mismatch");
    assert!((solution.enu_m[2] - truth_enu_m[2]).abs() < 0.10, "up mismatch");
}

fn assert_float_solutions_close(
    actual: &RtkFloatBaselineSolution,
    expected: &RtkFloatBaselineSolution,
    tolerance: f64,
) {
    for axis in 0..3 {
        assert!((actual.enu_m[axis] - expected.enu_m[axis]).abs() < tolerance);
    }
    assert_eq!(actual.float_ambiguities.len(), expected.float_ambiguities.len());
    for (actual_ambiguity, expected_ambiguity) in
        actual.float_ambiguities.iter().zip(expected.float_ambiguities.iter())
    {
        assert_eq!(actual_ambiguity.sig, expected_ambiguity.sig);
        assert_eq!(actual_ambiguity.ref_sig, expected_ambiguity.ref_sig);
        assert!(
            (actual_ambiguity.float_cycles - expected_ambiguity.float_cycles).abs() < tolerance,
            "actual={} expected={}",
            actual_ambiguity.float_cycles,
            expected_ambiguity.float_cycles
        );
    }
}

#[test]
fn rtk_float_baseline_solver_recovers_truth_and_ambiguities() {
    let scenario = clean_gps_l1_short_baseline_case();

    let solution = rtk_float_baseline_from_double_differences(
        &scenario.double_differences,
        scenario.base_ecef_m,
        &scenario.ephemerides,
        scenario.receive_gps_time.tow_s,
    )
    .expect("float baseline");

    assert!((solution.enu_m[0] - scenario.truth_enu_m[0]).abs() < 0.05, "east mismatch");
    assert!((solution.enu_m[1] - scenario.truth_enu_m[1]).abs() < 0.05, "north mismatch");
    assert!((solution.enu_m[2] - scenario.truth_enu_m[2]).abs() < 0.10, "up mismatch");
    assert_eq!(solution.float_ambiguities.len(), scenario.double_differences.len());
    assert_eq!(solution.ambiguity_covariance_cycles2.len(), scenario.double_differences.len());
    assert_eq!(solution.enu_ambiguity_covariance_m_cycles.len(), 3);
    assert!(solution.covariance_enu_m2[0][0].is_finite());
    assert!(solution.covariance_enu_m2[1][1].is_finite());
    assert!(solution.covariance_enu_m2[2][2].is_finite());
    assert!(solution.covariance_enu_m2[0][0] > 0.0);
    assert!(solution.covariance_enu_m2[1][1] > 0.0);
    assert!(solution.covariance_enu_m2[2][2] > 0.0);
    for row in &solution.ambiguity_covariance_cycles2 {
        assert_eq!(row.len(), scenario.double_differences.len());
        assert!(row.iter().all(|value| value.is_finite()));
    }
    for row in &solution.enu_ambiguity_covariance_m_cycles {
        assert_eq!(row.len(), scenario.double_differences.len());
        assert!(row.iter().all(|value| value.is_finite()));
    }

    for ambiguity in &solution.float_ambiguities {
        let expected_cycles = scenario.rover_ambiguities_cycles[&ambiguity.sig.sat]
            - scenario.rover_ambiguities_cycles[&scenario.reference_sig.sat];
        assert!((ambiguity.float_cycles - expected_cycles).abs() < 0.05);
        assert!(ambiguity.variance_cycles2 >= 0.0);
    }
}

#[test]
fn rtk_float_baseline_reference_switch_avoids_phase_jump() {
    let scenario = clean_gps_l1_short_baseline_case();
    let original_solution = rtk_float_baseline_from_double_differences(
        &scenario.double_differences,
        scenario.base_ecef_m,
        &scenario.ephemerides,
        scenario.receive_gps_time.tow_s,
    )
    .expect("original float baseline");
    let new_reference = scenario.double_differences[0].sig;
    let transformed_solution =
        rtk_transform_float_baseline_reference(&original_solution, new_reference)
            .expect("transformed solution");
    let switched_double_differences =
        rtk_switch_double_difference_reference(&scenario.double_differences, new_reference)
            .expect("switched double differences");
    let switched_solution = rtk_float_baseline_from_double_differences(
        &switched_double_differences,
        scenario.base_ecef_m,
        &scenario.ephemerides,
        scenario.receive_gps_time.tow_s,
    )
    .expect("switched float baseline");

    assert_float_solutions_close(&switched_solution, &transformed_solution, 1.0e-6);
    assert_solution_matches_truth(&switched_solution, scenario.truth_enu_m);
}

#[test]
fn rtk_float_baseline_solver_uses_differenced_covariance_coupling() {
    let scenario = clean_gps_l1_short_baseline_case();
    let independent_solution = rtk_float_baseline_from_double_differences(
        &scenario.double_differences,
        scenario.base_ecef_m,
        &scenario.ephemerides,
        scenario.receive_gps_time.tow_s,
    )
    .expect("independent covariance solution");

    let mut correlated_observations = scenario.double_differences.clone();
    for observation in &mut correlated_observations {
        observation.covariance_evidence.signal.shared_phase_covariance_cycles2 += 2.0e-5;
        observation.covariance_evidence.reference.shared_phase_covariance_cycles2 += 2.0e-5;
    }
    let correlated_solution = rtk_float_baseline_from_double_differences(
        &correlated_observations,
        scenario.base_ecef_m,
        &scenario.ephemerides,
        scenario.receive_gps_time.tow_s,
    )
    .expect("correlated covariance solution");

    let independent_ambiguity_trace: f64 = independent_solution
        .ambiguity_covariance_cycles2
        .iter()
        .enumerate()
        .map(|(index, row)| row[index])
        .sum();
    let correlated_ambiguity_trace: f64 = correlated_solution
        .ambiguity_covariance_cycles2
        .iter()
        .enumerate()
        .map(|(index, row)| row[index])
        .sum();
    assert!((independent_ambiguity_trace - correlated_ambiguity_trace).abs() > 1.0e-8);
    assert_solution_matches_truth(&correlated_solution, scenario.truth_enu_m);
}

#[test]
fn rtk_float_baseline_solver_accepts_explicit_rover_prior() {
    let scenario = clean_gps_l1_short_baseline_case();
    let rover_prior_ecef_m = enu_to_ecef(
        scenario.base_ecef_m,
        [
            scenario.truth_enu_m[0] + 1.5,
            scenario.truth_enu_m[1] - 1.0,
            scenario.truth_enu_m[2] + 0.5,
        ],
    );

    let solution = rtk_float_baseline_from_double_differences_with_rover_prior(
        &scenario.double_differences,
        scenario.base_ecef_m,
        rover_prior_ecef_m,
        &scenario.ephemerides,
        scenario.receive_gps_time.tow_s,
    )
    .expect("float baseline with prior");

    assert!((solution.enu_m[0] - scenario.truth_enu_m[0]).abs() < 0.05);
    assert!((solution.enu_m[1] - scenario.truth_enu_m[1]).abs() < 0.05);
    assert!((solution.enu_m[2] - scenario.truth_enu_m[2]).abs() < 0.10);
}

#[test]
fn rtk_float_baseline_solver_supports_gps_l2c_wavelengths() {
    let scenario = clean_gps_l1_short_baseline_case();
    let double_differences = retarget_double_differences_signal(
        &scenario.double_differences,
        SignalBand::L2,
        SignalCode::L2C,
    );

    let solution = rtk_float_baseline_from_double_differences(
        &double_differences,
        scenario.base_ecef_m,
        &scenario.ephemerides,
        scenario.receive_gps_time.tow_s,
    )
    .expect("float baseline");

    assert_solution_matches_truth(&solution, scenario.truth_enu_m);
}

#[test]
fn rtk_float_baseline_solver_supports_gps_l5_wavelengths() {
    let scenario = clean_gps_l1_short_baseline_case();
    let double_differences = retarget_double_differences_signal(
        &scenario.double_differences,
        SignalBand::L5,
        SignalCode::L5I,
    );

    let solution = rtk_float_baseline_from_double_differences(
        &double_differences,
        scenario.base_ecef_m,
        &scenario.ephemerides,
        scenario.receive_gps_time.tow_s,
    )
    .expect("float baseline");

    assert_solution_matches_truth(&solution, scenario.truth_enu_m);
}

#[test]
fn rtk_float_baseline_artifact_validation_rejects_invalid_values() {
    let ambiguity = RtkFloatAmbiguityEstimate {
        sig: SigId {
            sat: SatId { constellation: Constellation::Gps, prn: 7 },
            band: SignalBand::L1,
            code: bijux_gnss_core::api::SignalCode::Ca,
        },
        ref_sig: SigId {
            sat: SatId { constellation: Constellation::Gps, prn: 3 },
            band: SignalBand::L1,
            code: bijux_gnss_core::api::SignalCode::Ca,
        },
        float_cycles: f64::NAN,
        variance_cycles2: -1.0,
    };
    let solution = RtkFloatBaselineSolution {
        enu_m: [0.0, f64::INFINITY, 0.0],
        covariance_enu_m2: [[0.0, 0.0, 0.0], [0.0, f64::NAN, 0.0], [0.0, 0.0, 0.0]],
        enu_ambiguity_covariance_m_cycles: vec![vec![f64::INFINITY], vec![0.0], Vec::new()],
        float_ambiguities: vec![ambiguity],
        ambiguity_covariance_cycles2: vec![vec![f64::NAN]],
    };

    let diagnostics = solution.validate_payload();

    assert!(diagnostics.iter().any(|event| event.code == "RTK_FLOAT_BASELINE_NUMERIC_INVALID"));
    assert!(diagnostics.iter().any(|event| event.code == "RTK_FLOAT_BASELINE_COVARIANCE_INVALID"));
    assert!(diagnostics.iter().any(|event| {
        event.code == "RTK_FLOAT_BASELINE_CROSS_COVARIANCE_SHAPE_INVALID"
            || event.code == "RTK_FLOAT_BASELINE_CROSS_COVARIANCE_NUMERIC_INVALID"
    }));
    assert!(diagnostics.iter().any(|event| event.code == "RTK_FLOAT_AMBIGUITY_NUMERIC_INVALID"));
    assert!(diagnostics.iter().any(|event| event.code == "RTK_FLOAT_AMBIGUITY_VARIANCE_INVALID"));
    assert!(diagnostics.iter().any(|event| {
        event.code == "RTK_FLOAT_AMBIGUITY_COVARIANCE_NUMERIC_INVALID"
            || event.code == "RTK_FLOAT_AMBIGUITY_COVARIANCE_SHAPE_INVALID"
    }));
}

#[test]
fn rtk_float_baseline_solver_refuses_underdetermined_inputs() {
    let scenario = clean_gps_l1_short_baseline_case();

    let solution = rtk_float_baseline_from_double_differences(
        &scenario.double_differences[..2],
        scenario.base_ecef_m,
        &scenario.ephemerides,
        scenario.receive_gps_time.tow_s,
    );

    assert!(solution.is_none());
}

#[test]
fn rtk_float_baseline_solver_refuses_unregistered_signal_identities() {
    let scenario = clean_gps_l1_short_baseline_case();
    let mut double_differences = scenario.double_differences.clone();
    for observation in &mut double_differences {
        observation.sig.band = SignalBand::L2;
        observation.sig.code = SignalCode::Unknown;
        observation.ref_sig.band = SignalBand::L2;
        observation.ref_sig.code = SignalCode::Unknown;
    }

    let solution = rtk_float_baseline_from_double_differences(
        &double_differences,
        scenario.base_ecef_m,
        &scenario.ephemerides,
        scenario.receive_gps_time.tow_s,
    );

    assert!(solution.is_none());
}
