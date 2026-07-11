#![allow(missing_docs)]

use bijux_gnss_core::api::{ArtifactPayloadValidate, Constellation, SatId, SigId, SignalBand};
use bijux_gnss_nav::api::{
    rtk_float_baseline_from_double_differences,
    rtk_float_baseline_from_double_differences_with_rover_prior, RtkFloatAmbiguityEstimate,
    RtkFloatBaselineSolution,
};
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
    assert!(solution.covariance_enu_m2[0][0].is_finite());
    assert!(solution.covariance_enu_m2[1][1].is_finite());
    assert!(solution.covariance_enu_m2[2][2].is_finite());
    assert!(solution.covariance_enu_m2[0][0] > 0.0);
    assert!(solution.covariance_enu_m2[1][1] > 0.0);
    assert!(solution.covariance_enu_m2[2][2] > 0.0);

    for ambiguity in &solution.float_ambiguities {
        let expected_cycles = scenario.rover_ambiguities_cycles[&ambiguity.sig.sat]
            - scenario.rover_ambiguities_cycles[&scenario.reference_sig.sat];
        assert!((ambiguity.float_cycles - expected_cycles).abs() < 0.05);
        assert!(ambiguity.variance_cycles2 >= 0.0);
    }
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
        float_ambiguities: vec![ambiguity],
    };

    let diagnostics = solution.validate_payload();

    assert!(diagnostics.iter().any(|event| event.code == "RTK_FLOAT_BASELINE_NUMERIC_INVALID"));
    assert!(diagnostics.iter().any(|event| event.code == "RTK_FLOAT_BASELINE_COVARIANCE_INVALID"));
    assert!(diagnostics.iter().any(|event| event.code == "RTK_FLOAT_AMBIGUITY_NUMERIC_INVALID"));
    assert!(diagnostics.iter().any(|event| event.code == "RTK_FLOAT_AMBIGUITY_VARIANCE_INVALID"));
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
fn rtk_float_baseline_solver_refuses_unsupported_signal_codes() {
    let scenario = clean_gps_l1_short_baseline_case();
    let mut double_differences = scenario.double_differences.clone();
    for observation in &mut double_differences {
        observation.sig.code = bijux_gnss_core::api::SignalCode::Py;
        observation.ref_sig.code = bijux_gnss_core::api::SignalCode::Py;
    }

    let solution = rtk_float_baseline_from_double_differences(
        &double_differences,
        scenario.base_ecef_m,
        &scenario.ephemerides,
        scenario.receive_gps_time.tow_s,
    );

    assert!(solution.is_none());
}
