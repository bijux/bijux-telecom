#![allow(missing_docs)]

use std::env;
use std::fs;
use std::path::Path;

mod support;

use bijux_gnss_core::api::{Llh, Seconds};
use bijux_gnss_nav::api::{
    ecef_to_geodetic, elevation_azimuth_deg, sat_state_gps_l1ca_from_observation,
    NiellMappingFunction, SaastamoinenModel, TroposphereMeteorology, TroposphereModel,
};
use bijux_gnss_testkit::reference_data::troposphere_elevation::{
    build_public_troposphere_elevation_report, PublicTroposphereElevationReport,
    TroposphereElevationBucketReport, LOW_ELEVATION_CEILING_DEG,
};
use support::position_truth::{
    add_saastamoinen_delay_to_observations, four_satellite_position_scenario,
};
use support::public_spp_case::ab43_public_spp_case;

const REGENERATE_PUBLIC_TROPOSPHERE_FIXTURE_ENV: &str =
    "BIJUX_REGENERATE_PUBLIC_TROPOSPHERE_FIXTURE";
const RESIDUAL_REPORT_TOLERANCE_M: f64 = 1.0e-8;
const TIME_REPORT_TOLERANCE_S: f64 = 1.0e-9;

#[derive(Debug, Clone, Copy)]
struct ResidualSample {
    elevation_deg: f64,
    corrected_abs_residual_m: f64,
    uncorrected_abs_residual_m: f64,
}

#[test]
fn synthetic_low_elevation_residuals_improve_with_troposphere_correction() {
    let scenario = four_satellite_position_scenario(0.0);
    let biased_observations = add_saastamoinen_delay_to_observations(
        &scenario.observations,
        &scenario.ephemerides,
        scenario.truth_ecef_m,
        scenario.t_rx_s,
    );
    let samples = synthetic_residual_samples(&scenario, &biased_observations);
    let low_samples = samples
        .iter()
        .copied()
        .filter(|sample| {
            sample.elevation_deg > 0.0 && sample.elevation_deg < LOW_ELEVATION_CEILING_DEG
        })
        .collect::<Vec<_>>();
    let high_samples =
        samples.iter().copied().filter(|sample| sample.elevation_deg >= 45.0).collect::<Vec<_>>();

    assert!(!low_samples.is_empty(), "synthetic scenario should include low-elevation satellites");
    assert!(
        !high_samples.is_empty(),
        "synthetic scenario should include high-elevation satellites"
    );
    assert!(
        mean_abs_improvement_m(&samples) > 0.5,
        "synthetic troposphere correction should reduce residuals overall: {samples:?}"
    );
    assert!(
        mean_abs_improvement_m(&low_samples) > 1.0,
        "low-elevation synthetic satellites should see clear residual improvement: {low_samples:?}"
    );
    assert!(
        mean_abs_improvement_m(&low_samples) > mean_abs_improvement_m(&high_samples),
        "low-elevation residual improvement should exceed high-elevation improvement; low={low_samples:?} high={high_samples:?}"
    );
}

#[test]
fn public_saastamoinen_components_use_niell_mapping_and_meteorology() {
    let receiver = Llh { lat_deg: 45.0, lon_deg: 8.0, alt_m: 100.0 };
    let epoch = Seconds((28.0 - 1.0) * 86_400.0);
    let mapping = NiellMappingFunction::mapping_factors(receiver, 10.0, epoch);
    let standard_components = SaastamoinenModel::delay_components_m(receiver, 10.0, epoch);
    let humid_components = SaastamoinenModel::delay_components_with_meteorology_m(
        receiver,
        10.0,
        epoch,
        TroposphereMeteorology::new(980.0, 301.15, 0.9),
    );

    assert!((mapping.hydrostatic - 5.556_157_591).abs() < 1.0e-9);
    assert!((mapping.wet - 5.657_127_345).abs() < 1.0e-9);
    assert_eq!(standard_components.hydrostatic_mapping, mapping.hydrostatic);
    assert_eq!(standard_components.wet_mapping, mapping.wet);
    assert!(humid_components.slant_wet_m() > standard_components.slant_wet_m() + 1.0);
    assert!(humid_components.slant_total_m() > standard_components.slant_total_m());
}

#[test]
fn public_ab43_troposphere_correction_improves_low_elevation_residuals() {
    let case = ab43_public_spp_case();
    let report = build_public_troposphere_elevation_report(
        &case.observations,
        &case.navigation,
        &case.station_truth,
    )
    .expect("build AB43 public troposphere elevation report");
    if env::var_os(REGENERATE_PUBLIC_TROPOSPHERE_FIXTURE_ENV).is_some() {
        write_public_troposphere_fixture("public_troposphere_elevation/ab43.json", &report);
    }
    let expected = load_public_troposphere_fixture("public_troposphere_elevation/ab43.json");
    assert_troposphere_report_close(&report, &expected);

    let low_bucket = report
        .buckets
        .iter()
        .find(|bucket| bucket.bucket_name == "low")
        .expect("low-elevation bucket");
    let high_bucket = report
        .buckets
        .iter()
        .find(|bucket| bucket.bucket_name == "high")
        .expect("high-elevation bucket");

    assert!(report.compared_epoch_count > 0, "AB43 report should compare solved epochs");
    assert!(low_bucket.sample_count > 0, "AB43 report should include low-elevation samples");
    assert!(
        report.overall_mean_abs_improvement_m > 0.5,
        "AB43 overall residual improvement should remain meaningful: {report:?}"
    );
    assert!(
        low_bucket.mean_abs_improvement_m > 0.5,
        "AB43 low-elevation residuals should improve with troposphere correction: {low_bucket:?}"
    );
    assert!(
        low_bucket.mean_abs_improvement_m > high_bucket.mean_abs_improvement_m,
        "AB43 low-elevation improvement should exceed high-elevation improvement; low={low_bucket:?} high={high_bucket:?}"
    );
}

fn synthetic_residual_samples(
    scenario: &support::position_truth::SyntheticPositionScenario,
    observations: &[bijux_gnss_nav::api::PositionObservation],
) -> Vec<ResidualSample> {
    let (lat_deg, lon_deg, alt_m) =
        ecef_to_geodetic(scenario.truth_ecef_m.0, scenario.truth_ecef_m.1, scenario.truth_ecef_m.2);
    let receiver = Llh { lat_deg, lon_deg, alt_m };
    let model = SaastamoinenModel;

    scenario
        .observations
        .iter()
        .filter_map(|reference_observation| {
            let biased_observation = observations
                .iter()
                .find(|observation| observation.sat == reference_observation.sat)?;
            let ephemeris = scenario
                .ephemerides
                .iter()
                .find(|ephemeris| ephemeris.sat == reference_observation.sat)?;
            let state = sat_state_gps_l1ca_from_observation(
                ephemeris,
                scenario.t_rx_s,
                reference_observation.pseudorange_m,
                reference_observation.signal_timing,
            );
            let (_azimuth_deg, elevation_deg) = elevation_azimuth_deg(
                scenario.truth_ecef_m.0,
                scenario.truth_ecef_m.1,
                scenario.truth_ecef_m.2,
                state.x_m,
                state.y_m,
                state.z_m,
            );
            let modeled_delay_m = model.delay_m(receiver, elevation_deg, Seconds(scenario.t_rx_s));
            let injected_delay_m =
                biased_observation.pseudorange_m - reference_observation.pseudorange_m;

            Some(ResidualSample {
                elevation_deg,
                corrected_abs_residual_m: (injected_delay_m - modeled_delay_m).abs(),
                uncorrected_abs_residual_m: injected_delay_m.abs(),
            })
        })
        .collect()
}

fn mean_abs_improvement_m(samples: &[ResidualSample]) -> f64 {
    samples
        .iter()
        .map(|sample| sample.uncorrected_abs_residual_m - sample.corrected_abs_residual_m)
        .sum::<f64>()
        / samples.len() as f64
}

fn load_public_troposphere_fixture(fixture_file: &str) -> PublicTroposphereElevationReport {
    let contents = fs::read_to_string(public_troposphere_fixture_path(fixture_file))
        .expect("public troposphere elevation fixture");
    serde_json::from_str(&contents).expect("valid public troposphere elevation fixture")
}

fn write_public_troposphere_fixture(fixture_file: &str, report: &PublicTroposphereElevationReport) {
    let path = public_troposphere_fixture_path(fixture_file);
    let parent = path.parent().expect("fixture parent directory");
    fs::create_dir_all(parent).expect("create troposphere fixture directory");
    let contents =
        serde_json::to_string_pretty(report).expect("serialize public troposphere fixture");
    fs::write(path, format!("{contents}\n")).expect("write public troposphere fixture");
}

fn public_troposphere_fixture_path(fixture_file: &str) -> std::path::PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/data").join(fixture_file)
}

fn assert_troposphere_report_close(
    actual: &PublicTroposphereElevationReport,
    expected: &PublicTroposphereElevationReport,
) {
    assert_eq!(actual.marker_name, expected.marker_name);
    assert_eq!(actual.fixture_name, expected.fixture_name);
    assert_eq!(actual.compared_epoch_count, expected.compared_epoch_count);
    assert_eq!(actual.compared_sample_count, expected.compared_sample_count);
    assert_eq!(actual.low_elevation_sample_count, expected.low_elevation_sample_count);
    assert_close(
        "overall_mean_abs_improvement_m",
        actual.overall_mean_abs_improvement_m,
        expected.overall_mean_abs_improvement_m,
        RESIDUAL_REPORT_TOLERANCE_M,
    );
    assert_close(
        "overall_rms_improvement_m",
        actual.overall_rms_improvement_m,
        expected.overall_rms_improvement_m,
        RESIDUAL_REPORT_TOLERANCE_M,
    );
    assert_eq!(actual.buckets.len(), expected.buckets.len());
    for (index, (actual_bucket, expected_bucket)) in
        actual.buckets.iter().zip(&expected.buckets).enumerate()
    {
        assert_bucket_close(index, actual_bucket, expected_bucket);
    }
    assert_eq!(actual.epochs.len(), expected.epochs.len());
    for (index, (actual_epoch, expected_epoch)) in
        actual.epochs.iter().zip(&expected.epochs).enumerate()
    {
        assert_eq!(actual_epoch.gps_time.week, expected_epoch.gps_time.week);
        assert_close(
            &format!("epochs[{index}].gps_time.tow_s"),
            actual_epoch.gps_time.tow_s,
            expected_epoch.gps_time.tow_s,
            TIME_REPORT_TOLERANCE_S,
        );
        assert_eq!(actual_epoch.compared_satellite_count, expected_epoch.compared_satellite_count);
        assert_eq!(
            actual_epoch.low_elevation_satellite_count,
            expected_epoch.low_elevation_satellite_count
        );
        assert_optional_close(
            &format!("epochs[{index}].low_elevation_mean_abs_improvement_m"),
            actual_epoch.low_elevation_mean_abs_improvement_m,
            expected_epoch.low_elevation_mean_abs_improvement_m,
            RESIDUAL_REPORT_TOLERANCE_M,
        );
    }
}

fn assert_bucket_close(
    index: usize,
    actual: &TroposphereElevationBucketReport,
    expected: &TroposphereElevationBucketReport,
) {
    assert_eq!(actual.bucket_name, expected.bucket_name);
    assert_close(
        &format!("buckets[{index}].min_elevation_deg"),
        actual.min_elevation_deg,
        expected.min_elevation_deg,
        f64::EPSILON,
    );
    assert_optional_close(
        &format!("buckets[{index}].max_elevation_deg"),
        actual.max_elevation_deg,
        expected.max_elevation_deg,
        f64::EPSILON,
    );
    assert_eq!(actual.sample_count, expected.sample_count);
    for (label, actual_value, expected_value) in [
        (
            "corrected_mean_abs_residual_m",
            actual.corrected_mean_abs_residual_m,
            expected.corrected_mean_abs_residual_m,
        ),
        (
            "uncorrected_mean_abs_residual_m",
            actual.uncorrected_mean_abs_residual_m,
            expected.uncorrected_mean_abs_residual_m,
        ),
        (
            "corrected_rms_residual_m",
            actual.corrected_rms_residual_m,
            expected.corrected_rms_residual_m,
        ),
        (
            "uncorrected_rms_residual_m",
            actual.uncorrected_rms_residual_m,
            expected.uncorrected_rms_residual_m,
        ),
        ("mean_abs_improvement_m", actual.mean_abs_improvement_m, expected.mean_abs_improvement_m),
        ("rms_improvement_m", actual.rms_improvement_m, expected.rms_improvement_m),
    ] {
        assert_close(
            &format!("buckets[{index}].{label}"),
            actual_value,
            expected_value,
            RESIDUAL_REPORT_TOLERANCE_M,
        );
    }
}

fn assert_optional_close(label: &str, actual: Option<f64>, expected: Option<f64>, tolerance: f64) {
    match (actual, expected) {
        (Some(actual), Some(expected)) => assert_close(label, actual, expected, tolerance),
        (None, None) => {}
        _ => panic!("{label} presence mismatch: actual={actual:?} expected={expected:?}"),
    }
}

fn assert_close(label: &str, actual: f64, expected: f64, tolerance: f64) {
    let error = (actual - expected).abs();
    assert!(
        actual.is_finite() && expected.is_finite() && error <= tolerance,
        "{label} mismatch: actual={actual:.15} expected={expected:.15} error={error:.15} tolerance={tolerance:.15}"
    );
}
