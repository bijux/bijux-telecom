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
use bijux_gnss_testkit::public_troposphere::{
    build_public_troposphere_elevation_report, PublicTroposphereElevationReport,
    LOW_ELEVATION_CEILING_DEG,
};
use support::position_truth::{
    add_saastamoinen_delay_to_observations, four_satellite_position_scenario,
};
use support::public_spp_case::ab43_public_spp_case;

const REGENERATE_PUBLIC_TROPOSPHERE_FIXTURE_ENV: &str =
    "BIJUX_REGENERATE_PUBLIC_TROPOSPHERE_FIXTURE";

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
    let expected = load_public_troposphere_fixture_text("public_troposphere_elevation/ab43.json");
    let actual = format!(
        "{}\n",
        serde_json::to_string_pretty(&report).expect("serialize troposphere report")
    );

    assert_eq!(
        actual,
        expected,
        "AB43 troposphere elevation report drifted; set {REGENERATE_PUBLIC_TROPOSPHERE_FIXTURE_ENV}=1 to refresh the fixture when the new behavior is intended",
    );

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

fn load_public_troposphere_fixture_text(fixture_file: &str) -> String {
    fs::read_to_string(public_troposphere_fixture_path(fixture_file))
        .expect("public troposphere elevation fixture")
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
