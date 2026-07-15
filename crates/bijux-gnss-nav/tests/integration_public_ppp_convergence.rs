#![allow(missing_docs)]

use std::env;
use std::fs;
use std::path::Path;

mod support;

use bijux_gnss_nav::api::{BroadcastProductsProvider, PppConfig, PppFilter, PppTroposphereSource};
use bijux_gnss_testkit::public_ppp::{
    build_public_ppp_convergence_report, PublicPppConvergenceReport,
};
use support::public_spp_case::ab43_public_spp_case;

const REGENERATE_PUBLIC_PPP_CONVERGENCE_FIXTURE_ENV: &str =
    "BIJUX_REGENERATE_PUBLIC_PPP_CONVERGENCE_FIXTURE";

#[test]
fn public_ab43_ppp_run_reports_convergence_metrics() {
    let case = ab43_public_spp_case();
    let report = build_public_ppp_convergence_report(
        &case.observations,
        &case.navigation,
        &case.station_truth,
        public_ab43_ppp_config(),
    )
    .expect("build AB43 public PPP convergence report");
    if env::var_os(REGENERATE_PUBLIC_PPP_CONVERGENCE_FIXTURE_ENV).is_some() {
        write_convergence_fixture("public_ppp_convergence/ab43.json", &report);
    }
    let expected = load_convergence_fixture_text("public_ppp_convergence/ab43.json");
    let actual =
        format!("{}\n", serde_json::to_string_pretty(&report).expect("serialize report fixture"));

    assert_eq!(
        actual,
        expected,
        "AB43 PPP convergence report drifted; set {REGENERATE_PUBLIC_PPP_CONVERGENCE_FIXTURE_ENV}=1 to refresh the fixture when the new behavior is intended",
    );
    assert!(
        report.all_epochs_solved,
        "AB43 PPP should solve every public epoch; diagnostics={}",
        format_diagnostics(&report.epochs),
    );
    assert!(
        report.time_to_1m_s.is_some(),
        "AB43 PPP must report time-to-1m; diagnostics={}",
        format_diagnostics(&report.epochs),
    );
    assert!(
        report.best_3d_error_m < 1.0,
        "AB43 PPP best 3D error {:.3} m exceeds sub-meter budget; diagnostics={}",
        report.best_3d_error_m,
        format_diagnostics(&report.epochs),
    );
    assert!(
        report.final_3d_error_m < 5.0,
        "AB43 PPP final 3D error {:.3} m exceeds budget; diagnostics={}",
        report.final_3d_error_m,
        format_diagnostics(&report.epochs),
    );
}

#[test]
fn public_ab43_ppp_run_keeps_zenith_troposphere_state_physical() {
    let case = ab43_public_spp_case();
    let initial_position = case
        .observations
        .approx_position_ecef_m
        .expect("AB43 public PPP run needs APPROX POSITION XYZ");
    let mut filter = PppFilter::new(public_ab43_ppp_config());
    filter.seed_receiver_state([initial_position.0, initial_position.1, initial_position.2], 0.0);
    let products = BroadcastProductsProvider::new(case.navigation.ephemerides.clone());

    let mut solved_ztd_m = Vec::new();
    let mut solved_ztd_sigma_m = Vec::new();
    let mut troposphere_sources = Vec::new();
    for epoch in &case.observations.epochs {
        if let Some(solution) = filter.solve_epoch(epoch, &case.navigation.ephemerides, &products) {
            solved_ztd_m.push(solution.ztd_m);
            solved_ztd_sigma_m.push(solution.ztd_sigma_m.expect("AB43 PPP ZTD sigma"));
            troposphere_sources.push(solution.troposphere_source);
        }
    }

    assert!(!solved_ztd_m.is_empty(), "AB43 PPP should emit solved ZTD states");
    assert!(
        solved_ztd_m.iter().all(|ztd_m| ztd_m.is_finite() && (1.0..10.0).contains(ztd_m)),
        "AB43 PPP ZTD estimates must remain physical: {:?}",
        solved_ztd_m
    );
    assert!(
        solved_ztd_sigma_m.iter().all(|ztd_sigma_m| ztd_sigma_m.is_finite() && *ztd_sigma_m >= 0.0),
        "AB43 PPP ZTD sigmas must remain finite and non-negative: {:?}",
        solved_ztd_sigma_m
    );

    let final_ztd_sigma_m = *solved_ztd_sigma_m.last().expect("AB43 final ZTD sigma");
    let min_ztd_m = solved_ztd_m.iter().copied().fold(f64::INFINITY, f64::min);
    let max_ztd_m = solved_ztd_m.iter().copied().fold(f64::NEG_INFINITY, f64::max);

    assert!(
        final_ztd_sigma_m < 1.0,
        "AB43 final ZTD sigma {:.3} m should remain bounded",
        final_ztd_sigma_m
    );
    assert!(
        max_ztd_m - min_ztd_m < 3.0,
        "AB43 ZTD trajectory should stay bounded; min={min_ztd_m:.3} max={max_ztd_m:.3}"
    );
    assert!(
        troposphere_sources
            .iter()
            .all(|source| *source == PppTroposphereSource::StandardAtmosphere),
        "AB43 default PPP run should report standard atmosphere: {troposphere_sources:?}"
    );
}

#[test]
fn public_ab43_ppp_zenith_delay_tracks_local_meteorology() {
    let case = ab43_public_spp_case();
    let initial_position = case
        .observations
        .approx_position_ecef_m
        .expect("AB43 public PPP run needs APPROX POSITION XYZ");
    let dry_config = PppConfig {
        use_iono_free: true,
        reset_gap_s: 30.0,
        tropo_pressure_hpa: Some(940.0),
        tropo_temperature_k: Some(288.15),
        tropo_relative_humidity: Some(0.15),
        ..PppConfig::default()
    };
    let humid_config = PppConfig {
        use_iono_free: true,
        reset_gap_s: 30.0,
        tropo_pressure_hpa: Some(1_020.0),
        tropo_temperature_k: Some(303.15),
        tropo_relative_humidity: Some(0.95),
        ..PppConfig::default()
    };
    let mut dry_filter = PppFilter::new(dry_config);
    let mut humid_filter = PppFilter::new(humid_config);
    dry_filter
        .seed_receiver_state([initial_position.0, initial_position.1, initial_position.2], 0.0);
    humid_filter
        .seed_receiver_state([initial_position.0, initial_position.1, initial_position.2], 0.0);
    let products = BroadcastProductsProvider::new(case.navigation.ephemerides.clone());
    let first_epoch = case.observations.epochs.first().expect("AB43 first PPP epoch");

    let dry_solution = dry_filter
        .solve_epoch(first_epoch, &case.navigation.ephemerides, &products)
        .expect("dry PPP solution");
    let humid_solution = humid_filter
        .solve_epoch(first_epoch, &case.navigation.ephemerides, &products)
        .expect("humid PPP solution");

    assert!(
        humid_solution.ztd_m > dry_solution.ztd_m + 0.05,
        "humid AB43 meteorology should produce higher PPP ZTD; dry={:.3} humid={:.3}",
        dry_solution.ztd_m,
        humid_solution.ztd_m
    );
    assert_eq!(dry_solution.troposphere_source, PppTroposphereSource::LocalMeteorology);
    assert_eq!(humid_solution.troposphere_source, PppTroposphereSource::LocalMeteorology);
}

fn public_ab43_ppp_config() -> PppConfig {
    PppConfig { use_iono_free: true, reset_gap_s: 30.0, ..PppConfig::default() }
}

fn load_convergence_fixture_text(fixture_file: &str) -> String {
    fs::read_to_string(fixture_path(fixture_file)).expect("public PPP convergence fixture")
}

fn write_convergence_fixture(fixture_file: &str, report: &PublicPppConvergenceReport) {
    let path = fixture_path(fixture_file);
    let parent = path.parent().expect("fixture parent directory");
    fs::create_dir_all(parent).expect("create PPP convergence fixture directory");
    let contents =
        serde_json::to_string_pretty(report).expect("serialize public PPP convergence fixture");
    fs::write(path, format!("{contents}\n")).expect("write public PPP convergence fixture");
}

fn fixture_path(fixture_file: &str) -> std::path::PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/data").join(fixture_file)
}

fn format_diagnostics(
    diagnostics: &[bijux_gnss_testkit::public_ppp::PublicPppConvergenceEpoch],
) -> String {
    diagnostics
        .iter()
        .map(|epoch| {
            format!(
                "week={} tow={:.1} elapsed={:.1}s solved={} error={:?} rejection={:?}",
                epoch.gps_time.week,
                epoch.gps_time.tow_s,
                epoch.elapsed_s,
                epoch.solved,
                epoch.three_dimensional_error_m,
                epoch.last_rejection,
            )
        })
        .collect::<Vec<_>>()
        .join("; ")
}
