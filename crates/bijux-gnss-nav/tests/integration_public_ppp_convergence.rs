#![allow(missing_docs)]

use std::env;
use std::fs;
use std::path::Path;

mod support;

use bijux_gnss_nav::api::PppConfig;
use bijux_gnss_testkit::public_ppp::{build_public_ppp_convergence_report, PublicPppConvergenceReport};
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
        report.time_to_decimeter_s.is_some(),
        "AB43 PPP must report time-to-decimeter; diagnostics={}",
        format_diagnostics(&report.epochs),
    );
    assert!(
        report.final_3d_error_m < 1.0,
        "AB43 PPP final 3D error {:.3} m exceeds budget; diagnostics={}",
        report.final_3d_error_m,
        format_diagnostics(&report.epochs),
    );
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
