#![allow(missing_docs)]

mod support;

use support::public_ppp_case::{
    diagnose_public_ab43_ppp_epochs, public_ab43_ppp_config, public_ppp_convergence_metrics,
    solve_public_ab43_ppp,
};

#[test]
fn public_ab43_ppp_run_reports_convergence_metrics() {
    let diagnostics = diagnose_public_ab43_ppp_epochs(public_ab43_ppp_config())
        .expect("diagnose AB43 public PPP epochs");
    let solutions = solve_public_ab43_ppp(public_ab43_ppp_config())
        .expect("solve AB43 public PPP case");
    let metrics = public_ppp_convergence_metrics(&solutions)
        .expect("compute AB43 public PPP convergence metrics");

    assert_eq!(
        metrics.solved_epochs,
        diagnostics.len(),
        "AB43 PPP should solve every public epoch; diagnostics={}",
        format_diagnostics(&diagnostics),
    );
    assert!(
        metrics.time_to_1m_s.is_some(),
        "AB43 PPP must report time-to-1m; diagnostics={}",
        format_diagnostics(&diagnostics),
    );
    assert!(
        metrics.time_to_decimeter_s.is_some(),
        "AB43 PPP must report time-to-decimeter; diagnostics={}",
        format_diagnostics(&diagnostics),
    );
    assert!(
        metrics.final_3d_error_m < 1.0,
        "AB43 PPP final 3D error {:.3} m exceeds budget; diagnostics={}",
        metrics.final_3d_error_m,
        format_diagnostics(&diagnostics),
    );
}

fn format_diagnostics(
    diagnostics: &[support::public_ppp_case::PublicPppEpochDiagnostics],
) -> String {
    diagnostics
        .iter()
        .map(|epoch| {
            format!(
                "week={} tow={:.1} solved={} error={:?} rejection={:?}",
                epoch.gps_time.week,
                epoch.gps_time.tow_s,
                epoch.solved,
                epoch.three_dimensional_error_m,
                epoch.last_rejection,
            )
        })
        .collect::<Vec<_>>()
        .join("; ")
}
