#![allow(missing_docs)]

mod support;

use bijux_gnss_core::api::SignalBand;

use support::public_broadcast_ionosphere_case::{
    ab43_public_gps_broadcast_ionosphere_case, valid_broadcast_residuals,
};

#[test]
fn public_gps_broadcast_ionosphere_residuals_emit_many_valid_ab43_observations() {
    let case = ab43_public_gps_broadcast_ionosphere_case();
    let valid = valid_broadcast_residuals();

    assert!(
        !case.residuals.is_empty(),
        "AB43 dual-frequency dataset should emit broadcast ionosphere residual observations",
    );
    assert!(
        valid.len() > 50,
        "expected many valid broadcast ionosphere comparisons, got {}",
        valid.len()
    );
    assert_eq!(case.summary.observation_count, case.residuals.len());
    assert_eq!(case.summary.broadcast_valid_count, valid.len());
}

#[test]
fn public_gps_broadcast_ionosphere_residuals_preserve_model_and_signal_metadata() {
    let valid = valid_broadcast_residuals();

    assert!(
        valid.iter().all(|observation| observation.broadcast_model == "gps_klobuchar"),
        "expected all AB43 valid residuals to use the GPS Klobuchar model",
    );
    assert!(
        valid.iter().all(|observation| observation.band_1 == SignalBand::L1),
        "expected AB43 valid residuals to preserve L1 as the first signal band",
    );
    assert!(
        valid.iter().all(|observation| observation.band_2 == SignalBand::L2),
        "expected AB43 valid residuals to preserve L2 as the second signal band",
    );
    assert!(
        valid.iter().all(|observation| observation.signal_1.is_some() && observation.signal_2.is_some()),
        "expected AB43 valid residuals to preserve concrete signal identifiers",
    );
    assert!(
        valid.iter().all(|observation| observation.broadcast_reason == "ok"),
        "expected AB43 valid residuals to report successful broadcast comparisons",
    );
}
