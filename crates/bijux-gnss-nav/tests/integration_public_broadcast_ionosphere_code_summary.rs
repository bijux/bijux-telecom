#![allow(missing_docs)]

mod support;

use bijux_gnss_core::api::{signal_spec_gps_l1_ca, signal_spec_gps_l2_py};

use support::public_broadcast_ionosphere_case::ab43_public_gps_broadcast_ionosphere_case;

fn approx_eq(left: f64, right: f64, relative_tolerance: f64) {
    let scale = left.abs().max(right.abs()).max(1.0);
    let error = (left - right).abs();
    assert!(
        error <= relative_tolerance * scale,
        "left={left} right={right} error={error} scale={scale} tolerance={relative_tolerance}",
    );
}

#[test]
fn public_gps_broadcast_ionosphere_code_summary_stays_finite_and_nontrivial() {
    let summary = &ab43_public_gps_broadcast_ionosphere_case().summary;
    let code_band_1 = summary.code_band_1.expect("AB43 L1 code residual summary");
    let code_band_2 = summary.code_band_2.expect("AB43 L2 code residual summary");

    assert!(code_band_1.sample_count > 50);
    assert_eq!(code_band_1.sample_count, code_band_2.sample_count);
    assert!(code_band_1.mean_abs_residual_m.is_finite());
    assert!(code_band_2.mean_abs_residual_m.is_finite());
    assert!(code_band_1.rms_residual_m.is_finite());
    assert!(code_band_2.rms_residual_m.is_finite());
    assert!(code_band_1.max_abs_residual_m.is_finite());
    assert!(code_band_2.max_abs_residual_m.is_finite());
    assert!(code_band_1.mean_abs_residual_m > 0.1);
    assert!(code_band_2.mean_abs_residual_m > code_band_1.mean_abs_residual_m);
    assert!(code_band_1.rms_residual_m >= code_band_1.mean_abs_residual_m);
    assert!(code_band_2.rms_residual_m >= code_band_2.mean_abs_residual_m);
    assert!(code_band_1.max_abs_residual_m >= code_band_1.rms_residual_m);
    assert!(code_band_2.max_abs_residual_m >= code_band_2.rms_residual_m);
}

#[test]
fn public_gps_broadcast_ionosphere_code_summary_preserves_l1_l2_scaling() {
    let summary = &ab43_public_gps_broadcast_ionosphere_case().summary;
    let code_band_1 = summary.code_band_1.expect("AB43 L1 code residual summary");
    let code_band_2 = summary.code_band_2.expect("AB43 L2 code residual summary");
    let l1_hz = signal_spec_gps_l1_ca().carrier_hz.value();
    let l2_hz = signal_spec_gps_l2_py().carrier_hz.value();
    let scaling = (l1_hz * l1_hz) / (l2_hz * l2_hz);

    approx_eq(code_band_2.mean_residual_m, code_band_1.mean_residual_m * scaling, 1.0e-12);
    approx_eq(
        code_band_2.mean_abs_residual_m,
        code_band_1.mean_abs_residual_m * scaling,
        1.0e-12,
    );
    approx_eq(code_band_2.rms_residual_m, code_band_1.rms_residual_m * scaling, 1.0e-12);
    approx_eq(
        code_band_2.max_abs_residual_m,
        code_band_1.max_abs_residual_m * scaling,
        1.0e-12,
    );
}
