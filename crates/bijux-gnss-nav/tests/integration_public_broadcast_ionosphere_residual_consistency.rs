#![allow(missing_docs)]

mod support;

use bijux_gnss_signal::api::{signal_spec_gps_l1_ca, signal_spec_gps_l2_py};

use support::public_broadcast_ionosphere_case::valid_broadcast_residuals;

fn approx_eq(left: f64, right: f64, relative_tolerance: f64) {
    let scale = left.abs().max(right.abs()).max(1.0);
    let error = (left - right).abs();
    assert!(
        error <= relative_tolerance * scale,
        "left={left} right={right} error={error} scale={scale} tolerance={relative_tolerance}",
    );
}

#[test]
fn public_gps_broadcast_ionosphere_code_residuals_match_measured_minus_broadcast() {
    let valid_code = valid_broadcast_residuals()
        .into_iter()
        .filter(|observation| {
            observation.measured_code_delay_band_1_m.is_some()
                && observation.measured_code_delay_band_2_m.is_some()
                && observation.broadcast_delay_band_1_m.is_some()
                && observation.broadcast_delay_band_2_m.is_some()
                && observation.code_residual_band_1_m.is_some()
                && observation.code_residual_band_2_m.is_some()
        })
        .collect::<Vec<_>>();

    assert!(valid_code.len() > 50);

    for observation in valid_code {
        approx_eq(
            observation.code_residual_band_1_m.expect("L1 code residual"),
            observation.measured_code_delay_band_1_m.expect("L1 measured code delay")
                - observation.broadcast_delay_band_1_m.expect("L1 broadcast delay"),
            1.0e-12,
        );
        approx_eq(
            observation.code_residual_band_2_m.expect("L2 code residual"),
            observation.measured_code_delay_band_2_m.expect("L2 measured code delay")
                - observation.broadcast_delay_band_2_m.expect("L2 broadcast delay"),
            1.0e-12,
        );
    }
}

#[test]
fn public_gps_broadcast_ionosphere_phase_residuals_preserve_l1_l2_scaling() {
    let valid_phase = valid_broadcast_residuals()
        .into_iter()
        .filter(|observation| {
            observation.phase_residual_band_1_m.is_some()
                && observation.phase_residual_band_2_m.is_some()
        })
        .collect::<Vec<_>>();
    let l1_hz = signal_spec_gps_l1_ca().carrier_hz.value();
    let l2_hz = signal_spec_gps_l2_py().carrier_hz.value();
    let scaling = (l1_hz * l1_hz) / (l2_hz * l2_hz);

    assert!(valid_phase.len() > 50);

    for observation in valid_phase {
        approx_eq(
            observation.phase_residual_band_2_m.expect("L2 phase residual"),
            observation.phase_residual_band_1_m.expect("L1 phase residual") * scaling,
            1.0e-12,
        );
    }
}
