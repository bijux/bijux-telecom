#![allow(missing_docs)]

use bijux_gnss_nav::api::sat_state_gps_l1ca_from_observation;
use bijux_gnss_testkit::antenna_validation::{
    gps_l1_ppp_antenna_effect_case, GpsL1PppAntennaEffectCase,
};

#[test]
fn ppp_antenna_corrections_reduce_synthetic_code_residuals() {
    let case = gps_l1_ppp_antenna_effect_case();

    let uncorrected_rms_m = ppp_code_residual_rms_m(&case, false);
    let corrected_rms_m = ppp_code_residual_rms_m(&case, true);

    assert!(corrected_rms_m < 1.0e-4);
    assert!(uncorrected_rms_m > corrected_rms_m * 100.0);
}

fn ppp_code_residual_rms_m(case: &GpsL1PppAntennaEffectCase, antenna_enabled: bool) -> f64 {
    let residuals_m: Vec<f64> = case
        .epoch
        .sats
        .iter()
        .map(|sat| {
            let ephemeris = case
                .ephemerides
                .iter()
                .find(|candidate| candidate.sat == sat.signal_id.sat)
                .expect("PPP antenna-effect ephemeris");
            let state = sat_state_gps_l1ca_from_observation(
                ephemeris,
                case.receive_gps_time.tow_s,
                sat.pseudorange_m.0,
                sat.timing,
            );
            let sat_ecef_m = [state.x_m, state.y_m, state.z_m];
            let mut modeled_m = geometric_range_m(case.receiver_ecef_m, sat_ecef_m)
                - state.clock_correction.bias_s * 299_792_458.0;
            if antenna_enabled {
                modeled_m += case
                    .satellite_calibrations
                    .range_correction_m(
                        sat.signal_id.sat,
                        sat.signal_id.band,
                        Some(case.receive_gps_time),
                        sat_ecef_m,
                        case.receiver_ecef_m,
                    )
                    .expect("PPP satellite antenna correction");
                modeled_m += case
                    .receiver_calibrations
                    .range_correction_m(
                        &case.receiver_antenna_type,
                        sat.signal_id.band,
                        Some(case.receive_gps_time),
                        case.receiver_ecef_m,
                        sat_ecef_m,
                    )
                    .expect("PPP receiver antenna correction");
            }
            sat.pseudorange_m.0 - modeled_m
        })
        .collect();

    (residuals_m.iter().map(|residual_m| residual_m * residual_m).sum::<f64>()
        / residuals_m.len() as f64)
        .sqrt()
}

fn geometric_range_m(receiver_ecef_m: [f64; 3], sat_ecef_m: [f64; 3]) -> f64 {
    let dx = receiver_ecef_m[0] - sat_ecef_m[0];
    let dy = receiver_ecef_m[1] - sat_ecef_m[1];
    let dz = receiver_ecef_m[2] - sat_ecef_m[2];
    (dx * dx + dy * dy + dz * dz).sqrt()
}
