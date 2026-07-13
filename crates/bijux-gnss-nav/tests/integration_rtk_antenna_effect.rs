#![allow(missing_docs)]

use bijux_gnss_nav::api::{
    choose_rtk_single_difference_reference_signal, rtk_double_difference_residual_metrics,
    rtk_double_difference_residual_metrics_with_antenna_corrections,
    rtk_double_differences_from_single_differences, rtk_single_difference_residual_metrics,
    rtk_single_difference_residual_metrics_with_antenna_corrections,
    rtk_single_differences_from_obs_epochs, RtkAntennaCorrectionConfig,
};
use bijux_gnss_testkit::antenna_validation::gps_l1_rtk_antenna_effect_case;

#[test]
fn rtk_antenna_corrections_reduce_synthetic_sd_and_dd_residuals() {
    let case = gps_l1_rtk_antenna_effect_case();
    let antenna_config = RtkAntennaCorrectionConfig {
        base_antenna_type: Some(case.base_antenna_type.clone()),
        rover_antenna_type: Some(case.rover_antenna_type.clone()),
        receiver_calibrations: Some(case.receiver_calibrations.clone()),
        satellite_calibrations: Some(case.satellite_calibrations.clone()),
    };

    let single_differences =
        rtk_single_differences_from_obs_epochs(&case.base_epoch, &case.rover_epoch);
    let uncorrected_sd = rtk_single_difference_residual_metrics(
        &single_differences,
        case.base_ecef_m,
        case.rover_ecef_m,
        &case.ephemerides,
        case.receive_gps_time.tow_s,
        case.receive_gps_time.tow_s,
    )
    .expect("uncorrected RTK single-difference residuals");
    let corrected_sd = rtk_single_difference_residual_metrics_with_antenna_corrections(
        &single_differences,
        case.base_ecef_m,
        case.rover_ecef_m,
        &case.ephemerides,
        case.receive_gps_time.tow_s,
        case.receive_gps_time.tow_s,
        Some(&antenna_config),
    )
    .expect("corrected RTK single-difference residuals");

    let reference =
        choose_rtk_single_difference_reference_signal(&single_differences).expect("RTK reference");
    let double_differences =
        rtk_double_differences_from_single_differences(&single_differences, reference);
    let uncorrected_dd = rtk_double_difference_residual_metrics(
        &double_differences,
        case.base_ecef_m,
        case.rover_enu_m,
        &case.ephemerides,
        case.receive_gps_time.tow_s,
    )
    .expect("uncorrected RTK double-difference residuals");
    let corrected_dd = rtk_double_difference_residual_metrics_with_antenna_corrections(
        &double_differences,
        case.base_ecef_m,
        case.rover_enu_m,
        &case.ephemerides,
        case.receive_gps_time.tow_s,
        Some(&antenna_config),
    )
    .expect("corrected RTK double-difference residuals");

    assert!(corrected_sd.residual_rms_m < 1.0e-4);
    assert!(uncorrected_sd.residual_rms_m > corrected_sd.residual_rms_m * 100.0);
    assert!(corrected_dd.residual_rms_m < 1.0e-4);
    assert!(uncorrected_dd.residual_rms_m > corrected_dd.residual_rms_m * 100.0);
}
