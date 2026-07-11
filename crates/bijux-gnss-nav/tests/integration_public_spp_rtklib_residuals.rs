#![allow(missing_docs)]

use std::collections::BTreeMap;

mod support;

use bijux_gnss_core::api::SatId;
use support::public_spp_case::{ab43_public_spp_case, solve_public_ab43_epoch_with_satellites};
use support::rtklib_reference::ab43_rtklib_single_reference;

const RTKLIB_RESIDUAL_DELTA_TOLERANCE_M: f64 = 2.5;
const RTKLIB_RESIDUAL_RMS_DELTA_TOLERANCE_M: f64 = 1.0;

#[test]
fn public_single_point_residuals_match_rtklib_satellite_set() {
    let case = ab43_public_spp_case();
    let reference = ab43_rtklib_single_reference();

    for (epoch, reference_epoch) in case.observations.epochs.iter().zip(reference.iter()) {
        let reference_sats =
            reference_epoch.residuals.iter().map(|residual| residual.sat).collect::<Vec<_>>();
        let solved = solve_public_ab43_epoch_with_satellites(epoch, Some(&reference_sats))
            .expect("solve AB43 public epoch on RTKLIB satellite set");
        let solved_sats =
            solved.residuals.iter().map(|(sat, _residual_m, _weight)| *sat).collect::<Vec<_>>();

        assert_eq!(
            solved_sats, reference_sats,
            "AB43 residual satellite set should match RTKLIB at GPS week {} TOW {:.1}s",
            solved.gps_time.week, solved.gps_time.tow_s
        );
    }
}

#[test]
fn public_single_point_residuals_stay_within_rtklib_tolerance() {
    let case = ab43_public_spp_case();
    let reference = ab43_rtklib_single_reference();

    let mut squared_delta_sum_m2 = 0.0;
    let mut compared_residuals = 0usize;

    for (epoch, reference_epoch) in case.observations.epochs.iter().zip(reference.iter()) {
        let reference_sats =
            reference_epoch.residuals.iter().map(|residual| residual.sat).collect::<Vec<_>>();
        let solved = solve_public_ab43_epoch_with_satellites(epoch, Some(&reference_sats))
            .expect("solve AB43 public epoch on RTKLIB satellite set");
        let solved_by_sat = solved
            .residuals
            .iter()
            .map(|(sat, residual_m, _weight)| (*sat, *residual_m))
            .collect::<BTreeMap<SatId, f64>>();

        for reference_residual in &reference_epoch.residuals {
            let solved_residual_m = solved_by_sat
                .get(&reference_residual.sat)
                .copied()
                .expect("solved residual for RTKLIB satellite");
            let residual_delta_m = solved_residual_m - reference_residual.residual_m;
            squared_delta_sum_m2 += residual_delta_m * residual_delta_m;
            compared_residuals += 1;

            assert!(
                residual_delta_m.abs() <= RTKLIB_RESIDUAL_DELTA_TOLERANCE_M,
                "AB43 residual for {:?} at GPS week {} TOW {:.1}s drifted {:.3} m from RTKLIB (bijux={:.3} m, rtklib={:.3} m)",
                reference_residual.sat,
                solved.gps_time.week,
                solved.gps_time.tow_s,
                residual_delta_m,
                solved_residual_m,
                reference_residual.residual_m
            );
        }
    }

    let rms_delta_m = (squared_delta_sum_m2 / compared_residuals as f64).sqrt();
    assert!(
        rms_delta_m <= RTKLIB_RESIDUAL_RMS_DELTA_TOLERANCE_M,
        "AB43 residual RMS delta {:.3} m exceeds RTKLIB comparison tolerance across {} residuals",
        rms_delta_m,
        compared_residuals
    );
}
