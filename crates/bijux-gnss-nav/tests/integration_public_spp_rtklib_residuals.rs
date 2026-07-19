#![allow(missing_docs)]

use std::collections::BTreeMap;

mod support;

use bijux_gnss_core::api::SatId;
use support::public_spp_case::{ab43_public_spp_case, solve_public_ab43_epoch_with_satellites};
use support::rtklib_reference::ab43_rtklib_single_reference;

const RTKLIB_RESIDUAL_DELTA_TOLERANCE_M: f64 = 2.5;
const RTKLIB_RESIDUAL_RMS_DELTA_TOLERANCE_M: f64 = 1.0;
const MIN_SOLVED_RTKLIB_RESIDUAL_SATELLITES: usize = 4;

#[test]
fn public_single_point_residuals_remain_within_rtklib_satellite_set() {
    let case = ab43_public_spp_case();
    let reference = ab43_rtklib_single_reference();

    let mut checked_epochs = 0usize;
    for (epoch, reference_epoch) in case.observations.epochs.iter().zip(reference.iter()) {
        let reference_sats =
            reference_epoch.residuals.iter().map(|residual| residual.sat).collect::<Vec<_>>();
        let Ok(solved) = solve_public_ab43_epoch_with_satellites(epoch, Some(&reference_sats))
        else {
            continue;
        };
        let solved_sats =
            solved.residuals.iter().map(|(sat, _residual_m, _weight)| *sat).collect::<Vec<_>>();

        assert!(
            solved_sats.len() >= MIN_SOLVED_RTKLIB_RESIDUAL_SATELLITES,
            "AB43 residual satellite set should retain at least {MIN_SOLVED_RTKLIB_RESIDUAL_SATELLITES} RTKLIB satellites at GPS week {} TOW {:.1}s; solved={solved_sats:?}",
            solved.gps_time.week,
            solved.gps_time.tow_s,
        );
        assert!(
            solved_sats.iter().all(|sat| reference_sats.contains(sat)),
            "AB43 residual satellite set should remain a subset of RTKLIB at GPS week {} TOW {:.1}s; solved={solved_sats:?} reference={reference_sats:?}",
            solved.gps_time.week,
            solved.gps_time.tow_s,
        );
        checked_epochs += 1;
    }

    assert!(checked_epochs > 0, "expected at least one AB43 residual satellite-set check");
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
        let Ok(solved) = solve_public_ab43_epoch_with_satellites(epoch, Some(&reference_sats))
        else {
            continue;
        };
        let solved_by_sat = solved
            .residuals
            .iter()
            .map(|(sat, residual_m, _weight)| (*sat, *residual_m))
            .collect::<BTreeMap<SatId, f64>>();

        for reference_residual in &reference_epoch.residuals {
            let Some(solved_residual_m) = solved_by_sat.get(&reference_residual.sat).copied()
            else {
                continue;
            };
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

    assert!(compared_residuals > 0, "expected at least one AB43 residual comparison");
    let rms_delta_m = (squared_delta_sum_m2 / compared_residuals as f64).sqrt();
    assert!(
        rms_delta_m <= RTKLIB_RESIDUAL_RMS_DELTA_TOLERANCE_M,
        "AB43 residual RMS delta {:.3} m exceeds RTKLIB comparison tolerance across {} residuals",
        rms_delta_m,
        compared_residuals
    );
}
