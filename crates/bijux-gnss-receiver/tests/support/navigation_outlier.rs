#![allow(dead_code, missing_docs)]

use bijux_gnss_core::api::{Constellation, MeasurementRejectReason, SatId};

use super::navigation_pipeline::{
    noisy_synthetic_navigation_run, noisy_synthetic_navigation_run_with_satellite_prns,
    position_error_3d_m, SatellitePseudorangeNoise, SyntheticPseudorangeNoiseProfile,
};

pub const BAD_SATELLITE_PSEUDORANGE_BIAS_M: f64 = 1_000.0;
const BAD_SATELLITE_PRN: u8 = 29;
const SECOND_BAD_SATELLITE_PRN: u8 = 23;
const UNDERDETERMINED_RAIM_VISIBLE_PRNS: [u8; 5] = [3, 7, 11, 19, BAD_SATELLITE_PRN];

pub fn single_bad_satellite_profile() -> SyntheticPseudorangeNoiseProfile {
    SyntheticPseudorangeNoiseProfile {
        profile_name: "single_bad_satellite",
        satellites: vec![SatellitePseudorangeNoise {
            sat: SatId { constellation: Constellation::Gps, prn: BAD_SATELLITE_PRN },
            pseudorange_noise_m: BAD_SATELLITE_PSEUDORANGE_BIAS_M,
        }],
    }
}

pub fn simultaneous_bad_satellite_profile() -> SyntheticPseudorangeNoiseProfile {
    SyntheticPseudorangeNoiseProfile {
        profile_name: "simultaneous_bad_satellites",
        satellites: vec![
            SatellitePseudorangeNoise {
                sat: SatId { constellation: Constellation::Gps, prn: SECOND_BAD_SATELLITE_PRN },
                pseudorange_noise_m: BAD_SATELLITE_PSEUDORANGE_BIAS_M,
            },
            SatellitePseudorangeNoise {
                sat: SatId { constellation: Constellation::Gps, prn: BAD_SATELLITE_PRN },
                pseudorange_noise_m: BAD_SATELLITE_PSEUDORANGE_BIAS_M,
            },
        ],
    }
}

pub fn bad_satellite_prn() -> u8 {
    BAD_SATELLITE_PRN
}

pub fn simultaneous_bad_satellite_prns() -> [u8; 2] {
    [SECOND_BAD_SATELLITE_PRN, BAD_SATELLITE_PRN]
}

pub fn solution_position_errors_m(
    run: &super::navigation_pipeline::NoisySyntheticNavigationRun,
) -> Vec<f64> {
    run.run
        .solutions
        .iter()
        .map(|solution| position_error_3d_m(solution, run.run.profile.truth_ecef_m))
        .collect()
}

pub fn rejected_outlier_prns(
    run: &super::navigation_pipeline::NoisySyntheticNavigationRun,
) -> Vec<u8> {
    run.run
        .solutions
        .iter()
        .flat_map(|solution| solution.residuals.iter())
        .filter(|residual| {
            residual.rejected && residual.reject_reason == Some(MeasurementRejectReason::Outlier)
        })
        .map(|residual| residual.sat.prn)
        .collect()
}

pub fn single_bad_satellite_navigation_run(
) -> super::navigation_pipeline::NoisySyntheticNavigationRun {
    noisy_synthetic_navigation_run(single_bad_satellite_profile())
}

pub fn simultaneous_bad_satellite_navigation_run(
) -> super::navigation_pipeline::NoisySyntheticNavigationRun {
    noisy_synthetic_navigation_run(simultaneous_bad_satellite_profile())
}

pub fn underdetermined_bad_satellite_navigation_run(
) -> super::navigation_pipeline::NoisySyntheticNavigationRun {
    noisy_synthetic_navigation_run_with_satellite_prns(
        single_bad_satellite_profile(),
        &UNDERDETERMINED_RAIM_VISIBLE_PRNS,
    )
}
