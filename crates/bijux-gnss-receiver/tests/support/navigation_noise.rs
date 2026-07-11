#![allow(dead_code, missing_docs)]

use bijux_gnss_core::api::{Constellation, SatId};

use super::navigation_pipeline::{
    noisy_synthetic_navigation_run, position_error_3d_m, SatellitePseudorangeNoise,
    SyntheticPseudorangeNoiseProfile,
};

const NOISE_SHAPE_BY_SATELLITE: [(u8, f64); 5] =
    [(3, -0.85), (7, 0.40), (11, 1.25), (19, -1.10), (23, 0.55)];
const NOISE_LEVELS_M: [(&str, f64); 4] = [
    ("clean_reference", 0.0),
    ("light_pseudorange_noise", 2.0),
    ("moderate_pseudorange_noise", 5.0),
    ("heavy_pseudorange_noise", 10.0),
];

#[derive(Debug, Clone)]
pub struct SyntheticPvtNoiseSweepPoint {
    pub profile_name: &'static str,
    pub max_abs_pseudorange_noise_m: f64,
    pub max_position_error_3d_m: f64,
}

pub fn synthetic_pseudorange_noise_sweep_profiles() -> Vec<SyntheticPseudorangeNoiseProfile> {
    NOISE_LEVELS_M
        .into_iter()
        .map(|(profile_name, noise_level_m)| SyntheticPseudorangeNoiseProfile {
            profile_name,
            satellites: NOISE_SHAPE_BY_SATELLITE
                .into_iter()
                .map(|(prn, scale)| satellite_noise(prn, noise_level_m * scale))
                .collect(),
        })
        .collect()
}

pub fn synthetic_pvt_noise_sweep() -> Vec<SyntheticPvtNoiseSweepPoint> {
    synthetic_pseudorange_noise_sweep_profiles()
        .into_iter()
        .map(|noise_profile| {
            let max_abs_pseudorange_noise_m = noise_profile
                .satellites
                .iter()
                .map(|satellite| satellite.pseudorange_noise_m.abs())
                .fold(0.0, f64::max);
            let run = noisy_synthetic_navigation_run(noise_profile.clone());
            let max_position_error_3d_m = run
                .run
                .solutions
                .iter()
                .map(|solution| position_error_3d_m(solution, run.run.profile.truth_ecef_m))
                .fold(0.0, f64::max);

            SyntheticPvtNoiseSweepPoint {
                profile_name: noise_profile.profile_name,
                max_abs_pseudorange_noise_m,
                max_position_error_3d_m,
            }
        })
        .collect()
}

pub fn synthetic_pvt_noise_evidence(points: &[SyntheticPvtNoiseSweepPoint]) -> String {
    points
        .iter()
        .map(|point| {
            format!(
                "{}: noise_max={:.3}m position_error_max={:.3}m",
                point.profile_name,
                point.max_abs_pseudorange_noise_m,
                point.max_position_error_3d_m
            )
        })
        .collect::<Vec<_>>()
        .join(", ")
}

fn satellite_noise(prn: u8, pseudorange_noise_m: f64) -> SatellitePseudorangeNoise {
    SatellitePseudorangeNoise {
        sat: SatId { constellation: Constellation::Gps, prn },
        pseudorange_noise_m,
    }
}
