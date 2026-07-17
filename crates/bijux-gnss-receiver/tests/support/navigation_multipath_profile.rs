#![allow(missing_docs)]

use std::collections::BTreeSet;

use bijux_gnss_core::api::{Constellation, SatId};
use bijux_gnss_receiver::api::{
    build_validation_report_with_budgets,
    sim::{
        truth_guided_receiver_accuracy_budgets, validate_pvt_accuracy_budget,
        validate_truth_guided_pvt_table, SyntheticPvtAccuracyReport, SyntheticPvtTruthTableReport,
    },
    ValidationReport, ValidationSciencePolicy,
};

use crate::navigation_pipeline::{
    clean_synthetic_pvt_budgets, noisy_synthetic_navigation_run, pvt_truth_reference_epochs,
    SatellitePseudorangeNoise, SyntheticPseudorangeNoiseProfile,
};

const MULTIPATH_SHAPE_BY_SATELLITE: [(u8, f64); 4] = [(7, 0.15), (11, 1.0), (19, 0.45), (23, 0.8)];
const MULTIPATH_LEVELS_M: [(&str, f64); 4] = [
    ("clean_reference", 0.0),
    ("light_reflection_bias", 6.0),
    ("moderate_reflection_bias", 18.0),
    ("severe_reflection_bias", 90.0),
];

#[allow(dead_code)]
pub struct NavigationMultipathCase {
    pub scenario_id: String,
    pub noise_profile: SyntheticPseudorangeNoiseProfile,
    pub truth_table: SyntheticPvtTruthTableReport,
    pub pvt_accuracy: SyntheticPvtAccuracyReport,
    pub validation_report: ValidationReport,
}

pub fn synthetic_navigation_multipath_profiles() -> Vec<SyntheticPseudorangeNoiseProfile> {
    MULTIPATH_LEVELS_M
        .into_iter()
        .map(|(profile_name, level_m)| {
            if level_m == 0.0 {
                return multipath_bias_profile(profile_name, &[]);
            }
            multipath_bias_profile(
                profile_name,
                &MULTIPATH_SHAPE_BY_SATELLITE
                    .into_iter()
                    .map(|(prn, scale)| (prn, level_m * scale))
                    .collect::<Vec<_>>(),
            )
        })
        .collect()
}

pub fn build_navigation_multipath_case(
    noise_profile: SyntheticPseudorangeNoiseProfile,
) -> NavigationMultipathCase {
    let scenario_id = format!("navigation_multipath_profile_{}", noise_profile.profile_name);
    let run = noisy_synthetic_navigation_run(noise_profile.clone());
    let truth_table = validate_truth_guided_pvt_table(
        &scenario_id,
        &run.run.solutions,
        &pvt_truth_reference_epochs(&run.run),
    );
    let pvt_accuracy =
        validate_pvt_accuracy_budget(&truth_table, truth_guided_receiver_accuracy_budgets().pvt);
    let science_policy =
        ValidationSciencePolicy { max_residual_rms_m: 4.0, ..ValidationSciencePolicy::default() };
    let validation_report = build_validation_report_with_budgets(
        &run.run.tracking,
        &run.run.observations,
        &run.run.solutions,
        &run.run.reference_epochs,
        run.run.config.sampling_freq_hz,
        false,
        Vec::new(),
        science_policy,
        clean_synthetic_pvt_budgets(),
    )
    .expect("multipath validation report");

    NavigationMultipathCase {
        scenario_id,
        noise_profile,
        truth_table,
        pvt_accuracy,
        validation_report,
    }
}

pub fn multipath_bias_profile(
    profile_name: &'static str,
    satellite_biases_m: &[(u8, f64)],
) -> SyntheticPseudorangeNoiseProfile {
    let visible_prns = satellite_biases_m.iter().map(|(prn, _)| *prn).collect::<BTreeSet<_>>();
    assert_eq!(
        visible_prns.len(),
        satellite_biases_m.len(),
        "multipath profile requires unique visible-satellite PRNs",
    );
    assert!(
        satellite_biases_m.iter().all(|(_, bias_m)| *bias_m >= 0.0),
        "multipath profile requires nonnegative pseudorange biases",
    );

    SyntheticPseudorangeNoiseProfile {
        profile_name,
        satellites: satellite_biases_m
            .iter()
            .map(|(prn, pseudorange_noise_m)| SatellitePseudorangeNoise {
                sat: SatId { constellation: Constellation::Gps, prn: *prn },
                pseudorange_noise_m: *pseudorange_noise_m,
            })
            .collect(),
    }
}

#[allow(dead_code)]
pub fn mean_abs_pseudorange_bias_m(profile: &SyntheticPseudorangeNoiseProfile) -> f64 {
    let count = profile.satellites.len();
    if count == 0 {
        0.0
    } else {
        profile.satellites.iter().map(|satellite| satellite.pseudorange_noise_m.abs()).sum::<f64>()
            / count as f64
    }
}

#[allow(dead_code)]
pub fn max_abs_pseudorange_bias_m(profile: &SyntheticPseudorangeNoiseProfile) -> f64 {
    profile
        .satellites
        .iter()
        .map(|satellite| satellite.pseudorange_noise_m.abs())
        .fold(0.0, f64::max)
}
