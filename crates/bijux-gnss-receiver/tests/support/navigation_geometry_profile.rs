#![allow(missing_docs)]

use std::collections::BTreeSet;

use bijux_gnss_receiver::api::sim::{
    truth_guided_receiver_accuracy_budgets, validate_pvt_accuracy_budget,
    validate_truth_guided_pvt_table, SyntheticPvtAccuracyReport,
};

use crate::navigation_pipeline::{
    noisy_synthetic_navigation_run_with_satellite_prns, pvt_truth_reference_epochs,
    SyntheticPseudorangeNoiseProfile,
};

#[allow(dead_code)]
pub struct TruthSeededNavigationGeometryCase {
    pub scenario_id: String,
    pub visible_satellite_prns: Vec<u8>,
    pub pvt_accuracy: SyntheticPvtAccuracyReport,
}

pub fn build_truth_seeded_navigation_geometry_case(
    visible_satellite_prns: &[u8],
    scenario_id_prefix: &str,
) -> TruthSeededNavigationGeometryCase {
    validate_visible_satellite_prns(visible_satellite_prns);

    let scenario_id = format!(
        "{scenario_id_prefix}_prns_{}",
        visible_satellite_prns.iter().map(|prn| format!("{prn:02}")).collect::<Vec<_>>().join("_")
    );
    let run = noisy_synthetic_navigation_run_with_satellite_prns(
        SyntheticPseudorangeNoiseProfile {
            profile_name: "zero_geometry_profile",
            satellites: Vec::new(),
        },
        visible_satellite_prns,
    );
    let truth_table = validate_truth_guided_pvt_table(
        &scenario_id,
        &run.run.solutions,
        &pvt_truth_reference_epochs(&run.run),
    );
    let pvt_accuracy =
        validate_pvt_accuracy_budget(&truth_table, truth_guided_receiver_accuracy_budgets().pvt);

    TruthSeededNavigationGeometryCase {
        scenario_id,
        visible_satellite_prns: visible_satellite_prns.to_vec(),
        pvt_accuracy,
    }
}

fn validate_visible_satellite_prns(visible_satellite_prns: &[u8]) {
    assert!(
        visible_satellite_prns.len() >= 4,
        "geometry profile requires at least four visible satellites",
    );

    let visible_prns = visible_satellite_prns.iter().copied().collect::<BTreeSet<_>>();
    assert_eq!(
        visible_prns.len(),
        visible_satellite_prns.len(),
        "geometry profile requires unique visible-satellite PRNs",
    );
}
