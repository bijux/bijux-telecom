#![allow(dead_code, missing_docs)]

use bijux_gnss_receiver::api::{
    build_validation_report_with_budgets, ValidationReport, ValidationSciencePolicy,
};

use super::navigation_noise::synthetic_pseudorange_noise_sweep_profiles;
use super::navigation_pipeline::{
    clean_synthetic_pvt_budgets, noisy_synthetic_navigation_run, CleanSyntheticNavigationRun,
    NoisySyntheticNavigationRun,
};

#[derive(Debug, Clone)]
pub struct SyntheticProtectionFaultPoint {
    pub profile_name: &'static str,
    pub max_abs_pseudorange_noise_m: f64,
    pub max_hpl_m: f64,
    pub max_vpl_m: f64,
    pub invalid_epoch_count: usize,
    pub horizontal_breach_epoch_count: usize,
    pub vertical_breach_epoch_count: usize,
}

pub fn protection_validation_report(run: &CleanSyntheticNavigationRun) -> ValidationReport {
    build_validation_report_with_budgets(
        &run.tracking,
        &run.observations,
        &run.solutions,
        &run.reference_epochs,
        run.config.sampling_freq_hz,
        false,
        Vec::new(),
        ValidationSciencePolicy::default(),
        clean_synthetic_pvt_budgets(),
    )
    .expect("validation report")
}

pub fn max_reported_hpl_m(run: &CleanSyntheticNavigationRun) -> Option<f64> {
    run.solutions.iter().filter_map(|solution| solution.integrity_hpl_m).reduce(f64::max)
}

pub fn max_reported_vpl_m(run: &CleanSyntheticNavigationRun) -> Option<f64> {
    run.solutions.iter().filter_map(|solution| solution.integrity_vpl_m).reduce(f64::max)
}

pub fn invalid_epoch_count(run: &CleanSyntheticNavigationRun) -> usize {
    run.solutions.iter().filter(|solution| !solution.valid).count()
}

pub fn synthetic_protection_fault_sweep() -> Vec<SyntheticProtectionFaultPoint> {
    synthetic_pseudorange_noise_sweep_profiles()
        .into_iter()
        .map(|noise_profile| {
            let max_abs_pseudorange_noise_m = noise_profile
                .satellites
                .iter()
                .map(|satellite| satellite.pseudorange_noise_m.abs())
                .fold(0.0, f64::max);
            let run = noisy_synthetic_navigation_run(noise_profile.clone());
            let report = protection_validation_report(&run.run);

            SyntheticProtectionFaultPoint {
                profile_name: noise_profile.profile_name,
                max_abs_pseudorange_noise_m,
                max_hpl_m: max_reported_hpl_m(&run.run).unwrap_or(0.0),
                max_vpl_m: max_reported_vpl_m(&run.run).unwrap_or(0.0),
                invalid_epoch_count: invalid_epoch_count(&run.run),
                horizontal_breach_epoch_count: report
                    .protection_levels
                    .horizontal_breach_epochs
                    .len(),
                vertical_breach_epoch_count: report.protection_levels.vertical_breach_epochs.len(),
            }
        })
        .collect()
}

pub fn synthetic_protection_fault_evidence(points: &[SyntheticProtectionFaultPoint]) -> String {
    points
        .iter()
        .map(|point| {
            format!(
                "{}: noise_max={:.3}m hpl_max={:.3}m vpl_max={:.3}m invalid_epochs={} hpl_breaches={} vpl_breaches={}",
                point.profile_name,
                point.max_abs_pseudorange_noise_m,
                point.max_hpl_m,
                point.max_vpl_m,
                point.invalid_epoch_count,
                point.horizontal_breach_epoch_count,
                point.vertical_breach_epoch_count,
            )
        })
        .collect::<Vec<_>>()
        .join(", ")
}

pub fn noisy_run_validation(run: &NoisySyntheticNavigationRun) -> ValidationReport {
    protection_validation_report(&run.run)
}
