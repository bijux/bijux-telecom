#![allow(dead_code, missing_docs)]

use bijux_gnss_core::api::{
    ecef_to_geodetic, NavSolutionEpoch, ObsEpoch, ReceiverSampleTrace, ValidationReferenceEpoch,
};
use bijux_gnss_receiver::api::{
    observations_from_tracking_results, sim::generate_l1_ca_multi, Navigation,
    ReceiverPipelineConfig, ReceiverRuntime, TrackingEngine, TrackingResult, ValidationBudgets,
};

use super::navigation_truth::{
    four_satellite_pvt_scenario, truth_seeded_acquisition_results, SyntheticPvtScenario,
};

pub const CLEAN_SYNTHETIC_PVT_POSITION_ERROR_MAX_M: f64 = 5.0;

pub struct CleanSyntheticNavigationRun {
    pub config: ReceiverPipelineConfig,
    pub profile: SyntheticPvtScenario,
    pub tracking: Vec<TrackingResult>,
    pub observations: Vec<ObsEpoch>,
    pub solutions: Vec<NavSolutionEpoch>,
    pub reference_epochs: Vec<ValidationReferenceEpoch>,
}

pub fn clean_synthetic_navigation_run() -> CleanSyntheticNavigationRun {
    let config = clean_synthetic_navigation_config();
    let profile = four_satellite_pvt_scenario(&config);
    let frame = generate_l1_ca_multi(&config, &profile.scenario);
    let source_time = ReceiverSampleTrace::from_sample_time(frame.t0);
    let acquisition_results =
        truth_seeded_acquisition_results(&config, source_time, &profile.scenario);
    let tracking = TrackingEngine::new(config.clone(), ReceiverRuntime::default());
    let tracking_results = tracking.track_from_acquisition(&frame, &acquisition_results);
    let observation_report = observations_from_tracking_results(&config, &tracking_results, 10);
    let observations = observation_report
        .output
        .into_iter()
        .filter(|epoch| {
            epoch.epoch_idx >= profile.target_epoch_idx && epoch.valid && epoch.sats.len() >= 4
        })
        .collect::<Vec<_>>();
    let mut navigation = Navigation::new(config.clone(), ReceiverRuntime::default());
    let solutions = observations
        .iter()
        .filter_map(|epoch| navigation.solve_epoch(epoch, &profile.ephemerides))
        .collect::<Vec<_>>();
    let reference_epochs = reference_epochs_for_truth(&solutions, profile.truth_ecef_m);

    CleanSyntheticNavigationRun {
        config,
        profile,
        tracking: tracking_results,
        observations,
        solutions,
        reference_epochs,
    }
}

pub fn clean_synthetic_pvt_budgets() -> ValidationBudgets {
    ValidationBudgets {
        reference_position_error_3d_m_max: Some(CLEAN_SYNTHETIC_PVT_POSITION_ERROR_MAX_M),
        ..ValidationBudgets::default()
    }
}

pub fn position_error_3d_m(solution: &NavSolutionEpoch, truth_ecef_m: (f64, f64, f64)) -> f64 {
    let dx = solution.ecef_x_m.0 - truth_ecef_m.0;
    let dy = solution.ecef_y_m.0 - truth_ecef_m.1;
    let dz = solution.ecef_z_m.0 - truth_ecef_m.2;
    (dx * dx + dy * dy + dz * dz).sqrt()
}

fn clean_synthetic_navigation_config() -> ReceiverPipelineConfig {
    ReceiverPipelineConfig {
        sampling_freq_hz: 1_023_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 5,
        tracking_budget_ms: 100.0,
        tracking_over_budget_action: "continue".to_string(),
        tropo_enable: false,
        ..ReceiverPipelineConfig::default()
    }
}

fn reference_epochs_for_truth(
    solutions: &[NavSolutionEpoch],
    truth_ecef_m: (f64, f64, f64),
) -> Vec<ValidationReferenceEpoch> {
    let (latitude_deg, longitude_deg, altitude_m) =
        ecef_to_geodetic(truth_ecef_m.0, truth_ecef_m.1, truth_ecef_m.2);

    solutions
        .iter()
        .map(|solution| ValidationReferenceEpoch {
            epoch_idx: solution.epoch.index,
            t_rx_s: Some(solution.t_rx_s.0),
            latitude_deg,
            longitude_deg,
            altitude_m,
            ecef_x_m: Some(truth_ecef_m.0),
            ecef_y_m: Some(truth_ecef_m.1),
            ecef_z_m: Some(truth_ecef_m.2),
            vel_x_mps: None,
            vel_y_mps: None,
            vel_z_mps: None,
        })
        .collect()
}
