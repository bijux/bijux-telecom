#![allow(missing_docs)]

use bijux_gnss_core::api::{GpsTime, ObsEpoch, ReceiverSampleTrace};
use bijux_gnss_receiver::api::ValidationReferenceEpoch;
use bijux_gnss_receiver::api::{
    observations_from_tracking_results_with_gps_anchor,
    sim::{
        generate_l1_ca_multi, truth_guided_receiver_accuracy_budgets, validate_pvt_accuracy_budget,
        validate_truth_guided_pvt_table, SyntheticPvtAccuracyReport,
        SyntheticPvtTruthReferenceEpoch,
    },
    Navigation, ReceiverPipelineConfig, ReceiverRuntime, TrackingEngine,
};

use crate::navigation_truth::{multisatellite_pvt_scenario, truth_seeded_acquisition_results};

pub struct TruthSeededNavigationCn0Case {
    pub scenario_id: String,
    pub observations: Vec<ObsEpoch>,
    pub pvt_accuracy: SyntheticPvtAccuracyReport,
}

pub fn build_truth_seeded_navigation_cn0_case(
    cn0_db_hz: f32,
    scenario_id_prefix: &str,
) -> TruthSeededNavigationCn0Case {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 1_023_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 5,
        tracking_budget_ms: 100.0,
        tracking_over_budget_action: "continue".to_string(),
        ..ReceiverPipelineConfig::default()
    };
    let scenario_id = format!("{scenario_id_prefix}_cn0_{:03}", (cn0_db_hz * 10.0).round() as i32);
    let mut profile = multisatellite_pvt_scenario(&config, 0.08, &scenario_id);
    for signal in &mut profile.scenario.satellites {
        signal.cn0_db_hz = cn0_db_hz;
    }

    let frame = generate_l1_ca_multi(&config, &profile.scenario);
    let source_time = ReceiverSampleTrace::from_sample_time(frame.t0);
    let acquisitions = truth_seeded_acquisition_results(&config, source_time, &profile.scenario);
    let tracking = TrackingEngine::new(config.clone(), ReceiverRuntime::default());
    let tracks = tracking.track_from_acquisition(&frame, &acquisitions);
    let receive_time_s = profile.ephemerides.first().expect("synthetic ephemeris").toe_s;
    let gps_time = Some(GpsTime {
        week: profile.ephemerides.first().expect("synthetic ephemeris").week,
        tow_s: receive_time_s,
    });
    let observations =
        observations_from_tracking_results_with_gps_anchor(&config, gps_time, &tracks, 10)
            .output
            .into_iter()
            .filter(|epoch| {
                epoch.epoch_idx >= profile.target_epoch_idx && epoch.valid && epoch.sats.len() >= 4
            })
            .collect::<Vec<_>>();
    let mut navigation = Navigation::new(config, ReceiverRuntime::default());
    let solutions = observations
        .iter()
        .filter_map(|epoch| navigation.solve_epoch(epoch, &profile.ephemerides))
        .collect::<Vec<_>>();
    let reference = solutions
        .iter()
        .map(|solution| SyntheticPvtTruthReferenceEpoch {
            position: ValidationReferenceEpoch {
                epoch_idx: solution.epoch.index,
                t_rx_s: Some(solution.t_rx_s.0),
                latitude_deg: 0.0,
                longitude_deg: 0.0,
                altitude_m: 0.0,
                ecef_x_m: Some(profile.truth_ecef_m.0),
                ecef_y_m: Some(profile.truth_ecef_m.1),
                ecef_z_m: Some(profile.truth_ecef_m.2),
                vel_x_mps: None,
                vel_y_mps: None,
                vel_z_mps: None,
            },
            clock_bias_s: 0.0,
        })
        .collect::<Vec<_>>();
    let truth_table = validate_truth_guided_pvt_table(&scenario_id, &solutions, &reference);
    let pvt_accuracy =
        validate_pvt_accuracy_budget(&truth_table, truth_guided_receiver_accuracy_budgets().pvt);

    TruthSeededNavigationCn0Case { scenario_id, observations, pvt_accuracy }
}
