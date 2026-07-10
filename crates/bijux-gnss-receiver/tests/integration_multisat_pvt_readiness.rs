#![allow(missing_docs)]

mod support;

use bijux_gnss_core::api::{
    AcqCodePhaseRefinement, AcqHypothesis, AcqResult, AcqUncertainty, Hertz,
    ObservationEpochDecision, ReceiverSampleTrace, SignalBand,
};
use bijux_gnss_receiver::api::{
    carrier_hz_from_doppler_hz, observations_from_tracking_results,
    sim::{generate_l1_ca_multi, SyntheticSignalSource},
    Receiver, ReceiverPipelineConfig, ReceiverRuntime, TrackingEngine, TrackingResult,
};

use support::navigation_truth::{four_satellite_pvt_scenario, SyntheticPvtScenario};

#[test]
fn receiver_tracks_four_satellites_with_persisted_channel_history() {
    let (_profile, artifacts) = run_multisat_receiver();

    assert_eq!(artifacts.tracking.len(), 4, "receiver must track four channels");
    assert!(
        artifacts.tracking.iter().all(|track| track.epochs.len() >= 25),
        "each tracked channel must persist well beyond acquisition handoff",
    );
    assert!(
        artifacts
            .tracking
            .iter()
            .all(|track| track.epochs.iter().any(|epoch| epoch.lock && epoch.dll_lock)),
        "each tracked channel must emit a locked tracking epoch",
    );
}

#[test]
fn receiver_groups_four_satellites_into_one_observation_epoch() {
    let (config, profile, tracks) = track_multisat_from_truth_seeds();
    let observation_report = observations_from_tracking_results(&config, &tracks, 10);
    let grouped_epoch_summaries = observation_report
        .output
        .iter()
        .filter(|epoch| epoch.epoch_idx >= profile.target_epoch_idx && epoch.sats.len() == 4)
        .map(|epoch| {
            format!(
                "epoch={} valid={} decision={:?} reasons={:?} sats={:?}",
                epoch.epoch_idx,
                epoch.valid,
                epoch.decision,
                epoch.decision_reason,
                epoch
                    .sats
                    .iter()
                    .map(|sat| (
                        sat.signal_id.sat.prn,
                        sat.observation_status,
                        sat.observation_reject_reasons.clone(),
                    ))
                    .collect::<Vec<_>>()
            )
        })
        .collect::<Vec<_>>();
    let observation_epoch = observation_report
        .output
        .iter()
        .find(|epoch| epoch.epoch_idx >= profile.target_epoch_idx && epoch.valid && epoch.sats.len() == 4)
        .unwrap_or_else(|| {
            panic!(
                "valid shared observation epoch; grouped epochs: {}",
                grouped_epoch_summaries.join(" | ")
            )
        });

    assert_eq!(observation_epoch.sats.len(), 4, "shared epoch must contain four satellites");
    assert_eq!(observation_epoch.decision, ObservationEpochDecision::Accepted);
}

fn run_multisat_receiver() -> (SyntheticPvtScenario, bijux_gnss_receiver::api::RunArtifacts) {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 1_023_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 4,
        tracking_budget_ms: 100.0,
        tracking_over_budget_action: "continue".to_string(),
        ..ReceiverPipelineConfig::default()
    };
    let profile = four_satellite_pvt_scenario(&config);
    let receiver = Receiver::new(config.clone(), ReceiverRuntime::default());
    let mut source = SyntheticSignalSource::new(&config, &profile.scenario);
    let artifacts = receiver.run(&mut source).expect("receiver run");
    (profile, artifacts)
}

fn track_multisat_from_truth_seeds(
) -> (ReceiverPipelineConfig, SyntheticPvtScenario, Vec<TrackingResult>) {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 1_023_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 4,
        tracking_budget_ms: 100.0,
        tracking_over_budget_action: "continue".to_string(),
        ..ReceiverPipelineConfig::default()
    };
    let profile = four_satellite_pvt_scenario(&config);
    let frame = generate_l1_ca_multi(&config, &profile.scenario);
    let source_time = ReceiverSampleTrace::from_sample_time(frame.t0);
    let acquisition_results = profile
        .scenario
        .satellites
        .iter()
        .map(|signal| {
            let code_phase_samples = signal.code_phase_chips.round().clamp(0.0, 1022.0);
            AcqResult {
                sat: signal.sat,
                signal_band: SignalBand::L1,
                source_time,
                candidate_rank: 1,
                is_primary_candidate: true,
                doppler_hz: Hertz(signal.doppler_hz),
                carrier_hz: Hertz(carrier_hz_from_doppler_hz(
                    config.intermediate_freq_hz,
                    signal.doppler_hz,
                )),
                code_phase_samples: code_phase_samples as usize,
                peak: 1.0,
                second_peak: 0.1,
                mean: 0.1,
                peak_mean_ratio: 10.0,
                peak_second_ratio: 10.0,
                cn0_proxy: signal.cn0_db_hz,
                score: 1.0,
                hypothesis: AcqHypothesis::Accepted,
                assumptions: None,
                evidence: Vec::new(),
                threshold_provenance: None,
                explain_selection_reason: Some("truth_seed".to_string()),
                doppler_refinement: None,
                code_phase_refinement: Some(AcqCodePhaseRefinement {
                    method: "truth_seed".to_string(),
                    offset_samples: 0.0,
                    refined_code_phase_samples: signal.code_phase_chips,
                    left_correlation_norm: 1.0,
                    center_correlation_norm: 1.0,
                    right_correlation_norm: 1.0,
                }),
                uncertainty: Some(AcqUncertainty { doppler_hz: 1.0, code_phase_samples: 0.25 }),
            }
        })
        .collect::<Vec<_>>();
    let tracking = TrackingEngine::new(config.clone(), ReceiverRuntime::default());
    let tracks = tracking.track_from_acquisition(&frame, &acquisition_results);
    (config, profile, tracks)
}
