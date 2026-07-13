#![allow(missing_docs)]

mod support;

use bijux_gnss_core::api::{
    GpsTime, ObservationEpochDecision, ReceiverSampleTrace, SignalDelayAlignment,
};
use bijux_gnss_receiver::api::{
    observations_from_tracking_results,
    sim::{generate_l1_ca_multi, SyntheticSignalDelayAlignment, SyntheticSignalSource},
    Metric, MetricsSink, NullLogger, Receiver, ReceiverPipelineConfig, ReceiverRuntime,
    ReceiverRuntimeConfig, TraceRecord, TraceSink, TrackingEngine, TrackingResult,
};
use std::sync::{Arc, Mutex};

use support::navigation_truth::{
    four_satellite_pvt_scenario, truth_seeded_acquisition_results, SyntheticPvtScenario,
};

#[test]
fn receiver_tracks_four_satellites_with_persisted_channel_history() {
    let (_profile, artifacts) = run_multisat_receiver();
    let persistently_locked_channels = artifacts
        .tracking
        .iter()
        .filter(|track| track.epochs.iter().any(|epoch| epoch.lock && epoch.dll_lock))
        .count();

    assert!(artifacts.tracking.len() >= 4, "receiver must track at least four channels",);
    assert!(
        artifacts.tracking.iter().all(|track| track.epochs.len() >= 25),
        "each tracked channel must persist well beyond acquisition handoff",
    );
    assert!(
        persistently_locked_channels >= 4,
        "at least four tracked channels must emit a locked tracking epoch",
    );
}

#[test]
fn receiver_groups_four_satellites_into_one_observation_epoch() {
    let (config, profile, tracks) = track_multisat_from_truth_seeds();
    let observation_report = observations_from_tracking_results(&config, &tracks, 10);
    let grouped_epoch_summaries = observation_report
        .output
        .iter()
        .filter(|epoch| epoch.epoch_idx >= profile.target_epoch_idx && epoch.sats.len() >= 4)
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
        .find(|epoch| {
            epoch.epoch_idx >= profile.target_epoch_idx && epoch.valid && epoch.sats.len() >= 4
        })
        .unwrap_or_else(|| {
            panic!(
                "valid shared observation epoch; grouped epochs: {}",
                grouped_epoch_summaries.join(" | ")
            )
        });

    assert!(
        observation_epoch.sats.len() >= 4,
        "shared epoch must contain at least four satellites",
    );
    assert_eq!(observation_epoch.decision, ObservationEpochDecision::Accepted);
}

#[test]
fn receiver_run_emits_navigation_epochs_for_multisatellite_synthetic_input() {
    let config = multisat_receiver_config();
    let profile = four_satellite_pvt_scenario(&config);
    let scenario = navigation_ready_scenario(&profile);
    let receiver = Receiver::new(config.clone(), navigation_runtime_for_profile(&profile));
    let mut source = SyntheticSignalSource::new_with_signal_delay_alignments(
        &config,
        &scenario,
        multisat_signal_delay_alignments(&profile),
    );

    let artifacts = receiver.run(&mut source).expect("receiver run");
    let navigation_summary = artifacts
        .navigation
        .iter()
        .map(|epoch| {
            format!(
                "epoch={} valid={} status={:?} sats={}/{}",
                epoch.epoch.index,
                epoch.valid,
                epoch.status,
                epoch.used_sat_count,
                epoch.sat_count,
            )
        })
        .collect::<Vec<_>>();
    let observation_summary = artifacts
        .observations
        .iter()
        .map(|epoch| {
            format!(
                "epoch={} valid={} reason={:?} sats={} model={}",
                epoch.epoch_idx,
                epoch.valid,
                epoch.decision_reason,
                epoch.sats.len(),
                epoch.sats
                    .first()
                    .map(|sat| sat.metadata.pseudorange_model.clone())
                    .unwrap_or_else(|| "none".to_string()),
            )
        })
        .collect::<Vec<_>>();
    let acquisition_alignment_summary = artifacts
        .acquisitions
        .iter()
        .map(|result| {
            format!(
                "{:?}-{}:{}",
                result.sat.constellation,
                result.sat.prn,
                result
                    .signal_delay_alignment
                    .as_ref()
                    .map(|alignment| alignment.whole_code_periods.to_string())
                    .unwrap_or_else(|| "none".to_string()),
            )
        })
        .collect::<Vec<_>>();
    let tracking_alignment_summary = artifacts
        .tracking
        .iter()
        .map(|result| {
            let alignment = result
                .epochs
                .first()
                .and_then(|epoch| epoch.signal_delay_alignment.as_ref())
                .map(|alignment| alignment.whole_code_periods.to_string())
                .unwrap_or_else(|| "none".to_string());
            format!("{:?}-{}:{}", result.sat.constellation, result.sat.prn, alignment)
        })
        .collect::<Vec<_>>();

    assert!(
        !artifacts.navigation.is_empty(),
        "receiver run must emit navigation epochs when observations and navigation inputs exist",
    );
    assert!(
        artifacts.observations.iter().any(|epoch| epoch.valid && epoch.sats.len() >= 4),
        "receiver run must retain at least one valid multisatellite observation epoch; acquisitions: {}; tracking: {}; observations: {}; navigation: {}",
        acquisition_alignment_summary.join(" | "),
        tracking_alignment_summary.join(" | "),
        observation_summary.join(" | "),
        navigation_summary.join(" | "),
    );
    assert!(
        artifacts
            .navigation
            .iter()
            .any(|epoch| epoch.sat_count >= 4 && epoch.used_sat_count >= 4),
        "receiver run must feed a multisatellite observation set into navigation instead of leaving the stage unexecuted; acquisitions: {}; tracking: {}; observations: {}; navigation: {}",
        acquisition_alignment_summary.join(" | "),
        tracking_alignment_summary.join(" | "),
        observation_summary.join(" | "),
        navigation_summary.join(" | "),
    );
}

#[derive(Default)]
struct CapturedTrace {
    events: Mutex<Vec<TraceRecord>>,
}

#[derive(Default)]
struct NoopMetrics;

impl TraceSink for CapturedTrace {
    fn record(&self, t: TraceRecord) {
        self.events.lock().expect("trace lock").push(t);
    }
}

impl MetricsSink for NoopMetrics {
    fn metric(&self, _m: Metric) {}
}

#[test]
fn receiver_run_reports_missing_navigation_anchor_instead_of_not_executed() {
    let config = multisat_receiver_config();
    let profile = four_satellite_pvt_scenario(&config);
    let trace = Arc::new(CapturedTrace::default());
    let runtime = ReceiverRuntime::with_sinks(
        ReceiverRuntimeConfig::default(),
        Arc::new(NullLogger),
        trace.clone(),
        Arc::new(NoopMetrics),
    );
    let receiver = Receiver::new(config.clone(), runtime);
    let scenario = navigation_ready_scenario(&profile);
    let mut source = SyntheticSignalSource::new_with_signal_delay_alignments(
        &config,
        &scenario,
        multisat_signal_delay_alignments(&profile),
    );

    let artifacts = receiver.run(&mut source).expect("receiver run");
    let events = trace.events.lock().expect("trace lock");

    assert!(artifacts.navigation.is_empty(), "navigation must stay empty without a time anchor");
    assert!(events.iter().any(|event| {
        event.name == "pipeline_stage_complete"
            && event.fields.iter().any(|(key, value)| *key == "stage" && value == "navigation")
            && event
                .fields
                .iter()
                .any(|(key, value)| *key == "status" && value == "missing_capture_start_gps_time")
    }));
    assert!(!events.iter().any(|event| {
        event.name == "pipeline_stage_complete"
            && event
                .fields
                .iter()
                .any(|(key, value)| *key == "status" && value == "not_executed")
    }));
}

fn run_multisat_receiver() -> (SyntheticPvtScenario, bijux_gnss_receiver::api::RunArtifacts) {
    let config = multisat_receiver_config();
    let profile = four_satellite_pvt_scenario(&config);
    let receiver = Receiver::new(config.clone(), ReceiverRuntime::default());
    let mut source = SyntheticSignalSource::new(&config, &profile.scenario);
    let artifacts = receiver.run(&mut source).expect("receiver run");
    (profile, artifacts)
}

fn multisat_receiver_config() -> ReceiverPipelineConfig {
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

fn navigation_runtime_for_profile(profile: &SyntheticPvtScenario) -> ReceiverRuntime {
    let anchor_ephemeris = profile.ephemerides.first().expect("multisatellite ephemeris");
    ReceiverRuntime::new(ReceiverRuntimeConfig {
        capture_start_gps_time: Some(GpsTime {
            week: anchor_ephemeris.week,
            tow_s: anchor_ephemeris.toe_s,
        }),
        ..ReceiverRuntimeConfig::default()
    })
}

fn multisat_signal_delay_alignments(
    profile: &SyntheticPvtScenario,
) -> Vec<SyntheticSignalDelayAlignment> {
    profile
        .scenario
        .satellites
        .iter()
        .map(|signal| SyntheticSignalDelayAlignment {
            sat: signal.sat,
            signal_delay_alignment: SignalDelayAlignment {
                whole_code_periods: profile.pseudorange_epoch_base,
                source: "synthetic_navigation_truth".to_string(),
            },
        })
        .collect()
}

fn navigation_ready_scenario(profile: &SyntheticPvtScenario) -> bijux_gnss_receiver::api::sim::SyntheticScenario {
    let mut scenario = profile.scenario.clone();
    for signal in &mut scenario.satellites {
        signal.cn0_db_hz = 42.0;
    }
    scenario
}

fn track_multisat_from_truth_seeds(
) -> (ReceiverPipelineConfig, SyntheticPvtScenario, Vec<TrackingResult>) {
    let config = multisat_receiver_config();
    let profile = four_satellite_pvt_scenario(&config);
    let frame = generate_l1_ca_multi(&config, &profile.scenario);
    let source_time = ReceiverSampleTrace::from_sample_time(frame.t0);
    let acquisition_results =
        truth_seeded_acquisition_results(&config, source_time, &profile.scenario);
    let tracking = TrackingEngine::new(config.clone(), ReceiverRuntime::default());
    let tracks = tracking.track_from_acquisition(&frame, &acquisition_results);
    (config, profile, tracks)
}
