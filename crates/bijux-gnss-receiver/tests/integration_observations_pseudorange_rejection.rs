#![allow(missing_docs)]

use bijux_gnss_core::api::{
    Chips, Constellation, Cycles, Epoch, Hertz, ObservationEpochDecision, ObservationStatus,
    ReceiverSampleTrace, SatId, SignalDelayAlignment, TrackEpoch,
};
use bijux_gnss_receiver::api::{
    observations_from_tracking_results, ReceiverPipelineConfig, TrackingResult,
};

fn observation_config() -> ReceiverPipelineConfig {
    ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 4,
        early_late_spacing_chips: 0.5,
        dll_bw_hz: 2.0,
        pll_bw_hz: 15.0,
        fll_bw_hz: 10.0,
        ..ReceiverPipelineConfig::default()
    }
}

fn samples_per_epoch(config: &ReceiverPipelineConfig) -> u64 {
    ((config.sampling_freq_hz * config.code_length as f64) / config.code_freq_basis_hz).round()
        as u64
}

fn aligned_tracking_epoch(
    config: &ReceiverPipelineConfig,
    sat: SatId,
    epoch_idx: u64,
    whole_code_periods: u64,
    code_phase_samples: f64,
) -> TrackEpoch {
    let sample_index = epoch_idx * samples_per_epoch(config);
    TrackEpoch {
        epoch: Epoch { index: epoch_idx },
        sample_index,
        source_time: ReceiverSampleTrace::from_sample_index(sample_index, config.sampling_freq_hz),
        sat,
        prompt_i: 1.0,
        prompt_q: 0.0,
        early_i: 0.0,
        early_q: 0.0,
        late_i: 0.0,
        late_q: 0.0,
        carrier_hz: Hertz(0.0),
        carrier_phase_cycles: Cycles(0.0),
        code_rate_hz: Hertz(config.code_freq_basis_hz),
        code_phase_samples: Chips(code_phase_samples),
        lock: true,
        cn0_dbhz: 48.0,
        pll_lock: true,
        dll_lock: true,
        fll_lock: true,
        cycle_slip: false,
        nav_bit_lock: false,
        navigation_bit_sign: None,
        dll_err: 0.0,
        pll_err: 0.0,
        fll_err: 0.0,
        anti_false_lock: false,
        cycle_slip_reason: None,
        lock_state: "tracking".to_string(),
        lock_state_reason: Some("stable_tracking".to_string()),
        channel_id: Some(sat.prn),
        channel_uid: format!("Gps-{:02}-ch{:02}", sat.prn, sat.prn),
        tracking_provenance: "integration_observations_pseudorange_rejection".to_string(),
        tracking_assumptions: None,
        signal_delay_alignment: Some(SignalDelayAlignment {
            whole_code_periods,
            source: "synthetic_truth".to_string(),
        }),
        tracking_uncertainty: None,
        processing_ms: None,
    }
}

fn track_from_epoch(epoch: TrackEpoch) -> TrackingResult {
    let sat = epoch.sat;
    TrackingResult {
        sat,
        carrier_hz: epoch.carrier_hz.0,
        code_phase_samples: epoch.code_phase_samples.0,
        acquisition_hypothesis: "accepted".to_string(),
        acquisition_score: 1.0,
        acquisition_code_phase_samples: 0,
        acquisition_carrier_hz: epoch.carrier_hz.0,
        acq_to_track_state: "accepted".to_string(),
        epochs: vec![epoch],
        transitions: Vec::new(),
    }
}

#[test]
fn grouped_epoch_rejects_non_positive_pseudorange_satellite() {
    let config = observation_config();
    let valid_sat = SatId { constellation: Constellation::Gps, prn: 3 };
    let invalid_sat = SatId { constellation: Constellation::Gps, prn: 7 };
    let tracks = vec![
        track_from_epoch(aligned_tracking_epoch(&config, valid_sat, 70, 68, 0.0)),
        track_from_epoch(aligned_tracking_epoch(&config, invalid_sat, 70, 0, -8.0)),
    ];

    let report = observations_from_tracking_results(&config, &tracks, 10);
    let epoch =
        report.output.iter().find(|epoch| epoch.epoch_idx == 70).expect("shared observation epoch");
    let valid =
        epoch.sats.iter().find(|sat| sat.signal_id.sat == valid_sat).expect("valid satellite");
    let invalid =
        epoch.sats.iter().find(|sat| sat.signal_id.sat == invalid_sat).expect("invalid satellite");

    assert_eq!(valid.observation_status, ObservationStatus::Accepted);
    assert_eq!(epoch.decision, ObservationEpochDecision::Rejected);
    assert_eq!(epoch.decision_reason.as_deref(), Some("inconsistent_observable"));
    assert!(invalid
        .observation_reject_reasons
        .iter()
        .any(|reason| reason == "non_positive_pseudorange"));
}

#[test]
fn grouped_epoch_rejects_out_of_bounds_pseudorange_satellite() {
    let config = observation_config();
    let valid_sat = SatId { constellation: Constellation::Gps, prn: 4 };
    let invalid_sat = SatId { constellation: Constellation::Gps, prn: 8 };
    let tracks = vec![
        track_from_epoch(aligned_tracking_epoch(&config, valid_sat, 70, 68, 0.0)),
        track_from_epoch(aligned_tracking_epoch(&config, invalid_sat, 70, 200_000, 0.0)),
    ];

    let report = observations_from_tracking_results(&config, &tracks, 10);
    let epoch =
        report.output.iter().find(|epoch| epoch.epoch_idx == 70).expect("shared observation epoch");
    let invalid =
        epoch.sats.iter().find(|sat| sat.signal_id.sat == invalid_sat).expect("invalid satellite");

    assert_eq!(epoch.decision, ObservationEpochDecision::Rejected);
    assert_eq!(epoch.decision_reason.as_deref(), Some("inconsistent_observable"));
    assert!(invalid
        .observation_reject_reasons
        .iter()
        .any(|reason| reason == "pseudorange_out_of_bounds"));
}
