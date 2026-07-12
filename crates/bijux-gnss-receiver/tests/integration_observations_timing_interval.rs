#![allow(missing_docs)]

use bijux_gnss_core::api::{
    Chips, Constellation, Cycles, Epoch, Hertz, ReceiverSampleTrace, SatId, TrackEpoch,
};
use bijux_gnss_receiver::api::{
    observations_from_tracking_results, ReceiverPipelineConfig, TrackingResult,
};
use bijux_gnss_signal::api::samples_per_code;

fn observation_config(tracking_integration_ms: u32) -> ReceiverPipelineConfig {
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
        tracking_integration_ms,
        ..ReceiverPipelineConfig::default()
    }
}

fn observation_interval_samples(config: &ReceiverPipelineConfig) -> u64 {
    samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length) as u64
        * config.tracking_integration_ms.max(1) as u64
}

fn tracking_epoch(
    config: &ReceiverPipelineConfig,
    sat: SatId,
    epoch_idx: u64,
    sample_index: u64,
) -> TrackEpoch {
    TrackEpoch {
        epoch: Epoch { index: epoch_idx },
        sample_index,
        source_time: ReceiverSampleTrace::from_sample_index(sample_index, config.sampling_freq_hz),
        sat,
        signal_band: bijux_gnss_core::api::SignalBand::L1,
        glonass_frequency_channel: None,
        prompt_i: 1.0,
        prompt_q: 0.0,
        early_i: 0.0,
        early_q: 0.0,
        late_i: 0.0,
        late_q: 0.0,
        carrier_hz: Hertz(0.0),
        carrier_phase_cycles: Cycles(epoch_idx as f64 * 0.125),
        code_rate_hz: Hertz(config.code_freq_basis_hz),
        code_phase_samples: Chips(0.0),
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
        tracking_provenance: "integration_observations_timing_interval".to_string(),
        tracking_assumptions: None,
        signal_delay_alignment: None,
        tracking_uncertainty: None,
        processing_ms: None,
    }
}

fn track(config: &ReceiverPipelineConfig, sat: SatId, start_epoch_idx: u64) -> TrackingResult {
    let base_sample_index = start_epoch_idx * observation_interval_samples(config);
    let epochs = [0_u64, 1_u64]
        .into_iter()
        .map(|offset| {
            tracking_epoch(
                config,
                sat,
                start_epoch_idx + offset,
                base_sample_index + offset * observation_interval_samples(config),
            )
        })
        .collect::<Vec<_>>();
    TrackingResult {
        sat,
        carrier_hz: 0.0,
        code_phase_samples: 0.0,
        acquisition_hypothesis: "accepted".to_string(),
        acquisition_score: 1.0,
        acquisition_code_phase_samples: 0,
        acquisition_carrier_hz: 0.0,
        acq_to_track_state: "accepted".to_string(),
        epochs,
        transitions: Vec::new(),
    }
}

fn track_from_epochs(sat: SatId, epochs: Vec<TrackEpoch>) -> TrackingResult {
    TrackingResult {
        sat,
        carrier_hz: 0.0,
        code_phase_samples: 0.0,
        acquisition_hypothesis: "accepted".to_string(),
        acquisition_score: 1.0,
        acquisition_code_phase_samples: 0,
        acquisition_carrier_hz: 0.0,
        acq_to_track_state: "accepted".to_string(),
        epochs,
        transitions: Vec::new(),
    }
}

#[test]
fn grouped_observation_epochs_follow_configured_tracking_interval() {
    let config = observation_config(10);
    let sat_a = SatId { constellation: Constellation::Gps, prn: 3 };
    let sat_b = SatId { constellation: Constellation::Gps, prn: 7 };
    let report = observations_from_tracking_results(
        &config,
        &[track(&config, sat_a, 70), track(&config, sat_b, 70)],
        10,
    );

    let first_epoch = report.output.iter().find(|epoch| epoch.epoch_idx == 70).expect("epoch 70");
    let second_epoch = report.output.iter().find(|epoch| epoch.epoch_idx == 71).expect("epoch 71");
    let expected_interval_samples = observation_interval_samples(&config);
    let actual_interval_samples =
        second_epoch.source_time.sample_index - first_epoch.source_time.sample_index;

    assert!(report.events.iter().all(|event| event.code != "GNSS_OBS_TIME_INTERVAL_INVALID"));
    assert_eq!(actual_interval_samples, expected_interval_samples);
    assert!((second_epoch.t_rx_s.0 - first_epoch.t_rx_s.0 - 0.010).abs() <= 1.0e-12);
    assert!(report.output.iter().all(|epoch| epoch.valid));
}

#[test]
fn grouped_observation_epochs_reject_off_interval_spacing() {
    let config = observation_config(10);
    let sat = SatId { constellation: Constellation::Gps, prn: 3 };
    let base_epoch_idx = 70;
    let base_sample_index = base_epoch_idx * observation_interval_samples(&config);
    let first_epoch = tracking_epoch(&config, sat, base_epoch_idx, base_sample_index);
    let second_epoch = tracking_epoch(
        &config,
        sat,
        base_epoch_idx + 1,
        base_sample_index + observation_interval_samples(&config) + 512,
    );

    let report = observations_from_tracking_results(
        &config,
        &[track_from_epochs(sat, vec![first_epoch, second_epoch])],
        10,
    );
    let invalid_epoch = report.output.iter().find(|epoch| epoch.epoch_idx == 71).expect("epoch 71");

    assert!(!invalid_epoch.valid);
    assert_eq!(invalid_epoch.decision_reason.as_deref(), Some("invalid_observation_timing"));
    assert!(report.events.iter().any(|event| event.code == "GNSS_OBS_TIME_INTERVAL_INVALID"));
}
