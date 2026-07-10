#![allow(missing_docs)]

use bijux_gnss_core::api::{
    Chips, Constellation, Cycles, Epoch, Hertz, ObservationStatus,
    OBSERVATION_DOPPLER_MODEL_TRACKED_CARRIER_IF_OFFSET, ReceiverSampleTrace, SatId, TrackEpoch,
};
use bijux_gnss_receiver::api::{observations_from_tracking_results, ReceiverPipelineConfig, TrackingResult};

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

fn tracking_epoch(
    config: &ReceiverPipelineConfig,
    sat: SatId,
    epoch_idx: u64,
    doppler_hz: f64,
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
        carrier_hz: Hertz(doppler_hz),
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
        tracking_provenance: "integration_observations_multisat_doppler_epochs".to_string(),
        tracking_assumptions: None,
        signal_delay_alignment: None,
        tracking_uncertainty: None,
        processing_ms: None,
    }
}

fn track(config: &ReceiverPipelineConfig, sat: SatId, dopplers_hz: &[f64]) -> TrackingResult {
    let epochs = dopplers_hz
        .iter()
        .enumerate()
        .map(|(offset, doppler_hz)| tracking_epoch(config, sat, 70 + offset as u64, *doppler_hz))
        .collect::<Vec<_>>();
    TrackingResult {
        sat,
        carrier_hz: epochs.last().map(|epoch| epoch.carrier_hz.0).unwrap_or_default(),
        code_phase_samples: 0.0,
        acquisition_hypothesis: "accepted".to_string(),
        acquisition_score: 1.0,
        acquisition_code_phase_samples: 0,
        acquisition_carrier_hz: epochs.first().map(|epoch| epoch.carrier_hz.0).unwrap_or_default(),
        acq_to_track_state: "accepted".to_string(),
        epochs,
        transitions: Vec::new(),
    }
}

#[test]
fn grouped_observation_epochs_emit_doppler_for_each_satellite() {
    let config = observation_config();
    let sat_a = SatId { constellation: Constellation::Gps, prn: 3 };
    let sat_b = SatId { constellation: Constellation::Gps, prn: 7 };
    let report = observations_from_tracking_results(
        &config,
        &[track(&config, sat_a, &[125.0, 130.0]), track(&config, sat_b, &[-80.0, -78.0])],
        10,
    );

    let grouped_epochs = report
        .output
        .iter()
        .filter(|epoch| epoch.sats.len() == 2)
        .collect::<Vec<_>>();

    assert_eq!(grouped_epochs.len(), 2, "report={:?}", report.output);
    for epoch in grouped_epochs {
        for sat in &epoch.sats {
            assert_eq!(sat.observation_status, ObservationStatus::Accepted);
            assert_eq!(
                sat.metadata.doppler_model,
                OBSERVATION_DOPPLER_MODEL_TRACKED_CARRIER_IF_OFFSET
            );
            assert!(sat.doppler_hz.0.is_finite(), "{sat:?}");
        }
    }
}

#[test]
fn grouped_observation_epochs_preserve_per_satellite_doppler_values() {
    let config = observation_config();
    let sat_a = SatId { constellation: Constellation::Gps, prn: 5 };
    let sat_b = SatId { constellation: Constellation::Gps, prn: 9 };
    let report = observations_from_tracking_results(
        &config,
        &[track(&config, sat_a, &[40.0, 42.0]), track(&config, sat_b, &[-25.0, -27.0])],
        10,
    );

    let epoch70 = report.output.iter().find(|epoch| epoch.epoch_idx == 70).expect("epoch 70");
    let epoch71 = report.output.iter().find(|epoch| epoch.epoch_idx == 71).expect("epoch 71");

    let sat70 = epoch70
        .sats
        .iter()
        .map(|sat| (sat.signal_id.sat.prn, sat.doppler_hz.0))
        .collect::<Vec<_>>();
    let sat71 = epoch71
        .sats
        .iter()
        .map(|sat| (sat.signal_id.sat.prn, sat.doppler_hz.0))
        .collect::<Vec<_>>();

    assert_eq!(sat70, vec![(5, 40.0), (9, -25.0)]);
    assert_eq!(sat71, vec![(5, 42.0), (9, -27.0)]);
}
