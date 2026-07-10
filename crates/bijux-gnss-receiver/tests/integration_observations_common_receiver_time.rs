#![allow(missing_docs)]

use bijux_gnss_core::api::{
    Chips, Constellation, Cycles, Epoch, Hertz, ReceiverSampleTrace, SatId, TrackEpoch,
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

fn tracking_epoch(config: &ReceiverPipelineConfig, sat: SatId, epoch_idx: u64) -> TrackEpoch {
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
        tracking_provenance: "integration_observations_common_receiver_time".to_string(),
        tracking_assumptions: None,
        signal_delay_alignment: None,
        tracking_uncertainty: None,
        processing_ms: None,
    }
}

fn track(config: &ReceiverPipelineConfig, sat: SatId, epoch_indices: &[u64]) -> TrackingResult {
    let epochs =
        epoch_indices.iter().map(|epoch_idx| tracking_epoch(config, sat, *epoch_idx)).collect();
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
fn grouped_observation_epochs_share_one_receiver_time() {
    let config = observation_config();
    let sat_a = SatId { constellation: Constellation::Gps, prn: 3 };
    let sat_b = SatId { constellation: Constellation::Gps, prn: 7 };
    let report = observations_from_tracking_results(
        &config,
        &[track(&config, sat_a, &[70, 71]), track(&config, sat_b, &[70, 71])],
        10,
    );

    let grouped_epochs =
        report.output.iter().filter(|epoch| epoch.sats.len() == 2).collect::<Vec<_>>();

    assert_eq!(grouped_epochs.len(), 2, "report={:?}", report.output);
    for epoch in grouped_epochs {
        let expected_sample_index = epoch.epoch_idx * samples_per_epoch(&config);
        let manifest = epoch.manifest.as_ref().expect("manifest");

        assert_eq!(epoch.source_time.sample_index, expected_sample_index);
        assert_eq!(epoch.source_time.sample_rate_hz, config.sampling_freq_hz);
        assert_eq!(epoch.source_time.receiver_time_s, epoch.t_rx_s);
        assert_eq!(manifest.source_sample_index, expected_sample_index);
        assert_eq!(manifest.source_time, epoch.source_time);
        assert!(epoch
            .sats
            .iter()
            .all(|sat| sat.metadata.time_tag_sample_index == expected_sample_index));
    }
}
