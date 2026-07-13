#![allow(missing_docs)]

use bijux_gnss_core::api::{
    Chips, Constellation, Cycles, Epoch, Hertz, ReceiverSampleTrace, SatId, SignalDelayAlignment,
    TrackEpoch,
};
use bijux_gnss_receiver::api::{
    observations_from_tracking_results, ReceiverPipelineConfig, TrackingResult,
};

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

fn samples_per_epoch(config: &ReceiverPipelineConfig) -> u64 {
    ((config.sampling_freq_hz * config.code_length as f64) / config.code_freq_basis_hz).round()
        as u64
}

fn tracking_epoch(
    config: &ReceiverPipelineConfig,
    sat: SatId,
    epoch_idx: u64,
    cn0_dbhz: f64,
) -> TrackEpoch {
    let sample_index = epoch_idx * samples_per_epoch(config);
    TrackEpoch {
        epoch: Epoch { index: epoch_idx },
        sample_index,
        source_time: ReceiverSampleTrace::from_sample_index(sample_index, config.sampling_freq_hz),
        sat,
        signal_band: bijux_gnss_core::api::SignalBand::L1,
        signal_code: bijux_gnss_core::api::SignalCode::Unknown,
        glonass_frequency_channel: None,
        prompt_i: 1.0,
        prompt_q: 0.0,
        early_i: 0.0,
        early_q: 0.0,
        late_i: 0.0,
        late_q: 0.0,
        carrier_hz: Hertz(0.0),
        carrier_phase_cycles: Cycles(0.0),
        code_rate_hz: Hertz(config.code_freq_basis_hz),
        code_phase_samples: Chips(128.0),
        lock: true,
        cn0_dbhz,
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
        tracking_provenance: "integration_observations_uncertainty".to_string(),
        tracking_assumptions: None,
        signal_delay_alignment: Some(SignalDelayAlignment {
            whole_code_periods: 68,
            sample_delay_samples: 0,
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

fn observation_variance(config: &ReceiverPipelineConfig, epoch: TrackEpoch) -> f64 {
    let report = observations_from_tracking_results(config, &[track_from_epoch(epoch)], 10);
    report.output[0].sats[0].pseudorange_var_m2
}

#[test]
fn pseudorange_variance_responds_to_cn0_and_integration() {
    let sat = SatId { constellation: Constellation::Gps, prn: 7 };
    let short_config = observation_config(1);
    let long_config = observation_config(10);

    let strong_short =
        observation_variance(&short_config, tracking_epoch(&short_config, sat, 70, 48.0));
    let weak_short =
        observation_variance(&short_config, tracking_epoch(&short_config, sat, 70, 28.0));
    let strong_long =
        observation_variance(&long_config, tracking_epoch(&long_config, sat, 70, 48.0));

    assert!(
        weak_short > strong_short,
        "weaker C/N0 should inflate pseudorange variance: strong_short={strong_short} weak_short={weak_short}"
    );
    assert!(
        strong_long < strong_short,
        "longer coherent integration should tighten pseudorange variance: strong_short={strong_short} strong_long={strong_long}"
    );
}

#[test]
fn pseudorange_variance_responds_to_dll_lock_and_lock_quality() {
    let config = observation_config(1);
    let sat = SatId { constellation: Constellation::Gps, prn: 8 };

    let locked_epoch = tracking_epoch(&config, sat, 70, 45.0);
    let mut dll_unlocked_epoch = tracking_epoch(&config, sat, 70, 45.0);
    dll_unlocked_epoch.dll_lock = false;
    let mut guarded_epoch = tracking_epoch(&config, sat, 70, 45.0);
    guarded_epoch.anti_false_lock = true;

    let locked = observation_variance(&config, locked_epoch);
    let dll_unlocked = observation_variance(&config, dll_unlocked_epoch);
    let guarded = observation_variance(&config, guarded_epoch);

    assert!(
        dll_unlocked > locked,
        "loss of DLL lock should inflate pseudorange variance: locked={locked} dll_unlocked={dll_unlocked}"
    );
    assert!(
        guarded > locked,
        "lower lock quality should inflate pseudorange variance: locked={locked} guarded={guarded}"
    );
}
