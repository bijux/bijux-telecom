#![allow(missing_docs)]

use bijux_gnss_core::api::{
    Chips, Constellation, Cycles, Epoch, Hertz, ReceiverSampleTrace, SatId, SignalDelayAlignment,
    TrackEpoch,
};
use bijux_gnss_receiver::api::{
    carrier_hz_from_doppler_hz, observations_from_tracking_results, ReceiverPipelineConfig,
    TrackingResult,
};

const OBSERVATION_CN0_DBHZ: f64 = 60.0;

fn tracking_config() -> ReceiverPipelineConfig {
    ReceiverPipelineConfig {
        sampling_freq_hz: 1_023_000.0,
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
    doppler_hz: f64,
    carrier_phase_cycles: f64,
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
        carrier_hz: Hertz(carrier_hz_from_doppler_hz(config.intermediate_freq_hz, doppler_hz)),
        carrier_phase_cycles: Cycles(carrier_phase_cycles),
        code_rate_hz: Hertz(config.code_freq_basis_hz),
        code_phase_samples: Chips(0.0),
        lock: true,
        cn0_dbhz: OBSERVATION_CN0_DBHZ,
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
        channel_id: Some(0),
        channel_uid: format!("Gps-{:02}-ch00", sat.prn),
        tracking_provenance: "observation_cycle_slip_unlock".to_string(),
        tracking_assumptions: None,
        signal_delay_alignment: Some(SignalDelayAlignment {
            whole_code_periods: 68,
            source: "synthetic_truth".to_string(),
        }),
        tracking_uncertainty: None,
        processing_ms: None,
    }
}

fn observation_track(sat: SatId, epochs: Vec<TrackEpoch>) -> TrackingResult {
    TrackingResult {
        sat,
        carrier_hz: epochs.last().map(|epoch| epoch.carrier_hz.0).unwrap_or_default(),
        code_phase_samples: epochs
            .last()
            .map(|epoch| epoch.code_phase_samples.0)
            .unwrap_or_default(),
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
fn observations_flag_loss_of_lock_cycle_slips_across_unlock_boundary() {
    let config = tracking_config();
    let sat = SatId { constellation: Constellation::Gps, prn: 25 };
    let mut unlocked_epoch = aligned_tracking_epoch(&config, sat, 71, 125.0, 10.125);
    unlocked_epoch.lock = false;
    unlocked_epoch.pll_lock = false;
    unlocked_epoch.dll_lock = false;
    unlocked_epoch.fll_lock = false;
    unlocked_epoch.lock_state = "lost".to_string();
    unlocked_epoch.lock_state_reason = Some("prompt_power_drop".to_string());

    let report = observations_from_tracking_results(
        &config,
        &[observation_track(
            sat,
            vec![
                aligned_tracking_epoch(&config, sat, 70, 125.0, 10.000),
                unlocked_epoch,
                aligned_tracking_epoch(&config, sat, 72, 125.0, 10.250),
            ],
        )],
        10,
    );
    let unlocked = &report.output[1].sats[0];
    let relocked = &report.output[2].sats[0];

    assert!(unlocked.lock_flags.cycle_slip, "{unlocked:?}");
    assert_eq!(unlocked.metadata.observation_lock_state, "cycle_slip");
    assert_eq!(unlocked.metadata.observation_lock_reason.as_deref(), Some("loss_of_lock"));
    assert_eq!(unlocked.metadata.carrier_phase_continuity, "unusable");
    assert!(unlocked.observation_reject_reasons.iter().any(|reason| reason == "loss_of_lock"));

    assert!(relocked.lock_flags.cycle_slip, "{relocked:?}");
    assert_eq!(relocked.metadata.observation_lock_state, "cycle_slip");
    assert_eq!(relocked.metadata.observation_lock_reason.as_deref(), Some("loss_of_lock"));
    assert_eq!(relocked.metadata.carrier_phase_continuity, "reset_after_unlock");
}
