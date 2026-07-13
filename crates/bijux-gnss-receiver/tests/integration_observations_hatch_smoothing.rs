#![allow(missing_docs)]

use bijux_gnss_core::api::{
    Chips, Constellation, Cycles, Epoch, Hertz, ReceiverSampleTrace, SatId, SignalDelayAlignment,
    TrackEpoch,
};
use bijux_gnss_receiver::api::{
    carrier_hz_from_doppler_hz, observations_from_tracking_results, ReceiverPipelineConfig,
    TrackingResult,
};

const HATCH_OBSERVATION_CN0_DBHZ: f64 = 60.0;

fn hatch_tracking_config() -> ReceiverPipelineConfig {
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
    carrier_hz: f64,
    carrier_phase_cycles: f64,
    whole_code_periods: u64,
    code_phase_samples: f64,
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
        carrier_hz: Hertz(carrier_hz),
        carrier_phase_cycles: Cycles(carrier_phase_cycles),
        code_rate_hz: Hertz(config.code_freq_basis_hz),
        code_phase_samples: Chips(tracking_code_phase_samples(config, code_phase_samples)),
        lock: true,
        cn0_dbhz: HATCH_OBSERVATION_CN0_DBHZ,
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
        lock_state_reason: None,
        channel_id: Some(0),
        channel_uid: format!("Gps-{:02}-ch00", sat.prn),
        tracking_provenance: "integration_observations_hatch_smoothing".to_string(),
        tracking_assumptions: None,
        signal_delay_alignment: Some(SignalDelayAlignment {
            whole_code_periods,
            source: "synthetic_truth".to_string(),
        }),
        tracking_uncertainty: None,
        processing_ms: None,
    }
}

fn tracking_code_phase_samples(
    config: &ReceiverPipelineConfig,
    aligned_code_phase_samples: f64,
) -> f64 {
    (samples_per_epoch(config) as f64 - aligned_code_phase_samples)
        .rem_euclid(samples_per_epoch(config) as f64)
}

fn aligned_pseudorange_m(
    config: &ReceiverPipelineConfig,
    whole_code_periods: u64,
    code_phase_samples: f64,
) -> f64 {
    let code_phase_chips =
        code_phase_samples / samples_per_epoch(config) as f64 * config.code_length as f64;
    ((whole_code_periods as f64 * config.code_length as f64) + code_phase_chips)
        / config.code_freq_basis_hz
        * 299_792_458.0
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
fn observations_hold_hatch_reset_across_unlock_boundaries() {
    let config = hatch_tracking_config();
    let sat = SatId { constellation: Constellation::Gps, prn: 19 };
    let carrier_hz = carrier_hz_from_doppler_hz(config.intermediate_freq_hz, 125.0);
    let mut unlocked_epoch = aligned_tracking_epoch(&config, sat, 71, carrier_hz, 10.125, 68, 0.0);
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
                aligned_tracking_epoch(&config, sat, 70, carrier_hz, 10.0, 68, 0.0),
                unlocked_epoch,
                aligned_tracking_epoch(&config, sat, 72, carrier_hz, 10.250, 68, 0.0),
            ],
        )],
        10,
    );
    let sats = report.output.iter().map(|epoch| &epoch.sats[0]).collect::<Vec<_>>();

    assert_eq!(
        sats.iter().map(|sat| sat.metadata.smoothing_age).collect::<Vec<_>>(),
        vec![1, 0, 1]
    );
    assert_eq!(
        sats.iter().map(|sat| sat.metadata.smoothing_resets).collect::<Vec<_>>(),
        vec![0, 1, 1]
    );
}

#[test]
fn observations_restart_hatch_smoothing_when_alignment_diverges() {
    let config = hatch_tracking_config();
    let sat = SatId { constellation: Constellation::Gps, prn: 20 };
    let carrier_hz = carrier_hz_from_doppler_hz(config.intermediate_freq_hz, 125.0);
    let expected_raw = aligned_pseudorange_m(&config, 68, 0.02);

    let report = observations_from_tracking_results(
        &config,
        &[observation_track(
            sat,
            vec![
                aligned_tracking_epoch(&config, sat, 70, carrier_hz, 10.0, 68, 0.0),
                aligned_tracking_epoch(&config, sat, 71, carrier_hz, 10.125, 68, 0.0),
                aligned_tracking_epoch(&config, sat, 72, carrier_hz, 10.250, 68, 0.02),
                aligned_tracking_epoch(&config, sat, 73, carrier_hz, 10.375, 68, 0.02),
            ],
        )],
        10,
    );
    let sats = report.output.iter().map(|epoch| &epoch.sats[0]).collect::<Vec<_>>();

    assert_eq!(
        sats.iter().map(|sat| sat.metadata.smoothing_age).collect::<Vec<_>>(),
        vec![1, 2, 1, 2]
    );
    assert_eq!(
        sats.iter().map(|sat| sat.metadata.smoothing_resets).collect::<Vec<_>>(),
        vec![0, 0, 1, 1]
    );
    assert!(sats[2].lock_flags.cycle_slip);
    assert!((sats[2].pseudorange_m.0 - expected_raw).abs() <= f64::EPSILON);
}
