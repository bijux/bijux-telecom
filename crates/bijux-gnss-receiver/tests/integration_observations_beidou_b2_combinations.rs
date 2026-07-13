#![allow(missing_docs)]

use bijux_gnss_core::api::{
    signal_spec_beidou_b1i, signal_spec_beidou_b2i, Chips, Constellation, Cycles, Epoch, Hertz,
    Meters, ReceiverSampleTrace, SatId, SignalBand, SignalDelayAlignment, SignalSpec, TrackEpoch,
};
use bijux_gnss_nav::api::{
    combinations_from_obs_epochs, iono_free_code_from_obs_epochs, iono_free_phase_from_obs_epochs,
};
use bijux_gnss_receiver::api::{
    carrier_hz_from_doppler_hz, observation_artifacts_from_tracking_results,
    ReceiverPipelineConfig, TrackingResult,
};

fn beidou_observation_config() -> ReceiverPipelineConfig {
    ReceiverPipelineConfig {
        sampling_freq_hz: 2_046_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 2_046_000.0,
        code_length: 2046,
        channels: 2,
        ..ReceiverPipelineConfig::default()
    }
}

fn aligned_code_phase_samples(
    config: &ReceiverPipelineConfig,
    code_rate_hz: f64,
    code_length: usize,
    aligned_code_phase_chips: f64,
) -> f64 {
    let samples_per_chip = config.sampling_freq_hz / code_rate_hz;
    let period_samples = samples_per_chip * code_length as f64;
    let aligned_code_phase_samples = aligned_code_phase_chips * samples_per_chip;
    (period_samples - aligned_code_phase_samples).rem_euclid(period_samples)
}

fn beidou_tracking_result(
    config: &ReceiverPipelineConfig,
    sat: SatId,
    signal: bijux_gnss_core::api::SignalSpec,
    code_length: usize,
    whole_code_periods: u64,
    aligned_code_phase_chips: f64,
) -> TrackingResult {
    let tracked_center_hz = config.intermediate_freq_hz
        + (signal.carrier_hz.value() - bijux_gnss_core::api::GPS_L1_CA_CARRIER_HZ.value());
    let epoch = TrackEpoch {
        epoch: Epoch { index: 0 },
        sample_index: 0,
        source_time: ReceiverSampleTrace::from_sample_index(0, config.sampling_freq_hz),
        sat,
        signal_band: signal.band,
        glonass_frequency_channel: None,
        prompt_i: 1.0,
        prompt_q: 0.0,
        early_i: 0.0,
        early_q: 0.0,
        late_i: 0.0,
        late_q: 0.0,
        carrier_hz: Hertz(carrier_hz_from_doppler_hz(tracked_center_hz, 0.0)),
        carrier_phase_cycles: Cycles(0.0),
        code_rate_hz: Hertz(signal.code_rate_hz),
        code_phase_samples: Chips(aligned_code_phase_samples(
            config,
            signal.code_rate_hz,
            code_length,
            aligned_code_phase_chips,
        )),
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
        channel_uid: format!("beidou-{:02}-{:?}", sat.prn, signal.band),
        tracking_provenance: "integration_observations_beidou_b2_combinations".to_string(),
        tracking_assumptions: None,
        signal_delay_alignment: Some(SignalDelayAlignment {
            whole_code_periods,
            source: "synthetic_truth".to_string(),
        }),
        tracking_uncertainty: None,
        processing_ms: None,
    };

    TrackingResult {
        sat,
        carrier_hz: epoch.carrier_hz.0,
        code_phase_samples: epoch.code_phase_samples.0,
        acquisition_hypothesis: "accepted".to_string(),
        acquisition_score: 1.0,
        acquisition_code_phase_samples: epoch.code_phase_samples.0.round() as usize,
        acquisition_carrier_hz: epoch.carrier_hz.0,
        acq_to_track_state: "accepted".to_string(),
        epochs: vec![epoch],
        transitions: Vec::new(),
    }
}

fn dispersive_delay_at_band(
    iono_b1_m: f64,
    b1_signal: SignalSpec,
    target_signal: SignalSpec,
) -> f64 {
    iono_b1_m * (b1_signal.carrier_hz.value() * b1_signal.carrier_hz.value())
        / (target_signal.carrier_hz.value() * target_signal.carrier_hz.value())
}

#[test]
fn generated_beidou_b1_b2_observations_support_dual_frequency_combinations() {
    let config = beidou_observation_config();
    let sat = SatId { constellation: Constellation::Beidou, prn: 11 };
    let b1 = beidou_tracking_result(&config, sat, signal_spec_beidou_b1i(), 2046, 10, 512.0);
    let b2 = beidou_tracking_result(&config, sat, signal_spec_beidou_b2i(), 2046, 7, 768.0);

    let artifacts = observation_artifacts_from_tracking_results(&config, &[b1, b2], 10);

    assert_eq!(artifacts.output.epochs.len(), 1);
    assert_eq!(artifacts.output.epochs[0].sats.len(), 2);
    assert!(artifacts.output.epochs[0]
        .sats
        .iter()
        .any(|satellite| satellite.signal_id.band == SignalBand::B1));
    assert!(artifacts.output.epochs[0]
        .sats
        .iter()
        .any(|satellite| satellite.signal_id.band == SignalBand::B2));

    let combinations =
        combinations_from_obs_epochs(&artifacts.output.epochs, SignalBand::B1, SignalBand::B2);
    let iono_free_code =
        iono_free_code_from_obs_epochs(&artifacts.output.epochs, SignalBand::B1, SignalBand::B2);
    let iono_free_phase =
        iono_free_phase_from_obs_epochs(&artifacts.output.epochs, SignalBand::B1, SignalBand::B2);

    assert_eq!(combinations.len(), 1);
    assert_eq!(iono_free_code.len(), 1);
    assert_eq!(iono_free_phase.len(), 1);
}

#[test]
fn generated_beidou_b1_b2_observations_reduce_synthetic_ionospheric_delay() {
    let config = beidou_observation_config();
    let sat = SatId { constellation: Constellation::Beidou, prn: 11 };
    let b1_signal = signal_spec_beidou_b1i();
    let b2_signal = signal_spec_beidou_b2i();
    let b1 = beidou_tracking_result(&config, sat, b1_signal, 2046, 10, 512.0);
    let b2 = beidou_tracking_result(&config, sat, b2_signal, 2046, 7, 768.0);

    let artifacts = observation_artifacts_from_tracking_results(&config, &[b1, b2], 10);
    let mut delayed_epoch = artifacts.output.epochs[0].clone();
    let base_range_m = 24_100_000.0;
    let iono_b1_m = 3.75;
    let iono_b2_m = dispersive_delay_at_band(iono_b1_m, b1_signal, b2_signal);

    for satellite in &mut delayed_epoch.sats {
        satellite.pseudorange_m = Meters(match satellite.signal_id.band {
            SignalBand::B1 => base_range_m + iono_b1_m,
            SignalBand::B2 => base_range_m + iono_b2_m,
            band => panic!("unexpected band in BeiDou observation epoch: {band:?}"),
        });
    }

    let iono_free_code =
        iono_free_code_from_obs_epochs(&[delayed_epoch], SignalBand::B1, SignalBand::B2);

    assert_eq!(iono_free_code.len(), 1);
    assert_eq!(iono_free_code[0].status, "ok");
    assert!((iono_free_code[0].code_m.expect("iono-free code") - base_range_m).abs() < 1.0e-6);
}
