#![allow(missing_docs)]

use bijux_gnss_core::api::{
    Chips, Constellation, Cycles, Epoch, Hertz, Meters, ReceiverSampleTrace, SatId, SignalBand,
    SignalDelayAlignment, SignalSpec, TrackEpoch,
};
use bijux_gnss_nav::api::{
    combinations_from_obs_epochs, iono_free_code_from_obs_epochs, iono_free_phase_from_obs_epochs,
};
use bijux_gnss_receiver::api::{
    carrier_hz_from_doppler_hz, observation_artifacts_from_tracking_results,
    ReceiverPipelineConfig, TrackingResult,
};
use bijux_gnss_signal::api::{signal_spec_galileo_e1b, signal_spec_galileo_e5a};

fn galileo_observation_config() -> ReceiverPipelineConfig {
    ReceiverPipelineConfig {
        sampling_freq_hz: 10_230_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 4092,
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

fn galileo_tracking_result(
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
        signal_code: bijux_gnss_core::api::SignalCode::Unknown,
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
        channel_uid: format!("galileo-{:02}-{:?}", sat.prn, signal.band),
        tracking_provenance: "integration_observations_galileo_e5_combinations".to_string(),
        tracking_assumptions: None,
        signal_delay_alignment: Some(SignalDelayAlignment {
            whole_code_periods,
            sample_delay_samples: 0,
            source: "synthetic_truth".to_string(),
        }),
        transmit_time: None,
        tracking_uncertainty: Some(bijux_gnss_core::api::TrackingUncertainty {
            code_phase_samples: 0.05,
            carrier_phase_cycles: 0.02,
            doppler_hz: 1.0,
            cn0_dbhz: 0.5,
        }),
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
    iono_e1_m: f64,
    e1_signal: SignalSpec,
    target_signal: SignalSpec,
) -> f64 {
    iono_e1_m * (e1_signal.carrier_hz.value() * e1_signal.carrier_hz.value())
        / (target_signal.carrier_hz.value() * target_signal.carrier_hz.value())
}

#[test]
fn generated_galileo_e1_e5_observations_support_dual_frequency_combinations() {
    let config = galileo_observation_config();
    let sat = SatId { constellation: Constellation::Galileo, prn: 11 };
    let e1 = galileo_tracking_result(&config, sat, signal_spec_galileo_e1b(), 4092, 10, 512.0);
    let e5 = galileo_tracking_result(&config, sat, signal_spec_galileo_e5a(), 10230, 4, 2048.0);

    let artifacts = observation_artifacts_from_tracking_results(&config, &[e1, e5], 10);

    assert_eq!(artifacts.output.epochs.len(), 1);
    assert_eq!(artifacts.output.epochs[0].sats.len(), 2);
    assert!(artifacts.output.epochs[0]
        .sats
        .iter()
        .any(|satellite| satellite.signal_id.band == SignalBand::E1));
    assert!(artifacts.output.epochs[0]
        .sats
        .iter()
        .any(|satellite| satellite.signal_id.band == SignalBand::E5));

    let combinations =
        combinations_from_obs_epochs(&artifacts.output.epochs, SignalBand::E1, SignalBand::E5);
    let iono_free_code =
        iono_free_code_from_obs_epochs(&artifacts.output.epochs, SignalBand::E1, SignalBand::E5);
    let iono_free_phase =
        iono_free_phase_from_obs_epochs(&artifacts.output.epochs, SignalBand::E1, SignalBand::E5);

    assert_eq!(combinations.len(), 1);
    assert_eq!(iono_free_code.len(), 1);
    assert_eq!(iono_free_phase.len(), 1);
}

#[test]
fn generated_galileo_e1_e5_observations_reduce_synthetic_ionospheric_delay() {
    let config = galileo_observation_config();
    let sat = SatId { constellation: Constellation::Galileo, prn: 11 };
    let e1_signal = signal_spec_galileo_e1b();
    let e5_signal = signal_spec_galileo_e5a();
    let e1 = galileo_tracking_result(&config, sat, e1_signal, 4092, 10, 512.0);
    let e5 = galileo_tracking_result(&config, sat, e5_signal, 10230, 4, 2048.0);

    let artifacts = observation_artifacts_from_tracking_results(&config, &[e1, e5], 10);
    let mut delayed_epoch = artifacts.output.epochs[0].clone();
    let base_range_m = 24_000_000.0;
    let iono_e1_m = 4.5;
    let iono_e5_m = dispersive_delay_at_band(iono_e1_m, e1_signal, e5_signal);

    for satellite in &mut delayed_epoch.sats {
        satellite.pseudorange_m = Meters(match satellite.signal_id.band {
            SignalBand::E1 => base_range_m + iono_e1_m,
            SignalBand::E5 => base_range_m + iono_e5_m,
            band => panic!("unexpected band in Galileo observation epoch: {band:?}"),
        });
    }

    let iono_free_code =
        iono_free_code_from_obs_epochs(&[delayed_epoch], SignalBand::E1, SignalBand::E5);

    assert_eq!(iono_free_code.len(), 1);
    assert_eq!(iono_free_code[0].status, "ok");
    assert!((iono_free_code[0].code_m.expect("iono-free code") - base_range_m).abs() < 1.0e-6);
}
