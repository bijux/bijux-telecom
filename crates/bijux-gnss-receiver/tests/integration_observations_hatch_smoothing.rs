#![allow(missing_docs)]

use bijux_gnss_core::api::{
    Chips, Constellation, Cycles, Epoch, Hertz, ReceiverSampleTrace, SatId, SignalDelayAlignment,
    SignalSpec, TrackEpoch, GPS_L1_CA_CARRIER_HZ,
};
use bijux_gnss_receiver::api::{
    carrier_hz_from_doppler_hz, observation_artifacts_from_tracking_results,
    observations_from_tracking_results, ReceiverPipelineConfig, TrackingResult,
};
use bijux_gnss_signal::api::{
    resolved_signal_registry_entry, samples_per_code, signal_spec_gps_l1_ca, signal_spec_gps_l2c,
};

const HATCH_OBSERVATION_CN0_DBHZ: f64 = 60.0;
const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;

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

fn signal_code_model(sat: SatId, signal: SignalSpec) -> (usize, f64) {
    let entry = resolved_signal_registry_entry(sat, signal.band, signal.code, None)
        .expect("signal registry lookup")
        .expect("signal registry entry");
    let component = entry.default_component().expect("default signal component");
    (component.primary_code_chips as usize, component.primary_code_rate_hz)
}

fn signal_tracking_code_phase_samples(
    config: &ReceiverPipelineConfig,
    sat: SatId,
    signal: SignalSpec,
    pseudorange_m: f64,
) -> (u64, f64) {
    let (code_length, code_rate_hz) = signal_code_model(sat, signal);
    let code_length_chips = code_length as f64;
    let total_chips = pseudorange_m / SPEED_OF_LIGHT_MPS * code_rate_hz;
    let whole_code_periods = (total_chips / code_length_chips).floor() as u64;
    let code_delay_chips = total_chips - whole_code_periods as f64 * code_length_chips;
    assert!(
        code_delay_chips >= 0.0 && code_delay_chips < code_length_chips,
        "code delay must stay within one primary code period"
    );
    let period_samples = samples_per_code(config.sampling_freq_hz, code_rate_hz, code_length);
    let tracking_code_phase_chips =
        (code_length_chips - code_delay_chips).rem_euclid(code_length_chips);
    let code_phase_samples = tracking_code_phase_chips * period_samples as f64 / code_length_chips;
    (whole_code_periods, code_phase_samples)
}

fn signal_center_hz(config: &ReceiverPipelineConfig, signal: SignalSpec) -> f64 {
    config.intermediate_freq_hz + signal.carrier_hz.value() - GPS_L1_CA_CARRIER_HZ.value()
}

fn ionosphere_delay_m(l1_delay_m: f64, signal: SignalSpec) -> f64 {
    l1_delay_m * (GPS_L1_CA_CARRIER_HZ.value() / signal.carrier_hz.value()).powi(2)
}

fn signal_wavelength_m(signal: SignalSpec) -> f64 {
    SPEED_OF_LIGHT_MPS / signal.carrier_hz.value()
}

fn signal_tracking_epoch(
    config: &ReceiverPipelineConfig,
    sat: SatId,
    signal: SignalSpec,
    epoch_idx: u64,
    pseudorange_m: f64,
    carrier_hz: f64,
    carrier_phase_cycles: f64,
) -> TrackEpoch {
    let (whole_code_periods, code_phase_samples) =
        signal_tracking_code_phase_samples(config, sat, signal, pseudorange_m);
    let sample_index = epoch_idx * samples_per_epoch(config);
    TrackEpoch {
        epoch: Epoch { index: epoch_idx },
        sample_index,
        source_time: ReceiverSampleTrace::from_sample_index(sample_index, config.sampling_freq_hz),
        sat,
        signal_band: signal.band,
        signal_code: signal.code,
        glonass_frequency_channel: None,
        prompt_i: 1.0,
        prompt_q: 0.0,
        early_i: 0.0,
        early_q: 0.0,
        late_i: 0.0,
        late_q: 0.0,
        carrier_hz: Hertz(carrier_hz),
        carrier_phase_cycles: Cycles(carrier_phase_cycles),
        code_rate_hz: Hertz(signal.code_rate_hz),
        code_phase_samples: Chips(code_phase_samples),
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
        channel_id: Some(match signal.band {
            bijux_gnss_core::api::SignalBand::L1 => 0,
            bijux_gnss_core::api::SignalBand::L2 => 1,
            _ => 2,
        }),
        channel_uid: format!("Gps-{:02}-{:?}", sat.prn, signal.band),
        tracking_provenance: "integration_observations_hatch_smoothing".to_string(),
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
fn observations_decompose_ionosphere_divergence_without_multipath() {
    let config = hatch_tracking_config();
    let sat = SatId { constellation: Constellation::Gps, prn: 21 };
    let l1 = signal_spec_gps_l1_ca();
    let l2 = signal_spec_gps_l2c();
    let geometry_m = 21_000_000.0;
    let l1_delay_before_m = 4.0;
    let l1_delay_after_m = 4.6;
    let dt_s = samples_per_epoch(&config) as f64 / config.sampling_freq_hz;

    let l1_before_phase_cycles =
        (geometry_m - ionosphere_delay_m(l1_delay_before_m, l1)) / signal_wavelength_m(l1);
    let l1_after_phase_cycles =
        (geometry_m - ionosphere_delay_m(l1_delay_after_m, l1)) / signal_wavelength_m(l1);
    let l2_before_phase_cycles =
        (geometry_m - ionosphere_delay_m(l1_delay_before_m, l2)) / signal_wavelength_m(l2);
    let l2_after_phase_cycles =
        (geometry_m - ionosphere_delay_m(l1_delay_after_m, l2)) / signal_wavelength_m(l2);
    let l1_carrier_hz =
        signal_center_hz(&config, l1) + (l1_after_phase_cycles - l1_before_phase_cycles) / dt_s;
    let l2_carrier_hz =
        signal_center_hz(&config, l2) + (l2_after_phase_cycles - l2_before_phase_cycles) / dt_s;

    let report = observation_artifacts_from_tracking_results(
        &config,
        &[
            observation_track(
                sat,
                vec![
                    signal_tracking_epoch(
                        &config,
                        sat,
                        l1,
                        70,
                        geometry_m + ionosphere_delay_m(l1_delay_before_m, l1),
                        l1_carrier_hz,
                        l1_before_phase_cycles,
                    ),
                    signal_tracking_epoch(
                        &config,
                        sat,
                        l1,
                        71,
                        geometry_m + ionosphere_delay_m(l1_delay_after_m, l1),
                        l1_carrier_hz,
                        l1_after_phase_cycles,
                    ),
                ],
            ),
            observation_track(
                sat,
                vec![
                    signal_tracking_epoch(
                        &config,
                        sat,
                        l2,
                        70,
                        geometry_m + ionosphere_delay_m(l1_delay_before_m, l2),
                        l2_carrier_hz,
                        l2_before_phase_cycles,
                    ),
                    signal_tracking_epoch(
                        &config,
                        sat,
                        l2,
                        71,
                        geometry_m + ionosphere_delay_m(l1_delay_after_m, l2),
                        l2_carrier_hz,
                        l2_after_phase_cycles,
                    ),
                ],
            ),
        ],
        1,
    );
    assert!(
        report.events.iter().all(|event| !matches!(
            event.severity,
            bijux_gnss_core::api::DiagnosticSeverity::Error
        )),
        "{:?}",
        report.events
    );
    let epoch = report.output.epochs.iter().find(|epoch| epoch.epoch_idx == 71).expect("epoch");
    let l1_sat = epoch
        .sats
        .iter()
        .find(|sat| sat.signal_id.band == bijux_gnss_core::api::SignalBand::L1)
        .expect("L1 observation");
    let divergence =
        l1_sat.metadata.code_carrier_divergence.expect("code-carrier divergence decomposition");
    let expected_ionosphere_m = 2.0 * (l1_delay_after_m - l1_delay_before_m);

    assert!(!l1_sat.multipath_suspect, "{l1_sat:?}");
    assert!(
        (divergence.expected_ionosphere_m - expected_ionosphere_m).abs() < 1.0e-3,
        "divergence={divergence:?} expected_ionosphere_m={expected_ionosphere_m}"
    );
    assert!(divergence.multipath_m.abs() < 1.0e-9);
    assert!(divergence.unexplained_m.abs() < 1.0e-3);
    assert!(
        (divergence.jump_m - expected_ionosphere_m).abs() < 1.0e-3,
        "divergence={divergence:?} expected_ionosphere_m={expected_ionosphere_m}"
    );
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
