#![allow(missing_docs)]

mod support;

use bijux_gnss_core::api::{
    AcqHypothesis, AcqResult, Constellation, Hertz, ReceiverSampleTrace, SatId, SignalBand,
    SignalCode,
};
use bijux_gnss_receiver::api::{
    sim::{
        expected_acquisition_code_phase_samples, generate_l1_ca_multi_with_receiver_oscillator,
        SyntheticReceiverOscillatorModel, SyntheticReceiverOscillatorNoiseModel, SyntheticScenario,
        SyntheticSignalParams,
    },
    ReceiverPipelineConfig, ReceiverRuntime, TrackingEngine,
};
use support::tracking_truth::{
    cycle_slip_rate, stable_carrier_phase_step_jitter_cycles, stable_pll_error_rms,
    stable_tracking_window,
};

const OSCILLATOR_NOISE_CN0_DB_HZ: f32 = 72.0;
const OSCILLATOR_NOISE_DURATION_S: f64 = 0.100;
const OSCILLATOR_NOISE_MIN_STABLE_EPOCHS: usize = 6;
const BOUNDED_WHITE_PHASE_STD_RAD: f64 = 0.018;
const BOUNDED_WHITE_FREQUENCY_STD_HZ: f64 = 0.18;
const BOUNDED_RANDOM_WALK_FREQUENCY_STEP_STD_HZ: f64 = 0.035;
const EXCESSIVE_WHITE_PHASE_STD_RAD: f64 = 0.28;
const BOUNDED_MAX_CYCLE_SLIP_RATE: f64 = 0.05;

#[derive(Debug)]
struct OscillatorTrackingMetrics {
    stable_epoch_count: usize,
    pll_error_rms: f64,
    carrier_phase_step_jitter_cycles: f64,
    cycle_slip_rate: f64,
}

fn oscillator_noise_tracking_config() -> ReceiverPipelineConfig {
    ReceiverPipelineConfig {
        sampling_freq_hz: 1_023_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 4,
        early_late_spacing_chips: 0.5,
        dll_bw_hz: 2.0,
        pll_bw_hz: 18.0,
        fll_bw_hz: 8.0,
        ..ReceiverPipelineConfig::default()
    }
}

fn accepted_acquisition(sat: SatId, doppler_hz: f64, code_phase_samples: usize) -> AcqResult {
    AcqResult {
        sat,
        signal_band: SignalBand::L1,
        signal_code: SignalCode::Unknown,
        glonass_frequency_channel: None,
        source_time: ReceiverSampleTrace::default(),
        candidate_rank: 1,
        is_primary_candidate: true,
        doppler_hz: Hertz(doppler_hz),
        doppler_rate_hz_per_s: 0.0,
        carrier_hz: Hertz(doppler_hz),
        code_phase_samples,
        peak: 1.0,
        second_peak: 0.1,
        mean: 0.01,
        peak_mean_ratio: 20.0,
        peak_second_ratio: 10.0,
        cn0_proxy: OSCILLATOR_NOISE_CN0_DB_HZ,
        score: 1.0,
        hypothesis: AcqHypothesis::Accepted,
        assumptions: None,
        evidence: Vec::new(),
        threshold_provenance: None,
        explain_selection_reason: Some("oscillator_noise_tracking_start".to_string()),
        doppler_refinement: None,
        code_phase_refinement: None,
        signal_delay_alignment: None,
        uncertainty: None,
    }
}

fn oscillator_noise_signal(
    sat: SatId,
    doppler_hz: f64,
    code_phase_chips: f64,
) -> SyntheticSignalParams {
    SyntheticSignalParams {
        sat,
        glonass_frequency_channel: None,
        signal_band: SignalBand::L1,
        signal_code: SignalCode::Unknown,
        doppler_hz,
        code_phase_chips,
        carrier_phase_rad: 0.0,
        cn0_db_hz: OSCILLATOR_NOISE_CN0_DB_HZ,
        navigation_data: false.into(),
    }
}

fn oscillator_noise_scenario(signal: SyntheticSignalParams, seed: u64) -> SyntheticScenario {
    SyntheticScenario {
        sample_rate_hz: 1_023_000.0,
        intermediate_freq_hz: 0.0,
        receiver_clock_frequency_bias_hz: 0.0,
        duration_s: OSCILLATOR_NOISE_DURATION_S,
        seed,
        satellites: vec![signal],
        ephemerides: Vec::new(),
        id: "oscillator-noise-tracking".to_string(),
    }
}

fn tracking_metrics_for_oscillator(
    receiver_oscillator: SyntheticReceiverOscillatorModel,
    seed: u64,
) -> OscillatorTrackingMetrics {
    let config = oscillator_noise_tracking_config();
    let sat = SatId { constellation: Constellation::Gps, prn: 26 };
    let doppler_hz = 140.0;
    let code_phase_chips = 291.25;
    let signal = oscillator_noise_signal(sat, doppler_hz, code_phase_chips);
    let scenario = oscillator_noise_scenario(signal, seed);
    let frame =
        generate_l1_ca_multi_with_receiver_oscillator(&config, &scenario, &receiver_oscillator);
    let seeded_code_phase_samples =
        expected_acquisition_code_phase_samples(&config, &frame, code_phase_chips);
    let tracking = TrackingEngine::new(config, ReceiverRuntime::default());
    let tracks = tracking.track_from_acquisition(
        &frame,
        &[accepted_acquisition(sat, doppler_hz, seeded_code_phase_samples)],
    );
    let epochs = &tracks.first().expect("track").epochs;
    let stable_window = stable_tracking_window(epochs, OSCILLATOR_NOISE_MIN_STABLE_EPOCHS);

    OscillatorTrackingMetrics {
        stable_epoch_count: stable_window.len(),
        pll_error_rms: stable_pll_error_rms(epochs, OSCILLATOR_NOISE_MIN_STABLE_EPOCHS)
            .unwrap_or(f64::INFINITY),
        carrier_phase_step_jitter_cycles: stable_carrier_phase_step_jitter_cycles(
            epochs,
            OSCILLATOR_NOISE_MIN_STABLE_EPOCHS,
        )
        .unwrap_or(f64::INFINITY),
        cycle_slip_rate: cycle_slip_rate(epochs),
    }
}

#[test]
fn tracking_error_increases_but_lock_remains_stable_under_bounded_white_phase_noise() {
    let nominal =
        tracking_metrics_for_oscillator(SyntheticReceiverOscillatorModel::default(), 0xA551_2901);
    let white_phase = tracking_metrics_for_oscillator(
        SyntheticReceiverOscillatorModel {
            noise: SyntheticReceiverOscillatorNoiseModel {
                seed: 0xB010_9A51,
                update_interval_samples: 1_023,
                white_phase_std_rad: BOUNDED_WHITE_PHASE_STD_RAD,
                white_frequency_std_hz: 0.0,
                random_walk_frequency_step_std_hz: 0.0,
            },
            ..SyntheticReceiverOscillatorModel::default()
        },
        0xA551_2901,
    );

    assert!(
        white_phase.stable_epoch_count >= OSCILLATOR_NOISE_MIN_STABLE_EPOCHS,
        "bounded white phase noise should retain a stable lock window: white_phase={white_phase:?}, nominal={nominal:?}"
    );
    assert!(
        white_phase.cycle_slip_rate <= BOUNDED_MAX_CYCLE_SLIP_RATE,
        "bounded white phase noise produced excessive slips: white_phase={white_phase:?}, nominal={nominal:?}"
    );
    assert!(
        white_phase.pll_error_rms > nominal.pll_error_rms,
        "white phase noise should increase measured PLL error: white_phase={white_phase:?}, nominal={nominal:?}"
    );
    assert!(
        white_phase.carrier_phase_step_jitter_cycles > nominal.carrier_phase_step_jitter_cycles,
        "white phase noise should increase phase-step jitter: white_phase={white_phase:?}, nominal={nominal:?}"
    );
}

#[test]
fn tracking_error_increases_but_lock_remains_stable_under_bounded_white_frequency_noise() {
    let nominal =
        tracking_metrics_for_oscillator(SyntheticReceiverOscillatorModel::default(), 0xA551_2902);
    let white_frequency = tracking_metrics_for_oscillator(
        SyntheticReceiverOscillatorModel {
            noise: SyntheticReceiverOscillatorNoiseModel {
                seed: 0xF2E1_4451,
                update_interval_samples: 1_023,
                white_phase_std_rad: 0.0,
                white_frequency_std_hz: BOUNDED_WHITE_FREQUENCY_STD_HZ,
                random_walk_frequency_step_std_hz: 0.0,
            },
            ..SyntheticReceiverOscillatorModel::default()
        },
        0xA551_2902,
    );

    assert!(
        white_frequency.stable_epoch_count >= OSCILLATOR_NOISE_MIN_STABLE_EPOCHS,
        "bounded white frequency noise should retain a stable lock window: white_frequency={white_frequency:?}, nominal={nominal:?}"
    );
    assert!(
        white_frequency.cycle_slip_rate <= BOUNDED_MAX_CYCLE_SLIP_RATE,
        "bounded white frequency noise produced excessive slips: white_frequency={white_frequency:?}, nominal={nominal:?}"
    );
    assert!(
        white_frequency.pll_error_rms > nominal.pll_error_rms,
        "white frequency noise should increase measured PLL error: white_frequency={white_frequency:?}, nominal={nominal:?}"
    );
    assert!(
        white_frequency.carrier_phase_step_jitter_cycles
            > nominal.carrier_phase_step_jitter_cycles,
        "white frequency noise should increase phase-step jitter: white_frequency={white_frequency:?}, nominal={nominal:?}"
    );
}

#[test]
fn tracking_error_increases_but_lock_remains_stable_under_bounded_random_walk_frequency_noise() {
    let nominal =
        tracking_metrics_for_oscillator(SyntheticReceiverOscillatorModel::default(), 0xA551_2903);
    let random_walk_frequency = tracking_metrics_for_oscillator(
        SyntheticReceiverOscillatorModel {
            noise: SyntheticReceiverOscillatorNoiseModel {
                seed: 0xD107_7A51,
                update_interval_samples: 1_023,
                white_phase_std_rad: 0.0,
                white_frequency_std_hz: 0.0,
                random_walk_frequency_step_std_hz: BOUNDED_RANDOM_WALK_FREQUENCY_STEP_STD_HZ,
            },
            ..SyntheticReceiverOscillatorModel::default()
        },
        0xA551_2903,
    );

    assert!(
        random_walk_frequency.stable_epoch_count >= OSCILLATOR_NOISE_MIN_STABLE_EPOCHS,
        "bounded random-walk frequency noise should retain a stable lock window: random_walk_frequency={random_walk_frequency:?}, nominal={nominal:?}"
    );
    assert!(
        random_walk_frequency.cycle_slip_rate <= BOUNDED_MAX_CYCLE_SLIP_RATE,
        "bounded random-walk frequency noise produced excessive slips: random_walk_frequency={random_walk_frequency:?}, nominal={nominal:?}"
    );
    assert!(
        random_walk_frequency.pll_error_rms > nominal.pll_error_rms,
        "random-walk frequency noise should increase measured PLL error: random_walk_frequency={random_walk_frequency:?}, nominal={nominal:?}"
    );
    assert!(
        random_walk_frequency.carrier_phase_step_jitter_cycles
            > nominal.carrier_phase_step_jitter_cycles,
        "random-walk frequency noise should increase phase-step jitter: random_walk_frequency={random_walk_frequency:?}, nominal={nominal:?}"
    );
}

#[test]
fn tracking_slip_rate_or_stable_window_responds_to_excessive_white_phase_noise() {
    let nominal =
        tracking_metrics_for_oscillator(SyntheticReceiverOscillatorModel::default(), 0xA551_2904);
    let excessive_white_phase = tracking_metrics_for_oscillator(
        SyntheticReceiverOscillatorModel {
            noise: SyntheticReceiverOscillatorNoiseModel {
                seed: 0xBAAD_9A51,
                update_interval_samples: 1_023,
                white_phase_std_rad: EXCESSIVE_WHITE_PHASE_STD_RAD,
                white_frequency_std_hz: 0.0,
                random_walk_frequency_step_std_hz: 0.0,
            },
            ..SyntheticReceiverOscillatorModel::default()
        },
        0xA551_2904,
    );

    assert!(
        excessive_white_phase.stable_epoch_count < nominal.stable_epoch_count
            || excessive_white_phase.cycle_slip_rate > nominal.cycle_slip_rate,
        "excessive oscillator phase noise must degrade stable tracking or increase slips instead of looking nominal: excessive_white_phase={excessive_white_phase:?}, nominal={nominal:?}"
    );
    assert!(
        excessive_white_phase.pll_error_rms > nominal.pll_error_rms
            || excessive_white_phase.stable_epoch_count == 0,
        "excessive oscillator phase noise must increase measured error when a stable window remains: excessive_white_phase={excessive_white_phase:?}, nominal={nominal:?}"
    );
}
