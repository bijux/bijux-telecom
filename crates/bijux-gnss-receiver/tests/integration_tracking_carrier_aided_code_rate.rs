#![allow(missing_docs)]

mod support;

use bijux_gnss_core::api::{
    AcqHypothesis, AcqResult, Constellation, Hertz, ReceiverSampleTrace, SatId, SignalBand,
    SignalCode, SignalSpec, BEIDOU_B1_CARRIER_HZ, GPS_L1_CA_CARRIER_HZ, GPS_L5_CARRIER_HZ,
};
use bijux_gnss_receiver::api::{
    sim::{
        expected_acquisition_code_phase_samples, generate_l1_ca_multi_with_receiver_oscillator,
        SyntheticNavigationData, SyntheticReceiverOscillatorModel, SyntheticScenario,
        SyntheticSignalParams,
    },
    ReceiverPipelineConfig, ReceiverRuntime, TrackingEngine,
};
use bijux_gnss_signal::api::{
    receiver_search_code_phase_samples, shared_path_code_rate_hz, signal_spec_beidou_b1i,
    signal_spec_gps_l5_i,
};

use support::tracking_truth::{post_lock_code_phase_errors_samples, stable_tracking_window};

const CARRIER_AID_DURATION_S: f64 = 0.060;
const CARRIER_AID_CN0_DB_HZ: f32 = 75.0;
const CARRIER_AID_MIN_STABLE_EPOCHS: usize = 20;
const CARRIER_AID_CODE_PHASE_ERROR_MAX_SAMPLES: f64 = 2.0;
const CARRIER_AID_CODE_RATE_ERROR_MAX_HZ: f64 = 2.0;
const CARRIER_AID_DRIFT_CODE_PHASE_ERROR_MAX_SAMPLES: f64 = 2.5;
const CARRIER_AID_DRIFT_CODE_RATE_ERROR_MAX_HZ: f64 = 3.0;

fn accepted_acquisition(
    sat: SatId,
    signal_band: SignalBand,
    signal_code: SignalCode,
    doppler_hz: f64,
    carrier_hz: f64,
    code_phase_samples: usize,
) -> AcqResult {
    AcqResult {
        sat,
        signal_band,
        signal_code,
        glonass_frequency_channel: None,
        source_time: ReceiverSampleTrace::default(),
        candidate_rank: 1,
        is_primary_candidate: true,
        doppler_hz: Hertz(doppler_hz),
        doppler_rate_hz_per_s: 0.0,
        carrier_hz: Hertz(carrier_hz),
        code_phase_samples,
        peak: 1.0,
        second_peak: 0.1,
        mean: 0.01,
        peak_mean_ratio: 20.0,
        peak_second_ratio: 10.0,
        cn0_proxy: CARRIER_AID_CN0_DB_HZ,
        score: 1.0,
        hypothesis: AcqHypothesis::Accepted,
        assumptions: None,
        evidence: Vec::new(),
        threshold_provenance: None,
        explain_selection_reason: Some("carrier_aided_tracking_seed".to_string()),
        doppler_refinement: None,
        code_phase_refinement: None,
        signal_delay_alignment: None,
        uncertainty: None,
    }
}

fn gps_l5_tracking_config() -> ReceiverPipelineConfig {
    ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: GPS_L1_CA_CARRIER_HZ.value() - GPS_L5_CARRIER_HZ.value(),
        code_freq_basis_hz: 10_230_000.0,
        code_length: 10_230,
        channels: 4,
        early_late_spacing_chips: 0.5,
        dll_bw_hz: 2.0,
        pll_bw_hz: 15.0,
        fll_bw_hz: 10.0,
        ..ReceiverPipelineConfig::default()
    }
}

fn beidou_b1_tracking_config() -> ReceiverPipelineConfig {
    ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: GPS_L1_CA_CARRIER_HZ.value() - BEIDOU_B1_CARRIER_HZ.value(),
        code_freq_basis_hz: 2_046_000.0,
        code_length: 2_046,
        channels: 4,
        early_late_spacing_chips: 0.5,
        dll_bw_hz: 2.0,
        pll_bw_hz: 15.0,
        fll_bw_hz: 10.0,
        ..ReceiverPipelineConfig::default()
    }
}

fn synthetic_signal_with_carrier_aided_code_rate(
    config: &ReceiverPipelineConfig,
    signal: SyntheticSignalParams,
    receiver_oscillator: &SyntheticReceiverOscillatorModel,
) -> bijux_gnss_core::api::SamplesFrame {
    generate_l1_ca_multi_with_receiver_oscillator(
        config,
        &SyntheticScenario {
            sample_rate_hz: config.sampling_freq_hz,
            intermediate_freq_hz: config.intermediate_freq_hz,
            receiver_clock_frequency_bias_hz: 0.0,
            duration_s: CARRIER_AID_DURATION_S,
            seed: 0xC0DE_A1D5,
            satellites: vec![signal],
            ephemerides: Vec::new(),
            id: "carrier-aided-code-rate".to_string(),
        },
        receiver_oscillator,
    )
}

fn oscillator_effective_elapsed_s(
    sample_index: u64,
    sample_rate_hz: f64,
    receiver_oscillator: &SyntheticReceiverOscillatorModel,
) -> f64 {
    let nominal_elapsed_s = sample_index as f64 / sample_rate_hz;
    nominal_elapsed_s * (1.0 + receiver_oscillator.sampling_clock_fractional_error)
        + 0.5
            * receiver_oscillator.sampling_clock_fractional_drift_per_s
            * nominal_elapsed_s
            * nominal_elapsed_s
}

fn dynamic_expected_code_rate_hz(
    sample_index: u64,
    sample_rate_hz: f64,
    signal: SignalSpec,
    receiver_oscillator: &SyntheticReceiverOscillatorModel,
) -> f64 {
    signal.code_rate_hz
        * (1.0
            + receiver_oscillator.sampling_clock_fractional_error
            + receiver_oscillator.sampling_clock_fractional_drift_per_s
                * (sample_index as f64 / sample_rate_hz))
}

fn mean(values: &[f64]) -> f64 {
    assert!(!values.is_empty(), "mean requires at least one value");
    values.iter().sum::<f64>() / values.len() as f64
}

fn assert_carrier_aiding_reduces_dynamic_code_rate_error(
    stable_code_rate_errors_hz: &[f64],
    stable_epochs: &[bijux_gnss_core::api::TrackEpoch],
    config: &ReceiverPipelineConfig,
    signal: SignalSpec,
    receiver_oscillator: &SyntheticReceiverOscillatorModel,
) {
    let nominal_code_rate_errors_hz = stable_epochs
        .iter()
        .map(|epoch| {
            (signal.code_rate_hz
                - dynamic_expected_code_rate_hz(
                    epoch.sample_index,
                    config.sampling_freq_hz,
                    signal,
                    receiver_oscillator,
                ))
            .abs()
        })
        .collect::<Vec<_>>();
    let aided_mean_hz = mean(stable_code_rate_errors_hz);
    let nominal_mean_hz = mean(&nominal_code_rate_errors_hz);

    assert!(
        aided_mean_hz * 4.0 < nominal_mean_hz,
        "carrier aiding must materially reduce dynamic code-rate error: aided_mean_hz={aided_mean_hz} nominal_mean_hz={nominal_mean_hz} aided_errors={stable_code_rate_errors_hz:?} nominal_errors={nominal_code_rate_errors_hz:?}",
    );
}

fn dynamic_expected_code_phase_samples(
    config: &ReceiverPipelineConfig,
    sample_index: u64,
    initial_code_phase_chips: f64,
    signal: SignalSpec,
    receiver_oscillator: &SyntheticReceiverOscillatorModel,
) -> f64 {
    let code_phase_chips = initial_code_phase_chips
        + signal.code_rate_hz
            * oscillator_effective_elapsed_s(
                sample_index,
                config.sampling_freq_hz,
                receiver_oscillator,
            );
    receiver_search_code_phase_samples(
        config.sampling_freq_hz,
        config.code_freq_basis_hz,
        config.code_length,
        sample_index,
        code_phase_chips,
    )
    .expect("dynamic carrier-aided tracking scenario requires a valid code phase")
}

#[test]
fn tracking_holds_gps_l5_code_lock_with_carrier_aided_code_rate() {
    let config = gps_l5_tracking_config();
    let sat = SatId { constellation: Constellation::Gps, prn: 18 };
    let signal = signal_spec_gps_l5_i();
    let carrier_doppler_hz = 1_500.0;
    let code_phase_chips = 1_537.25;
    let receiver_oscillator = SyntheticReceiverOscillatorModel {
        carrier_frequency_bias_hz: carrier_doppler_hz,
        carrier_frequency_drift_hz_per_s: 0.0,
        sampling_clock_fractional_error: carrier_doppler_hz / signal.carrier_hz.value(),
        ..SyntheticReceiverOscillatorModel::default()
    };
    let frame = synthetic_signal_with_carrier_aided_code_rate(
        &config,
        SyntheticSignalParams {
            sat,
            glonass_frequency_channel: None,
            signal_band: SignalBand::L5,
            signal_code: SignalCode::L5I,
            doppler_hz: 0.0,
            code_phase_chips,
            carrier_phase_rad: 0.0,
            cn0_db_hz: CARRIER_AID_CN0_DB_HZ,
            navigation_data: SyntheticNavigationData::from(false),
        },
        &receiver_oscillator,
    );
    let seeded_code_phase_samples =
        expected_acquisition_code_phase_samples(&config, &frame, code_phase_chips);
    let tracking = TrackingEngine::new(config.clone(), ReceiverRuntime::default());
    let tracks = tracking.track_from_acquisition(
        &frame,
        &[accepted_acquisition(
            sat,
            SignalBand::L5,
            SignalCode::L5I,
            carrier_doppler_hz,
            carrier_doppler_hz,
            seeded_code_phase_samples,
        )],
    );
    let epochs = &tracks.first().expect("track").epochs;
    let stable_epochs = stable_tracking_window(epochs, CARRIER_AID_MIN_STABLE_EPOCHS);
    let expected_code_rate_hz = shared_path_code_rate_hz(carrier_doppler_hz, signal, signal)
        .expect("carrier-aided code rate");
    let post_lock_errors_samples =
        post_lock_code_phase_errors_samples(&config, epochs, seeded_code_phase_samples as f64);
    let stable_code_rate_errors_hz = stable_epochs
        .iter()
        .map(|epoch| (epoch.code_rate_hz.0 - expected_code_rate_hz).abs())
        .collect::<Vec<_>>();

    assert!(
        stable_epochs.len() >= CARRIER_AID_MIN_STABLE_EPOCHS,
        "tracking did not sustain enough GPS L5 lock epochs: epochs={epochs:?}",
    );
    assert!(
        stable_epochs
            .iter()
            .all(|epoch| epoch.lock && epoch.dll_lock && epoch.pll_lock && epoch.fll_lock),
        "GPS L5 stable tracking window lost lock flags: epochs={epochs:?}",
    );
    assert!(
        post_lock_errors_samples
            .iter()
            .all(|error_samples| *error_samples <= CARRIER_AID_CODE_PHASE_ERROR_MAX_SAMPLES),
        "GPS L5 code phase drifted under carrier-aided code tracking: errors={post_lock_errors_samples:?} epochs={epochs:?}",
    );
    assert!(
        stable_code_rate_errors_hz
            .iter()
            .all(|error_hz| *error_hz <= CARRIER_AID_CODE_RATE_ERROR_MAX_HZ),
        "GPS L5 code rate did not follow carrier-aided expectation {expected_code_rate_hz}: errors={stable_code_rate_errors_hz:?} epochs={stable_epochs:?}",
    );
    assert!(
        stable_epochs.iter().all(|epoch| epoch
            .tracking_provenance
            .contains("code_rate_reference=carrier_doppler")),
        "GPS L5 tracking provenance must expose carrier-Doppler code-rate aiding: epochs={stable_epochs:?}",
    );
}

#[test]
fn tracking_rejects_cross_band_carrier_seed_for_gps_l5_code_loop() {
    let config = gps_l5_tracking_config();
    let sat = SatId { constellation: Constellation::Gps, prn: 18 };
    let signal = signal_spec_gps_l5_i();
    let carrier_doppler_hz = 1_500.0;
    let code_phase_chips = 1_537.25;
    let receiver_oscillator = SyntheticReceiverOscillatorModel {
        carrier_frequency_bias_hz: carrier_doppler_hz,
        carrier_frequency_drift_hz_per_s: 0.0,
        sampling_clock_fractional_error: carrier_doppler_hz / signal.carrier_hz.value(),
        ..SyntheticReceiverOscillatorModel::default()
    };
    let frame = synthetic_signal_with_carrier_aided_code_rate(
        &config,
        SyntheticSignalParams {
            sat,
            glonass_frequency_channel: None,
            signal_band: SignalBand::L5,
            signal_code: SignalCode::L5I,
            doppler_hz: 0.0,
            code_phase_chips,
            carrier_phase_rad: 0.0,
            cn0_db_hz: CARRIER_AID_CN0_DB_HZ,
            navigation_data: SyntheticNavigationData::from(false),
        },
        &receiver_oscillator,
    );
    let seeded_code_phase_samples =
        expected_acquisition_code_phase_samples(&config, &frame, code_phase_chips);
    let tracking = TrackingEngine::new(config, ReceiverRuntime::default());
    let tracks = tracking.track_from_acquisition(
        &frame,
        &[accepted_acquisition(
            sat,
            SignalBand::L5,
            SignalCode::L5I,
            carrier_doppler_hz,
            GPS_L1_CA_CARRIER_HZ.value() + carrier_doppler_hz,
            seeded_code_phase_samples,
        )],
    );

    assert!(
        tracks.is_empty(),
        "GPS L1 carrier metadata must not aid GPS L5 code tracking: tracks={tracks:?}",
    );
}

#[test]
fn tracking_holds_beidou_b1i_code_lock_with_carrier_aided_code_rate() {
    let config = beidou_b1_tracking_config();
    let sat = SatId { constellation: Constellation::Beidou, prn: 11 };
    let signal = signal_spec_beidou_b1i();
    let carrier_doppler_hz = 1_250.0;
    let code_phase_chips = 768.5;
    let receiver_oscillator = SyntheticReceiverOscillatorModel {
        carrier_frequency_bias_hz: carrier_doppler_hz,
        carrier_frequency_drift_hz_per_s: 0.0,
        sampling_clock_fractional_error: carrier_doppler_hz / signal.carrier_hz.value(),
        ..SyntheticReceiverOscillatorModel::default()
    };
    let frame = synthetic_signal_with_carrier_aided_code_rate(
        &config,
        SyntheticSignalParams {
            sat,
            glonass_frequency_channel: None,
            signal_band: SignalBand::B1,
            signal_code: SignalCode::B1I,
            doppler_hz: 0.0,
            code_phase_chips,
            carrier_phase_rad: 0.0,
            cn0_db_hz: CARRIER_AID_CN0_DB_HZ,
            navigation_data: SyntheticNavigationData::from(false),
        },
        &receiver_oscillator,
    );
    let seeded_code_phase_samples =
        expected_acquisition_code_phase_samples(&config, &frame, code_phase_chips);
    let tracking = TrackingEngine::new(config.clone(), ReceiverRuntime::default());
    let tracks = tracking.track_from_acquisition(
        &frame,
        &[accepted_acquisition(
            sat,
            SignalBand::B1,
            SignalCode::B1I,
            carrier_doppler_hz,
            carrier_doppler_hz,
            seeded_code_phase_samples,
        )],
    );
    let epochs = &tracks.first().expect("track").epochs;
    let stable_epochs = stable_tracking_window(epochs, CARRIER_AID_MIN_STABLE_EPOCHS);
    let expected_code_rate_hz = shared_path_code_rate_hz(carrier_doppler_hz, signal, signal)
        .expect("carrier-aided code rate");
    let post_lock_errors_samples =
        post_lock_code_phase_errors_samples(&config, epochs, seeded_code_phase_samples as f64);
    let stable_code_rate_errors_hz = stable_epochs
        .iter()
        .map(|epoch| (epoch.code_rate_hz.0 - expected_code_rate_hz).abs())
        .collect::<Vec<_>>();

    assert!(
        stable_epochs.len() >= CARRIER_AID_MIN_STABLE_EPOCHS,
        "tracking did not sustain enough BeiDou B1I lock epochs: epochs={epochs:?}",
    );
    assert!(
        stable_epochs
            .iter()
            .all(|epoch| epoch.lock && epoch.dll_lock && epoch.pll_lock && epoch.fll_lock),
        "BeiDou B1I stable tracking window lost lock flags: epochs={epochs:?}",
    );
    assert!(
        post_lock_errors_samples
            .iter()
            .all(|error_samples| *error_samples <= CARRIER_AID_CODE_PHASE_ERROR_MAX_SAMPLES),
        "BeiDou B1I code phase drifted under carrier-aided code tracking: errors={post_lock_errors_samples:?} epochs={epochs:?}",
    );
    assert!(
        stable_code_rate_errors_hz
            .iter()
            .all(|error_hz| *error_hz <= CARRIER_AID_CODE_RATE_ERROR_MAX_HZ),
        "BeiDou B1I code rate did not follow carrier-aided expectation {expected_code_rate_hz}: errors={stable_code_rate_errors_hz:?} epochs={stable_epochs:?}",
    );
}

#[test]
fn tracking_follows_dynamic_gps_l5_code_rate_from_common_oscillator_drift() {
    let config = gps_l5_tracking_config();
    let sat = SatId { constellation: Constellation::Gps, prn: 18 };
    let signal = signal_spec_gps_l5_i();
    let carrier_doppler_hz = 1_500.0;
    let carrier_drift_hz_per_s = 350.0;
    let code_phase_chips = 1_537.25;
    let receiver_oscillator = SyntheticReceiverOscillatorModel {
        carrier_frequency_bias_hz: carrier_doppler_hz,
        carrier_frequency_drift_hz_per_s: carrier_drift_hz_per_s,
        sampling_clock_fractional_error: carrier_doppler_hz / signal.carrier_hz.value(),
        sampling_clock_fractional_drift_per_s: carrier_drift_hz_per_s / signal.carrier_hz.value(),
        ..SyntheticReceiverOscillatorModel::default()
    };
    let frame = synthetic_signal_with_carrier_aided_code_rate(
        &config,
        SyntheticSignalParams {
            sat,
            glonass_frequency_channel: None,
            signal_band: SignalBand::L5,
            signal_code: SignalCode::L5I,
            doppler_hz: 0.0,
            code_phase_chips,
            carrier_phase_rad: 0.0,
            cn0_db_hz: CARRIER_AID_CN0_DB_HZ,
            navigation_data: SyntheticNavigationData::from(false),
        },
        &receiver_oscillator,
    );
    let seeded_code_phase_samples =
        expected_acquisition_code_phase_samples(&config, &frame, code_phase_chips);
    let tracking = TrackingEngine::new(config.clone(), ReceiverRuntime::default());
    let tracks = tracking.track_from_acquisition(
        &frame,
        &[accepted_acquisition(
            sat,
            SignalBand::L5,
            SignalCode::L5I,
            carrier_doppler_hz,
            carrier_doppler_hz,
            seeded_code_phase_samples,
        )],
    );
    let epochs = &tracks.first().expect("track").epochs;
    let stable_epochs = stable_tracking_window(epochs, CARRIER_AID_MIN_STABLE_EPOCHS);
    let stable_code_rate_errors_hz = stable_epochs
        .iter()
        .map(|epoch| {
            (epoch.code_rate_hz.0
                - dynamic_expected_code_rate_hz(
                    epoch.sample_index,
                    config.sampling_freq_hz,
                    signal,
                    &receiver_oscillator,
                ))
            .abs()
        })
        .collect::<Vec<_>>();
    let stable_code_phase_errors_samples = stable_epochs
        .iter()
        .map(|epoch| {
            (epoch.code_phase_samples.0
                - dynamic_expected_code_phase_samples(
                    &config,
                    epoch.sample_index,
                    code_phase_chips,
                    signal,
                    &receiver_oscillator,
                ))
            .abs()
        })
        .collect::<Vec<_>>();

    assert!(
        stable_epochs.len() >= CARRIER_AID_MIN_STABLE_EPOCHS,
        "tracking did not sustain enough dynamic GPS L5 lock epochs: epochs={epochs:?}",
    );
    assert!(
        stable_epochs
            .iter()
            .all(|epoch| epoch.lock && epoch.dll_lock && epoch.pll_lock && epoch.fll_lock),
        "GPS L5 dynamic tracking window lost lock flags: epochs={epochs:?}",
    );
    assert!(
        stable_code_rate_errors_hz
            .iter()
            .all(|error_hz| *error_hz <= CARRIER_AID_DRIFT_CODE_RATE_ERROR_MAX_HZ),
        "GPS L5 dynamic code rate did not follow the common oscillator drift: errors={stable_code_rate_errors_hz:?} epochs={stable_epochs:?}",
    );
    assert_carrier_aiding_reduces_dynamic_code_rate_error(
        &stable_code_rate_errors_hz,
        &stable_epochs,
        &config,
        signal,
        &receiver_oscillator,
    );
    assert!(
        stable_code_phase_errors_samples
            .iter()
            .all(|error_samples| *error_samples <= CARRIER_AID_DRIFT_CODE_PHASE_ERROR_MAX_SAMPLES),
        "GPS L5 dynamic code phase drifted under common oscillator drift: errors={stable_code_phase_errors_samples:?} epochs={stable_epochs:?}",
    );
}

#[test]
fn tracking_follows_dynamic_beidou_b1i_code_rate_from_common_oscillator_drift() {
    let config = beidou_b1_tracking_config();
    let sat = SatId { constellation: Constellation::Beidou, prn: 11 };
    let signal = signal_spec_beidou_b1i();
    let carrier_doppler_hz = 1_250.0;
    let carrier_drift_hz_per_s = 280.0;
    let code_phase_chips = 768.5;
    let receiver_oscillator = SyntheticReceiverOscillatorModel {
        carrier_frequency_bias_hz: carrier_doppler_hz,
        carrier_frequency_drift_hz_per_s: carrier_drift_hz_per_s,
        sampling_clock_fractional_error: carrier_doppler_hz / signal.carrier_hz.value(),
        sampling_clock_fractional_drift_per_s: carrier_drift_hz_per_s / signal.carrier_hz.value(),
        ..SyntheticReceiverOscillatorModel::default()
    };
    let frame = synthetic_signal_with_carrier_aided_code_rate(
        &config,
        SyntheticSignalParams {
            sat,
            glonass_frequency_channel: None,
            signal_band: SignalBand::B1,
            signal_code: SignalCode::B1I,
            doppler_hz: 0.0,
            code_phase_chips,
            carrier_phase_rad: 0.0,
            cn0_db_hz: CARRIER_AID_CN0_DB_HZ,
            navigation_data: SyntheticNavigationData::from(false),
        },
        &receiver_oscillator,
    );
    let seeded_code_phase_samples =
        expected_acquisition_code_phase_samples(&config, &frame, code_phase_chips);
    let tracking = TrackingEngine::new(config.clone(), ReceiverRuntime::default());
    let tracks = tracking.track_from_acquisition(
        &frame,
        &[accepted_acquisition(
            sat,
            SignalBand::B1,
            SignalCode::B1I,
            carrier_doppler_hz,
            carrier_doppler_hz,
            seeded_code_phase_samples,
        )],
    );
    let epochs = &tracks.first().expect("track").epochs;
    let stable_epochs = stable_tracking_window(epochs, CARRIER_AID_MIN_STABLE_EPOCHS);
    let stable_code_rate_errors_hz = stable_epochs
        .iter()
        .map(|epoch| {
            (epoch.code_rate_hz.0
                - dynamic_expected_code_rate_hz(
                    epoch.sample_index,
                    config.sampling_freq_hz,
                    signal,
                    &receiver_oscillator,
                ))
            .abs()
        })
        .collect::<Vec<_>>();
    let stable_code_phase_errors_samples = stable_epochs
        .iter()
        .map(|epoch| {
            (epoch.code_phase_samples.0
                - dynamic_expected_code_phase_samples(
                    &config,
                    epoch.sample_index,
                    code_phase_chips,
                    signal,
                    &receiver_oscillator,
                ))
            .abs()
        })
        .collect::<Vec<_>>();

    assert!(
        stable_epochs.len() >= CARRIER_AID_MIN_STABLE_EPOCHS,
        "tracking did not sustain enough dynamic BeiDou B1I lock epochs: epochs={epochs:?}",
    );
    assert!(
        stable_epochs
            .iter()
            .all(|epoch| epoch.lock && epoch.dll_lock && epoch.pll_lock && epoch.fll_lock),
        "BeiDou B1I dynamic tracking window lost lock flags: epochs={epochs:?}",
    );
    assert!(
        stable_code_rate_errors_hz
            .iter()
            .all(|error_hz| *error_hz <= CARRIER_AID_DRIFT_CODE_RATE_ERROR_MAX_HZ),
        "BeiDou B1I dynamic code rate did not follow the common oscillator drift: errors={stable_code_rate_errors_hz:?} epochs={stable_epochs:?}",
    );
    assert_carrier_aiding_reduces_dynamic_code_rate_error(
        &stable_code_rate_errors_hz,
        &stable_epochs,
        &config,
        signal,
        &receiver_oscillator,
    );
    assert!(
        stable_code_phase_errors_samples
            .iter()
            .all(|error_samples| *error_samples <= CARRIER_AID_DRIFT_CODE_PHASE_ERROR_MAX_SAMPLES),
        "BeiDou B1I dynamic code phase drifted under common oscillator drift: errors={stable_code_phase_errors_samples:?} epochs={stable_epochs:?}",
    );
}
