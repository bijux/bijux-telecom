#![allow(missing_docs)]

mod support;

use bijux_gnss_core::api::{
    AcqHypothesis, AcqResult, Constellation, Hertz, ReceiverSampleTrace, SatId, SignalBand,
    SignalCode, GPS_L1_CA_CARRIER_HZ, GPS_L5_CARRIER_HZ,
};
use bijux_gnss_receiver::api::{
    sim::{
        expected_acquisition_code_phase_samples, generate_l1_ca_multi_with_receiver_oscillator,
        SyntheticNavigationData, SyntheticReceiverOscillatorModel, SyntheticScenario,
        SyntheticSignalParams,
    },
    ReceiverPipelineConfig, ReceiverRuntime, TrackingEngine,
};
use bijux_gnss_signal::api::{shared_path_code_rate_hz, signal_spec_gps_l5_i};

use support::tracking_truth::{post_lock_code_phase_errors_samples, stable_tracking_window};

const CARRIER_AID_DURATION_S: f64 = 0.060;
const CARRIER_AID_CN0_DB_HZ: f32 = 75.0;
const CARRIER_AID_MIN_STABLE_EPOCHS: usize = 20;
const CARRIER_AID_CODE_PHASE_ERROR_MAX_SAMPLES: f64 = 2.0;
const CARRIER_AID_CODE_RATE_ERROR_MAX_HZ: f64 = 2.0;

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
}
