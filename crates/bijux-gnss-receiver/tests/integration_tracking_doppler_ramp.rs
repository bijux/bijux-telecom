#![allow(missing_docs)]

mod support;

use bijux_gnss_core::api::{
    AcqHypothesis, AcqRequest, AcqResult, Constellation, Hertz, ReceiverSampleTrace, SatId,
    SignalBand,
};
use bijux_gnss_receiver::api::{
    sim::{
        expected_acquisition_code_phase_samples, expected_acquisition_code_phase_samples_f64,
        generate_l1_ca_with_doppler_ramp, SyntheticDopplerRampParams, SyntheticSignalParams,
    },
    AcquisitionEngine, ReceiverPipelineConfig, ReceiverRuntime, TrackingEngine,
};

use support::tracking_truth::{
    carrier_frequency_error_under_linear_doppler_hz, code_phase_error_samples,
    first_tracking_lock_epoch_index, post_lock_carrier_frequency_errors_under_linear_doppler_hz,
    post_lock_epochs,
};

const CLEAN_DOPPLER_RAMP_CN0_DB_HZ: f32 = 75.0;
const CLEAN_DOPPLER_RAMP_DURATION_S: f64 = 0.060;
const CLEAN_DOPPLER_RAMP_LOCKED_CARRIER_ERROR_MAX_HZ: f64 = 10.0;
const CLEAN_DOPPLER_RAMP_LOCKED_CODE_ERROR_MAX_SAMPLES: f64 = 1.0;
const CLEAN_DOPPLER_RAMP_MIN_FULLY_LOCKED_EPOCHS: usize = 40;
const ACQUISITION_DOPPLER_RAMP_DURATION_S: f64 = 0.020;
const ACQUISITION_DOPPLER_RAMP_PEAK_MEAN_THRESHOLD: f32 = 8.0;
const ACQUISITION_DOPPLER_RAMP_RATE_HZ_PER_S: f64 = 20_000.0;
const ACQUISITION_DOPPLER_RAMP_RATE_STEP_HZ_PER_S: i32 = 5_000;
const ACQUISITION_DOPPLER_RAMP_RATE_SEARCH_HZ_PER_S: i32 = 25_000;

fn accepted_ramp_acquisition(
    sat: SatId,
    doppler_hz: f64,
    doppler_rate_hz_per_s: f64,
    code_phase_samples: usize,
) -> AcqResult {
    AcqResult {
        sat,
        signal_band: SignalBand::L1,
        signal_code: bijux_gnss_core::api::SignalCode::Unknown,
        glonass_frequency_channel: None,
        source_time: ReceiverSampleTrace::default(),
        candidate_rank: 1,
        is_primary_candidate: true,
        doppler_hz: Hertz(doppler_hz),
        doppler_rate_hz_per_s,
        carrier_hz: Hertz(doppler_hz),
        code_phase_samples,
        peak: 1.0,
        second_peak: 0.1,
        mean: 0.01,
        peak_mean_ratio: 20.0,
        peak_second_ratio: 10.0,
        cn0_proxy: CLEAN_DOPPLER_RAMP_CN0_DB_HZ,
        score: 1.0,
        hypothesis: AcqHypothesis::Accepted,
        assumptions: None,
        evidence: Vec::new(),
        threshold_provenance: None,
        explain_selection_reason: Some("seeded_doppler_ramp_tracking_start".to_string()),
        doppler_refinement: None,
        code_phase_refinement: None,
        signal_delay_alignment: None,
        uncertainty: None,
    }
}

fn doppler_ramp_tracking_config() -> ReceiverPipelineConfig {
    doppler_ramp_tracking_config_with_profile(true, 1)
}

fn doppler_ramp_tracking_config_with_profile(
    adaptive_tracking_enabled: bool,
    tracking_integration_ms: u32,
) -> ReceiverPipelineConfig {
    ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 4,
        early_late_spacing_chips: 0.5,
        dll_bw_hz: 2.0,
        pll_bw_hz: 18.0,
        fll_bw_hz: 12.0,
        adaptive_tracking_enabled,
        tracking_integration_ms,
        ..ReceiverPipelineConfig::default()
    }
}

fn doppler_ramp_acquisition_config(doppler_rate_search_hz_per_s: i32) -> ReceiverPipelineConfig {
    ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 4,
        acquisition_doppler_search_hz: 500,
        acquisition_doppler_step_hz: 250,
        acquisition_doppler_rate_search_hz_per_s: doppler_rate_search_hz_per_s,
        acquisition_doppler_rate_step_hz_per_s: ACQUISITION_DOPPLER_RAMP_RATE_STEP_HZ_PER_S,
        acquisition_integration_ms: 20,
        acquisition_noncoherent: 1,
        acquisition_peak_mean_threshold: ACQUISITION_DOPPLER_RAMP_PEAK_MEAN_THRESHOLD,
        acquisition_peak_second_threshold: 1.5,
        ..ReceiverPipelineConfig::default()
    }
}

fn track_clean_doppler_ramp_case(
    config: &ReceiverPipelineConfig,
    sat: SatId,
    initial_doppler_hz: f64,
    doppler_rate_hz_per_s: f64,
    code_phase_chips: f64,
    carrier_phase_rad: f64,
) -> Vec<bijux_gnss_core::api::TrackEpoch> {
    let frame = generate_l1_ca_with_doppler_ramp(
        config,
        SyntheticDopplerRampParams {
            signal: SyntheticSignalParams {
                sat,
                glonass_frequency_channel: None,
                signal_band: bijux_gnss_core::api::SignalBand::L1,
                signal_code: bijux_gnss_core::api::SignalCode::Unknown,
                doppler_hz: initial_doppler_hz,
                code_phase_chips,
                carrier_phase_rad,
                cn0_db_hz: CLEAN_DOPPLER_RAMP_CN0_DB_HZ,
                navigation_data: false.into(),
            },
            doppler_rate_hz_per_s,
        },
        0xD077_601E,
        CLEAN_DOPPLER_RAMP_DURATION_S,
    );
    let seeded_code_phase_samples =
        expected_acquisition_code_phase_samples(config, &frame, code_phase_chips);
    let tracking = TrackingEngine::new(config.clone(), ReceiverRuntime::default());
    let tracks = tracking.track_from_acquisition(
        &frame,
        &[accepted_ramp_acquisition(
            sat,
            initial_doppler_hz,
            doppler_rate_hz_per_s,
            seeded_code_phase_samples,
        )],
    );

    tracks.first().expect("track").epochs.clone()
}

fn acquire_clean_doppler_ramp_case(
    config: &ReceiverPipelineConfig,
    sat: SatId,
    initial_doppler_hz: f64,
    doppler_rate_hz_per_s: f64,
    code_phase_chips: f64,
    carrier_phase_rad: f64,
) -> (AcqResult, bijux_gnss_core::api::SamplesFrame) {
    let frame = generate_l1_ca_with_doppler_ramp(
        config,
        SyntheticDopplerRampParams {
            signal: SyntheticSignalParams {
                sat,
                glonass_frequency_channel: None,
                signal_band: bijux_gnss_core::api::SignalBand::L1,
                signal_code: bijux_gnss_core::api::SignalCode::Ca,
                doppler_hz: initial_doppler_hz,
                code_phase_chips,
                carrier_phase_rad,
                cn0_db_hz: CLEAN_DOPPLER_RAMP_CN0_DB_HZ,
                navigation_data: false.into(),
            },
            doppler_rate_hz_per_s,
        },
        0xAC05_7100,
        ACQUISITION_DOPPLER_RAMP_DURATION_S,
    );
    let request = AcqRequest {
        sat,
        glonass_frequency_channel: None,
        signal_band: SignalBand::L1,
        signal_code: bijux_gnss_core::api::SignalCode::Ca,
        doppler_center_hz: 0.0,
        doppler_rate_center_hz_per_s: 0.0,
        expected_line_of_sight_doppler_hz: None,
        assistance_bounds: None,
        doppler_search_hz: config.acquisition_doppler_search_hz,
        doppler_step_hz: config.acquisition_doppler_step_hz,
        doppler_rate_search_hz_per_s: config.acquisition_doppler_rate_search_hz_per_s,
        doppler_rate_step_hz_per_s: config.acquisition_doppler_rate_step_hz_per_s,
        coherent_ms: config.acquisition_integration_ms,
        noncoherent: config.acquisition_noncoherent,
    };
    let result = AcquisitionEngine::new(config.clone(), ReceiverRuntime::default())
        .run_fft_for_requests(&frame, &[request])
        .remove(0);

    (result, frame)
}

fn lock_preserved_epoch(epoch: &bijux_gnss_core::api::TrackEpoch) -> bool {
    epoch.lock
        && matches!(epoch.lock_state.as_str(), "tracking" | "degraded")
        && !epoch.cycle_slip
        && epoch.lock_state_reason.as_deref() != Some("lock_lost")
}

fn fully_locked_epoch(epoch: &bijux_gnss_core::api::TrackEpoch) -> bool {
    epoch.lock_state == "tracking" && epoch.pll_lock && epoch.dll_lock && epoch.fll_lock
}

fn count_fully_locked_epochs(epochs: &[bijux_gnss_core::api::TrackEpoch]) -> usize {
    epochs.iter().filter(|epoch| fully_locked_epoch(epoch)).count()
}

#[test]
fn acquisition_requires_declared_doppler_rate_support_for_strong_ramp() {
    let sat = SatId { constellation: Constellation::Gps, prn: 16 };
    let initial_doppler_hz = 250.0;
    let code_phase_chips = 211.25;
    let carrier_phase_rad = 0.40;
    let without_rate_config = doppler_ramp_acquisition_config(0);
    let with_rate_config =
        doppler_ramp_acquisition_config(ACQUISITION_DOPPLER_RAMP_RATE_SEARCH_HZ_PER_S);

    let (without_rate_support, _) = acquire_clean_doppler_ramp_case(
        &without_rate_config,
        sat,
        initial_doppler_hz,
        ACQUISITION_DOPPLER_RAMP_RATE_HZ_PER_S,
        code_phase_chips,
        carrier_phase_rad,
    );
    let (with_rate_support, frame) = acquire_clean_doppler_ramp_case(
        &with_rate_config,
        sat,
        initial_doppler_hz,
        ACQUISITION_DOPPLER_RAMP_RATE_HZ_PER_S,
        code_phase_chips,
        carrier_phase_rad,
    );
    let expected_code_phase_samples =
        expected_acquisition_code_phase_samples(&with_rate_config, &frame, code_phase_chips);

    assert!(
        (without_rate_support.doppler_hz.0 - initial_doppler_hz).abs()
            >= without_rate_config.acquisition_doppler_step_hz as f64 - f64::EPSILON,
        "zero-rate search should bias the Doppler estimate by at least one coarse bin under the ramp: {without_rate_support:?}"
    );
    assert!(
        without_rate_support.peak_second_ratio
            < without_rate_config.acquisition_peak_second_threshold,
        "zero-rate search should fail the declared peak-separation threshold under the ramp: {without_rate_support:?}"
    );
    assert!(
        matches!(
            without_rate_support.hypothesis,
            AcqHypothesis::Rejected | AcqHypothesis::Ambiguous
        ),
        "{without_rate_support:?}"
    );
    assert!(
        matches!(with_rate_support.hypothesis, AcqHypothesis::Accepted | AcqHypothesis::Ambiguous),
        "{with_rate_support:?}"
    );
    assert!(
        with_rate_support.peak_mean_ratio >= with_rate_config.acquisition_peak_mean_threshold,
        "rate-aware acquisition should clear the declared threshold: {with_rate_support:?}"
    );
    assert!(
        with_rate_support.peak_mean_ratio > without_rate_support.peak_mean_ratio * 1.1,
        "rate-aware acquisition should materially outperform the zero-rate search: without={without_rate_support:?}, with={with_rate_support:?}"
    );
    assert_eq!(with_rate_support.code_phase_samples, expected_code_phase_samples);
    assert!(
        (with_rate_support.doppler_hz.0 - initial_doppler_hz).abs()
            <= with_rate_config.acquisition_doppler_step_hz as f64 + f64::EPSILON,
        "{with_rate_support:?}"
    );
    assert!(
        (with_rate_support.doppler_rate_hz_per_s - ACQUISITION_DOPPLER_RAMP_RATE_HZ_PER_S).abs()
            <= with_rate_config.acquisition_doppler_rate_step_hz_per_s as f64 + f64::EPSILON,
        "{with_rate_support:?}"
    );
}

#[test]
fn tracking_reaches_and_preserves_lock_under_positive_doppler_ramp() {
    let config = doppler_ramp_tracking_config();
    let sat = SatId { constellation: Constellation::Gps, prn: 16 };
    let epochs = track_clean_doppler_ramp_case(&config, sat, 180.0, 40.0, 211.25, 0.40);
    let first_lock_epoch_index = first_tracking_lock_epoch_index(&epochs).unwrap_or_else(|| {
        panic!("tracking never reached stable lock under ramp: epochs={epochs:?}")
    });
    let post_lock = post_lock_epochs(&epochs);

    assert!(epochs.len() >= 60, "epochs={epochs:?}");
    assert!(!post_lock.is_empty(), "tracking never locked under ramp: epochs={epochs:?}");
    assert!(
        post_lock.iter().all(lock_preserved_epoch),
        "tracking did not preserve lock after ramp lock at epoch {first_lock_epoch_index}: epochs={epochs:?}"
    );
    assert!(
        count_fully_locked_epochs(post_lock) >= CLEAN_DOPPLER_RAMP_MIN_FULLY_LOCKED_EPOCHS,
        "tracking did not sustain a long fully locked span under positive ramp: fully_locked_epochs={}, epochs={epochs:?}",
        count_fully_locked_epochs(post_lock)
    );
}

#[test]
fn tracking_keeps_bounded_carrier_error_under_positive_doppler_ramp() {
    let config = doppler_ramp_tracking_config();
    let sat = SatId { constellation: Constellation::Gps, prn: 16 };
    let initial_doppler_hz = 180.0;
    let doppler_rate_hz_per_s = 40.0;
    let epochs = track_clean_doppler_ramp_case(
        &config,
        sat,
        initial_doppler_hz,
        doppler_rate_hz_per_s,
        211.25,
        0.40,
    );
    let post_lock_errors_hz = post_lock_carrier_frequency_errors_under_linear_doppler_hz(
        &epochs,
        config.sampling_freq_hz,
        initial_doppler_hz,
        doppler_rate_hz_per_s,
    );

    assert!(!post_lock_errors_hz.is_empty(), "epochs={epochs:?}");
    assert!(
        post_lock_errors_hz
            .iter()
            .all(|error_hz| *error_hz <= CLEAN_DOPPLER_RAMP_LOCKED_CARRIER_ERROR_MAX_HZ),
        "carrier error exceeded doppler ramp threshold {CLEAN_DOPPLER_RAMP_LOCKED_CARRIER_ERROR_MAX_HZ} Hz: post_lock_errors_hz={post_lock_errors_hz:?}, epochs={epochs:?}"
    );
}

#[test]
fn tracking_preserves_lock_and_code_phase_under_negative_doppler_ramp() {
    let config = doppler_ramp_tracking_config();
    let sat = SatId { constellation: Constellation::Gps, prn: 22 };
    let initial_doppler_hz = -220.0;
    let doppler_rate_hz_per_s = -35.0;
    let code_phase_chips = 388.5;
    let epochs = track_clean_doppler_ramp_case(
        &config,
        sat,
        initial_doppler_hz,
        doppler_rate_hz_per_s,
        code_phase_chips,
        0.15,
    );
    let frame = generate_l1_ca_with_doppler_ramp(
        &config,
        SyntheticDopplerRampParams {
            signal: SyntheticSignalParams {
                sat,
                glonass_frequency_channel: None,
                signal_band: bijux_gnss_core::api::SignalBand::L1,
                signal_code: bijux_gnss_core::api::SignalCode::Unknown,
                doppler_hz: initial_doppler_hz,
                code_phase_chips,
                carrier_phase_rad: 0.15,
                cn0_db_hz: CLEAN_DOPPLER_RAMP_CN0_DB_HZ,
                navigation_data: false.into(),
            },
            doppler_rate_hz_per_s,
        },
        0xD077_601E,
        CLEAN_DOPPLER_RAMP_DURATION_S,
    );
    let expected_code_phase_samples =
        expected_acquisition_code_phase_samples_f64(&config, &frame, code_phase_chips);
    let post_lock = post_lock_epochs(&epochs);
    let carrier_errors_hz = post_lock
        .iter()
        .map(|epoch| {
            carrier_frequency_error_under_linear_doppler_hz(
                epoch,
                config.sampling_freq_hz,
                initial_doppler_hz,
                doppler_rate_hz_per_s,
            )
        })
        .collect::<Vec<_>>();
    let code_errors_samples = post_lock
        .iter()
        .map(|epoch| code_phase_error_samples(&config, epoch, expected_code_phase_samples))
        .collect::<Vec<_>>();

    assert!(!post_lock.is_empty(), "epochs={epochs:?}");
    assert!(
        post_lock.iter().all(lock_preserved_epoch),
        "tracking did not preserve lock after pull-in under negative ramp: epochs={epochs:?}"
    );
    assert!(
        count_fully_locked_epochs(post_lock) >= CLEAN_DOPPLER_RAMP_MIN_FULLY_LOCKED_EPOCHS,
        "tracking did not sustain a long fully locked span under negative ramp: fully_locked_epochs={}, epochs={epochs:?}",
        count_fully_locked_epochs(post_lock)
    );
    assert!(
        carrier_errors_hz
            .iter()
            .all(|error_hz| *error_hz <= CLEAN_DOPPLER_RAMP_LOCKED_CARRIER_ERROR_MAX_HZ),
        "carrier error exceeded negative-ramp threshold {CLEAN_DOPPLER_RAMP_LOCKED_CARRIER_ERROR_MAX_HZ} Hz: carrier_errors_hz={carrier_errors_hz:?}, epochs={epochs:?}"
    );
    assert!(
        code_errors_samples
            .iter()
            .all(|error_samples| *error_samples <= CLEAN_DOPPLER_RAMP_LOCKED_CODE_ERROR_MAX_SAMPLES),
        "code error exceeded negative-ramp threshold {CLEAN_DOPPLER_RAMP_LOCKED_CODE_ERROR_MAX_SAMPLES} samples: code_errors_samples={code_errors_samples:?}, epochs={epochs:?}"
    );
}

#[test]
fn adaptive_tracking_shortens_coherent_integration_under_strong_doppler_ramp() {
    let sat = SatId { constellation: Constellation::Gps, prn: 23 };
    let initial_doppler_hz = 250.0;
    let doppler_rate_hz_per_s = 400.0;
    let code_phase_chips = 211.25;
    let carrier_phase_rad = 0.40;
    let adaptive_config = doppler_ramp_tracking_config_with_profile(true, 5);
    let fixed_config = doppler_ramp_tracking_config_with_profile(false, 5);

    let adaptive_epochs = track_clean_doppler_ramp_case(
        &adaptive_config,
        sat,
        initial_doppler_hz,
        doppler_rate_hz_per_s,
        code_phase_chips,
        carrier_phase_rad,
    );
    let fixed_epochs = track_clean_doppler_ramp_case(
        &fixed_config,
        sat,
        initial_doppler_hz,
        doppler_rate_hz_per_s,
        code_phase_chips,
        carrier_phase_rad,
    );
    let shared_end_sample_index =
        fixed_epochs.last().expect("fixed strong-ramp tracking epoch").sample_index;
    let adaptive_epochs_within_shared_window = adaptive_epochs
        .iter()
        .filter(|epoch| epoch.sample_index <= shared_end_sample_index)
        .cloned()
        .collect::<Vec<_>>();
    let adaptive_integration_ms_values = adaptive_epochs
        .iter()
        .map(|epoch| {
            epoch
                .tracking_assumptions
                .as_ref()
                .map(|assumptions| assumptions.integration_ms.max(1))
                .unwrap_or(1)
        })
        .collect::<Vec<_>>();
    let fixed_integration_ms_values = fixed_epochs
        .iter()
        .map(|epoch| {
            epoch
                .tracking_assumptions
                .as_ref()
                .map(|assumptions| assumptions.integration_ms.max(1))
                .unwrap_or(1)
        })
        .collect::<Vec<_>>();
    let dynamic_profile_assumptions = adaptive_epochs
        .iter()
        .filter_map(|epoch| epoch.tracking_assumptions.as_ref())
        .find(|assumptions| assumptions.integration_ms == 1)
        .expect("adaptive dynamic profile assumptions");

    assert!(
        adaptive_integration_ms_values.iter().any(|integration_ms| *integration_ms == 1),
        "adaptive_integration_ms_values={adaptive_integration_ms_values:?} adaptive_epochs={adaptive_epochs:?}"
    );
    assert!(
        fixed_integration_ms_values.iter().all(|integration_ms| *integration_ms == 5),
        "fixed_integration_ms_values={fixed_integration_ms_values:?} fixed_epochs={fixed_epochs:?}"
    );
    assert!(
        adaptive_epochs_within_shared_window.len() > fixed_epochs.len(),
        "shared_end_sample_index={shared_end_sample_index} adaptive_epoch_count={} fixed_epoch_count={} adaptive_epochs={adaptive_epochs:?} fixed_epochs={fixed_epochs:?}",
        adaptive_epochs_within_shared_window.len(),
        fixed_epochs.len()
    );
    assert_eq!(dynamic_profile_assumptions.dll_bw_hz, adaptive_config.dll_bw_hz * 2.0);
    assert_eq!(dynamic_profile_assumptions.pll_bw_hz, adaptive_config.pll_bw_hz * 3.5);
    assert_eq!(dynamic_profile_assumptions.fll_bw_hz, adaptive_config.fll_bw_hz * 4.0);
}
