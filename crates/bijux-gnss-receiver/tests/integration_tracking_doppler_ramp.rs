#![allow(missing_docs)]

mod support;

use bijux_gnss_core::api::{
    AcqHypothesis, AcqResult, Constellation, Hertz, ReceiverSampleTrace, SatId, SignalBand,
};
use bijux_gnss_receiver::api::{
    sim::{
        expected_acquisition_code_phase_samples, expected_acquisition_code_phase_samples_f64,
        generate_l1_ca_with_doppler_ramp, SyntheticDopplerRampParams, SyntheticSignalParams,
    },
    ReceiverPipelineConfig, ReceiverRuntime, TrackingEngine,
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

fn accepted_acquisition(sat: SatId, doppler_hz: f64, code_phase_samples: usize) -> AcqResult {
    AcqResult {
        sat,
        signal_band: SignalBand::L1,
        signal_code: bijux_gnss_core::api::SignalCode::Unknown,
        glonass_frequency_channel: None,
        source_time: ReceiverSampleTrace::default(),
        candidate_rank: 1,
        is_primary_candidate: true,
        doppler_hz: Hertz(doppler_hz),
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
                data_bit_flip: false,
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
        &[accepted_acquisition(sat, initial_doppler_hz, seeded_code_phase_samples)],
    );

    tracks.first().expect("track").epochs.clone()
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
                data_bit_flip: false,
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
