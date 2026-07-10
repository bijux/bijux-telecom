#![allow(missing_docs)]

mod support;

use bijux_gnss_core::api::{
    AcqHypothesis, AcqResult, Constellation, Hertz, ReceiverSampleTrace, SatId, SignalBand,
};
use bijux_gnss_receiver::api::{
    sim::{generate_l1_ca_with_doppler_ramp, SyntheticDopplerRampParams, SyntheticSignalParams},
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

fn accepted_acquisition(sat: SatId, doppler_hz: f64, code_phase_samples: usize) -> AcqResult {
    AcqResult {
        sat,
        signal_band: SignalBand::L1,
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
    let expected_code_phase_samples =
        code_phase_chips * config.sampling_freq_hz / config.code_freq_basis_hz;
    let frame = generate_l1_ca_with_doppler_ramp(
        config,
        SyntheticDopplerRampParams {
            signal: SyntheticSignalParams {
                sat,
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
    let tracking = TrackingEngine::new(config.clone(), ReceiverRuntime::default());
    let tracks = tracking.track_from_acquisition(
        &frame,
        &[accepted_acquisition(
            sat,
            initial_doppler_hz,
            expected_code_phase_samples.round() as usize,
        )],
    );

    tracks.first().expect("track").epochs.clone()
}

fn stable_tracking_window(
    epochs: &[bijux_gnss_core::api::TrackEpoch],
) -> &[bijux_gnss_core::api::TrackEpoch] {
    epochs
        .iter()
        .enumerate()
        .find_map(|(start, _)| {
            epochs[start..]
                .iter()
                .all(|epoch| {
                    epoch.lock
                        && epoch.lock_state == "tracking"
                        && !epoch.cycle_slip
                        && epoch.lock_state_reason.as_deref() != Some("lock_lost")
                })
                .then_some(&epochs[start..])
        })
        .unwrap_or(&[])
}

#[test]
fn tracking_reaches_and_preserves_lock_under_positive_doppler_ramp() {
    let config = doppler_ramp_tracking_config();
    let sat = SatId { constellation: Constellation::Gps, prn: 16 };
    let epochs = track_clean_doppler_ramp_case(&config, sat, 180.0, 40.0, 211.25, 0.40);
    let first_lock_epoch_index = first_tracking_lock_epoch_index(&epochs)
        .unwrap_or_else(|| panic!("tracking never reached stable lock under ramp: epochs={epochs:?}"));
    let post_lock = post_lock_epochs(&epochs);

    assert!(epochs.len() >= 60, "epochs={epochs:?}");
    assert!(!post_lock.is_empty(), "tracking never locked under ramp: epochs={epochs:?}");
    assert!(
        post_lock.iter().all(|epoch| epoch.lock),
        "prompt lock dropped after ramp lock at epoch {first_lock_epoch_index}: epochs={epochs:?}"
    );
    assert!(
        post_lock
            .iter()
            .all(|epoch| epoch.lock_state == "tracking" && epoch.pll_lock && epoch.fll_lock),
        "tracking left the locked state under ramp at epoch {first_lock_epoch_index}: epochs={epochs:?}"
    );
    assert!(
        post_lock
            .iter()
            .all(|epoch| !epoch.cycle_slip && epoch.lock_state_reason.as_deref() != Some("lock_lost")),
        "tracking reported cycle slip or lock loss after ramp lock at epoch {first_lock_epoch_index}: epochs={epochs:?}"
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
    let expected_code_phase_samples =
        code_phase_chips * config.sampling_freq_hz / config.code_freq_basis_hz;
    let stable_window = stable_tracking_window(&epochs);
    let carrier_errors_hz = stable_window
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
    let code_errors_samples = stable_window
        .iter()
        .map(|epoch| code_phase_error_samples(&config, epoch, expected_code_phase_samples))
        .collect::<Vec<_>>();

    assert!(!stable_window.is_empty(), "epochs={epochs:?}");
    assert!(
        stable_window.len() >= 40,
        "tracking did not maintain a long stable window under negative ramp: stable_window_len={}, epochs={epochs:?}",
        stable_window.len()
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
