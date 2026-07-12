#![allow(missing_docs)]

mod support;

use bijux_gnss_core::api::{
    AcqHypothesis, AcqResult, Constellation, Hertz, ReceiverSampleTrace, SatId, SignalBand,
};
use bijux_gnss_receiver::api::{
    sim::{generate_l1_ca, SyntheticSignalParams},
    ReceiverPipelineConfig, ReceiverRuntime, TrackingEngine,
};
use support::tracking_truth::{
    carrier_phase_steps_cycles, first_tracking_lock_epoch_index,
    post_lock_carrier_frequency_errors_hz, stable_tracking_window,
};

const CLEAN_SIGNAL_LOCKED_CARRIER_ERROR_MAX_HZ: f64 = 5.0;
const CLEAN_SIGNAL_MIN_LOCKED_EPOCHS: usize = 5;
const CLEAN_SIGNAL_MIN_PHASE_STEP_CYCLES: f64 = 0.01;
const CLEAN_SIGNAL_MAX_PHASE_STEP_CYCLES: f64 = 0.35;

fn assert_clean_signal_carrier_lock(
    epochs: &[bijux_gnss_core::api::TrackEpoch],
    true_doppler_hz: f64,
) {
    let first_lock_epoch_index = first_tracking_lock_epoch_index(epochs).unwrap_or_else(|| {
        panic!("tracking never reached a stable carrier lock window: epochs={epochs:?}")
    });
    let post_lock_errors_hz = post_lock_carrier_frequency_errors_hz(epochs, true_doppler_hz);
    assert!(
        post_lock_errors_hz.len() >= CLEAN_SIGNAL_MIN_LOCKED_EPOCHS,
        "tracking did not maintain enough locked epochs for clean-signal validation: first_lock_epoch_index={first_lock_epoch_index}, post_lock_errors_hz={post_lock_errors_hz:?}, epochs={epochs:?}"
    );
    assert!(
        post_lock_errors_hz
            .iter()
            .all(|error_hz| *error_hz <= CLEAN_SIGNAL_LOCKED_CARRIER_ERROR_MAX_HZ),
        "locked carrier frequency error exceeded clean-signal threshold {CLEAN_SIGNAL_LOCKED_CARRIER_ERROR_MAX_HZ} Hz: post_lock_errors_hz={post_lock_errors_hz:?}, epochs={epochs:?}"
    );
    assert!(
        epochs[first_lock_epoch_index..]
            .iter()
            .all(|epoch| epoch.lock_state == "tracking" && epoch.pll_lock && epoch.fll_lock),
        "carrier lock window must remain in tracking once declared: epochs={epochs:?}"
    );
}

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
        cn0_proxy: 60.0,
        score: 1.0,
        hypothesis: AcqHypothesis::Accepted,
        assumptions: None,
        evidence: Vec::new(),
        threshold_provenance: None,
        explain_selection_reason: Some("seeded_pll_tracking_start".to_string()),
        doppler_refinement: None,
        code_phase_refinement: None,
        signal_delay_alignment: None,
        uncertainty: None,
    }
}

#[test]
fn tracking_reduces_seeded_carrier_error_and_emits_tracked_phase() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 4,
        early_late_spacing_chips: 0.5,
        dll_bw_hz: 2.0,
        pll_bw_hz: 15.0,
        fll_bw_hz: 0.0,
        ..ReceiverPipelineConfig::default()
    };
    let sat = SatId { constellation: Constellation::Gps, prn: 23 };
    let true_doppler_hz = 120.0;
    let seeded_doppler_hz = 80.0;
    let carrier_phase_rad = 0.30;
    let frame = generate_l1_ca(
        &config,
        SyntheticSignalParams {
            sat,
            glonass_frequency_channel: None,
            doppler_hz: true_doppler_hz,
            code_phase_chips: 0.0,
            carrier_phase_rad,
            cn0_db_hz: 60.0,
            data_bit_flip: false,
        },
        0xA112_0052,
        0.020,
    );
    let tracking = TrackingEngine::new(config, ReceiverRuntime::default());

    let tracks =
        tracking.track_from_acquisition(&frame, &[accepted_acquisition(sat, seeded_doppler_hz, 0)]);
    let epochs = &tracks.first().expect("track").epochs;
    assert!(epochs.len() >= 10, "epochs={epochs:?}");

    let first_freq_error = (epochs[0].carrier_hz.0 - true_doppler_hz).abs();
    let best_freq_error = epochs
        .iter()
        .map(|epoch| (epoch.carrier_hz.0 - true_doppler_hz).abs())
        .min_by(|lhs, rhs| lhs.partial_cmp(rhs).expect("finite carrier error"))
        .unwrap_or(f64::INFINITY);

    assert!(
        best_freq_error < first_freq_error,
        "tracking did not reduce carrier frequency error: first_freq_error={first_freq_error}, best_freq_error={best_freq_error}, epochs={epochs:?}"
    );
    assert_clean_signal_carrier_lock(epochs, true_doppler_hz);
    assert!(
        epochs.iter().all(|epoch| epoch.carrier_phase_cycles.0.is_finite()),
        "tracked carrier phase must remain finite: epochs={epochs:?}"
    );
    assert!(
        epochs.iter().any(|epoch| epoch.carrier_phase_cycles.0.abs() > 0.01),
        "tracked carrier phase never moved off zero: epochs={epochs:?}"
    );
    assert!(
        epochs.last().map(|epoch| epoch.pll_err.abs()).unwrap_or(f32::INFINITY)
            < epochs[0].pll_err.abs(),
        "tracking did not reduce pll phase error: epochs={epochs:?}"
    );
}

#[test]
fn tracking_keeps_carrier_phase_continuous_after_carrier_lock() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 4,
        early_late_spacing_chips: 0.5,
        dll_bw_hz: 2.0,
        pll_bw_hz: 15.0,
        fll_bw_hz: 0.0,
        ..ReceiverPipelineConfig::default()
    };
    let sat = SatId { constellation: Constellation::Gps, prn: 23 };
    let true_doppler_hz = 120.0;
    let frame = generate_l1_ca(
        &config,
        SyntheticSignalParams {
            sat,
            glonass_frequency_channel: None,
            doppler_hz: true_doppler_hz,
            code_phase_chips: 0.0,
            carrier_phase_rad: 0.30,
            cn0_db_hz: 60.0,
            data_bit_flip: false,
        },
        0xA112_0052,
        0.020,
    );
    let tracking = TrackingEngine::new(config, ReceiverRuntime::default());
    let tracks = tracking.track_from_acquisition(&frame, &[accepted_acquisition(sat, 80.0, 0)]);
    let epochs = &tracks.first().expect("track").epochs;
    let stable_window = stable_tracking_window(epochs, CLEAN_SIGNAL_MIN_LOCKED_EPOCHS);
    let phase_steps_cycles = carrier_phase_steps_cycles(stable_window);

    assert!(
        stable_window.len() >= CLEAN_SIGNAL_MIN_LOCKED_EPOCHS,
        "tracking never produced a sustained carrier-lock window: epochs={epochs:?}"
    );
    assert!(
        phase_steps_cycles
            .iter()
            .all(|step_cycles| step_cycles.is_finite()),
        "carrier phase steps must remain finite after lock: phase_steps_cycles={phase_steps_cycles:?}, epochs={epochs:?}"
    );
    assert!(
        phase_steps_cycles
            .iter()
            .all(|step_cycles| *step_cycles >= CLEAN_SIGNAL_MIN_PHASE_STEP_CYCLES),
        "carrier phase must keep moving forward after lock instead of stalling or wrapping backward: phase_steps_cycles={phase_steps_cycles:?}, epochs={epochs:?}"
    );
    assert!(
        phase_steps_cycles
            .iter()
            .all(|step_cycles| *step_cycles <= CLEAN_SIGNAL_MAX_PHASE_STEP_CYCLES),
        "carrier phase must stay continuous after lock instead of taking slip-sized jumps: phase_steps_cycles={phase_steps_cycles:?}, epochs={epochs:?}"
    );
}
