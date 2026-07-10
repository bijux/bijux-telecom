#![allow(missing_docs)]

mod support;

use bijux_gnss_core::api::{
    AcqHypothesis, AcqResult, Constellation, Hertz, ReceiverSampleTrace, SatId, SignalBand,
};
use bijux_gnss_receiver::api::{
    sim::{generate_l1_ca_with_fades, SyntheticFadeWindow, SyntheticSignalParams},
    ReceiverPipelineConfig, ReceiverRuntime, TrackingEngine,
};

use support::tracking_truth::{
    carrier_phase_steps_cycles, epoch_indices_with_lock_state, epoch_indices_with_lock_state_reason,
};

const SHORT_FADE_PRELOCK_CN0_DBHZ: f32 = 60.0;
const SHORT_FADE_RECOVERY_MIN_TRACKING_EPOCHS: usize = 5;
const SHORT_FADE_RECOVERY_MIN_CARRIER_LOCK_EPOCHS: usize = 3;
const SHORT_FADE_MIN_PHASE_STEP_CYCLES: f64 = 0.01;
const SHORT_FADE_MAX_PHASE_STEP_CYCLES: f64 = 0.35;

fn stable_carrier_lock_window(
    epochs: &[bijux_gnss_core::api::TrackEpoch],
    min_locked_epochs: usize,
) -> &[bijux_gnss_core::api::TrackEpoch] {
    if min_locked_epochs == 0 {
        return epochs;
    }

    let mut stable_start = None;
    for (index, epoch) in epochs.iter().enumerate() {
        let stable = epoch.lock_state == "tracking"
            && epoch.pll_lock
            && epoch.fll_lock
            && !epoch.cycle_slip
            && epoch.lock_state_reason.as_deref() != Some("lock_lost");
        match (stable_start, stable) {
            (None, true) => stable_start = Some(index),
            (Some(start), false) => {
                if index - start >= min_locked_epochs {
                    return &epochs[start..index];
                }
                stable_start = None;
            }
            _ => {}
        }
    }

    if let Some(start) = stable_start {
        if epochs.len() - start >= min_locked_epochs {
            return &epochs[start..];
        }
    }

    &[]
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
        cn0_proxy: SHORT_FADE_PRELOCK_CN0_DBHZ,
        score: 1.0,
        hypothesis: AcqHypothesis::Accepted,
        assumptions: None,
        evidence: Vec::new(),
        threshold_provenance: None,
        explain_selection_reason: Some("short_fade_tracking_start".to_string()),
        doppler_refinement: None,
        code_phase_refinement: None,
        signal_delay_alignment: None,
        uncertainty: None,
    }
}

fn tracking_config() -> ReceiverPipelineConfig {
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

fn assert_tracking_recovers_after_short_fade(
    fade_start_s: f64,
    fade_end_s: f64,
    duration_s: f64,
    doppler_hz: f64,
) {
    let config = tracking_config();
    let sat = SatId { constellation: Constellation::Gps, prn: 21 };
    let frame = generate_l1_ca_with_fades(
        &config,
        SyntheticSignalParams {
            sat,
            doppler_hz,
            code_phase_chips: 0.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: SHORT_FADE_PRELOCK_CN0_DBHZ,
            data_bit_flip: false,
        },
        &[SyntheticFadeWindow { start_s: fade_start_s, end_s: fade_end_s, signal_scale: 0.0 }],
        0x5FADE021,
        duration_s,
    );
    let tracking = TrackingEngine::new(config.clone(), ReceiverRuntime::default());
    let tracks =
        tracking.track_from_acquisition(&frame, &[accepted_acquisition(sat, doppler_hz, 0)]);
    let epochs = &tracks.first().expect("track").epochs;

    let fade_start_sample = (fade_start_s * config.sampling_freq_hz).round() as u64;
    let fade_end_sample = (fade_end_s * config.sampling_freq_hz).round() as u64;
    let degraded_indices = epoch_indices_with_lock_state(epochs, "degraded");
    let fade_reason_indices = epoch_indices_with_lock_state_reason(epochs, "signal_fade");
    let recovered_reason_indices = epoch_indices_with_lock_state_reason(epochs, "fade_recovered");
    let lost_indices = epoch_indices_with_lock_state(epochs, "lost");
    assert!(
        epochs.iter().any(|epoch| {
            epoch.sample_index < fade_start_sample
                && epoch.lock_state == "tracking"
                && epoch.pll_lock
                && epoch.fll_lock
        }),
        "tracking must lock before the fade begins: epochs={epochs:?}"
    );
    assert!(
        degraded_indices.iter().any(|index| {
            let sample_index = epochs[*index].sample_index;
            sample_index >= fade_start_sample && sample_index < fade_end_sample
        }),
        "tracking must enter degraded state during the fade window: fade=[{fade_start_sample}, {fade_end_sample}), epochs={epochs:?}"
    );
    assert!(
        !fade_reason_indices.is_empty(),
        "short fade must emit explicit fade reasons instead of a false continuous lock: epochs={epochs:?}"
    );
    assert!(
        lost_indices.iter().all(|index| {
            let sample_index = epochs[*index].sample_index;
            sample_index < fade_start_sample || sample_index >= fade_end_sample
        }),
        "short fades must remain recoverable without entering lost state inside the fade window: epochs={epochs:?}"
    );

    let recovery_index = *recovered_reason_indices
        .iter()
        .find(|index| epochs[**index].sample_index >= fade_end_sample)
        .unwrap_or_else(|| panic!("tracking must emit a fade recovery reason: epochs={epochs:?}"));

    let post_recovery_tracking_epochs = epochs[recovery_index..]
        .iter()
        .filter(|epoch| {
            epoch.sample_index >= fade_end_sample
                && epoch.lock_state == "tracking"
                && epoch.pll_lock
                && epoch.fll_lock
                && !epoch.cycle_slip
        })
        .count();
    assert!(
        post_recovery_tracking_epochs >= SHORT_FADE_RECOVERY_MIN_TRACKING_EPOCHS,
        "tracking must recover into a stable post-fade lock window: recovery_index={recovery_index}, post_recovery_tracking_epochs={post_recovery_tracking_epochs}, epochs={epochs:?}"
    );
}

#[test]
fn tracking_marks_ten_millisecond_signal_fade_as_degraded_and_recovers() {
    assert_tracking_recovers_after_short_fade(0.030, 0.040, 0.090, 0.0);
}

#[test]
fn tracking_marks_hundred_millisecond_signal_fade_as_degraded_and_recovers() {
    assert_tracking_recovers_after_short_fade(0.030, 0.130, 0.200, 0.0);
}

#[test]
fn tracking_keeps_carrier_phase_continuous_after_short_fade_recovery() {
    let fade_start_s = 0.030;
    let fade_end_s = 0.040;
    let duration_s = 0.090;
    let doppler_hz = 120.0;
    let config = tracking_config();
    let sat = SatId { constellation: Constellation::Gps, prn: 21 };
    let frame = generate_l1_ca_with_fades(
        &config,
        SyntheticSignalParams {
            sat,
            doppler_hz,
            code_phase_chips: 0.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: SHORT_FADE_PRELOCK_CN0_DBHZ,
            data_bit_flip: false,
        },
        &[SyntheticFadeWindow { start_s: fade_start_s, end_s: fade_end_s, signal_scale: 0.0 }],
        0x5FADE022,
        duration_s,
    );
    let tracking = TrackingEngine::new(config.clone(), ReceiverRuntime::default());
    let tracks =
        tracking.track_from_acquisition(&frame, &[accepted_acquisition(sat, doppler_hz, 0)]);
    let epochs = &tracks.first().expect("track").epochs;
    let fade_end_sample = (fade_end_s * config.sampling_freq_hz).round() as u64;
    let recovery_index = *epoch_indices_with_lock_state_reason(epochs, "fade_recovered")
        .iter()
        .find(|index| epochs[**index].sample_index >= fade_end_sample)
        .unwrap_or_else(|| {
            panic!("tracking must emit a post-fade recovery epoch: epochs={epochs:?}")
        });
    let stable_post_recovery_window = stable_carrier_lock_window(
        &epochs[recovery_index..],
        SHORT_FADE_RECOVERY_MIN_CARRIER_LOCK_EPOCHS,
    );
    let phase_steps_cycles = carrier_phase_steps_cycles(stable_post_recovery_window);

    assert!(
        stable_post_recovery_window.len() >= SHORT_FADE_RECOVERY_MIN_CARRIER_LOCK_EPOCHS,
        "tracking never re-established a sustained post-fade lock window: epochs={epochs:?}"
    );
    assert!(
        phase_steps_cycles
            .iter()
            .all(|step_cycles| *step_cycles >= SHORT_FADE_MIN_PHASE_STEP_CYCLES),
        "carrier phase must resume forward continuity after a recoverable fade: phase_steps_cycles={phase_steps_cycles:?}, epochs={epochs:?}"
    );
    assert!(
        phase_steps_cycles
            .iter()
            .all(|step_cycles| *step_cycles <= SHORT_FADE_MAX_PHASE_STEP_CYCLES),
        "carrier phase must not take slip-sized jumps after fade recovery: phase_steps_cycles={phase_steps_cycles:?}, epochs={epochs:?}"
    );
}
