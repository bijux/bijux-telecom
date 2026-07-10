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
    epoch_indices_with_lock_state, epoch_indices_with_lock_state_reason,
};

const SHORT_FADE_PRELOCK_CN0_DBHZ: f32 = 60.0;
const SHORT_FADE_RECOVERY_MIN_TRACKING_EPOCHS: usize = 5;

fn accepted_acquisition(sat: SatId, code_phase_samples: usize) -> AcqResult {
    AcqResult {
        sat,
        signal_band: SignalBand::L1,
        source_time: ReceiverSampleTrace::default(),
        candidate_rank: 1,
        is_primary_candidate: true,
        doppler_hz: Hertz(0.0),
        carrier_hz: Hertz(0.0),
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

fn assert_tracking_recovers_after_short_fade(fade_start_s: f64, fade_end_s: f64, duration_s: f64) {
    let config = tracking_config();
    let sat = SatId { constellation: Constellation::Gps, prn: 21 };
    let frame = generate_l1_ca_with_fades(
        &config,
        SyntheticSignalParams {
            sat,
            doppler_hz: 0.0,
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
    let tracks = tracking.track_from_acquisition(&frame, &[accepted_acquisition(sat, 0)]);
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
    assert_tracking_recovers_after_short_fade(0.030, 0.040, 0.090);
}

#[test]
fn tracking_marks_hundred_millisecond_signal_fade_as_degraded_and_recovers() {
    assert_tracking_recovers_after_short_fade(0.030, 0.130, 0.200);
}
