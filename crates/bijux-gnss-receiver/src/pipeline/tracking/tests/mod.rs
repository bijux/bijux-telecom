use super::*;
use crate::engine::receiver_config::{BandTrackingSpec, ReceiverPipelineConfig, TrackingParams};
use crate::engine::runtime::ReceiverRuntime;
use crate::pipeline::observations::observations_from_tracking_results_with_gps_anchor;
use crate::sim::synthetic::{generate_l1_ca, SyntheticSignalParams};
use bijux_gnss_core::api::{
    AcqHypothesis, AcqUncertainty, Chips, Constellation, Epoch, GpsTime, Hertz,
    ReceiverSampleTrace, SampleTime, SamplesFrame, SatId, Seconds, SignalBand, SignalCode,
    SignalComponentRole, TrackEpoch, GPS_L1_CA_CARRIER_HZ,
};
use bijux_gnss_signal::api::TrackingLoopProfile as SignalTrackingLoopProfile;
use bijux_gnss_signal::api::{
    advance_code_phase_seconds, delay_lock_loop_coefficients, discriminators,
    first_order_angular_loop_coefficients, normalize_dll_discriminator,
    phase_lock_loop_coefficients, sample_ca_code, sample_galileo_e1_cboc, samples_per_code,
    shared_path_code_rate_hz, signal_spec_gps_l5_i, wrapped_code_phase_delta_samples,
    LocalCodeModel, Prn,
};
use num_complex::Complex;
use serde::Deserialize;

mod carrier_phase_continuity;
mod channel_state_policy;
mod doppler_evidence;
mod incremental_tracking;
#[path = "loop_dynamics.rs"]
mod loop_dynamics;
#[path = "multipath_bias.rs"]
mod multipath_bias;
#[path = "navigation_time.rs"]
mod navigation_time;
mod reacquisition;
#[path = "reporting.rs"]
mod reporting;
#[path = "secondary_code_sync.rs"]
mod secondary_code_sync;
#[path = "signal_model.rs"]
mod signal_model;
mod tracking_loop_feedback;
mod uncertainty;
#[path = "vector_tracking.rs"]
mod vector_tracking;

fn empty_loop_state() -> super::LoopState {
    super::LoopState {
        carrier_hz: 0.0,
        carrier_phase_cycles: 0.0,
        carrier_rate_hz_per_s: 0.0,
        code_rate_hz: 0.0,
        code_rate_reference_hz: 0.0,
        code_phase_samples: 0.0,
        tracking_adaptation_state: Default::default(),
        tracking_loop_profile: SignalTrackingLoopProfile {
            dll_bw_hz: 2.0,
            pll_bw_hz: 15.0,
            fll_bw_hz: 10.0,
            integration_ms: 1,
        },
        signal_delay_alignment: None,
        subcarrier_code_phase_refined: false,
        acquisition_cn0_proxy_dbhz: 45.0,
        lock_reference_cn0_dbhz: 45.0,
        prev_prompt: None,
        prev_prompt_phase_cycles: None,
        secondary_code_prompt_history: std::collections::VecDeque::new(),
        secondary_code_sync: None,
        nav_bit_phase_offset_cycles: 0.0,
        nav_bit_transition_count: 0,
        pull_in_stable_epochs: 0,
        weak_cn0_epochs: 0,
        degraded_epochs: 0,
        prompt_power_reference: 0.0,
        prompt_cn0_window: std::collections::VecDeque::new(),
        code_error_window_samples: std::collections::VecDeque::new(),
        carrier_phase_error_window_cycles: std::collections::VecDeque::new(),
        doppler_error_window_hz: std::collections::VecDeque::new(),
        cn0_estimate_window_dbhz: std::collections::VecDeque::new(),
        unstable_discriminator_epochs: 0,
        state: ChannelState::Tracking,
        unlocked_count: 0,
        lost_reason: None,
        reacquisition_candidate: None,
        reacquisition_candidate_streak: 0,
        reacquisition_pending: false,
        reacquisition_attempt_epochs: 0,
        reacquisition_stable_tracking_epochs: 0,
    }
}

#[test]
fn short_fade_epoch_budget_reserves_post_fade_recovery_window() {
    let tracking_params = crate::engine::receiver_config::TrackingParams {
        early_late_spacing_chips: 0.5,
        dll_bw_hz: 2.0,
        pll_bw_hz: 15.0,
        fll_bw_hz: 10.0,
        integration_ms: 1,
    };

    assert_eq!(super::short_fade_epoch_budget(tracking_params), 105);
}

#[test]
fn tracking_epoch_count_includes_trailing_partial_epoch() {
    assert_eq!(super::tracking_epoch_count(60, 20), 3);
    assert_eq!(super::tracking_epoch_count(61, 20), 4);
    assert_eq!(super::tracking_epoch_count(0, 20), 0);
}

#[test]
fn code_value_at_phase_wraps_fractional_chip_phases() {
    let code = vec![1_i8, -1, 1, -1];

    assert_eq!(bijux_gnss_signal::api::code_value_at_phase(&code, 0.25).unwrap(), 1.0);
    assert_eq!(bijux_gnss_signal::api::code_value_at_phase(&code, 1.75).unwrap(), -1.0);
    assert_eq!(bijux_gnss_signal::api::code_value_at_phase(&code, 4.10).unwrap(), 1.0);
    assert_eq!(bijux_gnss_signal::api::code_value_at_phase(&code, -0.10).unwrap(), -1.0);
}

#[test]
fn tracking_frame_start_offset_clamps_to_visible_frame_samples() {
    let frame = SamplesFrame::new(
        SampleTime { sample_index: 100, sample_rate_hz: 1_023_000.0 },
        Seconds(1.0 / 1_023_000.0),
        vec![Complex::new(0.0_f32, 0.0); 16],
    );

    assert_eq!(super::tracking_frame_start_offset(&frame, None), Some(0));
    assert_eq!(super::tracking_frame_start_offset(&frame, Some(90)), Some(0));
    assert_eq!(super::tracking_frame_start_offset(&frame, Some(108)), Some(8));
    assert_eq!(super::tracking_frame_start_offset(&frame, Some(116)), None);
}

#[test]
fn refresh_lock_reference_cn0_dbhz_only_updates_on_reliable_lock() {
    assert_eq!(super::refresh_lock_reference_cn0_dbhz(48.0, 52.0, true), 52.0);
    assert_eq!(super::refresh_lock_reference_cn0_dbhz(48.0, 30.0, false), 48.0);
    assert_eq!(super::refresh_lock_reference_cn0_dbhz(48.0, f64::NAN, true), 48.0);
}

#[test]
fn deterministic_transition_rule_holds_pull_in_until_carrier_converges() {
    let decision = super::deterministic_transition_rule(
        ChannelState::PullIn,
        true,
        false,
        false,
        None,
        1,
        0,
        100,
        false,
        None,
    );

    assert_eq!(decision.to_state, ChannelState::PullIn);
    assert_eq!(decision.reason, "carrier_pull_in");
    assert_eq!(decision.next_unlocked_count, 0);
    assert_eq!(decision.next_degraded_epochs, 0);
}

#[test]
fn prelock_cn0_refusal_trips_after_repeated_weak_epochs() {
    let (weak_epochs, supports_lock, refuse_lock) =
        super::update_prelock_cn0_refusal(ChannelState::PullIn, 2, 20.0);

    assert_eq!(weak_epochs, 3);
    assert!(!supports_lock);
    assert!(refuse_lock);
}

#[test]
fn prelock_cn0_refusal_resets_on_supported_signal_or_established_tracking() {
    let (weak_epochs, supports_lock, refuse_lock) =
        super::update_prelock_cn0_refusal(ChannelState::PullIn, 2, 30.0);
    assert_eq!(weak_epochs, 0);
    assert!(supports_lock);
    assert!(!refuse_lock);

    let (weak_epochs, supports_lock, refuse_lock) =
        super::update_prelock_cn0_refusal(ChannelState::Tracking, 2, 20.0);
    assert_eq!(weak_epochs, 0);
    assert!(!supports_lock);
    assert!(!refuse_lock);

    let (weak_epochs, supports_lock, refuse_lock) =
        super::update_prelock_cn0_refusal(ChannelState::Degraded, 2, 20.0);
    assert_eq!(weak_epochs, 0);
    assert!(!supports_lock);
    assert!(!refuse_lock);
}

fn track_epoch_with_state(
    epoch_index: u32,
    lock: bool,
    lock_state: &str,
    lock_state_reason: Option<&str>,
) -> TrackEpoch {
    TrackEpoch {
        epoch: Epoch { index: epoch_index as u64 },
        sample_index: epoch_index as u64,
        sat: SatId { constellation: Constellation::Gps, prn: 1 },
        carrier_hz: Hertz(epoch_index as f64),
        code_phase_samples: Chips(epoch_index as f64 + 0.5),
        lock,
        lock_state: lock_state.to_string(),
        lock_state_reason: lock_state_reason.map(str::to_string),
        ..TrackEpoch::default()
    }
}
