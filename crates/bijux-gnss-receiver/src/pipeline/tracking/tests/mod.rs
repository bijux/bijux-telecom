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

#[test]
fn incremental_tracking_matches_single_frame_tracking() {
    let config = crate::engine::receiver_config::ReceiverPipelineConfig {
        sampling_freq_hz: 1_023_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 4,
        ..crate::engine::receiver_config::ReceiverPipelineConfig::default()
    };
    let runtime = crate::engine::runtime::ReceiverRuntime::default();
    let tracking = Tracking::new(config.clone(), runtime.clone());
    let sat = SatId { constellation: bijux_gnss_core::api::Constellation::Gps, prn: 3 };
    let duration_s = 0.012;
    let frame = crate::sim::synthetic::generate_l1_ca(
        &config,
        crate::sim::synthetic::SyntheticSignalParams {
            sat,
            glonass_frequency_channel: None,
            signal_band: bijux_gnss_core::api::SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Unknown,
            doppler_hz: 1_000.0,
            code_phase_chips: 10.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 48.0,
            navigation_data: false.into(),
        },
        7,
        duration_s,
    );
    let acquisition = bijux_gnss_core::api::AcqResult {
        sat,
        signal_band: bijux_gnss_core::api::SignalBand::L1,
        signal_code: bijux_gnss_core::api::SignalCode::Unknown,
        glonass_frequency_channel: None,
        source_time: bijux_gnss_core::api::ReceiverSampleTrace::from_sample_time(frame.t0),
        candidate_rank: 1,
        is_primary_candidate: true,
        doppler_hz: bijux_gnss_core::api::Hertz(1_000.0),
        doppler_rate_hz_per_s: 0.0,
        carrier_hz: bijux_gnss_core::api::Hertz(1_000.0),
        code_phase_samples: 10,
        peak: 10.0,
        second_peak: 2.0,
        mean: 1.0,
        peak_mean_ratio: 10.0,
        peak_second_ratio: 5.0,
        cn0_proxy: 48.0,
        score: 0.98,
        hypothesis: AcqHypothesis::Accepted,
        assumptions: None,
        evidence: Vec::new(),
        threshold_provenance: None,
        explain_selection_reason: None,
        doppler_refinement: None,
        code_phase_refinement: None,
        signal_delay_alignment: None,
        uncertainty: None,
    };

    let single = tracking.track_from_acquisition(&frame, &[acquisition.clone()]);
    let mut incremental = tracking.begin_incremental_tracking(&[acquisition]);
    for chunk in split_frame(&frame, 4 * 1_023) {
        tracking.track_incremental_frame(&mut incremental, &chunk);
    }
    let streamed = tracking.finish_incremental_tracking(incremental);

    assert_eq!(single.len(), streamed.len());
    assert_eq!(single[0].epochs.len(), streamed[0].epochs.len());
    let single_keys = single[0]
        .epochs
        .iter()
        .map(|epoch| {
            (
                epoch.epoch.index,
                epoch.sample_index,
                epoch.lock,
                epoch.lock_state.clone(),
                epoch.lock_state_reason.clone(),
            )
        })
        .collect::<Vec<_>>();
    let streamed_keys = streamed[0]
        .epochs
        .iter()
        .map(|epoch| {
            (
                epoch.epoch.index,
                epoch.sample_index,
                epoch.lock,
                epoch.lock_state.clone(),
                epoch.lock_state_reason.clone(),
            )
        })
        .collect::<Vec<_>>();
    assert_eq!(single_keys, streamed_keys);
}

#[test]
fn begin_incremental_tracking_prioritizes_stronger_trackable_acquisitions() {
    let config = crate::engine::receiver_config::ReceiverPipelineConfig {
        channels: 2,
        ..crate::engine::receiver_config::ReceiverPipelineConfig::default()
    };
    let tracking = Tracking::new(config, ReceiverRuntime::default());
    let acquisitions = vec![
        bijux_gnss_core::api::AcqResult {
            sat: SatId { constellation: Constellation::Gps, prn: 3 },
            signal_band: SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Unknown,
            glonass_frequency_channel: None,
            source_time: ReceiverSampleTrace::default(),
            candidate_rank: 1,
            is_primary_candidate: true,
            doppler_hz: Hertz(0.0),
            doppler_rate_hz_per_s: 0.0,
            carrier_hz: Hertz(0.0),
            code_phase_samples: 0,
            peak: 1.0,
            second_peak: 0.1,
            mean: 0.01,
            peak_mean_ratio: 6.0,
            peak_second_ratio: 2.0,
            cn0_proxy: 38.0,
            score: 1.4,
            hypothesis: AcqHypothesis::Ambiguous,
            assumptions: None,
            evidence: Vec::new(),
            threshold_provenance: None,
            explain_selection_reason: None,
            doppler_refinement: None,
            code_phase_refinement: None,
            signal_delay_alignment: None,
            uncertainty: None,
        },
        bijux_gnss_core::api::AcqResult {
            sat: SatId { constellation: Constellation::Gps, prn: 7 },
            signal_band: SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Unknown,
            glonass_frequency_channel: None,
            source_time: ReceiverSampleTrace::default(),
            candidate_rank: 1,
            is_primary_candidate: true,
            doppler_hz: Hertz(0.0),
            doppler_rate_hz_per_s: 0.0,
            carrier_hz: Hertz(0.0),
            code_phase_samples: 0,
            peak: 1.0,
            second_peak: 0.1,
            mean: 0.01,
            peak_mean_ratio: 8.0,
            peak_second_ratio: 3.0,
            cn0_proxy: 52.0,
            score: 5.0,
            hypothesis: AcqHypothesis::Accepted,
            assumptions: None,
            evidence: Vec::new(),
            threshold_provenance: None,
            explain_selection_reason: None,
            doppler_refinement: None,
            code_phase_refinement: None,
            signal_delay_alignment: None,
            uncertainty: None,
        },
        bijux_gnss_core::api::AcqResult {
            sat: SatId { constellation: Constellation::Gps, prn: 23 },
            signal_band: SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Unknown,
            glonass_frequency_channel: None,
            source_time: ReceiverSampleTrace::default(),
            candidate_rank: 1,
            is_primary_candidate: true,
            doppler_hz: Hertz(0.0),
            doppler_rate_hz_per_s: 0.0,
            carrier_hz: Hertz(0.0),
            code_phase_samples: 0,
            peak: 1.0,
            second_peak: 0.1,
            mean: 0.01,
            peak_mean_ratio: 9.0,
            peak_second_ratio: 3.5,
            cn0_proxy: 48.0,
            score: 3.5,
            hypothesis: AcqHypothesis::Ambiguous,
            assumptions: None,
            evidence: Vec::new(),
            threshold_provenance: None,
            explain_selection_reason: None,
            doppler_refinement: None,
            code_phase_refinement: None,
            signal_delay_alignment: None,
            uncertainty: None,
        },
    ];

    let incremental = tracking.begin_incremental_tracking(&acquisitions);
    let selected_prns =
        incremental.channels.iter().map(|channel| channel.sat.prn).collect::<Vec<_>>();

    assert_eq!(selected_prns, vec![7, 23]);
}

#[test]
fn begin_incremental_tracking_preserves_glonass_channel_and_carrier_seed() {
    let channel =
        bijux_gnss_core::api::GlonassFrequencyChannel::new(-4).expect("channel -4 must be valid");
    let carrier_hz = 2_048_125.0;
    let config = crate::engine::receiver_config::ReceiverPipelineConfig {
        code_freq_basis_hz: 511_000.0,
        code_length: 511,
        ..crate::engine::receiver_config::ReceiverPipelineConfig::default()
    };
    let tracking = Tracking::new(config, ReceiverRuntime::default());
    let acquisition = bijux_gnss_core::api::AcqResult {
        sat: SatId { constellation: Constellation::Glonass, prn: 8 },
        signal_band: SignalBand::L1,
        signal_code: bijux_gnss_core::api::SignalCode::Unknown,
        glonass_frequency_channel: Some(channel),
        source_time: ReceiverSampleTrace::default(),
        candidate_rank: 1,
        is_primary_candidate: true,
        doppler_hz: Hertz(125.0),
        doppler_rate_hz_per_s: 0.0,
        carrier_hz: Hertz(carrier_hz),
        code_phase_samples: 37,
        peak: 1.0,
        second_peak: 0.1,
        mean: 0.01,
        peak_mean_ratio: 8.0,
        peak_second_ratio: 3.0,
        cn0_proxy: 48.0,
        score: 5.0,
        hypothesis: AcqHypothesis::Accepted,
        assumptions: None,
        evidence: Vec::new(),
        threshold_provenance: None,
        explain_selection_reason: None,
        doppler_refinement: None,
        code_phase_refinement: None,
        signal_delay_alignment: None,
        uncertainty: None,
    };

    let incremental = tracking.begin_incremental_tracking(&[acquisition]);
    let channel_state = incremental.channels.first().expect("GLONASS tracking channel");

    assert_eq!(channel_state.signal_model.signal_band, SignalBand::L1);
    assert_eq!(channel_state.signal_model.glonass_frequency_channel, Some(channel));
    assert_eq!(channel_state.state.carrier_hz, carrier_hz);
    assert!((channel_state.signal_model.code_rate_hz - 511_000.0).abs() <= f64::EPSILON);
    assert_eq!(channel_state.signal_model.code_length, 511);
}

#[test]
fn begin_incremental_tracking_seeds_code_rate_from_carrier_aid() {
    let config = crate::engine::receiver_config::ReceiverPipelineConfig {
        intermediate_freq_hz: GPS_L1_CA_CARRIER_HZ.value()
            - signal_spec_gps_l5_i().carrier_hz.value(),
        code_freq_basis_hz: 10_230_000.0,
        code_length: 10_230,
        ..crate::engine::receiver_config::ReceiverPipelineConfig::default()
    };
    let tracking = Tracking::new(config, ReceiverRuntime::default());
    let acquisition = bijux_gnss_core::api::AcqResult {
        sat: SatId { constellation: Constellation::Gps, prn: 18 },
        signal_band: SignalBand::L5,
        signal_code: SignalCode::L5I,
        glonass_frequency_channel: None,
        source_time: ReceiverSampleTrace::default(),
        candidate_rank: 1,
        is_primary_candidate: true,
        doppler_hz: Hertz(1_500.0),
        doppler_rate_hz_per_s: 0.0,
        carrier_hz: Hertz(1_500.0),
        code_phase_samples: 37,
        peak: 1.0,
        second_peak: 0.1,
        mean: 0.01,
        peak_mean_ratio: 8.0,
        peak_second_ratio: 3.0,
        cn0_proxy: 48.0,
        score: 5.0,
        hypothesis: AcqHypothesis::Accepted,
        assumptions: None,
        evidence: Vec::new(),
        threshold_provenance: None,
        explain_selection_reason: None,
        doppler_refinement: None,
        code_phase_refinement: None,
        signal_delay_alignment: None,
        uncertainty: None,
    };

    let incremental = tracking.begin_incremental_tracking(&[acquisition]);
    let channel_state = incremental.channels.first().expect("GPS L5 tracking channel");
    let expected_code_rate_hz =
        shared_path_code_rate_hz(1_500.0, signal_spec_gps_l5_i(), signal_spec_gps_l5_i())
            .expect("carrier-aided code rate");

    assert!((channel_state.state.code_rate_hz - expected_code_rate_hz).abs() <= 1.0e-9);
    assert!((channel_state.state.code_rate_reference_hz - expected_code_rate_hz).abs() <= 1.0e-9);
}

#[test]
fn begin_incremental_tracking_refuses_incompatible_carrier_aiding_seed() {
    let config = crate::engine::receiver_config::ReceiverPipelineConfig {
        intermediate_freq_hz: GPS_L1_CA_CARRIER_HZ.value()
            - signal_spec_gps_l5_i().carrier_hz.value(),
        code_freq_basis_hz: 10_230_000.0,
        code_length: 10_230,
        ..crate::engine::receiver_config::ReceiverPipelineConfig::default()
    };
    let tracking = Tracking::new(config, ReceiverRuntime::default());
    let acquisition = bijux_gnss_core::api::AcqResult {
        sat: SatId { constellation: Constellation::Gps, prn: 18 },
        signal_band: SignalBand::L5,
        signal_code: SignalCode::L5I,
        glonass_frequency_channel: None,
        source_time: ReceiverSampleTrace::default(),
        candidate_rank: 1,
        is_primary_candidate: true,
        doppler_hz: Hertz(1_500.0),
        doppler_rate_hz_per_s: 0.0,
        carrier_hz: Hertz(GPS_L1_CA_CARRIER_HZ.value() + 1_500.0),
        code_phase_samples: 37,
        peak: 1.0,
        second_peak: 0.1,
        mean: 0.01,
        peak_mean_ratio: 8.0,
        peak_second_ratio: 3.0,
        cn0_proxy: 48.0,
        score: 5.0,
        hypothesis: AcqHypothesis::Accepted,
        assumptions: None,
        evidence: Vec::new(),
        threshold_provenance: None,
        explain_selection_reason: None,
        doppler_refinement: None,
        code_phase_refinement: None,
        signal_delay_alignment: None,
        uncertainty: None,
    };

    let incremental = tracking.begin_incremental_tracking(&[acquisition]);

    assert!(
        incremental.channels.is_empty(),
        "wrong-band carrier seeds must not enter carrier-aided tracking: {incremental:?}",
    );
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

fn split_frame(frame: &SamplesFrame, chunk_len: usize) -> Vec<SamplesFrame> {
    let mut chunks = Vec::new();
    let mut start = 0usize;
    while start < frame.len() {
        let end = (start + chunk_len.max(1)).min(frame.len());
        chunks.push(SamplesFrame::new(
            SampleTime {
                sample_index: frame.t0.sample_index + start as u64,
                sample_rate_hz: frame.t0.sample_rate_hz,
            },
            Seconds(frame.dt_s.0),
            frame.iq[start..end].to_vec(),
        ));
        start = end;
    }
    chunks
}
