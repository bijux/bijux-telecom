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

#[path = "loop_dynamics.rs"]
mod loop_dynamics;
#[path = "multipath_bias.rs"]
mod multipath_bias;
#[path = "navigation_time.rs"]
mod navigation_time;
#[path = "reporting.rs"]
mod reporting;
#[path = "secondary_code_sync.rs"]
mod secondary_code_sync;
#[path = "signal_model.rs"]
mod signal_model;
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
fn tracking_uncertainty_rewards_longer_coherent_integration() {
    let state = empty_loop_state();
    let short = super::estimate_tracking_uncertainty(
        &state,
        super::TrackingUncertaintyInputs {
            samples_per_chip: 4.0,
            dll_err: 0.0,
            pll_err_rad: 0.0,
            fll_err_hz: 0.0,
            cn0_dbhz: 45.0,
            cn0_reference_dbhz: 45.0,
            integration_ms: 1,
            channel_locked: true,
            dll_locked: true,
            anti_false_lock: false,
            cycle_slip: false,
            channel_state: ChannelState::Tracking,
        },
    );
    let long = super::estimate_tracking_uncertainty(
        &state,
        super::TrackingUncertaintyInputs {
            integration_ms: 10,
            ..super::TrackingUncertaintyInputs {
                samples_per_chip: 4.0,
                dll_err: 0.0,
                pll_err_rad: 0.0,
                fll_err_hz: 0.0,
                cn0_dbhz: 45.0,
                cn0_reference_dbhz: 45.0,
                integration_ms: 1,
                channel_locked: true,
                dll_locked: true,
                anti_false_lock: false,
                cycle_slip: false,
                channel_state: ChannelState::Tracking,
            }
        },
    );

    assert!(
            long.code_phase_samples < short.code_phase_samples,
            "longer coherent integration should tighten code-phase uncertainty: short={short:?} long={long:?}"
        );
}

#[test]
fn tracking_uncertainty_penalizes_dll_unlock() {
    let state = empty_loop_state();
    let locked = super::estimate_tracking_uncertainty(
        &state,
        super::TrackingUncertaintyInputs {
            samples_per_chip: 4.0,
            dll_err: 0.0,
            pll_err_rad: 0.0,
            fll_err_hz: 0.0,
            cn0_dbhz: 45.0,
            cn0_reference_dbhz: 45.0,
            integration_ms: 1,
            channel_locked: true,
            dll_locked: true,
            anti_false_lock: false,
            cycle_slip: false,
            channel_state: ChannelState::Tracking,
        },
    );
    let unlocked = super::estimate_tracking_uncertainty(
        &state,
        super::TrackingUncertaintyInputs {
            dll_locked: false,
            ..super::TrackingUncertaintyInputs {
                samples_per_chip: 4.0,
                dll_err: 0.0,
                pll_err_rad: 0.0,
                fll_err_hz: 0.0,
                cn0_dbhz: 45.0,
                cn0_reference_dbhz: 45.0,
                integration_ms: 1,
                channel_locked: true,
                dll_locked: true,
                anti_false_lock: false,
                cycle_slip: false,
                channel_state: ChannelState::Tracking,
            }
        },
    );

    assert!(
            unlocked.code_phase_samples > locked.code_phase_samples,
            "loss of DLL lock should inflate code-phase uncertainty: locked={locked:?} unlocked={unlocked:?}"
        );
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
fn apply_dll_code_loop_decreases_code_rate_for_positive_discriminator() {
    let coherent_integration_s = 5_000.0 / 1_023_000.0;
    let update = super::apply_dll_code_loop(super::CodeLoopInput {
        current_code_rate_hz: 1_023_000.0,
        previous_reference_code_rate_hz: 1_023_000.0,
        reference_code_rate_hz: 1_023_000.0,
        current_code_phase_samples: 250.0,
        epoch_len_samples: 5_000,
        coherent_integration_s,
        nominal_code_rate_hz: 1_023_000.0,
        dll_bw_hz: 2.0,
        dll_err: 0.25,
        samples_per_chip: 4.887585532746823,
        samples_per_code: 5_000,
    });

    let expected = 1_023_000.0
        - delay_lock_loop_coefficients(2.0, coherent_integration_s).rate_gain_hz_per_chip * 0.25;
    assert!((update.code_rate_hz - expected).abs() < 1.0e-9, "{update:?}");
}

#[test]
fn apply_dll_code_loop_follows_reference_code_rate_step_without_dll_error() {
    let update = super::apply_dll_code_loop(super::CodeLoopInput {
        current_code_rate_hz: 1_023_000.0,
        previous_reference_code_rate_hz: 1_023_000.0,
        reference_code_rate_hz: 1_023_450.0,
        current_code_phase_samples: 250.0,
        epoch_len_samples: 5_000,
        coherent_integration_s: 5_000.0 / 1_023_000.0,
        nominal_code_rate_hz: 1_023_000.0,
        dll_bw_hz: 2.0,
        dll_err: 0.0,
        samples_per_chip: 4.887585532746823,
        samples_per_code: 5_000,
    });

    assert!((update.code_rate_hz - 1_023_450.0).abs() < 1.0e-9, "{update:?}");
}

#[test]
fn apply_dll_code_loop_uses_coherent_interval_to_scale_gain() {
    let short = super::apply_dll_code_loop(super::CodeLoopInput {
        current_code_rate_hz: 1_023_000.0,
        previous_reference_code_rate_hz: 1_023_000.0,
        reference_code_rate_hz: 1_023_000.0,
        current_code_phase_samples: 250.0,
        epoch_len_samples: 4_092,
        coherent_integration_s: 0.001,
        nominal_code_rate_hz: 1_023_000.0,
        dll_bw_hz: 2.0,
        dll_err: 0.25,
        samples_per_chip: 4.0,
        samples_per_code: 4_092,
    });
    let long = super::apply_dll_code_loop(super::CodeLoopInput {
        current_code_rate_hz: 1_023_000.0,
        previous_reference_code_rate_hz: 1_023_000.0,
        reference_code_rate_hz: 1_023_000.0,
        current_code_phase_samples: 250.0,
        epoch_len_samples: 40_920,
        coherent_integration_s: 0.010,
        nominal_code_rate_hz: 1_023_000.0,
        dll_bw_hz: 2.0,
        dll_err: 0.25,
        samples_per_chip: 4.0,
        samples_per_code: 40_920,
    });

    assert!(
        (long.code_rate_hz - 1_023_000.0).abs() > (short.code_rate_hz - 1_023_000.0).abs(),
        "short={short:?} long={long:?}"
    );
}

#[test]
fn apply_dll_code_loop_pulls_positive_code_error_toward_prompt() {
    let current_code_phase_samples = 250.0;
    let update = super::apply_dll_code_loop(super::CodeLoopInput {
        current_code_rate_hz: 1_023_000.0,
        previous_reference_code_rate_hz: 1_023_000.0,
        reference_code_rate_hz: 1_023_000.0,
        current_code_phase_samples,
        epoch_len_samples: 5_000,
        coherent_integration_s: 5_000.0 / 1_023_000.0,
        nominal_code_rate_hz: 1_023_000.0,
        dll_bw_hz: 2.0,
        dll_err: 0.4,
        samples_per_chip: 4.887585532746823,
        samples_per_code: 5_000,
    });

    assert!(update.code_phase_samples > current_code_phase_samples, "update={update:?}");
}

#[test]
fn apply_dll_code_loop_pulls_negative_code_error_toward_prompt() {
    let current_code_phase_samples = 250.0;
    let update = super::apply_dll_code_loop(super::CodeLoopInput {
        current_code_rate_hz: 1_023_000.0,
        previous_reference_code_rate_hz: 1_023_000.0,
        reference_code_rate_hz: 1_023_000.0,
        current_code_phase_samples,
        epoch_len_samples: 5_000,
        coherent_integration_s: 5_000.0 / 1_023_000.0,
        nominal_code_rate_hz: 1_023_000.0,
        dll_bw_hz: 2.0,
        dll_err: -0.4,
        samples_per_chip: 4.887585532746823,
        samples_per_code: 5_000,
    });

    assert!(update.code_phase_samples < current_code_phase_samples, "update={update:?}");
}

#[test]
fn correlate_epoch_aligns_prompt_with_non_integer_rate_sampled_code() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_000_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 12,
        ..ReceiverPipelineConfig::default()
    };
    let tracking = Tracking::new(config.clone(), ReceiverRuntime::default());
    let sample_count =
        samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length);
    let code_phase_chips = 245.25;
    let sat = SatId { constellation: bijux_gnss_core::api::Constellation::Gps, prn: 3 };
    let samples = sample_ca_code(
        Prn(sat.prn),
        config.sampling_freq_hz,
        config.code_freq_basis_hz,
        code_phase_chips,
        sample_count,
    )
    .expect("valid sampled code");
    let frame = SamplesFrame::new(
        SampleTime { sample_index: 0, sample_rate_hz: config.sampling_freq_hz },
        Seconds(1.0 / config.sampling_freq_hz),
        samples.into_iter().map(|value| Complex::new(value, 0.0)).collect(),
    );
    let code_phase_samples = crate::sim::synthetic::expected_acquisition_code_phase_samples_f64(
        &config,
        &frame,
        code_phase_chips,
    );

    let correlator = tracking.correlate_epoch(
        &frame,
        sat,
        0.0,
        0.0,
        config.code_freq_basis_hz,
        code_phase_samples,
        0.5,
    );

    assert!(correlator.prompt.norm() > correlator.early.norm());
    assert!(correlator.prompt.norm() > correlator.late.norm());
}

#[test]
fn correlate_epoch_uses_tracked_code_rate_for_code_rate_offset_signal() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 12,
        ..ReceiverPipelineConfig::default()
    };
    let tracking = Tracking::new(config.clone(), ReceiverRuntime::default());
    let sample_count =
        samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length);
    let signal_code_rate_hz = config.code_freq_basis_hz + 1_200.0;
    let code_phase_chips = 87.375;
    let sat = SatId { constellation: bijux_gnss_core::api::Constellation::Gps, prn: 9 };
    let samples = sample_ca_code(
        Prn(sat.prn),
        config.sampling_freq_hz,
        signal_code_rate_hz,
        code_phase_chips,
        sample_count,
    )
    .expect("valid sampled code with signal code-rate offset");
    let frame = SamplesFrame::new(
        SampleTime { sample_index: 0, sample_rate_hz: config.sampling_freq_hz },
        Seconds(1.0 / config.sampling_freq_hz),
        samples.into_iter().map(|value| Complex::new(value, 0.0)).collect(),
    );
    let code_phase_samples = crate::sim::synthetic::expected_acquisition_code_phase_samples_f64(
        &config,
        &frame,
        code_phase_chips,
    );

    let nominal = tracking.correlate_epoch(
        &frame,
        sat,
        0.0,
        0.0,
        config.code_freq_basis_hz,
        code_phase_samples,
        0.5,
    );
    let matched = tracking.correlate_epoch(
        &frame,
        sat,
        0.0,
        0.0,
        signal_code_rate_hz,
        code_phase_samples,
        0.5,
    );

    assert!(
        matched.prompt.norm() > nominal.prompt.norm(),
        "matched_prompt={} nominal_prompt={}",
        matched.prompt.norm(),
        nominal.prompt.norm(),
    );
    assert!(matched.prompt.norm() > matched.early.norm());
    assert!(matched.prompt.norm() > matched.late.norm());
}

#[test]
fn correlate_epoch_uses_tracked_carrier_phase_for_phase_offset_signal() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 12,
        ..ReceiverPipelineConfig::default()
    };
    let tracking = Tracking::new(config.clone(), ReceiverRuntime::default());
    let sat = SatId { constellation: bijux_gnss_core::api::Constellation::Gps, prn: 5 };
    let carrier_phase_rad = 0.35;
    let carrier_phase_cycles = carrier_phase_rad / std::f64::consts::TAU;
    let frame = generate_l1_ca(
        &config,
        SyntheticSignalParams {
            sat,
            glonass_frequency_channel: None,
            signal_band: bijux_gnss_core::api::SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Unknown,
            doppler_hz: 0.0,
            code_phase_chips: 0.0,
            carrier_phase_rad,
            cn0_db_hz: 70.0,
            navigation_data: false.into(),
        },
        0xC0A5_1E,
        0.001,
    );

    let unmatched =
        tracking.correlate_epoch(&frame, sat, 0.0, 0.0, config.code_freq_basis_hz, 0.0, 0.5);
    let matched = tracking.correlate_epoch(
        &frame,
        sat,
        0.0,
        carrier_phase_cycles,
        config.code_freq_basis_hz,
        0.0,
        0.5,
    );

    assert!(
        matched.prompt.re > unmatched.prompt.re,
        "matched={:?} unmatched={:?}",
        matched.prompt,
        unmatched.prompt,
    );
    assert!(
        matched.prompt.im.abs() < unmatched.prompt.im.abs(),
        "matched={:?} unmatched={:?}",
        matched.prompt,
        unmatched.prompt,
    );
}

#[test]
fn correlate_epoch_keeps_carrier_phase_aligned_across_nonzero_epoch_starts() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 12,
        ..ReceiverPipelineConfig::default()
    };
    let tracking = Tracking::new(config.clone(), ReceiverRuntime::default());
    let sat = SatId { constellation: bijux_gnss_core::api::Constellation::Gps, prn: 5 };
    let carrier_hz = 120.0;
    let epoch_len_samples =
        samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length);
    let second_epoch_start_s = epoch_len_samples as f64 / config.sampling_freq_hz;
    let second_epoch_start_phase_cycles = 0.18;
    let initial_phase_cycles = second_epoch_start_phase_cycles - carrier_hz * second_epoch_start_s;
    let frame = generate_l1_ca(
        &config,
        SyntheticSignalParams {
            sat,
            glonass_frequency_channel: None,
            signal_band: bijux_gnss_core::api::SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Unknown,
            doppler_hz: carrier_hz,
            code_phase_chips: 0.0,
            carrier_phase_rad: initial_phase_cycles * std::f64::consts::TAU,
            cn0_db_hz: 90.0,
            navigation_data: false.into(),
        },
        0xC0A5_1E,
        0.002,
    );
    let second_epoch = super::frame_slice(&frame, epoch_len_samples, epoch_len_samples * 2);

    let correlator = tracking.correlate_epoch(
        &second_epoch,
        sat,
        carrier_hz,
        second_epoch_start_phase_cycles,
        config.code_freq_basis_hz,
        0.0,
        0.5,
    );

    assert!(
        correlator.prompt.re > correlator.prompt.im.abs() * 10.0,
        "prompt={:?}",
        correlator.prompt,
    );
}

#[test]
fn apply_carrier_loop_advances_phase_and_frequency_from_pll_error() {
    let update = super::apply_carrier_loop(super::CarrierLoopInput {
        current_carrier_hz: 1_000.0,
        current_carrier_phase_cycles: 12.0,
        current_carrier_rate_hz_per_s: 0.0,
        epoch_len_samples: 4_092,
        sample_rate_hz: 4_092_000.0,
        coherent_integration_s: 0.001,
        pll_bw_hz: 8.0,
        pll_err_rad: 0.25,
        fll_bw_hz: 0.0,
        fll_err_hz: 0.0,
        apply_fll: false,
        apply_pll_frequency: true,
    });

    let pll_coefficients = phase_lock_loop_coefficients(8.0, 0.001);
    assert!(
        (update.carrier_hz - (1_000.0 + pll_coefficients.frequency_gain_hz_per_rad * 0.25)).abs()
            < 1.0e-9,
        "{update:?}"
    );
    assert!(
        (update.carrier_rate_hz_per_s
            - pll_coefficients.frequency_rate_gain_hz_per_s_per_rad * 0.25)
            .abs()
            < 1.0e-9,
        "{update:?}"
    );
    let expected_phase_cycles = 12.0
        + (1_000.0 + (1_000.0 + pll_coefficients.frequency_gain_hz_per_rad * 0.25)) * 0.0005
        + pll_coefficients.phase_blend * 0.25 / std::f64::consts::TAU;
    assert!((update.carrier_phase_cycles - expected_phase_cycles).abs() < 1.0e-9, "{update:?}",);
}

#[test]
fn apply_carrier_loop_uses_fll_correction_during_pull_in() {
    let update = super::apply_carrier_loop(super::CarrierLoopInput {
        current_carrier_hz: 80.0,
        current_carrier_phase_cycles: 12.0,
        current_carrier_rate_hz_per_s: 0.0,
        epoch_len_samples: 4_092,
        sample_rate_hz: 4_092_000.0,
        coherent_integration_s: 0.001,
        pll_bw_hz: 8.0,
        pll_err_rad: 0.25,
        fll_bw_hz: 10.0,
        fll_err_hz: 30.0,
        apply_fll: true,
        apply_pll_frequency: false,
    });

    let fll_coefficients = first_order_angular_loop_coefficients(10.0, 0.001);
    let expected_carrier_hz = 80.0 + (30.0 * fll_coefficients.error_blend).clamp(-40.0, 40.0);
    assert!((update.carrier_hz - expected_carrier_hz).abs() < 1.0e-9, "{update:?}");
}

#[test]
fn apply_carrier_loop_accumulates_unwrapped_phase_across_epochs() {
    let first = super::apply_carrier_loop(super::CarrierLoopInput {
        current_carrier_hz: 1_500.0,
        current_carrier_phase_cycles: 128.25,
        current_carrier_rate_hz_per_s: 0.0,
        epoch_len_samples: 4_092,
        sample_rate_hz: 4_092_000.0,
        coherent_integration_s: 0.001,
        pll_bw_hz: 8.0,
        pll_err_rad: 0.0,
        fll_bw_hz: 0.0,
        fll_err_hz: 0.0,
        apply_fll: false,
        apply_pll_frequency: true,
    });
    let second = super::apply_carrier_loop(super::CarrierLoopInput {
        current_carrier_hz: first.carrier_hz,
        current_carrier_phase_cycles: first.carrier_phase_cycles,
        current_carrier_rate_hz_per_s: first.carrier_rate_hz_per_s,
        epoch_len_samples: 4_092,
        sample_rate_hz: 4_092_000.0,
        coherent_integration_s: 0.001,
        pll_bw_hz: 8.0,
        pll_err_rad: 0.0,
        fll_bw_hz: 0.0,
        fll_err_hz: 0.0,
        apply_fll: false,
        apply_pll_frequency: true,
    });

    assert!((first.carrier_phase_cycles - 129.75).abs() < 1.0e-9, "{first:?}");
    assert!((second.carrier_phase_cycles - 131.25).abs() < 1.0e-9, "{second:?}");
    assert!(
        (second.carrier_phase_cycles - first.carrier_phase_cycles - 1.5).abs() < 1.0e-9,
        "{first:?} {second:?}"
    );
}

#[test]
fn apply_carrier_loop_preserves_continuous_phase_for_negative_doppler() {
    let update = super::apply_carrier_loop(super::CarrierLoopInput {
        current_carrier_hz: -850.0,
        current_carrier_phase_cycles: 512.875,
        current_carrier_rate_hz_per_s: 0.0,
        epoch_len_samples: 8_184,
        sample_rate_hz: 4_092_000.0,
        coherent_integration_s: 0.002,
        pll_bw_hz: 8.0,
        pll_err_rad: 0.0,
        fll_bw_hz: 0.0,
        fll_err_hz: 0.0,
        apply_fll: false,
        apply_pll_frequency: true,
    });

    assert!((update.carrier_hz + 850.0).abs() < 1.0e-9, "{update:?}");
    assert!((update.carrier_phase_cycles - 511.175).abs() < 1.0e-9, "{update:?}");
}

#[test]
fn apply_carrier_loop_uses_coherent_interval_to_scale_frequency_gain() {
    let short = super::apply_carrier_loop(super::CarrierLoopInput {
        current_carrier_hz: 1_000.0,
        current_carrier_phase_cycles: 12.0,
        current_carrier_rate_hz_per_s: 0.0,
        epoch_len_samples: 4_092,
        sample_rate_hz: 4_092_000.0,
        coherent_integration_s: 0.001,
        pll_bw_hz: 8.0,
        pll_err_rad: 0.25,
        fll_bw_hz: 0.0,
        fll_err_hz: 0.0,
        apply_fll: false,
        apply_pll_frequency: true,
    });
    let long = super::apply_carrier_loop(super::CarrierLoopInput {
        current_carrier_hz: 1_000.0,
        current_carrier_phase_cycles: 12.0,
        current_carrier_rate_hz_per_s: 0.0,
        epoch_len_samples: 40_920,
        sample_rate_hz: 4_092_000.0,
        coherent_integration_s: 0.010,
        pll_bw_hz: 8.0,
        pll_err_rad: 0.25,
        fll_bw_hz: 0.0,
        fll_err_hz: 0.0,
        apply_fll: false,
        apply_pll_frequency: true,
    });

    assert!(
        (long.carrier_hz - 1_000.0).abs() > (short.carrier_hz - 1_000.0).abs(),
        "short={short:?} long={long:?}"
    );
}

#[test]
fn dll_pull_in_increases_code_rate_for_faster_signal() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 12,
        ..ReceiverPipelineConfig::default()
    };
    let tracking = Tracking::new(config.clone(), ReceiverRuntime::default());
    let samples_per_epoch =
        samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length);
    let signal_code_rate_hz = config.code_freq_basis_hz + 300.0;
    let code_phase_chips = 211.625;
    let sat = SatId { constellation: bijux_gnss_core::api::Constellation::Gps, prn: 14 };
    let samples = sample_ca_code(
        Prn(sat.prn),
        config.sampling_freq_hz,
        signal_code_rate_hz,
        code_phase_chips,
        samples_per_epoch,
    )
    .expect("valid sampled code with faster signal code rate");
    let frame = SamplesFrame::new(
        SampleTime { sample_index: 0, sample_rate_hz: config.sampling_freq_hz },
        Seconds(1.0 / config.sampling_freq_hz),
        samples.into_iter().map(|value| Complex::new(value, 0.0)).collect(),
    );
    let code_phase_samples = crate::sim::synthetic::expected_acquisition_code_phase_samples_f64(
        &config,
        &frame,
        code_phase_chips,
    );

    let correlator = tracking.correlate_epoch(
        &frame,
        sat,
        0.0,
        0.0,
        config.code_freq_basis_hz,
        code_phase_samples,
        0.5,
    );
    let (dll_err, _, _, _) =
        discriminators(correlator.early, correlator.prompt, correlator.late, None);
    let code_loop = super::apply_dll_code_loop(super::CodeLoopInput {
        current_code_rate_hz: config.code_freq_basis_hz,
        previous_reference_code_rate_hz: config.code_freq_basis_hz,
        reference_code_rate_hz: config.code_freq_basis_hz,
        current_code_phase_samples: code_phase_samples,
        epoch_len_samples: samples_per_epoch,
        coherent_integration_s: samples_per_epoch as f64 / config.sampling_freq_hz,
        nominal_code_rate_hz: config.code_freq_basis_hz,
        dll_bw_hz: 900.0,
        dll_err,
        samples_per_chip: samples_per_epoch as f64 / config.code_length as f64,
        samples_per_code: samples_per_epoch,
    });

    assert!(
        code_loop.code_rate_hz > config.code_freq_basis_hz,
        "dll_err={dll_err} code_loop={code_loop:?}",
    );
}

#[test]
fn detect_sample_rate_mismatch_flags_persistent_phase_drift() {
    let sat = SatId { constellation: bijux_gnss_core::api::Constellation::Gps, prn: 11 };
    let epochs = [0.0, 24.0, 48.0, 72.0, 96.0]
        .into_iter()
        .enumerate()
        .map(|(index, code_phase_samples)| TrackEpoch {
            epoch: Epoch { index: index as u64 },
            sample_index: (index as u64) * 5_000,
            sat,
            code_phase_samples: Chips(code_phase_samples),
            dll_err: 0.35,
            cn0_dbhz: 46.0,
            lock: true,
            pll_lock: true,
            dll_lock: true,
            fll_lock: true,
            lock_state: ChannelState::Tracking.to_string(),
            lock_state_reason: Some("locked".to_string()),
            ..TrackEpoch::default()
        })
        .collect::<Vec<_>>();

    let diagnostic =
        super::detect_sample_rate_mismatch(&epochs, 5_000).expect("diagnostic must exist");
    assert_eq!(diagnostic.first_unstable_epoch_index, 1);
    assert!(diagnostic.max_abs_phase_step_samples >= 24.0);
}

#[test]
fn detect_sample_rate_mismatch_ignores_small_stable_phase_steps() {
    let sat = SatId { constellation: bijux_gnss_core::api::Constellation::Gps, prn: 11 };
    let epochs = [0.0, 1.5, 2.0, 1.0, 0.5]
        .into_iter()
        .enumerate()
        .map(|(index, code_phase_samples)| TrackEpoch {
            epoch: Epoch { index: index as u64 },
            sample_index: (index as u64) * 5_000,
            sat,
            code_phase_samples: Chips(code_phase_samples),
            dll_err: 0.05,
            cn0_dbhz: 46.0,
            lock: true,
            pll_lock: true,
            dll_lock: true,
            fll_lock: true,
            lock_state: ChannelState::Tracking.to_string(),
            lock_state_reason: Some("locked".to_string()),
            ..TrackEpoch::default()
        })
        .collect::<Vec<_>>();

    assert!(super::detect_sample_rate_mismatch(&epochs, 5_000).is_none());
}

#[test]
fn detect_sample_rate_mismatch_flags_catastrophic_pull_in_phase_jump() {
    let sat = SatId { constellation: bijux_gnss_core::api::Constellation::Gps, prn: 11 };
    let epochs = [0.0, 2.5, 4.0, 4_452.0, 4_452.5]
        .into_iter()
        .enumerate()
        .map(|(index, code_phase_samples)| TrackEpoch {
            epoch: Epoch { index: index as u64 },
            sample_index: (index as u64) * 5_050,
            sat,
            code_phase_samples: Chips(code_phase_samples),
            cn0_dbhz: 48.0,
            lock: true,
            fll_lock: index > 0,
            lock_state: ChannelState::PullIn.to_string(),
            lock_state_reason: Some("carrier_pull_in".to_string()),
            ..TrackEpoch::default()
        })
        .collect::<Vec<_>>();

    let diagnostic =
        super::detect_sample_rate_mismatch(&epochs, 5_050).expect("diagnostic must exist");
    assert_eq!(diagnostic.first_unstable_epoch_index, 3);
    assert!(diagnostic.max_abs_phase_step_samples >= 500.0);
}

#[test]
fn tracking_lock_detector_thresholds_derive_from_spacing_and_dynamics() {
    let tracking_params = super::TrackingParams {
        early_late_spacing_chips: 0.5,
        dll_bw_hz: 2.0,
        pll_bw_hz: 15.0,
        fll_bw_hz: 10.0,
        integration_ms: 1,
    };
    let high_resolution =
        super::tracking_lock_detector_thresholds(45.0, 0.001, 4.0, tracking_params, 0.0);
    let low_resolution =
        super::tracking_lock_detector_thresholds(45.0, 0.001, 1.0, tracking_params, 25_000.0);

    assert!(low_resolution.dll_lock > high_resolution.dll_lock);
    assert!(low_resolution.fll_lock_hz > high_resolution.fll_lock_hz);
    assert!(high_resolution.pll_hold_rad > high_resolution.pll_lock_rad);
}

#[test]
fn low_resolution_code_lock_retains_supported_tracking_when_carrier_is_stable() {
    assert!(super::low_resolution_code_lock(1.0, 0.5, true, true, true, false, false));
    assert!(super::low_resolution_code_lock(1.0, 0.5, true, false, true, false, false));
}

#[test]
fn low_resolution_code_lock_requires_prompt_and_lock_safety_guards() {
    assert!(!super::low_resolution_code_lock(1.0, 0.5, false, true, true, false, false));
    assert!(!super::low_resolution_code_lock(1.0, 0.5, true, false, false, false, false));
    assert!(!super::low_resolution_code_lock(1.0, 0.5, true, true, true, true, false));
    assert!(!super::low_resolution_code_lock(1.0, 0.5, true, true, true, false, true));
    assert!(!super::low_resolution_code_lock(4.0, 0.5, true, true, true, false, false));
}

#[test]
fn clean_seeded_tracking_clears_false_lock_once_loops_converge() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 1,
        early_late_spacing_chips: 0.5,
        dll_bw_hz: 2.0,
        pll_bw_hz: 18.0,
        fll_bw_hz: 12.0,
        ..ReceiverPipelineConfig::default()
    };
    let sat = SatId { constellation: Constellation::Gps, prn: 7 };
    let frame = generate_l1_ca(
        &config,
        SyntheticSignalParams {
            sat,
            glonass_frequency_channel: None,
            signal_band: bijux_gnss_core::api::SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Unknown,
            doppler_hz: -750.0,
            code_phase_chips: 211.25,
            carrier_phase_rad: 0.2,
            cn0_db_hz: 52.0,
            navigation_data: false.into(),
        },
        0x710C_A000,
        0.012,
    );
    let code_phase_samples =
        crate::sim::synthetic::expected_acquisition_code_phase_samples(&config, &frame, 211.25);
    let tracking = Tracking::new(config, ReceiverRuntime::default());
    let tracks = tracking.track_from_acquisition(
        &frame,
        &[bijux_gnss_core::api::AcqResult {
            sat,
            signal_band: bijux_gnss_core::api::SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Unknown,
            glonass_frequency_channel: None,
            source_time: bijux_gnss_core::api::ReceiverSampleTrace::default(),
            candidate_rank: 1,
            is_primary_candidate: true,
            doppler_hz: bijux_gnss_core::api::Hertz(-750.0),
            doppler_rate_hz_per_s: 0.0,
            carrier_hz: bijux_gnss_core::api::Hertz(-750.0),
            code_phase_samples,
            peak: 1.0,
            second_peak: 0.1,
            mean: 0.01,
            peak_mean_ratio: 20.0,
            peak_second_ratio: 10.0,
            cn0_proxy: 52.0,
            score: 1.0,
            hypothesis: bijux_gnss_core::api::AcqHypothesis::Accepted,
            assumptions: None,
            evidence: Vec::new(),
            threshold_provenance: None,
            explain_selection_reason: Some("clean_seeded_tracking".to_string()),
            doppler_refinement: None,
            code_phase_refinement: None,
            signal_delay_alignment: None,
            uncertainty: None,
        }],
    );
    let epochs = &tracks.first().expect("track").epochs;

    assert!(
        epochs
            .iter()
            .skip(4)
            .filter(|epoch| epoch.pll_lock && epoch.fll_lock)
            .all(|epoch| !epoch.anti_false_lock),
        "clean, converged tracking epochs must not remain marked as false lock: {epochs:?}"
    );
}

#[test]
fn correlate_epoch_honors_receiver_code_phase_seed_at_low_sample_rate() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 2_046_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 1,
        early_late_spacing_chips: 0.5,
        dll_bw_hz: 2.0,
        pll_bw_hz: 18.0,
        fll_bw_hz: 12.0,
        ..ReceiverPipelineConfig::default()
    };
    let sat = SatId { constellation: Constellation::Gps, prn: 3 };
    let frame = generate_l1_ca(
        &config,
        SyntheticSignalParams {
            sat,
            glonass_frequency_channel: None,
            signal_band: bijux_gnss_core::api::SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Unknown,
            doppler_hz: 750.0,
            code_phase_chips: 200.25,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 58.0,
            navigation_data: false.into(),
        },
        0x330C_2000,
        0.04,
    );
    let refined_code_phase_samples =
        crate::sim::synthetic::expected_acquisition_code_phase_samples_f64(&config, &frame, 200.25);
    let tracking = Tracking::new(config, ReceiverRuntime::default());
    let correlator = tracking.correlate_epoch(
        &frame,
        sat,
        750.0,
        0.0,
        1_023_000.0,
        refined_code_phase_samples,
        0.5,
    );

    assert!(
        correlator.prompt.norm() > 10_000.0,
        "receiver-seeded code phase must produce a strong prompt correlation: {:?}",
        correlator.prompt
    );
    assert!(
        correlator.prompt.re.abs() > correlator.prompt.im.abs() * 100.0,
        "receiver-seeded code phase must align carrier phase near the real axis: {:?}",
        correlator.prompt
    );
}

#[test]
fn tracking_recovery_from_loss_of_lock() {
    let lost = Tracking::transition_state(1, ChannelState::Tracking, ChannelState::Lost);
    assert_eq!(lost, ChannelState::Lost);
    let pull_in = Tracking::transition_state(1, lost, ChannelState::PullIn);
    assert_eq!(pull_in, ChannelState::PullIn);
    let tracking = Tracking::transition_state(1, pull_in, ChannelState::Tracking);
    assert_eq!(tracking, ChannelState::Tracking);
}

#[test]
fn tracking_reset_after_gap() {
    let lost = Tracking::transition_state(2, ChannelState::Tracking, ChannelState::Lost);
    let reset = Tracking::transition_state(2, lost, ChannelState::Idle);
    assert_eq!(reset, ChannelState::Idle);
}

#[test]
fn ambiguous_hypothesis_is_degraded_for_tracking() {
    assert_eq!(super::acq_to_track_state(&AcqHypothesis::Accepted), "accepted");
    assert_eq!(super::acq_to_track_state(&AcqHypothesis::Ambiguous), "degraded");
    assert_eq!(super::acq_to_track_state(&AcqHypothesis::Rejected), "rejected");
    assert_eq!(super::acq_to_track_state(&AcqHypothesis::Deferred), "deferred");
}

#[test]
fn deterministic_transition_rule_handles_cycle_slip_first() {
    let decision = super::deterministic_transition_rule(
        ChannelState::Tracking,
        false,
        false,
        false,
        Some(super::LossOfLockCause::PhaseJump),
        1,
        0,
        100,
        false,
        None,
    );
    assert_eq!(decision.to_state, ChannelState::Lost);
    assert_eq!(decision.reason, "phase_jump");
    assert_eq!(decision.next_unlocked_count, 2);
    assert_eq!(decision.next_degraded_epochs, 0);
}

#[test]
fn deterministic_transition_rule_promotes_lock() {
    let decision = super::deterministic_transition_rule(
        ChannelState::PullIn,
        true,
        true,
        false,
        None,
        2,
        0,
        100,
        false,
        None,
    );
    assert_eq!(decision.to_state, ChannelState::Tracking);
    assert_eq!(decision.reason, "carrier_converged");
    assert_eq!(decision.next_unlocked_count, 0);
    assert_eq!(decision.next_degraded_epochs, 0);
}

#[test]
fn deterministic_transition_rule_degrades_tracking_during_short_fade() {
    let decision = super::deterministic_transition_rule(
        ChannelState::Tracking,
        false,
        false,
        false,
        None,
        1,
        0,
        100,
        false,
        None,
    );
    assert_eq!(decision.to_state, ChannelState::Degraded);
    assert_eq!(decision.reason, "signal_fade");
    assert_eq!(decision.next_unlocked_count, 0);
    assert_eq!(decision.next_degraded_epochs, 1);
}

#[test]
fn deterministic_transition_rule_reports_doppler_estimator_divergence() {
    let decision = super::deterministic_transition_rule(
        ChannelState::Tracking,
        true,
        false,
        false,
        None,
        0,
        0,
        100,
        false,
        Some("doppler_estimator_divergence"),
    );

    assert_eq!(decision.to_state, ChannelState::Degraded);
    assert_eq!(decision.reason, "doppler_estimator_divergence");
    assert_eq!(decision.next_unlocked_count, 0);
    assert_eq!(decision.next_degraded_epochs, 1);
}

#[test]
fn deterministic_transition_rule_recovers_after_short_fade() {
    let decision = super::deterministic_transition_rule(
        ChannelState::Degraded,
        true,
        true,
        false,
        None,
        0,
        4,
        100,
        false,
        None,
    );
    assert_eq!(decision.to_state, ChannelState::Tracking);
    assert_eq!(decision.reason, "fade_recovered");
    assert_eq!(decision.next_unlocked_count, 0);
    assert_eq!(decision.next_degraded_epochs, 0);
}

#[test]
fn deterministic_transition_rule_preserves_doppler_reason_while_degraded() {
    let decision = super::deterministic_transition_rule(
        ChannelState::Degraded,
        true,
        false,
        false,
        None,
        0,
        4,
        100,
        false,
        Some("doppler_estimator_divergence"),
    );

    assert_eq!(decision.to_state, ChannelState::Degraded);
    assert_eq!(decision.reason, "doppler_estimator_divergence");
    assert_eq!(decision.next_unlocked_count, 0);
    assert_eq!(decision.next_degraded_epochs, 5);
}

#[test]
fn deterministic_transition_rule_keeps_degraded_state_during_fade_cycle_slip() {
    let decision = super::deterministic_transition_rule(
        ChannelState::Degraded,
        false,
        false,
        false,
        Some(super::LossOfLockCause::PhaseJump),
        0,
        2,
        100,
        false,
        None,
    );
    assert_eq!(decision.to_state, ChannelState::Lost);
    assert_eq!(decision.reason, "phase_jump");
    assert_eq!(decision.next_unlocked_count, 1);
    assert_eq!(decision.next_degraded_epochs, 0);
}

#[test]
fn deterministic_transition_rule_marks_loss_after_fade_budget_exhaustion() {
    let decision = super::deterministic_transition_rule(
        ChannelState::Degraded,
        false,
        false,
        false,
        Some(super::LossOfLockCause::PromptPowerDrop),
        0,
        100,
        100,
        false,
        None,
    );
    assert_eq!(decision.to_state, ChannelState::Lost);
    assert_eq!(decision.reason, "prompt_power_drop");
    assert_eq!(decision.next_unlocked_count, 1);
    assert_eq!(decision.next_degraded_epochs, 0);
}

#[test]
fn deterministic_transition_rule_grants_short_fade_grace_to_degraded_instability() {
    let decision = super::deterministic_transition_rule(
        ChannelState::Degraded,
        false,
        false,
        false,
        Some(super::LossOfLockCause::DiscriminatorInstability),
        0,
        super::DEGRADED_FADE_INSTABILITY_GRACE_EPOCHS,
        100,
        false,
        None,
    );
    assert_eq!(decision.to_state, ChannelState::Degraded);
    assert_eq!(decision.reason, "signal_fade");
    assert_eq!(decision.next_degraded_epochs, super::DEGRADED_FADE_INSTABILITY_GRACE_EPOCHS + 1);
}

#[test]
fn classify_loss_of_lock_cause_prioritizes_phase_jump() {
    let cause = super::classify_loss_of_lock_cause(ChannelState::Tracking, true, Some(0.9), 3);

    assert_eq!(cause, Some(super::LossOfLockCause::PhaseJump));
}

#[test]
fn classify_loss_of_lock_cause_treats_weak_prompt_cycle_slips_as_prompt_power_drop() {
    let cause = super::classify_loss_of_lock_cause(ChannelState::Tracking, true, Some(0.1), 0);

    assert_eq!(cause, Some(super::LossOfLockCause::PromptPowerDrop));
}

#[test]
fn classify_loss_of_lock_cause_detects_prompt_power_drop() {
    let cause = super::classify_loss_of_lock_cause(ChannelState::Tracking, false, Some(0.1), 0);

    assert_eq!(cause, Some(super::LossOfLockCause::PromptPowerDrop));
}

#[test]
fn classify_loss_of_lock_cause_detects_discriminator_instability() {
    let cause = super::classify_loss_of_lock_cause(ChannelState::Tracking, false, Some(0.8), 2);

    assert_eq!(cause, Some(super::LossOfLockCause::DiscriminatorInstability));
}

#[test]
fn update_discriminator_instability_epochs_requires_strong_prompt() {
    let epochs = super::update_discriminator_instability_epochs(
        1,
        ChannelState::Tracking,
        Some(0.7),
        false,
        false,
        false,
        false,
    );
    assert_eq!(epochs, 2);

    let reset = super::update_discriminator_instability_epochs(
        epochs,
        ChannelState::Tracking,
        Some(0.1),
        false,
        false,
        false,
        false,
    );
    assert_eq!(reset, 0);
}

#[test]
fn sustained_lock_loss_reacquire_seed_ignores_degraded_short_fade_epochs() {
    let epochs = vec![
        track_epoch_with_state(0, false, "degraded", Some("signal_fade")),
        track_epoch_with_state(1, false, "degraded", Some("signal_fade")),
        track_epoch_with_state(2, false, "degraded", Some("signal_fade")),
    ];

    assert_eq!(super::sustained_lock_loss_reacquire_seed(&epochs), None);
}

#[test]
fn sustained_lock_loss_reacquire_seed_requires_three_lost_epochs() {
    let epochs = vec![
        track_epoch_with_state(0, false, "lost", Some("lock_lost")),
        track_epoch_with_state(1, false, "lost", Some("lock_lost")),
        track_epoch_with_state(2, false, "lost", Some("lock_lost")),
    ];

    assert_eq!(
        super::sustained_lock_loss_reacquire_seed(&epochs),
        Some(super::SustainedLockLossSeed {
            carrier_hz: 2.0,
            code_phase_samples: 2.5,
            code_rate_hz: 0.0,
            sample_index: 2,
        })
    );
}

#[test]
fn sustained_lock_loss_reacquire_seed_accepts_explicit_loss_causes() {
    let epochs = vec![
        track_epoch_with_state(0, false, "lost", Some("prompt_power_drop")),
        track_epoch_with_state(1, false, "lost", Some("prompt_power_drop")),
        track_epoch_with_state(2, false, "lost", Some("prompt_power_drop")),
    ];

    assert_eq!(
        super::sustained_lock_loss_reacquire_seed(&epochs),
        Some(super::SustainedLockLossSeed {
            carrier_hz: 2.0,
            code_phase_samples: 2.5,
            code_rate_hz: 0.0,
            sample_index: 2,
        })
    );
}

#[test]
fn sustained_lock_loss_reacquire_seed_allows_retry_after_failed_attempt() {
    let epochs = vec![
        track_epoch_with_state(0, false, "lost", Some("reacquisition_failed")),
        track_epoch_with_state(1, false, "lost", Some("reacquisition_failed")),
        track_epoch_with_state(2, false, "lost", Some("reacquisition_failed")),
    ];

    assert_eq!(
        super::sustained_lock_loss_reacquire_seed(&epochs),
        Some(super::SustainedLockLossSeed {
            carrier_hz: 2.0,
            code_phase_samples: 2.5,
            code_rate_hz: 0.0,
            sample_index: 2,
        })
    );
}

#[test]
fn project_reacquisition_code_phase_samples_advances_loss_anchor_to_current_epoch() {
    let seed = super::SustainedLockLossSeed {
        carrier_hz: 1200.0,
        code_phase_samples: 5.5,
        code_rate_hz: 1_023_000.0,
        sample_index: 1_000,
    };

    let projected = super::project_reacquisition_code_phase_samples(
        seed,
        1_000 + 2 * 1_023,
        1_023_000.0,
        1_023,
    );

    assert_eq!(projected, 5.5);
}

fn reacquisition_candidate(
    doppler_bin: i8,
    code_bin: i8,
    carrier_sign: super::ReacquisitionCarrierSign,
    secondary_code_phase_periods: Option<usize>,
    cn0_dbhz: f64,
    prompt_power: f32,
) -> super::ReacquisitionCandidate {
    super::ReacquisitionCandidate {
        hypothesis: super::ReacquisitionHypothesis {
            doppler_bin,
            code_bin,
            carrier_sign,
            secondary_code_phase_periods,
        },
        carrier_hz: 1000.0 + f64::from(doppler_bin),
        code_phase_samples: 42.0 + f64::from(code_bin),
        cn0_dbhz,
        prompt_power,
    }
}

#[test]
fn reacquisition_selection_chooses_strongest_hypothesis_not_first_peak() {
    let selection = super::select_reacquisition_candidate(
        &[
            reacquisition_candidate(
                -2,
                -2,
                super::ReacquisitionCarrierSign::Aligned,
                None,
                39.0,
                200.0,
            ),
            reacquisition_candidate(
                1,
                1,
                super::ReacquisitionCarrierSign::Aligned,
                None,
                44.0,
                300.0,
            ),
        ],
        35.0,
        0.0,
    );

    assert_eq!(
        selection,
        super::ReacquisitionSelection::Accepted(super::ReacquisitionSeed {
            carrier_hz: 1001.0,
            code_phase_samples: 43.0,
            cn0_dbhz: 44.0,
            carrier_sign: super::ReacquisitionCarrierSign::Aligned,
            secondary_code_phase_periods: None,
        })
    );
}

#[test]
fn reacquisition_selection_refuses_ambiguous_hypotheses() {
    let selection = super::select_reacquisition_candidate(
        &[
            reacquisition_candidate(
                0,
                0,
                super::ReacquisitionCarrierSign::Aligned,
                None,
                44.0,
                300.0,
            ),
            reacquisition_candidate(
                2,
                2,
                super::ReacquisitionCarrierSign::Inverted,
                None,
                43.0,
                292.0,
            ),
        ],
        35.0,
        0.0,
    );

    assert_eq!(selection, super::ReacquisitionSelection::Refused);
}

#[test]
fn reacquisition_selection_refuses_competing_secondary_code_phases() {
    let selection = super::select_reacquisition_candidate(
        &[
            reacquisition_candidate(
                0,
                0,
                super::ReacquisitionCarrierSign::Aligned,
                Some(3),
                44.0,
                300.0,
            ),
            reacquisition_candidate(
                0,
                0,
                super::ReacquisitionCarrierSign::Aligned,
                Some(4),
                43.5,
                294.0,
            ),
        ],
        35.0,
        0.0,
    );

    assert_eq!(selection, super::ReacquisitionSelection::Refused);
}

#[test]
fn reacquisition_selection_treats_carrier_sign_only_tie_as_same_location() {
    let selection = super::select_reacquisition_candidate(
        &[
            reacquisition_candidate(
                0,
                0,
                super::ReacquisitionCarrierSign::Aligned,
                None,
                44.0,
                300.0,
            ),
            reacquisition_candidate(
                0,
                0,
                super::ReacquisitionCarrierSign::Inverted,
                None,
                44.0,
                300.0,
            ),
        ],
        35.0,
        0.0,
    );

    assert_eq!(
        selection,
        super::ReacquisitionSelection::Accepted(super::ReacquisitionSeed {
            carrier_hz: 1000.0,
            code_phase_samples: 42.0,
            cn0_dbhz: 44.0,
            carrier_sign: super::ReacquisitionCarrierSign::Aligned,
            secondary_code_phase_periods: None,
        })
    );
}

#[test]
fn reacquisition_selection_preserves_aligned_sign_for_same_location() {
    let selection = super::select_reacquisition_candidate(
        &[
            reacquisition_candidate(
                0,
                0,
                super::ReacquisitionCarrierSign::Aligned,
                None,
                44.0,
                300.0,
            ),
            reacquisition_candidate(
                0,
                0,
                super::ReacquisitionCarrierSign::Inverted,
                None,
                44.1,
                301.0,
            ),
        ],
        35.0,
        0.0,
    );

    assert_eq!(
        selection,
        super::ReacquisitionSelection::Accepted(super::ReacquisitionSeed {
            carrier_hz: 1000.0,
            code_phase_samples: 42.0,
            cn0_dbhz: 44.0,
            carrier_sign: super::ReacquisitionCarrierSign::Aligned,
            secondary_code_phase_periods: None,
        })
    );
}

#[test]
fn reacquisition_selection_accepts_strong_prompt_when_cn0_is_conservative() {
    let selection = super::select_reacquisition_candidate(
        &[reacquisition_candidate(
            0,
            0,
            super::ReacquisitionCarrierSign::Aligned,
            None,
            45.0,
            300_000.0,
        )],
        48.0,
        100_000.0,
    );

    assert_eq!(
        selection,
        super::ReacquisitionSelection::Accepted(super::ReacquisitionSeed {
            carrier_hz: 1000.0,
            code_phase_samples: 42.0,
            cn0_dbhz: 45.0,
            carrier_sign: super::ReacquisitionCarrierSign::Aligned,
            secondary_code_phase_periods: None,
        })
    );
}

#[test]
fn reacquisition_selection_rejects_strong_prompt_when_cn0_is_too_low() {
    let selection = super::select_reacquisition_candidate(
        &[reacquisition_candidate(
            0,
            0,
            super::ReacquisitionCarrierSign::Aligned,
            None,
            41.0,
            300_000.0,
        )],
        48.0,
        100_000.0,
    );

    assert_eq!(selection, super::ReacquisitionSelection::Refused);
}

#[test]
fn quick_reacquire_recovers_offset_code_hypothesis() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 1_023_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let sat = SatId { constellation: Constellation::Gps, prn: 21 };
    let frame = generate_l1_ca(
        &config,
        SyntheticSignalParams {
            sat,
            glonass_frequency_channel: None,
            signal_band: SignalBand::L1,
            signal_code: SignalCode::Ca,
            doppler_hz: 0.0,
            code_phase_chips: 0.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 70.0,
            navigation_data: false.into(),
        },
        0x51AC_0301,
        0.001,
    );
    let tracking = Tracking::new(config.clone(), ReceiverRuntime::default());

    let seed = tracking
        .quick_reacquire(&frame, sat, 0.0, 2.0, 35.0, 0.0, None)
        .expect("reacquisition seed");
    let samples_per_code =
        samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length);
    let code_error =
        wrapped_code_phase_delta_samples(seed.code_phase_samples, 0.0, samples_per_code).abs();

    assert!(
            code_error <= 0.1,
            "reacquisition should recover the actual code hypothesis: seed={seed:?}, code_error={code_error}"
        );
    assert!((seed.carrier_hz - 0.0).abs() <= f64::EPSILON, "{seed:?}");
}

#[test]
fn reacquisition_secondary_code_phases_collapse_when_signal_has_no_secondary_code() {
    let config = ReceiverPipelineConfig::default();
    let signal_model = super::TrackingSignalModel::for_sat_signal_band(
        &config,
        SatId { constellation: Constellation::Gps, prn: 7 },
        SignalBand::L1,
        SignalCode::Ca,
        None,
    );

    assert_eq!(super::reacquisition_secondary_code_phase_periods(&signal_model), vec![None]);
}

#[test]
fn reacquisition_secondary_code_phases_cover_supported_period() {
    let config = ReceiverPipelineConfig::default();
    let signal_model = super::TrackingSignalModel::for_sat_signal_band(
        &config,
        SatId { constellation: Constellation::Gps, prn: 18 },
        SignalBand::L5,
        SignalCode::L5I,
        None,
    );
    let phases = super::reacquisition_secondary_code_phase_periods(&signal_model);

    assert!(phases.len() > 1);
    assert_eq!(phases.first(), Some(&Some(0)));
    assert_eq!(phases.last(), Some(&Some(phases.len() - 1)));
}

#[test]
fn reacquisition_seed_matches_respects_acquisition_uncertainty_tolerances() {
    let tracking = Tracking::new(ReceiverPipelineConfig::default(), ReceiverRuntime::default());
    let uncertainty = AcqUncertainty {
        doppler_hz: 250.0,
        code_phase_samples: 0.75,
        doppler_rate_hz_per_s: None,
        covariance: None,
    };

    assert!(tracking.reacquisition_seed_matches(
        super::ReacquisitionSeed {
            carrier_hz: 1250.0,
            code_phase_samples: 42.0,
            cn0_dbhz: 36.0,
            carrier_sign: super::ReacquisitionCarrierSign::Aligned,
            secondary_code_phase_periods: None,
        },
        super::ReacquisitionSeed {
            carrier_hz: 1400.0,
            code_phase_samples: 42.5,
            cn0_dbhz: 34.0,
            carrier_sign: super::ReacquisitionCarrierSign::Aligned,
            secondary_code_phase_periods: None,
        },
        Some(&uncertainty),
    ));
    assert!(!tracking.reacquisition_seed_matches(
        super::ReacquisitionSeed {
            carrier_hz: 1250.0,
            code_phase_samples: 42.0,
            cn0_dbhz: 36.0,
            carrier_sign: super::ReacquisitionCarrierSign::Aligned,
            secondary_code_phase_periods: None,
        },
        super::ReacquisitionSeed {
            carrier_hz: 1705.0,
            code_phase_samples: 43.0,
            cn0_dbhz: 34.0,
            carrier_sign: super::ReacquisitionCarrierSign::Aligned,
            secondary_code_phase_periods: None,
        },
        Some(&uncertainty),
    ));
}

#[test]
fn reacquisition_min_cn0_dbhz_preserves_lock_reference_headroom() {
    assert_eq!(super::reacquisition_min_cn0_dbhz(60.0), 48.0);
    assert_eq!(super::reacquisition_min_cn0_dbhz(35.0), 28.0);
    assert_eq!(super::reacquisition_min_cn0_dbhz(f64::NAN), super::TRACKING_LOCK_MIN_CN0_DBHZ);
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
fn doppler_estimator_consistency_accepts_aligned_residuals() {
    let consistency = super::doppler_estimator_consistency(0.0, 4.0, -3.0, 8.0);

    assert!(consistency.consistent, "{consistency:?}");
    assert_eq!(consistency.spread_hz, 7.0);
    assert_eq!(consistency.limit_hz, super::DOPPLER_ESTIMATOR_MIN_SPREAD_LIMIT_HZ);
}

#[test]
fn doppler_estimator_consistency_rejects_divergent_residuals() {
    let consistency = super::doppler_estimator_consistency(0.0, 12.0, 95.0, 8.0);

    assert!(!consistency.consistent, "{consistency:?}");
    assert_eq!(consistency.spread_hz, 95.0);
}

#[test]
fn doppler_estimator_consistency_rejects_non_finite_residuals() {
    let consistency = super::doppler_estimator_consistency(0.0, f64::NAN, 2.0, 8.0);

    assert!(!consistency.consistent, "{consistency:?}");
    assert!(consistency.spread_hz.is_infinite());
}

#[test]
fn doppler_estimator_uncertainty_sample_carries_estimator_spread() {
    let consistency = super::doppler_estimator_consistency(0.0, -10.0, 88.0, 8.0);
    let sample = super::doppler_estimator_uncertainty_sample_hz(consistency, 4.0);

    assert_eq!(sample, consistency.spread_hz);
}

#[test]
fn doppler_estimator_provenance_reports_divergence() {
    let consistency = super::doppler_estimator_consistency(0.0, 12.0, 95.0, 8.0);
    let provenance = super::doppler_estimator_provenance(consistency);

    assert!(provenance.contains("doppler_estimator_consistency=divergent"));
    assert!(provenance.contains("doppler_estimator_spread_hz=95.000"));
}

#[test]
fn tracking_provenance_segment_preserves_doppler_estimator_evidence() {
    let provenance = "tracking lock_detector_fll_hz=10.000 doppler_estimator_consistency=divergent doppler_estimator_spread_hz=95.000 doppler_estimator_limit_hz=25.000 doppler_loop_residual_hz=0.000 doppler_phase_rate_residual_hz=12.000 doppler_prompt_residual_hz=95.000 unrelated=true";
    let segment = super::tracking_provenance_segment(
        provenance,
        "doppler_estimator_consistency=",
        super::DOPPLER_ESTIMATOR_PROVENANCE_TOKEN_COUNT,
    )
    .expect("doppler estimator segment");

    assert_eq!(
            segment,
            "doppler_estimator_consistency=divergent doppler_estimator_spread_hz=95.000 doppler_estimator_limit_hz=25.000 doppler_loop_residual_hz=0.000 doppler_phase_rate_residual_hz=12.000 doppler_prompt_residual_hz=95.000"
        );
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
fn classify_prompt_phase_recovers_continuity_across_nav_bit_flip() {
    let decision = super::classify_prompt_phase(
        -0.39,
        Some(0.11),
        0.0,
        super::TrackingPhaseTransitionSource::DataSymbol,
    );

    assert!(decision.nav_bit_transition);
    assert!(!decision.cycle_slip);
    assert!((decision.aligned_phase_cycles - 0.11).abs() <= 0.02);
    assert!(decision.aligned_phase_delta_cycles.abs() <= 0.02);
    assert!((decision.nav_bit_phase_offset_cycles - 0.5).abs() <= f64::EPSILON);
}

#[test]
fn classify_prompt_phase_preserves_cycle_slip_for_non_nav_jump() {
    let decision = super::classify_prompt_phase(
        0.36,
        Some(0.0),
        0.0,
        super::TrackingPhaseTransitionSource::DataSymbol,
    );

    assert!(!decision.nav_bit_transition);
    assert!(decision.cycle_slip);
    assert!((decision.aligned_phase_delta_cycles - 0.36).abs() <= 1.0e-9);
    assert!((decision.nav_bit_phase_offset_cycles - 0.0).abs() <= f64::EPSILON);
}

#[test]
fn classify_prompt_phase_keeps_half_cycle_jump_as_slip_without_transition_metadata() {
    let decision = super::classify_prompt_phase(
        -0.39,
        Some(0.11),
        0.0,
        super::TrackingPhaseTransitionSource::None,
    );

    assert!(!decision.nav_bit_transition);
    assert!(decision.cycle_slip);
    assert!(decision.aligned_phase_delta_cycles.abs() > 0.35);
}

#[test]
fn secondary_code_phase_transition_waits_for_accepted_sync() {
    let config = ReceiverPipelineConfig::default();
    let signal_model = super::TrackingSignalModel::for_sat_signal_band(
        &config,
        SatId { constellation: Constellation::Gps, prn: 18 },
        SignalBand::L5,
        SignalCode::L5I,
        None,
    );

    let transition_source = super::carrier_phase_transition_source_for_prompt(&signal_model, None);
    let decision = super::classify_prompt_phase(-0.39, Some(0.11), 0.0, transition_source);

    assert_eq!(transition_source, super::TrackingPhaseTransitionSource::None);
    assert!(!decision.nav_bit_transition);
    assert!(decision.cycle_slip);
}

#[test]
fn secondary_code_phase_transition_accepts_synchronized_half_cycle_jump() {
    let config = ReceiverPipelineConfig::default();
    let signal_model = super::TrackingSignalModel::for_sat_signal_band(
        &config,
        SatId { constellation: Constellation::Gps, prn: 18 },
        SignalBand::L5,
        SignalCode::L5I,
        None,
    );
    let sync = super::SecondaryCodeSyncResult {
        phase_periods: 7,
        confidence: 1.0,
        best_likelihood: 1.0,
        next_best_likelihood: 0.0,
        observed_periods: 20,
        accepted: true,
    };

    let transition_source =
        super::carrier_phase_transition_source_for_prompt(&signal_model, Some(sync));
    let decision = super::classify_prompt_phase(-0.39, Some(0.11), 0.0, transition_source);

    assert_eq!(transition_source, super::TrackingPhaseTransitionSource::SecondaryCode);
    assert!(decision.nav_bit_transition);
    assert!(!decision.cycle_slip);
}

#[test]
fn secondary_code_phase_transitions_preserve_continuity_across_multiple_flips() {
    let config = ReceiverPipelineConfig::default();
    let signal_model = super::TrackingSignalModel::for_sat_signal_band(
        &config,
        SatId { constellation: Constellation::Gps, prn: 18 },
        SignalBand::L5,
        SignalCode::L5I,
        None,
    );
    let sync = super::SecondaryCodeSyncResult {
        phase_periods: 7,
        confidence: 1.0,
        best_likelihood: 1.0,
        next_best_likelihood: 0.0,
        observed_periods: 20,
        accepted: true,
    };
    let transition_source =
        super::carrier_phase_transition_source_for_prompt(&signal_model, Some(sync));
    let raw_phase_cycles = [0.10, -0.39, -0.38, 0.13, 0.14];
    let expected_transitions = [false, true, false, true, false];
    let mut previous_aligned_phase_cycles = None;
    let mut secondary_code_phase_offset_cycles = 0.0;

    for (raw_phase_cycles, expected_transition) in
        raw_phase_cycles.into_iter().zip(expected_transitions)
    {
        let decision = super::classify_prompt_phase(
            raw_phase_cycles,
            previous_aligned_phase_cycles,
            secondary_code_phase_offset_cycles,
            transition_source,
        );

        assert_eq!(decision.nav_bit_transition, expected_transition, "{decision:?}");
        assert!(!decision.cycle_slip, "{decision:?}");
        if previous_aligned_phase_cycles.is_some() {
            assert!(decision.aligned_phase_delta_cycles.abs() <= 0.03, "{decision:?}");
        }
        previous_aligned_phase_cycles = Some(decision.aligned_phase_cycles);
        secondary_code_phase_offset_cycles = decision.nav_bit_phase_offset_cycles;
    }
}

#[test]
fn classify_prompt_phase_handles_real_synthetic_nav_bit_transitions() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 12,
        ..ReceiverPipelineConfig::default()
    };
    let sat = SatId { constellation: bijux_gnss_core::api::Constellation::Gps, prn: 7 };
    let code_phase_chips = 321.0;
    let samples_per_epoch =
        samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length);
    let frame = generate_l1_ca(
        &config,
        SyntheticSignalParams {
            sat,
            glonass_frequency_channel: None,
            signal_band: bijux_gnss_core::api::SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Unknown,
            doppler_hz: 0.0,
            code_phase_chips,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 70.0,
            navigation_data: true.into(),
        },
        0x7A91B17,
        0.05,
    );
    let tracking = Tracking::new(config.clone(), ReceiverRuntime::default());
    let mut previous_aligned_phase_cycles = None;
    let mut nav_bit_phase_offset_cycles = 0.0;
    let mut transition_epochs = Vec::new();

    for (epoch_index, epoch_samples) in frame.iq.chunks_exact(samples_per_epoch).enumerate() {
        let sample_index = epoch_index as u64 * samples_per_epoch as u64;
        let epoch_frame = SamplesFrame::new(
            SampleTime { sample_index, sample_rate_hz: config.sampling_freq_hz },
            Seconds(1.0 / config.sampling_freq_hz),
            epoch_samples.to_vec(),
        );
        let epoch_code_phase_chips = advance_code_phase_seconds(
            code_phase_chips,
            config.code_freq_basis_hz,
            sample_index as f64 / config.sampling_freq_hz,
            config.code_length,
        )
        .expect("valid epoch code phase");
        let epoch_code_phase_samples =
            crate::sim::synthetic::expected_acquisition_code_phase_samples_f64(
                &config,
                &epoch_frame,
                epoch_code_phase_chips,
            );
        let (epoch, _) = tracking.track_epoch(
            &epoch_frame,
            0,
            sat,
            0.0,
            0.0,
            config.code_freq_basis_hz,
            epoch_code_phase_samples,
            0.5,
        );
        let raw_phase_cycles =
            (epoch.prompt_q as f64).atan2(epoch.prompt_i as f64) / (2.0 * std::f64::consts::PI);
        let decision = super::classify_prompt_phase(
            raw_phase_cycles,
            previous_aligned_phase_cycles,
            nav_bit_phase_offset_cycles,
            super::TrackingPhaseTransitionSource::DataSymbol,
        );
        if decision.nav_bit_transition {
            transition_epochs.push(epoch_index);
        }
        assert!(
            !decision.cycle_slip,
            "unexpected cycle slip at epoch {epoch_index}: raw_phase_cycles={raw_phase_cycles}"
        );
        previous_aligned_phase_cycles = Some(decision.aligned_phase_cycles);
        nav_bit_phase_offset_cycles = decision.nav_bit_phase_offset_cycles;
    }

    assert_eq!(transition_epochs, vec![20, 40]);
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
