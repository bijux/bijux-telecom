use super::{
    anti_false_lock_detected, apply_carrier_tracking_loop, apply_code_loop,
    calibrated_lock_detector_thresholds, carrier_frequency_error_hz_from_phase_delta,
    carrier_phase_offset_radians, coherent_integration_seconds, correlate_early_prompt_late,
    delay_lock_loop_coefficients, discriminators, dll_discriminator_from_early_late,
    dll_lock_threshold, double_delta_dll_discriminator, estimate_cn0_dbhz,
    estimate_tracking_uncertainty, first_order_angular_loop_coefficients,
    first_order_loop_coefficients, lock_detector_distributions, lock_detector_probability_summary,
    normalize_dll_discriminator, phase_lock_loop_coefficients, predict_code_phase_samples,
    wrap_phase_cycles_signed, wrap_phase_radians_positive, wrapped_phase_delta_cycles,
    CarrierTrackingLoopInput, CodeLoopInput, EarlyPromptLateCorrelatorInput,
    LockDetectorCalibrationInput, LockDetectorProbabilityInput, TrackingQualityClass,
    TrackingUncertaintyInputs,
};
use crate::dsp::tracking_adaptation::{
    advance_tracking_adaptation, TrackingAdaptationInput, TrackingAdaptationState,
    TrackingLoopProfile, TrackingLoopProfileKind, DYNAMIC_STRESS_INTEGRATION_MS,
    MAX_TRACKING_ADAPTATION_PENDING_EPOCHS, WEAK_SIGNAL_INTEGRATION_MS,
};
use std::collections::VecDeque;

use num_complex::Complex;

#[path = "tests/adaptation.rs"]
mod adaptation;
#[path = "tests/discriminator_guards.rs"]
mod discriminator_guards;
#[path = "tests/lock_detection.rs"]
mod lock_detection;
#[path = "tests/loop_filter_design.rs"]
mod loop_filter_design;
#[path = "tests/loop_response.rs"]
mod loop_response;
#[path = "tests/signal_measurements.rs"]
mod signal_measurements;
#[path = "tests/tracking_uncertainty.rs"]
mod tracking_uncertainty;
