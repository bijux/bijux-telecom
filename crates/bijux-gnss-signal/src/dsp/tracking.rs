//! Tracking math utilities.

use std::collections::VecDeque;

use crate::dsp::nco::Nco;
use crate::dsp::signal::{
    code_value_at_phase, epoch_start_code_phase_samples_from_receiver_phase,
    receiver_code_phase_samples_from_epoch_start_phase, wrap_code_phase_samples,
};
use bijux_gnss_core::api::TrackingUncertainty;
use num_complex::Complex;

const DLL_LOCK_MAX_CODE_ERROR: f32 = 0.2;
const DLL_HOLD_MAX_CODE_ERROR: f32 = 0.4;
const DLL_LOW_RESOLUTION_LOCK_MAX_CODE_ERROR: f32 = 0.6;
const PLL_LOCK_MIN_PHASE_ERROR_RAD: f32 = 0.35;
const DLL_REFERENCE_EARLY_LATE_SPACING_CHIPS: f64 = 0.5;
const DOUBLE_DELTA_OUTER_WEIGHT: f32 = 0.5;
const ANTI_FALSE_LOCK_MAX_EARLY_LATE_TO_PROMPT_RATIO: f32 = 0.95;
const FLL_PULL_IN_MAX_CORRECTION_BW_MULTIPLIER: f64 = 4.0;
const TRACKING_UNCERTAINTY_MIN_CODE_PHASE_SAMPLES: f64 = 0.01;
const TRACKING_UNCERTAINTY_MIN_CARRIER_PHASE_CYCLES: f64 = 0.001;
const TRACKING_UNCERTAINTY_MIN_DOPPLER_HZ: f64 = 0.01;
const TRACKING_UNCERTAINTY_MIN_CN0_DBHZ: f64 = 0.05;
const LOOP_FILTER_BANDWIDTH_SOLVER_ITERATIONS: usize = 80;
const LOOP_FILTER_IMPULSE_RESPONSE_EPOCHS: usize = 16_384;
const LOOP_FILTER_IMPULSE_RESPONSE_EPSILON: f64 = 1.0e-15;
const WEAK_SIGNAL_ENTRY_CN0_DBHZ: f64 = 31.0;
const WEAK_SIGNAL_EXIT_CN0_DBHZ: f64 = 34.0;
const DYNAMIC_STRESS_ENTRY_FLL_ERROR_HZ: f64 = 12.0;
const DYNAMIC_STRESS_EXIT_FLL_ERROR_HZ: f64 = 6.0;
const DYNAMIC_STRESS_ENTRY_CARRIER_RATE_HZ_PER_S: f64 = 200.0;
const DYNAMIC_STRESS_EXIT_CARRIER_RATE_HZ_PER_S: f64 = 50.0;
const WEAK_SIGNAL_INTEGRATION_MS: u32 = 5;
const DYNAMIC_STRESS_INTEGRATION_MS: u32 = 1;
const MAX_TRACKING_ADAPTATION_PENDING_EPOCHS: u8 = 4;

/// First-order loop coefficients derived from noise bandwidth and coherent integration time.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FirstOrderLoopCoefficients {
    /// Dimensionless fraction of the discriminator error to blend into the state each update.
    pub error_blend: f64,
    /// Effective gain in Hz applied to one unit of discriminator error.
    pub rate_gain_hz: f64,
}

/// Second-order DLL coefficients derived from noise bandwidth and coherent integration time.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DelayLockLoopCoefficients {
    /// Dimensionless fraction of the code-phase error to blend into the prompt alignment.
    pub phase_blend: f64,
    /// Effective gain in Hz applied to one chip of DLL error.
    pub rate_gain_hz_per_chip: f64,
}

/// Phase-lock loop coefficients derived from noise bandwidth and coherent integration time.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PhaseLockLoopCoefficients {
    /// Dimensionless phase correction applied to the phase accumulator.
    pub phase_blend: f64,
    /// Frequency correction gain in Hz per radian of phase error.
    pub frequency_gain_hz_per_rad: f64,
    /// Frequency-rate correction gain in Hz/s per radian of phase error.
    pub frequency_rate_gain_hz_per_s_per_rad: f64,
}

/// Early/prompt/late correlation accumulators for one coherent interval.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EarlyPromptLateCorrelation {
    /// Early replica accumulation.
    pub early: Complex<f32>,
    /// Prompt replica accumulation.
    pub prompt: Complex<f32>,
    /// Late replica accumulation.
    pub late: Complex<f32>,
    /// Energy of the early-minus-late weighting used by CN0 estimation.
    pub early_late_noise_weight_energy: f64,
}

/// Inputs for one early/prompt/late correlation interval.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EarlyPromptLateCorrelatorInput<'a> {
    /// Complex baseband samples for the coherent interval.
    pub samples: &'a [Complex<f32>],
    /// Receiver sample rate in Hz.
    pub sample_rate_hz: f64,
    /// Carrier frequency in Hz used for wipeoff.
    pub carrier_hz: f64,
    /// Carrier phase offset in radians at the interval start.
    pub carrier_phase_offset_radians: f64,
    /// Prompt replica chip phase at the interval start.
    pub base_chip_phase: f64,
    /// Replica chip advance per sample.
    pub chips_per_sample: f64,
    /// Early/late spacing in chips.
    pub early_late_spacing_chips: f64,
}

/// Pure DLL update inputs for one coherent tracking interval.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CodeLoopInput {
    /// Current code rate estimate in Hz.
    pub current_code_rate_hz: f64,
    /// Previous code-rate reference in Hz.
    pub previous_reference_code_rate_hz: f64,
    /// Current code-rate reference in Hz.
    pub reference_code_rate_hz: f64,
    /// Current receiver code phase in samples.
    pub current_code_phase_samples: f64,
    /// Coherent interval length in samples.
    pub epoch_len_samples: usize,
    /// Coherent interval length in seconds.
    pub coherent_integration_s: f64,
    /// Nominal code rate for the tracked signal in Hz.
    pub nominal_code_rate_hz: f64,
    /// Effective DLL bandwidth in Hz.
    pub dll_bw_hz: f64,
    /// DLL discriminator output.
    pub dll_err: f32,
    /// Samples per code chip at the receiver rate.
    pub samples_per_chip: f64,
    /// Samples per code period at the receiver rate.
    pub samples_per_code: usize,
}

/// DLL update outputs for one coherent interval.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CodeLoopUpdate {
    /// Updated code rate estimate in Hz.
    pub code_rate_hz: f64,
    /// Updated receiver code phase in samples.
    pub code_phase_samples: f64,
}

/// Combined carrier PLL/FLL update inputs for one coherent interval.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CarrierTrackingLoopInput {
    /// Current carrier frequency estimate in Hz.
    pub current_carrier_hz: f64,
    /// Current unwrapped carrier phase estimate in cycles.
    pub current_carrier_phase_cycles: f64,
    /// Current carrier frequency-rate estimate in Hz/s.
    pub current_carrier_rate_hz_per_s: f64,
    /// Coherent interval length in samples.
    pub epoch_len_samples: usize,
    /// Receiver sample rate in Hz.
    pub sample_rate_hz: f64,
    /// Coherent interval length in seconds.
    pub coherent_integration_s: f64,
    /// Effective PLL bandwidth in Hz.
    pub pll_bw_hz: f64,
    /// PLL discriminator output in radians.
    pub pll_err_rad: f64,
    /// Effective FLL bandwidth in Hz.
    pub fll_bw_hz: f64,
    /// FLL discriminator output in Hz.
    pub fll_err_hz: f64,
    /// Whether to apply the FLL contribution this interval.
    pub apply_fll: bool,
    /// Whether to apply PLL frequency feedback this interval.
    pub apply_pll_frequency: bool,
}

/// Combined carrier PLL/FLL update outputs for one coherent interval.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CarrierTrackingLoopUpdate {
    /// Updated carrier frequency estimate in Hz.
    pub carrier_hz: f64,
    /// Updated unwrapped carrier phase estimate in cycles.
    pub carrier_phase_cycles: f64,
    /// Updated carrier frequency-rate estimate in Hz/s.
    pub carrier_rate_hz_per_s: f64,
}

/// Coarse tracking quality class used to scale uncertainty estimates.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TrackingQualityClass {
    /// Stable steady-state tracking.
    Tracking,
    /// Tracking is retained but degraded.
    Degraded,
    /// Pull-in or acquisition-adjacent tracking.
    PullIn,
    /// Tracking has been lost.
    Lost,
}

/// Inputs for tracking uncertainty estimation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TrackingUncertaintyInputs {
    /// Samples per chip at the receiver rate.
    pub samples_per_chip: f64,
    /// DLL discriminator output.
    pub dll_err: f32,
    /// PLL discriminator output in radians.
    pub pll_err_rad: f64,
    /// FLL discriminator output in Hz.
    pub fll_err_hz: f64,
    /// Current CN0 estimate.
    pub cn0_dbhz: f64,
    /// Reference CN0 estimate.
    pub cn0_reference_dbhz: f64,
    /// Coherent integration in milliseconds.
    pub integration_ms: u32,
    /// Whether the channel remains logically locked.
    pub channel_locked: bool,
    /// Whether the DLL remains locked.
    pub dll_locked: bool,
    /// Whether anti-false-lock diagnostics are active.
    pub anti_false_lock: bool,
    /// Whether a cycle slip was detected this interval.
    pub cycle_slip: bool,
    /// Current coarse tracking quality class.
    pub quality_class: TrackingQualityClass,
}

/// Inputs used to derive tracking lock-detector thresholds from discriminator statistics.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LockDetectorCalibrationInput {
    /// Estimated carrier-to-noise density for the coherent interval.
    pub cn0_dbhz: f64,
    /// Coherent integration duration in seconds.
    pub coherent_integration_s: f64,
    /// Samples per code chip at the receiver rate.
    pub samples_per_chip: f64,
    /// Early/late correlator spacing in chips.
    pub early_late_spacing_chips: f64,
    /// Target probability that a truly locked DLL epoch is rejected by discriminator noise.
    pub dll_false_unlock_probability: f64,
    /// Target probability that a truly locked PLL epoch is rejected by discriminator noise.
    pub pll_false_unlock_probability: f64,
    /// Target probability that a truly locked FLL epoch is rejected by discriminator noise.
    pub fll_false_unlock_probability: f64,
    /// Effective FLL noise bandwidth in Hz for practical hold-floor calibration.
    pub fll_bw_hz: f64,
    /// Extra carrier-frequency stress in Hz from known dynamics or oscillator motion.
    pub dynamic_stress_hz: f64,
}

/// Estimated one-sigma discriminator spreads used by the lock detectors.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LockDetectorDistributions {
    /// Coherent carrier-to-noise ratio for one integration interval.
    pub coherent_snr_linear: f64,
    /// One-sigma normalized DLL discriminator spread.
    pub dll_sigma: f64,
    /// One-sigma PLL phase spread in radians.
    pub pll_sigma_rad: f64,
    /// One-sigma FLL frequency spread in Hz.
    pub fll_sigma_hz: f64,
    /// Dynamic frequency stress carried into the FLL threshold, in Hz.
    pub dynamic_stress_hz: f64,
}

/// Lock-detector thresholds derived from the estimated discriminator distributions.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LockDetectorThresholds {
    /// Normalized DLL discriminator threshold for entering lock.
    pub dll_lock: f32,
    /// Normalized DLL discriminator threshold for retaining lock.
    pub dll_hold: f32,
    /// PLL phase threshold in radians for entering lock.
    pub pll_lock_rad: f32,
    /// PLL phase threshold in radians for retaining lock.
    pub pll_hold_rad: f32,
    /// FLL frequency threshold in Hz for entering lock.
    pub fll_lock_hz: f64,
    /// Discriminator distribution estimates used to produce the thresholds.
    pub distributions: LockDetectorDistributions,
}

/// Distribution assumptions used to estimate lock-detector error probabilities.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LockDetectorProbabilityInput {
    /// Calibrated lock-detector thresholds and locked-state discriminator spreads.
    pub thresholds: LockDetectorThresholds,
    /// Half-width of the unlocked DLL discriminator support.
    pub unlocked_dll_half_width: f64,
    /// Half-width of the unlocked PLL phase support in radians.
    pub unlocked_pll_half_width_rad: f64,
    /// Half-width of the unlocked FLL frequency support in Hz.
    pub unlocked_fll_half_width_hz: f64,
    /// Lost-lock discriminator bias in one-sigma units used for missed-unlock estimation.
    pub missed_unlock_bias_sigma: f64,
}

/// Estimated lock-detector error probabilities for one operating point.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LockDetectorProbabilitySummary {
    /// Probability that DLL noise rejects a genuinely locked epoch.
    pub dll_false_unlock_probability: f64,
    /// Probability that PLL noise rejects a genuinely locked epoch.
    pub pll_false_unlock_probability: f64,
    /// Probability that FLL noise rejects a genuinely locked epoch.
    pub fll_false_unlock_probability: f64,
    /// Probability that any detector rejects a genuinely locked epoch.
    pub false_unlock_probability: f64,
    /// Probability that an unlocked DLL residual falls inside the lock gate.
    pub dll_false_lock_probability: f64,
    /// Probability that an unlocked PLL residual falls inside the lock gate.
    pub pll_false_lock_probability: f64,
    /// Probability that an unlocked FLL residual falls inside the lock gate.
    pub fll_false_lock_probability: f64,
    /// Probability that all detector gates accept an unlocked epoch.
    pub false_lock_probability: f64,
    /// Probability that a biased lost-lock residual still passes all detector gates.
    pub missed_unlock_probability: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum LoopFilterOrder {
    First,
    Second,
    Third,
}

impl LoopFilterOrder {
    fn as_usize(self) -> usize {
        match self {
            Self::First => 1,
            Self::Second => 2,
            Self::Third => 3,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct LoopObserverDesign {
    correction_gains: [f64; 3],
    closed_loop_pole: f64,
}

/// Tracking loop profile selected by the adaptation controller.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TrackingLoopProfileKind {
    /// Use the configured base loop parameters unchanged.
    Nominal,
    /// Narrow the loops and lengthen coherent integration for weak but stable signals.
    WeakSignal,
    /// Widen the loops and shorten coherent integration under strong dynamics.
    DynamicStress,
}

/// Loop parameters chosen for the next tracking epoch.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TrackingLoopProfile {
    /// Effective DLL bandwidth in Hz.
    pub dll_bw_hz: f64,
    /// Effective PLL bandwidth in Hz.
    pub pll_bw_hz: f64,
    /// Effective FLL bandwidth in Hz.
    pub fll_bw_hz: f64,
    /// Effective coherent integration length in milliseconds.
    pub integration_ms: u32,
}

/// Inputs evaluated by the tracking adaptation controller.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TrackingAdaptationInput {
    /// Current CN0 estimate in dB-Hz.
    pub cn0_dbhz: f64,
    /// Current FLL discriminator error in Hz.
    pub fll_error_hz: f64,
    /// Current carrier frequency-rate estimate in Hz/s.
    pub carrier_rate_hz_per_s: f64,
    /// Whether the carrier loop is stable enough to support longer coherent integration.
    pub carrier_lock_ready: bool,
    /// Whether the channel is already in a steady tracking state.
    pub steady_state_lock: bool,
    /// Whether recent discriminator behavior is stable.
    pub discriminator_stable: bool,
}

/// Stateful hysteresis memory for tracking adaptation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TrackingAdaptationState {
    /// Currently active loop profile.
    pub active_profile: TrackingLoopProfileKind,
    pending_profile: Option<TrackingLoopProfileKind>,
    pending_epochs: u8,
}

impl Default for TrackingAdaptationState {
    fn default() -> Self {
        Self {
            active_profile: TrackingLoopProfileKind::Nominal,
            pending_profile: None,
            pending_epochs: 0,
        }
    }
}

/// Result of one tracking adaptation step.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TrackingAdaptationDecision {
    /// Updated adaptation state to persist into the next epoch.
    pub state: TrackingAdaptationState,
    /// Active loop profile kind chosen for the next epoch.
    pub profile_kind: TrackingLoopProfileKind,
    /// Effective loop parameters chosen for the next epoch.
    pub profile: TrackingLoopProfile,
}

/// Advance the tracking adaptation controller by one epoch.
pub fn advance_tracking_adaptation(
    base_profile: TrackingLoopProfile,
    state: TrackingAdaptationState,
    input: TrackingAdaptationInput,
) -> TrackingAdaptationDecision {
    let requested_profile = requested_tracking_loop_profile(state.active_profile, input);
    let required_confirmation_epochs = required_confirmation_epochs(requested_profile);
    let mut next_state = state;

    if requested_profile == state.active_profile {
        next_state.pending_profile = None;
        next_state.pending_epochs = 0;
    } else if next_state.pending_profile == Some(requested_profile) {
        next_state.pending_epochs =
            next_state.pending_epochs.saturating_add(1).min(MAX_TRACKING_ADAPTATION_PENDING_EPOCHS);
    } else {
        next_state.pending_profile = Some(requested_profile);
        next_state.pending_epochs = 1;
    }

    if requested_profile != state.active_profile
        && next_state.pending_epochs >= required_confirmation_epochs
    {
        next_state.active_profile = requested_profile;
        next_state.pending_profile = None;
        next_state.pending_epochs = 0;
    }

    let profile = tracking_loop_profile(base_profile, next_state.active_profile);
    TrackingAdaptationDecision {
        state: next_state,
        profile_kind: next_state.active_profile,
        profile,
    }
}

fn tracking_loop_profile(
    base_profile: TrackingLoopProfile,
    profile_kind: TrackingLoopProfileKind,
) -> TrackingLoopProfile {
    match profile_kind {
        TrackingLoopProfileKind::Nominal => TrackingLoopProfile {
            integration_ms: base_profile.integration_ms.max(1),
            ..base_profile
        },
        TrackingLoopProfileKind::WeakSignal => TrackingLoopProfile {
            dll_bw_hz: base_profile.dll_bw_hz * 0.6,
            pll_bw_hz: base_profile.pll_bw_hz * 0.55,
            fll_bw_hz: base_profile.fll_bw_hz * 0.5,
            integration_ms: weak_signal_integration_ms(base_profile.integration_ms),
        },
        TrackingLoopProfileKind::DynamicStress => TrackingLoopProfile {
            dll_bw_hz: base_profile.dll_bw_hz * 2.0,
            pll_bw_hz: base_profile.pll_bw_hz * 3.5,
            fll_bw_hz: base_profile.fll_bw_hz * 4.0,
            integration_ms: DYNAMIC_STRESS_INTEGRATION_MS,
        },
    }
}

fn weak_signal_integration_ms(base_integration_ms: u32) -> u32 {
    base_integration_ms.clamp(WEAK_SIGNAL_INTEGRATION_MS, 10)
}

fn requested_tracking_loop_profile(
    active_profile: TrackingLoopProfileKind,
    input: TrackingAdaptationInput,
) -> TrackingLoopProfileKind {
    if weak_signal_requested(active_profile, input) {
        return TrackingLoopProfileKind::WeakSignal;
    }
    if dynamic_stress_requested(active_profile, input) {
        return TrackingLoopProfileKind::DynamicStress;
    }
    TrackingLoopProfileKind::Nominal
}

fn dynamic_stress_requested(
    active_profile: TrackingLoopProfileKind,
    input: TrackingAdaptationInput,
) -> bool {
    let fll_error_hz = input.fll_error_hz.abs();
    let carrier_rate_hz_per_s = input.carrier_rate_hz_per_s.abs();
    let reliable_dynamic_evidence = input.cn0_dbhz.is_finite()
        && (input.cn0_dbhz >= WEAK_SIGNAL_ENTRY_CN0_DBHZ || !input.steady_state_lock);
    let (fll_threshold_hz, carrier_rate_threshold_hz_per_s) =
        if active_profile == TrackingLoopProfileKind::DynamicStress {
            (DYNAMIC_STRESS_EXIT_FLL_ERROR_HZ, DYNAMIC_STRESS_EXIT_CARRIER_RATE_HZ_PER_S)
        } else {
            (DYNAMIC_STRESS_ENTRY_FLL_ERROR_HZ, DYNAMIC_STRESS_ENTRY_CARRIER_RATE_HZ_PER_S)
        };

    (reliable_dynamic_evidence
        && (fll_error_hz >= fll_threshold_hz
            || carrier_rate_hz_per_s >= carrier_rate_threshold_hz_per_s))
        || (!input.discriminator_stable
            && !input.steady_state_lock
            && input.cn0_dbhz.is_finite()
            && input.cn0_dbhz >= WEAK_SIGNAL_EXIT_CN0_DBHZ)
}

fn weak_signal_requested(
    active_profile: TrackingLoopProfileKind,
    input: TrackingAdaptationInput,
) -> bool {
    if !input.carrier_lock_ready || !input.steady_state_lock || !input.discriminator_stable {
        return false;
    }
    let cn0_threshold_dbhz = if active_profile == TrackingLoopProfileKind::WeakSignal {
        WEAK_SIGNAL_EXIT_CN0_DBHZ
    } else {
        WEAK_SIGNAL_ENTRY_CN0_DBHZ
    };
    input.cn0_dbhz.is_finite() && input.cn0_dbhz <= cn0_threshold_dbhz
}

fn required_confirmation_epochs(profile_kind: TrackingLoopProfileKind) -> u8 {
    match profile_kind {
        TrackingLoopProfileKind::Nominal => 4,
        TrackingLoopProfileKind::WeakSignal => 2,
        TrackingLoopProfileKind::DynamicStress => 1,
    }
}

/// Wrap phase in cycles into a signed interval centered at zero.
pub fn wrap_phase_cycles_signed(phase_cycles: f64) -> f64 {
    let mut wrapped = phase_cycles.rem_euclid(1.0);
    if wrapped > 0.5 {
        wrapped -= 1.0;
    }
    wrapped
}

/// Wrap phase in radians into the positive `[0, 2π)` interval.
pub fn wrap_phase_radians_positive(phase_radians: f64) -> f64 {
    phase_radians.rem_euclid(std::f64::consts::TAU)
}

/// Convert carrier phase cycles into a wrapped carrier phase offset in radians.
pub fn carrier_phase_offset_radians(carrier_phase_cycles: f64) -> f64 {
    wrap_phase_radians_positive(carrier_phase_cycles * std::f64::consts::TAU)
}

/// Measure the shortest signed phase delta between two carrier phases in cycles.
pub fn wrapped_phase_delta_cycles(next_phase_cycles: f64, previous_phase_cycles: f64) -> f64 {
    wrap_phase_cycles_signed(next_phase_cycles - previous_phase_cycles)
}

/// Correlate one coherent interval against early, prompt, and late replicas.
pub fn correlate_early_prompt_late<F>(
    input: EarlyPromptLateCorrelatorInput<'_>,
    code_value_at_phase: F,
) -> EarlyPromptLateCorrelation
where
    F: Fn(f64) -> f32,
{
    if !input.sample_rate_hz.is_finite()
        || input.sample_rate_hz <= 0.0
        || !input.carrier_hz.is_finite()
        || !input.carrier_phase_offset_radians.is_finite()
        || !input.base_chip_phase.is_finite()
        || !input.chips_per_sample.is_finite()
        || !input.early_late_spacing_chips.is_finite()
    {
        return EarlyPromptLateCorrelation {
            early: Complex::new(0.0, 0.0),
            prompt: Complex::new(0.0, 0.0),
            late: Complex::new(0.0, 0.0),
            early_late_noise_weight_energy: 0.0,
        };
    }

    let mut carrier_nco =
        Nco::with_phase(input.carrier_hz, input.sample_rate_hz, input.carrier_phase_offset_radians);
    let mut early = Complex::new(0.0f32, 0.0f32);
    let mut prompt = Complex::new(0.0f32, 0.0f32);
    let mut late = Complex::new(0.0f32, 0.0f32);
    let mut early_late_noise_weight_energy = 0.0f64;

    for (sample_index, sample) in input.samples.iter().enumerate() {
        let (sin, cos) = carrier_nco.next_sin_cos();
        let mixed_sample = *sample * Complex::new(cos as f32, -sin as f32);
        let chip_phase = input.base_chip_phase + sample_index as f64 * input.chips_per_sample;
        let early_code =
            Complex::new(code_value_at_phase(chip_phase - input.early_late_spacing_chips), 0.0);
        let prompt_code = Complex::new(code_value_at_phase(chip_phase), 0.0);
        let late_code =
            Complex::new(code_value_at_phase(chip_phase + input.early_late_spacing_chips), 0.0);
        let noise_weight = early_code - late_code;
        early_late_noise_weight_energy += noise_weight.norm_sqr() as f64;

        early += mixed_sample * early_code;
        prompt += mixed_sample * prompt_code;
        late += mixed_sample * late_code;
    }

    EarlyPromptLateCorrelation { early, prompt, late, early_late_noise_weight_energy }
}

/// Convert a carrier phase delta over one coherent interval into residual frequency error.
pub fn carrier_frequency_error_hz_from_phase_delta(
    phase_delta_rad: f64,
    coherent_integration_s: f64,
) -> f64 {
    if !phase_delta_rad.is_finite()
        || !coherent_integration_s.is_finite()
        || coherent_integration_s <= 0.0
    {
        return 0.0;
    }
    phase_delta_rad / (std::f64::consts::TAU * coherent_integration_s)
}

fn loop_observer_design(
    order: LoopFilterOrder,
    noise_bandwidth_hz: f64,
    coherent_integration_s: f64,
) -> LoopObserverDesign {
    if !noise_bandwidth_hz.is_finite()
        || noise_bandwidth_hz <= 0.0
        || !coherent_integration_s.is_finite()
        || coherent_integration_s <= 0.0
    {
        return LoopObserverDesign { correction_gains: [0.0; 3], closed_loop_pole: 1.0 };
    }

    let nyquist_bandwidth_hz = 0.5 / coherent_integration_s;
    if noise_bandwidth_hz >= nyquist_bandwidth_hz {
        return LoopObserverDesign {
            correction_gains: loop_observer_gains(order, 0.0, coherent_integration_s),
            closed_loop_pole: 0.0,
        };
    }

    let mut low_pole = 0.0;
    let mut high_pole = 1.0 - f64::EPSILON;
    for _ in 0..LOOP_FILTER_BANDWIDTH_SOLVER_ITERATIONS {
        let pole = (low_pole + high_pole) * 0.5;
        let gains = loop_observer_gains(order, pole, coherent_integration_s);
        let measured_bandwidth_hz = loop_noise_bandwidth_hz(order, coherent_integration_s, gains);
        if measured_bandwidth_hz > noise_bandwidth_hz {
            low_pole = pole;
        } else {
            high_pole = pole;
        }
    }

    let closed_loop_pole = high_pole;
    LoopObserverDesign {
        correction_gains: loop_observer_gains(order, closed_loop_pole, coherent_integration_s),
        closed_loop_pole,
    }
}

fn loop_observer_gains(
    order: LoopFilterOrder,
    closed_loop_pole: f64,
    coherent_integration_s: f64,
) -> [f64; 3] {
    let dimension = order.as_usize();
    let transition = loop_state_transition(order, coherent_integration_s);
    let output = loop_output_matrix();
    let transition_transpose = transpose_square_matrix(transition, dimension);
    let input = output;
    let controllability = controllability_matrix(transition_transpose, input, dimension);
    let controllability_inverse = invert_square_matrix(controllability, dimension)
        .expect("integrator-chain observer design must remain controllable");
    let characteristic = repeated_pole_characteristic_coefficients(closed_loop_pole, dimension);
    let desired_matrix =
        evaluate_matrix_polynomial(transition_transpose, characteristic, dimension);
    let mut gain = [0.0; 3];
    for (column, gain_value) in gain.iter_mut().enumerate().take(dimension) {
        *gain_value = controllability_inverse[dimension - 1]
            .iter()
            .zip(desired_matrix.iter())
            .take(dimension)
            .map(|(inverse, desired_row)| inverse * desired_row[column])
            .sum();
    }

    let mut correction_gains = [0.0; 3];
    correction_gains[..dimension].copy_from_slice(&gain[..dimension]);
    correction_gains
}

fn loop_noise_bandwidth_hz(
    order: LoopFilterOrder,
    coherent_integration_s: f64,
    correction_gains: [f64; 3],
) -> f64 {
    let dimension = order.as_usize();
    let corrected_transition =
        corrected_loop_transition(order, coherent_integration_s, correction_gains);
    let mut state = [0.0; 3];
    let mut impulse_energy = 0.0;
    let mut impulse_sum = 0.0;

    for epoch in 0..LOOP_FILTER_IMPULSE_RESPONSE_EPOCHS {
        let input = if epoch == 0 { 1.0 } else { 0.0 };
        let mut next_state = [0.0; 3];
        for (row, next_state_value) in next_state.iter_mut().enumerate().take(dimension) {
            *next_state_value = correction_gains[row] * input;
            *next_state_value += corrected_transition[row]
                .iter()
                .zip(state.iter())
                .take(dimension)
                .map(|(transition_value, state_value)| transition_value * state_value)
                .sum::<f64>();
        }
        state = next_state;
        let output = state[0];
        impulse_sum += output;
        impulse_energy += output * output;
        if epoch + 1 >= dimension
            && state[..dimension]
                .iter()
                .all(|value| value.abs() <= LOOP_FILTER_IMPULSE_RESPONSE_EPSILON)
        {
            break;
        }
    }

    if impulse_sum.abs() <= f64::EPSILON {
        return 0.0;
    }

    impulse_energy * 0.5 / coherent_integration_s / (impulse_sum * impulse_sum)
}

fn corrected_loop_transition(
    order: LoopFilterOrder,
    coherent_integration_s: f64,
    correction_gains: [f64; 3],
) -> [[f64; 3]; 3] {
    let dimension = order.as_usize();
    let transition = loop_state_transition(order, coherent_integration_s);
    let output = loop_output_matrix();
    let mut corrected = [[0.0; 3]; 3];

    for (row, corrected_row) in corrected.iter_mut().enumerate().take(dimension) {
        for (column, corrected_value) in corrected_row.iter_mut().enumerate().take(dimension) {
            *corrected_value = transition[row][column] - correction_gains[row] * output[column];
        }
    }

    corrected
}

fn loop_state_transition(order: LoopFilterOrder, coherent_integration_s: f64) -> [[f64; 3]; 3] {
    let mut transition = identity_square_matrix(order.as_usize());
    match order {
        LoopFilterOrder::First => {}
        LoopFilterOrder::Second => {
            transition[0][1] = coherent_integration_s;
        }
        LoopFilterOrder::Third => {
            transition[0][1] = coherent_integration_s;
            transition[0][2] = 0.5 * coherent_integration_s * coherent_integration_s;
            transition[1][2] = coherent_integration_s;
        }
    }
    transition
}

fn loop_output_matrix() -> [f64; 3] {
    [1.0, 0.0, 0.0]
}

fn repeated_pole_characteristic_coefficients(closed_loop_pole: f64, dimension: usize) -> [f64; 4] {
    match dimension {
        1 => [1.0, -closed_loop_pole, 0.0, 0.0],
        2 => [1.0, -2.0 * closed_loop_pole, closed_loop_pole * closed_loop_pole, 0.0],
        3 => [
            1.0,
            -3.0 * closed_loop_pole,
            3.0 * closed_loop_pole * closed_loop_pole,
            -closed_loop_pole * closed_loop_pole * closed_loop_pole,
        ],
        _ => panic!("unsupported loop-filter order"),
    }
}

fn evaluate_matrix_polynomial(
    matrix: [[f64; 3]; 3],
    coefficients: [f64; 4],
    dimension: usize,
) -> [[f64; 3]; 3] {
    let mut matrix_power = identity_square_matrix(dimension);
    let mut polynomial = scale_square_matrix(matrix_power, coefficients[dimension], dimension);

    for power in 1..=dimension {
        matrix_power = multiply_square_matrices(matrix_power, matrix, dimension);
        polynomial = add_square_matrices(
            polynomial,
            scale_square_matrix(matrix_power, coefficients[dimension - power], dimension),
            dimension,
        );
    }

    polynomial
}

fn controllability_matrix(
    transition: [[f64; 3]; 3],
    input: [f64; 3],
    dimension: usize,
) -> [[f64; 3]; 3] {
    let mut controllability = [[0.0; 3]; 3];
    let mut column = input;
    for column_index in 0..dimension {
        for (row, controllability_row) in controllability.iter_mut().enumerate().take(dimension) {
            controllability_row[column_index] = column[row];
        }
        column = multiply_square_matrix_vector(transition, column, dimension);
    }
    controllability
}

fn multiply_square_matrix_vector(
    matrix: [[f64; 3]; 3],
    vector: [f64; 3],
    dimension: usize,
) -> [f64; 3] {
    let mut result = [0.0; 3];
    for (row, result_value) in result.iter_mut().enumerate().take(dimension) {
        *result_value = matrix[row]
            .iter()
            .zip(vector.iter())
            .take(dimension)
            .map(|(matrix_value, vector_value)| matrix_value * vector_value)
            .sum();
    }
    result
}

fn multiply_square_matrices(
    left: [[f64; 3]; 3],
    right: [[f64; 3]; 3],
    dimension: usize,
) -> [[f64; 3]; 3] {
    let mut result = [[0.0; 3]; 3];
    for (row, result_row) in result.iter_mut().enumerate().take(dimension) {
        for (column, result_value) in result_row.iter_mut().enumerate().take(dimension) {
            *result_value = left[row]
                .iter()
                .take(dimension)
                .enumerate()
                .map(|(index, left_value)| left_value * right[index][column])
                .sum();
        }
    }
    result
}

fn add_square_matrices(
    left: [[f64; 3]; 3],
    right: [[f64; 3]; 3],
    dimension: usize,
) -> [[f64; 3]; 3] {
    let mut result = [[0.0; 3]; 3];
    for (row, result_row) in result.iter_mut().enumerate().take(dimension) {
        for (column, result_value) in result_row.iter_mut().enumerate().take(dimension) {
            *result_value = left[row][column] + right[row][column];
        }
    }
    result
}

fn scale_square_matrix(matrix: [[f64; 3]; 3], scale: f64, dimension: usize) -> [[f64; 3]; 3] {
    let mut result = [[0.0; 3]; 3];
    for (row, result_row) in result.iter_mut().enumerate().take(dimension) {
        for (column, result_value) in result_row.iter_mut().enumerate().take(dimension) {
            *result_value = matrix[row][column] * scale;
        }
    }
    result
}

fn transpose_square_matrix(matrix: [[f64; 3]; 3], dimension: usize) -> [[f64; 3]; 3] {
    let mut transpose = [[0.0; 3]; 3];
    for (row, matrix_row) in matrix.iter().enumerate().take(dimension) {
        for (column, matrix_value) in matrix_row.iter().enumerate().take(dimension) {
            transpose[column][row] = *matrix_value;
        }
    }
    transpose
}

fn identity_square_matrix(dimension: usize) -> [[f64; 3]; 3] {
    let mut identity = [[0.0; 3]; 3];
    for (index, identity_row) in identity.iter_mut().enumerate().take(dimension) {
        identity_row[index] = 1.0;
    }
    identity
}

fn invert_square_matrix(matrix: [[f64; 3]; 3], dimension: usize) -> Option<[[f64; 3]; 3]> {
    let mut augmented = [[0.0; 6]; 3];
    for (row, augmented_row) in augmented.iter_mut().enumerate().take(dimension) {
        augmented_row[..dimension].copy_from_slice(&matrix[row][..dimension]);
        augmented_row[dimension + row] = 1.0;
    }

    for pivot_index in 0..dimension {
        let mut best_pivot_row = pivot_index;
        let mut best_pivot_abs = augmented[pivot_index][pivot_index].abs();
        for (row, augmented_row) in
            augmented.iter().enumerate().take(dimension).skip(pivot_index + 1)
        {
            let pivot_abs = augmented_row[pivot_index].abs();
            if pivot_abs > best_pivot_abs {
                best_pivot_abs = pivot_abs;
                best_pivot_row = row;
            }
        }
        if best_pivot_abs <= f64::EPSILON {
            return None;
        }
        if best_pivot_row != pivot_index {
            augmented.swap(best_pivot_row, pivot_index);
        }

        let pivot = augmented[pivot_index][pivot_index];
        for pivot_value in augmented[pivot_index][pivot_index..(dimension * 2)].iter_mut() {
            *pivot_value /= pivot;
        }
        for row in 0..dimension {
            if row == pivot_index {
                continue;
            }
            let factor = augmented[row][pivot_index];
            if factor.abs() <= f64::EPSILON {
                continue;
            }
            let pivot_row = augmented[pivot_index];
            for (offset, value) in
                augmented[row][pivot_index..(dimension * 2)].iter_mut().enumerate()
            {
                *value -= factor * pivot_row[pivot_index + offset];
            }
        }
    }

    let mut inverse = [[0.0; 3]; 3];
    for (row, inverse_row) in inverse.iter_mut().enumerate().take(dimension) {
        inverse_row[..dimension].copy_from_slice(&augmented[row][dimension..(dimension * 2)]);
    }
    Some(inverse)
}

/// Derive first-order loop coefficients from noise bandwidth and coherent integration time.
pub fn first_order_loop_coefficients(
    noise_bandwidth_hz: f64,
    coherent_integration_s: f64,
) -> FirstOrderLoopCoefficients {
    let design =
        loop_observer_design(LoopFilterOrder::First, noise_bandwidth_hz, coherent_integration_s);
    FirstOrderLoopCoefficients {
        error_blend: design.correction_gains[0],
        rate_gain_hz: design.correction_gains[0] / coherent_integration_s.max(f64::EPSILON),
    }
}

/// Derive first-order coefficients for carrier-frequency loops from angular bandwidth.
pub fn first_order_angular_loop_coefficients(
    noise_bandwidth_hz: f64,
    coherent_integration_s: f64,
) -> FirstOrderLoopCoefficients {
    first_order_loop_coefficients(noise_bandwidth_hz, coherent_integration_s)
}

/// Derive second-order DLL coefficients from loop bandwidth and coherent integration time.
pub fn delay_lock_loop_coefficients(
    noise_bandwidth_hz: f64,
    coherent_integration_s: f64,
) -> DelayLockLoopCoefficients {
    let design =
        loop_observer_design(LoopFilterOrder::Second, noise_bandwidth_hz, coherent_integration_s);
    DelayLockLoopCoefficients {
        phase_blend: design.correction_gains[0],
        rate_gain_hz_per_chip: design.correction_gains[1],
    }
}

/// Derive a phase-lock filter from loop bandwidth and coherent integration time.
pub fn phase_lock_loop_coefficients(
    noise_bandwidth_hz: f64,
    coherent_integration_s: f64,
) -> PhaseLockLoopCoefficients {
    let design =
        loop_observer_design(LoopFilterOrder::Third, noise_bandwidth_hz, coherent_integration_s);
    PhaseLockLoopCoefficients {
        phase_blend: design.correction_gains[0],
        frequency_gain_hz_per_rad: design.correction_gains[1] / std::f64::consts::TAU,
        frequency_rate_gain_hz_per_s_per_rad: design.correction_gains[2] / std::f64::consts::TAU,
    }
}

/// DLL/PLL/FLL discriminators from early/prompt/late.
pub fn discriminators(
    early: Complex<f32>,
    prompt: Complex<f32>,
    late: Complex<f32>,
    prev_prompt: Option<Complex<f32>>,
) -> (f32, f32, f32, bool) {
    let p = prompt.norm();
    let dll = dll_discriminator_from_early_late(early, late);
    let pll = prompt.im.atan2(prompt.re);
    let fll = if let Some(prev) = prev_prompt {
        let dot = prompt.re * prev.re + prompt.im * prev.im;
        let det = prompt.im * prev.re - prompt.re * prev.im;
        det.atan2(dot)
    } else {
        0.0
    };
    let lock = p > 0.1 && dll.abs() < 0.5;
    (dll, pll, fll, lock)
}

/// Noncoherent early-minus-late DLL discriminator.
pub fn dll_discriminator_from_early_late(early: Complex<f32>, late: Complex<f32>) -> f32 {
    let early_norm = early.norm();
    let late_norm = late.norm();
    if early_norm + late_norm > 0.0 {
        (early_norm - late_norm) / (early_norm + late_norm)
    } else {
        0.0
    }
}

/// Double-delta DLL discriminator using inner and outer early/late pairs.
pub fn double_delta_dll_discriminator(
    inner_early: Complex<f32>,
    inner_late: Complex<f32>,
    outer_early: Complex<f32>,
    outer_late: Complex<f32>,
) -> f32 {
    let inner_early_norm = inner_early.norm();
    let inner_late_norm = inner_late.norm();
    let denominator = inner_early_norm + inner_late_norm;
    if denominator <= f32::EPSILON {
        return 0.0;
    }
    let inner_difference = inner_early_norm - inner_late_norm;
    let outer_difference = outer_early.norm() - outer_late.norm();
    ((inner_difference - DOUBLE_DELTA_OUTER_WEIGHT * outer_difference) / denominator)
        .clamp(-1.0, 1.0)
}

/// Normalize a noncoherent early-minus-late DLL discriminator to the legacy 0.5-chip loop gain.
pub fn normalize_dll_discriminator(discriminator: f32, early_late_spacing_chips: f64) -> f32 {
    if !discriminator.is_finite()
        || !early_late_spacing_chips.is_finite()
        || early_late_spacing_chips <= 0.0
        || early_late_spacing_chips >= 1.0
    {
        return 0.0;
    }
    let normalized = discriminator as f64
        * ((2.0 - early_late_spacing_chips) / (2.0 - DLL_REFERENCE_EARLY_LATE_SPACING_CHIPS));
    normalized.clamp(-1.0, 1.0) as f32
}

/// Estimate CN0 in dB-Hz from coherent prompt energy and a signal-free noise proxy.
pub fn estimate_cn0_dbhz(
    prompt: Complex<f32>,
    noise: Complex<f32>,
    sample_rate_hz: f64,
    prompt_coherent_gain: f64,
    noise_weight_energy: f64,
) -> f64 {
    if !sample_rate_hz.is_finite()
        || sample_rate_hz <= 0.0
        || !prompt_coherent_gain.is_finite()
        || prompt_coherent_gain <= 0.0
        || !noise_weight_energy.is_finite()
        || noise_weight_energy <= 0.0
    {
        return 0.0;
    }
    let signal_power = (prompt.norm_sqr() as f64).max(1e-12);
    let noise_power = (noise.norm_sqr() as f64).max(1e-12);
    let noise_power_per_sample = noise_power / noise_weight_energy;
    let prompt_noise_power = prompt_coherent_gain * noise_power_per_sample;
    let coherent_signal_power = (signal_power - prompt_noise_power).max(1e-12);
    let signal_power_per_sample = coherent_signal_power / prompt_coherent_gain.powi(2);
    let cn0_linear = (signal_power_per_sample * sample_rate_hz / noise_power_per_sample).max(1e-12);
    10.0 * cn0_linear.log10()
}

/// Convert a coherent interval length from samples into seconds.
pub fn coherent_integration_seconds(epoch_len_samples: usize, sample_rate_hz: f64) -> f64 {
    if !sample_rate_hz.is_finite() || sample_rate_hz <= 0.0 {
        return 0.0;
    }
    epoch_len_samples as f64 / sample_rate_hz
}

/// Threshold for treating FLL error as locked.
pub fn fll_lock_threshold_hz(fll_bw_hz: f64) -> f64 {
    fll_bw_hz.max(5.0)
}

/// Estimate discriminator distributions from C/N0, integration time, spacing, and dynamics.
pub fn lock_detector_distributions(
    input: LockDetectorCalibrationInput,
) -> LockDetectorDistributions {
    let coherent_snr_linear =
        coherent_snr_linear(input.cn0_dbhz, input.coherent_integration_s).max(1.0e-6);
    let discriminator_noise_scale = (2.0 * coherent_snr_linear).sqrt().max(1.0e-6);
    let spacing_chips = input.early_late_spacing_chips.clamp(0.05, 0.95);
    let effective_sample_separation = (input.samples_per_chip * spacing_chips).max(1.0e-6);
    let slope_gain =
        ((2.0 - spacing_chips) / (2.0 - DLL_REFERENCE_EARLY_LATE_SPACING_CHIPS)).max(0.1);
    let sample_resolution_sigma = if effective_sample_separation < 1.0 {
        (1.0 - effective_sample_separation) * 0.35
    } else {
        0.0
    };
    let dll_sigma = ((1.0 / (discriminator_noise_scale * slope_gain)).powi(2)
        + sample_resolution_sigma.powi(2))
    .sqrt();
    let pll_sigma_rad = 1.0 / discriminator_noise_scale;
    let fll_sigma_hz = 1.0
        / (std::f64::consts::TAU * input.coherent_integration_s.max(1.0e-6))
        / discriminator_noise_scale;
    LockDetectorDistributions {
        coherent_snr_linear,
        dll_sigma,
        pll_sigma_rad,
        fll_sigma_hz,
        dynamic_stress_hz: input.dynamic_stress_hz.max(0.0),
    }
}

/// Derive lock and hold thresholds from estimated discriminator distributions.
pub fn calibrated_lock_detector_thresholds(
    input: LockDetectorCalibrationInput,
) -> LockDetectorThresholds {
    let distributions = lock_detector_distributions(input);
    let dll_quantile = two_sided_normal_quantile(input.dll_false_unlock_probability);
    let pll_quantile = two_sided_normal_quantile(input.pll_false_unlock_probability);
    let fll_quantile = two_sided_normal_quantile(input.fll_false_unlock_probability);
    let dll_lock = (dll_quantile * distributions.dll_sigma)
        .clamp(dll_lock_threshold(input.samples_per_chip, input.early_late_spacing_chips) as f64, 0.95)
        as f32;
    let pll_lock_upper_rad = std::f64::consts::PI - f32::EPSILON as f64;
    let pll_lock_rad = (pll_quantile * distributions.pll_sigma_rad)
        .clamp(PLL_LOCK_MIN_PHASE_ERROR_RAD as f64, pll_lock_upper_rad) as f32;
    let fll_lock_hz = (fll_quantile * distributions.fll_sigma_hz + distributions.dynamic_stress_hz)
        .max(fll_lock_threshold_hz(input.fll_bw_hz));
    LockDetectorThresholds {
        dll_lock,
        dll_hold: (dll_lock as f64 * 1.5)
            .clamp(dll_hold_threshold(input.samples_per_chip, input.early_late_spacing_chips) as f64, 0.98)
            as f32,
        pll_lock_rad,
        pll_hold_rad: (pll_lock_rad as f64 * 1.35).clamp(pll_lock_rad as f64, std::f64::consts::PI)
            as f32,
        fll_lock_hz,
        distributions,
    }
}

/// Estimate false-lock, false-unlock, and missed-unlock rates for calibrated lock gates.
pub fn lock_detector_probability_summary(
    input: LockDetectorProbabilityInput,
) -> LockDetectorProbabilitySummary {
    let thresholds = input.thresholds;
    let dll_false_unlock_probability = two_sided_gaussian_tail_probability(
        thresholds.dll_lock as f64,
        thresholds.distributions.dll_sigma,
    );
    let pll_false_unlock_probability = two_sided_gaussian_tail_probability(
        thresholds.pll_lock_rad as f64,
        thresholds.distributions.pll_sigma_rad,
    );
    let fll_false_unlock_probability = two_sided_gaussian_tail_probability(
        thresholds.fll_lock_hz,
        thresholds.distributions.fll_sigma_hz,
    );
    let false_unlock_probability = union_probability([
        dll_false_unlock_probability,
        pll_false_unlock_probability,
        fll_false_unlock_probability,
    ]);
    let dll_false_lock_probability =
        uniform_gate_probability(thresholds.dll_lock as f64, input.unlocked_dll_half_width);
    let pll_false_lock_probability =
        uniform_gate_probability(thresholds.pll_lock_rad as f64, input.unlocked_pll_half_width_rad);
    let fll_false_lock_probability =
        uniform_gate_probability(thresholds.fll_lock_hz, input.unlocked_fll_half_width_hz);
    let false_lock_probability =
        dll_false_lock_probability * pll_false_lock_probability * fll_false_lock_probability;
    let missed_unlock_bias_sigma = input.missed_unlock_bias_sigma.max(0.0);
    let missed_unlock_probability = biased_gaussian_gate_probability(
        thresholds.dll_lock as f64,
        thresholds.distributions.dll_sigma,
        missed_unlock_bias_sigma,
    ) * biased_gaussian_gate_probability(
        thresholds.pll_lock_rad as f64,
        thresholds.distributions.pll_sigma_rad,
        missed_unlock_bias_sigma,
    ) * biased_gaussian_gate_probability(
        thresholds.fll_lock_hz,
        thresholds.distributions.fll_sigma_hz,
        missed_unlock_bias_sigma,
    );

    LockDetectorProbabilitySummary {
        dll_false_unlock_probability,
        pll_false_unlock_probability,
        fll_false_unlock_probability,
        false_unlock_probability,
        dll_false_lock_probability,
        pll_false_lock_probability,
        fll_false_lock_probability,
        false_lock_probability,
        missed_unlock_probability,
    }
}

/// Threshold for treating DLL error as locked.
pub fn dll_lock_threshold(samples_per_chip: f64, early_late_spacing_chips: f64) -> f32 {
    let effective_sample_separation = samples_per_chip * early_late_spacing_chips.abs();
    if effective_sample_separation + f64::EPSILON < 1.0 {
        DLL_LOW_RESOLUTION_LOCK_MAX_CODE_ERROR
    } else {
        DLL_LOCK_MAX_CODE_ERROR
    }
}

/// Relaxed threshold for retaining DLL lock during tracking.
pub fn dll_hold_threshold(samples_per_chip: f64, early_late_spacing_chips: f64) -> f32 {
    dll_lock_threshold(samples_per_chip, early_late_spacing_chips).max(DLL_HOLD_MAX_CODE_ERROR)
}

fn coherent_snr_linear(cn0_dbhz: f64, coherent_integration_s: f64) -> f64 {
    if !cn0_dbhz.is_finite() || !coherent_integration_s.is_finite() || coherent_integration_s <= 0.0
    {
        return 1.0e-6;
    }
    10.0_f64.powf(cn0_dbhz / 10.0) * coherent_integration_s
}

fn two_sided_normal_quantile(tail_probability: f64) -> f64 {
    let bounded_tail =
        if tail_probability.is_finite() { tail_probability.clamp(1.0e-12, 0.5) } else { 1.0e-6 };
    inverse_standard_normal_cdf(1.0 - bounded_tail / 2.0)
}

fn two_sided_gaussian_tail_probability(threshold: f64, sigma: f64) -> f64 {
    if !threshold.is_finite() || !sigma.is_finite() || sigma <= 0.0 {
        return 1.0;
    }
    (2.0 * (1.0 - standard_normal_cdf((threshold / sigma).max(0.0)))).clamp(0.0, 1.0)
}

fn biased_gaussian_gate_probability(threshold: f64, sigma: f64, bias_sigma: f64) -> f64 {
    if !threshold.is_finite() || !sigma.is_finite() || sigma <= 0.0 {
        return 0.0;
    }
    let bias = bias_sigma.max(0.0) * sigma;
    let high = (threshold - bias) / sigma;
    let low = (-threshold - bias) / sigma;
    (standard_normal_cdf(high) - standard_normal_cdf(low)).clamp(0.0, 1.0)
}

fn uniform_gate_probability(threshold: f64, half_width: f64) -> f64 {
    if !threshold.is_finite() || !half_width.is_finite() || half_width <= 0.0 {
        return 0.0;
    }
    (threshold.abs() / half_width).clamp(0.0, 1.0)
}

fn union_probability<const N: usize>(probabilities: [f64; N]) -> f64 {
    let survival = probabilities
        .into_iter()
        .map(|probability| 1.0 - probability.clamp(0.0, 1.0))
        .product::<f64>();
    (1.0 - survival).clamp(0.0, 1.0)
}

fn standard_normal_cdf(value: f64) -> f64 {
    if !value.is_finite() {
        return if value.is_sign_positive() { 1.0 } else { 0.0 };
    }
    let sign = if value < 0.0 { -1.0 } else { 1.0 };
    let x = value.abs() / std::f64::consts::SQRT_2;
    let t = 1.0 / (1.0 + 0.327_591_1 * x);
    let erf = 1.0
        - (((((1.061_405_429 * t - 1.453_152_027) * t) + 1.421_413_741) * t - 0.284_496_736) * t
            + 0.254_829_592)
            * t
            * (-x * x).exp();
    0.5 * (1.0 + sign * erf)
}

fn inverse_standard_normal_cdf(probability: f64) -> f64 {
    const A: [f64; 6] = [
        -3.969_683_028_665_376e1,
        2.209_460_984_245_205e2,
        -2.759_285_104_469_687e2,
        1.383_577_518_672_69e2,
        -3.066_479_806_614_716e1,
        2.506_628_277_459_239,
    ];
    const B: [f64; 5] = [
        -5.447_609_879_822_406e1,
        1.615_858_368_580_409e2,
        -1.556_989_798_598_866e2,
        6.680_131_188_771_972e1,
        -1.328_068_155_288_572e1,
    ];
    const C: [f64; 6] = [
        -7.784_894_002_430_293e-3,
        -3.223_964_580_411_365e-1,
        -2.400_758_277_161_838,
        -2.549_732_539_343_734,
        4.374_664_141_464_968,
        2.938_163_982_698_783,
    ];
    const D: [f64; 4] = [
        7.784_695_709_041_462e-3,
        3.224_671_290_700_398e-1,
        2.445_134_137_142_996,
        3.754_408_661_907_416,
    ];
    let p = probability.clamp(1.0e-12, 1.0 - 1.0e-12);
    let lower = 0.024_25;
    let upper = 1.0 - lower;
    if p < lower {
        let q = (-2.0 * p.ln()).sqrt();
        return (((((C[0] * q + C[1]) * q + C[2]) * q + C[3]) * q + C[4]) * q + C[5])
            / ((((D[0] * q + D[1]) * q + D[2]) * q + D[3]) * q + 1.0);
    }
    if p > upper {
        let q = (-2.0 * (1.0 - p).ln()).sqrt();
        return -(((((C[0] * q + C[1]) * q + C[2]) * q + C[3]) * q + C[4]) * q + C[5])
            / ((((D[0] * q + D[1]) * q + D[2]) * q + D[3]) * q + 1.0);
    }
    let q = p - 0.5;
    let r = q * q;
    (((((A[0] * r + A[1]) * r + A[2]) * r + A[3]) * r + A[4]) * r + A[5]) * q
        / (((((B[0] * r + B[1]) * r + B[2]) * r + B[3]) * r + B[4]) * r + 1.0)
}

/// Ratio between prompt power and a stored reference.
pub fn prompt_power_ratio(prompt_power: f32, prompt_power_reference: f32) -> Option<f32> {
    if !prompt_power.is_finite()
        || !prompt_power_reference.is_finite()
        || prompt_power_reference <= 0.0
    {
        return None;
    }
    Some(prompt_power / prompt_power_reference)
}

/// Detect false lock from the relative early/late and prompt energies.
pub fn anti_false_lock_detected(
    early: Complex<f32>,
    prompt: Complex<f32>,
    late: Complex<f32>,
) -> bool {
    let prompt_norm = prompt.norm();
    if !prompt_norm.is_finite() || prompt_norm <= 0.0 {
        return true;
    }

    let early_late_mean = (early.norm() + late.norm()) * 0.5;
    !early_late_mean.is_finite()
        || early_late_mean >= prompt_norm * ANTI_FALSE_LOCK_MAX_EARLY_LATE_TO_PROMPT_RATIO
}

/// Update the prompt power reference with exponential decay.
pub fn update_prompt_power_reference(current_reference: f32, prompt_power: f32) -> f32 {
    if !prompt_power.is_finite() || prompt_power <= 0.0 {
        return current_reference;
    }
    if !current_reference.is_finite() || current_reference <= 0.0 {
        return prompt_power;
    }
    (current_reference * 0.98).max(prompt_power)
}

/// Refresh prompt power reference only while the channel is in clean steady tracking.
pub fn refresh_prompt_power_reference(
    current_reference: f32,
    prompt_power: f32,
    quality_class: TrackingQualityClass,
    anti_false_lock: bool,
) -> f32 {
    if !current_reference.is_finite() || current_reference <= 0.0 {
        return update_prompt_power_reference(current_reference, prompt_power);
    }
    if matches!(quality_class, TrackingQualityClass::Tracking) && !anti_false_lock {
        return update_prompt_power_reference(current_reference, prompt_power);
    }
    current_reference
}

/// Refresh the lock-reference CN0 only when current tracking evidence is reliable.
pub fn refresh_lock_reference_cn0_dbhz(
    current_reference: f64,
    cn0_dbhz: f64,
    reliable_tracking_lock: bool,
) -> f64 {
    if reliable_tracking_lock && cn0_dbhz.is_finite() && cn0_dbhz > 0.0 {
        return cn0_dbhz;
    }
    current_reference
}

/// Update a sliding-window CN0 estimate using linear-power averaging.
pub fn update_windowed_tracking_cn0_estimate(
    prompt_cn0_window: &mut VecDeque<f64>,
    epoch_cn0_dbhz: f64,
    max_epochs: usize,
    min_epochs: usize,
) -> Option<f64> {
    if !epoch_cn0_dbhz.is_finite() || epoch_cn0_dbhz <= 0.0 {
        return None;
    }
    prompt_cn0_window.push_back(epoch_cn0_dbhz);
    while prompt_cn0_window.len() > max_epochs {
        prompt_cn0_window.pop_front();
    }
    if prompt_cn0_window.len() < min_epochs {
        return None;
    }

    let mean_cn0_linear =
        prompt_cn0_window.iter().map(|value| 10.0_f64.powf(*value / 10.0)).sum::<f64>()
            / prompt_cn0_window.len() as f64;
    if !mean_cn0_linear.is_finite() || mean_cn0_linear <= 0.0 {
        return None;
    }
    Some(10.0 * mean_cn0_linear.log10())
}

/// Push a finite non-negative value into a bounded uncertainty window.
pub fn push_tracking_uncertainty_sample(window: &mut VecDeque<f64>, value: f64, max_epochs: usize) {
    if !value.is_finite() || value < 0.0 {
        return;
    }
    window.push_back(value);
    while window.len() > max_epochs {
        window.pop_front();
    }
}

/// Estimate tracking uncertainty from recent discriminator history.
pub fn estimate_tracking_uncertainty(
    code_error_window_samples: &VecDeque<f64>,
    carrier_phase_error_window_cycles: &VecDeque<f64>,
    doppler_error_window_hz: &VecDeque<f64>,
    cn0_estimate_window_dbhz: &VecDeque<f64>,
    input: TrackingUncertaintyInputs,
) -> TrackingUncertainty {
    let state_scale = tracking_uncertainty_state_scale(
        input.quality_class,
        input.channel_locked,
        input.dll_locked,
        input.anti_false_lock,
        input.cycle_slip,
    );
    let coherent_integration_scale = 1.0 / (input.integration_ms.max(1) as f64).sqrt();
    let cn0_scale = if input.cn0_dbhz.is_finite() {
        10.0_f64.powf(((45.0 - input.cn0_dbhz).clamp(-15.0, 20.0)) / 20.0)
    } else {
        4.0
    };
    let code_fallback = ((input.dll_err.abs() as f64) * input.samples_per_chip)
        .max(TRACKING_UNCERTAINTY_MIN_CODE_PHASE_SAMPLES * cn0_scale * coherent_integration_scale);
    let carrier_fallback = ((input.pll_err_rad.abs()) / std::f64::consts::TAU)
        .max(TRACKING_UNCERTAINTY_MIN_CARRIER_PHASE_CYCLES);
    let doppler_fallback = input.fll_err_hz.abs().max(TRACKING_UNCERTAINTY_MIN_DOPPLER_HZ);
    let cn0_fallback =
        (input.cn0_dbhz - input.cn0_reference_dbhz).abs().max(TRACKING_UNCERTAINTY_MIN_CN0_DBHZ);

    TrackingUncertainty {
        code_phase_samples: rms_window(code_error_window_samples)
            .unwrap_or(code_fallback)
            .max(code_fallback)
            * state_scale,
        carrier_phase_cycles: rms_window(carrier_phase_error_window_cycles)
            .unwrap_or(carrier_fallback)
            .max(carrier_fallback)
            * state_scale,
        doppler_hz: rms_window(doppler_error_window_hz)
            .unwrap_or(doppler_fallback)
            .max(doppler_fallback)
            * state_scale,
        cn0_dbhz: stddev_window(cn0_estimate_window_dbhz)
            .unwrap_or(cn0_fallback)
            .max(TRACKING_UNCERTAINTY_MIN_CN0_DBHZ)
            * state_scale,
    }
}

/// Apply the code loop update for one coherent interval.
pub fn apply_code_loop(input: CodeLoopInput) -> CodeLoopUpdate {
    let coefficients = delay_lock_loop_coefficients(input.dll_bw_hz, input.coherent_integration_s);
    let dll_error_chips = input.dll_err as f64;
    let reference_rate_delta_hz =
        input.reference_code_rate_hz - input.previous_reference_code_rate_hz;
    let code_rate_hz = input.current_code_rate_hz + reference_rate_delta_hz
        - coefficients.rate_gain_hz_per_chip * dll_error_chips;
    let predicted_code_phase_samples = predict_code_phase_samples(
        input.current_code_phase_samples,
        input.epoch_len_samples,
        code_rate_hz,
        input.nominal_code_rate_hz,
        input.samples_per_code,
    );
    let code_phase_samples = wrap_code_phase_samples(
        predicted_code_phase_samples
            + coefficients.phase_blend * dll_error_chips * input.samples_per_chip,
        input.samples_per_code,
    );
    CodeLoopUpdate { code_rate_hz, code_phase_samples }
}

/// Apply the carrier PLL/FLL update for one coherent interval.
pub fn apply_carrier_tracking_loop(input: CarrierTrackingLoopInput) -> CarrierTrackingLoopUpdate {
    let pll_coefficients =
        phase_lock_loop_coefficients(input.pll_bw_hz, input.coherent_integration_s);
    let fll_coefficients =
        first_order_angular_loop_coefficients(input.fll_bw_hz, input.coherent_integration_s);
    let coherent_integration_s = input.coherent_integration_s;
    let mut carrier_rate_hz_per_s = input.current_carrier_rate_hz_per_s;
    let mut carrier_hz =
        input.current_carrier_hz + input.current_carrier_rate_hz_per_s * coherent_integration_s;
    if input.apply_fll {
        carrier_hz += bounded_fll_pull_in_correction_hz(
            input.fll_err_hz * fll_coefficients.error_blend,
            input.fll_bw_hz,
        );
    }
    if input.apply_pll_frequency {
        carrier_rate_hz_per_s +=
            pll_coefficients.frequency_rate_gain_hz_per_s_per_rad * input.pll_err_rad;
        carrier_hz += pll_coefficients.frequency_gain_hz_per_rad * input.pll_err_rad;
    }

    let carrier_phase_cycles = input.current_carrier_phase_cycles
        + (input.current_carrier_hz + carrier_hz) * coherent_integration_s * 0.5
        + pll_coefficients.phase_blend * input.pll_err_rad / std::f64::consts::TAU;

    CarrierTrackingLoopUpdate { carrier_hz, carrier_phase_cycles, carrier_rate_hz_per_s }
}

/// Return code chip at a fractional sample index.
pub fn code_at(code: &[i8], samples_per_chip: f64, sample_index: f64) -> Complex<f32> {
    if !samples_per_chip.is_finite() || samples_per_chip <= 0.0 || !sample_index.is_finite() {
        return Complex::new(0.0, 0.0);
    }

    let chip_phase = sample_index / samples_per_chip;
    let chip = code_value_at_phase(code, chip_phase).unwrap_or(0.0);
    Complex::new(chip, 0.0)
}

/// Advance receiver-aligned code phase across one coherent interval.
fn predict_code_phase_samples(
    current_code_phase_samples: f64,
    epoch_len_samples: usize,
    tracked_code_rate_hz: f64,
    nominal_code_rate_hz: f64,
    samples_per_code: usize,
) -> f64 {
    let nominal_code_rate_hz = nominal_code_rate_hz.max(1.0);
    let epoch_start_code_phase_samples = epoch_start_code_phase_samples_from_receiver_phase(
        current_code_phase_samples,
        samples_per_code,
    );
    let code_period_advance_samples =
        epoch_len_samples as f64 * (tracked_code_rate_hz / nominal_code_rate_hz);
    let next_epoch_start_code_phase_samples = wrap_code_phase_samples(
        epoch_start_code_phase_samples + code_period_advance_samples,
        samples_per_code,
    );
    receiver_code_phase_samples_from_epoch_start_phase(
        next_epoch_start_code_phase_samples,
        samples_per_code,
    )
}

/// Limit FLL pull-in correction to a bounded multiple of loop bandwidth.
pub fn bounded_fll_pull_in_correction_hz(fll_err_hz: f64, fll_bw_hz: f64) -> f64 {
    let max_correction_hz = fll_bw_hz.abs().max(1.0) * FLL_PULL_IN_MAX_CORRECTION_BW_MULTIPLIER;
    fll_err_hz.clamp(-max_correction_hz, max_correction_hz)
}

fn rms_window(window: &VecDeque<f64>) -> Option<f64> {
    let count = window.len();
    if count == 0 {
        return None;
    }
    Some((window.iter().map(|value| value * value).sum::<f64>() / count as f64).sqrt())
}

fn stddev_window(window: &VecDeque<f64>) -> Option<f64> {
    let count = window.len();
    if count < 2 {
        return None;
    }
    let mean = window.iter().sum::<f64>() / count as f64;
    Some(
        (window.iter().map(|value| (value - mean).powi(2)).sum::<f64>() / (count - 1) as f64)
            .sqrt(),
    )
}

fn tracking_uncertainty_state_scale(
    quality_class: TrackingQualityClass,
    channel_locked: bool,
    dll_locked: bool,
    anti_false_lock: bool,
    cycle_slip: bool,
) -> f64 {
    let mut scale = match quality_class {
        TrackingQualityClass::Tracking => 1.0,
        TrackingQualityClass::Degraded => 2.0,
        TrackingQualityClass::PullIn => 4.0,
        TrackingQualityClass::Lost => 8.0,
    };
    if !channel_locked {
        scale *= 2.0;
    }
    if !dll_locked {
        scale *= 1.5;
    }
    if anti_false_lock {
        scale *= 2.0;
    }
    if cycle_slip {
        scale *= 4.0;
    }
    scale
}

#[cfg(test)]
#[path = "tracking/tests.rs"]
mod tests;
