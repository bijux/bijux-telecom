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
const ANTI_FALSE_LOCK_MAX_EARLY_LATE_TO_PROMPT_RATIO: f32 = 0.9;
const FLL_PULL_IN_MAX_CORRECTION_BW_MULTIPLIER: f64 = 4.0;
const TRACKING_UNCERTAINTY_MIN_CODE_PHASE_SAMPLES: f64 = 0.01;
const TRACKING_UNCERTAINTY_MIN_CARRIER_PHASE_CYCLES: f64 = 0.001;
const TRACKING_UNCERTAINTY_MIN_DOPPLER_HZ: f64 = 0.01;
const TRACKING_UNCERTAINTY_MIN_CN0_DBHZ: f64 = 0.05;
const LOOP_FILTER_BANDWIDTH_SOLVER_ITERATIONS: usize = 80;
const LOOP_FILTER_IMPULSE_RESPONSE_EPOCHS: usize = 16_384;
const LOOP_FILTER_IMPULSE_RESPONSE_EPSILON: f64 = 1.0e-15;

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

/// Pure DLL update inputs for one coherent tracking interval.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CodeLoopInput {
    /// Current code rate estimate in Hz.
    pub current_code_rate_hz: f64,
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

/// Adaptive loop bandwidth scaling based on CN0.
pub fn adaptive_bandwidth(dll_bw: f64, pll_bw: f64, fll_bw: f64, cn0_dbhz: f64) -> (f64, f64, f64) {
    if cn0_dbhz < 25.0 {
        (dll_bw * 0.5, pll_bw * 0.5, fll_bw * 0.5)
    } else if cn0_dbhz > 40.0 {
        (dll_bw * 1.5, pll_bw * 1.5, fll_bw * 1.5)
    } else {
        (dll_bw, pll_bw, fll_bw)
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
    samples: &[Complex<f32>],
    sample_rate_hz: f64,
    carrier_hz: f64,
    carrier_phase_offset_radians: f64,
    base_chip_phase: f64,
    chips_per_sample: f64,
    early_late_spacing_chips: f64,
    code_value_at_phase: F,
) -> EarlyPromptLateCorrelation
where
    F: Fn(f64) -> f32,
{
    if !sample_rate_hz.is_finite()
        || sample_rate_hz <= 0.0
        || !carrier_hz.is_finite()
        || !carrier_phase_offset_radians.is_finite()
        || !base_chip_phase.is_finite()
        || !chips_per_sample.is_finite()
        || !early_late_spacing_chips.is_finite()
    {
        return EarlyPromptLateCorrelation {
            early: Complex::new(0.0, 0.0),
            prompt: Complex::new(0.0, 0.0),
            late: Complex::new(0.0, 0.0),
            early_late_noise_weight_energy: 0.0,
        };
    }

    let mut carrier_nco = Nco::with_phase(carrier_hz, sample_rate_hz, carrier_phase_offset_radians);
    let mut early = Complex::new(0.0f32, 0.0f32);
    let mut prompt = Complex::new(0.0f32, 0.0f32);
    let mut late = Complex::new(0.0f32, 0.0f32);
    let mut early_late_noise_weight_energy = 0.0f64;

    for (sample_index, sample) in samples.iter().enumerate() {
        let (sin, cos) = carrier_nco.next_sin_cos();
        let mixed_sample = *sample * Complex::new(cos as f32, -sin as f32);
        let chip_phase = base_chip_phase + sample_index as f64 * chips_per_sample;
        let early_code =
            Complex::new(code_value_at_phase(chip_phase - early_late_spacing_chips), 0.0);
        let prompt_code = Complex::new(code_value_at_phase(chip_phase), 0.0);
        let late_code =
            Complex::new(code_value_at_phase(chip_phase + early_late_spacing_chips), 0.0);
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
    for column in 0..dimension {
        let mut projected = 0.0;
        for index in 0..dimension {
            projected +=
                controllability_inverse[dimension - 1][index] * desired_matrix[index][column];
        }
        gain[column] = projected;
    }

    let mut correction_gains = [0.0; 3];
    for index in 0..dimension {
        correction_gains[index] = gain[index];
    }
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
        for row in 0..dimension {
            next_state[row] = correction_gains[row] * input;
            for column in 0..dimension {
                next_state[row] += corrected_transition[row][column] * state[column];
            }
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

    for row in 0..dimension {
        for column in 0..dimension {
            corrected[row][column] =
                transition[row][column] - correction_gains[row] * output[column];
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
        for row in 0..dimension {
            controllability[row][column_index] = column[row];
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
    for row in 0..dimension {
        for column in 0..dimension {
            result[row] += matrix[row][column] * vector[column];
        }
    }
    result
}

fn multiply_square_matrices(
    left: [[f64; 3]; 3],
    right: [[f64; 3]; 3],
    dimension: usize,
) -> [[f64; 3]; 3] {
    let mut result = [[0.0; 3]; 3];
    for row in 0..dimension {
        for column in 0..dimension {
            for index in 0..dimension {
                result[row][column] += left[row][index] * right[index][column];
            }
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
    for row in 0..dimension {
        for column in 0..dimension {
            result[row][column] = left[row][column] + right[row][column];
        }
    }
    result
}

fn scale_square_matrix(matrix: [[f64; 3]; 3], scale: f64, dimension: usize) -> [[f64; 3]; 3] {
    let mut result = [[0.0; 3]; 3];
    for row in 0..dimension {
        for column in 0..dimension {
            result[row][column] = matrix[row][column] * scale;
        }
    }
    result
}

fn transpose_square_matrix(matrix: [[f64; 3]; 3], dimension: usize) -> [[f64; 3]; 3] {
    let mut transpose = [[0.0; 3]; 3];
    for row in 0..dimension {
        for column in 0..dimension {
            transpose[column][row] = matrix[row][column];
        }
    }
    transpose
}

fn identity_square_matrix(dimension: usize) -> [[f64; 3]; 3] {
    let mut identity = [[0.0; 3]; 3];
    for index in 0..dimension {
        identity[index][index] = 1.0;
    }
    identity
}

fn invert_square_matrix(matrix: [[f64; 3]; 3], dimension: usize) -> Option<[[f64; 3]; 3]> {
    let mut augmented = [[0.0; 6]; 3];
    for row in 0..dimension {
        for column in 0..dimension {
            augmented[row][column] = matrix[row][column];
        }
        augmented[row][dimension + row] = 1.0;
    }

    for pivot_index in 0..dimension {
        let mut best_pivot_row = pivot_index;
        let mut best_pivot_abs = augmented[pivot_index][pivot_index].abs();
        for row in (pivot_index + 1)..dimension {
            let pivot_abs = augmented[row][pivot_index].abs();
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
        for column in pivot_index..(dimension * 2) {
            augmented[pivot_index][column] /= pivot;
        }
        for row in 0..dimension {
            if row == pivot_index {
                continue;
            }
            let factor = augmented[row][pivot_index];
            if factor.abs() <= f64::EPSILON {
                continue;
            }
            for column in pivot_index..(dimension * 2) {
                augmented[row][column] -= factor * augmented[pivot_index][column];
            }
        }
    }

    let mut inverse = [[0.0; 3]; 3];
    for row in 0..dimension {
        for column in 0..dimension {
            inverse[row][column] = augmented[row][dimension + column];
        }
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
    const PLL_DAMPING_RATIO: f64 = std::f64::consts::FRAC_1_SQRT_2;

    if !noise_bandwidth_hz.is_finite()
        || noise_bandwidth_hz <= 0.0
        || !coherent_integration_s.is_finite()
        || coherent_integration_s <= 0.0
    {
        return PhaseLockLoopCoefficients { phase_blend: 0.0, frequency_gain_hz_per_rad: 0.0 };
    }

    let normalized_bandwidth = std::f64::consts::TAU * noise_bandwidth_hz * coherent_integration_s;
    let natural_frequency = normalized_bandwidth * (8.0 * PLL_DAMPING_RATIO)
        / (4.0 * PLL_DAMPING_RATIO * PLL_DAMPING_RATIO + 1.0);
    let denominator =
        1.0 + 2.0 * PLL_DAMPING_RATIO * natural_frequency + natural_frequency * natural_frequency;
    let phase_blend = (4.0 * PLL_DAMPING_RATIO * natural_frequency) / denominator;
    let frequency_blend = (4.0 * natural_frequency * natural_frequency) / denominator;
    let frequency_gain_hz_per_rad =
        frequency_blend / (std::f64::consts::TAU * coherent_integration_s);

    PhaseLockLoopCoefficients { phase_blend, frequency_gain_hz_per_rad }
}

/// DLL/PLL/FLL discriminators from early/prompt/late.
pub fn discriminators(
    early: Complex<f32>,
    prompt: Complex<f32>,
    late: Complex<f32>,
    prev_prompt: Option<Complex<f32>>,
) -> (f32, f32, f32, bool) {
    let e = early.norm();
    let l = late.norm();
    let p = prompt.norm();
    let dll = if e + l > 0.0 { (e - l) / (e + l) } else { 0.0 };
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
    let code_rate_hz =
        input.current_code_rate_hz + coefficients.rate_gain_hz_per_chip * dll_error_chips;
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
    let mut carrier_hz = input.current_carrier_hz;
    if input.apply_fll {
        carrier_hz += bounded_fll_pull_in_correction_hz(
            input.fll_err_hz * fll_coefficients.error_blend,
            input.fll_bw_hz,
        );
    }
    if input.apply_pll_frequency {
        carrier_hz += pll_coefficients.frequency_gain_hz_per_rad * input.pll_err_rad;
    }

    let carrier_phase_cycles = input.current_carrier_phase_cycles
        + carrier_hz * input.epoch_len_samples as f64 / input.sample_rate_hz
        + pll_coefficients.phase_blend * input.pll_err_rad / std::f64::consts::TAU;

    CarrierTrackingLoopUpdate { carrier_hz, carrier_phase_cycles }
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
mod tests {
    use super::{
        anti_false_lock_detected, apply_carrier_tracking_loop, apply_code_loop,
        carrier_frequency_error_hz_from_phase_delta, carrier_phase_offset_radians,
        coherent_integration_seconds, correlate_early_prompt_late, delay_lock_loop_coefficients,
        discriminators, dll_lock_threshold, estimate_cn0_dbhz, estimate_tracking_uncertainty,
        first_order_angular_loop_coefficients, first_order_loop_coefficients,
        phase_lock_loop_coefficients, wrap_phase_cycles_signed, wrap_phase_radians_positive,
        wrapped_phase_delta_cycles, CarrierTrackingLoopInput, CodeLoopInput, TrackingQualityClass,
        TrackingUncertaintyInputs,
    };
    use std::collections::VecDeque;

    use num_complex::Complex;

    #[test]
    fn dll_discriminator_is_positive_when_early_energy_exceeds_late_energy() {
        let (dll, pll, fll, lock) = discriminators(
            Complex::new(8.0, 0.0),
            Complex::new(12.0, 0.0),
            Complex::new(4.0, 0.0),
            None,
        );

        assert!(dll > 0.0, "dll={dll}");
        assert_eq!(pll, 0.0);
        assert_eq!(fll, 0.0);
        assert!(lock);
    }

    #[test]
    fn dll_discriminator_is_negative_when_late_energy_exceeds_early_energy() {
        let (dll, pll, fll, lock) = discriminators(
            Complex::new(4.0, 0.0),
            Complex::new(12.0, 0.0),
            Complex::new(8.0, 0.0),
            None,
        );

        assert!(dll < 0.0, "dll={dll}");
        assert_eq!(pll, 0.0);
        assert_eq!(fll, 0.0);
        assert!(lock);
    }

    #[test]
    fn estimate_cn0_dbhz_recovers_known_cn0_from_prompt_and_noise_weighting() {
        let sample_rate_hz = 4_092_000.0_f64;
        let coherent_samples = 4092.0_f64;
        let noise_weight_energy = 8_184.0_f64;
        let expected_cn0_dbhz = 58.0;
        let expected_cn0_linear = 10.0_f64.powf(expected_cn0_dbhz / 10.0);
        let noise = Complex::new(1.0, 0.0);
        let noise_power_per_sample = noise.norm_sqr() as f64 / noise_weight_energy;
        let prompt_noise_power = coherent_samples * noise_power_per_sample;
        let coherent_signal_power =
            expected_cn0_linear * coherent_samples.powi(2) * noise_power_per_sample
                / sample_rate_hz;
        let prompt = Complex::new((coherent_signal_power + prompt_noise_power).sqrt() as f32, 0.0);

        let measured =
            estimate_cn0_dbhz(prompt, noise, sample_rate_hz, coherent_samples, noise_weight_energy);

        assert!((measured - expected_cn0_dbhz).abs() < 1e-6, "measured={measured}");
    }

    #[test]
    fn estimate_cn0_dbhz_returns_zero_for_invalid_estimator_parameters() {
        let prompt = Complex::new(1.0, 0.0);
        let noise = Complex::new(1.0, 0.0);

        assert_eq!(estimate_cn0_dbhz(prompt, noise, 0.0, 4092.0, 8_184.0), 0.0);
        assert_eq!(estimate_cn0_dbhz(prompt, noise, 4_092_000.0, 0.0, 8_184.0), 0.0);
        assert_eq!(estimate_cn0_dbhz(prompt, noise, 4_092_000.0, 4092.0, 0.0), 0.0);
    }

    #[test]
    fn carrier_frequency_error_hz_from_phase_delta_matches_quarter_cycle_over_one_ms() {
        let measured =
            carrier_frequency_error_hz_from_phase_delta(std::f64::consts::FRAC_PI_2, 0.001);

        assert!((measured - 250.0).abs() < 1.0e-9, "measured={measured}");
    }

    #[test]
    fn carrier_frequency_error_hz_from_phase_delta_rejects_invalid_inputs() {
        assert_eq!(carrier_frequency_error_hz_from_phase_delta(f64::NAN, 0.001), 0.0);
        assert_eq!(carrier_frequency_error_hz_from_phase_delta(0.5, 0.0), 0.0);
    }

    #[test]
    fn wrap_phase_cycles_signed_centers_large_offsets_around_zero() {
        assert!((wrap_phase_cycles_signed(0.75) + 0.25).abs() < 1.0e-9);
        assert!((wrap_phase_cycles_signed(-0.75) - 0.25).abs() < 1.0e-9);
    }

    #[test]
    fn wrap_phase_radians_positive_keeps_results_in_positive_turn() {
        let wrapped = wrap_phase_radians_positive(-std::f64::consts::FRAC_PI_2);
        assert!((wrapped - (std::f64::consts::TAU - std::f64::consts::FRAC_PI_2)).abs() < 1.0e-9);
    }

    #[test]
    fn carrier_phase_offset_radians_wraps_whole_cycle_offsets() {
        let wrapped = carrier_phase_offset_radians(1.25);
        assert!((wrapped - std::f64::consts::FRAC_PI_2).abs() < 1.0e-9);
    }

    #[test]
    fn wrapped_phase_delta_cycles_chooses_shortest_signed_delta() {
        assert!((wrapped_phase_delta_cycles(0.05, 0.95) - 0.10).abs() < 1.0e-9);
        assert!((wrapped_phase_delta_cycles(0.95, 0.05) + 0.10).abs() < 1.0e-9);
    }

    #[test]
    fn correlate_early_prompt_late_accumulates_constant_prompt_signal() {
        let samples = vec![Complex::new(1.0, 0.0); 8];

        let correlation =
            correlate_early_prompt_late(&samples, 4_000.0, 0.0, 0.0, 0.0, 0.25, 0.5, |_| 1.0);

        assert_eq!(correlation.early, Complex::new(8.0, 0.0));
        assert_eq!(correlation.prompt, Complex::new(8.0, 0.0));
        assert_eq!(correlation.late, Complex::new(8.0, 0.0));
        assert_eq!(correlation.early_late_noise_weight_energy, 0.0);
    }

    #[test]
    fn correlate_early_prompt_late_wipes_off_known_carrier_rotation() {
        let sample_rate_hz = 4_000.0;
        let carrier_hz = 250.0;
        let samples = (0..8)
            .map(|sample_index| {
                let phase =
                    std::f64::consts::TAU * carrier_hz * sample_index as f64 / sample_rate_hz;
                Complex::new(phase.cos() as f32, phase.sin() as f32)
            })
            .collect::<Vec<_>>();

        let correlation = correlate_early_prompt_late(
            &samples,
            sample_rate_hz,
            carrier_hz,
            0.0,
            0.0,
            0.25,
            0.5,
            |_| 1.0,
        );

        assert!((correlation.prompt.re - 8.0).abs() < 1.0e-5, "{correlation:?}");
        assert!(correlation.prompt.im.abs() < 1.0e-5, "{correlation:?}");
    }

    #[test]
    fn first_order_loop_coefficients_scale_with_bandwidth_and_integration_time() {
        let narrow = first_order_loop_coefficients(2.0, 0.001);
        let wide = first_order_loop_coefficients(10.0, 0.001);
        let long = first_order_loop_coefficients(2.0, 0.020);

        assert!(wide.error_blend > narrow.error_blend, "{wide:?} {narrow:?}");
        assert!(wide.rate_gain_hz > narrow.rate_gain_hz, "{wide:?} {narrow:?}");
        assert!(long.error_blend > narrow.error_blend, "{long:?} {narrow:?}");
        assert!(long.rate_gain_hz < narrow.rate_gain_hz, "{long:?} {narrow:?}");
    }

    #[test]
    fn first_order_loop_design_matches_requested_noise_bandwidth() {
        let coherent_integration_s = 0.001;
        let target_bandwidth_hz = 12.5;
        let design = super::loop_observer_design(
            super::LoopFilterOrder::First,
            target_bandwidth_hz,
            coherent_integration_s,
        );
        let measured_bandwidth_hz = super::loop_noise_bandwidth_hz(
            super::LoopFilterOrder::First,
            coherent_integration_s,
            design.correction_gains,
        );

        assert!(
            (measured_bandwidth_hz - target_bandwidth_hz).abs() <= 1.0e-6,
            "design={design:?} measured_bandwidth_hz={measured_bandwidth_hz}",
        );
    }

    #[test]
    fn first_order_loop_design_places_requested_closed_loop_pole() {
        let coherent_integration_s = 0.001;
        let design =
            super::loop_observer_design(super::LoopFilterOrder::First, 8.0, coherent_integration_s);
        let transition = super::corrected_loop_transition(
            super::LoopFilterOrder::First,
            coherent_integration_s,
            design.correction_gains,
        );

        assert!(
            (transition[0][0] - design.closed_loop_pole).abs() <= 1.0e-12,
            "design={design:?} transition={transition:?}",
        );
    }

    #[test]
    fn second_order_loop_design_matches_requested_noise_bandwidth() {
        let coherent_integration_s = 0.001;
        let target_bandwidth_hz = 6.0;
        let design = super::loop_observer_design(
            super::LoopFilterOrder::Second,
            target_bandwidth_hz,
            coherent_integration_s,
        );
        let measured_bandwidth_hz = super::loop_noise_bandwidth_hz(
            super::LoopFilterOrder::Second,
            coherent_integration_s,
            design.correction_gains,
        );

        assert!(
            (measured_bandwidth_hz - target_bandwidth_hz).abs() <= 1.0e-6,
            "design={design:?} measured_bandwidth_hz={measured_bandwidth_hz}",
        );
    }

    #[test]
    fn second_order_loop_design_repeats_the_requested_pole() {
        let coherent_integration_s = 0.001;
        let design = super::loop_observer_design(
            super::LoopFilterOrder::Second,
            6.0,
            coherent_integration_s,
        );
        let transition = super::corrected_loop_transition(
            super::LoopFilterOrder::Second,
            coherent_integration_s,
            design.correction_gains,
        );
        let trace = transition[0][0] + transition[1][1];
        let determinant = transition[0][0] * transition[1][1] - transition[0][1] * transition[1][0];

        assert!((trace - 2.0 * design.closed_loop_pole).abs() <= 1.0e-9, "{transition:?}");
        assert!(
            (determinant - design.closed_loop_pole * design.closed_loop_pole).abs() <= 1.0e-9,
            "{transition:?}",
        );
    }

    #[test]
    fn third_order_loop_design_matches_requested_noise_bandwidth() {
        let coherent_integration_s = 0.001;
        let target_bandwidth_hz = 18.0;
        let design = super::loop_observer_design(
            super::LoopFilterOrder::Third,
            target_bandwidth_hz,
            coherent_integration_s,
        );
        let measured_bandwidth_hz = super::loop_noise_bandwidth_hz(
            super::LoopFilterOrder::Third,
            coherent_integration_s,
            design.correction_gains,
        );

        assert!(
            (measured_bandwidth_hz - target_bandwidth_hz).abs() <= 1.0e-6,
            "design={design:?} measured_bandwidth_hz={measured_bandwidth_hz}",
        );
    }

    #[test]
    fn third_order_loop_design_repeats_the_requested_pole() {
        let coherent_integration_s = 0.001;
        let design = super::loop_observer_design(
            super::LoopFilterOrder::Third,
            18.0,
            coherent_integration_s,
        );
        let transition = super::corrected_loop_transition(
            super::LoopFilterOrder::Third,
            coherent_integration_s,
            design.correction_gains,
        );
        let trace = transition[0][0] + transition[1][1] + transition[2][2];
        let principal_minors = transition[0][0] * transition[1][1]
            + transition[0][0] * transition[2][2]
            + transition[1][1] * transition[2][2]
            - transition[0][1] * transition[1][0]
            - transition[0][2] * transition[2][0]
            - transition[1][2] * transition[2][1];
        let determinant = transition[0][0]
            * (transition[1][1] * transition[2][2] - transition[1][2] * transition[2][1])
            - transition[0][1]
                * (transition[1][0] * transition[2][2] - transition[1][2] * transition[2][0])
            + transition[0][2]
                * (transition[1][0] * transition[2][1] - transition[1][1] * transition[2][0]);

        assert!((trace - 3.0 * design.closed_loop_pole).abs() <= 1.0e-9, "{transition:?}");
        assert!(
            (principal_minors - 3.0 * design.closed_loop_pole * design.closed_loop_pole).abs()
                <= 1.0e-9,
            "{transition:?}",
        );
        assert!((determinant - design.closed_loop_pole.powi(3)).abs() <= 1.0e-9, "{transition:?}",);
    }

    #[test]
    fn phase_lock_loop_coefficients_are_finite_and_stronger_for_wider_bandwidths() {
        let narrow = phase_lock_loop_coefficients(5.0, 0.001);
        let wide = phase_lock_loop_coefficients(15.0, 0.001);

        assert!(narrow.phase_blend.is_finite());
        assert!(narrow.frequency_gain_hz_per_rad.is_finite());
        assert!(wide.phase_blend > narrow.phase_blend, "{wide:?} {narrow:?}");
        assert!(
            wide.frequency_gain_hz_per_rad > narrow.frequency_gain_hz_per_rad,
            "{wide:?} {narrow:?}"
        );
    }

    #[test]
    fn angular_first_order_loop_coefficients_strengthen_frequency_response() {
        let linear = first_order_loop_coefficients(10.0, 0.001);
        let angular = first_order_angular_loop_coefficients(10.0, 0.001);

        assert_eq!(angular, linear);
    }

    #[test]
    fn delay_lock_loop_coefficients_are_finite_and_stronger_for_wider_bandwidths() {
        let narrow = delay_lock_loop_coefficients(2.0, 0.001);
        let wide = delay_lock_loop_coefficients(8.0, 0.001);

        assert!(narrow.phase_blend.is_finite());
        assert!(narrow.rate_gain_hz_per_chip.is_finite());
        assert!(wide.phase_blend > narrow.phase_blend, "{wide:?} {narrow:?}");
        assert!(wide.rate_gain_hz_per_chip > narrow.rate_gain_hz_per_chip, "{wide:?} {narrow:?}");
    }

    #[test]
    fn dll_lock_threshold_relaxes_for_subsample_early_late_spacing() {
        assert_eq!(dll_lock_threshold(1.0, 0.5), 0.6);
        assert_eq!(dll_lock_threshold(4.0, 0.5), 0.2);
    }

    #[test]
    fn apply_code_loop_updates_code_rate_from_discriminator() {
        let coherent_integration_s = coherent_integration_seconds(5_000, 1_023_000.0);
        let update = apply_code_loop(CodeLoopInput {
            current_code_rate_hz: 1_023_000.0,
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
            + delay_lock_loop_coefficients(2.0, coherent_integration_s).rate_gain_hz_per_chip
                * 0.25;
        assert!((update.code_rate_hz - expected).abs() < 1.0e-9, "{update:?}");
    }

    #[test]
    fn delay_lock_loop_step_response_reduces_prompt_misalignment() {
        let coherent_integration_s = 0.001;
        let input = CodeLoopInput {
            current_code_rate_hz: 1_023_000.0,
            current_code_phase_samples: 250.0,
            epoch_len_samples: 4_092,
            coherent_integration_s,
            nominal_code_rate_hz: 1_023_000.0,
            dll_bw_hz: 4.0,
            dll_err: 0.25,
            samples_per_chip: 4.0,
            samples_per_code: 4_092,
        };
        let coefficients = delay_lock_loop_coefficients(input.dll_bw_hz, coherent_integration_s);
        let predicted = super::predict_code_phase_samples(
            input.current_code_phase_samples,
            input.epoch_len_samples,
            input.current_code_rate_hz,
            input.nominal_code_rate_hz,
            input.samples_per_code,
        );
        let corrected = apply_code_loop(input);

        assert!(
            corrected.code_phase_samples > predicted,
            "coefficients={coefficients:?} corrected={corrected:?} predicted={predicted}",
        );
    }

    #[test]
    fn apply_carrier_tracking_loop_advances_phase_and_frequency_from_pll_error() {
        let update = apply_carrier_tracking_loop(CarrierTrackingLoopInput {
            current_carrier_hz: 1_000.0,
            current_carrier_phase_cycles: 12.0,
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
            (update.carrier_hz - (1_000.0 + pll_coefficients.frequency_gain_hz_per_rad * 0.25))
                .abs()
                < 1.0e-9,
            "{update:?}"
        );
        let expected_phase_cycles = 12.0
            + (1_000.0 + pll_coefficients.frequency_gain_hz_per_rad * 0.25) * 0.001
            + pll_coefficients.phase_blend * 0.25 / std::f64::consts::TAU;
        assert!((update.carrier_phase_cycles - expected_phase_cycles).abs() < 1.0e-9, "{update:?}");
    }

    #[test]
    fn anti_false_lock_detected_rejects_early_late_energy_near_prompt() {
        assert!(anti_false_lock_detected(
            Complex::new(8.0, 0.0),
            Complex::new(8.0, 0.0),
            Complex::new(8.0, 0.0),
        ));
        assert!(!anti_false_lock_detected(
            Complex::new(1.0, 0.0),
            Complex::new(8.0, 0.0),
            Complex::new(1.0, 0.0),
        ));
    }

    #[test]
    fn estimate_tracking_uncertainty_rewards_longer_coherent_integration() {
        let empty = VecDeque::<f64>::new();
        let short = estimate_tracking_uncertainty(
            &empty,
            &empty,
            &empty,
            &empty,
            TrackingUncertaintyInputs {
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
                quality_class: TrackingQualityClass::Tracking,
            },
        );
        let long = estimate_tracking_uncertainty(
            &empty,
            &empty,
            &empty,
            &empty,
            TrackingUncertaintyInputs {
                integration_ms: 10,
                ..TrackingUncertaintyInputs {
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
                    quality_class: TrackingQualityClass::Tracking,
                }
            },
        );

        assert!(
            long.code_phase_samples < short.code_phase_samples,
            "short={short:?} long={long:?}"
        );
    }
}
