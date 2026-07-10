//! Tracking math utilities.

use crate::dsp::signal::code_value_at_phase;
use num_complex::Complex;

/// First-order loop coefficients derived from noise bandwidth and coherent integration time.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FirstOrderLoopCoefficients {
    /// Dimensionless fraction of the discriminator error to blend into the state each update.
    pub error_blend: f64,
    /// Effective gain in Hz applied to one unit of discriminator error.
    pub rate_gain_hz: f64,
}

/// Second-order PLL coefficients derived from noise bandwidth and coherent integration time.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PhaseLockLoopCoefficients {
    /// Dimensionless phase correction applied to the phase accumulator.
    pub phase_blend: f64,
    /// Frequency correction gain in Hz per radian of phase error.
    pub frequency_gain_hz_per_rad: f64,
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

/// Derive first-order loop coefficients from noise bandwidth and coherent integration time.
pub fn first_order_loop_coefficients(
    noise_bandwidth_hz: f64,
    coherent_integration_s: f64,
) -> FirstOrderLoopCoefficients {
    if !noise_bandwidth_hz.is_finite()
        || noise_bandwidth_hz <= 0.0
        || !coherent_integration_s.is_finite()
        || coherent_integration_s <= 0.0
    {
        return FirstOrderLoopCoefficients { error_blend: 0.0, rate_gain_hz: 0.0 };
    }

    let normalized_bandwidth =
        std::f64::consts::TAU * noise_bandwidth_hz * coherent_integration_s;
    let error_blend = 1.0 - (-normalized_bandwidth).exp();
    let rate_gain_hz = error_blend / coherent_integration_s;

    FirstOrderLoopCoefficients { error_blend, rate_gain_hz }
}

/// Derive a second-order PLL filter from loop bandwidth and coherent integration time.
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
        return PhaseLockLoopCoefficients {
            phase_blend: 0.0,
            frequency_gain_hz_per_rad: 0.0,
        };
    }

    let normalized_bandwidth =
        std::f64::consts::TAU * noise_bandwidth_hz * coherent_integration_s;
    let natural_frequency = normalized_bandwidth * (8.0 * PLL_DAMPING_RATIO)
        / (4.0 * PLL_DAMPING_RATIO * PLL_DAMPING_RATIO + 1.0);
    let denominator = 1.0
        + 2.0 * PLL_DAMPING_RATIO * natural_frequency
        + natural_frequency * natural_frequency;
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
    let snr = (signal_power / noise_power).max(1e-12);
    let cn0_linear =
        (snr * noise_weight_energy * sample_rate_hz / prompt_coherent_gain.powi(2)).max(1e-12);
    10.0 * cn0_linear.log10()
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

#[cfg(test)]
mod tests {
    use super::{
        carrier_frequency_error_hz_from_phase_delta, discriminators, estimate_cn0_dbhz,
        first_order_loop_coefficients, phase_lock_loop_coefficients,
    };
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
        let prompt_to_noise_power_ratio =
            expected_cn0_linear * coherent_samples.powi(2) / (noise_weight_energy * sample_rate_hz);
        let prompt = Complex::new(prompt_to_noise_power_ratio.sqrt() as f32, 0.0);
        let noise = Complex::new(1.0, 0.0);

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
}
