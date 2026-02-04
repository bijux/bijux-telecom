//! Tracking math utilities.

use num_complex::Complex;

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

/// Estimate CN0 in dB-Hz from prompt and noise.
pub fn estimate_cn0_dbhz(prompt: Complex<f32>, noise: Complex<f32>) -> f64 {
    let signal_power = (prompt.norm_sqr() as f64).max(1e-12);
    let noise_power = (noise.norm_sqr() as f64).max(1e-12);
    let snr = (signal_power / noise_power).max(1e-12);
    10.0 * snr.log10() + 30.0
}

/// Return code chip at a fractional sample index.
pub fn code_at(code: &[i8], samples_per_chip: f64, sample_index: f64) -> Complex<f32> {
    let code_len_samples = samples_per_chip * code.len() as f64;
    let wrapped = sample_index.rem_euclid(code_len_samples);
    let chip_index = (wrapped / samples_per_chip).floor() as usize;
    let chip = code[chip_index] as f32;
    Complex::new(chip, 0.0)
}
