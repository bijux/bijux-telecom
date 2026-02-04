#![allow(missing_docs)]

use num_complex::Complex;

use crate::stages::tracking::CorrelatorOutput;

pub(crate) fn adaptive_bandwidth(
    dll_bw: f64,
    pll_bw: f64,
    fll_bw: f64,
    cn0_dbhz: f64,
) -> (f64, f64, f64) {
    if cn0_dbhz < 25.0 {
        (dll_bw * 0.5, pll_bw * 0.5, fll_bw * 0.5)
    } else if cn0_dbhz > 40.0 {
        (dll_bw * 1.5, pll_bw * 1.5, fll_bw * 1.5)
    } else {
        (dll_bw, pll_bw, fll_bw)
    }
}

pub(crate) fn discriminators(
    corr: &CorrelatorOutput,
    prev_prompt: Option<Complex<f32>>,
) -> (f32, f32, f32, bool) {
    let e = corr.early.norm();
    let l = corr.late.norm();
    let p = corr.prompt.norm();
    let dll = if e + l > 0.0 { (e - l) / (e + l) } else { 0.0 };
    let pll = corr.prompt.im.atan2(corr.prompt.re);
    let fll = if let Some(prev) = prev_prompt {
        let dot = corr.prompt.re * prev.re + corr.prompt.im * prev.im;
        let det = corr.prompt.im * prev.re - corr.prompt.re * prev.im;
        det.atan2(dot)
    } else {
        0.0
    };
    let lock = p > 0.1 && dll.abs() < 0.5;
    (dll, pll, fll, lock)
}

pub(crate) fn estimate_cn0_dbhz(prompt: Complex<f32>, noise: Complex<f32>) -> f64 {
    let signal_power = (prompt.norm_sqr() as f64).max(1e-12);
    let noise_power = (noise.norm_sqr() as f64).max(1e-12);
    let snr = (signal_power / noise_power).max(1e-12);
    10.0 * snr.log10() + 30.0
}

pub(crate) fn code_at(code: &[i8], samples_per_chip: f64, sample_index: f64) -> Complex<f32> {
    let code_len_samples = samples_per_chip * code.len() as f64;
    let wrapped = sample_index.rem_euclid(code_len_samples);
    let chip_index = (wrapped / samples_per_chip).floor() as usize;
    let chip = code[chip_index] as f32;
    Complex::new(chip, 0.0)
}
