use std::collections::HashMap;
use std::sync::Mutex;

use bijux_gnss_core::{AcqResult, SamplesFrame, SatId};
use num_complex::Complex;
use rustfft::{num_traits::Zero, FftPlanner};

use crate::ca_code::{generate_ca_code, Prn};
use crate::logging;
use crate::signal::samples_per_code;
#[cfg(feature = "trace-dump")]
use crate::trace_dump::{dump_acq_trace, AcqTrace};
use crate::types::ReceiverConfig;
use bijux_gnss_signal::Nco;

/// Acquisition engine (coarse search).
pub struct Acquisition {
    config: ReceiverConfig,
    doppler_search_hz: i32,
    doppler_step_hz: i32,
    cache: Mutex<CodeFftCache>,
}

type CodeFftCache = HashMap<(usize, SatId), Vec<Complex<f32>>>;

impl Acquisition {
    pub fn new(config: ReceiverConfig) -> Self {
        Self {
            config,
            doppler_search_hz: 10_000,
            doppler_step_hz: 500,
            cache: Mutex::new(HashMap::new()),
        }
    }

    pub fn with_doppler(mut self, search_hz: i32, step_hz: i32) -> Self {
        self.doppler_search_hz = search_hz;
        self.doppler_step_hz = step_hz.max(1);
        self
    }

    /// Perform satellite acquisition on a 1 ms buffer using FFT-based circular correlation.
    pub fn run_fft(&self, frame: &SamplesFrame, sats: &[SatId]) -> Vec<AcqResult> {
        self.run_fft_topn(frame, sats, 1, 1, 1)
            .into_iter()
            .map(|mut v| v.remove(0))
            .collect()
    }

    pub fn run_fft_topn(
        &self,
        frame: &SamplesFrame,
        sats: &[SatId],
        top_n: usize,
        coherent_ms: u32,
        noncoherent: u32,
    ) -> Vec<Vec<AcqResult>> {
        let samples_per_code = samples_per_code(
            self.config.sampling_freq_hz,
            self.config.code_freq_basis_hz,
            self.config.code_length,
        );
        let total_ms = (coherent_ms * noncoherent).max(1) as usize;
        let required = samples_per_code * total_ms;
        if frame.len() < required {
            return Vec::new();
        }

        let mut planner = FftPlanner::<f32>::new();
        let fft = planner.plan_fft_forward(samples_per_code);
        let ifft = planner.plan_fft_inverse(samples_per_code);

        let mut results = Vec::new();
        for &sat in sats {
            let code_fft = self.code_fft(sat, samples_per_code, fft.as_ref());
            let mut candidates = Vec::new();

            let mut doppler = -self.doppler_search_hz;
            while doppler <= self.doppler_search_hz {
                let carrier = self.config.intermediate_freq_hz + doppler as f64;
                let mut noncoherent_acc = vec![0.0f32; samples_per_code];

                for nc in 0..noncoherent {
                    let mut coherent_corr: Vec<Complex<f32>> =
                        vec![Complex::zero(); samples_per_code];
                    for c in 0..coherent_ms {
                        let offset_ms = (nc * coherent_ms + c) as usize;
                        let start = offset_ms * samples_per_code;
                        let end = start + samples_per_code;
                        let block = &frame.iq[start..end];

                        let mut mixed = vec![Complex::zero(); samples_per_code];
                        let mut nco = Nco::new(-carrier, self.config.sampling_freq_hz);
                        for (i, sample) in block.iter().enumerate() {
                            let (sin, cos) = nco.next_sin_cos();
                            let rot = Complex::new(cos as f32, -sin as f32);
                            mixed[i] = *sample * rot;
                        }

                        let mut input_fft = mixed;
                        fft.process(&mut input_fft);

                        let mut prod = vec![Complex::zero(); samples_per_code];
                        for i in 0..samples_per_code {
                            prod[i] = input_fft[i] * code_fft[i].conj();
                        }

                        ifft.process(&mut prod);
                        for i in 0..samples_per_code {
                            coherent_corr[i] += prod[i];
                        }
                    }

                    for i in 0..samples_per_code {
                        noncoherent_acc[i] += coherent_corr[i].norm();
                    }
                }

                let (peak_idx, peak, second, mean) = correlation_metrics(&noncoherent_acc);
                let peak_mean_ratio = peak / (mean + 1e-6);
                let peak_second_ratio = peak / (second + 1e-6);
                let cn0_proxy = peak_mean_ratio * 10.0;

                candidates.push(AcqResult {
                    sat,
                    carrier_hz: carrier,
                    code_phase_samples: peak_idx,
                    peak,
                    second_peak: second,
                    mean,
                    peak_mean_ratio,
                    peak_second_ratio,
                    cn0_proxy,
                });

                doppler += self.doppler_step_hz;
            }

            candidates.sort_by(|a, b| {
                b.peak_mean_ratio
                    .partial_cmp(&a.peak_mean_ratio)
                    .unwrap_or(std::cmp::Ordering::Equal)
            });
            candidates.truncate(top_n.max(1));

            if let Some(best) = candidates.first() {
                logging::acquisition_hit(
                    best.sat,
                    best.carrier_hz,
                    best.code_phase_samples,
                    best.peak,
                    best.peak_mean_ratio,
                );
                #[cfg(feature = "trace-dump")]
                if let Ok(dir) = std::env::var("BIJUX_TRACE_DIR") {
                    let trace = AcqTrace {
                        sat: best.sat,
                        doppler_hz: best.carrier_hz,
                        code_phase_samples: best.code_phase_samples,
                        peak: best.peak,
                        mean: best.mean,
                        second_peak: best.second_peak,
                    };
                    let _ = dump_acq_trace(std::path::Path::new(&dir), &trace);
                }
            }

            results.push(candidates);
        }

        results
    }

    fn code_fft(
        &self,
        sat: SatId,
        samples_per_code: usize,
        fft: &dyn rustfft::Fft<f32>,
    ) -> Vec<Complex<f32>> {
        let key = (samples_per_code, sat);
        if let Some(cached) = self.cache.lock().ok().and_then(|m| m.get(&key).cloned()) {
            return cached;
        }
        let code = generate_ca_code(Prn(sat.prn));
        let local_code = upsample_code(&code, samples_per_code);
        let mut code_fft: Vec<Complex<f32>> =
            local_code.iter().map(|&x| Complex::new(x, 0.0)).collect();
        fft.process(&mut code_fft);
        if let Ok(mut cache) = self.cache.lock() {
            cache.insert(key, code_fft.clone());
        }
        code_fft
    }
}

fn correlation_metrics(corr: &[f32]) -> (usize, f32, f32, f32) {
    let mut peak_idx = 0;
    let mut peak = 0.0f32;
    let mut second = 0.0f32;
    let mut sum = 0.0f32;

    for (idx, &mag) in corr.iter().enumerate() {
        sum += mag;
        if mag > peak {
            second = peak;
            peak = mag;
            peak_idx = idx;
        } else if mag > second {
            second = mag;
        }
    }

    let mean = sum / corr.len().max(1) as f32;
    (peak_idx, peak, second, mean)
}

fn upsample_code(code: &[i8], samples_per_code: usize) -> Vec<f32> {
    let chips = code.len();
    let samples_per_chip = samples_per_code as f64 / chips as f64;
    let mut out = vec![0.0f32; samples_per_code];
    for (i, value) in out.iter_mut().enumerate() {
        let chip_index = (i as f64 / samples_per_chip).floor() as usize;
        *value = code[chip_index] as f32;
    }
    out
}
