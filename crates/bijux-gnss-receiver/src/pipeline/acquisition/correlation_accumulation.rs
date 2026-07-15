use bijux_gnss_core::api::{AcqComponentCombinationMode, SamplesFrame};
use num_complex::Complex;
use rustfft::num_traits::Zero;

use crate::pipeline::acquisition_components::AcquisitionComponentPlan;
use crate::pipeline::acquisition_symbol_hypotheses::coherent_secondary_code_phase_hypotheses;

use super::code_phase_profile::wipeoff_search_carrier;
use super::peak_metrics::{correlation_metrics, CorrelationMetrics};
use super::strategy_components::component_data_sign_hypotheses;

#[derive(Debug, Clone)]
pub(super) struct ComponentCorrelationAccumulation {
    pub(super) per_noncoherent: Vec<Vec<Complex<f32>>>,
    pub(super) noncoherent_accumulator: Vec<f32>,
    pub(super) secondary_code_phase_periods: Option<u32>,
}

pub(super) fn combine_component_accumulations(
    combination_mode: AcqComponentCombinationMode,
    component_indexes: &[usize],
    component_accumulations: &[ComponentCorrelationAccumulation],
    samples_per_code: usize,
    noncoherent: u32,
) -> Vec<f32> {
    match combination_mode {
        AcqComponentCombinationMode::SingleComponent => {
            component_accumulations[component_indexes[0]].noncoherent_accumulator.clone()
        }
        AcqComponentCombinationMode::NoncoherentComponentSum => {
            let mut combined = vec![0.0f32; samples_per_code];
            for &component_index in component_indexes {
                for (combined_value, component_value) in combined
                    .iter_mut()
                    .zip(component_accumulations[component_index].noncoherent_accumulator.iter())
                {
                    *combined_value += *component_value;
                }
            }
            combined
        }
        AcqComponentCombinationMode::CoherentComponentSum => {
            let mut combined = vec![0.0f32; samples_per_code];
            for nc in 0..noncoherent as usize {
                for sample_index in 0..samples_per_code {
                    let mut coherent_sum: Complex<f32> = Complex::zero();
                    for &component_index in component_indexes {
                        coherent_sum += component_accumulations[component_index].per_noncoherent
                            [nc][sample_index];
                    }
                    combined[sample_index] += coherent_sum.norm();
                }
            }
            combined
        }
    }
}

pub(super) fn accumulate_component_correlations(
    frame: &SamplesFrame,
    component: &AcquisitionComponentPlan,
    code_fft: &[Complex<f32>],
    carrier_hz: f64,
    doppler_rate_hz_per_s: f64,
    sample_rate_hz: f64,
    samples_per_code: usize,
    coherent_periods: u32,
    noncoherent: u32,
    fft: &dyn rustfft::Fft<f32>,
    ifft: &dyn rustfft::Fft<f32>,
) -> ComponentCorrelationAccumulation {
    let total_periods = coherent_periods as usize * noncoherent as usize;
    let mut per_period = Vec::with_capacity(total_periods);
    for offset_period in 0..total_periods {
        let start = offset_period * samples_per_code;
        let end = start + samples_per_code;
        let block = &frame.iq[start..end];

        let mixed = wipeoff_search_carrier(
            block,
            carrier_hz,
            doppler_rate_hz_per_s,
            sample_rate_hz,
            start as u64,
            0.0,
        )
        .expect("acquisition carrier wipeoff requires finite carrier inputs");

        let mut input_fft = mixed;
        fft.process(&mut input_fft);

        let mut prod = vec![Complex::zero(); samples_per_code];
        for i in 0..samples_per_code {
            prod[i] = input_fft[i] * code_fft[i].conj();
        }

        ifft.process(&mut prod);
        per_period.push(prod);
    }

    if let Some(secondary_code_period_signs) = component.secondary_code_period_signs() {
        return best_coherent_secondary_code_phase_correlation(
            &per_period,
            coherent_periods as usize,
            secondary_code_period_signs,
        );
    }

    let mut noncoherent_accumulator = vec![0.0f32; samples_per_code];
    let mut per_noncoherent = Vec::with_capacity(noncoherent as usize);
    let coherent_sign_hypotheses =
        component_data_sign_hypotheses(component, coherent_periods).unwrap_or_default();

    for nc in 0..noncoherent as usize {
        let start = nc * coherent_periods as usize;
        let end = start + coherent_periods as usize;
        let coherent_correlation = if coherent_sign_hypotheses.is_empty() {
            coherent_correlation_with_signs(&per_period[start..end], &vec![1; end - start])
        } else {
            best_coherent_data_correlation(&per_period[start..end], &coherent_sign_hypotheses)
        };
        for (accumulator, sample) in
            noncoherent_accumulator.iter_mut().zip(coherent_correlation.iter())
        {
            *accumulator += sample.norm();
        }
        per_noncoherent.push(coherent_correlation);
    }

    ComponentCorrelationAccumulation {
        per_noncoherent,
        noncoherent_accumulator,
        secondary_code_phase_periods: None,
    }
}

pub(super) fn best_coherent_data_correlation(
    per_period: &[Vec<Complex<f32>>],
    sign_hypotheses: &[Vec<i8>],
) -> Vec<Complex<f32>> {
    let samples_per_code = per_period.first().map_or(0, Vec::len);
    let mut best = vec![Complex::zero(); samples_per_code];
    let mut best_norms = vec![0.0f32; samples_per_code];

    for signs in sign_hypotheses {
        let candidate = coherent_correlation_with_signs(per_period, signs);
        for sample_index in 0..samples_per_code {
            let candidate_norm = candidate[sample_index].norm_sqr();
            if candidate_norm > best_norms[sample_index] {
                best_norms[sample_index] = candidate_norm;
                best[sample_index] = candidate[sample_index];
            }
        }
    }

    best
}

pub(super) fn best_coherent_secondary_code_phase_correlation(
    per_period: &[Vec<Complex<f32>>],
    coherent_periods: usize,
    secondary_code_period_signs: &[i8],
) -> ComponentCorrelationAccumulation {
    let samples_per_code = per_period.first().map_or(0, Vec::len);
    let noncoherent_periods = per_period.len() / coherent_periods.max(1);
    let hypotheses =
        coherent_secondary_code_phase_hypotheses(per_period.len(), secondary_code_period_signs);
    let mut best_accumulation = None;
    let mut best_metrics: Option<(CorrelationMetrics, u32)> = None;

    for hypothesis in hypotheses {
        let mut per_noncoherent = Vec::with_capacity(noncoherent_periods);
        let mut noncoherent_accumulator = vec![0.0f32; samples_per_code];

        for nc in 0..noncoherent_periods {
            let start = nc * coherent_periods;
            let end = start + coherent_periods;
            let coherent_correlation = coherent_correlation_with_signs(
                &per_period[start..end],
                &hypothesis.period_signs[start..end],
            );
            for (accumulator, sample) in
                noncoherent_accumulator.iter_mut().zip(coherent_correlation.iter())
            {
                *accumulator += sample.norm();
            }
            per_noncoherent.push(coherent_correlation);
        }

        let metrics = correlation_metrics(&noncoherent_accumulator);
        let candidate_peak_mean_ratio = metrics.peak / (metrics.mean + 1e-6);
        let candidate_peak_second_ratio = metrics.peak / (metrics.second + 1e-6);
        let replace = best_metrics.is_none_or(|(best, best_phase)| {
            let best_peak_mean_ratio = best.peak / (best.mean + 1e-6);
            let best_peak_second_ratio = best.peak / (best.second + 1e-6);
            candidate_peak_mean_ratio > best_peak_mean_ratio + f32::EPSILON
                || ((candidate_peak_mean_ratio - best_peak_mean_ratio).abs() <= f32::EPSILON
                    && (candidate_peak_second_ratio > best_peak_second_ratio + f32::EPSILON
                        || ((candidate_peak_second_ratio - best_peak_second_ratio).abs()
                            <= f32::EPSILON
                            && hypothesis.secondary_code_phase_periods < best_phase)))
        });
        if replace {
            best_metrics = Some((metrics, hypothesis.secondary_code_phase_periods));
            best_accumulation = Some(ComponentCorrelationAccumulation {
                per_noncoherent,
                noncoherent_accumulator,
                secondary_code_phase_periods: Some(hypothesis.secondary_code_phase_periods),
            });
        }
    }

    best_accumulation.expect("secondary-code phase search must yield at least one hypothesis")
}

pub(super) fn coherent_correlation_with_signs(
    per_period: &[Vec<Complex<f32>>],
    signs: &[i8],
) -> Vec<Complex<f32>> {
    let samples_per_code = per_period.first().map_or(0, Vec::len);
    let mut coherent = vec![Complex::zero(); samples_per_code];

    for (period_index, period) in per_period.iter().enumerate() {
        let sign = signs.get(period_index).copied().unwrap_or(1) as f32;
        for sample_index in 0..samples_per_code {
            coherent[sample_index] += period[sample_index] * sign;
        }
    }

    coherent
}
