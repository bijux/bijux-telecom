use crate::error::SignalError;
use bijux_gnss_core::api::{SignalComponentSpec, SignalRegistryEntry, SignalSubcarrierSpec};
use num_complex::Complex;
use rustfft::FftPlanner;

const MIN_SPECTRUM_SEGMENT_LEN: usize = 16;
const MIN_POWER_DENSITY: f64 = 1.0e-18;

/// One point of a two-sided baseband power spectral density.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PowerSpectralDensityPoint {
    /// Baseband frequency of the point in Hz.
    pub frequency_hz: f64,
    /// Power spectral density at the point in power-per-Hz units.
    pub power_density: f64,
}

/// Summary metrics derived from a two-sided baseband power spectral density.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PowerSpectralDensitySummary {
    /// Power-weighted spectral centroid in Hz.
    pub center_frequency_hz: f64,
    /// Symmetric occupied bandwidth in Hz containing the requested power fraction.
    pub occupied_bandwidth_hz: f64,
    /// Integrated two-sided power across the sampled frequency grid.
    pub integrated_power: f64,
    /// Normalized symmetry error across mirrored positive and negative bins.
    pub symmetry_error: f64,
}

/// One detected deep spectral null.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SpectrumNull {
    /// Null frequency in Hz.
    pub frequency_hz: f64,
    /// Power density measured at the null.
    pub power_density: f64,
}

/// Welch estimator configuration for generated-signal PSD measurements.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SpectrumEstimatorConfig {
    /// FFT segment length used per Welch window.
    pub segment_len: usize,
    /// Overlap between consecutive Welch windows.
    pub overlap_len: usize,
}

impl Default for SpectrumEstimatorConfig {
    fn default() -> Self {
        Self { segment_len: 4096, overlap_len: 2048 }
    }
}

impl SpectrumEstimatorConfig {
    /// Validate the estimator configuration.
    pub fn validate(&self) -> Result<(), SignalError> {
        if self.segment_len < MIN_SPECTRUM_SEGMENT_LEN {
            return Err(SignalError::InvalidSpectrumAnalysis {
                message: format!(
                    "segment_len must be >= {MIN_SPECTRUM_SEGMENT_LEN}, got {}",
                    self.segment_len
                ),
            });
        }
        if !self.segment_len.is_power_of_two() {
            return Err(SignalError::InvalidSpectrumAnalysis {
                message: format!("segment_len must be a power of two, got {}", self.segment_len),
            });
        }
        if self.overlap_len >= self.segment_len {
            return Err(SignalError::InvalidSpectrumAnalysis {
                message: format!(
                    "overlap_len must be smaller than segment_len, got {} >= {}",
                    self.overlap_len, self.segment_len
                ),
            });
        }
        Ok(())
    }
}

/// Compute the expected two-sided baseband PSD for one signal component.
pub fn expected_component_power_spectral_density(
    component: SignalComponentSpec,
    frequencies_hz: &[f64],
) -> Result<Vec<PowerSpectralDensityPoint>, SignalError> {
    validate_frequency_grid(frequencies_hz)?;
    validate_component(component)?;

    let chip_period_s = 1.0 / component.primary_code_rate_hz;
    let segments = normalized_chip_segments(component.subcarrier)?;
    Ok(frequencies_hz
        .iter()
        .map(|&frequency_hz| PowerSpectralDensityPoint {
            frequency_hz,
            power_density: chip_period_power_spectral_density(
                &segments,
                chip_period_s,
                frequency_hz,
            ),
        })
        .collect())
}

/// Compute the expected two-sided baseband PSD for a full registered signal.
pub fn expected_signal_power_spectral_density(
    entry: &SignalRegistryEntry,
    frequencies_hz: &[f64],
) -> Result<Vec<PowerSpectralDensityPoint>, SignalError> {
    validate_frequency_grid(frequencies_hz)?;
    if entry.components.is_empty() {
        return Err(SignalError::InvalidSpectrumAnalysis {
            message: "cannot compute expected spectrum for a signal without components".to_string(),
        });
    }

    let mut total = vec![
        PowerSpectralDensityPoint { frequency_hz: 0.0, power_density: 0.0 };
        frequencies_hz.len()
    ];
    for component in &entry.components {
        let component_spectrum =
            expected_component_power_spectral_density(*component, frequencies_hz)?;
        for (total_point, component_point) in total.iter_mut().zip(component_spectrum.iter()) {
            total_point.frequency_hz = component_point.frequency_hz;
            total_point.power_density +=
                component_point.power_density * f64::from(component.power_fraction);
        }
    }
    Ok(total)
}

/// Estimate a two-sided baseband PSD from generated samples using Welch averaging.
pub fn estimate_power_spectral_density(
    samples: &[f32],
    sample_rate_hz: f64,
    config: SpectrumEstimatorConfig,
) -> Result<Vec<PowerSpectralDensityPoint>, SignalError> {
    validate_sample_rate(sample_rate_hz)?;
    config.validate()?;
    if samples.len() < config.segment_len {
        return Err(SignalError::InvalidSpectrumAnalysis {
            message: format!(
                "sample slice length {} must be >= segment_len {}",
                samples.len(),
                config.segment_len
            ),
        });
    }

    let step = config.segment_len - config.overlap_len;
    let window = hann_window(config.segment_len);
    let window_power = window.iter().map(|value| value * value).sum::<f64>();
    let mut planner = FftPlanner::<f64>::new();
    let fft = planner.plan_fft_forward(config.segment_len);
    let mut averaged = vec![0.0_f64; config.segment_len];
    let mut segment_count = 0usize;

    for start in (0..=samples.len() - config.segment_len).step_by(step) {
        let mut spectrum = samples[start..start + config.segment_len]
            .iter()
            .zip(window.iter())
            .map(|(sample, weight)| Complex::new(f64::from(*sample) * *weight, 0.0))
            .collect::<Vec<_>>();
        fft.process(&mut spectrum);
        for (bin, value) in averaged.iter_mut().zip(spectrum.iter()) {
            *bin += value.norm_sqr() / (sample_rate_hz * window_power);
        }
        segment_count += 1;
    }

    if segment_count == 0 {
        return Err(SignalError::InvalidSpectrumAnalysis {
            message: "Welch estimator produced no segments".to_string(),
        });
    }

    for bin in &mut averaged {
        *bin /= segment_count as f64;
    }

    let frequency_resolution_hz = sample_rate_hz / config.segment_len as f64;
    let half_len = config.segment_len / 2;
    let mut points = Vec::with_capacity(config.segment_len);
    for signed_index in -(half_len as isize)..(half_len as isize) {
        let source_index = usize::try_from(signed_index.rem_euclid(config.segment_len as isize))
            .expect("index fits");
        points.push(PowerSpectralDensityPoint {
            frequency_hz: signed_index as f64 * frequency_resolution_hz,
            power_density: averaged[source_index],
        });
    }
    Ok(points)
}

/// Summarize a PSD with centroid, occupied bandwidth, integrated power, and symmetry error.
pub fn summarize_power_spectral_density(
    points: &[PowerSpectralDensityPoint],
    occupied_power_fraction: f64,
) -> Result<PowerSpectralDensitySummary, SignalError> {
    validate_power_density_points(points)?;
    validate_occupied_power_fraction(occupied_power_fraction)?;

    let frequency_resolution_hz = frequency_resolution_hz(points)?;
    let total_power =
        points.iter().map(|point| point.power_density).sum::<f64>() * frequency_resolution_hz;
    if total_power <= MIN_POWER_DENSITY {
        return Err(SignalError::InvalidSpectrumAnalysis {
            message: "cannot summarize a near-zero spectrum".to_string(),
        });
    }

    let center_frequency_hz =
        points.iter().map(|point| point.frequency_hz * point.power_density).sum::<f64>()
            * frequency_resolution_hz
            / total_power;

    let symmetry_error = normalized_symmetry_error(points)?;
    let occupied_bandwidth_hz =
        occupied_bandwidth(points, occupied_power_fraction, frequency_resolution_hz, total_power)?;

    Ok(PowerSpectralDensitySummary {
        center_frequency_hz,
        occupied_bandwidth_hz,
        integrated_power: total_power,
        symmetry_error,
    })
}

/// Find deep local minima below a relative threshold, returned on the non-negative side.
pub fn find_deep_spectrum_nulls(
    points: &[PowerSpectralDensityPoint],
    relative_threshold_db: f64,
) -> Result<Vec<SpectrumNull>, SignalError> {
    validate_power_density_points(points)?;
    if !relative_threshold_db.is_finite() || relative_threshold_db >= 0.0 {
        return Err(SignalError::InvalidSpectrumAnalysis {
            message: format!(
                "relative_threshold_db must be finite and negative, got {relative_threshold_db}"
            ),
        });
    }

    let threshold_power = points.iter().map(|point| point.power_density).fold(0.0_f64, f64::max)
        * 10.0_f64.powf(relative_threshold_db / 10.0);
    let merge_spacing_hz = frequency_resolution_hz(points)? * 1.5;
    let mut nulls: Vec<SpectrumNull> = Vec::new();

    for index in 1..points.len() - 1 {
        let point = points[index];
        if point.frequency_hz < -merge_spacing_hz {
            continue;
        }
        if point.power_density > threshold_power {
            continue;
        }
        if point.power_density > points[index - 1].power_density
            || point.power_density > points[index + 1].power_density
        {
            continue;
        }

        if let Some(previous) = nulls.last_mut() {
            if (point.frequency_hz - previous.frequency_hz).abs() <= merge_spacing_hz {
                if point.power_density < previous.power_density {
                    *previous = SpectrumNull {
                        frequency_hz: point.frequency_hz,
                        power_density: point.power_density,
                    };
                }
                continue;
            }
        }

        nulls.push(SpectrumNull {
            frequency_hz: point.frequency_hz,
            power_density: point.power_density,
        });
    }

    Ok(nulls)
}

fn validate_component(component: SignalComponentSpec) -> Result<(), SignalError> {
    if !component.primary_code_rate_hz.is_finite() || component.primary_code_rate_hz <= 0.0 {
        return Err(SignalError::InvalidSpectrumAnalysis {
            message: format!(
                "component primary_code_rate_hz must be finite and positive, got {}",
                component.primary_code_rate_hz
            ),
        });
    }
    if !component.power_fraction.is_finite() || component.power_fraction < 0.0 {
        return Err(SignalError::InvalidSpectrumAnalysis {
            message: format!(
                "component power_fraction must be finite and non-negative, got {}",
                component.power_fraction
            ),
        });
    }
    Ok(())
}

fn validate_sample_rate(sample_rate_hz: f64) -> Result<(), SignalError> {
    if !sample_rate_hz.is_finite() || sample_rate_hz <= 0.0 {
        return Err(SignalError::InvalidSpectrumAnalysis {
            message: format!("sample_rate_hz must be finite and positive, got {sample_rate_hz}"),
        });
    }
    Ok(())
}

fn validate_occupied_power_fraction(occupied_power_fraction: f64) -> Result<(), SignalError> {
    if !occupied_power_fraction.is_finite()
        || occupied_power_fraction <= 0.0
        || occupied_power_fraction >= 1.0
    {
        return Err(SignalError::InvalidSpectrumAnalysis {
            message: format!(
                "occupied_power_fraction must be finite and inside (0, 1), got {occupied_power_fraction}"
            ),
        });
    }
    Ok(())
}

fn validate_frequency_grid(frequencies_hz: &[f64]) -> Result<(), SignalError> {
    if frequencies_hz.is_empty() {
        return Err(SignalError::InvalidSpectrumAnalysis {
            message: "frequency grid must not be empty".to_string(),
        });
    }
    if !frequencies_hz[0].is_finite() {
        return Err(SignalError::InvalidSpectrumAnalysis {
            message: "frequency grid values must be finite".to_string(),
        });
    }
    for window in frequencies_hz.windows(2) {
        if !window[0].is_finite() || !window[1].is_finite() {
            return Err(SignalError::InvalidSpectrumAnalysis {
                message: "frequency grid values must be finite".to_string(),
            });
        }
        if window[1] <= window[0] {
            return Err(SignalError::InvalidSpectrumAnalysis {
                message: "frequency grid must be strictly increasing".to_string(),
            });
        }
    }
    Ok(())
}

fn validate_power_density_points(points: &[PowerSpectralDensityPoint]) -> Result<(), SignalError> {
    if points.len() < 3 {
        return Err(SignalError::InvalidSpectrumAnalysis {
            message: format!("need at least 3 PSD points, got {}", points.len()),
        });
    }
    for window in points.windows(2) {
        if !window[0].frequency_hz.is_finite()
            || !window[1].frequency_hz.is_finite()
            || !window[0].power_density.is_finite()
            || !window[1].power_density.is_finite()
            || window[0].power_density < 0.0
            || window[1].power_density < 0.0
        {
            return Err(SignalError::InvalidSpectrumAnalysis {
                message: "PSD points must be finite with non-negative power density".to_string(),
            });
        }
        if window[1].frequency_hz <= window[0].frequency_hz {
            return Err(SignalError::InvalidSpectrumAnalysis {
                message: "PSD points must be strictly ordered by frequency".to_string(),
            });
        }
    }
    Ok(())
}

fn frequency_resolution_hz(points: &[PowerSpectralDensityPoint]) -> Result<f64, SignalError> {
    let resolution_hz = points[1].frequency_hz - points[0].frequency_hz;
    if !resolution_hz.is_finite() || resolution_hz <= 0.0 {
        return Err(SignalError::InvalidSpectrumAnalysis {
            message: format!("invalid PSD frequency resolution {resolution_hz}"),
        });
    }
    for window in points.windows(2) {
        let delta_hz = window[1].frequency_hz - window[0].frequency_hz;
        if (delta_hz - resolution_hz).abs() > resolution_hz * 1.0e-6 {
            return Err(SignalError::InvalidSpectrumAnalysis {
                message: "PSD frequency grid must be uniformly spaced".to_string(),
            });
        }
    }
    Ok(resolution_hz)
}

fn occupied_bandwidth(
    points: &[PowerSpectralDensityPoint],
    occupied_power_fraction: f64,
    frequency_resolution_hz: f64,
    total_power: f64,
) -> Result<f64, SignalError> {
    let target_power = total_power * occupied_power_fraction;
    let center_index = nearest_zero_frequency_index(points);
    let mut accumulated_power = points[center_index].power_density * frequency_resolution_hz;
    let mut occupied_edge_hz = points[center_index].frequency_hz.abs();
    let mut offset = 1usize;

    while accumulated_power < target_power
        && (center_index >= offset || center_index + offset < points.len())
    {
        if center_index >= offset {
            accumulated_power +=
                points[center_index - offset].power_density * frequency_resolution_hz;
            occupied_edge_hz =
                occupied_edge_hz.max(points[center_index - offset].frequency_hz.abs());
        }
        if center_index + offset < points.len() {
            accumulated_power +=
                points[center_index + offset].power_density * frequency_resolution_hz;
            occupied_edge_hz =
                occupied_edge_hz.max(points[center_index + offset].frequency_hz.abs());
        }
        offset += 1;
    }

    if accumulated_power < target_power {
        return Err(SignalError::InvalidSpectrumAnalysis {
            message: "frequency grid did not capture enough power to compute occupied bandwidth"
                .to_string(),
        });
    }
    Ok(occupied_edge_hz * 2.0)
}

fn nearest_zero_frequency_index(points: &[PowerSpectralDensityPoint]) -> usize {
    let mut best_index = 0usize;
    let mut best_distance = points[0].frequency_hz.abs();
    for (index, point) in points.iter().enumerate().skip(1) {
        let distance = point.frequency_hz.abs();
        if distance < best_distance {
            best_index = index;
            best_distance = distance;
        }
    }
    best_index
}

fn normalized_symmetry_error(points: &[PowerSpectralDensityPoint]) -> Result<f64, SignalError> {
    let frequency_resolution_hz = frequency_resolution_hz(points)?;
    let total_power =
        points.iter().map(|point| point.power_density).sum::<f64>() * frequency_resolution_hz;
    let center_index = nearest_zero_frequency_index(points);
    let mirrored_pair_count = center_index.min(points.len() - center_index - 1);
    if mirrored_pair_count == 0 {
        return Err(SignalError::InvalidSpectrumAnalysis {
            message: "need mirrored positive and negative bins to measure spectrum symmetry"
                .to_string(),
        });
    }

    let mut asymmetry = 0.0_f64;
    for offset in 1..=mirrored_pair_count {
        let negative = points[center_index - offset].power_density;
        let positive = points[center_index + offset].power_density;
        asymmetry += (positive - negative).abs() * frequency_resolution_hz;
    }
    Ok(asymmetry / total_power.max(MIN_POWER_DENSITY))
}

fn normalized_chip_segments(
    subcarrier: SignalSubcarrierSpec,
) -> Result<Vec<(f64, f64, f64)>, SignalError> {
    let mut segments = chip_segments(subcarrier)?;
    let mean_power = segments
        .iter()
        .map(|(start, end, amplitude)| (end - start) * amplitude * amplitude)
        .sum::<f64>();
    if mean_power <= MIN_POWER_DENSITY {
        return Err(SignalError::InvalidSpectrumAnalysis {
            message: "chip waveform mean power must be positive".to_string(),
        });
    }
    let scale = mean_power.sqrt().recip();
    for (_, _, amplitude) in &mut segments {
        *amplitude *= scale;
    }
    Ok(segments)
}

fn chip_segments(subcarrier: SignalSubcarrierSpec) -> Result<Vec<(f64, f64, f64)>, SignalError> {
    match subcarrier {
        SignalSubcarrierSpec::None => Ok(vec![(0.0, 1.0, 1.0)]),
        SignalSubcarrierSpec::Boc { cycles_per_chip } => boc_segments(cycles_per_chip, 1.0),
        SignalSubcarrierSpec::Cboc { boc11_weight, boc61_weight } => {
            cboc_segments(f64::from(boc11_weight), f64::from(boc61_weight))
        }
    }
}

fn boc_segments(cycles_per_chip: u32, amplitude: f64) -> Result<Vec<(f64, f64, f64)>, SignalError> {
    if cycles_per_chip == 0 {
        return Err(SignalError::InvalidSpectrumAnalysis {
            message: "Boc cycles_per_chip must be > 0".to_string(),
        });
    }
    let segment_count = usize::try_from(cycles_per_chip).expect("u32 fits usize").saturating_mul(2);
    let segment_width = 1.0 / segment_count as f64;
    let mut segments = Vec::with_capacity(segment_count);
    for index in 0..segment_count {
        let sign = if index % 2 == 0 { 1.0 } else { -1.0 };
        segments.push((
            index as f64 * segment_width,
            (index + 1) as f64 * segment_width,
            amplitude * sign,
        ));
    }
    Ok(segments)
}

fn cboc_segments(
    boc11_weight: f64,
    boc61_weight: f64,
) -> Result<Vec<(f64, f64, f64)>, SignalError> {
    if !boc11_weight.is_finite() || !boc61_weight.is_finite() {
        return Err(SignalError::InvalidSpectrumAnalysis {
            message: "Cboc weights must be finite".to_string(),
        });
    }

    let boc11 = boc_segments(1, boc11_weight)?;
    let boc61 = boc_segments(6, boc61_weight)?;
    let segment_count = 12usize;
    let segment_width = 1.0 / segment_count as f64;
    let mut segments = Vec::with_capacity(segment_count);

    for index in 0..segment_count {
        let midpoint = (index as f64 + 0.5) * segment_width;
        let boc11_amplitude = segment_amplitude_at_phase(&boc11, midpoint);
        let boc61_amplitude = segment_amplitude_at_phase(&boc61, midpoint);
        segments.push((
            index as f64 * segment_width,
            (index + 1) as f64 * segment_width,
            boc11_amplitude + boc61_amplitude,
        ));
    }
    Ok(segments)
}

fn segment_amplitude_at_phase(segments: &[(f64, f64, f64)], phase: f64) -> f64 {
    segments
        .iter()
        .find(|(start, end, _)| phase >= *start && phase < *end)
        .map(|(_, _, amplitude)| *amplitude)
        .unwrap_or_else(|| segments.last().map(|(_, _, amplitude)| *amplitude).unwrap_or(0.0))
}

fn chip_period_power_spectral_density(
    segments: &[(f64, f64, f64)],
    chip_period_s: f64,
    frequency_hz: f64,
) -> f64 {
    let omega = 2.0 * std::f64::consts::PI * frequency_hz;
    let transform =
        segments.iter().fold(Complex::new(0.0, 0.0), |accumulator, (start, end, amplitude)| {
            let t0 = start * chip_period_s;
            let t1 = end * chip_period_s;
            let integral = if omega.abs() <= 1.0e-12 {
                Complex::new(t1 - t0, 0.0)
            } else {
                let phasor_0 = Complex::from_polar(1.0, -omega * t0);
                let phasor_1 = Complex::from_polar(1.0, -omega * t1);
                (phasor_1 - phasor_0) / Complex::new(0.0, -omega)
            };
            accumulator + integral * *amplitude
        });
    transform.norm_sqr() / chip_period_s
}

fn hann_window(segment_len: usize) -> Vec<f64> {
    if segment_len == 1 {
        return vec![1.0];
    }
    (0..segment_len)
        .map(|index| {
            0.5 - 0.5
                * ((2.0 * std::f64::consts::PI * index as f64) / (segment_len - 1) as f64).cos()
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::{
        estimate_power_spectral_density, expected_component_power_spectral_density,
        find_deep_spectrum_nulls, summarize_power_spectral_density, PowerSpectralDensityPoint,
        SpectrumEstimatorConfig,
    };
    use crate::error::SignalError;
    use bijux_gnss_core::api::{SignalComponentRole, SignalComponentSpec, SignalSubcarrierSpec};

    fn bpsk_component(chip_rate_hz: f64) -> SignalComponentSpec {
        SignalComponentSpec {
            role: SignalComponentRole::Data,
            primary_code_rate_hz: chip_rate_hz,
            primary_code_chips: chip_rate_hz.round() as u32,
            primary_code_period_s: 1.0,
            secondary_code: None,
            subcarrier: SignalSubcarrierSpec::None,
            symbol_period_s: None,
            power_fraction: 1.0,
        }
    }

    fn symmetric_frequency_grid(max_frequency_hz: f64, point_count: usize) -> Vec<f64> {
        let step_hz = (2.0 * max_frequency_hz) / (point_count - 1) as f64;
        (0..point_count).map(|index| -max_frequency_hz + index as f64 * step_hz).collect()
    }

    fn tone_samples(sample_rate_hz: f64, frequency_hz: f64, sample_count: usize) -> Vec<f32> {
        (0..sample_count)
            .map(|index| {
                let phase =
                    2.0 * std::f64::consts::PI * frequency_hz * index as f64 / sample_rate_hz;
                phase.cos() as f32
            })
            .collect()
    }

    #[test]
    fn expected_bpsk_spectrum_has_first_null_at_chip_rate() {
        let chip_rate_hz = 1_023_000.0;
        let grid = symmetric_frequency_grid(2_500_000.0, 20_001);
        let spectrum =
            expected_component_power_spectral_density(bpsk_component(chip_rate_hz), &grid)
                .expect("expected BPSK spectrum");
        let nulls = find_deep_spectrum_nulls(&spectrum, -40.0).expect("deep BPSK nulls");

        assert!((nulls[0].frequency_hz - 1_023_000.0).abs() <= 400.0, "{nulls:?}");
        assert!((nulls[1].frequency_hz - 2_046_000.0).abs() <= 400.0, "{nulls:?}");
    }

    #[test]
    fn expected_cboc_spectrum_has_deep_baseband_null() {
        let grid = symmetric_frequency_grid(8_000_000.0, 24_001);
        let component = SignalComponentSpec {
            role: SignalComponentRole::Pilot,
            primary_code_rate_hz: 1_023_000.0,
            primary_code_chips: 4_092,
            primary_code_period_s: 0.004,
            secondary_code: None,
            subcarrier: SignalSubcarrierSpec::Cboc {
                boc11_weight: 0.953_462_6,
                boc61_weight: 0.301_511_35,
            },
            symbol_period_s: None,
            power_fraction: 1.0,
        };
        let spectrum = expected_component_power_spectral_density(component, &grid)
            .expect("expected CBOC spectrum");
        let nulls = find_deep_spectrum_nulls(&spectrum, -30.0).expect("deep CBOC nulls");

        assert!(nulls.iter().any(|null| null.frequency_hz.abs() <= 400.0), "{nulls:?}");
    }

    #[test]
    fn welch_estimator_preserves_real_tone_symmetry_and_power() {
        let sample_rate_hz = 8_192.0;
        let samples = tone_samples(sample_rate_hz, 512.0, 16_384);
        let spectrum = estimate_power_spectral_density(
            &samples,
            sample_rate_hz,
            SpectrumEstimatorConfig { segment_len: 1024, overlap_len: 512 },
        )
        .expect("Welch PSD");
        let summary = summarize_power_spectral_density(&spectrum, 0.99).expect("PSD summary");

        assert!(summary.center_frequency_hz.abs() <= 1.0e-6, "{summary:?}");
        assert!((summary.integrated_power - 0.5).abs() <= 0.05, "{summary:?}");
        assert!(summary.symmetry_error <= 1.0e-6, "{summary:?}");
    }

    #[test]
    fn summarize_rejects_non_uniform_frequency_spacing() {
        let points = [
            PowerSpectralDensityPoint { frequency_hz: -1.0, power_density: 1.0 },
            PowerSpectralDensityPoint { frequency_hz: 0.0, power_density: 1.0 },
            PowerSpectralDensityPoint { frequency_hz: 1.5, power_density: 1.0 },
        ];
        let error = summarize_power_spectral_density(&points, 0.99).expect_err("must fail");
        assert_eq!(
            error,
            SignalError::InvalidSpectrumAnalysis {
                message: "PSD frequency grid must be uniformly spaced".to_string(),
            }
        );
    }
}
