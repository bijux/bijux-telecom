use crate::error::SignalError;
use bijux_gnss_core::api::Sample;
use num_complex::Complex;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

const MIN_FRONT_END_TAPS: usize = 3;
const MIN_RESPONSE_MAGNITUDE: f64 = 1.0e-12;

/// Configurable complex front-end FIR filter.
#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema, PartialEq)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum FrontEndFilterSpec {
    /// Linear-phase low-pass filter centered at baseband.
    LowPass {
        /// One-sided passband edge in Hz.
        cutoff_hz: f64,
        /// Odd tap count used by the FIR design.
        taps: usize,
    },
    /// Linear-phase complex band-pass filter.
    BandPass {
        /// Passband center in Hz relative to complex baseband.
        center_hz: f64,
        /// Full passband width in Hz.
        bandwidth_hz: f64,
        /// Odd tap count used by the FIR design.
        taps: usize,
    },
}

impl FrontEndFilterSpec {
    /// FIR group delay in samples.
    pub fn group_delay_samples(&self) -> usize {
        (self.tap_count() - 1) / 2
    }

    /// FIR group delay in seconds for the provided sample rate.
    pub fn group_delay_seconds(&self, sample_rate_hz: f64) -> Result<f64, SignalError> {
        validate_sample_rate(sample_rate_hz)?;
        Ok(self.group_delay_samples() as f64 / sample_rate_hz)
    }

    /// Stable bandwidth descriptor for reporting and validation.
    pub fn bandwidth_hz(&self) -> f64 {
        match self {
            Self::LowPass { cutoff_hz, .. } => cutoff_hz * 2.0,
            Self::BandPass { bandwidth_hz, .. } => *bandwidth_hz,
        }
    }

    /// Validate the filter spec for a specific sample rate.
    pub fn validate(&self, sample_rate_hz: f64) -> Result<(), SignalError> {
        validate_sample_rate(sample_rate_hz)?;
        validate_tap_count(self.tap_count())?;
        let nyquist_hz = sample_rate_hz * 0.5;
        match self {
            Self::LowPass { cutoff_hz, .. } => {
                validate_positive_finite("cutoff_hz", *cutoff_hz)?;
                if *cutoff_hz >= nyquist_hz {
                    return Err(SignalError::InvalidFrontEndFilter {
                        message: format!(
                            "low-pass cutoff_hz must be below Nyquist ({nyquist_hz:.3} Hz)"
                        ),
                    });
                }
            }
            Self::BandPass { center_hz, bandwidth_hz, .. } => {
                validate_finite("center_hz", *center_hz)?;
                validate_positive_finite("bandwidth_hz", *bandwidth_hz)?;
                let half_bandwidth_hz = *bandwidth_hz * 0.5;
                let lower_edge_hz = *center_hz - half_bandwidth_hz;
                let upper_edge_hz = *center_hz + half_bandwidth_hz;
                if lower_edge_hz <= -nyquist_hz || upper_edge_hz >= nyquist_hz {
                    return Err(SignalError::InvalidFrontEndFilter {
                        message: format!(
                            "band-pass edges [{lower_edge_hz:.3}, {upper_edge_hz:.3}] Hz must remain inside (-{nyquist_hz:.3}, {nyquist_hz:.3})"
                        ),
                    });
                }
            }
        }
        Ok(())
    }

    /// Build a stateful filter instance for streaming sample frames.
    pub fn design(&self, sample_rate_hz: f64) -> Result<FrontEndFirFilter, SignalError> {
        self.validate(sample_rate_hz)?;
        let taps = match self {
            Self::LowPass { cutoff_hz, taps } => {
                let mut coefficients = design_low_pass_taps(*cutoff_hz, sample_rate_hz, *taps);
                normalize_for_frequency(&mut coefficients, sample_rate_hz, 0.0)?;
                coefficients
            }
            Self::BandPass { center_hz, bandwidth_hz, taps } => {
                let mut coefficients =
                    design_band_pass_taps(*center_hz, *bandwidth_hz, sample_rate_hz, *taps);
                normalize_for_frequency(&mut coefficients, sample_rate_hz, *center_hz)?;
                coefficients
            }
        };
        Ok(FrontEndFirFilter::new(self.clone(), sample_rate_hz, taps))
    }

    fn tap_count(&self) -> usize {
        match self {
            Self::LowPass { taps, .. } | Self::BandPass { taps, .. } => *taps,
        }
    }
}

/// Statefully applies a designed complex FIR front-end filter.
#[derive(Debug, Clone)]
pub struct FrontEndFirFilter {
    spec: FrontEndFilterSpec,
    sample_rate_hz: f64,
    taps: Vec<Sample>,
    history: Vec<Sample>,
    next_slot: usize,
}

impl FrontEndFirFilter {
    fn new(spec: FrontEndFilterSpec, sample_rate_hz: f64, taps: Vec<Sample>) -> Self {
        Self {
            spec,
            sample_rate_hz,
            history: vec![Complex::new(0.0, 0.0); taps.len()],
            taps,
            next_slot: 0,
        }
    }

    /// Filter configuration used to build this instance.
    pub fn spec(&self) -> &FrontEndFilterSpec {
        &self.spec
    }

    /// Sample rate used for this filter instance.
    pub fn sample_rate_hz(&self) -> f64 {
        self.sample_rate_hz
    }

    /// FIR group delay in samples.
    pub fn group_delay_samples(&self) -> usize {
        self.spec.group_delay_samples()
    }

    /// Reset internal filter state.
    pub fn reset(&mut self) {
        self.history.fill(Complex::new(0.0, 0.0));
        self.next_slot = 0;
    }

    /// Borrow the complex FIR coefficients in convolution order.
    pub fn taps(&self) -> &[Sample] {
        &self.taps
    }

    /// Apply the filter to one frame in place, preserving cross-frame history.
    pub fn apply_in_place(&mut self, samples: &mut [Sample]) {
        for sample in samples {
            *sample = self.push_sample(*sample);
        }
    }

    /// Measure complex gain at a specific frequency.
    pub fn response_at(&self, frequency_hz: f64) -> Result<Complex<f64>, SignalError> {
        validate_finite("frequency_hz", frequency_hz)?;
        let nyquist_hz = self.sample_rate_hz * 0.5;
        if frequency_hz < -nyquist_hz || frequency_hz > nyquist_hz {
            return Err(SignalError::InvalidFrontEndFilter {
                message: format!(
                    "response frequency {frequency_hz:.3} Hz must be within [-{nyquist_hz:.3}, {nyquist_hz:.3}]"
                ),
            });
        }
        Ok(compute_frequency_response(&self.taps, self.sample_rate_hz, frequency_hz))
    }

    fn push_sample(&mut self, sample: Sample) -> Sample {
        self.history[self.next_slot] = sample;
        let mut accumulator = Complex::new(0.0_f32, 0.0_f32);
        let mut history_index = self.next_slot;
        for tap in &self.taps {
            accumulator += *tap * self.history[history_index];
            history_index =
                if history_index == 0 { self.history.len() - 1 } else { history_index - 1 };
        }
        self.next_slot = (self.next_slot + 1) % self.history.len();
        accumulator
    }
}

/// Measured transfer response for one frequency.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MeasuredFrontEndResponse {
    /// Frequency at which the response was measured.
    pub frequency_hz: f64,
    /// Measured gain in decibels.
    pub gain_db: f64,
}

/// Measure front-end transfer response at selected frequencies.
pub fn measure_transfer_response_db(
    input: &[Sample],
    output: &[Sample],
    sample_rate_hz: f64,
    frequencies_hz: &[f64],
) -> Result<Vec<MeasuredFrontEndResponse>, SignalError> {
    validate_sample_rate(sample_rate_hz)?;
    if input.len() != output.len() {
        return Err(SignalError::InvalidFrontEndFilter {
            message: format!(
                "input/output length mismatch while measuring transfer response: left={} right={}",
                input.len(),
                output.len()
            ),
        });
    }
    frequencies_hz
        .iter()
        .map(|&frequency_hz| {
            let input_magnitude = projected_tone_magnitude(input, sample_rate_hz, frequency_hz)?;
            let output_magnitude = projected_tone_magnitude(output, sample_rate_hz, frequency_hz)?;
            let gain_db = magnitude_ratio_db(input_magnitude, output_magnitude);
            Ok(MeasuredFrontEndResponse { frequency_hz, gain_db })
        })
        .collect()
}

fn validate_sample_rate(sample_rate_hz: f64) -> Result<(), SignalError> {
    if !sample_rate_hz.is_finite() || sample_rate_hz <= 0.0 {
        return Err(SignalError::InvalidSampleRate);
    }
    Ok(())
}

fn validate_tap_count(taps: usize) -> Result<(), SignalError> {
    if taps < MIN_FRONT_END_TAPS {
        return Err(SignalError::InvalidFrontEndFilter {
            message: format!("front-end tap count must be >= {MIN_FRONT_END_TAPS}"),
        });
    }
    if taps % 2 == 0 {
        return Err(SignalError::InvalidFrontEndFilter {
            message: "front-end tap count must be odd to keep an integer group delay".to_string(),
        });
    }
    Ok(())
}

fn validate_positive_finite(name: &str, value: f64) -> Result<(), SignalError> {
    validate_finite(name, value)?;
    if value <= 0.0 {
        return Err(SignalError::InvalidFrontEndFilter { message: format!("{name} must be > 0") });
    }
    Ok(())
}

fn validate_finite(name: &str, value: f64) -> Result<(), SignalError> {
    if !value.is_finite() {
        return Err(SignalError::InvalidFrontEndFilter {
            message: format!("{name} must be finite"),
        });
    }
    Ok(())
}

fn design_low_pass_taps(cutoff_hz: f64, sample_rate_hz: f64, taps: usize) -> Vec<Sample> {
    let normalized_cutoff = cutoff_hz / sample_rate_hz;
    let delay = (taps - 1) as f64 * 0.5;
    (0..taps)
        .map(|index| {
            let centered = index as f64 - delay;
            let sinc = if centered.abs() <= f64::EPSILON {
                2.0 * normalized_cutoff
            } else {
                (2.0 * std::f64::consts::PI * normalized_cutoff * centered).sin()
                    / (std::f64::consts::PI * centered)
            };
            let window = hamming_window(index, taps);
            Complex::new((sinc * window) as f32, 0.0)
        })
        .collect()
}

fn design_band_pass_taps(
    center_hz: f64,
    bandwidth_hz: f64,
    sample_rate_hz: f64,
    taps: usize,
) -> Vec<Sample> {
    let half_bandwidth_hz = bandwidth_hz * 0.5;
    let delay = (taps - 1) as f64 * 0.5;
    design_low_pass_taps(half_bandwidth_hz, sample_rate_hz, taps)
        .into_iter()
        .enumerate()
        .map(|(index, coefficient)| {
            let centered = index as f64 - delay;
            let phase = 2.0 * std::f64::consts::PI * center_hz * centered / sample_rate_hz;
            let rotation = Complex::new(phase.cos() as f32, phase.sin() as f32);
            coefficient * rotation
        })
        .collect()
}

fn normalize_for_frequency(
    taps: &mut [Sample],
    sample_rate_hz: f64,
    frequency_hz: f64,
) -> Result<(), SignalError> {
    let response = compute_frequency_response(taps, sample_rate_hz, frequency_hz);
    let magnitude = response.norm();
    if magnitude <= MIN_RESPONSE_MAGNITUDE {
        return Err(SignalError::InvalidFrontEndFilter {
            message: "front-end filter normalization encountered a zero passband response"
                .to_string(),
        });
    }
    let scale = 1.0_f32 / magnitude as f32;
    for tap in taps {
        *tap *= scale;
    }
    Ok(())
}

fn compute_frequency_response(
    taps: &[Sample],
    sample_rate_hz: f64,
    frequency_hz: f64,
) -> Complex<f64> {
    taps.iter().enumerate().fold(Complex::new(0.0, 0.0), |accumulator, (index, tap)| {
        let phase = -2.0 * std::f64::consts::PI * frequency_hz * index as f64 / sample_rate_hz;
        let phasor = Complex::new(phase.cos(), phase.sin());
        accumulator + Complex::new(tap.re as f64, tap.im as f64) * phasor
    })
}

fn projected_tone_magnitude(
    samples: &[Sample],
    sample_rate_hz: f64,
    frequency_hz: f64,
) -> Result<f64, SignalError> {
    validate_finite("frequency_hz", frequency_hz)?;
    if samples.is_empty() {
        return Err(SignalError::InvalidFrontEndFilter {
            message: "cannot measure transfer response from an empty sample slice".to_string(),
        });
    }
    let coefficient =
        samples.iter().enumerate().fold(Complex::new(0.0, 0.0), |accumulator, (index, sample)| {
            let phase = -2.0 * std::f64::consts::PI * frequency_hz * index as f64 / sample_rate_hz;
            let phasor = Complex::new(phase.cos(), phase.sin());
            accumulator + Complex::new(sample.re as f64, sample.im as f64) * phasor
        }) / samples.len() as f64;
    Ok(coefficient.norm())
}

fn magnitude_ratio_db(input_magnitude: f64, output_magnitude: f64) -> f64 {
    let numerator = output_magnitude.max(MIN_RESPONSE_MAGNITUDE);
    let denominator = input_magnitude.max(MIN_RESPONSE_MAGNITUDE);
    20.0 * (numerator / denominator).log10()
}

fn hamming_window(index: usize, taps: usize) -> f64 {
    if taps == 1 {
        return 1.0;
    }
    0.54 - 0.46 * ((2.0 * std::f64::consts::PI * index as f64) / (taps - 1) as f64).cos()
}

#[cfg(test)]
mod tests {
    use super::{measure_transfer_response_db, FrontEndFilterSpec, MeasuredFrontEndResponse};
    use bijux_gnss_core::api::Sample;
    use num_complex::Complex;

    fn synthesize_tone_comb(
        sample_rate_hz: f64,
        sample_count: usize,
        frequencies_hz: &[f64],
    ) -> Vec<Sample> {
        let normalization = frequencies_hz.len().max(1) as f32;
        (0..sample_count)
            .map(|index| {
                frequencies_hz.iter().fold(
                    Complex::new(0.0_f32, 0.0_f32),
                    |accumulator, frequency_hz| {
                        let phase = 2.0 * std::f64::consts::PI * frequency_hz * index as f64
                            / sample_rate_hz;
                        accumulator + Complex::new(phase.cos() as f32, phase.sin() as f32)
                    },
                ) / normalization
            })
            .collect()
    }

    fn measured_gain_db(measured: &[MeasuredFrontEndResponse], frequency_hz: f64) -> f64 {
        measured
            .iter()
            .find(|point| (point.frequency_hz - frequency_hz).abs() <= f64::EPSILON)
            .map(|point| point.gain_db)
            .expect("measured response point")
    }

    #[test]
    fn low_pass_filter_reports_known_group_delay() {
        let spec = FrontEndFilterSpec::LowPass { cutoff_hz: 600_000.0, taps: 41 };
        assert_eq!(spec.group_delay_samples(), 20);
        assert!(
            (spec.group_delay_seconds(4_092_000.0).expect("group delay seconds")
                - (20.0 / 4_092_000.0))
                .abs()
                <= 1.0e-12
        );
    }

    #[test]
    fn streaming_filter_matches_single_block_processing() {
        let sample_rate_hz = 4_092_000.0;
        let spec = FrontEndFilterSpec::BandPass {
            center_hz: 350_000.0,
            bandwidth_hz: 500_000.0,
            taps: 63,
        };
        let mut single_block = spec.design(sample_rate_hz).expect("single block filter");
        let mut segmented = spec.design(sample_rate_hz).expect("segmented filter");
        let mut block = synthesize_tone_comb(
            sample_rate_hz,
            1024,
            &[-850_000.0, -200_000.0, 350_000.0, 700_000.0],
        );
        let mut left = block.clone();
        let mut right_a = block[..341].to_vec();
        let mut right_b = block[341..777].to_vec();
        let mut right_c = block[777..].to_vec();

        single_block.apply_in_place(&mut left);
        segmented.apply_in_place(&mut right_a);
        segmented.apply_in_place(&mut right_b);
        segmented.apply_in_place(&mut right_c);

        let mut right = Vec::new();
        right.extend_from_slice(&right_a);
        right.extend_from_slice(&right_b);
        right.extend_from_slice(&right_c);

        assert_eq!(left.len(), right.len());
        for (lhs, rhs) in left.iter().zip(right.iter()) {
            assert!((lhs.re - rhs.re).abs() <= 1.0e-5);
            assert!((lhs.im - rhs.im).abs() <= 1.0e-5);
        }

        block.clear();
    }

    #[test]
    fn low_pass_measured_spectrum_matches_designed_response() {
        let sample_rate_hz = 4_092_000.0;
        let frequencies_hz = [-1_100_000.0, -450_000.0, 0.0, 450_000.0, 1_100_000.0];
        let spec = FrontEndFilterSpec::LowPass { cutoff_hz: 600_000.0, taps: 81 };
        let mut filter = spec.design(sample_rate_hz).expect("low-pass filter");
        let input = synthesize_tone_comb(sample_rate_hz, 16_384, &frequencies_hz);
        let mut output = input.clone();
        filter.apply_in_place(&mut output);
        let measured =
            measure_transfer_response_db(&input, &output, sample_rate_hz, &frequencies_hz)
                .expect("measured transfer response");

        assert!(measured_gain_db(&measured, 0.0) >= -0.6);
        assert!(measured_gain_db(&measured, 450_000.0) >= -1.2);
        assert!(measured_gain_db(&measured, 1_100_000.0) <= -16.0);
        assert!(measured_gain_db(&measured, -1_100_000.0) <= -16.0);
    }

    #[test]
    fn band_pass_measured_spectrum_matches_designed_response() {
        let sample_rate_hz = 4_092_000.0;
        let frequencies_hz = [-500_000.0, 50_000.0, 350_000.0, 550_000.0, 1_200_000.0];
        let spec = FrontEndFilterSpec::BandPass {
            center_hz: 350_000.0,
            bandwidth_hz: 500_000.0,
            taps: 81,
        };
        let mut filter = spec.design(sample_rate_hz).expect("band-pass filter");
        let input = synthesize_tone_comb(sample_rate_hz, 16_384, &frequencies_hz);
        let mut output = input.clone();
        filter.apply_in_place(&mut output);
        let measured =
            measure_transfer_response_db(&input, &output, sample_rate_hz, &frequencies_hz)
                .expect("measured transfer response");

        assert!(measured_gain_db(&measured, 350_000.0) >= -0.8);
        assert!(measured_gain_db(&measured, 550_000.0) >= -1.8);
        assert!(measured_gain_db(&measured, 50_000.0) <= -10.0);
        assert!(measured_gain_db(&measured, -500_000.0) <= -14.0);
        assert!(measured_gain_db(&measured, 1_200_000.0) <= -14.0);
    }

    #[test]
    fn filter_validation_rejects_even_tap_counts() {
        let spec = FrontEndFilterSpec::LowPass { cutoff_hz: 600_000.0, taps: 40 };
        let error = spec.validate(4_092_000.0).expect_err("even tap count must fail");
        assert_eq!(
            error,
            crate::error::SignalError::InvalidFrontEndFilter {
                message: "front-end tap count must be odd to keep an integer group delay"
                    .to_string(),
            }
        );
    }
}
