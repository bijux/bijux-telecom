/// Per-satellite power and carrier-phase recovery row for a synthetic composite capture.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticCompositeComponentRecoverySatellite {
    /// Satellite identifier.
    pub sat: SatId,
    /// GLONASS FDMA channel when this row models a GLONASS L1 signal.
    pub glonass_frequency_channel: Option<GlonassFrequencyChannel>,
    /// Explicit signal band carried by this synthetic signal.
    pub signal_band: SignalBand,
    /// Explicit signal code carried by this synthetic signal.
    pub signal_code: SignalCode,
    /// Injected output-scaled signal amplitude.
    pub truth_output_signal_amplitude: f64,
    /// Recovered output-scaled signal amplitude.
    pub recovered_output_signal_amplitude: f64,
    /// Injected output-scaled signal power per complex sample.
    pub truth_output_signal_power_per_complex_sample: f64,
    /// Recovered output-scaled signal power per complex sample.
    pub recovered_output_signal_power_per_complex_sample: f64,
    /// Recovered power error relative to truth, in dB.
    pub power_error_db: f64,
    /// Injected carrier phase at sample zero, in radians.
    pub truth_carrier_phase_rad: f64,
    /// Recovered carrier phase at sample zero, in radians.
    pub recovered_carrier_phase_rad: f64,
    /// Recovered complex coefficient magnitude relative to the truth waveform.
    pub recovered_coefficient_magnitude: f64,
    /// Recovered complex coefficient phase relative to the truth waveform, in radians.
    pub recovered_coefficient_phase_rad: f64,
    /// Wrapped carrier-phase error relative to truth, in radians.
    pub phase_error_rad: f64,
    /// Whether both power and phase stayed within tolerance.
    pub pass: bool,
}

/// Truth-guided power and phase recovery report for a synthetic composite capture.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticCompositeComponentRecoveryReport {
    /// Stable scenario identifier for this capture.
    pub scenario_id: String,
    /// Capture sample rate in Hz.
    pub sample_rate_hz: f64,
    /// Number of complex samples recovered.
    pub sample_count: usize,
    /// Output scale applied before quantization.
    pub output_scale_applied: f32,
    /// Allowed absolute power error in dB.
    pub power_tolerance_db: f64,
    /// Allowed wrapped absolute phase error in radians.
    pub phase_tolerance_rad: f64,
    /// Root-mean-square magnitude of the residual after joint component reconstruction.
    pub residual_rms: f64,
    /// Least-squares solver status for the truth waveform basis.
    pub solver_status: String,
    /// Whether every recovered satellite component passed the requested tolerances.
    pub pass: bool,
    /// Per-satellite recovery rows.
    pub satellites: Vec<SyntheticCompositeComponentRecoverySatellite>,
}

/// Recover per-component power and phase from one truth-complete synthetic composite capture.
///
/// The recovery solves one joint complex least-squares system over all truth waveforms so that
/// overlapping components do not bias one another's recovered amplitude or carrier phase.
pub fn validate_truth_guided_composite_component_recovery(
    config: &ReceiverPipelineConfig,
    frame: &SamplesFrame,
    truth: &SyntheticIqTruthBundle,
    power_tolerance_db: f64,
    phase_tolerance_rad: f64,
) -> SyntheticCompositeComponentRecoveryReport {
    let predicted_components = truth
        .satellites
        .iter()
        .map(|sat_truth| {
            regenerate_isolated_scaled_satellite_signal_only_frame(config, frame, truth, sat_truth)
        })
        .collect::<Vec<_>>();

    let Some(coefficients) = recover_truth_guided_component_coefficients(frame, &predicted_components)
    else {
        return SyntheticCompositeComponentRecoveryReport {
            scenario_id: truth.scenario_id.clone(),
            sample_rate_hz: truth.sample_rate_hz,
            sample_count: frame.len(),
            output_scale_applied: truth.output_scale_applied,
            power_tolerance_db,
            phase_tolerance_rad,
            residual_rms: f64::NAN,
            solver_status: "degenerate_truth_basis".to_string(),
            pass: false,
            satellites: truth
                .satellites
                .iter()
                .map(|sat_truth| {
                    let truth_output_signal_amplitude =
                        sat_truth.signal_amplitude as f64 * truth.output_scale_applied as f64;
                    SyntheticCompositeComponentRecoverySatellite {
                        sat: sat_truth.sat,
                        glonass_frequency_channel: sat_truth.glonass_frequency_channel,
                        signal_band: sat_truth.signal_band,
                        signal_code: sat_truth.signal_code,
                        truth_output_signal_amplitude,
                        recovered_output_signal_amplitude: f64::NAN,
                        truth_output_signal_power_per_complex_sample:
                            truth_output_signal_amplitude * truth_output_signal_amplitude,
                        recovered_output_signal_power_per_complex_sample: f64::NAN,
                        power_error_db: f64::NAN,
                        truth_carrier_phase_rad: sat_truth.carrier_phase_rad,
                        recovered_carrier_phase_rad: f64::NAN,
                        recovered_coefficient_magnitude: f64::NAN,
                        recovered_coefficient_phase_rad: f64::NAN,
                        phase_error_rad: f64::NAN,
                        pass: false,
                    }
                })
                .collect(),
        };
    };

    let residual_rms = composite_recovery_residual_rms(frame, &predicted_components, &coefficients);
    let satellites = truth
        .satellites
        .iter()
        .zip(coefficients.iter())
        .map(|(sat_truth, coefficient)| {
            let truth_output_signal_amplitude =
                sat_truth.signal_amplitude as f64 * truth.output_scale_applied as f64;
            let truth_output_signal_power_per_complex_sample =
                truth_output_signal_amplitude * truth_output_signal_amplitude;
            let recovered_coefficient_magnitude = coefficient.norm();
            let recovered_coefficient_phase_rad = wrapped_phase_rad(coefficient.arg());
            let recovered_output_signal_amplitude =
                truth_output_signal_amplitude * recovered_coefficient_magnitude;
            let recovered_output_signal_power_per_complex_sample =
                truth_output_signal_power_per_complex_sample
                    * recovered_coefficient_magnitude
                    * recovered_coefficient_magnitude;
            let power_error_db =
                power_error_db(recovered_output_signal_power_per_complex_sample,
                    truth_output_signal_power_per_complex_sample);
            let recovered_carrier_phase_rad =
                wrapped_phase_rad(sat_truth.carrier_phase_rad + recovered_coefficient_phase_rad);
            let phase_error_rad =
                wrapped_phase_rad(recovered_carrier_phase_rad - sat_truth.carrier_phase_rad);
            let pass = power_error_db.abs() <= power_tolerance_db + f64::EPSILON
                && phase_error_rad.abs() <= phase_tolerance_rad + f64::EPSILON;

            SyntheticCompositeComponentRecoverySatellite {
                sat: sat_truth.sat,
                glonass_frequency_channel: sat_truth.glonass_frequency_channel,
                signal_band: sat_truth.signal_band,
                signal_code: sat_truth.signal_code,
                truth_output_signal_amplitude,
                recovered_output_signal_amplitude,
                truth_output_signal_power_per_complex_sample,
                recovered_output_signal_power_per_complex_sample,
                power_error_db,
                truth_carrier_phase_rad: sat_truth.carrier_phase_rad,
                recovered_carrier_phase_rad,
                recovered_coefficient_magnitude,
                recovered_coefficient_phase_rad,
                phase_error_rad,
                pass,
            }
        })
        .collect::<Vec<_>>();
    let pass = !satellites.is_empty() && satellites.iter().all(|satellite| satellite.pass);

    SyntheticCompositeComponentRecoveryReport {
        scenario_id: truth.scenario_id.clone(),
        sample_rate_hz: truth.sample_rate_hz,
        sample_count: frame.len(),
        output_scale_applied: truth.output_scale_applied,
        power_tolerance_db,
        phase_tolerance_rad,
        residual_rms,
        solver_status: "ok".to_string(),
        pass,
        satellites,
    }
}

fn recover_truth_guided_component_coefficients(
    frame: &SamplesFrame,
    predicted_components: &[SamplesFrame],
) -> Option<Vec<Complex<f64>>> {
    if predicted_components.is_empty() {
        return Some(Vec::new());
    }

    let dimension = predicted_components.len();
    let mut gram = vec![vec![Complex::new(0.0_f64, 0.0_f64); dimension]; dimension];
    let mut rhs = vec![Complex::new(0.0_f64, 0.0_f64); dimension];
    for row in 0..dimension {
        rhs[row] = complex_inner_product(&predicted_components[row].iq, &frame.iq);
        for column in row..dimension {
            let value = complex_inner_product(
                &predicted_components[row].iq,
                &predicted_components[column].iq,
            );
            gram[row][column] = value;
            gram[column][row] = value.conj();
        }
    }
    solve_complex_linear_system(gram, rhs)
}

fn complex_inner_product(lhs: &[Complex<f32>], rhs: &[Complex<f32>]) -> Complex<f64> {
    lhs.iter()
        .zip(rhs.iter())
        .fold(Complex::new(0.0_f64, 0.0_f64), |sum, (left, right)| {
            sum + complex32_to_64(left.conj()) * complex32_to_64(*right)
        })
}

fn solve_complex_linear_system(
    mut matrix: Vec<Vec<Complex<f64>>>,
    mut rhs: Vec<Complex<f64>>,
) -> Option<Vec<Complex<f64>>> {
    let dimension = matrix.len();
    for pivot_index in 0..dimension {
        let pivot_row = (pivot_index..dimension).max_by(|left, right| {
            matrix[*left][pivot_index]
                .norm_sqr()
                .partial_cmp(&matrix[*right][pivot_index].norm_sqr())
                .unwrap_or(std::cmp::Ordering::Equal)
        })?;
        if matrix[pivot_row][pivot_index].norm_sqr() <= f64::EPSILON {
            return None;
        }
        if pivot_row != pivot_index {
            matrix.swap(pivot_row, pivot_index);
            rhs.swap(pivot_row, pivot_index);
        }

        let pivot = matrix[pivot_index][pivot_index];
        let pivot_rhs = rhs[pivot_index];
        let pivot_row = matrix[pivot_index].clone();
        for row in pivot_index + 1..dimension {
            let factor = matrix[row][pivot_index] / pivot;
            if factor.norm_sqr() <= f64::EPSILON {
                continue;
            }
            rhs[row] -= factor * pivot_rhs;
            for column in pivot_index..dimension {
                matrix[row][column] -= factor * pivot_row[column];
            }
        }
    }

    let mut solution = vec![Complex::new(0.0_f64, 0.0_f64); dimension];
    for row in (0..dimension).rev() {
        let mut sum = rhs[row];
        for column in row + 1..dimension {
            sum -= matrix[row][column] * solution[column];
        }
        let pivot = matrix[row][row];
        if pivot.norm_sqr() <= f64::EPSILON {
            return None;
        }
        solution[row] = sum / pivot;
    }
    Some(solution)
}

fn composite_recovery_residual_rms(
    frame: &SamplesFrame,
    predicted_components: &[SamplesFrame],
    coefficients: &[Complex<f64>],
) -> f64 {
    if frame.iq.is_empty() {
        return 0.0;
    }
    let residual_energy = frame
        .iq
        .iter()
        .enumerate()
        .map(|(sample_index, measured)| {
            let reconstructed = predicted_components
                .iter()
                .zip(coefficients.iter())
                .fold(Complex::new(0.0_f64, 0.0_f64), |sum, (component, coefficient)| {
                    sum + *coefficient * complex32_to_64(component.iq[sample_index])
                });
            let residual = complex32_to_64(*measured) - reconstructed;
            residual.norm_sqr()
        })
        .sum::<f64>();
    (residual_energy / frame.len() as f64).sqrt()
}

fn power_error_db(recovered_power: f64, truth_power: f64) -> f64 {
    if recovered_power <= f64::EPSILON || truth_power <= f64::EPSILON {
        return f64::NEG_INFINITY;
    }
    10.0 * (recovered_power / truth_power).log10()
}

fn wrapped_phase_rad(phase_rad: f64) -> f64 {
    let tau = std::f64::consts::TAU;
    (phase_rad + std::f64::consts::PI).rem_euclid(tau) - std::f64::consts::PI
}

fn complex32_to_64(value: Complex<f32>) -> Complex<f64> {
    Complex::new(value.re as f64, value.im as f64)
}
