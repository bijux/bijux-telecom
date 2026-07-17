use super::*;

#[derive(Debug, Clone, Copy)]
struct DelayedPathProfile {
    delay_chips: f64,
    relative_amplitude: f32,
    carrier_phase_rad: f32,
}

#[derive(Debug, Clone, Copy)]
enum CodeBiasDiscriminator {
    StandardEarlyLate,
    NarrowEarlyLate,
    DoubleDelta,
}

impl CodeBiasDiscriminator {
    fn early_late_spacing_chips(self) -> f64 {
        match self {
            Self::StandardEarlyLate => 0.5,
            Self::NarrowEarlyLate | Self::DoubleDelta => 0.25,
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct MultipathBiasObservation {
    delayed_path: DelayedPathProfile,
    standard_bias_chips: f64,
    narrow_bias_chips: f64,
    double_delta_bias_chips: f64,
}

#[derive(Debug, Clone, Copy)]
struct CleanMultipathBiasBaseline {
    standard_zero_chips: f64,
    narrow_zero_chips: f64,
    double_delta_zero_chips: f64,
}

fn multipath_tracking_config() -> ReceiverPipelineConfig {
    ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        tracking_budget_ms: 100.0,
        tracking_over_budget_action: "continue".to_string(),
        ..ReceiverPipelineConfig::default()
    }
}

fn multipath_bias_frame(
    config: &ReceiverPipelineConfig,
    sat: SatId,
    direct_code_phase_chips: f64,
    delayed_path: DelayedPathProfile,
) -> SamplesFrame {
    let sample_count =
        samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length);
    let direct = sample_ca_code(
        Prn(sat.prn),
        config.sampling_freq_hz,
        config.code_freq_basis_hz,
        direct_code_phase_chips,
        sample_count,
    )
    .expect("direct path code");
    let reflected = sample_ca_code(
        Prn(sat.prn),
        config.sampling_freq_hz,
        config.code_freq_basis_hz,
        direct_code_phase_chips - delayed_path.delay_chips,
        sample_count,
    )
    .expect("delayed path code");
    let iq = direct
        .into_iter()
        .zip(reflected)
        .map(|(direct, reflected)| {
            let phase = Complex::new(
                delayed_path.carrier_phase_rad.cos(),
                delayed_path.carrier_phase_rad.sin(),
            );
            Complex::new(direct, 0.0) + phase * delayed_path.relative_amplitude * reflected
        })
        .collect();
    SamplesFrame::new(
        SampleTime { sample_index: 0, sample_rate_hz: config.sampling_freq_hz },
        Seconds(1.0 / config.sampling_freq_hz),
        iq,
    )
}

fn measured_code_zero_crossing_chips(
    config: &ReceiverPipelineConfig,
    tracking: &Tracking,
    frame: &SamplesFrame,
    sat: SatId,
    direct_code_phase_chips: f64,
    discriminator: CodeBiasDiscriminator,
) -> f64 {
    let samples_per_chip = config.sampling_freq_hz / config.code_freq_basis_hz;
    let early_late_spacing_chips = discriminator.early_late_spacing_chips();
    let direct_code_phase_samples =
        crate::sim::synthetic::expected_acquisition_code_phase_samples_f64(
            config,
            frame,
            direct_code_phase_chips,
        );
    let mut best_abs_discriminator = f32::INFINITY;
    let mut best_offset_chips = 0.0;
    let signal_model = super::TrackingSignalModel::for_sat(config, sat);
    for offset_index in -50..=50 {
        let offset_chips = offset_index as f64 * 0.01;
        let correlation =
            tracking.tracking_epoch_correlation(super::TrackingEpochCorrelationRequest {
                range: super::TrackingCorrelationRange {
                    frame,
                    start: 0,
                    end: frame.len(),
                    sample_index: frame.t0.sample_index,
                },
                signal_model: &signal_model,
                estimate: super::TrackingSignalEstimate {
                    carrier_hz: 0.0,
                    carrier_phase_cycles: 0.0,
                    code_rate_hz: config.code_freq_basis_hz,
                    code_phase_samples: direct_code_phase_samples + offset_chips * samples_per_chip,
                    early_late_spacing_chips,
                },
            });
        let dll_err = match discriminator {
            CodeBiasDiscriminator::DoubleDelta => super::tracking_dll_discriminator(&correlation),
            CodeBiasDiscriminator::StandardEarlyLate | CodeBiasDiscriminator::NarrowEarlyLate => {
                let (dll_err, _, _, _) = discriminators(
                    correlation.primary.early,
                    correlation.primary.prompt,
                    correlation.primary.late,
                    None,
                );
                dll_err
            }
        };
        let dll_err = normalize_dll_discriminator(dll_err, early_late_spacing_chips);
        let abs_discriminator = dll_err.abs();
        if abs_discriminator < best_abs_discriminator {
            best_abs_discriminator = abs_discriminator;
            best_offset_chips = offset_chips;
        }
    }
    best_offset_chips
}

fn measured_early_late_zero_crossing_chips(
    config: &ReceiverPipelineConfig,
    tracking: &Tracking,
    frame: &SamplesFrame,
    sat: SatId,
    direct_code_phase_chips: f64,
    early_late_spacing_chips: f64,
) -> f64 {
    let discriminator = if (early_late_spacing_chips - 0.5).abs() <= f64::EPSILON {
        CodeBiasDiscriminator::StandardEarlyLate
    } else {
        CodeBiasDiscriminator::NarrowEarlyLate
    };
    measured_code_zero_crossing_chips(
        config,
        tracking,
        frame,
        sat,
        direct_code_phase_chips,
        discriminator,
    )
}

fn multipath_bias_observation(
    config: &ReceiverPipelineConfig,
    tracking: &Tracking,
    sat: SatId,
    direct_code_phase_chips: f64,
    baseline: CleanMultipathBiasBaseline,
    delayed_path: DelayedPathProfile,
) -> MultipathBiasObservation {
    let frame = multipath_bias_frame(config, sat, direct_code_phase_chips, delayed_path);
    let standard_zero_chips = measured_code_zero_crossing_chips(
        config,
        tracking,
        &frame,
        sat,
        direct_code_phase_chips,
        CodeBiasDiscriminator::StandardEarlyLate,
    );
    let narrow_zero_chips = measured_code_zero_crossing_chips(
        config,
        tracking,
        &frame,
        sat,
        direct_code_phase_chips,
        CodeBiasDiscriminator::NarrowEarlyLate,
    );
    let double_delta_zero_chips = measured_code_zero_crossing_chips(
        config,
        tracking,
        &frame,
        sat,
        direct_code_phase_chips,
        CodeBiasDiscriminator::DoubleDelta,
    );
    MultipathBiasObservation {
        delayed_path,
        standard_bias_chips: (standard_zero_chips - baseline.standard_zero_chips).abs(),
        narrow_bias_chips: (narrow_zero_chips - baseline.narrow_zero_chips).abs(),
        double_delta_bias_chips: (double_delta_zero_chips - baseline.double_delta_zero_chips).abs(),
    }
}

fn multipath_bias_observations(
    config: &ReceiverPipelineConfig,
    tracking: &Tracking,
    sat: SatId,
    direct_code_phase_chips: f64,
    delayed_paths: &[DelayedPathProfile],
) -> Vec<MultipathBiasObservation> {
    let clean_frame = multipath_bias_frame(
        config,
        sat,
        direct_code_phase_chips,
        DelayedPathProfile { delay_chips: 0.0, relative_amplitude: 0.0, carrier_phase_rad: 0.0 },
    );
    let baseline = CleanMultipathBiasBaseline {
        standard_zero_chips: measured_code_zero_crossing_chips(
            config,
            tracking,
            &clean_frame,
            sat,
            direct_code_phase_chips,
            CodeBiasDiscriminator::StandardEarlyLate,
        ),
        narrow_zero_chips: measured_code_zero_crossing_chips(
            config,
            tracking,
            &clean_frame,
            sat,
            direct_code_phase_chips,
            CodeBiasDiscriminator::NarrowEarlyLate,
        ),
        double_delta_zero_chips: measured_code_zero_crossing_chips(
            config,
            tracking,
            &clean_frame,
            sat,
            direct_code_phase_chips,
            CodeBiasDiscriminator::DoubleDelta,
        ),
    };
    delayed_paths
        .iter()
        .copied()
        .map(|delayed_path| {
            multipath_bias_observation(
                config,
                tracking,
                sat,
                direct_code_phase_chips,
                baseline,
                delayed_path,
            )
        })
        .collect()
}

#[test]
fn narrow_correlator_reduces_delayed_path_code_bias() {
    let config = multipath_tracking_config();
    let sat = SatId { constellation: Constellation::Gps, prn: 7 };
    let tracking = Tracking::new(config.clone(), ReceiverRuntime::default());
    let direct_code_phase_chips = 245.25;
    let clean_frame = multipath_bias_frame(
        &config,
        sat,
        direct_code_phase_chips,
        DelayedPathProfile { delay_chips: 0.0, relative_amplitude: 0.0, carrier_phase_rad: 0.0 },
    );
    let clean_standard_zero = measured_early_late_zero_crossing_chips(
        &config,
        &tracking,
        &clean_frame,
        sat,
        direct_code_phase_chips,
        0.5,
    );
    let clean_narrow_zero = measured_early_late_zero_crossing_chips(
        &config,
        &tracking,
        &clean_frame,
        sat,
        direct_code_phase_chips,
        0.25,
    );
    let delayed_paths = [
        DelayedPathProfile { delay_chips: 0.45, relative_amplitude: 0.35, carrier_phase_rad: 0.0 },
        DelayedPathProfile { delay_chips: 0.65, relative_amplitude: 0.35, carrier_phase_rad: 0.0 },
    ];

    for delayed_path in delayed_paths {
        let frame = multipath_bias_frame(&config, sat, direct_code_phase_chips, delayed_path);
        let standard_zero_chips = measured_early_late_zero_crossing_chips(
            &config,
            &tracking,
            &frame,
            sat,
            direct_code_phase_chips,
            0.5,
        );
        let narrow_zero_chips = measured_early_late_zero_crossing_chips(
            &config,
            &tracking,
            &frame,
            sat,
            direct_code_phase_chips,
            0.25,
        );
        let standard_bias_chips = (standard_zero_chips - clean_standard_zero).abs();
        let narrow_bias_chips = (narrow_zero_chips - clean_narrow_zero).abs();

        assert!(
                standard_bias_chips > 0.0 && narrow_bias_chips < standard_bias_chips,
                "narrow correlator must reduce delayed-path code bias: delayed_path={delayed_path:?} standard_bias_chips={standard_bias_chips:.6} narrow_bias_chips={narrow_bias_chips:.6}",
            );
    }
}

#[test]
fn double_delta_multipath_bias_sweep_quantifies_benefit_and_failure_region() {
    let config = multipath_tracking_config();
    let sat = SatId { constellation: Constellation::Gps, prn: 7 };
    let tracking = Tracking::new(config.clone(), ReceiverRuntime::default());
    let direct_code_phase_chips = 245.25;
    let delayed_paths = [
        DelayedPathProfile { delay_chips: 0.20, relative_amplitude: 0.20, carrier_phase_rad: 0.0 },
        DelayedPathProfile {
            delay_chips: 0.20,
            relative_amplitude: 0.50,
            carrier_phase_rad: std::f32::consts::FRAC_PI_2,
        },
        DelayedPathProfile { delay_chips: 0.45, relative_amplitude: 0.20, carrier_phase_rad: 0.0 },
        DelayedPathProfile {
            delay_chips: 0.45,
            relative_amplitude: 0.35,
            carrier_phase_rad: std::f32::consts::FRAC_PI_2,
        },
        DelayedPathProfile {
            delay_chips: 0.45,
            relative_amplitude: 0.50,
            carrier_phase_rad: std::f32::consts::PI,
        },
        DelayedPathProfile { delay_chips: 0.65, relative_amplitude: 0.35, carrier_phase_rad: 0.0 },
        DelayedPathProfile {
            delay_chips: 0.65,
            relative_amplitude: 0.50,
            carrier_phase_rad: std::f32::consts::PI,
        },
    ];

    let observations = multipath_bias_observations(
        &config,
        &tracking,
        sat,
        direct_code_phase_chips,
        &delayed_paths,
    );
    let benefit_count = observations
        .iter()
        .filter(|observation| observation.double_delta_bias_chips < observation.narrow_bias_chips)
        .count();
    let failure_count = observations
        .iter()
        .filter(|observation| observation.double_delta_bias_chips > observation.narrow_bias_chips)
        .count();
    let neutral_count = observations.len() - benefit_count - failure_count;
    let standard_biased_count =
        observations.iter().filter(|observation| observation.standard_bias_chips > 0.0).count();

    assert_eq!(observations.len(), 7);
    assert_eq!(benefit_count, 1, "observations={observations:#?}");
    assert_eq!(failure_count, 4, "observations={observations:#?}");
    assert_eq!(neutral_count, 2, "observations={observations:#?}");
    assert_eq!(standard_biased_count, 3, "observations={observations:#?}");
    assert!(observations.iter().any(|observation| {
        (observation.delayed_path.delay_chips - 0.45).abs() <= f64::EPSILON
            && (observation.delayed_path.relative_amplitude - 0.50).abs() <= f32::EPSILON
            && (observation.delayed_path.carrier_phase_rad - std::f32::consts::PI).abs()
                <= f32::EPSILON
            && observation.double_delta_bias_chips < observation.narrow_bias_chips
    }));
    assert!(observations.iter().any(|observation| {
        (observation.delayed_path.delay_chips - 0.65).abs() <= f64::EPSILON
            && (observation.delayed_path.relative_amplitude - 0.50).abs() <= f32::EPSILON
            && (observation.delayed_path.carrier_phase_rad - std::f32::consts::PI).abs()
                <= f32::EPSILON
            && observation.double_delta_bias_chips > observation.narrow_bias_chips
    }));
}

#[test]
fn tracking_dll_discriminator_uses_double_delta_outer_pair() {
    let correlation = super::TrackingEpochCorrelation {
        primary: super::CorrelatorOutput {
            early: Complex::new(8.0, 0.0),
            prompt: Complex::new(10.0, 0.0),
            late: Complex::new(4.0, 0.0),
            early_late_noise_weight_energy: 0.0,
        },
        double_delta_outer: Some(super::CorrelatorOutput {
            early: Complex::new(6.0, 0.0),
            prompt: Complex::new(10.0, 0.0),
            late: Complex::new(2.0, 0.0),
            early_late_noise_weight_energy: 0.0,
        }),
        carrier_prompt: Complex::new(10.0, 0.0),
        carrier_prompt_source: super::CarrierPromptSource::Primary,
        data_prompt: None,
        secondary_code_prompt_period_index: 0,
        subcarrier_ambiguity_guard: None,
    };

    assert!((super::tracking_dll_discriminator(&correlation) - 0.166_666_67).abs() <= f32::EPSILON);
}

#[test]
fn tracking_dll_discriminator_keeps_inner_pair_when_double_delta_is_larger() {
    let correlation = super::TrackingEpochCorrelation {
        primary: super::CorrelatorOutput {
            early: Complex::new(5.0, 0.0),
            prompt: Complex::new(10.0, 0.0),
            late: Complex::new(5.0, 0.0),
            early_late_noise_weight_energy: 0.0,
        },
        double_delta_outer: Some(super::CorrelatorOutput {
            early: Complex::new(2.0, 0.0),
            prompt: Complex::new(10.0, 0.0),
            late: Complex::new(8.0, 0.0),
            early_late_noise_weight_energy: 0.0,
        }),
        carrier_prompt: Complex::new(10.0, 0.0),
        carrier_prompt_source: super::CarrierPromptSource::Primary,
        data_prompt: None,
        secondary_code_prompt_period_index: 0,
        subcarrier_ambiguity_guard: None,
    };

    assert_eq!(super::tracking_dll_discriminator(&correlation), 0.0);
}
