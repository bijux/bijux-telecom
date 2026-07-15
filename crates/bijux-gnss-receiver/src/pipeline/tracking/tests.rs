    use super::{ChannelState, Tracking};
    use crate::engine::receiver_config::{
        BandTrackingSpec, ReceiverPipelineConfig, TrackingParams,
    };
    use crate::engine::runtime::ReceiverRuntime;
    use crate::pipeline::observations::observations_from_tracking_results_with_gps_anchor;
    use crate::sim::synthetic::{generate_l1_ca, SyntheticSignalParams};
    use bijux_gnss_core::api::{
        AcqHypothesis, AcqUncertainty, Chips, Constellation, Epoch, GpsTime, Hertz,
        ReceiverSampleTrace, SampleTime, SamplesFrame, SatId, Seconds, SignalBand, SignalCode,
        SignalComponentRole, TrackEpoch, GPS_L1_CA_CARRIER_HZ,
    };
    use bijux_gnss_signal::api::TrackingLoopProfile as SignalTrackingLoopProfile;
    use bijux_gnss_signal::api::{
        advance_code_phase_seconds, delay_lock_loop_coefficients, discriminators,
        first_order_angular_loop_coefficients, normalize_dll_discriminator,
        phase_lock_loop_coefficients, sample_ca_code, sample_galileo_e1_cboc, samples_per_code,
        shared_path_code_rate_hz, signal_spec_gps_l5_i, wrapped_code_phase_delta_samples,
        LocalCodeModel, Prn,
    };
    use num_complex::Complex;
    use serde::Deserialize;

    fn vector_measurement(
        channel_id: u8,
        sample_index: u64,
        cn0_dbhz: f64,
        fll_error_hz: f64,
        code_rate_error_hz: f64,
        carrier_rate_hz_per_s: f64,
    ) -> super::VectorTrackingMeasurement {
        super::VectorTrackingMeasurement {
            sat: SatId { constellation: Constellation::Gps, prn: channel_id + 1 },
            channel_id,
            epoch_idx: sample_index / 5_000,
            sample_index,
            cn0_dbhz,
            dll_error_samples: 0.05,
            pll_error_rad: 0.02,
            fll_error_hz,
            code_rate_error_hz,
            carrier_rate_hz_per_s,
            prompt_locked: true,
            dll_locked: true,
            pll_locked: true,
            fll_locked: true,
            channel_state: ChannelState::Tracking,
        }
    }

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
        let sample_count = samples_per_code(
            config.sampling_freq_hz,
            config.code_freq_basis_hz,
            config.code_length,
        );
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
            let correlation = tracking.tracking_epoch_correlation(
                frame,
                0,
                frame.len(),
                frame.t0.sample_index,
                &signal_model,
                0.0,
                0.0,
                config.code_freq_basis_hz,
                direct_code_phase_samples + offset_chips * samples_per_chip,
                early_late_spacing_chips,
            );
            let dll_err = match discriminator {
                CodeBiasDiscriminator::DoubleDelta => {
                    super::tracking_dll_discriminator(&correlation)
                }
                CodeBiasDiscriminator::StandardEarlyLate
                | CodeBiasDiscriminator::NarrowEarlyLate => {
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
            double_delta_bias_chips: (double_delta_zero_chips - baseline.double_delta_zero_chips)
                .abs(),
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
            DelayedPathProfile {
                delay_chips: 0.0,
                relative_amplitude: 0.0,
                carrier_phase_rad: 0.0,
            },
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
            DelayedPathProfile {
                delay_chips: 0.0,
                relative_amplitude: 0.0,
                carrier_phase_rad: 0.0,
            },
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
            DelayedPathProfile {
                delay_chips: 0.45,
                relative_amplitude: 0.35,
                carrier_phase_rad: 0.0,
            },
            DelayedPathProfile {
                delay_chips: 0.65,
                relative_amplitude: 0.35,
                carrier_phase_rad: 0.0,
            },
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
            DelayedPathProfile {
                delay_chips: 0.20,
                relative_amplitude: 0.20,
                carrier_phase_rad: 0.0,
            },
            DelayedPathProfile {
                delay_chips: 0.20,
                relative_amplitude: 0.50,
                carrier_phase_rad: std::f32::consts::FRAC_PI_2,
            },
            DelayedPathProfile {
                delay_chips: 0.45,
                relative_amplitude: 0.20,
                carrier_phase_rad: 0.0,
            },
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
            DelayedPathProfile {
                delay_chips: 0.65,
                relative_amplitude: 0.35,
                carrier_phase_rad: 0.0,
            },
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
            .filter(|observation| {
                observation.double_delta_bias_chips < observation.narrow_bias_chips
            })
            .count();
        let failure_count = observations
            .iter()
            .filter(|observation| {
                observation.double_delta_bias_chips > observation.narrow_bias_chips
            })
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

        assert!(
            (super::tracking_dll_discriminator(&correlation) - 0.166_666_67).abs() <= f32::EPSILON
        );
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

    fn galileo_e1_subcarrier_guard_config() -> ReceiverPipelineConfig {
        ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 4092,
            channels: 1,
            early_late_spacing_chips: 0.5,
            dll_bw_hz: 2.0,
            pll_bw_hz: 15.0,
            fll_bw_hz: 10.0,
            ..ReceiverPipelineConfig::default()
        }
    }

    fn galileo_e1_subcarrier_guard_for_seed(
        config: &ReceiverPipelineConfig,
        sat: SatId,
        code_phase_chips: f64,
        seed_offset_chips: f64,
    ) -> super::SubcarrierAmbiguityGuard {
        let sample_count = samples_per_code(
            config.sampling_freq_hz,
            config.code_freq_basis_hz,
            config.code_length,
        );
        let frame = SamplesFrame::new(
            SampleTime { sample_index: 0, sample_rate_hz: config.sampling_freq_hz },
            Seconds(1.0 / config.sampling_freq_hz),
            sample_galileo_e1_cboc(
                sat.prn,
                config.sampling_freq_hz,
                code_phase_chips,
                sample_count,
                0,
                1,
            )
            .expect("Galileo E1 CBOC samples")
            .into_iter()
            .map(|value| Complex::new(value, 0.0))
            .collect(),
        );
        let signal_model = super::TrackingSignalModel::for_sat_signal_band(
            config,
            sat,
            SignalBand::E1,
            SignalCode::E1B,
            None,
        );
        let tracking = Tracking::new(config.clone(), ReceiverRuntime::default());
        let samples_per_chip = config.sampling_freq_hz / config.code_freq_basis_hz;
        let code_phase_samples = (code_phase_chips + seed_offset_chips) * samples_per_chip;
        tracking
            .tracking_epoch_correlation(
                &frame,
                0,
                frame.len(),
                0,
                &signal_model,
                0.0,
                0.0,
                config.code_freq_basis_hz,
                code_phase_samples,
                config.early_late_spacing_chips,
            )
            .subcarrier_ambiguity_guard
            .expect("Galileo E1 requires subcarrier ambiguity guard")
    }

    fn gps_l5q_secondary_code_component() -> super::TrackingComponentModel {
        super::TrackingComponentModel {
            role: SignalComponentRole::Pilot,
            code_length: 10_230,
            phase_transition_source: super::TrackingPhaseTransitionSource::SecondaryCode,
            local_code_model: super::TrackingComponentLocalCodeModel::Local(
                LocalCodeModel::gps_l5_q(18).expect("GPS L5Q local code"),
            ),
        }
    }

    fn secondary_code_prompt_history(
        component: &super::TrackingComponentModel,
        phase_periods: usize,
        observed_periods: usize,
        prompt: Complex<f32>,
    ) -> Vec<super::SecondaryCodePromptSample> {
        (0..observed_periods)
            .map(|period_offset| {
                let primary_code_period_index = phase_periods + period_offset;
                assert!(component.secondary_code_symbol(primary_code_period_index).is_some());
                super::SecondaryCodePromptSample { primary_code_period_index, prompt }
            })
            .collect()
    }

    #[test]
    fn secondary_code_sync_accepts_best_likelihood_phase() {
        let component = gps_l5q_secondary_code_component();
        let history = secondary_code_prompt_history(&component, 7, 20, Complex::new(2.0, 0.25));

        let sync =
            super::secondary_code_sync_from_prompt_history(&component, &history).expect("sync");

        assert!(sync.accepted, "sync result should be accepted: {sync:?}");
        assert_eq!(sync.phase_periods, 7);
        assert_eq!(sync.observed_periods, 20);
        assert!(sync.best_likelihood > sync.next_best_likelihood);
        assert!(sync.confidence >= super::SECONDARY_CODE_SYNC_MIN_CONFIDENCE);
    }

    #[test]
    fn secondary_code_sync_scores_incorrect_phase_lower() {
        let component = gps_l5q_secondary_code_component();
        let history = secondary_code_prompt_history(&component, 11, 20, Complex::new(1.5, -0.5));

        let correct = super::secondary_code_phase_score(&component, &history, 11);
        let incorrect = super::secondary_code_phase_score(&component, &history, 12);

        assert!(correct.likelihood > incorrect.likelihood);
    }

    #[test]
    fn secondary_code_sync_waits_for_enough_observations() {
        let component = gps_l5q_secondary_code_component();
        let history = secondary_code_prompt_history(&component, 3, 3, Complex::new(1.0, 0.0));

        assert!(super::secondary_code_sync_from_prompt_history(&component, &history).is_none());
    }

    #[test]
    fn secondary_code_sync_rejects_zero_energy_prompt_history() {
        let component = gps_l5q_secondary_code_component();
        let history = secondary_code_prompt_history(&component, 5, 20, Complex::new(0.0, 0.0));

        let sync =
            super::secondary_code_sync_from_prompt_history(&component, &history).expect("sync");

        assert!(!sync.accepted);
        assert_eq!(sync.confidence, 0.0);
    }

    #[test]
    fn secondary_code_sync_updates_from_pilot_carrier_prompt_history() {
        let config = ReceiverPipelineConfig::default();
        let signal_model = super::TrackingSignalModel::for_sat_signal_band(
            &config,
            SatId { constellation: Constellation::Gps, prn: 18 },
            SignalBand::L5,
            SignalCode::L5I,
            None,
        );
        let mut state = empty_loop_state();

        for period_offset in 0..20 {
            super::update_secondary_code_synchronization(
                &signal_model,
                &mut state,
                7 + period_offset,
                Complex::new(1.0, 0.125),
            );
        }

        let sync = state.secondary_code_sync.expect("secondary code sync");
        assert!(sync.accepted, "pilot prompt sync should be accepted: {sync:?}");
        assert_eq!(sync.phase_periods, 7);
        assert_eq!(sync.observed_periods, 20);
        assert_eq!(state.secondary_code_prompt_history.len(), 20);
    }

    #[test]
    fn secondary_code_sync_accepts_galileo_e5_pilot_phase() {
        let config = ReceiverPipelineConfig::default();
        for (signal_code, phase_periods) in [(SignalCode::E5a, 37), (SignalCode::E5b, 61)] {
            let signal_model = super::TrackingSignalModel::for_sat_signal_band(
                &config,
                SatId { constellation: Constellation::Galileo, prn: 11 },
                SignalBand::E5,
                signal_code,
                None,
            );
            let component = signal_model.carrier_component();
            let history = secondary_code_prompt_history(
                &component,
                phase_periods,
                100,
                Complex::new(1.0, 0.2),
            );

            let sync =
                super::secondary_code_sync_from_prompt_history(&component, &history).expect("sync");

            assert!(sync.accepted, "Galileo {signal_code:?} sync should be accepted: {sync:?}");
            assert_eq!(sync.phase_periods, phase_periods);
            assert!(sync.best_likelihood > sync.next_best_likelihood);
        }
    }

    #[test]
    fn secondary_code_sync_ignores_signals_without_secondary_code() {
        let config = ReceiverPipelineConfig::default();
        let signal_model = super::TrackingSignalModel::for_sat(
            &config,
            SatId { constellation: Constellation::Gps, prn: 1 },
        );
        let mut state = empty_loop_state();

        let sync = super::update_secondary_code_synchronization(
            &signal_model,
            &mut state,
            0,
            Complex::new(1.0, 0.0),
        );

        assert!(sync.is_none());
        assert!(state.secondary_code_sync.is_none());
        assert!(state.secondary_code_prompt_history.is_empty());
    }

    #[test]
    fn secondary_code_sync_provenance_reports_accepted_phase() {
        let config = ReceiverPipelineConfig::default();
        let signal_model = super::TrackingSignalModel::for_sat_signal_band(
            &config,
            SatId { constellation: Constellation::Gps, prn: 18 },
            SignalBand::L5,
            SignalCode::L5I,
            None,
        );
        let sync = super::SecondaryCodeSyncResult {
            phase_periods: 7,
            confidence: 0.5,
            best_likelihood: 1.0,
            next_best_likelihood: 0.5,
            observed_periods: 20,
            accepted: true,
        };

        let provenance = super::secondary_code_sync_provenance(&signal_model, Some(sync))
            .expect("secondary-code provenance");

        assert!(provenance.contains("secondary_code_sync=accepted"));
        assert!(provenance.contains("secondary_code_phase_periods=7"));
        assert!(provenance.contains("secondary_code_sync_confidence=0.500000"));
        assert!(provenance.contains("secondary_code_observed_periods=20"));
    }

    #[test]
    fn secondary_code_sync_provenance_reports_insufficient_evidence() {
        let config = ReceiverPipelineConfig::default();
        let signal_model = super::TrackingSignalModel::for_sat_signal_band(
            &config,
            SatId { constellation: Constellation::Gps, prn: 18 },
            SignalBand::L5,
            SignalCode::L5I,
            None,
        );

        let provenance =
            super::secondary_code_sync_provenance(&signal_model, None).expect("provenance");

        assert_eq!(provenance, " secondary_code_sync=insufficient");
    }

    #[test]
    fn secondary_code_sync_keeps_accepted_phase_over_weaker_candidate() {
        let accepted = super::SecondaryCodeSyncResult {
            phase_periods: 7,
            confidence: 0.25,
            best_likelihood: 1.0,
            next_best_likelihood: 0.75,
            observed_periods: 20,
            accepted: true,
        };
        let rejected = super::SecondaryCodeSyncResult {
            phase_periods: 3,
            confidence: 0.01,
            best_likelihood: 0.6,
            next_best_likelihood: 0.594,
            observed_periods: 20,
            accepted: false,
        };

        let selected = super::select_secondary_code_synchronization(Some(accepted), Some(rejected))
            .expect("selected sync");

        assert_eq!(selected, accepted);
    }

    #[test]
    fn secondary_code_sync_replaces_rejected_phase_with_accepted_candidate() {
        let rejected = super::SecondaryCodeSyncResult {
            phase_periods: 3,
            confidence: 0.01,
            best_likelihood: 0.6,
            next_best_likelihood: 0.594,
            observed_periods: 20,
            accepted: false,
        };
        let accepted = super::SecondaryCodeSyncResult {
            phase_periods: 7,
            confidence: 0.25,
            best_likelihood: 1.0,
            next_best_likelihood: 0.75,
            observed_periods: 20,
            accepted: true,
        };

        let selected = super::select_secondary_code_synchronization(Some(rejected), Some(accepted))
            .expect("selected sync");

        assert_eq!(selected, accepted);
    }

    #[test]
    fn prompt_center_period_index_follows_code_phase_across_epoch_boundary() {
        assert_eq!(
            super::prompt_center_primary_code_period_index(0, 100.0, 1.0, 10_230, 10_230),
            0
        );
        assert_eq!(
            super::prompt_center_primary_code_period_index(0, 8_182.0, 1.0, 10_230, 10_230),
            1
        );
    }

    #[cfg(feature = "nav")]
    fn gps_l1ca_tracking_signal_model() -> super::TrackingSignalModel {
        let config = ReceiverPipelineConfig::default();
        super::TrackingSignalModel::for_sat(
            &config,
            SatId { constellation: Constellation::Gps, prn: 1 },
        )
    }

    #[cfg(feature = "nav")]
    fn prompt_history_epochs(prompt_i: &[f32]) -> Vec<TrackEpoch> {
        prompt_i
            .iter()
            .enumerate()
            .map(|(index, prompt_i)| TrackEpoch {
                epoch: Epoch { index: index as u64 },
                sample_index: index as u64,
                source_time: ReceiverSampleTrace::from_sample_index(index as u64, 1000.0),
                sat: SatId { constellation: Constellation::Gps, prn: 1 },
                prompt_i: *prompt_i,
                tracking_provenance: "tracking".to_string(),
                ..TrackEpoch::default()
            })
            .collect()
    }

    #[cfg(feature = "nav")]
    fn set_lnav_bits(data: &mut u32, start: usize, len: usize, value: u32) {
        let shift = 24 - (start - 1) - len;
        let mask = ((1_u32 << len) - 1) << shift;
        *data &= !mask;
        *data |= (value << shift) & mask;
    }

    #[cfg(feature = "nav")]
    fn lnav_word_parity(data_bits: &[u8], d29_star: u8, d30_star: u8) -> (u8, u8, u8, u8, u8, u8) {
        let d = |i: usize| -> u8 { data_bits[i - 1] };
        let p1 = d(1)
            ^ d(2)
            ^ d(3)
            ^ d(5)
            ^ d(6)
            ^ d(10)
            ^ d(11)
            ^ d(12)
            ^ d(13)
            ^ d(14)
            ^ d(15)
            ^ d(17)
            ^ d(18)
            ^ d(20)
            ^ d(23)
            ^ d(24)
            ^ d29_star;
        let p2 = d(2)
            ^ d(3)
            ^ d(4)
            ^ d(6)
            ^ d(7)
            ^ d(11)
            ^ d(12)
            ^ d(13)
            ^ d(14)
            ^ d(15)
            ^ d(16)
            ^ d(18)
            ^ d(19)
            ^ d(21)
            ^ d(24)
            ^ d29_star
            ^ d30_star;
        let p3 = d(1)
            ^ d(3)
            ^ d(4)
            ^ d(5)
            ^ d(7)
            ^ d(8)
            ^ d(12)
            ^ d(13)
            ^ d(14)
            ^ d(15)
            ^ d(16)
            ^ d(17)
            ^ d(19)
            ^ d(20)
            ^ d(22)
            ^ d29_star
            ^ d30_star;
        let p4 = d(1)
            ^ d(2)
            ^ d(4)
            ^ d(5)
            ^ d(6)
            ^ d(8)
            ^ d(9)
            ^ d(13)
            ^ d(14)
            ^ d(15)
            ^ d(16)
            ^ d(17)
            ^ d(18)
            ^ d(20)
            ^ d(21)
            ^ d(23)
            ^ d30_star;
        let p5 = d(1)
            ^ d(2)
            ^ d(3)
            ^ d(5)
            ^ d(6)
            ^ d(7)
            ^ d(9)
            ^ d(10)
            ^ d(14)
            ^ d(15)
            ^ d(16)
            ^ d(17)
            ^ d(18)
            ^ d(19)
            ^ d(21)
            ^ d(22)
            ^ d(24)
            ^ d29_star;
        let p6 = d(2)
            ^ d(3)
            ^ d(4)
            ^ d(6)
            ^ d(7)
            ^ d(8)
            ^ d(10)
            ^ d(11)
            ^ d(15)
            ^ d(16)
            ^ d(17)
            ^ d(18)
            ^ d(19)
            ^ d(20)
            ^ d(22)
            ^ d(23)
            ^ d(24)
            ^ d30_star;
        (p1, p2, p3, p4, p5, p6)
    }

    #[cfg(feature = "nav")]
    fn encode_lnav_word(data: u32, prev_d29: u8, prev_d30: u8) -> [u8; 30] {
        let mut bits = [0_u8; 30];
        for (index, bit) in bits.iter_mut().enumerate().take(24) {
            let shift = 23 - index;
            *bit = ((data >> shift) & 1) as u8;
        }
        let (p1, p2, p3, p4, p5, p6) = lnav_word_parity(&bits[..24], prev_d29, prev_d30);
        bits[24] = p1;
        bits[25] = p2;
        bits[26] = p3;
        bits[27] = p4;
        bits[28] = p5;
        bits[29] = p6;
        if prev_d30 == 1 {
            for bit in &mut bits {
                *bit = 1 - *bit;
            }
        }
        bits
    }

    #[cfg(feature = "nav")]
    fn encode_lnav_subframe(subframe_id: u8, tow_count: u32) -> Vec<i8> {
        let mut tlm = 0_u32;
        set_lnav_bits(&mut tlm, 1, 8, 0x8B);

        let mut how = 0_u32;
        set_lnav_bits(&mut how, 1, 17, tow_count);
        set_lnav_bits(&mut how, 20, 3, subframe_id as u32);

        let mut words = vec![tlm, how];
        for offset in 0..8_u32 {
            words.push(0x012345 + offset * 0x010101);
        }

        let mut prev_d29 = 0_u8;
        let mut prev_d30 = 0_u8;
        let mut bits = Vec::with_capacity(300);
        for data in words {
            let encoded = encode_lnav_word(data, prev_d29, prev_d30);
            prev_d29 = encoded[28];
            prev_d30 = encoded[29];
            bits.extend(encoded.into_iter().map(|bit| if bit == 1 { 1 } else { -1 }));
        }
        bits
    }

    #[cfg(feature = "nav")]
    fn lnav_prompt_history_with_offset(
        offset_ms: usize,
        subframe_id: u8,
        tow_count: u32,
    ) -> Vec<f32> {
        let mut prompt = vec![0.25_f32; offset_ms];
        for bit in encode_lnav_subframe(subframe_id, tow_count) {
            prompt.extend(std::iter::repeat_n(bit as f32, 20));
        }
        prompt
    }

    #[test]
    fn tracking_recovery_from_loss_of_lock() {
        let lost = Tracking::transition_state(1, ChannelState::Tracking, ChannelState::Lost);
        assert_eq!(lost, ChannelState::Lost);
        let pull_in = Tracking::transition_state(1, lost, ChannelState::PullIn);
        assert_eq!(pull_in, ChannelState::PullIn);
        let tracking = Tracking::transition_state(1, pull_in, ChannelState::Tracking);
        assert_eq!(tracking, ChannelState::Tracking);
    }

    #[test]
    fn tracking_reset_after_gap() {
        let lost = Tracking::transition_state(2, ChannelState::Tracking, ChannelState::Lost);
        let reset = Tracking::transition_state(2, lost, ChannelState::Idle);
        assert_eq!(reset, ChannelState::Idle);
    }

    #[test]
    fn ambiguous_hypothesis_is_degraded_for_tracking() {
        assert_eq!(super::acq_to_track_state(&AcqHypothesis::Accepted), "accepted");
        assert_eq!(super::acq_to_track_state(&AcqHypothesis::Ambiguous), "degraded");
        assert_eq!(super::acq_to_track_state(&AcqHypothesis::Rejected), "rejected");
        assert_eq!(super::acq_to_track_state(&AcqHypothesis::Deferred), "deferred");
    }

    #[cfg(feature = "nav")]
    #[test]
    fn annotate_navigation_bit_signs_waits_for_boundary_confidence() {
        let signal_model = gps_l1ca_tracking_signal_model();
        let prompt = vec![1.0_f32; 60];
        let mut epochs = prompt_history_epochs(&prompt);

        super::annotate_navigation_bit_signs(&signal_model, &mut epochs);

        assert!(
            epochs.iter().all(|epoch| epoch.navigation_bit_sign.is_none()),
            "ambiguous symbol boundaries must not emit bit signs: {epochs:?}"
        );
        assert!(epochs.iter().all(|epoch| !epoch.nav_bit_lock));
        assert!(epochs
            .iter()
            .all(|epoch| !epoch.tracking_provenance.contains("nav_symbol_sync=confident")));
    }

    #[cfg(feature = "nav")]
    #[test]
    fn annotate_navigation_bit_signs_emits_confident_symbol_windows() {
        let signal_model = gps_l1ca_tracking_signal_model();
        let mut prompt = vec![0.2_f32; 6];
        prompt.extend(std::iter::repeat_n(1.0_f32, 20));
        prompt.extend(std::iter::repeat_n(-1.0_f32, 20));
        prompt.extend(std::iter::repeat_n(1.0_f32, 20));
        let mut epochs = prompt_history_epochs(&prompt);

        super::annotate_navigation_bit_signs(&signal_model, &mut epochs);

        assert!(epochs[..6].iter().all(|epoch| epoch.navigation_bit_sign.is_none()));
        assert!(epochs[6..26]
            .iter()
            .all(|epoch| epoch.navigation_bit_sign == Some(1) && epoch.nav_bit_lock));
        assert!(epochs[26..46]
            .iter()
            .all(|epoch| epoch.navigation_bit_sign == Some(-1) && epoch.nav_bit_lock));
        assert!(epochs[46..66]
            .iter()
            .all(|epoch| epoch.navigation_bit_sign == Some(1) && epoch.nav_bit_lock));
        assert!(epochs[6..66]
            .iter()
            .all(|epoch| epoch.tracking_provenance.contains("nav_symbol_sync=confident")));
    }

    #[cfg(feature = "nav")]
    #[test]
    fn annotate_lnav_transmit_times_maps_decoded_how_to_tracking_epochs() {
        let signal_model = gps_l1ca_tracking_signal_model();
        let tow_count = 57_600;
        let prompt = lnav_prompt_history_with_offset(7, 1, tow_count);
        let mut epochs = prompt_history_epochs(&prompt);

        super::annotate_lnav_transmit_times(
            &signal_model,
            Some(GpsTime { week: 2200, tow_s: 345_600.073 }),
            &mut epochs,
        );

        assert!(epochs[..7].iter().all(|epoch| epoch.transmit_time.is_none()));
        let subframe_start = epochs[7].transmit_time.as_ref().expect("subframe start time");
        assert_eq!(subframe_start.source, "gps_l1ca_lnav_how");
        assert_eq!(subframe_start.transmit_gps_time.week, 2200);
        assert!((subframe_start.transmit_gps_time.tow_s - 345_600.0).abs() <= 1.0e-12);

        let later = epochs[257].transmit_time.as_ref().expect("offset transmit time");
        assert!((later.transmit_gps_time.tow_s - 345_600.250).abs() <= 1.0e-12);
        assert!(epochs[257].tracking_provenance.contains("lnav_transmit_time=decoded"));
    }

    #[cfg(feature = "nav")]
    #[test]
    fn lnav_transmit_time_week_resolution_tracks_receive_time_boundary() {
        let epoch = TrackEpoch {
            source_time: ReceiverSampleTrace::from_sample_index(20, 1000.0),
            ..TrackEpoch::default()
        };

        let transmit_time = super::gps_transmit_time_near_receive_time(
            GpsTime { week: 2201, tow_s: 0.010 },
            &epoch,
            604_799.950,
        );

        assert_eq!(transmit_time.week, 2200);
        assert!((transmit_time.tow_s - 604_799.950).abs() <= 1.0e-12);
    }

    #[cfg(feature = "nav")]
    #[test]
    fn decoded_lnav_tracking_time_feeds_absolute_observation_pseudorange() {
        let signal_model = gps_l1ca_tracking_signal_model();
        let prompt = lnav_prompt_history_with_offset(7, 1, 57_600);
        let mut epochs = prompt_history_epochs(&prompt);
        let tracking_capture_start = GpsTime { week: 2200, tow_s: 345_600.073 };
        super::annotate_lnav_transmit_times(
            &signal_model,
            Some(tracking_capture_start),
            &mut epochs,
        );

        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            ..ReceiverPipelineConfig::default()
        };
        let observation_capture_start = GpsTime { week: 2200, tow_s: 345_600.0 };
        let receive_gps_time = GpsTime { week: 2200, tow_s: 345_600.330 };
        let mut epoch = epochs[257].clone();
        epoch.sample_index = ((receive_gps_time.tow_s - observation_capture_start.tow_s)
            * config.sampling_freq_hz)
            .round() as u64;
        epoch.source_time =
            ReceiverSampleTrace::from_sample_index(epoch.sample_index, config.sampling_freq_hz);
        epoch.sat = SatId { constellation: Constellation::Gps, prn: 1 };
        epoch.signal_band = SignalBand::L1;
        epoch.signal_code = SignalCode::Ca;
        epoch.code_rate_hz = Hertz(1_023_000.0);
        epoch.carrier_hz = Hertz(0.0);
        epoch.code_phase_samples = Chips(0.0);
        epoch.lock = true;
        epoch.dll_lock = true;
        epoch.pll_lock = true;
        epoch.fll_lock = true;
        epoch.signal_delay_alignment = None;

        let track = super::TrackingResult {
            sat: epoch.sat,
            carrier_hz: epoch.carrier_hz.0,
            code_phase_samples: epoch.code_phase_samples.0,
            acquisition_hypothesis: "accepted".to_string(),
            acquisition_score: 1.0,
            acquisition_code_phase_samples: 0,
            acquisition_carrier_hz: epoch.carrier_hz.0,
            acq_to_track_state: "accepted".to_string(),
            epochs: vec![epoch],
            transitions: Vec::new(),
        };

        let observations = observations_from_tracking_results_with_gps_anchor(
            &config,
            Some(observation_capture_start),
            &[track],
            10,
        );
        let sat = observations.output[0].sats.first().expect("decoded time observation");

        assert_eq!(sat.metadata.pseudorange_model, "decoded_transmit_time_code_phase");
        assert_eq!(sat.metadata.pseudorange_time_source, "gps_l1ca_lnav_how");
        assert_eq!(sat.metadata.pseudorange_integer_code_periods, Some(80));
        assert_eq!(sat.metadata.signal_delay_alignment_source, "");
        let timing = sat.timing.expect("decoded timing");
        assert!(
            (sat.pseudorange_m.0 - timing.signal_travel_time_s.0 * 299_792_458.0).abs() <= 1.0e-6
        );
        assert!((timing.signal_travel_time_s.0 - 0.080).abs() <= 2.5e-7);
    }

    #[test]
    fn deterministic_transition_rule_handles_cycle_slip_first() {
        let decision = super::deterministic_transition_rule(
            ChannelState::Tracking,
            false,
            false,
            false,
            Some(super::LossOfLockCause::PhaseJump),
            1,
            0,
            100,
            false,
            None,
        );
        assert_eq!(decision.to_state, ChannelState::Lost);
        assert_eq!(decision.reason, "phase_jump");
        assert_eq!(decision.next_unlocked_count, 2);
        assert_eq!(decision.next_degraded_epochs, 0);
    }

    #[test]
    fn deterministic_transition_rule_promotes_lock() {
        let decision = super::deterministic_transition_rule(
            ChannelState::PullIn,
            true,
            true,
            false,
            None,
            2,
            0,
            100,
            false,
            None,
        );
        assert_eq!(decision.to_state, ChannelState::Tracking);
        assert_eq!(decision.reason, "carrier_converged");
        assert_eq!(decision.next_unlocked_count, 0);
        assert_eq!(decision.next_degraded_epochs, 0);
    }

    #[test]
    fn deterministic_transition_rule_degrades_tracking_during_short_fade() {
        let decision = super::deterministic_transition_rule(
            ChannelState::Tracking,
            false,
            false,
            false,
            None,
            1,
            0,
            100,
            false,
            None,
        );
        assert_eq!(decision.to_state, ChannelState::Degraded);
        assert_eq!(decision.reason, "signal_fade");
        assert_eq!(decision.next_unlocked_count, 0);
        assert_eq!(decision.next_degraded_epochs, 1);
    }

    #[test]
    fn deterministic_transition_rule_reports_doppler_estimator_divergence() {
        let decision = super::deterministic_transition_rule(
            ChannelState::Tracking,
            true,
            false,
            false,
            None,
            0,
            0,
            100,
            false,
            Some("doppler_estimator_divergence"),
        );

        assert_eq!(decision.to_state, ChannelState::Degraded);
        assert_eq!(decision.reason, "doppler_estimator_divergence");
        assert_eq!(decision.next_unlocked_count, 0);
        assert_eq!(decision.next_degraded_epochs, 1);
    }

    #[test]
    fn deterministic_transition_rule_recovers_after_short_fade() {
        let decision = super::deterministic_transition_rule(
            ChannelState::Degraded,
            true,
            true,
            false,
            None,
            0,
            4,
            100,
            false,
            None,
        );
        assert_eq!(decision.to_state, ChannelState::Tracking);
        assert_eq!(decision.reason, "fade_recovered");
        assert_eq!(decision.next_unlocked_count, 0);
        assert_eq!(decision.next_degraded_epochs, 0);
    }

    #[test]
    fn deterministic_transition_rule_preserves_doppler_reason_while_degraded() {
        let decision = super::deterministic_transition_rule(
            ChannelState::Degraded,
            true,
            false,
            false,
            None,
            0,
            4,
            100,
            false,
            Some("doppler_estimator_divergence"),
        );

        assert_eq!(decision.to_state, ChannelState::Degraded);
        assert_eq!(decision.reason, "doppler_estimator_divergence");
        assert_eq!(decision.next_unlocked_count, 0);
        assert_eq!(decision.next_degraded_epochs, 5);
    }

    #[test]
    fn deterministic_transition_rule_keeps_degraded_state_during_fade_cycle_slip() {
        let decision = super::deterministic_transition_rule(
            ChannelState::Degraded,
            false,
            false,
            false,
            Some(super::LossOfLockCause::PhaseJump),
            0,
            2,
            100,
            false,
            None,
        );
        assert_eq!(decision.to_state, ChannelState::Lost);
        assert_eq!(decision.reason, "phase_jump");
        assert_eq!(decision.next_unlocked_count, 1);
        assert_eq!(decision.next_degraded_epochs, 0);
    }

    #[test]
    fn deterministic_transition_rule_marks_loss_after_fade_budget_exhaustion() {
        let decision = super::deterministic_transition_rule(
            ChannelState::Degraded,
            false,
            false,
            false,
            Some(super::LossOfLockCause::PromptPowerDrop),
            0,
            100,
            100,
            false,
            None,
        );
        assert_eq!(decision.to_state, ChannelState::Lost);
        assert_eq!(decision.reason, "prompt_power_drop");
        assert_eq!(decision.next_unlocked_count, 1);
        assert_eq!(decision.next_degraded_epochs, 0);
    }

    #[test]
    fn deterministic_transition_rule_grants_short_fade_grace_to_degraded_instability() {
        let decision = super::deterministic_transition_rule(
            ChannelState::Degraded,
            false,
            false,
            false,
            Some(super::LossOfLockCause::DiscriminatorInstability),
            0,
            super::DEGRADED_FADE_INSTABILITY_GRACE_EPOCHS,
            100,
            false,
            None,
        );
        assert_eq!(decision.to_state, ChannelState::Degraded);
        assert_eq!(decision.reason, "signal_fade");
        assert_eq!(
            decision.next_degraded_epochs,
            super::DEGRADED_FADE_INSTABILITY_GRACE_EPOCHS + 1
        );
    }

    #[test]
    fn classify_loss_of_lock_cause_prioritizes_phase_jump() {
        let cause = super::classify_loss_of_lock_cause(ChannelState::Tracking, true, Some(0.9), 3);

        assert_eq!(cause, Some(super::LossOfLockCause::PhaseJump));
    }

    #[test]
    fn classify_loss_of_lock_cause_treats_weak_prompt_cycle_slips_as_prompt_power_drop() {
        let cause = super::classify_loss_of_lock_cause(ChannelState::Tracking, true, Some(0.1), 0);

        assert_eq!(cause, Some(super::LossOfLockCause::PromptPowerDrop));
    }

    #[test]
    fn classify_loss_of_lock_cause_detects_prompt_power_drop() {
        let cause = super::classify_loss_of_lock_cause(ChannelState::Tracking, false, Some(0.1), 0);

        assert_eq!(cause, Some(super::LossOfLockCause::PromptPowerDrop));
    }

    #[test]
    fn classify_loss_of_lock_cause_detects_discriminator_instability() {
        let cause = super::classify_loss_of_lock_cause(ChannelState::Tracking, false, Some(0.8), 2);

        assert_eq!(cause, Some(super::LossOfLockCause::DiscriminatorInstability));
    }

    #[test]
    fn update_discriminator_instability_epochs_requires_strong_prompt() {
        let epochs = super::update_discriminator_instability_epochs(
            1,
            ChannelState::Tracking,
            Some(0.7),
            false,
            false,
            false,
            false,
        );
        assert_eq!(epochs, 2);

        let reset = super::update_discriminator_instability_epochs(
            epochs,
            ChannelState::Tracking,
            Some(0.1),
            false,
            false,
            false,
            false,
        );
        assert_eq!(reset, 0);
    }

    #[test]
    fn sustained_lock_loss_reacquire_seed_ignores_degraded_short_fade_epochs() {
        let epochs = vec![
            track_epoch_with_state(0, false, "degraded", Some("signal_fade")),
            track_epoch_with_state(1, false, "degraded", Some("signal_fade")),
            track_epoch_with_state(2, false, "degraded", Some("signal_fade")),
        ];

        assert_eq!(super::sustained_lock_loss_reacquire_seed(&epochs), None);
    }

    #[test]
    fn sustained_lock_loss_reacquire_seed_requires_three_lost_epochs() {
        let epochs = vec![
            track_epoch_with_state(0, false, "lost", Some("lock_lost")),
            track_epoch_with_state(1, false, "lost", Some("lock_lost")),
            track_epoch_with_state(2, false, "lost", Some("lock_lost")),
        ];

        assert_eq!(
            super::sustained_lock_loss_reacquire_seed(&epochs),
            Some(super::SustainedLockLossSeed {
                carrier_hz: 2.0,
                code_phase_samples: 2.5,
                code_rate_hz: 0.0,
                sample_index: 2,
            })
        );
    }

    #[test]
    fn sustained_lock_loss_reacquire_seed_accepts_explicit_loss_causes() {
        let epochs = vec![
            track_epoch_with_state(0, false, "lost", Some("prompt_power_drop")),
            track_epoch_with_state(1, false, "lost", Some("prompt_power_drop")),
            track_epoch_with_state(2, false, "lost", Some("prompt_power_drop")),
        ];

        assert_eq!(
            super::sustained_lock_loss_reacquire_seed(&epochs),
            Some(super::SustainedLockLossSeed {
                carrier_hz: 2.0,
                code_phase_samples: 2.5,
                code_rate_hz: 0.0,
                sample_index: 2,
            })
        );
    }

    #[test]
    fn sustained_lock_loss_reacquire_seed_allows_retry_after_failed_attempt() {
        let epochs = vec![
            track_epoch_with_state(0, false, "lost", Some("reacquisition_failed")),
            track_epoch_with_state(1, false, "lost", Some("reacquisition_failed")),
            track_epoch_with_state(2, false, "lost", Some("reacquisition_failed")),
        ];

        assert_eq!(
            super::sustained_lock_loss_reacquire_seed(&epochs),
            Some(super::SustainedLockLossSeed {
                carrier_hz: 2.0,
                code_phase_samples: 2.5,
                code_rate_hz: 0.0,
                sample_index: 2,
            })
        );
    }

    #[test]
    fn project_reacquisition_code_phase_samples_advances_loss_anchor_to_current_epoch() {
        let seed = super::SustainedLockLossSeed {
            carrier_hz: 1200.0,
            code_phase_samples: 5.5,
            code_rate_hz: 1_023_000.0,
            sample_index: 1_000,
        };

        let projected = super::project_reacquisition_code_phase_samples(
            seed,
            1_000 + 2 * 1_023,
            1_023_000.0,
            1_023,
        );

        assert_eq!(projected, 5.5);
    }

    fn reacquisition_candidate(
        doppler_bin: i8,
        code_bin: i8,
        carrier_sign: super::ReacquisitionCarrierSign,
        secondary_code_phase_periods: Option<usize>,
        cn0_dbhz: f64,
        prompt_power: f32,
    ) -> super::ReacquisitionCandidate {
        super::ReacquisitionCandidate {
            hypothesis: super::ReacquisitionHypothesis {
                doppler_bin,
                code_bin,
                carrier_sign,
                secondary_code_phase_periods,
            },
            carrier_hz: 1000.0 + f64::from(doppler_bin),
            code_phase_samples: 42.0 + f64::from(code_bin),
            cn0_dbhz,
            prompt_power,
        }
    }

    #[test]
    fn reacquisition_selection_chooses_strongest_hypothesis_not_first_peak() {
        let selection = super::select_reacquisition_candidate(
            &[
                reacquisition_candidate(
                    -2,
                    -2,
                    super::ReacquisitionCarrierSign::Aligned,
                    None,
                    39.0,
                    200.0,
                ),
                reacquisition_candidate(
                    1,
                    1,
                    super::ReacquisitionCarrierSign::Aligned,
                    None,
                    44.0,
                    300.0,
                ),
            ],
            35.0,
            0.0,
        );

        assert_eq!(
            selection,
            super::ReacquisitionSelection::Accepted(super::ReacquisitionSeed {
                carrier_hz: 1001.0,
                code_phase_samples: 43.0,
                cn0_dbhz: 44.0,
                carrier_sign: super::ReacquisitionCarrierSign::Aligned,
                secondary_code_phase_periods: None,
            })
        );
    }

    #[test]
    fn reacquisition_selection_refuses_ambiguous_hypotheses() {
        let selection = super::select_reacquisition_candidate(
            &[
                reacquisition_candidate(
                    0,
                    0,
                    super::ReacquisitionCarrierSign::Aligned,
                    None,
                    44.0,
                    300.0,
                ),
                reacquisition_candidate(
                    2,
                    2,
                    super::ReacquisitionCarrierSign::Inverted,
                    None,
                    43.0,
                    292.0,
                ),
            ],
            35.0,
            0.0,
        );

        assert_eq!(selection, super::ReacquisitionSelection::Refused);
    }

    #[test]
    fn reacquisition_selection_refuses_competing_secondary_code_phases() {
        let selection = super::select_reacquisition_candidate(
            &[
                reacquisition_candidate(
                    0,
                    0,
                    super::ReacquisitionCarrierSign::Aligned,
                    Some(3),
                    44.0,
                    300.0,
                ),
                reacquisition_candidate(
                    0,
                    0,
                    super::ReacquisitionCarrierSign::Aligned,
                    Some(4),
                    43.5,
                    294.0,
                ),
            ],
            35.0,
            0.0,
        );

        assert_eq!(selection, super::ReacquisitionSelection::Refused);
    }

    #[test]
    fn reacquisition_selection_treats_carrier_sign_only_tie_as_same_location() {
        let selection = super::select_reacquisition_candidate(
            &[
                reacquisition_candidate(
                    0,
                    0,
                    super::ReacquisitionCarrierSign::Aligned,
                    None,
                    44.0,
                    300.0,
                ),
                reacquisition_candidate(
                    0,
                    0,
                    super::ReacquisitionCarrierSign::Inverted,
                    None,
                    44.0,
                    300.0,
                ),
            ],
            35.0,
            0.0,
        );

        assert_eq!(
            selection,
            super::ReacquisitionSelection::Accepted(super::ReacquisitionSeed {
                carrier_hz: 1000.0,
                code_phase_samples: 42.0,
                cn0_dbhz: 44.0,
                carrier_sign: super::ReacquisitionCarrierSign::Aligned,
                secondary_code_phase_periods: None,
            })
        );
    }

    #[test]
    fn reacquisition_selection_preserves_aligned_sign_for_same_location() {
        let selection = super::select_reacquisition_candidate(
            &[
                reacquisition_candidate(
                    0,
                    0,
                    super::ReacquisitionCarrierSign::Aligned,
                    None,
                    44.0,
                    300.0,
                ),
                reacquisition_candidate(
                    0,
                    0,
                    super::ReacquisitionCarrierSign::Inverted,
                    None,
                    44.1,
                    301.0,
                ),
            ],
            35.0,
            0.0,
        );

        assert_eq!(
            selection,
            super::ReacquisitionSelection::Accepted(super::ReacquisitionSeed {
                carrier_hz: 1000.0,
                code_phase_samples: 42.0,
                cn0_dbhz: 44.0,
                carrier_sign: super::ReacquisitionCarrierSign::Aligned,
                secondary_code_phase_periods: None,
            })
        );
    }

    #[test]
    fn reacquisition_selection_accepts_strong_prompt_when_cn0_is_conservative() {
        let selection = super::select_reacquisition_candidate(
            &[reacquisition_candidate(
                0,
                0,
                super::ReacquisitionCarrierSign::Aligned,
                None,
                45.0,
                300_000.0,
            )],
            48.0,
            100_000.0,
        );

        assert_eq!(
            selection,
            super::ReacquisitionSelection::Accepted(super::ReacquisitionSeed {
                carrier_hz: 1000.0,
                code_phase_samples: 42.0,
                cn0_dbhz: 45.0,
                carrier_sign: super::ReacquisitionCarrierSign::Aligned,
                secondary_code_phase_periods: None,
            })
        );
    }

    #[test]
    fn reacquisition_selection_rejects_strong_prompt_when_cn0_is_too_low() {
        let selection = super::select_reacquisition_candidate(
            &[reacquisition_candidate(
                0,
                0,
                super::ReacquisitionCarrierSign::Aligned,
                None,
                41.0,
                300_000.0,
            )],
            48.0,
            100_000.0,
        );

        assert_eq!(selection, super::ReacquisitionSelection::Refused);
    }

    #[test]
    fn quick_reacquire_recovers_offset_code_hypothesis() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 1_023_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            ..ReceiverPipelineConfig::default()
        };
        let sat = SatId { constellation: Constellation::Gps, prn: 21 };
        let frame = generate_l1_ca(
            &config,
            SyntheticSignalParams {
                sat,
                glonass_frequency_channel: None,
                signal_band: SignalBand::L1,
                signal_code: SignalCode::Ca,
                doppler_hz: 0.0,
                code_phase_chips: 0.0,
                carrier_phase_rad: 0.0,
                cn0_db_hz: 70.0,
                navigation_data: false.into(),
            },
            0x51AC_0301,
            0.001,
        );
        let tracking = Tracking::new(config.clone(), ReceiverRuntime::default());

        let seed = tracking
            .quick_reacquire(&frame, sat, 0.0, 2.0, 35.0, 0.0, None)
            .expect("reacquisition seed");
        let samples_per_code = samples_per_code(
            config.sampling_freq_hz,
            config.code_freq_basis_hz,
            config.code_length,
        );
        let code_error =
            wrapped_code_phase_delta_samples(seed.code_phase_samples, 0.0, samples_per_code).abs();

        assert!(
            code_error <= 0.1,
            "reacquisition should recover the actual code hypothesis: seed={seed:?}, code_error={code_error}"
        );
        assert!((seed.carrier_hz - 0.0).abs() <= f64::EPSILON, "{seed:?}");
    }

    #[test]
    fn reacquisition_secondary_code_phases_collapse_when_signal_has_no_secondary_code() {
        let config = ReceiverPipelineConfig::default();
        let signal_model = super::TrackingSignalModel::for_sat_signal_band(
            &config,
            SatId { constellation: Constellation::Gps, prn: 7 },
            SignalBand::L1,
            SignalCode::Ca,
            None,
        );

        assert_eq!(super::reacquisition_secondary_code_phase_periods(&signal_model), vec![None]);
    }

    #[test]
    fn reacquisition_secondary_code_phases_cover_supported_period() {
        let config = ReceiverPipelineConfig::default();
        let signal_model = super::TrackingSignalModel::for_sat_signal_band(
            &config,
            SatId { constellation: Constellation::Gps, prn: 18 },
            SignalBand::L5,
            SignalCode::L5I,
            None,
        );
        let phases = super::reacquisition_secondary_code_phase_periods(&signal_model);

        assert!(phases.len() > 1);
        assert_eq!(phases.first(), Some(&Some(0)));
        assert_eq!(phases.last(), Some(&Some(phases.len() - 1)));
    }

    #[test]
    fn reacquisition_seed_matches_respects_acquisition_uncertainty_tolerances() {
        let tracking = Tracking::new(ReceiverPipelineConfig::default(), ReceiverRuntime::default());
        let uncertainty = AcqUncertainty {
            doppler_hz: 250.0,
            code_phase_samples: 0.75,
            doppler_rate_hz_per_s: None,
            covariance: None,
        };

        assert!(tracking.reacquisition_seed_matches(
            super::ReacquisitionSeed {
                carrier_hz: 1250.0,
                code_phase_samples: 42.0,
                cn0_dbhz: 36.0,
                carrier_sign: super::ReacquisitionCarrierSign::Aligned,
                secondary_code_phase_periods: None,
            },
            super::ReacquisitionSeed {
                carrier_hz: 1400.0,
                code_phase_samples: 42.5,
                cn0_dbhz: 34.0,
                carrier_sign: super::ReacquisitionCarrierSign::Aligned,
                secondary_code_phase_periods: None,
            },
            Some(&uncertainty),
        ));
        assert!(!tracking.reacquisition_seed_matches(
            super::ReacquisitionSeed {
                carrier_hz: 1250.0,
                code_phase_samples: 42.0,
                cn0_dbhz: 36.0,
                carrier_sign: super::ReacquisitionCarrierSign::Aligned,
                secondary_code_phase_periods: None,
            },
            super::ReacquisitionSeed {
                carrier_hz: 1705.0,
                code_phase_samples: 43.0,
                cn0_dbhz: 34.0,
                carrier_sign: super::ReacquisitionCarrierSign::Aligned,
                secondary_code_phase_periods: None,
            },
            Some(&uncertainty),
        ));
    }

    #[test]
    fn reacquisition_min_cn0_dbhz_preserves_lock_reference_headroom() {
        assert_eq!(super::reacquisition_min_cn0_dbhz(60.0), 48.0);
        assert_eq!(super::reacquisition_min_cn0_dbhz(35.0), 28.0);
        assert_eq!(super::reacquisition_min_cn0_dbhz(f64::NAN), super::TRACKING_LOCK_MIN_CN0_DBHZ);
    }

    #[test]
    fn code_value_at_phase_wraps_fractional_chip_phases() {
        let code = vec![1_i8, -1, 1, -1];

        assert_eq!(bijux_gnss_signal::api::code_value_at_phase(&code, 0.25).unwrap(), 1.0);
        assert_eq!(bijux_gnss_signal::api::code_value_at_phase(&code, 1.75).unwrap(), -1.0);
        assert_eq!(bijux_gnss_signal::api::code_value_at_phase(&code, 4.10).unwrap(), 1.0);
        assert_eq!(bijux_gnss_signal::api::code_value_at_phase(&code, -0.10).unwrap(), -1.0);
    }

    #[test]
    fn tracking_frame_start_offset_clamps_to_visible_frame_samples() {
        let frame = SamplesFrame::new(
            SampleTime { sample_index: 100, sample_rate_hz: 1_023_000.0 },
            Seconds(1.0 / 1_023_000.0),
            vec![Complex::new(0.0_f32, 0.0); 16],
        );

        assert_eq!(super::tracking_frame_start_offset(&frame, None), Some(0));
        assert_eq!(super::tracking_frame_start_offset(&frame, Some(90)), Some(0));
        assert_eq!(super::tracking_frame_start_offset(&frame, Some(108)), Some(8));
        assert_eq!(super::tracking_frame_start_offset(&frame, Some(116)), None);
    }

    #[test]
    fn refresh_lock_reference_cn0_dbhz_only_updates_on_reliable_lock() {
        assert_eq!(super::refresh_lock_reference_cn0_dbhz(48.0, 52.0, true), 52.0);
        assert_eq!(super::refresh_lock_reference_cn0_dbhz(48.0, 30.0, false), 48.0);
        assert_eq!(super::refresh_lock_reference_cn0_dbhz(48.0, f64::NAN, true), 48.0);
    }

    #[test]
    fn deterministic_transition_rule_holds_pull_in_until_carrier_converges() {
        let decision = super::deterministic_transition_rule(
            ChannelState::PullIn,
            true,
            false,
            false,
            None,
            1,
            0,
            100,
            false,
            None,
        );

        assert_eq!(decision.to_state, ChannelState::PullIn);
        assert_eq!(decision.reason, "carrier_pull_in");
        assert_eq!(decision.next_unlocked_count, 0);
        assert_eq!(decision.next_degraded_epochs, 0);
    }

    #[test]
    fn doppler_estimator_consistency_accepts_aligned_residuals() {
        let consistency = super::doppler_estimator_consistency(0.0, 4.0, -3.0, 8.0);

        assert!(consistency.consistent, "{consistency:?}");
        assert_eq!(consistency.spread_hz, 7.0);
        assert_eq!(consistency.limit_hz, super::DOPPLER_ESTIMATOR_MIN_SPREAD_LIMIT_HZ);
    }

    #[test]
    fn doppler_estimator_consistency_rejects_divergent_residuals() {
        let consistency = super::doppler_estimator_consistency(0.0, 12.0, 95.0, 8.0);

        assert!(!consistency.consistent, "{consistency:?}");
        assert_eq!(consistency.spread_hz, 95.0);
    }

    #[test]
    fn doppler_estimator_consistency_rejects_non_finite_residuals() {
        let consistency = super::doppler_estimator_consistency(0.0, f64::NAN, 2.0, 8.0);

        assert!(!consistency.consistent, "{consistency:?}");
        assert!(consistency.spread_hz.is_infinite());
    }

    #[test]
    fn doppler_estimator_uncertainty_sample_carries_estimator_spread() {
        let consistency = super::doppler_estimator_consistency(0.0, -10.0, 88.0, 8.0);
        let sample = super::doppler_estimator_uncertainty_sample_hz(consistency, 4.0);

        assert_eq!(sample, consistency.spread_hz);
    }

    #[test]
    fn doppler_estimator_provenance_reports_divergence() {
        let consistency = super::doppler_estimator_consistency(0.0, 12.0, 95.0, 8.0);
        let provenance = super::doppler_estimator_provenance(consistency);

        assert!(provenance.contains("doppler_estimator_consistency=divergent"));
        assert!(provenance.contains("doppler_estimator_spread_hz=95.000"));
    }

    #[test]
    fn tracking_provenance_segment_preserves_doppler_estimator_evidence() {
        let provenance = "tracking lock_detector_fll_hz=10.000 doppler_estimator_consistency=divergent doppler_estimator_spread_hz=95.000 doppler_estimator_limit_hz=25.000 doppler_loop_residual_hz=0.000 doppler_phase_rate_residual_hz=12.000 doppler_prompt_residual_hz=95.000 unrelated=true";
        let segment = super::tracking_provenance_segment(
            provenance,
            "doppler_estimator_consistency=",
            super::DOPPLER_ESTIMATOR_PROVENANCE_TOKEN_COUNT,
        )
        .expect("doppler estimator segment");

        assert_eq!(
            segment,
            "doppler_estimator_consistency=divergent doppler_estimator_spread_hz=95.000 doppler_estimator_limit_hz=25.000 doppler_loop_residual_hz=0.000 doppler_phase_rate_residual_hz=12.000 doppler_prompt_residual_hz=95.000"
        );
    }

    #[test]
    fn prelock_cn0_refusal_trips_after_repeated_weak_epochs() {
        let (weak_epochs, supports_lock, refuse_lock) =
            super::update_prelock_cn0_refusal(ChannelState::PullIn, 2, 20.0);

        assert_eq!(weak_epochs, 3);
        assert!(!supports_lock);
        assert!(refuse_lock);
    }

    #[test]
    fn prelock_cn0_refusal_resets_on_supported_signal_or_established_tracking() {
        let (weak_epochs, supports_lock, refuse_lock) =
            super::update_prelock_cn0_refusal(ChannelState::PullIn, 2, 30.0);
        assert_eq!(weak_epochs, 0);
        assert!(supports_lock);
        assert!(!refuse_lock);

        let (weak_epochs, supports_lock, refuse_lock) =
            super::update_prelock_cn0_refusal(ChannelState::Tracking, 2, 20.0);
        assert_eq!(weak_epochs, 0);
        assert!(!supports_lock);
        assert!(!refuse_lock);

        let (weak_epochs, supports_lock, refuse_lock) =
            super::update_prelock_cn0_refusal(ChannelState::Degraded, 2, 20.0);
        assert_eq!(weak_epochs, 0);
        assert!(!supports_lock);
        assert!(!refuse_lock);
    }

    #[test]
    fn classify_prompt_phase_recovers_continuity_across_nav_bit_flip() {
        let decision = super::classify_prompt_phase(
            -0.39,
            Some(0.11),
            0.0,
            super::TrackingPhaseTransitionSource::DataSymbol,
        );

        assert!(decision.nav_bit_transition);
        assert!(!decision.cycle_slip);
        assert!((decision.aligned_phase_cycles - 0.11).abs() <= 0.02);
        assert!(decision.aligned_phase_delta_cycles.abs() <= 0.02);
        assert!((decision.nav_bit_phase_offset_cycles - 0.5).abs() <= f64::EPSILON);
    }

    #[test]
    fn classify_prompt_phase_preserves_cycle_slip_for_non_nav_jump() {
        let decision = super::classify_prompt_phase(
            0.36,
            Some(0.0),
            0.0,
            super::TrackingPhaseTransitionSource::DataSymbol,
        );

        assert!(!decision.nav_bit_transition);
        assert!(decision.cycle_slip);
        assert!((decision.aligned_phase_delta_cycles - 0.36).abs() <= 1.0e-9);
        assert!((decision.nav_bit_phase_offset_cycles - 0.0).abs() <= f64::EPSILON);
    }

    #[test]
    fn classify_prompt_phase_keeps_half_cycle_jump_as_slip_without_transition_metadata() {
        let decision = super::classify_prompt_phase(
            -0.39,
            Some(0.11),
            0.0,
            super::TrackingPhaseTransitionSource::None,
        );

        assert!(!decision.nav_bit_transition);
        assert!(decision.cycle_slip);
        assert!(decision.aligned_phase_delta_cycles.abs() > 0.35);
    }

    #[test]
    fn secondary_code_phase_transition_waits_for_accepted_sync() {
        let config = ReceiverPipelineConfig::default();
        let signal_model = super::TrackingSignalModel::for_sat_signal_band(
            &config,
            SatId { constellation: Constellation::Gps, prn: 18 },
            SignalBand::L5,
            SignalCode::L5I,
            None,
        );

        let transition_source =
            super::carrier_phase_transition_source_for_prompt(&signal_model, None);
        let decision = super::classify_prompt_phase(-0.39, Some(0.11), 0.0, transition_source);

        assert_eq!(transition_source, super::TrackingPhaseTransitionSource::None);
        assert!(!decision.nav_bit_transition);
        assert!(decision.cycle_slip);
    }

    #[test]
    fn secondary_code_phase_transition_accepts_synchronized_half_cycle_jump() {
        let config = ReceiverPipelineConfig::default();
        let signal_model = super::TrackingSignalModel::for_sat_signal_band(
            &config,
            SatId { constellation: Constellation::Gps, prn: 18 },
            SignalBand::L5,
            SignalCode::L5I,
            None,
        );
        let sync = super::SecondaryCodeSyncResult {
            phase_periods: 7,
            confidence: 1.0,
            best_likelihood: 1.0,
            next_best_likelihood: 0.0,
            observed_periods: 20,
            accepted: true,
        };

        let transition_source =
            super::carrier_phase_transition_source_for_prompt(&signal_model, Some(sync));
        let decision = super::classify_prompt_phase(-0.39, Some(0.11), 0.0, transition_source);

        assert_eq!(transition_source, super::TrackingPhaseTransitionSource::SecondaryCode);
        assert!(decision.nav_bit_transition);
        assert!(!decision.cycle_slip);
    }

    #[test]
    fn secondary_code_phase_transitions_preserve_continuity_across_multiple_flips() {
        let config = ReceiverPipelineConfig::default();
        let signal_model = super::TrackingSignalModel::for_sat_signal_band(
            &config,
            SatId { constellation: Constellation::Gps, prn: 18 },
            SignalBand::L5,
            SignalCode::L5I,
            None,
        );
        let sync = super::SecondaryCodeSyncResult {
            phase_periods: 7,
            confidence: 1.0,
            best_likelihood: 1.0,
            next_best_likelihood: 0.0,
            observed_periods: 20,
            accepted: true,
        };
        let transition_source =
            super::carrier_phase_transition_source_for_prompt(&signal_model, Some(sync));
        let raw_phase_cycles = [0.10, -0.39, -0.38, 0.13, 0.14];
        let expected_transitions = [false, true, false, true, false];
        let mut previous_aligned_phase_cycles = None;
        let mut secondary_code_phase_offset_cycles = 0.0;

        for (raw_phase_cycles, expected_transition) in
            raw_phase_cycles.into_iter().zip(expected_transitions)
        {
            let decision = super::classify_prompt_phase(
                raw_phase_cycles,
                previous_aligned_phase_cycles,
                secondary_code_phase_offset_cycles,
                transition_source,
            );

            assert_eq!(decision.nav_bit_transition, expected_transition, "{decision:?}");
            assert!(!decision.cycle_slip, "{decision:?}");
            if previous_aligned_phase_cycles.is_some() {
                assert!(decision.aligned_phase_delta_cycles.abs() <= 0.03, "{decision:?}");
            }
            previous_aligned_phase_cycles = Some(decision.aligned_phase_cycles);
            secondary_code_phase_offset_cycles = decision.nav_bit_phase_offset_cycles;
        }
    }

    #[test]
    fn classify_prompt_phase_handles_real_synthetic_nav_bit_transitions() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            channels: 12,
            ..ReceiverPipelineConfig::default()
        };
        let sat = SatId { constellation: bijux_gnss_core::api::Constellation::Gps, prn: 7 };
        let code_phase_chips = 321.0;
        let samples_per_epoch = samples_per_code(
            config.sampling_freq_hz,
            config.code_freq_basis_hz,
            config.code_length,
        );
        let frame = generate_l1_ca(
            &config,
            SyntheticSignalParams {
                sat,
                glonass_frequency_channel: None,
                signal_band: bijux_gnss_core::api::SignalBand::L1,
                signal_code: bijux_gnss_core::api::SignalCode::Unknown,
                doppler_hz: 0.0,
                code_phase_chips,
                carrier_phase_rad: 0.0,
                cn0_db_hz: 70.0,
                navigation_data: true.into(),
            },
            0x7A91B17,
            0.05,
        );
        let tracking = Tracking::new(config.clone(), ReceiverRuntime::default());
        let mut previous_aligned_phase_cycles = None;
        let mut nav_bit_phase_offset_cycles = 0.0;
        let mut transition_epochs = Vec::new();

        for (epoch_index, epoch_samples) in frame.iq.chunks_exact(samples_per_epoch).enumerate() {
            let sample_index = epoch_index as u64 * samples_per_epoch as u64;
            let epoch_frame = SamplesFrame::new(
                SampleTime { sample_index, sample_rate_hz: config.sampling_freq_hz },
                Seconds(1.0 / config.sampling_freq_hz),
                epoch_samples.to_vec(),
            );
            let epoch_code_phase_chips = advance_code_phase_seconds(
                code_phase_chips,
                config.code_freq_basis_hz,
                sample_index as f64 / config.sampling_freq_hz,
                config.code_length,
            )
            .expect("valid epoch code phase");
            let epoch_code_phase_samples =
                crate::sim::synthetic::expected_acquisition_code_phase_samples_f64(
                    &config,
                    &epoch_frame,
                    epoch_code_phase_chips,
                );
            let (epoch, _) = tracking.track_epoch(
                &epoch_frame,
                0,
                sat,
                0.0,
                0.0,
                config.code_freq_basis_hz,
                epoch_code_phase_samples,
                0.5,
            );
            let raw_phase_cycles =
                (epoch.prompt_q as f64).atan2(epoch.prompt_i as f64) / (2.0 * std::f64::consts::PI);
            let decision = super::classify_prompt_phase(
                raw_phase_cycles,
                previous_aligned_phase_cycles,
                nav_bit_phase_offset_cycles,
                super::TrackingPhaseTransitionSource::DataSymbol,
            );
            if decision.nav_bit_transition {
                transition_epochs.push(epoch_index);
            }
            assert!(
                !decision.cycle_slip,
                "unexpected cycle slip at epoch {epoch_index}: raw_phase_cycles={raw_phase_cycles}"
            );
            previous_aligned_phase_cycles = Some(decision.aligned_phase_cycles);
            nav_bit_phase_offset_cycles = decision.nav_bit_phase_offset_cycles;
        }

        assert_eq!(transition_epochs, vec![20, 40]);
    }

    #[test]
    fn advance_code_phase_samples_wraps_nominal_epoch_step() {
        let next = super::apply_dll_code_loop(super::CodeLoopInput {
            current_code_rate_hz: 1_023_000.0,
            previous_reference_code_rate_hz: 1_023_000.0,
            reference_code_rate_hz: 1_023_000.0,
            current_code_phase_samples: 137.5,
            epoch_len_samples: 5_000,
            coherent_integration_s: 5_000.0 / 4_092_000.0,
            nominal_code_rate_hz: 1_023_000.0,
            dll_bw_hz: 2.0,
            dll_err: 0.0,
            samples_per_chip: 4.887585532746823,
            samples_per_code: 5_000,
        })
        .code_phase_samples;
        assert!((next - 137.5).abs() < 1.0e-9, "next={next}");
    }

    #[test]
    fn advance_code_phase_samples_applies_dll_correction() {
        let next = super::apply_dll_code_loop(super::CodeLoopInput {
            current_code_rate_hz: 1_023_000.0,
            previous_reference_code_rate_hz: 1_023_000.0,
            reference_code_rate_hz: 1_023_000.0,
            current_code_phase_samples: 250.0,
            epoch_len_samples: 5_000,
            coherent_integration_s: 5_000.0 / 4_092_000.0,
            nominal_code_rate_hz: 1_023_000.0,
            dll_bw_hz: 2.0,
            dll_err: 0.4,
            samples_per_chip: 4.887585532746823,
            samples_per_code: 5_000,
        })
        .code_phase_samples;
        assert!(next > 250.0, "next={next}");
    }

    #[test]
    fn tracking_epoch_samples_scale_with_configured_integration_ms() {
        let tracking_params = crate::engine::receiver_config::TrackingParams {
            early_late_spacing_chips: 0.5,
            dll_bw_hz: 2.0,
            pll_bw_hz: 15.0,
            fll_bw_hz: 10.0,
            integration_ms: 7,
        };

        let epoch_samples =
            super::tracking_epoch_samples(5_000_000.0, 1_023_000.0, 1023, tracking_params);

        assert_eq!(epoch_samples, 35_000);
    }

    #[test]
    fn tracking_params_for_state_uses_adapted_profile_when_enabled() {
        let tracking = Tracking::new(
            ReceiverPipelineConfig {
                adaptive_tracking_enabled: true,
                ..ReceiverPipelineConfig::default()
            },
            ReceiverRuntime::default(),
        );
        let base_tracking_params = TrackingParams {
            early_late_spacing_chips: 0.5,
            dll_bw_hz: 2.0,
            pll_bw_hz: 15.0,
            fll_bw_hz: 10.0,
            integration_ms: 1,
        };
        let mut state = empty_loop_state();
        state.tracking_loop_profile = SignalTrackingLoopProfile {
            dll_bw_hz: 1.2,
            pll_bw_hz: 8.25,
            fll_bw_hz: 5.0,
            integration_ms: 5,
        };

        let effective = tracking.tracking_params_for_state(base_tracking_params, &state);

        assert_eq!(
            effective.early_late_spacing_chips,
            base_tracking_params.early_late_spacing_chips
        );
        assert_eq!(effective.dll_bw_hz, 1.2);
        assert_eq!(effective.pll_bw_hz, 8.25);
        assert_eq!(effective.fll_bw_hz, 5.0);
        assert_eq!(effective.integration_ms, 5);
    }

    #[test]
    fn tracking_params_for_state_ignores_adapted_profile_when_disabled() {
        let tracking = Tracking::new(
            ReceiverPipelineConfig {
                adaptive_tracking_enabled: false,
                ..ReceiverPipelineConfig::default()
            },
            ReceiverRuntime::default(),
        );
        let base_tracking_params = TrackingParams {
            early_late_spacing_chips: 0.5,
            dll_bw_hz: 2.0,
            pll_bw_hz: 15.0,
            fll_bw_hz: 10.0,
            integration_ms: 1,
        };
        let mut state = empty_loop_state();
        state.tracking_loop_profile = SignalTrackingLoopProfile {
            dll_bw_hz: 1.2,
            pll_bw_hz: 8.25,
            fll_bw_hz: 5.0,
            integration_ms: 5,
        };

        let effective = tracking.tracking_params_for_state(base_tracking_params, &state);

        assert_eq!(effective.dll_bw_hz, base_tracking_params.dll_bw_hz);
        assert_eq!(effective.pll_bw_hz, base_tracking_params.pll_bw_hz);
        assert_eq!(effective.fll_bw_hz, base_tracking_params.fll_bw_hz);
        assert_eq!(effective.integration_ms, base_tracking_params.integration_ms);
    }

    fn empty_loop_state() -> super::LoopState {
        super::LoopState {
            carrier_hz: 0.0,
            carrier_phase_cycles: 0.0,
            carrier_rate_hz_per_s: 0.0,
            code_rate_hz: 0.0,
            code_rate_reference_hz: 0.0,
            code_phase_samples: 0.0,
            tracking_adaptation_state: Default::default(),
            tracking_loop_profile: SignalTrackingLoopProfile {
                dll_bw_hz: 2.0,
                pll_bw_hz: 15.0,
                fll_bw_hz: 10.0,
                integration_ms: 1,
            },
            signal_delay_alignment: None,
            subcarrier_code_phase_refined: false,
            acquisition_cn0_proxy_dbhz: 45.0,
            lock_reference_cn0_dbhz: 45.0,
            prev_prompt: None,
            prev_prompt_phase_cycles: None,
            secondary_code_prompt_history: std::collections::VecDeque::new(),
            secondary_code_sync: None,
            nav_bit_phase_offset_cycles: 0.0,
            nav_bit_transition_count: 0,
            pull_in_stable_epochs: 0,
            weak_cn0_epochs: 0,
            degraded_epochs: 0,
            prompt_power_reference: 0.0,
            prompt_cn0_window: std::collections::VecDeque::new(),
            code_error_window_samples: std::collections::VecDeque::new(),
            carrier_phase_error_window_cycles: std::collections::VecDeque::new(),
            doppler_error_window_hz: std::collections::VecDeque::new(),
            cn0_estimate_window_dbhz: std::collections::VecDeque::new(),
            unstable_discriminator_epochs: 0,
            state: ChannelState::Tracking,
            unlocked_count: 0,
            lost_reason: None,
            reacquisition_candidate: None,
            reacquisition_candidate_streak: 0,
            reacquisition_pending: false,
            reacquisition_attempt_epochs: 0,
            reacquisition_stable_tracking_epochs: 0,
        }
    }

    #[test]
    fn tracking_uncertainty_rewards_longer_coherent_integration() {
        let state = empty_loop_state();
        let short = super::estimate_tracking_uncertainty(
            &state,
            super::TrackingUncertaintyInputs {
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
                channel_state: ChannelState::Tracking,
            },
        );
        let long = super::estimate_tracking_uncertainty(
            &state,
            super::TrackingUncertaintyInputs {
                integration_ms: 10,
                ..super::TrackingUncertaintyInputs {
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
                    channel_state: ChannelState::Tracking,
                }
            },
        );

        assert!(
            long.code_phase_samples < short.code_phase_samples,
            "longer coherent integration should tighten code-phase uncertainty: short={short:?} long={long:?}"
        );
    }

    #[test]
    fn tracking_uncertainty_penalizes_dll_unlock() {
        let state = empty_loop_state();
        let locked = super::estimate_tracking_uncertainty(
            &state,
            super::TrackingUncertaintyInputs {
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
                channel_state: ChannelState::Tracking,
            },
        );
        let unlocked = super::estimate_tracking_uncertainty(
            &state,
            super::TrackingUncertaintyInputs {
                dll_locked: false,
                ..super::TrackingUncertaintyInputs {
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
                    channel_state: ChannelState::Tracking,
                }
            },
        );

        assert!(
            unlocked.code_phase_samples > locked.code_phase_samples,
            "loss of DLL lock should inflate code-phase uncertainty: locked={locked:?} unlocked={unlocked:?}"
        );
    }

    #[test]
    fn short_fade_epoch_budget_reserves_post_fade_recovery_window() {
        let tracking_params = crate::engine::receiver_config::TrackingParams {
            early_late_spacing_chips: 0.5,
            dll_bw_hz: 2.0,
            pll_bw_hz: 15.0,
            fll_bw_hz: 10.0,
            integration_ms: 1,
        };

        assert_eq!(super::short_fade_epoch_budget(tracking_params), 105);
    }

    #[test]
    fn tracking_epoch_count_includes_trailing_partial_epoch() {
        assert_eq!(super::tracking_epoch_count(60, 20), 3);
        assert_eq!(super::tracking_epoch_count(61, 20), 4);
        assert_eq!(super::tracking_epoch_count(0, 20), 0);
    }

    #[test]
    fn vector_tracking_rejects_weak_or_incomplete_evidence() {
        let weak = vector_measurement(0, 5_000, 34.9, 1.0, 0.1, 0.0);
        let mut unlocked = vector_measurement(1, 5_000, 42.0, 1.0, 0.1, 0.0);
        unlocked.pll_locked = false;
        let strong = vector_measurement(2, 5_000, 42.0, 1.0, 0.1, 0.0);

        assert!(!super::vector_tracking_measurement_is_usable(weak));
        assert!(!super::vector_tracking_measurement_is_usable(unlocked));
        assert!(super::vector_tracking_measurement_is_usable(strong));
        assert!(super::vector_tracking_prediction(&[strong]).is_none());
    }

    #[test]
    fn vector_tracking_prediction_uses_latest_measurement_per_channel() {
        let mut state = super::VectorTrackingState::default();
        state.record(vector_measurement(0, 100, 42.0, 1.0, 0.10, 4.0), 1_000.0);
        state.record(vector_measurement(0, 110, 42.0, 5.0, 0.50, 8.0), 1_000.0);
        state.record(vector_measurement(1, 108, 42.0, 7.0, 0.70, 12.0), 1_000.0);

        let prediction = state.prediction_for(120, 1_000.0).expect("vector prediction");

        assert_eq!(prediction.contributor_count, 2);
        assert_eq!(prediction.sample_index, 110);
        assert!((prediction.receiver_position_code_phase_error_samples - 0.05).abs() < 1.0e-9);
        assert!((prediction.receiver_clock_frequency_error_hz - 6.0).abs() < 1.0e-9);
        assert!((prediction.receiver_clock_frequency_residual_spread_hz - 2.0).abs() < 1.0e-9);
        assert!((prediction.receiver_clock_frequency_max_residual_hz - 1.0).abs() < 1.0e-9);
        assert!((prediction.receiver_code_rate_error_hz - 0.60).abs() < 1.0e-9);
        assert!((prediction.receiver_motion_frequency_rate_hz_per_s - 10.0).abs() < 1.0e-9);
    }

    #[test]
    fn common_tracking_frequency_estimate_uses_weighted_stable_channel_residuals() {
        let low_weight = vector_measurement(0, 100, 35.0, 4.0, 0.10, 1_000.0);
        let high_weight = vector_measurement(1, 110, 45.0, 8.0, 0.20, -1_000.0);

        let estimate = super::common_tracking_frequency_estimate(&[low_weight, high_weight])
            .expect("common tracking frequency estimate");

        assert_eq!(estimate.support_count, 2);
        assert_eq!(estimate.sample_index, 110);
        assert!((estimate.estimated_frequency_error_hz - 7.636_363_636_363_637).abs() < 1.0e-9);
        assert!((estimate.mean_cn0_dbhz - 44.090_909_090_909_09).abs() < 1.0e-9);
        assert_eq!(estimate.supporting_channels.len(), 2);
        assert!(estimate.supporting_channels.iter().all(|channel| channel.residual_hz.is_finite()));
        assert!(
            estimate.max_supporting_residual_hz > 0.0
                && estimate.residual_spread_hz > estimate.max_supporting_residual_hz,
            "estimate should expose residual spread and maximum residual: {estimate:?}"
        );
    }

    #[test]
    fn common_tracking_frequency_estimate_rejects_insufficient_stable_support() {
        let mut unlocked = vector_measurement(0, 100, 45.0, 4.0, 0.10, 0.0);
        unlocked.pll_locked = false;
        let stable = vector_measurement(1, 110, 45.0, 8.0, 0.20, 0.0);

        assert!(super::common_tracking_frequency_estimate(&[unlocked, stable]).is_none());
    }

    #[test]
    fn common_tracking_frequency_estimate_rejects_channel_frequency_outliers() {
        let common_left = vector_measurement(0, 100, 45.0, -2.0, 0.10, 0.0);
        let common_right = vector_measurement(1, 100, 45.0, 1.0, 0.10, 0.0);
        let outlier = vector_measurement(2, 100, 45.0, -148.0, 0.10, 0.0);

        let estimate =
            super::common_tracking_frequency_estimate(&[common_left, common_right, outlier])
                .expect("common tracking frequency estimate");

        assert_eq!(estimate.support_count, 2);
        assert!(
            estimate.supporting_channels.iter().all(|channel| channel.frequency_error_hz > -10.0),
            "{estimate:?}"
        );
        assert!(estimate.max_supporting_residual_hz <= 1.5, "{estimate:?}");
    }

    #[test]
    fn common_tracking_frequency_estimate_does_not_absorb_satellite_specific_motion() {
        let fast_approaching = vector_measurement(0, 100, 45.0, 6.0, 0.10, 4_000.0);
        let fast_receding = vector_measurement(1, 110, 45.0, 6.0, 0.20, -4_000.0);

        let estimate =
            super::common_tracking_frequency_estimate(&[fast_approaching, fast_receding])
                .expect("common tracking frequency estimate");

        assert!((estimate.estimated_frequency_error_hz - 6.0).abs() < 1.0e-9);
        assert!(estimate.max_supporting_residual_hz <= f64::EPSILON);
    }

    #[test]
    fn vector_tracking_state_reports_latest_common_frequency_estimate() {
        let mut state = super::VectorTrackingState::default();
        state.record(vector_measurement(0, 100, 45.0, 2.0, 0.10, 0.0), 1_000.0);
        state.record(vector_measurement(0, 120, 45.0, 4.0, 0.10, 0.0), 1_000.0);
        state.record(vector_measurement(1, 118, 45.0, 6.0, 0.10, 0.0), 1_000.0);

        let estimate = state.common_frequency_estimate(1_000.0).expect("common frequency estimate");

        assert_eq!(estimate.support_count, 2);
        assert_eq!(estimate.sample_index, 120);
        assert!((estimate.estimated_frequency_error_hz - 5.0).abs() < 1.0e-9);
        assert!(estimate
            .supporting_channels
            .iter()
            .all(|channel| channel.frequency_error_hz >= 4.0));
    }

    #[test]
    fn vector_tracking_application_bounds_weak_channel_aid() {
        let config = ReceiverPipelineConfig::default();
        let tracking = Tracking::new(config.clone(), ReceiverRuntime::default());
        let sat = SatId { constellation: Constellation::Gps, prn: 1 };
        let signal_model = super::TrackingSignalModel::for_sat(&config, sat);
        let mut loop_state = tracking.initial_loop_state(
            &signal_model,
            1500.0,
            0.0,
            31.0,
            None,
            false,
            config.tracking_params(SignalBand::L1),
            false,
        );
        loop_state.state = ChannelState::PullIn;
        let prediction = super::VectorTrackingPrediction {
            sample_index: 10_000,
            contributor_count: 3,
            mean_cn0_dbhz: 44.0,
            receiver_position_code_phase_error_samples: 10.0,
            receiver_clock_frequency_error_hz: 1_000.0,
            receiver_clock_frequency_residual_spread_hz: 0.0,
            receiver_clock_frequency_max_residual_hz: 0.0,
            receiver_code_rate_error_hz: 50.0,
            receiver_motion_frequency_rate_hz_per_s: 5_000.0,
        };

        let application =
            super::vector_tracking_application(prediction, &loop_state).expect("vector aid");

        assert!(
            (application.carrier_frequency_correction_hz
                + super::VECTOR_TRACKING_MAX_CARRIER_AID_HZ * 0.35)
                .abs()
                < 1.0e-9
        );
        assert!(
            (application.code_rate_correction_hz
                - super::VECTOR_TRACKING_MAX_CODE_RATE_AID_HZ * 0.35)
                .abs()
                < 1.0e-9
        );
        assert!(
            (application.code_phase_correction_samples
                - super::VECTOR_TRACKING_MAX_CODE_PHASE_AID_SAMPLES * 0.35)
                .abs()
                < 1.0e-9
        );
        assert!(
            (application.carrier_rate_correction_hz_per_s
                - super::VECTOR_TRACKING_MAX_CARRIER_RATE_AID_HZ_PER_S * 0.35)
                .abs()
                < 1.0e-9
        );
        loop_state.state = ChannelState::Lost;
        assert!(super::vector_tracking_application(prediction, &loop_state).is_none());
    }

    #[test]
    fn vector_tracking_application_preserves_stable_frequency_support_channels() {
        let config = ReceiverPipelineConfig::default();
        let tracking = Tracking::new(config.clone(), ReceiverRuntime::default());
        let sat = SatId { constellation: Constellation::Gps, prn: 1 };
        let signal_model = super::TrackingSignalModel::for_sat(&config, sat);
        let mut loop_state = tracking.initial_loop_state(
            &signal_model,
            1500.0,
            0.0,
            45.0,
            None,
            false,
            config.tracking_params(SignalBand::L1),
            false,
        );
        loop_state.state = ChannelState::Tracking;
        let prediction = super::VectorTrackingPrediction {
            sample_index: 10_000,
            contributor_count: 3,
            mean_cn0_dbhz: 45.0,
            receiver_position_code_phase_error_samples: 0.20,
            receiver_clock_frequency_error_hz: 6.0,
            receiver_clock_frequency_residual_spread_hz: 0.50,
            receiver_clock_frequency_max_residual_hz: 0.25,
            receiver_code_rate_error_hz: 1.0,
            receiver_motion_frequency_rate_hz_per_s: 4.0,
        };

        let application =
            super::vector_tracking_application(prediction, &loop_state).expect("vector aid");

        assert_eq!(application.carrier_frequency_correction_hz, 0.0);
        assert!(
            application.code_rate_correction_hz > 0.0,
            "stable support channels should still consume non-carrier vector aids: {application:?}"
        );
    }

    #[test]
    fn vector_tracking_application_reduces_common_frequency_prediction_error() {
        let config = ReceiverPipelineConfig::default();
        let tracking = Tracking::new(config.clone(), ReceiverRuntime::default());
        let sat = SatId { constellation: Constellation::Gps, prn: 1 };
        let signal_model = super::TrackingSignalModel::for_sat(&config, sat);
        let mut loop_state = tracking.initial_loop_state(
            &signal_model,
            43.0,
            0.0,
            31.0,
            None,
            false,
            config.tracking_params(SignalBand::L1),
            false,
        );
        loop_state.state = ChannelState::PullIn;
        let prediction = super::VectorTrackingPrediction {
            sample_index: 10_000,
            contributor_count: 3,
            mean_cn0_dbhz: 45.0,
            receiver_position_code_phase_error_samples: 0.0,
            receiver_clock_frequency_error_hz: -4.0,
            receiver_clock_frequency_residual_spread_hz: 0.50,
            receiver_clock_frequency_max_residual_hz: 0.25,
            receiver_code_rate_error_hz: 0.0,
            receiver_motion_frequency_rate_hz_per_s: 0.0,
        };

        let application =
            super::vector_tracking_application(prediction, &loop_state).expect("vector aid");
        let unaided_error_hz = (loop_state.carrier_hz - 45.0).abs();
        let aided_error_hz =
            (loop_state.carrier_hz + application.carrier_frequency_correction_hz - 45.0).abs();

        assert!(
            aided_error_hz < unaided_error_hz,
            "common frequency correction should improve the carrier prediction: application={application:?} unaided_error_hz={unaided_error_hz} aided_error_hz={aided_error_hz}"
        );
    }

    #[test]
    fn apply_dll_code_loop_updates_code_rate_from_discriminator() {
        let coherent_integration_s = 5_000.0 / 1_023_000.0;
        let update = super::apply_dll_code_loop(super::CodeLoopInput {
            current_code_rate_hz: 1_023_000.0,
            previous_reference_code_rate_hz: 1_023_000.0,
            reference_code_rate_hz: 1_023_000.0,
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
    fn apply_dll_code_loop_follows_reference_code_rate_step_without_dll_error() {
        let update = super::apply_dll_code_loop(super::CodeLoopInput {
            current_code_rate_hz: 1_023_000.0,
            previous_reference_code_rate_hz: 1_023_000.0,
            reference_code_rate_hz: 1_023_450.0,
            current_code_phase_samples: 250.0,
            epoch_len_samples: 5_000,
            coherent_integration_s: 5_000.0 / 1_023_000.0,
            nominal_code_rate_hz: 1_023_000.0,
            dll_bw_hz: 2.0,
            dll_err: 0.0,
            samples_per_chip: 4.887585532746823,
            samples_per_code: 5_000,
        });

        assert!((update.code_rate_hz - 1_023_450.0).abs() < 1.0e-9, "{update:?}");
    }

    #[test]
    fn apply_dll_code_loop_uses_coherent_interval_to_scale_gain() {
        let short = super::apply_dll_code_loop(super::CodeLoopInput {
            current_code_rate_hz: 1_023_000.0,
            previous_reference_code_rate_hz: 1_023_000.0,
            reference_code_rate_hz: 1_023_000.0,
            current_code_phase_samples: 250.0,
            epoch_len_samples: 4_092,
            coherent_integration_s: 0.001,
            nominal_code_rate_hz: 1_023_000.0,
            dll_bw_hz: 2.0,
            dll_err: 0.25,
            samples_per_chip: 4.0,
            samples_per_code: 4_092,
        });
        let long = super::apply_dll_code_loop(super::CodeLoopInput {
            current_code_rate_hz: 1_023_000.0,
            previous_reference_code_rate_hz: 1_023_000.0,
            reference_code_rate_hz: 1_023_000.0,
            current_code_phase_samples: 250.0,
            epoch_len_samples: 40_920,
            coherent_integration_s: 0.010,
            nominal_code_rate_hz: 1_023_000.0,
            dll_bw_hz: 2.0,
            dll_err: 0.25,
            samples_per_chip: 4.0,
            samples_per_code: 40_920,
        });

        assert!(
            (long.code_rate_hz - 1_023_000.0).abs() > (short.code_rate_hz - 1_023_000.0).abs(),
            "short={short:?} long={long:?}"
        );
    }

    #[test]
    fn apply_dll_code_loop_pulls_positive_code_error_toward_prompt() {
        let current_code_phase_samples = 250.0;
        let update = super::apply_dll_code_loop(super::CodeLoopInput {
            current_code_rate_hz: 1_023_000.0,
            previous_reference_code_rate_hz: 1_023_000.0,
            reference_code_rate_hz: 1_023_000.0,
            current_code_phase_samples,
            epoch_len_samples: 5_000,
            coherent_integration_s: 5_000.0 / 1_023_000.0,
            nominal_code_rate_hz: 1_023_000.0,
            dll_bw_hz: 2.0,
            dll_err: 0.4,
            samples_per_chip: 4.887585532746823,
            samples_per_code: 5_000,
        });

        assert!(update.code_phase_samples > current_code_phase_samples, "update={update:?}");
    }

    #[test]
    fn apply_dll_code_loop_pulls_negative_code_error_toward_prompt() {
        let current_code_phase_samples = 250.0;
        let update = super::apply_dll_code_loop(super::CodeLoopInput {
            current_code_rate_hz: 1_023_000.0,
            previous_reference_code_rate_hz: 1_023_000.0,
            reference_code_rate_hz: 1_023_000.0,
            current_code_phase_samples,
            epoch_len_samples: 5_000,
            coherent_integration_s: 5_000.0 / 1_023_000.0,
            nominal_code_rate_hz: 1_023_000.0,
            dll_bw_hz: 2.0,
            dll_err: -0.4,
            samples_per_chip: 4.887585532746823,
            samples_per_code: 5_000,
        });

        assert!(update.code_phase_samples < current_code_phase_samples, "update={update:?}");
    }

    #[test]
    fn correlate_epoch_aligns_prompt_with_non_integer_rate_sampled_code() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_000_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            channels: 12,
            ..ReceiverPipelineConfig::default()
        };
        let tracking = Tracking::new(config.clone(), ReceiverRuntime::default());
        let sample_count = samples_per_code(
            config.sampling_freq_hz,
            config.code_freq_basis_hz,
            config.code_length,
        );
        let code_phase_chips = 245.25;
        let sat = SatId { constellation: bijux_gnss_core::api::Constellation::Gps, prn: 3 };
        let samples = sample_ca_code(
            Prn(sat.prn),
            config.sampling_freq_hz,
            config.code_freq_basis_hz,
            code_phase_chips,
            sample_count,
        )
        .expect("valid sampled code");
        let frame = SamplesFrame::new(
            SampleTime { sample_index: 0, sample_rate_hz: config.sampling_freq_hz },
            Seconds(1.0 / config.sampling_freq_hz),
            samples.into_iter().map(|value| Complex::new(value, 0.0)).collect(),
        );
        let code_phase_samples = crate::sim::synthetic::expected_acquisition_code_phase_samples_f64(
            &config,
            &frame,
            code_phase_chips,
        );

        let correlator = tracking.correlate_epoch(
            &frame,
            sat,
            0.0,
            0.0,
            config.code_freq_basis_hz,
            code_phase_samples,
            0.5,
        );

        assert!(correlator.prompt.norm() > correlator.early.norm());
        assert!(correlator.prompt.norm() > correlator.late.norm());
    }

    #[test]
    fn correlate_epoch_uses_tracked_code_rate_for_code_rate_offset_signal() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            channels: 12,
            ..ReceiverPipelineConfig::default()
        };
        let tracking = Tracking::new(config.clone(), ReceiverRuntime::default());
        let sample_count = samples_per_code(
            config.sampling_freq_hz,
            config.code_freq_basis_hz,
            config.code_length,
        );
        let signal_code_rate_hz = config.code_freq_basis_hz + 1_200.0;
        let code_phase_chips = 87.375;
        let sat = SatId { constellation: bijux_gnss_core::api::Constellation::Gps, prn: 9 };
        let samples = sample_ca_code(
            Prn(sat.prn),
            config.sampling_freq_hz,
            signal_code_rate_hz,
            code_phase_chips,
            sample_count,
        )
        .expect("valid sampled code with signal code-rate offset");
        let frame = SamplesFrame::new(
            SampleTime { sample_index: 0, sample_rate_hz: config.sampling_freq_hz },
            Seconds(1.0 / config.sampling_freq_hz),
            samples.into_iter().map(|value| Complex::new(value, 0.0)).collect(),
        );
        let code_phase_samples = crate::sim::synthetic::expected_acquisition_code_phase_samples_f64(
            &config,
            &frame,
            code_phase_chips,
        );

        let nominal = tracking.correlate_epoch(
            &frame,
            sat,
            0.0,
            0.0,
            config.code_freq_basis_hz,
            code_phase_samples,
            0.5,
        );
        let matched = tracking.correlate_epoch(
            &frame,
            sat,
            0.0,
            0.0,
            signal_code_rate_hz,
            code_phase_samples,
            0.5,
        );

        assert!(
            matched.prompt.norm() > nominal.prompt.norm(),
            "matched_prompt={} nominal_prompt={}",
            matched.prompt.norm(),
            nominal.prompt.norm(),
        );
        assert!(matched.prompt.norm() > matched.early.norm());
        assert!(matched.prompt.norm() > matched.late.norm());
    }

    #[test]
    fn correlate_epoch_uses_tracked_carrier_phase_for_phase_offset_signal() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            channels: 12,
            ..ReceiverPipelineConfig::default()
        };
        let tracking = Tracking::new(config.clone(), ReceiverRuntime::default());
        let sat = SatId { constellation: bijux_gnss_core::api::Constellation::Gps, prn: 5 };
        let carrier_phase_rad = 0.35;
        let carrier_phase_cycles = carrier_phase_rad / std::f64::consts::TAU;
        let frame = generate_l1_ca(
            &config,
            SyntheticSignalParams {
                sat,
                glonass_frequency_channel: None,
                signal_band: bijux_gnss_core::api::SignalBand::L1,
                signal_code: bijux_gnss_core::api::SignalCode::Unknown,
                doppler_hz: 0.0,
                code_phase_chips: 0.0,
                carrier_phase_rad,
                cn0_db_hz: 70.0,
                navigation_data: false.into(),
            },
            0xC0A5_1E,
            0.001,
        );

        let unmatched =
            tracking.correlate_epoch(&frame, sat, 0.0, 0.0, config.code_freq_basis_hz, 0.0, 0.5);
        let matched = tracking.correlate_epoch(
            &frame,
            sat,
            0.0,
            carrier_phase_cycles,
            config.code_freq_basis_hz,
            0.0,
            0.5,
        );

        assert!(
            matched.prompt.re > unmatched.prompt.re,
            "matched={:?} unmatched={:?}",
            matched.prompt,
            unmatched.prompt,
        );
        assert!(
            matched.prompt.im.abs() < unmatched.prompt.im.abs(),
            "matched={:?} unmatched={:?}",
            matched.prompt,
            unmatched.prompt,
        );
    }

    #[test]
    fn correlate_epoch_keeps_carrier_phase_aligned_across_nonzero_epoch_starts() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            channels: 12,
            ..ReceiverPipelineConfig::default()
        };
        let tracking = Tracking::new(config.clone(), ReceiverRuntime::default());
        let sat = SatId { constellation: bijux_gnss_core::api::Constellation::Gps, prn: 5 };
        let carrier_hz = 120.0;
        let epoch_len_samples = samples_per_code(
            config.sampling_freq_hz,
            config.code_freq_basis_hz,
            config.code_length,
        );
        let second_epoch_start_s = epoch_len_samples as f64 / config.sampling_freq_hz;
        let second_epoch_start_phase_cycles = 0.18;
        let initial_phase_cycles =
            second_epoch_start_phase_cycles - carrier_hz * second_epoch_start_s;
        let frame = generate_l1_ca(
            &config,
            SyntheticSignalParams {
                sat,
                glonass_frequency_channel: None,
                signal_band: bijux_gnss_core::api::SignalBand::L1,
                signal_code: bijux_gnss_core::api::SignalCode::Unknown,
                doppler_hz: carrier_hz,
                code_phase_chips: 0.0,
                carrier_phase_rad: initial_phase_cycles * std::f64::consts::TAU,
                cn0_db_hz: 90.0,
                navigation_data: false.into(),
            },
            0xC0A5_1E,
            0.002,
        );
        let second_epoch = super::frame_slice(&frame, epoch_len_samples, epoch_len_samples * 2);

        let correlator = tracking.correlate_epoch(
            &second_epoch,
            sat,
            carrier_hz,
            second_epoch_start_phase_cycles,
            config.code_freq_basis_hz,
            0.0,
            0.5,
        );

        assert!(
            correlator.prompt.re > correlator.prompt.im.abs() * 10.0,
            "prompt={:?}",
            correlator.prompt,
        );
    }

    #[test]
    fn apply_carrier_loop_advances_phase_and_frequency_from_pll_error() {
        let update = super::apply_carrier_loop(super::CarrierLoopInput {
            current_carrier_hz: 1_000.0,
            current_carrier_phase_cycles: 12.0,
            current_carrier_rate_hz_per_s: 0.0,
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
        assert!(
            (update.carrier_rate_hz_per_s
                - pll_coefficients.frequency_rate_gain_hz_per_s_per_rad * 0.25)
                .abs()
                < 1.0e-9,
            "{update:?}"
        );
        let expected_phase_cycles = 12.0
            + (1_000.0 + (1_000.0 + pll_coefficients.frequency_gain_hz_per_rad * 0.25)) * 0.0005
            + pll_coefficients.phase_blend * 0.25 / std::f64::consts::TAU;
        assert!((update.carrier_phase_cycles - expected_phase_cycles).abs() < 1.0e-9, "{update:?}",);
    }

    #[test]
    fn apply_carrier_loop_uses_fll_correction_during_pull_in() {
        let update = super::apply_carrier_loop(super::CarrierLoopInput {
            current_carrier_hz: 80.0,
            current_carrier_phase_cycles: 12.0,
            current_carrier_rate_hz_per_s: 0.0,
            epoch_len_samples: 4_092,
            sample_rate_hz: 4_092_000.0,
            coherent_integration_s: 0.001,
            pll_bw_hz: 8.0,
            pll_err_rad: 0.25,
            fll_bw_hz: 10.0,
            fll_err_hz: 30.0,
            apply_fll: true,
            apply_pll_frequency: false,
        });

        let fll_coefficients = first_order_angular_loop_coefficients(10.0, 0.001);
        let expected_carrier_hz = 80.0 + (30.0 * fll_coefficients.error_blend).clamp(-40.0, 40.0);
        assert!((update.carrier_hz - expected_carrier_hz).abs() < 1.0e-9, "{update:?}");
    }

    #[test]
    fn apply_carrier_loop_accumulates_unwrapped_phase_across_epochs() {
        let first = super::apply_carrier_loop(super::CarrierLoopInput {
            current_carrier_hz: 1_500.0,
            current_carrier_phase_cycles: 128.25,
            current_carrier_rate_hz_per_s: 0.0,
            epoch_len_samples: 4_092,
            sample_rate_hz: 4_092_000.0,
            coherent_integration_s: 0.001,
            pll_bw_hz: 8.0,
            pll_err_rad: 0.0,
            fll_bw_hz: 0.0,
            fll_err_hz: 0.0,
            apply_fll: false,
            apply_pll_frequency: true,
        });
        let second = super::apply_carrier_loop(super::CarrierLoopInput {
            current_carrier_hz: first.carrier_hz,
            current_carrier_phase_cycles: first.carrier_phase_cycles,
            current_carrier_rate_hz_per_s: first.carrier_rate_hz_per_s,
            epoch_len_samples: 4_092,
            sample_rate_hz: 4_092_000.0,
            coherent_integration_s: 0.001,
            pll_bw_hz: 8.0,
            pll_err_rad: 0.0,
            fll_bw_hz: 0.0,
            fll_err_hz: 0.0,
            apply_fll: false,
            apply_pll_frequency: true,
        });

        assert!((first.carrier_phase_cycles - 129.75).abs() < 1.0e-9, "{first:?}");
        assert!((second.carrier_phase_cycles - 131.25).abs() < 1.0e-9, "{second:?}");
        assert!(
            (second.carrier_phase_cycles - first.carrier_phase_cycles - 1.5).abs() < 1.0e-9,
            "{first:?} {second:?}"
        );
    }

    #[test]
    fn apply_carrier_loop_preserves_continuous_phase_for_negative_doppler() {
        let update = super::apply_carrier_loop(super::CarrierLoopInput {
            current_carrier_hz: -850.0,
            current_carrier_phase_cycles: 512.875,
            current_carrier_rate_hz_per_s: 0.0,
            epoch_len_samples: 8_184,
            sample_rate_hz: 4_092_000.0,
            coherent_integration_s: 0.002,
            pll_bw_hz: 8.0,
            pll_err_rad: 0.0,
            fll_bw_hz: 0.0,
            fll_err_hz: 0.0,
            apply_fll: false,
            apply_pll_frequency: true,
        });

        assert!((update.carrier_hz + 850.0).abs() < 1.0e-9, "{update:?}");
        assert!((update.carrier_phase_cycles - 511.175).abs() < 1.0e-9, "{update:?}");
    }

    #[test]
    fn apply_carrier_loop_uses_coherent_interval_to_scale_frequency_gain() {
        let short = super::apply_carrier_loop(super::CarrierLoopInput {
            current_carrier_hz: 1_000.0,
            current_carrier_phase_cycles: 12.0,
            current_carrier_rate_hz_per_s: 0.0,
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
        let long = super::apply_carrier_loop(super::CarrierLoopInput {
            current_carrier_hz: 1_000.0,
            current_carrier_phase_cycles: 12.0,
            current_carrier_rate_hz_per_s: 0.0,
            epoch_len_samples: 40_920,
            sample_rate_hz: 4_092_000.0,
            coherent_integration_s: 0.010,
            pll_bw_hz: 8.0,
            pll_err_rad: 0.25,
            fll_bw_hz: 0.0,
            fll_err_hz: 0.0,
            apply_fll: false,
            apply_pll_frequency: true,
        });

        assert!(
            (long.carrier_hz - 1_000.0).abs() > (short.carrier_hz - 1_000.0).abs(),
            "short={short:?} long={long:?}"
        );
    }

    #[test]
    fn dll_pull_in_increases_code_rate_for_faster_signal() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            channels: 12,
            ..ReceiverPipelineConfig::default()
        };
        let tracking = Tracking::new(config.clone(), ReceiverRuntime::default());
        let samples_per_epoch = samples_per_code(
            config.sampling_freq_hz,
            config.code_freq_basis_hz,
            config.code_length,
        );
        let signal_code_rate_hz = config.code_freq_basis_hz + 300.0;
        let code_phase_chips = 211.625;
        let sat = SatId { constellation: bijux_gnss_core::api::Constellation::Gps, prn: 14 };
        let samples = sample_ca_code(
            Prn(sat.prn),
            config.sampling_freq_hz,
            signal_code_rate_hz,
            code_phase_chips,
            samples_per_epoch,
        )
        .expect("valid sampled code with faster signal code rate");
        let frame = SamplesFrame::new(
            SampleTime { sample_index: 0, sample_rate_hz: config.sampling_freq_hz },
            Seconds(1.0 / config.sampling_freq_hz),
            samples.into_iter().map(|value| Complex::new(value, 0.0)).collect(),
        );
        let code_phase_samples = crate::sim::synthetic::expected_acquisition_code_phase_samples_f64(
            &config,
            &frame,
            code_phase_chips,
        );

        let correlator = tracking.correlate_epoch(
            &frame,
            sat,
            0.0,
            0.0,
            config.code_freq_basis_hz,
            code_phase_samples,
            0.5,
        );
        let (dll_err, _, _, _) =
            discriminators(correlator.early, correlator.prompt, correlator.late, None);
        let code_loop = super::apply_dll_code_loop(super::CodeLoopInput {
            current_code_rate_hz: config.code_freq_basis_hz,
            previous_reference_code_rate_hz: config.code_freq_basis_hz,
            reference_code_rate_hz: config.code_freq_basis_hz,
            current_code_phase_samples: code_phase_samples,
            epoch_len_samples: samples_per_epoch,
            coherent_integration_s: samples_per_epoch as f64 / config.sampling_freq_hz,
            nominal_code_rate_hz: config.code_freq_basis_hz,
            dll_bw_hz: 900.0,
            dll_err,
            samples_per_chip: samples_per_epoch as f64 / config.code_length as f64,
            samples_per_code: samples_per_epoch,
        });

        assert!(
            code_loop.code_rate_hz > config.code_freq_basis_hz,
            "dll_err={dll_err} code_loop={code_loop:?}",
        );
    }

    #[test]
    fn detect_sample_rate_mismatch_flags_persistent_phase_drift() {
        let sat = SatId { constellation: bijux_gnss_core::api::Constellation::Gps, prn: 11 };
        let epochs = [0.0, 24.0, 48.0, 72.0, 96.0]
            .into_iter()
            .enumerate()
            .map(|(index, code_phase_samples)| TrackEpoch {
                epoch: Epoch { index: index as u64 },
                sample_index: (index as u64) * 5_000,
                sat,
                code_phase_samples: Chips(code_phase_samples),
                dll_err: 0.35,
                cn0_dbhz: 46.0,
                lock: true,
                pll_lock: true,
                dll_lock: true,
                fll_lock: true,
                lock_state: ChannelState::Tracking.to_string(),
                lock_state_reason: Some("locked".to_string()),
                ..TrackEpoch::default()
            })
            .collect::<Vec<_>>();

        let diagnostic =
            super::detect_sample_rate_mismatch(&epochs, 5_000).expect("diagnostic must exist");
        assert_eq!(diagnostic.first_unstable_epoch_index, 1);
        assert!(diagnostic.max_abs_phase_step_samples >= 24.0);
    }

    #[test]
    fn detect_sample_rate_mismatch_ignores_small_stable_phase_steps() {
        let sat = SatId { constellation: bijux_gnss_core::api::Constellation::Gps, prn: 11 };
        let epochs = [0.0, 1.5, 2.0, 1.0, 0.5]
            .into_iter()
            .enumerate()
            .map(|(index, code_phase_samples)| TrackEpoch {
                epoch: Epoch { index: index as u64 },
                sample_index: (index as u64) * 5_000,
                sat,
                code_phase_samples: Chips(code_phase_samples),
                dll_err: 0.05,
                cn0_dbhz: 46.0,
                lock: true,
                pll_lock: true,
                dll_lock: true,
                fll_lock: true,
                lock_state: ChannelState::Tracking.to_string(),
                lock_state_reason: Some("locked".to_string()),
                ..TrackEpoch::default()
            })
            .collect::<Vec<_>>();

        assert!(super::detect_sample_rate_mismatch(&epochs, 5_000).is_none());
    }

    #[test]
    fn detect_sample_rate_mismatch_flags_catastrophic_pull_in_phase_jump() {
        let sat = SatId { constellation: bijux_gnss_core::api::Constellation::Gps, prn: 11 };
        let epochs = [0.0, 2.5, 4.0, 4_452.0, 4_452.5]
            .into_iter()
            .enumerate()
            .map(|(index, code_phase_samples)| TrackEpoch {
                epoch: Epoch { index: index as u64 },
                sample_index: (index as u64) * 5_050,
                sat,
                code_phase_samples: Chips(code_phase_samples),
                cn0_dbhz: 48.0,
                lock: true,
                fll_lock: index > 0,
                lock_state: ChannelState::PullIn.to_string(),
                lock_state_reason: Some("carrier_pull_in".to_string()),
                ..TrackEpoch::default()
            })
            .collect::<Vec<_>>();

        let diagnostic =
            super::detect_sample_rate_mismatch(&epochs, 5_050).expect("diagnostic must exist");
        assert_eq!(diagnostic.first_unstable_epoch_index, 3);
        assert!(diagnostic.max_abs_phase_step_samples >= 500.0);
    }

    #[test]
    fn tracking_lock_detector_thresholds_derive_from_spacing_and_dynamics() {
        let tracking_params = super::TrackingParams {
            early_late_spacing_chips: 0.5,
            dll_bw_hz: 2.0,
            pll_bw_hz: 15.0,
            fll_bw_hz: 10.0,
            integration_ms: 1,
        };
        let high_resolution =
            super::tracking_lock_detector_thresholds(45.0, 0.001, 4.0, tracking_params, 0.0);
        let low_resolution =
            super::tracking_lock_detector_thresholds(45.0, 0.001, 1.0, tracking_params, 25_000.0);

        assert!(low_resolution.dll_lock > high_resolution.dll_lock);
        assert!(low_resolution.fll_lock_hz > high_resolution.fll_lock_hz);
        assert!(high_resolution.pll_hold_rad > high_resolution.pll_lock_rad);
    }

    #[test]
    fn low_resolution_code_lock_retains_supported_tracking_when_carrier_is_stable() {
        assert!(super::low_resolution_code_lock(1.0, 0.5, true, true, true, false, false));
        assert!(super::low_resolution_code_lock(1.0, 0.5, true, false, true, false, false));
    }

    #[test]
    fn low_resolution_code_lock_requires_prompt_and_lock_safety_guards() {
        assert!(!super::low_resolution_code_lock(1.0, 0.5, false, true, true, false, false));
        assert!(!super::low_resolution_code_lock(1.0, 0.5, true, false, false, false, false));
        assert!(!super::low_resolution_code_lock(1.0, 0.5, true, true, true, true, false));
        assert!(!super::low_resolution_code_lock(1.0, 0.5, true, true, true, false, true));
        assert!(!super::low_resolution_code_lock(4.0, 0.5, true, true, true, false, false));
    }

    #[test]
    fn clean_seeded_tracking_clears_false_lock_once_loops_converge() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            channels: 1,
            early_late_spacing_chips: 0.5,
            dll_bw_hz: 2.0,
            pll_bw_hz: 18.0,
            fll_bw_hz: 12.0,
            ..ReceiverPipelineConfig::default()
        };
        let sat = SatId { constellation: Constellation::Gps, prn: 7 };
        let frame = generate_l1_ca(
            &config,
            SyntheticSignalParams {
                sat,
                glonass_frequency_channel: None,
                signal_band: bijux_gnss_core::api::SignalBand::L1,
                signal_code: bijux_gnss_core::api::SignalCode::Unknown,
                doppler_hz: -750.0,
                code_phase_chips: 211.25,
                carrier_phase_rad: 0.2,
                cn0_db_hz: 52.0,
                navigation_data: false.into(),
            },
            0x710C_A000,
            0.012,
        );
        let code_phase_samples =
            crate::sim::synthetic::expected_acquisition_code_phase_samples(&config, &frame, 211.25);
        let tracking = Tracking::new(config, ReceiverRuntime::default());
        let tracks = tracking.track_from_acquisition(
            &frame,
            &[bijux_gnss_core::api::AcqResult {
                sat,
                signal_band: bijux_gnss_core::api::SignalBand::L1,
                signal_code: bijux_gnss_core::api::SignalCode::Unknown,
                glonass_frequency_channel: None,
                source_time: bijux_gnss_core::api::ReceiverSampleTrace::default(),
                candidate_rank: 1,
                is_primary_candidate: true,
                doppler_hz: bijux_gnss_core::api::Hertz(-750.0),
                doppler_rate_hz_per_s: 0.0,
                carrier_hz: bijux_gnss_core::api::Hertz(-750.0),
                code_phase_samples,
                peak: 1.0,
                second_peak: 0.1,
                mean: 0.01,
                peak_mean_ratio: 20.0,
                peak_second_ratio: 10.0,
                cn0_proxy: 52.0,
                score: 1.0,
                hypothesis: bijux_gnss_core::api::AcqHypothesis::Accepted,
                assumptions: None,
                evidence: Vec::new(),
                threshold_provenance: None,
                explain_selection_reason: Some("clean_seeded_tracking".to_string()),
                doppler_refinement: None,
                code_phase_refinement: None,
                signal_delay_alignment: None,
                uncertainty: None,
            }],
        );
        let epochs = &tracks.first().expect("track").epochs;

        assert!(
            epochs
                .iter()
                .skip(4)
                .filter(|epoch| epoch.pll_lock && epoch.fll_lock)
                .all(|epoch| !epoch.anti_false_lock),
            "clean, converged tracking epochs must not remain marked as false lock: {epochs:?}"
        );
    }

    #[test]
    fn correlate_epoch_honors_receiver_code_phase_seed_at_low_sample_rate() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 2_046_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            channels: 1,
            early_late_spacing_chips: 0.5,
            dll_bw_hz: 2.0,
            pll_bw_hz: 18.0,
            fll_bw_hz: 12.0,
            ..ReceiverPipelineConfig::default()
        };
        let sat = SatId { constellation: Constellation::Gps, prn: 3 };
        let frame = generate_l1_ca(
            &config,
            SyntheticSignalParams {
                sat,
                glonass_frequency_channel: None,
                signal_band: bijux_gnss_core::api::SignalBand::L1,
                signal_code: bijux_gnss_core::api::SignalCode::Unknown,
                doppler_hz: 750.0,
                code_phase_chips: 200.25,
                carrier_phase_rad: 0.0,
                cn0_db_hz: 58.0,
                navigation_data: false.into(),
            },
            0x330C_2000,
            0.04,
        );
        let refined_code_phase_samples =
            crate::sim::synthetic::expected_acquisition_code_phase_samples_f64(
                &config, &frame, 200.25,
            );
        let tracking = Tracking::new(config, ReceiverRuntime::default());
        let correlator = tracking.correlate_epoch(
            &frame,
            sat,
            750.0,
            0.0,
            1_023_000.0,
            refined_code_phase_samples,
            0.5,
        );

        assert!(
            correlator.prompt.norm() > 10_000.0,
            "receiver-seeded code phase must produce a strong prompt correlation: {:?}",
            correlator.prompt
        );
        assert!(
            correlator.prompt.re.abs() > correlator.prompt.im.abs() * 100.0,
            "receiver-seeded code phase must align carrier phase near the real axis: {:?}",
            correlator.prompt
        );
    }

    #[test]
    fn incremental_tracking_matches_single_frame_tracking() {
        let config = crate::engine::receiver_config::ReceiverPipelineConfig {
            sampling_freq_hz: 1_023_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            channels: 4,
            ..crate::engine::receiver_config::ReceiverPipelineConfig::default()
        };
        let runtime = crate::engine::runtime::ReceiverRuntime::default();
        let tracking = Tracking::new(config.clone(), runtime.clone());
        let sat = SatId { constellation: bijux_gnss_core::api::Constellation::Gps, prn: 3 };
        let duration_s = 0.012;
        let frame = crate::sim::synthetic::generate_l1_ca(
            &config,
            crate::sim::synthetic::SyntheticSignalParams {
                sat,
                glonass_frequency_channel: None,
                signal_band: bijux_gnss_core::api::SignalBand::L1,
                signal_code: bijux_gnss_core::api::SignalCode::Unknown,
                doppler_hz: 1_000.0,
                code_phase_chips: 10.0,
                carrier_phase_rad: 0.0,
                cn0_db_hz: 48.0,
                navigation_data: false.into(),
            },
            7,
            duration_s,
        );
        let acquisition = bijux_gnss_core::api::AcqResult {
            sat,
            signal_band: bijux_gnss_core::api::SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Unknown,
            glonass_frequency_channel: None,
            source_time: bijux_gnss_core::api::ReceiverSampleTrace::from_sample_time(frame.t0),
            candidate_rank: 1,
            is_primary_candidate: true,
            doppler_hz: bijux_gnss_core::api::Hertz(1_000.0),
            doppler_rate_hz_per_s: 0.0,
            carrier_hz: bijux_gnss_core::api::Hertz(1_000.0),
            code_phase_samples: 10,
            peak: 10.0,
            second_peak: 2.0,
            mean: 1.0,
            peak_mean_ratio: 10.0,
            peak_second_ratio: 5.0,
            cn0_proxy: 48.0,
            score: 0.98,
            hypothesis: AcqHypothesis::Accepted,
            assumptions: None,
            evidence: Vec::new(),
            threshold_provenance: None,
            explain_selection_reason: None,
            doppler_refinement: None,
            code_phase_refinement: None,
            signal_delay_alignment: None,
            uncertainty: None,
        };

        let single = tracking.track_from_acquisition(&frame, &[acquisition.clone()]);
        let mut incremental = tracking.begin_incremental_tracking(&[acquisition]);
        for chunk in split_frame(&frame, 4 * 1_023) {
            tracking.track_incremental_frame(&mut incremental, &chunk);
        }
        let streamed = tracking.finish_incremental_tracking(incremental);

        assert_eq!(single.len(), streamed.len());
        assert_eq!(single[0].epochs.len(), streamed[0].epochs.len());
        let single_keys = single[0]
            .epochs
            .iter()
            .map(|epoch| {
                (
                    epoch.epoch.index,
                    epoch.sample_index,
                    epoch.lock,
                    epoch.lock_state.clone(),
                    epoch.lock_state_reason.clone(),
                )
            })
            .collect::<Vec<_>>();
        let streamed_keys = streamed[0]
            .epochs
            .iter()
            .map(|epoch| {
                (
                    epoch.epoch.index,
                    epoch.sample_index,
                    epoch.lock,
                    epoch.lock_state.clone(),
                    epoch.lock_state_reason.clone(),
                )
            })
            .collect::<Vec<_>>();
        assert_eq!(single_keys, streamed_keys);
    }

    #[test]
    fn begin_incremental_tracking_prioritizes_stronger_trackable_acquisitions() {
        let config = crate::engine::receiver_config::ReceiverPipelineConfig {
            channels: 2,
            ..crate::engine::receiver_config::ReceiverPipelineConfig::default()
        };
        let tracking = Tracking::new(config, ReceiverRuntime::default());
        let acquisitions = vec![
            bijux_gnss_core::api::AcqResult {
                sat: SatId { constellation: Constellation::Gps, prn: 3 },
                signal_band: SignalBand::L1,
                signal_code: bijux_gnss_core::api::SignalCode::Unknown,
                glonass_frequency_channel: None,
                source_time: ReceiverSampleTrace::default(),
                candidate_rank: 1,
                is_primary_candidate: true,
                doppler_hz: Hertz(0.0),
                doppler_rate_hz_per_s: 0.0,
                carrier_hz: Hertz(0.0),
                code_phase_samples: 0,
                peak: 1.0,
                second_peak: 0.1,
                mean: 0.01,
                peak_mean_ratio: 6.0,
                peak_second_ratio: 2.0,
                cn0_proxy: 38.0,
                score: 1.4,
                hypothesis: AcqHypothesis::Ambiguous,
                assumptions: None,
                evidence: Vec::new(),
                threshold_provenance: None,
                explain_selection_reason: None,
                doppler_refinement: None,
                code_phase_refinement: None,
                signal_delay_alignment: None,
                uncertainty: None,
            },
            bijux_gnss_core::api::AcqResult {
                sat: SatId { constellation: Constellation::Gps, prn: 7 },
                signal_band: SignalBand::L1,
                signal_code: bijux_gnss_core::api::SignalCode::Unknown,
                glonass_frequency_channel: None,
                source_time: ReceiverSampleTrace::default(),
                candidate_rank: 1,
                is_primary_candidate: true,
                doppler_hz: Hertz(0.0),
                doppler_rate_hz_per_s: 0.0,
                carrier_hz: Hertz(0.0),
                code_phase_samples: 0,
                peak: 1.0,
                second_peak: 0.1,
                mean: 0.01,
                peak_mean_ratio: 8.0,
                peak_second_ratio: 3.0,
                cn0_proxy: 52.0,
                score: 5.0,
                hypothesis: AcqHypothesis::Accepted,
                assumptions: None,
                evidence: Vec::new(),
                threshold_provenance: None,
                explain_selection_reason: None,
                doppler_refinement: None,
                code_phase_refinement: None,
                signal_delay_alignment: None,
                uncertainty: None,
            },
            bijux_gnss_core::api::AcqResult {
                sat: SatId { constellation: Constellation::Gps, prn: 23 },
                signal_band: SignalBand::L1,
                signal_code: bijux_gnss_core::api::SignalCode::Unknown,
                glonass_frequency_channel: None,
                source_time: ReceiverSampleTrace::default(),
                candidate_rank: 1,
                is_primary_candidate: true,
                doppler_hz: Hertz(0.0),
                doppler_rate_hz_per_s: 0.0,
                carrier_hz: Hertz(0.0),
                code_phase_samples: 0,
                peak: 1.0,
                second_peak: 0.1,
                mean: 0.01,
                peak_mean_ratio: 9.0,
                peak_second_ratio: 3.5,
                cn0_proxy: 48.0,
                score: 3.5,
                hypothesis: AcqHypothesis::Ambiguous,
                assumptions: None,
                evidence: Vec::new(),
                threshold_provenance: None,
                explain_selection_reason: None,
                doppler_refinement: None,
                code_phase_refinement: None,
                signal_delay_alignment: None,
                uncertainty: None,
            },
        ];

        let incremental = tracking.begin_incremental_tracking(&acquisitions);
        let selected_prns =
            incremental.channels.iter().map(|channel| channel.sat.prn).collect::<Vec<_>>();

        assert_eq!(selected_prns, vec![7, 23]);
    }

    #[test]
    fn begin_incremental_tracking_preserves_glonass_channel_and_carrier_seed() {
        let channel = bijux_gnss_core::api::GlonassFrequencyChannel::new(-4)
            .expect("channel -4 must be valid");
        let carrier_hz = 2_048_125.0;
        let config = crate::engine::receiver_config::ReceiverPipelineConfig {
            code_freq_basis_hz: 511_000.0,
            code_length: 511,
            ..crate::engine::receiver_config::ReceiverPipelineConfig::default()
        };
        let tracking = Tracking::new(config, ReceiverRuntime::default());
        let acquisition = bijux_gnss_core::api::AcqResult {
            sat: SatId { constellation: Constellation::Glonass, prn: 8 },
            signal_band: SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Unknown,
            glonass_frequency_channel: Some(channel),
            source_time: ReceiverSampleTrace::default(),
            candidate_rank: 1,
            is_primary_candidate: true,
            doppler_hz: Hertz(125.0),
            doppler_rate_hz_per_s: 0.0,
            carrier_hz: Hertz(carrier_hz),
            code_phase_samples: 37,
            peak: 1.0,
            second_peak: 0.1,
            mean: 0.01,
            peak_mean_ratio: 8.0,
            peak_second_ratio: 3.0,
            cn0_proxy: 48.0,
            score: 5.0,
            hypothesis: AcqHypothesis::Accepted,
            assumptions: None,
            evidence: Vec::new(),
            threshold_provenance: None,
            explain_selection_reason: None,
            doppler_refinement: None,
            code_phase_refinement: None,
            signal_delay_alignment: None,
            uncertainty: None,
        };

        let incremental = tracking.begin_incremental_tracking(&[acquisition]);
        let channel_state = incremental.channels.first().expect("GLONASS tracking channel");

        assert_eq!(channel_state.signal_model.signal_band, SignalBand::L1);
        assert_eq!(channel_state.signal_model.glonass_frequency_channel, Some(channel));
        assert_eq!(channel_state.state.carrier_hz, carrier_hz);
        assert!((channel_state.signal_model.code_rate_hz - 511_000.0).abs() <= f64::EPSILON);
        assert_eq!(channel_state.signal_model.code_length, 511);
    }

    #[test]
    fn begin_incremental_tracking_seeds_code_rate_from_carrier_aid() {
        let config = crate::engine::receiver_config::ReceiverPipelineConfig {
            intermediate_freq_hz: GPS_L1_CA_CARRIER_HZ.value()
                - signal_spec_gps_l5_i().carrier_hz.value(),
            code_freq_basis_hz: 10_230_000.0,
            code_length: 10_230,
            ..crate::engine::receiver_config::ReceiverPipelineConfig::default()
        };
        let tracking = Tracking::new(config, ReceiverRuntime::default());
        let acquisition = bijux_gnss_core::api::AcqResult {
            sat: SatId { constellation: Constellation::Gps, prn: 18 },
            signal_band: SignalBand::L5,
            signal_code: SignalCode::L5I,
            glonass_frequency_channel: None,
            source_time: ReceiverSampleTrace::default(),
            candidate_rank: 1,
            is_primary_candidate: true,
            doppler_hz: Hertz(1_500.0),
            doppler_rate_hz_per_s: 0.0,
            carrier_hz: Hertz(1_500.0),
            code_phase_samples: 37,
            peak: 1.0,
            second_peak: 0.1,
            mean: 0.01,
            peak_mean_ratio: 8.0,
            peak_second_ratio: 3.0,
            cn0_proxy: 48.0,
            score: 5.0,
            hypothesis: AcqHypothesis::Accepted,
            assumptions: None,
            evidence: Vec::new(),
            threshold_provenance: None,
            explain_selection_reason: None,
            doppler_refinement: None,
            code_phase_refinement: None,
            signal_delay_alignment: None,
            uncertainty: None,
        };

        let incremental = tracking.begin_incremental_tracking(&[acquisition]);
        let channel_state = incremental.channels.first().expect("GPS L5 tracking channel");
        let expected_code_rate_hz =
            shared_path_code_rate_hz(1_500.0, signal_spec_gps_l5_i(), signal_spec_gps_l5_i())
                .expect("carrier-aided code rate");

        assert!((channel_state.state.code_rate_hz - expected_code_rate_hz).abs() <= 1.0e-9);
        assert!(
            (channel_state.state.code_rate_reference_hz - expected_code_rate_hz).abs() <= 1.0e-9
        );
    }

    #[test]
    fn begin_incremental_tracking_refuses_incompatible_carrier_aiding_seed() {
        let config = crate::engine::receiver_config::ReceiverPipelineConfig {
            intermediate_freq_hz: GPS_L1_CA_CARRIER_HZ.value()
                - signal_spec_gps_l5_i().carrier_hz.value(),
            code_freq_basis_hz: 10_230_000.0,
            code_length: 10_230,
            ..crate::engine::receiver_config::ReceiverPipelineConfig::default()
        };
        let tracking = Tracking::new(config, ReceiverRuntime::default());
        let acquisition = bijux_gnss_core::api::AcqResult {
            sat: SatId { constellation: Constellation::Gps, prn: 18 },
            signal_band: SignalBand::L5,
            signal_code: SignalCode::L5I,
            glonass_frequency_channel: None,
            source_time: ReceiverSampleTrace::default(),
            candidate_rank: 1,
            is_primary_candidate: true,
            doppler_hz: Hertz(1_500.0),
            doppler_rate_hz_per_s: 0.0,
            carrier_hz: Hertz(GPS_L1_CA_CARRIER_HZ.value() + 1_500.0),
            code_phase_samples: 37,
            peak: 1.0,
            second_peak: 0.1,
            mean: 0.01,
            peak_mean_ratio: 8.0,
            peak_second_ratio: 3.0,
            cn0_proxy: 48.0,
            score: 5.0,
            hypothesis: AcqHypothesis::Accepted,
            assumptions: None,
            evidence: Vec::new(),
            threshold_provenance: None,
            explain_selection_reason: None,
            doppler_refinement: None,
            code_phase_refinement: None,
            signal_delay_alignment: None,
            uncertainty: None,
        };

        let incremental = tracking.begin_incremental_tracking(&[acquisition]);

        assert!(
            incremental.channels.is_empty(),
            "wrong-band carrier seeds must not enter carrier-aided tracking: {incremental:?}",
        );
    }

    #[test]
    fn tracking_signal_model_uses_registry_metadata_for_gps_l5q() {
        let config = crate::engine::receiver_config::ReceiverPipelineConfig {
            code_freq_basis_hz: 10_230_000.0,
            code_length: 10_230,
            ..crate::engine::receiver_config::ReceiverPipelineConfig::default()
        };
        let sat = SatId { constellation: Constellation::Gps, prn: 18 };

        let signal_model = super::TrackingSignalModel::for_sat_signal_band(
            &config,
            sat,
            SignalBand::L5,
            SignalCode::L5Q,
            None,
        );

        assert_eq!(signal_model.component_role, SignalComponentRole::Pilot);
        assert!(signal_model.secondary_code.is_some());
        assert_eq!(
            signal_model.phase_transition_source,
            super::TrackingPhaseTransitionSource::SecondaryCode
        );
        assert_eq!(
            signal_model.discriminator_family,
            super::TrackingDiscriminatorFamily::EarlyPromptLate
        );
        assert!((signal_model.nominal_carrier_hz() - 1_176_450_000.0).abs() <= f64::EPSILON);
        assert!(!signal_model.supports_navigation_bit_sign_recovery());
    }

    #[test]
    fn tracked_signal_center_hz_rebases_non_l1_signals_into_receiver_if() {
        let signal = signal_spec_gps_l5_i();
        let center_hz = super::tracked_signal_center_hz(0.0, signal);

        assert!(
            (center_hz - (signal.carrier_hz.value() - GPS_L1_CA_CARRIER_HZ.value())).abs()
                <= f64::EPSILON
        );
    }

    #[test]
    fn tracked_signal_doppler_hz_removes_signal_specific_center_offset() {
        let signal = signal_spec_gps_l5_i();
        let tracked_carrier_hz = super::tracked_signal_center_hz(0.0, signal) + 875.0;

        assert!(
            (super::tracked_signal_doppler_hz(0.0, tracked_carrier_hz, signal) - 875.0).abs()
                <= f64::EPSILON
        );
    }

    #[test]
    fn normalize_acquisition_carrier_hz_rebases_doppler_only_l5_seed() {
        let config = crate::engine::receiver_config::ReceiverPipelineConfig {
            code_freq_basis_hz: 10_230_000.0,
            code_length: 10_230,
            ..crate::engine::receiver_config::ReceiverPipelineConfig::default()
        };
        let sat = SatId { constellation: Constellation::Gps, prn: 18 };
        let signal_model = super::TrackingSignalModel::for_sat_signal_band(
            &config,
            sat,
            SignalBand::L5,
            SignalCode::L5I,
            None,
        );
        let acquisition = bijux_gnss_core::api::AcqResult {
            sat,
            signal_band: SignalBand::L5,
            signal_code: SignalCode::L5I,
            glonass_frequency_channel: None,
            source_time: ReceiverSampleTrace::default(),
            candidate_rank: 1,
            is_primary_candidate: true,
            doppler_hz: Hertz(180.0),
            doppler_rate_hz_per_s: 0.0,
            carrier_hz: Hertz(180.0),
            code_phase_samples: 0,
            peak: 1.0,
            second_peak: 0.1,
            mean: 0.01,
            peak_mean_ratio: 20.0,
            peak_second_ratio: 10.0,
            cn0_proxy: 60.0,
            score: 1.0,
            hypothesis: AcqHypothesis::Accepted,
            assumptions: None,
            evidence: Vec::new(),
            threshold_provenance: None,
            explain_selection_reason: Some("seeded_joint_component_tracking".to_string()),
            doppler_refinement: None,
            code_phase_refinement: None,
            signal_delay_alignment: None,
            uncertainty: None,
        };

        let normalized =
            super::normalize_acquisition_carrier_hz(&config, &signal_model, &acquisition);

        assert!(
            (normalized
                - (super::tracked_signal_center_hz(
                    config.intermediate_freq_hz,
                    signal_model.signal_spec
                ) + acquisition.doppler_hz.0))
                .abs()
                <= f64::EPSILON
        );
    }

    #[test]
    fn normalize_acquisition_carrier_hz_keeps_absolute_wideband_carrier() {
        let config = crate::engine::receiver_config::ReceiverPipelineConfig {
            code_freq_basis_hz: 10_230_000.0,
            code_length: 10_230,
            ..crate::engine::receiver_config::ReceiverPipelineConfig::default()
        };
        let sat = SatId { constellation: Constellation::Galileo, prn: 11 };
        let signal_model = super::TrackingSignalModel::for_sat_signal_band(
            &config,
            sat,
            SignalBand::E5,
            SignalCode::E5b,
            None,
        );
        let absolute_carrier_hz =
            super::tracked_signal_center_hz(config.intermediate_freq_hz, signal_model.signal_spec)
                + 250.0;
        let acquisition = bijux_gnss_core::api::AcqResult {
            sat,
            signal_band: SignalBand::E5,
            signal_code: SignalCode::E5b,
            glonass_frequency_channel: None,
            source_time: ReceiverSampleTrace::default(),
            candidate_rank: 1,
            is_primary_candidate: true,
            doppler_hz: Hertz(250.0),
            doppler_rate_hz_per_s: 0.0,
            carrier_hz: Hertz(absolute_carrier_hz),
            code_phase_samples: 0,
            peak: 1.0,
            second_peak: 0.1,
            mean: 0.01,
            peak_mean_ratio: 20.0,
            peak_second_ratio: 10.0,
            cn0_proxy: 60.0,
            score: 1.0,
            hypothesis: AcqHypothesis::Accepted,
            assumptions: None,
            evidence: Vec::new(),
            threshold_provenance: None,
            explain_selection_reason: None,
            doppler_refinement: None,
            code_phase_refinement: None,
            signal_delay_alignment: None,
            uncertainty: None,
        };

        assert_eq!(
            super::normalize_acquisition_carrier_hz(&config, &signal_model, &acquisition),
            absolute_carrier_hz
        );
    }

    #[test]
    fn carrier_aided_code_rate_hz_uses_tracked_signal_center_and_code_rate() {
        let config = crate::engine::receiver_config::ReceiverPipelineConfig {
            code_freq_basis_hz: 10_230_000.0,
            code_length: 10_230,
            ..crate::engine::receiver_config::ReceiverPipelineConfig::default()
        };
        let sat = SatId { constellation: Constellation::Gps, prn: 18 };
        let signal_model = super::TrackingSignalModel::for_sat_signal_band(
            &config,
            sat,
            SignalBand::L5,
            SignalCode::L5I,
            None,
        );
        let tracked_carrier_hz =
            super::tracked_signal_center_hz(config.intermediate_freq_hz, signal_model.signal_spec)
                + 875.0;
        let aided_code_rate_hz =
            super::carrier_aided_code_rate_hz(&config, &signal_model, tracked_carrier_hz);
        let expected = signal_model.code_rate_hz
            + (875.0 * signal_model.code_rate_hz / signal_model.signal_spec.carrier_hz.value());

        assert!((aided_code_rate_hz - expected).abs() <= 1.0e-9);
    }

    #[test]
    fn carrier_aiding_reference_rejects_cross_band_center() {
        let config = crate::engine::receiver_config::ReceiverPipelineConfig {
            intermediate_freq_hz: GPS_L1_CA_CARRIER_HZ.value()
                - signal_spec_gps_l5_i().carrier_hz.value(),
            code_freq_basis_hz: 10_230_000.0,
            code_length: 10_230,
            ..crate::engine::receiver_config::ReceiverPipelineConfig::default()
        };
        let sat = SatId { constellation: Constellation::Gps, prn: 18 };
        let signal_model = super::TrackingSignalModel::for_sat_signal_band(
            &config,
            sat,
            SignalBand::L5,
            SignalCode::L5I,
            None,
        );

        let error = super::carrier_aiding_reference(
            &config,
            &signal_model,
            GPS_L1_CA_CARRIER_HZ.value() + 875.0,
        )
        .expect_err("GPS L1 carrier must not aid a GPS L5 code loop");

        assert_eq!(error, "carrier_aiding_incompatible_signal_center");
    }

    #[test]
    fn next_code_rate_reference_hz_holds_previous_value_until_carrier_lock_is_ready() {
        let config = crate::engine::receiver_config::ReceiverPipelineConfig {
            code_freq_basis_hz: 10_230_000.0,
            code_length: 10_230,
            ..crate::engine::receiver_config::ReceiverPipelineConfig::default()
        };
        let sat = SatId { constellation: Constellation::Gps, prn: 18 };
        let signal_model = super::TrackingSignalModel::for_sat_signal_band(
            &config,
            sat,
            SignalBand::L5,
            SignalCode::L5I,
            None,
        );
        let previous_reference_hz = signal_model.code_rate_hz;
        let tracked_carrier_hz =
            super::tracked_signal_center_hz(config.intermediate_freq_hz, signal_model.signal_spec)
                + 875.0;

        assert_eq!(
            super::next_code_rate_reference_hz(
                &config,
                &signal_model,
                tracked_carrier_hz,
                previous_reference_hz,
                false,
            ),
            previous_reference_hz,
        );
    }

    #[test]
    fn tracking_signal_model_resolves_joint_components_for_gps_l2c() {
        let config = crate::engine::receiver_config::ReceiverPipelineConfig {
            code_freq_basis_hz: 511_500.0,
            code_length: 10_230,
            ..crate::engine::receiver_config::ReceiverPipelineConfig::default()
        };
        let sat = SatId { constellation: Constellation::Gps, prn: 12 };

        let signal_model = super::TrackingSignalModel::for_sat_signal_band(
            &config,
            sat,
            SignalBand::L2,
            SignalCode::L2C,
            None,
        );

        assert_eq!(signal_model.aiding_mode, super::TrackingAidingMode::PilotCarrier);
        assert_eq!(signal_model.component_role, SignalComponentRole::Data);
        assert!(signal_model.pilot_component.is_some());
        assert!(signal_model.data_symbol_component.is_some());
        assert_eq!(
            signal_model.carrier_phase_transition_source(),
            super::TrackingPhaseTransitionSource::None
        );
        assert!(signal_model.supports_epoch_data_symbol_sign_recovery());
    }

    #[test]
    fn tracking_signal_model_resolves_joint_components_for_gps_l5i() {
        let config = crate::engine::receiver_config::ReceiverPipelineConfig {
            code_freq_basis_hz: 10_230_000.0,
            code_length: 10_230,
            ..crate::engine::receiver_config::ReceiverPipelineConfig::default()
        };
        let sat = SatId { constellation: Constellation::Gps, prn: 18 };

        let signal_model = super::TrackingSignalModel::for_sat_signal_band(
            &config,
            sat,
            SignalBand::L5,
            SignalCode::L5I,
            None,
        );

        assert_eq!(signal_model.aiding_mode, super::TrackingAidingMode::PilotCarrier);
        assert_eq!(signal_model.component_role, SignalComponentRole::Data);
        assert!(signal_model.pilot_component.is_some());
        assert!(signal_model.data_symbol_component.is_some());
        assert_eq!(
            signal_model.carrier_phase_transition_source(),
            super::TrackingPhaseTransitionSource::SecondaryCode
        );
        assert!(signal_model.supports_epoch_data_symbol_sign_recovery());
    }

    #[test]
    fn tracking_signal_model_resolves_joint_components_for_galileo_e5b() {
        let config = crate::engine::receiver_config::ReceiverPipelineConfig {
            code_freq_basis_hz: 10_230_000.0,
            code_length: 10_230,
            ..crate::engine::receiver_config::ReceiverPipelineConfig::default()
        };
        let sat = SatId { constellation: Constellation::Galileo, prn: 11 };

        let signal_model = super::TrackingSignalModel::for_sat_signal_band(
            &config,
            sat,
            SignalBand::E5,
            SignalCode::E5b,
            None,
        );

        assert_eq!(signal_model.aiding_mode, super::TrackingAidingMode::PilotCarrier);
        assert_eq!(signal_model.component_role, SignalComponentRole::Data);
        assert!(signal_model.pilot_component.is_some());
        assert!(signal_model.data_symbol_component.is_some());
        assert_eq!(
            signal_model.carrier_phase_transition_source(),
            super::TrackingPhaseTransitionSource::SecondaryCode
        );
        assert!(signal_model.supports_epoch_data_symbol_sign_recovery());
    }

    #[test]
    fn select_carrier_prompt_prefers_pilot_when_it_is_strong_enough() {
        let (selected, source) = super::select_carrier_prompt(
            Complex::new(0.25, 0.05),
            Some(Complex::new(0.60, -0.10)),
            super::TrackingAidingMode::PilotCarrier,
            false,
        );

        assert_eq!(selected, Complex::new(0.60, -0.10));
        assert_eq!(source, super::CarrierPromptSource::Pilot);
    }

    #[test]
    fn recover_epoch_navigation_bit_sign_reads_data_prompt_polarity_for_joint_tracking() {
        let config = crate::engine::receiver_config::ReceiverPipelineConfig {
            code_freq_basis_hz: 10_230_000.0,
            code_length: 10_230,
            ..crate::engine::receiver_config::ReceiverPipelineConfig::default()
        };
        let sat = SatId { constellation: Constellation::Gps, prn: 18 };
        let signal_model = super::TrackingSignalModel::for_sat_signal_band(
            &config,
            sat,
            SignalBand::L5,
            SignalCode::L5I,
            None,
        );

        assert_eq!(
            super::recover_epoch_navigation_bit_sign(
                &signal_model,
                Some(Complex::new(0.75, 0.05)),
                Complex::new(0.80, 0.02),
                super::CarrierPromptSource::Primary,
                0.0,
                true,
            ),
            Some(1)
        );
        assert_eq!(
            super::recover_epoch_navigation_bit_sign(
                &signal_model,
                Some(Complex::new(-0.75, 0.05)),
                Complex::new(0.80, 0.02),
                super::CarrierPromptSource::Primary,
                0.0,
                true,
            ),
            Some(-1)
        );
    }

    #[test]
    fn select_carrier_prompt_keeps_dedicated_galileo_pilot_even_when_primary_is_stronger() {
        let (selected, source) = super::select_carrier_prompt(
            Complex::new(0.90, 0.05),
            Some(Complex::new(0.20, 0.70)),
            super::TrackingAidingMode::PilotCarrier,
            true,
        );

        assert_eq!(selected, Complex::new(0.20, 0.70));
        assert_eq!(source, super::CarrierPromptSource::Pilot);
    }

    #[test]
    fn recover_epoch_navigation_bit_sign_compensates_half_cycle_pilot_flips() {
        let config = crate::engine::receiver_config::ReceiverPipelineConfig {
            code_freq_basis_hz: 10_230_000.0,
            code_length: 10_230,
            ..crate::engine::receiver_config::ReceiverPipelineConfig::default()
        };
        let sat = SatId { constellation: Constellation::Gps, prn: 18 };
        let signal_model = super::TrackingSignalModel::for_sat_signal_band(
            &config,
            sat,
            SignalBand::L5,
            SignalCode::L5I,
            None,
        );

        assert_eq!(
            super::recover_epoch_navigation_bit_sign(
                &signal_model,
                Some(Complex::new(0.72, -0.08)),
                Complex::new(-0.83, 0.04),
                super::CarrierPromptSource::Primary,
                0.5,
                true,
            ),
            Some(1)
        );
        assert_eq!(
            super::recover_epoch_navigation_bit_sign(
                &signal_model,
                Some(Complex::new(-0.72, 0.08)),
                Complex::new(-0.83, 0.04),
                super::CarrierPromptSource::Primary,
                0.5,
                true,
            ),
            Some(-1)
        );
    }

    #[test]
    fn recover_epoch_navigation_bit_sign_projects_onto_carrier_reference() {
        let config = crate::engine::receiver_config::ReceiverPipelineConfig {
            code_freq_basis_hz: 10_230_000.0,
            code_length: 10_230,
            ..crate::engine::receiver_config::ReceiverPipelineConfig::default()
        };
        let sat = SatId { constellation: Constellation::Galileo, prn: 11 };
        let signal_model = super::TrackingSignalModel::for_sat_signal_band(
            &config,
            sat,
            SignalBand::E5,
            SignalCode::E5b,
            None,
        );

        let carrier_reference = Complex::new(0.50, 0.50);
        let positive_data = Complex::new(0.62, 0.58);
        let negative_data = -positive_data;

        assert_eq!(
            super::recover_epoch_navigation_bit_sign(
                &signal_model,
                Some(positive_data),
                carrier_reference,
                super::CarrierPromptSource::Primary,
                0.0,
                true,
            ),
            Some(1)
        );
        assert_eq!(
            super::recover_epoch_navigation_bit_sign(
                &signal_model,
                Some(negative_data),
                carrier_reference,
                super::CarrierPromptSource::Primary,
                0.0,
                true,
            ),
            Some(-1)
        );
    }

    #[test]
    fn recover_epoch_navigation_bit_sign_rotates_galileo_pilot_axis_into_data_axis() {
        let config = crate::engine::receiver_config::ReceiverPipelineConfig {
            code_freq_basis_hz: 10_230_000.0,
            code_length: 10_230,
            ..crate::engine::receiver_config::ReceiverPipelineConfig::default()
        };
        let sat = SatId { constellation: Constellation::Galileo, prn: 11 };
        let signal_model = super::TrackingSignalModel::for_sat_signal_band(
            &config,
            sat,
            SignalBand::E5,
            SignalCode::E5b,
            None,
        );

        assert_eq!(
            super::recover_epoch_navigation_bit_sign(
                &signal_model,
                Some(Complex::new(0.70, 0.02)),
                Complex::new(0.01, 0.80),
                super::CarrierPromptSource::Pilot,
                0.0,
                true,
            ),
            Some(1)
        );
        assert_eq!(
            super::recover_epoch_navigation_bit_sign(
                &signal_model,
                Some(Complex::new(-0.70, -0.02)),
                Complex::new(0.01, 0.80),
                super::CarrierPromptSource::Pilot,
                0.0,
                true,
            ),
            Some(-1)
        );
    }

    #[test]
    fn tracking_signal_model_uses_registry_metadata_for_galileo_e1b() {
        let config = crate::engine::receiver_config::ReceiverPipelineConfig::default();
        let sat = SatId { constellation: Constellation::Galileo, prn: 11 };

        let signal_model = super::TrackingSignalModel::for_sat_signal_band(
            &config,
            sat,
            SignalBand::E1,
            SignalCode::E1B,
            None,
        );

        assert_eq!(signal_model.component_role, SignalComponentRole::Data);
        assert!(signal_model.secondary_code.is_none());
        assert_eq!(
            signal_model.discriminator_family,
            super::TrackingDiscriminatorFamily::CbocEarlyPromptLate
        );
        assert_eq!(
            signal_model.phase_transition_source,
            super::TrackingPhaseTransitionSource::DataSymbol
        );
    }

    #[test]
    fn galileo_e1_subcarrier_guard_accepts_main_lobe_prompt() {
        let config = galileo_e1_subcarrier_guard_config();
        let sat = SatId { constellation: Constellation::Galileo, prn: 11 };
        let code_phase_chips = 321.375;
        let guard = galileo_e1_subcarrier_guard_for_seed(&config, sat, code_phase_chips, 0.0);

        assert!(!guard.detected(), "guard={guard:?}");
        assert!(
            guard.prompt_relative_power >= super::SUBCARRIER_AMBIGUITY_MIN_PROMPT_RELATIVE_POWER,
            "guard={guard:?}",
        );
    }

    #[test]
    fn galileo_e1_subcarrier_guard_rejects_half_chip_side_lobe_prompt() {
        let config = galileo_e1_subcarrier_guard_config();
        let sat = SatId { constellation: Constellation::Galileo, prn: 11 };
        let code_phase_chips = 321.375;
        let guard = galileo_e1_subcarrier_guard_for_seed(&config, sat, code_phase_chips, 0.5);

        assert!(guard.detected(), "guard={guard:?}");
        assert!(
            guard.strongest_alternate_power > guard.prompt_power,
            "guard should find stronger half-chip evidence than the side-lobe prompt: {guard:?}",
        );
    }

    #[test]
    fn tracking_signal_model_uses_registry_metadata_for_beidou_b1i() {
        let config = crate::engine::receiver_config::ReceiverPipelineConfig {
            code_freq_basis_hz: 2_046_000.0,
            code_length: 2046,
            ..crate::engine::receiver_config::ReceiverPipelineConfig::default()
        };
        let sat = SatId { constellation: Constellation::Beidou, prn: 11 };

        let signal_model = super::TrackingSignalModel::for_sat_signal_band(
            &config,
            sat,
            SignalBand::B1,
            SignalCode::B1I,
            None,
        );

        assert_eq!(signal_model.component_role, SignalComponentRole::Data);
        assert!(signal_model.secondary_code.is_some());
        assert_eq!(
            signal_model.discriminator_family,
            super::TrackingDiscriminatorFamily::EarlyPromptLate
        );
        assert_eq!(
            signal_model.phase_transition_source,
            super::TrackingPhaseTransitionSource::SecondaryCode
        );
        assert!((signal_model.nominal_carrier_hz() - 1_561_098_000.0).abs() <= f64::EPSILON);
        assert!(!signal_model.supports_navigation_bit_sign_recovery());
    }

    #[test]
    fn signal_tracking_params_use_narrow_spacing_for_gps_l1_ca_defaults() {
        let config = crate::engine::receiver_config::ReceiverPipelineConfig::default();
        let sat = SatId { constellation: Constellation::Gps, prn: 3 };
        let signal_model = super::TrackingSignalModel::for_sat_signal_band(
            &config,
            sat,
            SignalBand::L1,
            SignalCode::Ca,
            None,
        );

        let tracking_params = super::resolve_signal_tracking_params(&config, &signal_model);

        assert_eq!(tracking_params.early_late_spacing_chips, 0.25);
    }

    #[test]
    fn code_discriminator_mode_uses_double_delta_for_bpsk_tracking() {
        let config = crate::engine::receiver_config::ReceiverPipelineConfig::default();
        let sat = SatId { constellation: Constellation::Gps, prn: 3 };
        let signal_model = super::TrackingSignalModel::for_sat_signal_band(
            &config,
            sat,
            SignalBand::L1,
            SignalCode::Ca,
            None,
        );

        assert_eq!(
            super::code_discriminator_mode(&signal_model),
            super::CodeDiscriminatorMode::DoubleDeltaEarlyPromptLate
        );
    }

    #[test]
    fn code_discriminator_mode_keeps_subcarrier_tracking_unambiguous() {
        let config = crate::engine::receiver_config::ReceiverPipelineConfig::default();
        let sat = SatId { constellation: Constellation::Galileo, prn: 11 };
        let signal_model = super::TrackingSignalModel::for_sat_signal_band(
            &config,
            sat,
            SignalBand::E1,
            SignalCode::E1B,
            None,
        );

        assert_eq!(
            super::code_discriminator_mode(&signal_model),
            super::CodeDiscriminatorMode::EarlyPromptLate
        );
    }

    #[test]
    fn code_discriminator_mode_keeps_secondary_code_tracking_unambiguous() {
        let config = crate::engine::receiver_config::ReceiverPipelineConfig::default();
        let sat = SatId { constellation: Constellation::Beidou, prn: 11 };
        let signal_model = super::TrackingSignalModel::for_sat_signal_band(
            &config,
            sat,
            SignalBand::B1,
            SignalCode::B1I,
            None,
        );

        assert_eq!(
            super::code_discriminator_mode(&signal_model),
            super::CodeDiscriminatorMode::EarlyPromptLate
        );
    }

    #[test]
    fn signal_tracking_params_keep_configured_spacing_for_secondary_code_tracking() {
        let config = crate::engine::receiver_config::ReceiverPipelineConfig::default();
        let sat = SatId { constellation: Constellation::Beidou, prn: 11 };
        let signal_model = super::TrackingSignalModel::for_sat_signal_band(
            &config,
            sat,
            SignalBand::B1,
            SignalCode::B1I,
            None,
        );

        let tracking_params = super::resolve_signal_tracking_params(&config, &signal_model);

        assert_eq!(tracking_params.early_late_spacing_chips, 0.5);
    }

    #[test]
    fn signal_tracking_params_use_tighter_spacing_for_high_rate_signals() {
        let config = crate::engine::receiver_config::ReceiverPipelineConfig::default();
        let sat = SatId { constellation: Constellation::Gps, prn: 18 };
        let signal_model = super::TrackingSignalModel::for_sat_signal_band(
            &config,
            sat,
            SignalBand::L5,
            SignalCode::L5I,
            None,
        );

        let tracking_params = super::resolve_signal_tracking_params(&config, &signal_model);

        assert_eq!(tracking_params.early_late_spacing_chips, 0.10);
    }

    #[test]
    fn signal_tracking_params_preserve_per_band_spacing_overrides() {
        let config = crate::engine::receiver_config::ReceiverPipelineConfig {
            tracking_per_band: vec![BandTrackingSpec {
                band: SignalBand::L1,
                early_late_spacing_chips: 0.5,
                dll_bw_hz: 2.0,
                pll_bw_hz: 15.0,
                fll_bw_hz: 10.0,
                integration_ms: 1,
            }],
            ..crate::engine::receiver_config::ReceiverPipelineConfig::default()
        };
        let sat = SatId { constellation: Constellation::Gps, prn: 3 };
        let signal_model = super::TrackingSignalModel::for_sat_signal_band(
            &config,
            sat,
            SignalBand::L1,
            SignalCode::Ca,
            None,
        );

        let tracking_params = super::resolve_signal_tracking_params(&config, &signal_model);

        assert_eq!(tracking_params.early_late_spacing_chips, 0.5);
    }

    #[test]
    fn tracking_assumptions_follow_signal_metadata() {
        let config = crate::engine::receiver_config::ReceiverPipelineConfig::default();
        let sat = SatId { constellation: Constellation::Galileo, prn: 11 };
        let signal_model = super::TrackingSignalModel::for_sat_signal_band(
            &config,
            sat,
            SignalBand::E1,
            SignalCode::E1B,
            None,
        );
        let tracking_params = super::resolve_signal_tracking_params(&config, &signal_model);

        let assumptions = super::tracking_assumptions(&signal_model, tracking_params);

        assert_eq!(assumptions.discriminator_family, "unambiguous_cboc_early_prompt_late");
        assert_eq!(assumptions.early_late_spacing_chips, 0.25);
    }

    #[derive(Debug, Deserialize)]
    struct TrackingEventFixture {
        lock: bool,
        anti_false_lock: bool,
        cycle_slip: bool,
    }

    #[derive(Debug, Deserialize)]
    struct TrackingScenarioFixture {
        id: String,
        initial_state: String,
        events: Vec<TrackingEventFixture>,
        expected_final_state: String,
    }

    #[test]
    fn tracking_scenario_fixtures_are_deterministic() {
        for (fixture_name, fixture_raw) in tracking_fixture_specs() {
            let fixture = load_tracking_fixture(fixture_raw, fixture_name);
            let mut state = parse_state(&fixture.initial_state);
            let mut unlocked = 0u8;
            let mut degraded_epochs = 0u16;
            for event in &fixture.events {
                let decision = super::deterministic_transition_rule(
                    state,
                    event.lock,
                    event.lock,
                    event.anti_false_lock,
                    event.cycle_slip.then_some(super::LossOfLockCause::PhaseJump),
                    unlocked,
                    degraded_epochs,
                    100,
                    false,
                    None,
                );
                state = decision.to_state;
                unlocked = decision.next_unlocked_count;
                degraded_epochs = decision.next_degraded_epochs;
            }
            assert_eq!(
                state,
                parse_state(&fixture.expected_final_state),
                "fixture {} final state mismatch",
                fixture.id
            );
        }
    }

    fn track_epoch_with_state(
        epoch_index: u32,
        lock: bool,
        lock_state: &str,
        lock_state_reason: Option<&str>,
    ) -> TrackEpoch {
        TrackEpoch {
            epoch: Epoch { index: epoch_index as u64 },
            sample_index: epoch_index as u64,
            sat: SatId { constellation: Constellation::Gps, prn: 1 },
            carrier_hz: Hertz(epoch_index as f64),
            code_phase_samples: Chips(epoch_index as f64 + 0.5),
            lock,
            lock_state: lock_state.to_string(),
            lock_state_reason: lock_state_reason.map(str::to_string),
            ..TrackEpoch::default()
        }
    }

    fn tracking_result_with_epochs(
        channel_id: u8,
        sat: SatId,
        epochs: Vec<TrackEpoch>,
    ) -> super::TrackingResult {
        let epochs = epochs
            .into_iter()
            .map(|mut epoch| {
                epoch.channel_id = Some(channel_id);
                epoch.channel_uid = super::tracking_channel_uid(sat, channel_id);
                epoch.sat = sat;
                epoch
            })
            .collect();
        super::TrackingResult {
            sat,
            carrier_hz: 0.0,
            code_phase_samples: 0.0,
            acquisition_hypothesis: "accepted".to_string(),
            acquisition_score: 1.0,
            acquisition_code_phase_samples: 0,
            acquisition_carrier_hz: 0.0,
            acq_to_track_state: "accepted".to_string(),
            epochs,
            transitions: Vec::new(),
        }
    }

    #[test]
    fn tracking_channel_state_report_emits_unique_steady_states_and_reacquired_marker() {
        let sat = SatId { constellation: Constellation::Gps, prn: 7 };
        let report = super::tracking_channel_state_report(&tracking_result_with_epochs(
            2,
            sat,
            vec![
                track_epoch_with_state(0, false, "pull_in", Some("carrier_pull_in")),
                track_epoch_with_state(1, true, "tracking", Some("carrier_converged")),
                track_epoch_with_state(2, false, "lost", Some("prompt_power_drop")),
                track_epoch_with_state(3, false, "pull_in", Some("carrier_pull_in")),
                track_epoch_with_state(4, true, "tracking", Some("reacquired")),
                track_epoch_with_state(5, true, "tracking", Some("stable_tracking")),
            ],
        ));

        let emitted_states =
            report.emitted_states.iter().map(|event| event.state).collect::<Vec<_>>();
        assert_eq!(
            emitted_states,
            vec![
                super::TrackingChannelState::Acquired,
                super::TrackingChannelState::PullIn,
                super::TrackingChannelState::Locked,
                super::TrackingChannelState::Lost,
                super::TrackingChannelState::PullIn,
                super::TrackingChannelState::Locked,
                super::TrackingChannelState::Reacquired,
            ]
        );
        assert_eq!(report.final_state, super::TrackingChannelState::Locked);
        assert_eq!(report.final_reason.as_deref(), Some("stable_tracking"));
    }

    #[test]
    fn tracking_channel_state_report_marks_cn0_lock_refusal() {
        let sat = SatId { constellation: Constellation::Gps, prn: 16 };
        let report = super::tracking_channel_state_report(&tracking_result_with_epochs(
            1,
            sat,
            vec![
                track_epoch_with_state(0, false, "pull_in", Some("carrier_pull_in")),
                track_epoch_with_state(1, false, "pull_in", Some("cn0_below_tracking_lock_floor")),
                track_epoch_with_state(2, false, "pull_in", Some("cn0_below_tracking_lock_floor")),
            ],
        ));

        let emitted_states =
            report.emitted_states.iter().map(|event| event.state).collect::<Vec<_>>();
        assert_eq!(
            emitted_states,
            vec![
                super::TrackingChannelState::Acquired,
                super::TrackingChannelState::PullIn,
                super::TrackingChannelState::Refused,
            ]
        );
        assert_eq!(report.final_state, super::TrackingChannelState::Refused);
        assert_eq!(report.final_reason.as_deref(), Some("cn0_below_tracking_lock_floor"));
    }

    #[test]
    fn tracking_channel_state_report_keeps_degraded_final_state() {
        let sat = SatId { constellation: Constellation::Gps, prn: 9 };
        let report = super::tracking_channel_state_report(&tracking_result_with_epochs(
            3,
            sat,
            vec![
                track_epoch_with_state(0, true, "tracking", Some("carrier_converged")),
                track_epoch_with_state(1, true, "degraded", Some("signal_fade")),
            ],
        ));

        assert_eq!(
            report.emitted_states.iter().map(|event| event.state).collect::<Vec<_>>(),
            vec![
                super::TrackingChannelState::Acquired,
                super::TrackingChannelState::Locked,
                super::TrackingChannelState::Degraded,
            ]
        );
        assert_eq!(report.final_state, super::TrackingChannelState::Degraded);
        assert_eq!(report.final_reason.as_deref(), Some("signal_fade"));
    }

    #[test]
    fn tracking_channel_state_report_suppresses_duplicate_refused_markers() {
        let sat = SatId { constellation: Constellation::Gps, prn: 16 };
        let report = super::tracking_channel_state_report(&tracking_result_with_epochs(
            1,
            sat,
            vec![
                track_epoch_with_state(0, false, "pull_in", Some("carrier_pull_in")),
                track_epoch_with_state(1, false, "pull_in", Some("cn0_below_tracking_lock_floor")),
                track_epoch_with_state(2, false, "pull_in", Some("cn0_below_tracking_lock_floor")),
                track_epoch_with_state(3, false, "pull_in", Some("cn0_below_tracking_lock_floor")),
            ],
        ));

        let refused_count = report
            .emitted_states
            .iter()
            .filter(|event| event.state == super::TrackingChannelState::Refused)
            .count();
        assert_eq!(refused_count, 1, "{report:?}");
    }

    #[test]
    #[cfg(feature = "alloc-audit")]
    fn tracking_allocations_under_threshold() {
        let config = crate::engine::receiver_config::ReceiverPipelineConfig::default();
        let runtime = crate::engine::runtime::ReceiverRuntime::default();
        let samples_per_code = bijux_gnss_signal::api::samples_per_code(
            config.sampling_freq_hz,
            config.code_freq_basis_hz,
            config.code_length,
        );
        let samples = vec![num_complex::Complex::new(0.0, 0.0); samples_per_code];
        let tracking = Tracking::new(config, runtime);

        let before = crate::engine::alloc::allocation_count();
        let _ = tracking.run(&samples);
        let after = crate::engine::alloc::allocation_count();

        let allocated = after.saturating_sub(before);
        let threshold = 200;
        assert!(
            allocated <= threshold,
            "tracking allocations exceeded threshold: {allocated} > {threshold}"
        );
    }

    fn tracking_fixture_specs() -> Vec<(&'static str, &'static str)> {
        vec![
            (
                "interference_like.json",
                include_str!(concat!(
                    env!("CARGO_MANIFEST_DIR"),
                    "/tests/data/tracking/interference_like.json"
                )),
            ),
            (
                "lock.json",
                include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/tracking/lock.json")),
            ),
            (
                "relock.json",
                include_str!(concat!(
                    env!("CARGO_MANIFEST_DIR"),
                    "/tests/data/tracking/relock.json"
                )),
            ),
            (
                "weak_signal.json",
                include_str!(concat!(
                    env!("CARGO_MANIFEST_DIR"),
                    "/tests/data/tracking/weak_signal.json"
                )),
            ),
        ]
    }

    fn split_frame(frame: &SamplesFrame, chunk_len: usize) -> Vec<SamplesFrame> {
        let mut chunks = Vec::new();
        let mut start = 0usize;
        while start < frame.len() {
            let end = (start + chunk_len.max(1)).min(frame.len());
            chunks.push(SamplesFrame::new(
                SampleTime {
                    sample_index: frame.t0.sample_index + start as u64,
                    sample_rate_hz: frame.t0.sample_rate_hz,
                },
                Seconds(frame.dt_s.0),
                frame.iq[start..end].to_vec(),
            ));
            start = end;
        }
        chunks
    }

    fn load_tracking_fixture(raw: &str, fixture_name: &str) -> TrackingScenarioFixture {
        serde_json::from_str(raw)
            .unwrap_or_else(|err| panic!("parse tracking fixture {fixture_name}: {err}"))
    }

    fn parse_state(value: &str) -> ChannelState {
        match value {
            "Idle" => ChannelState::Idle,
            "Acquired" => ChannelState::Acquired,
            "PullIn" => ChannelState::PullIn,
            "Tracking" => ChannelState::Tracking,
            "Degraded" => ChannelState::Degraded,
            "Lost" => ChannelState::Lost,
            _ => panic!("unsupported state {value}"),
        }
    }
