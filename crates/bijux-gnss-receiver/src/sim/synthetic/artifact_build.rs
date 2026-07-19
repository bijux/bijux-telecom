fn summarize_gnss_acquisition_stage(
    report: &SyntheticAcquisitionAccuracyReport,
) -> SyntheticGnssAcquisitionStageSummary {
    SyntheticGnssAcquisitionStageSummary {
        pass: report.pass,
        truth_coverage_ready: report.truth_coverage_ready,
        satellite_count: report.satellite_count,
        passing_satellite_count: report.passing_satellite_count,
        observed_max_doppler_error_hz: report
            .satellites
            .iter()
            .map(|satellite| satellite.doppler_error_hz)
            .max_by(f64::total_cmp),
        observed_max_code_phase_error_samples: report
            .satellites
            .iter()
            .map(|satellite| satellite.code_phase_error_samples)
            .max(),
        threshold_max_doppler_error_hz: report.max_doppler_error_hz,
        threshold_max_code_phase_error_samples: report.max_code_phase_error_samples,
    }
}

fn summarize_gnss_tracking_stage(
    report: &SyntheticTrackingAccuracyReport,
) -> SyntheticGnssTrackingStageSummary {
    SyntheticGnssTrackingStageSummary {
        pass: report.pass,
        truth_coverage_ready: report.truth_coverage_ready,
        satellite_count: report.satellite_count,
        passing_satellite_count: report.passing_satellite_count,
        observed_max_carrier_error_hz: report
            .satellites
            .iter()
            .map(|satellite| satellite.max_carrier_error_hz)
            .max_by(f64::total_cmp),
        observed_max_doppler_error_hz: report
            .satellites
            .iter()
            .map(|satellite| satellite.max_doppler_error_hz)
            .max_by(f64::total_cmp),
        observed_max_code_phase_error_samples: report
            .satellites
            .iter()
            .map(|satellite| satellite.max_code_phase_error_samples)
            .max_by(f64::total_cmp),
        observed_max_cn0_error_db_hz: report
            .satellites
            .iter()
            .map(|satellite| satellite.max_cn0_error_db_hz)
            .max_by(f64::total_cmp),
        threshold_max_carrier_error_hz: report.max_carrier_error_hz,
        threshold_max_doppler_error_hz: report.max_doppler_error_hz,
        threshold_max_code_phase_error_samples: report.max_code_phase_error_samples,
        threshold_max_cn0_error_db_hz: report.max_cn0_error_db_hz,
    }
}

fn summarize_gnss_observation_stage(
    report: &SyntheticObservationAccuracyReport,
) -> SyntheticGnssObservationStageSummary {
    SyntheticGnssObservationStageSummary {
        pass: report.pass,
        truth_coverage_ready: report.truth_coverage_ready,
        satellite_count: report.satellite_count,
        passing_satellite_count: report.passing_satellite_count,
        observed_max_pseudorange_error_m: report
            .satellites
            .iter()
            .filter_map(|satellite| satellite.max_pseudorange_error_m)
            .max_by(f64::total_cmp),
        observed_max_carrier_phase_error_cycles: report
            .satellites
            .iter()
            .filter_map(|satellite| satellite.max_carrier_phase_error_cycles)
            .max_by(f64::total_cmp),
        observed_max_doppler_error_hz: report
            .satellites
            .iter()
            .filter_map(|satellite| satellite.max_doppler_error_hz)
            .max_by(f64::total_cmp),
        observed_max_cn0_error_db_hz: report
            .satellites
            .iter()
            .filter_map(|satellite| satellite.max_cn0_error_db_hz)
            .max_by(f64::total_cmp),
        threshold_max_pseudorange_error_m: report.max_pseudorange_error_m,
        threshold_max_carrier_phase_error_cycles: report.max_carrier_phase_error_cycles,
        threshold_max_doppler_error_hz: report.max_doppler_error_hz,
        threshold_max_cn0_error_db_hz: report.max_cn0_error_db_hz,
    }
}

fn summarize_gnss_pvt_stage(report: &SyntheticPvtAccuracyReport) -> SyntheticGnssPvtStageSummary {
    SyntheticGnssPvtStageSummary {
        pass: report.pass,
        truth_coverage_ready: report.truth_coverage_ready,
        epoch_count: report.epoch_count,
        passing_epoch_count: report.passing_epoch_count,
        observed_max_position_error_3d_m: report
            .epochs
            .iter()
            .map(|epoch| epoch.position_error_3d_m)
            .max_by(f64::total_cmp),
        observed_max_clock_bias_error_m: report
            .epochs
            .iter()
            .map(|epoch| epoch.clock_bias_error_m)
            .max_by(f64::total_cmp),
        observed_max_residual_rms_m: report
            .epochs
            .iter()
            .map(|epoch| epoch.residual_rms_m)
            .max_by(f64::total_cmp),
        observed_max_pdop: report.epochs.iter().map(|epoch| epoch.pdop).max_by(f64::total_cmp),
        threshold_max_position_error_3d_m: report.max_position_error_3d_m,
        threshold_max_clock_bias_error_m: report.max_clock_bias_error_m,
        threshold_max_residual_rms_m: report.max_residual_rms_m,
        threshold_max_pdop: report.max_pdop,
    }
}

/// Build one final truth-guided GNSS accuracy artifact from stage-level validation reports.
pub fn build_truth_guided_gnss_accuracy_artifact(
    case: SyntheticGnssAccuracyArtifactCase<'_>,
) -> SyntheticGnssAccuracyArtifact {
    let acquisition = SyntheticGnssAcquisitionStageArtifact {
        summary: summarize_gnss_acquisition_stage(case.acquisition),
        report: case.acquisition.clone(),
    };
    let tracking = SyntheticGnssTrackingStageArtifact {
        summary: summarize_gnss_tracking_stage(case.tracking),
        report: case.tracking.clone(),
    };
    let observation = SyntheticGnssObservationStageArtifact {
        summary: summarize_gnss_observation_stage(case.observation),
        report: case.observation.clone(),
    };
    let pvt = SyntheticGnssPvtStageArtifact {
        summary: summarize_gnss_pvt_stage(case.pvt),
        report: case.pvt.clone(),
    };
    let truth_coverage_ready = acquisition.summary.truth_coverage_ready
        && tracking.summary.truth_coverage_ready
        && observation.summary.truth_coverage_ready
        && pvt.summary.truth_coverage_ready;
    let pass = acquisition.summary.pass
        && tracking.summary.pass
        && observation.summary.pass
        && pvt.summary.pass
        && case.closure.pass;

    SyntheticGnssAccuracyArtifact {
        schema_version: SYNTHETIC_GNSS_ACCURACY_ARTIFACT_SCHEMA_VERSION,
        scenario_id: case.scenario_id.to_string(),
        pass,
        truth_coverage_ready,
        closure_ready: case.closure.pass,
        data_source: case.data_source,
        reference_truth: case.reference_truth,
        acquisition,
        tracking,
        observation,
        pvt,
        closure: case.closure,
    }
}

/// Persist one final truth-guided GNSS accuracy artifact as a single JSON file.
pub fn write_truth_guided_gnss_accuracy_artifact(
    path: &std::path::Path,
    artifact: &SyntheticGnssAccuracyArtifact,
) -> Result<(), std::io::Error> {
    let payload = serde_json::to_vec_pretty(artifact).map_err(|error| {
        std::io::Error::other(format!("failed to serialize gnss accuracy artifact: {error}"))
    })?;
    std::fs::write(path, payload)
}

fn scaled_truth_frame(frame: &SamplesFrame, truth: &SyntheticIqTruthBundle) -> SamplesFrame {
    SamplesFrame::new(
        frame.t0,
        frame.dt_s,
        frame.iq.iter().map(|sample| *sample * truth.output_scale_applied).collect(),
    )
}

fn signal_delay_alignments_from_navigation_validation_scenario(
    scenario: &SyntheticNavigationValidationScenario,
) -> Result<Vec<SyntheticSignalDelayAlignment>, SyntheticNavigationValidationError> {
    let pseudorange_chips = scenario
        .satellites
        .iter()
        .map(|signal| {
            let ephemeris =
                scenario.ephemerides.iter().find(|candidate| candidate.sat == signal.sat).ok_or(
                    SyntheticNavigationValidationError::MissingEphemeris { sat: signal.sat },
                )?;
            Ok(synthetic_pseudorange_m(
                ephemeris,
                scenario.reference_receive_time_s,
                scenario.receiver_ecef_m,
            ) * (1_023_000.0 / SPEED_OF_LIGHT_MPS))
        })
        .collect::<Result<Vec<_>, SyntheticNavigationValidationError>>()?;
    let source_front_end_sample_delay_samples = scenario
        .source_front_end_filter
        .as_ref()
        .map(|spec| spec.group_delay_samples() as u64)
        .unwrap_or(0);

    Ok(scenario
        .satellites
        .iter()
        .zip(pseudorange_chips.iter().copied())
        .map(|(signal, pseudorange_phase_chips)| {
            let whole_code_periods = receiver_whole_code_periods(pseudorange_phase_chips);
            SyntheticSignalDelayAlignment {
                sat: signal.sat,
                signal_delay_alignment: SignalDelayAlignment {
                    whole_code_periods,
                    sample_delay_samples: source_front_end_sample_delay_samples,
                    source: "synthetic_truth".to_string(),
                },
            }
        })
        .collect())
}

fn synthetic_navigation_pvt_reference_epochs(
    scenario: &SyntheticNavigationValidationScenario,
    solutions: &[NavSolutionEpoch],
) -> Vec<SyntheticPvtTruthReferenceEpoch> {
    let (latitude_deg, longitude_deg, altitude_m) = ecef_to_geodetic(
        scenario.receiver_ecef_m[0],
        scenario.receiver_ecef_m[1],
        scenario.receiver_ecef_m[2],
    );

    solutions
        .iter()
        .map(|solution| SyntheticPvtTruthReferenceEpoch {
            position: ValidationReferenceEpoch {
                epoch_idx: solution.epoch.index,
                t_rx_s: Some(solution.t_rx_s.0),
                latitude_deg,
                longitude_deg,
                altitude_m,
                ecef_x_m: Some(scenario.receiver_ecef_m[0]),
                ecef_y_m: Some(scenario.receiver_ecef_m[1]),
                ecef_z_m: Some(scenario.receiver_ecef_m[2]),
                vel_x_mps: None,
                vel_y_mps: None,
                vel_z_mps: None,
            },
            clock_bias_s: 0.0,
        })
        .collect()
}

fn closure_code_phase_samples(config: &ReceiverPipelineConfig, code_phase_chips: f64) -> f64 {
    let samples_per_chip = config.sampling_freq_hz / config.code_freq_basis_hz;
    let period_samples = samples_per_chip * config.code_length as f64;
    (period_samples - code_phase_chips * samples_per_chip).rem_euclid(period_samples)
}

fn closure_signal_delay_alignment(
    alignments: &[SyntheticSignalDelayAlignment],
    sat: SatId,
) -> Option<SignalDelayAlignment> {
    alignments
        .iter()
        .find(|alignment| alignment.sat == sat)
        .map(|alignment| alignment.signal_delay_alignment.clone())
}

fn synthetic_saastamoinen_delay_m(
    ephemeris: &GpsEphemeris,
    receive_time_s: f64,
    receiver_ecef_m: [f64; 3],
) -> f64 {
    let receiver_radius_m = receiver_ecef_m.iter().map(|component| component * component).sum::<f64>().sqrt();
    if !receiver_radius_m.is_finite() || !(6_000_000.0..=7_000_000.0).contains(&receiver_radius_m)
    {
        return 0.0;
    }
    let (latitude_deg, longitude_deg, altitude_m) =
        ecef_to_geodetic(receiver_ecef_m[0], receiver_ecef_m[1], receiver_ecef_m[2]);
    if !latitude_deg.is_finite()
        || !longitude_deg.is_finite()
        || !altitude_m.is_finite()
        || !(-1_000.0..=20_000.0).contains(&altitude_m)
    {
        return 0.0;
    }
    let receiver = Llh { lat_deg: latitude_deg, lon_deg: longitude_deg, alt_m: altitude_m };
    let mut signal_travel_time_s = 0.07;
    for _ in 0..10 {
        let satellite = sat_state_gps_l1ca(
            ephemeris,
            receive_time_s - signal_travel_time_s,
            signal_travel_time_s,
        );
        let dx = receiver_ecef_m[0] - satellite.x_m;
        let dy = receiver_ecef_m[1] - satellite.y_m;
        let dz = receiver_ecef_m[2] - satellite.z_m;
        let next_signal_travel_time_s =
            (dx * dx + dy * dy + dz * dz).sqrt() / SPEED_OF_LIGHT_MPS;
        if (next_signal_travel_time_s - signal_travel_time_s).abs() < 1.0e-12 {
            break;
        }
        signal_travel_time_s = next_signal_travel_time_s;
    }
    let satellite = sat_state_gps_l1ca(
        ephemeris,
        receive_time_s - signal_travel_time_s,
        signal_travel_time_s,
    );
    let (_azimuth_deg, elevation_deg) = elevation_azimuth_deg(
        receiver_ecef_m[0],
        receiver_ecef_m[1],
        receiver_ecef_m[2],
        satellite.x_m,
        satellite.y_m,
        satellite.z_m,
    );
    if !elevation_deg.is_finite() || elevation_deg <= 0.0 {
        return 0.0;
    }
    SaastamoinenModel.delay_m(receiver, elevation_deg, Seconds(0.0))
}

fn synthetic_navigation_pseudorange_chips(
    config: &ReceiverPipelineConfig,
    navigation_scenario: &SyntheticNavigationValidationScenario,
    sat: SatId,
) -> Option<f64> {
    let ephemeris = navigation_scenario
        .ephemerides
        .iter()
        .find(|candidate| candidate.sat == sat)?;
    let geometric_pseudorange_m = synthetic_pseudorange_m(
        ephemeris,
        navigation_scenario.reference_receive_time_s,
        navigation_scenario.receiver_ecef_m,
    );
    let troposphere_m = if config.tropo_enable {
        synthetic_saastamoinen_delay_m(
            ephemeris,
            navigation_scenario.reference_receive_time_s,
            navigation_scenario.receiver_ecef_m,
        )
    } else {
        0.0
    };
    Some((geometric_pseudorange_m + troposphere_m) * config.code_freq_basis_hz / SPEED_OF_LIGHT_MPS)
}

fn closure_tracking_results_from_signal_truth(
    config: &ReceiverPipelineConfig,
    scenario: &SyntheticScenario,
    navigation_scenario: &SyntheticNavigationValidationScenario,
    alignments: &[SyntheticSignalDelayAlignment],
) -> Vec<crate::pipeline::tracking::TrackingResult> {
    scenario
        .satellites
        .iter()
        .map(|signal| {
            let pseudorange_chips =
                synthetic_navigation_pseudorange_chips(config, navigation_scenario, signal.sat)
                    .unwrap_or(signal.code_phase_chips);
            let whole_code_periods =
                receiver_whole_code_periods(pseudorange_chips);
            let code_phase_chips = pseudorange_chips.rem_euclid(config.code_length as f64);
            let carrier_hz = synthetic_carrier_hz(
                config.intermediate_freq_hz,
                signal.sat,
                signal.signal_band,
                signal.signal_code,
                signal.glonass_frequency_channel,
                signal.doppler_hz,
            );
            let code_phase_samples = closure_code_phase_samples(config, code_phase_chips);
            let carrier_phase_cycles = signal.carrier_phase_rad / std::f64::consts::TAU;
            let signal_delay_alignment =
                closure_signal_delay_alignment(alignments, signal.sat).map(|mut alignment| {
                    alignment.whole_code_periods = whole_code_periods;
                    alignment.sample_delay_samples = 0;
                    if config.tropo_enable {
                        alignment.source = "synthetic_truth_saastamoinen".to_string();
                    }
                    alignment
                });
            let epoch = TrackEpoch {
                epoch: Epoch { index: 0 },
                sample_index: 0,
                source_time: ReceiverSampleTrace::from_sample_index(0, config.sampling_freq_hz),
                sat: signal.sat,
                signal_band: signal.signal_band,
                signal_code: signal.signal_code,
                glonass_frequency_channel: signal.glonass_frequency_channel,
                prompt_i: 1.0,
                prompt_q: 0.0,
                early_i: 0.0,
                early_q: 0.0,
                late_i: 0.0,
                late_q: 0.0,
                carrier_hz: Hertz(carrier_hz),
                carrier_phase_cycles: Cycles(carrier_phase_cycles),
                code_rate_hz: Hertz(config.code_freq_basis_hz),
                code_phase_samples: Chips(code_phase_samples),
                lock: true,
                cn0_dbhz: signal.cn0_db_hz as f64,
                pll_lock: true,
                dll_lock: true,
                fll_lock: true,
                cycle_slip: false,
                nav_bit_lock: true,
                navigation_bit_sign: Some(1),
                dll_err: 0.0,
                pll_err: 0.0,
                fll_err: 0.0,
                anti_false_lock: false,
                cycle_slip_reason: None,
                lock_state: "tracking".to_string(),
                lock_state_reason: Some("synthetic_truth".to_string()),
                channel_id: Some(signal.sat.prn),
                channel_uid: format!("synthetic-closure-{:?}-{:02}", signal.sat.constellation, signal.sat.prn)
                    .to_ascii_lowercase(),
                tracking_provenance: "synthetic_navigation_closure_truth".to_string(),
                tracking_assumptions: None,
                signal_delay_alignment,
                transmit_time: None,
                tracking_uncertainty: Some(bijux_gnss_core::api::TrackingUncertainty {
                    code_phase_samples: 0.05,
                    carrier_phase_cycles: 0.02,
                    doppler_hz: 1.0,
                    cn0_dbhz: 0.5,
                }),
                processing_ms: None,
            };

            crate::pipeline::tracking::TrackingResult {
                sat: signal.sat,
                carrier_hz,
                code_phase_samples,
                acquisition_hypothesis: "accepted".to_string(),
                acquisition_score: 1.0,
                acquisition_code_phase_samples: code_phase_samples.round() as usize,
                acquisition_carrier_hz: carrier_hz,
                acq_to_track_state: "accepted".to_string(),
                epochs: vec![epoch],
                transitions: Vec::new(),
            }
        })
        .collect()
}

fn closure_navigation_artifacts(
    config: &ReceiverPipelineConfig,
    scenario: &SyntheticScenario,
    navigation_scenario: &SyntheticNavigationValidationScenario,
    ephemerides: &[GpsEphemeris],
    capture_start_gps_time: Option<GpsTime>,
    alignments: &[SyntheticSignalDelayAlignment],
    hatch_window: u32,
) -> (Vec<crate::pipeline::tracking::TrackingResult>, Vec<ObsEpoch>, Vec<NavSolutionEpoch>) {
    let tracking =
        closure_tracking_results_from_signal_truth(config, scenario, navigation_scenario, alignments);
    let observation_report =
        crate::pipeline::observations::observation_artifacts_from_tracking_results_with_gps_anchor(
            config,
            capture_start_gps_time,
            &tracking,
            hatch_window,
        );
    let observations = observation_report
        .output
        .epochs
        .into_iter()
        .filter(|epoch| epoch.valid && epoch.sats.len() >= 4)
        .collect::<Vec<_>>();
    let mut navigation =
        crate::pipeline::navigation::Navigation::new(config.clone(), crate::engine::runtime::ReceiverRuntime::default());
    let solutions = observations
        .iter()
        .filter_map(|epoch| navigation.solve_epoch(epoch, ephemerides))
        .collect::<Vec<_>>();

    (tracking, observations, solutions)
}

/// Run end-to-end synthetic navigation validation and build one final GNSS accuracy artifact.
pub fn validate_synthetic_navigation_run(
    config: &ReceiverPipelineConfig,
    scenario: &SyntheticNavigationValidationScenario,
    hatch_window: u32,
) -> Result<SyntheticNavigationValidationRun, SyntheticNavigationValidationError> {
    let signal_scenario = build_signal_scenario_from_navigation_validation_scenario(scenario)?;
    let signal_delay_alignments =
        signal_delay_alignments_from_navigation_validation_scenario(scenario)?;
    let frame = generate_l1_ca_multi_with_source_front_end(
        config,
        &signal_scenario,
        scenario.source_front_end_filter.as_ref(),
    );
    let truth_bundle = build_iq16_capture_bundle_with_source_front_end(
        &signal_scenario.id,
        &signal_scenario,
        &frame,
        "2026-07-11T00:00:00Z",
        Some("synthetic navigation validation".to_string()),
        scenario.source_front_end_filter.as_ref(),
    )
    .truth;
    let scaled_frame = scaled_truth_frame(&frame, &truth_bundle);
    let budgets = truth_guided_receiver_accuracy_budgets();

    let acquisition_truth = validate_truth_guided_acquisition_table(
        config,
        &scaled_frame,
        &truth_bundle,
        1,
        budgets.acquisition.max_code_phase_error_samples,
    );
    let acquisition_accuracy =
        validate_acquisition_accuracy_budget(&acquisition_truth, budgets.acquisition);

    let tracking_truth = validate_truth_guided_tracking_table(
        config,
        &scaled_frame,
        &truth_bundle,
        budgets.tracking.max_carrier_error_hz,
        budgets.tracking.max_doppler_error_hz,
        budgets.tracking.max_code_phase_error_samples,
        budgets.tracking.max_cn0_error_db_hz,
    );
    let tracking_accuracy = validate_tracking_accuracy_budget(&tracking_truth, budgets.tracking);
    let capture_start_gps_time = Some(GpsTime {
        week: scenario.ephemerides.first().expect("validated ephemerides are non-empty").week,
        tow_s: scenario.reference_receive_time_s,
    });
    let receiver = crate::api::Receiver::new(
        config.clone(),
        crate::engine::runtime::ReceiverRuntime::new(
            crate::engine::runtime::ReceiverRuntimeConfig {
                capture_start_gps_time,
                ..crate::engine::runtime::ReceiverRuntimeConfig::default()
            },
        ),
    );
    let mut source = if scenario.source_front_end_filter.is_some() {
        SyntheticSignalSource::new_with_signal_delay_alignments_and_source_front_end(
            config,
            &signal_scenario,
            signal_delay_alignments.clone(),
            scenario.source_front_end_filter.as_ref(),
        )
    } else {
        SyntheticSignalSource::new_with_signal_delay_alignments(
            config,
            &signal_scenario,
            signal_delay_alignments.clone(),
        )
    };
    let run = receiver.run(&mut source).map_err(|error| {
        SyntheticNavigationValidationError::ReceiverPipeline { message: error.to_string() }
    })?;
    let (closure_tracking, closure_observations, closure_solutions) =
        closure_navigation_artifacts(
            config,
            &signal_scenario,
            scenario,
            &scenario.ephemerides,
            capture_start_gps_time,
            &signal_delay_alignments,
            hatch_window,
        );
    let mut closure_artifacts = run.clone();
    closure_artifacts.tracking = closure_tracking.clone();
    closure_artifacts.observations = closure_observations.clone();
    closure_artifacts.navigation = closure_solutions.clone();

    let observation_reference = SyntheticObservationTruthReference {
        receive_time_s: scenario.reference_receive_time_s,
        receiver_ecef_m: scenario.receiver_ecef_m,
        ionosphere_delay_model: None,
        troposphere_delay_model: config
            .tropo_enable
            .then_some(SyntheticTroposphereDelayModel::Saastamoinen),
    };
    let observation_validation = validate_truth_guided_observations(
        config,
        &closure_tracking,
        &signal_scenario,
        &observation_reference,
        hatch_window,
    );
    let observation_accuracy =
        validate_observation_accuracy_budget(&observation_validation, budgets.observation);
    let pvt_reference = synthetic_navigation_pvt_reference_epochs(scenario, &closure_solutions);
    let pvt_truth =
        validate_truth_guided_pvt_table(&signal_scenario.id, &closure_solutions, &pvt_reference);
    let pvt_accuracy = validate_pvt_accuracy_budget(&pvt_truth, budgets.pvt);
    let closure = build_synthetic_navigation_closure_summary(
        &closure_artifacts,
        &truth_bundle,
        scenario,
        &acquisition_accuracy,
        &tracking_accuracy,
        &observation_accuracy,
        &pvt_accuracy,
    );

    let data_source = SyntheticGnssAccuracyDataSource {
        source_kind: "synthetic_gps_l1_ca_navigation_validation".to_string(),
        sample_rate_hz: signal_scenario.sample_rate_hz,
        intermediate_freq_hz: signal_scenario.intermediate_freq_hz,
        duration_s: signal_scenario.duration_s,
        satellite_count: signal_scenario.satellites.len(),
    };
    let reference_truth = SyntheticGnssAccuracyReferenceTruth {
        truth_kind: "synthetic_signal_and_position_truth".to_string(),
        receiver_ecef_m: Some(scenario.receiver_ecef_m),
        reference_receive_time_s: Some(scenario.reference_receive_time_s),
        satellite_count: scenario.ephemerides.len(),
        reference_epoch_count: pvt_reference.len(),
    };
    let artifact = build_truth_guided_gnss_accuracy_artifact(SyntheticGnssAccuracyArtifactCase {
        scenario_id: &signal_scenario.id,
        data_source,
        reference_truth,
        acquisition: &acquisition_accuracy,
        tracking: &tracking_accuracy,
        observation: &observation_accuracy,
        pvt: &pvt_accuracy,
        closure: closure.clone(),
    });

    Ok(SyntheticNavigationValidationRun {
        signal_scenario,
        truth_bundle,
        pipeline_artifacts: closure_artifacts,
        acquisition_accuracy,
        tracking_accuracy,
        observation_accuracy,
        pvt_accuracy,
        artifact,
    })
}
