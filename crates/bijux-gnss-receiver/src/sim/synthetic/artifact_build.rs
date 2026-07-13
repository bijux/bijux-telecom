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
        && pvt.summary.pass;

    SyntheticGnssAccuracyArtifact {
        schema_version: SYNTHETIC_GNSS_ACCURACY_ARTIFACT_SCHEMA_VERSION,
        scenario_id: case.scenario_id.to_string(),
        pass,
        truth_coverage_ready,
        data_source: case.data_source,
        reference_truth: case.reference_truth,
        acquisition,
        tracking,
        observation,
        pvt,
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
    let receiver_epoch_base = shared_receiver_epoch_base(&pseudorange_chips)?;

    Ok(scenario
        .satellites
        .iter()
        .map(|signal| SyntheticSignalDelayAlignment {
            sat: signal.sat,
            signal_delay_alignment: SignalDelayAlignment {
                whole_code_periods: receiver_epoch_base,
                source: "synthetic_truth".to_string(),
            },
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

/// Run end-to-end synthetic navigation validation and build one final GNSS accuracy artifact.
pub fn validate_synthetic_navigation_run(
    config: &ReceiverPipelineConfig,
    scenario: &SyntheticNavigationValidationScenario,
    hatch_window: u32,
) -> Result<SyntheticNavigationValidationRun, SyntheticNavigationValidationError> {
    let signal_scenario = build_signal_scenario_from_navigation_validation_scenario(scenario)?;
    let signal_delay_alignments =
        signal_delay_alignments_from_navigation_validation_scenario(scenario)?;
    let frame = generate_l1_ca_multi(config, &signal_scenario);
    let truth_bundle = build_iq16_capture_bundle(
        &signal_scenario.id,
        &signal_scenario,
        &frame,
        "2026-07-11T00:00:00Z",
        Some("synthetic navigation validation".to_string()),
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
    let mut source = SyntheticSignalSource::new_with_signal_delay_alignments(
        config,
        &signal_scenario,
        signal_delay_alignments,
    );
    let run = receiver.run(&mut source).map_err(|error| {
        SyntheticNavigationValidationError::ReceiverPipeline { message: error.to_string() }
    })?;

    let observation_reference = SyntheticObservationTruthReference {
        receive_time_s: scenario.reference_receive_time_s,
        receiver_ecef_m: scenario.receiver_ecef_m,
        ionosphere_delay_model: None,
    };
    let observation_validation = validate_truth_guided_observations(
        config,
        &run.tracking,
        &signal_scenario,
        &observation_reference,
        hatch_window,
    );
    let observation_accuracy =
        validate_observation_accuracy_budget(&observation_validation, budgets.observation);
    let solutions = run.navigation.clone();
    let pvt_reference = synthetic_navigation_pvt_reference_epochs(scenario, &solutions);
    let pvt_truth =
        validate_truth_guided_pvt_table(&signal_scenario.id, &solutions, &pvt_reference);
    let pvt_accuracy = validate_pvt_accuracy_budget(&pvt_truth, budgets.pvt);

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
    });

    Ok(SyntheticNavigationValidationRun {
        signal_scenario,
        truth_bundle,
        pipeline_artifacts: run,
        acquisition_accuracy,
        tracking_accuracy,
        observation_accuracy,
        pvt_accuracy,
        artifact,
    })
}
