fn closure_stage(
    stage: SyntheticClosureStageKind,
    status: SyntheticClosureStageStatus,
    executed: bool,
    truth_backed: bool,
    evidence_count: usize,
    reason: impl Into<String>,
) -> SyntheticClosureStageEvidence {
    SyntheticClosureStageEvidence {
        stage,
        status,
        executed,
        truth_backed,
        evidence_count,
        reason: reason.into(),
    }
}

fn passed_closure_stage(
    stage: SyntheticClosureStageKind,
    evidence_count: usize,
    reason: impl Into<String>,
) -> SyntheticClosureStageEvidence {
    closure_stage(stage, SyntheticClosureStageStatus::Passed, true, true, evidence_count, reason)
}

fn failed_closure_stage(
    stage: SyntheticClosureStageKind,
    executed: bool,
    truth_backed: bool,
    evidence_count: usize,
    reason: impl Into<String>,
) -> SyntheticClosureStageEvidence {
    closure_stage(
        stage,
        SyntheticClosureStageStatus::Failed,
        executed,
        truth_backed,
        evidence_count,
        reason,
    )
}

fn not_applicable_closure_stage(
    stage: SyntheticClosureStageKind,
    reason: impl Into<String>,
) -> SyntheticClosureStageEvidence {
    closure_stage(stage, SyntheticClosureStageStatus::NotApplicable, false, true, 0, reason)
}

fn closure_summary(stages: Vec<SyntheticClosureStageEvidence>) -> SyntheticClosureSummary {
    let applicable_stage_count = stages
        .iter()
        .filter(|stage| stage.status != SyntheticClosureStageStatus::NotApplicable)
        .count();
    let passed_stage_count =
        stages.iter().filter(|stage| stage.status == SyntheticClosureStageStatus::Passed).count();
    let not_applicable_stage_count = stages
        .iter()
        .filter(|stage| stage.status == SyntheticClosureStageStatus::NotApplicable)
        .count();
    let pass = stages.iter().all(SyntheticClosureStageEvidence::is_closure_success)
        && applicable_stage_count > 0;

    SyntheticClosureSummary {
        pass,
        applicable_stage_count,
        passed_stage_count,
        not_applicable_stage_count,
        stages,
    }
}

fn accepted_observation_satellite_count(run: &crate::api::RunArtifacts) -> usize {
    run.observations
        .iter()
        .flat_map(|epoch| epoch.sats.iter())
        .filter(|satellite| satellite.observation_status == ObservationStatus::Accepted)
        .count()
}

fn timed_observation_satellite_count(run: &crate::api::RunArtifacts) -> usize {
    run.observations
        .iter()
        .flat_map(|epoch| epoch.sats.iter())
        .filter(|satellite| {
            satellite.observation_status == ObservationStatus::Accepted
                && satellite.timing.is_some()
                && !satellite.metadata.pseudorange_time_source.trim().is_empty()
                && !satellite.metadata.time_tag_source.trim().is_empty()
        })
        .count()
}

fn accepted_navigation_solutions(run: &crate::api::RunArtifacts) -> Vec<&NavSolutionEpoch> {
    run.navigation.iter().filter(|solution| solution.status.is_valid()).collect()
}

fn solution_reason_count(run: &crate::api::RunArtifacts, predicate: impl Fn(&str) -> bool) -> usize {
    run.navigation
        .iter()
        .flat_map(|solution| solution.explain_reasons.iter())
        .filter(|reason| predicate(reason.as_str()))
        .count()
}

fn solution_with_integrity_decision_count(run: &crate::api::RunArtifacts) -> usize {
    run.navigation
        .iter()
        .filter(|solution| {
            !solution.explain_decision.trim().is_empty()
                && (solution.integrity_hpl_m.is_some()
                    || solution.integrity_vpl_m.is_some()
                    || matches!(
                        solution.status,
                        SolutionStatus::IntegrityFailed
                            | SolutionStatus::Refused
                            | SolutionStatus::Unavailable
                    ))
        })
        .count()
}

fn build_iq_input_closure_stage(
    run: &crate::api::RunArtifacts,
    truth: &SyntheticIqTruthBundle,
) -> SyntheticClosureStageEvidence {
    let explicit_metadata = truth.sample_rate_hz.is_finite()
        && truth.sample_rate_hz > 0.0
        && truth.intermediate_freq_hz.is_finite()
        && truth.sample_count > 0
        && truth.duration_s.is_finite()
        && truth.duration_s > 0.0;
    let executed = run.processed_input_samples > 0;
    if executed && explicit_metadata {
        passed_closure_stage(
            SyntheticClosureStageKind::IqInput,
            run.processed_input_samples as usize,
            "explicit_iq_metadata_consumed",
        )
    } else {
        failed_closure_stage(
            SyntheticClosureStageKind::IqInput,
            executed,
            explicit_metadata,
            run.processed_input_samples as usize,
            "missing_explicit_iq_input_evidence",
        )
    }
}

fn build_navigation_timing_closure_stage(
    run: &crate::api::RunArtifacts,
) -> SyntheticClosureStageEvidence {
    let accepted_satellite_count = accepted_observation_satellite_count(run);
    let timed_satellite_count = timed_observation_satellite_count(run);
    let epochs_with_gps_time =
        run.observations.iter().filter(|epoch| epoch.gps_time().is_some()).count();
    if accepted_satellite_count > 0
        && timed_satellite_count == accepted_satellite_count
        && epochs_with_gps_time == run.observations.len()
    {
        passed_closure_stage(
            SyntheticClosureStageKind::NavigationDataTiming,
            timed_satellite_count,
            "observation_timing_carries_transmit_and_receive_time",
        )
    } else {
        failed_closure_stage(
            SyntheticClosureStageKind::NavigationDataTiming,
            !run.observations.is_empty(),
            timed_satellite_count > 0,
            timed_satellite_count,
            "missing_observation_navigation_timing_evidence",
        )
    }
}

fn build_satellite_state_closure_stage(
    run: &crate::api::RunArtifacts,
    scenario: &SyntheticNavigationValidationScenario,
) -> SyntheticClosureStageEvidence {
    let accepted_solutions = accepted_navigation_solutions(run);
    let residual_count = accepted_solutions.iter().map(|solution| solution.residuals.len()).sum();
    let ephemeris_count = scenario.ephemerides.len();
    let has_satellite_state_support = !accepted_solutions.is_empty()
        && ephemeris_count >= scenario.satellites.len()
        && accepted_solutions
            .iter()
            .all(|solution| solution.assumptions.as_ref().is_some_and(|assumptions| {
                assumptions.ephemeris_count >= scenario.satellites.len()
                    && !assumptions.ephemeris_source.trim().is_empty()
            }));

    if has_satellite_state_support {
        passed_closure_stage(
            SyntheticClosureStageKind::SatelliteStates,
            residual_count,
            "broadcast_ephemeris_satellite_states_used_by_solution",
        )
    } else {
        failed_closure_stage(
            SyntheticClosureStageKind::SatelliteStates,
            !accepted_solutions.is_empty(),
            ephemeris_count >= scenario.satellites.len(),
            residual_count,
            "missing_satellite_state_evidence",
        )
    }
}

fn build_correction_closure_stage(run: &crate::api::RunArtifacts) -> SyntheticClosureStageEvidence {
    let ionosphere_count = solution_reason_count(run, |reason| {
        reason.starts_with("ionosphere_correction=") && reason != "ionosphere_uncorrected"
    });
    let troposphere_count =
        solution_reason_count(run, |reason| reason == "troposphere_correction=saastamoinen");
    let evidence_count = ionosphere_count.min(troposphere_count);

    if evidence_count > 0 {
        passed_closure_stage(
            SyntheticClosureStageKind::Corrections,
            evidence_count,
            "ionosphere_and_troposphere_corrections_recorded",
        )
    } else {
        failed_closure_stage(
            SyntheticClosureStageKind::Corrections,
            !run.navigation.is_empty(),
            false,
            evidence_count,
            "missing_navigation_correction_evidence",
        )
    }
}

fn build_estimator_closure_stage(
    run: &crate::api::RunArtifacts,
    pvt: &SyntheticPvtAccuracyReport,
) -> SyntheticClosureStageEvidence {
    let accepted_solution_count = accepted_navigation_solutions(run).len();
    if accepted_solution_count > 0 && pvt.pass {
        passed_closure_stage(
            SyntheticClosureStageKind::Estimator,
            accepted_solution_count,
            "spp_estimator_truth_checked",
        )
    } else {
        failed_closure_stage(
            SyntheticClosureStageKind::Estimator,
            !run.navigation.is_empty(),
            pvt.truth_coverage_ready,
            accepted_solution_count,
            "missing_truth_checked_estimator_solution",
        )
    }
}

fn build_ambiguity_closure_stage(run: &crate::api::RunArtifacts) -> SyntheticClosureStageEvidence {
    let fixed_solution_count =
        run.navigation.iter().filter(|solution| solution.status == SolutionStatus::Fixed).count();
    let float_solution_count =
        run.navigation.iter().filter(|solution| solution.status == SolutionStatus::Float).count();
    let code_only_solution_count = run
        .navigation
        .iter()
        .filter(|solution| solution.status == SolutionStatus::CodeOnly)
        .count();

    if fixed_solution_count > 0 || float_solution_count > 0 {
        failed_closure_stage(
            SyntheticClosureStageKind::AmbiguityProcessing,
            true,
            false,
            fixed_solution_count + float_solution_count,
            "ambiguity_solution_without_receiver_path_closure_evidence",
        )
    } else if code_only_solution_count > 0 {
        not_applicable_closure_stage(
            SyntheticClosureStageKind::AmbiguityProcessing,
            "code_only_spp_path_has_no_carrier_ambiguity_processing",
        )
    } else {
        failed_closure_stage(
            SyntheticClosureStageKind::AmbiguityProcessing,
            false,
            false,
            0,
            "missing_estimator_status_for_ambiguity_applicability",
        )
    }
}

fn build_integrity_closure_stage(run: &crate::api::RunArtifacts) -> SyntheticClosureStageEvidence {
    let evidence_count = solution_with_integrity_decision_count(run);
    if evidence_count > 0 {
        passed_closure_stage(
            SyntheticClosureStageKind::IntegrityDecision,
            evidence_count,
            "navigation_integrity_decision_recorded",
        )
    } else {
        failed_closure_stage(
            SyntheticClosureStageKind::IntegrityDecision,
            !run.navigation.is_empty(),
            false,
            evidence_count,
            "missing_navigation_integrity_decision",
        )
    }
}

/// Build a machine-checkable closure summary for the configured synthetic receiver path.
pub fn build_synthetic_navigation_closure_summary(
    run: &crate::api::RunArtifacts,
    truth: &SyntheticIqTruthBundle,
    scenario: &SyntheticNavigationValidationScenario,
    acquisition: &SyntheticAcquisitionAccuracyReport,
    tracking: &SyntheticTrackingAccuracyReport,
    observation: &SyntheticObservationAccuracyReport,
    pvt: &SyntheticPvtAccuracyReport,
) -> SyntheticClosureSummary {
    closure_summary(vec![
        build_iq_input_closure_stage(run, truth),
        if !run.acquisitions.is_empty() && acquisition.pass {
            passed_closure_stage(
                SyntheticClosureStageKind::Acquisition,
                run.acquisitions.len(),
                "truth_checked_acquisition_results",
            )
        } else {
            failed_closure_stage(
                SyntheticClosureStageKind::Acquisition,
                !run.acquisitions.is_empty(),
                acquisition.truth_coverage_ready,
                run.acquisitions.len(),
                "missing_truth_checked_acquisition_results",
            )
        },
        if !run.tracking.is_empty() && tracking.pass {
            passed_closure_stage(
                SyntheticClosureStageKind::Tracking,
                run.tracking.len(),
                "truth_checked_tracking_results",
            )
        } else {
            failed_closure_stage(
                SyntheticClosureStageKind::Tracking,
                !run.tracking.is_empty(),
                tracking.truth_coverage_ready,
                run.tracking.len(),
                "missing_truth_checked_tracking_results",
            )
        },
        build_navigation_timing_closure_stage(run),
        if !run.observations.is_empty() && observation.pass {
            passed_closure_stage(
                SyntheticClosureStageKind::Observations,
                accepted_observation_satellite_count(run),
                "truth_checked_observation_epochs",
            )
        } else {
            failed_closure_stage(
                SyntheticClosureStageKind::Observations,
                !run.observations.is_empty(),
                observation.truth_coverage_ready,
                accepted_observation_satellite_count(run),
                "missing_truth_checked_observation_epochs",
            )
        },
        build_satellite_state_closure_stage(run, scenario),
        build_correction_closure_stage(run),
        build_estimator_closure_stage(run, pvt),
        build_ambiguity_closure_stage(run),
        build_integrity_closure_stage(run),
    ])
}
