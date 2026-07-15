fn refused_execution_status(
    decision: &bijux_gnss_infra::api::receiver::AdvancedPrereqDecision,
) -> (bijux_gnss_infra::api::receiver::ExecutionStatus, Option<String>) {
    let status = if matches!(
        decision.refusal_class,
        Some(bijux_gnss_infra::api::receiver::AdvancedRefusalClass::UnsupportedModel)
    ) {
        bijux_gnss_infra::api::receiver::ExecutionStatus::Unsupported
    } else {
        bijux_gnss_infra::api::receiver::ExecutionStatus::NotReady
    };
    let reason = (!decision.reasons.is_empty()).then(|| decision.reasons.join(","));
    (status, reason)
}

fn rtk_advanced_solution_measurements(
    quality: Option<&bijux_gnss_infra::api::receiver::RtkBaselineQuality>,
) -> bijux_gnss_infra::api::receiver::AdvancedSolutionMeasurements {
    if let Some(quality) = quality {
        let sigma_h_m = (quality.sigma_e * quality.sigma_e + quality.sigma_n * quality.sigma_n)
            .sqrt();
        bijux_gnss_infra::api::receiver::AdvancedSolutionMeasurements {
            sigma_h_m: Some(sigma_h_m),
            sigma_v_m: Some(quality.sigma_u),
            residual_rms_m: Some(quality.residual_rms_m),
            predicted_rms_m: Some(quality.predicted_rms_m),
            hpl_m: Some(quality.hpl_m),
            vpl_m: Some(quality.vpl_m),
        }
    } else {
        bijux_gnss_infra::api::receiver::AdvancedSolutionMeasurements::default()
    }
}

fn evidence_backed_rtk_fix_accepted(
    claim: bijux_gnss_infra::api::receiver::AdvancedSolutionClaim,
) -> bool {
    matches!(claim, bijux_gnss_infra::api::receiver::AdvancedSolutionClaim::Fixed)
}

pub(super) fn handle_rtk(command: GnssCommand) -> Result<()> {
    let GnssCommand::Rtk { common, base_obs, rover_obs, eph, base_ecef, tolerance_s, ref_policy } =
        command
    else {
        bail!("invalid command for handler");
    };

    let _ = runtime_config_from_env(&common, None);
    let profile = load_config(&common)?;
    let dataset = load_dataset(&common)?;
    let header = artifact_header(&common, &profile, dataset.as_ref())?;
    let base_epochs = read_obs_epochs(&base_obs)?;
    let rover_epochs = read_obs_epochs(&rover_obs)?;
    let ephs = read_ephemeris(&eph)?;
    let base_xyz = parse_ecef(&base_ecef)?;

    let mut aligner = bijux_gnss_infra::api::receiver::EpochAligner::new(tolerance_s);
    let aligned = aligner.align(&base_epochs, &rover_epochs);

    let out_dir = artifacts_dir(&common, "rtk", dataset.as_ref())?;

    let mut sd_lines = Vec::new();
    let mut dd_lines = Vec::new();
    let mut baseline_lines = Vec::new();
    let mut baseline_quality_lines = Vec::new();
    let mut fix_audit_lines = Vec::new();
    let mut precision_lines = Vec::new();
    let mut correction_input_lines = Vec::new();
    let mut ambiguity_state_lines = Vec::new();
    let mut advanced_solution_lines = Vec::new();
    let mut baseline_executed_count = 0usize;
    let mut baseline_not_ready_count = 0usize;
    let mut baseline_unsupported_count = 0usize;
    let mut baseline_quality_executed_count = 0usize;
    let mut ambiguity_state_executed_count = 0usize;
    let mut ambiguity_state_not_ready_count = 0usize;
    let mut ambiguity_state_unsupported_count = 0usize;
    let mut fix_state = bijux_gnss_infra::api::receiver::RtkAmbiguityFixState::default();
    let fixer = bijux_gnss_infra::api::receiver::RtkRatioTestFixer::new(
        bijux_gnss_infra::api::receiver::RtkAmbiguityFixPolicy::default(),
    );
    let support_matrix = bijux_gnss_infra::api::receiver::support_status_matrix();
    let mut last_ref: Option<bijux_gnss_infra::api::core::SigId> = None;
    let mut ref_selector = bijux_gnss_infra::api::receiver::RefSatSelector::new(5);
    let mut ref_selectors: std::collections::BTreeMap<
        Constellation,
        bijux_gnss_infra::api::receiver::RefSatSelector,
    > = std::collections::BTreeMap::new();

    for (base, rover) in &aligned {
        let sd = bijux_gnss_infra::api::receiver::build_sd(base, rover);
        for item in &sd {
            let wrapped = bijux_gnss_infra::api::receiver::RtkSdEpochV1 {
                header: header.clone(),
                payload: item.clone(),
            };
            sd_lines.push(serde_json::to_string(&wrapped)?);
        }
        let dd = match ref_policy {
            RefPolicy::Global => {
                if let Some(ref_sig) = ref_selector.choose(&sd) {
                    bijux_gnss_infra::api::receiver::build_dd(&sd, ref_sig)
                } else {
                    Vec::new()
                }
            }
            RefPolicy::PerConstellation => {
                let refs = bijux_gnss_infra::api::receiver::choose_ref_sat_per_constellation(&sd);
                let mut chosen = std::collections::BTreeMap::new();
                for (constellation, sig) in refs {
                    let selector = ref_selectors
                        .entry(constellation)
                        .or_insert_with(|| bijux_gnss_infra::api::receiver::RefSatSelector::new(5));
                    let subset: Vec<_> = sd
                        .iter()
                        .filter(|s| s.sig.sat.constellation == constellation)
                        .cloned()
                        .collect();
                    if let Some(picked) = selector.choose(&subset) {
                        chosen.insert(constellation, picked);
                    } else {
                        chosen.insert(constellation, sig);
                    }
                }
                bijux_gnss_infra::api::receiver::build_dd_per_constellation(&sd, &chosen)
            }
        };
        for item in &dd {
            let wrapped = bijux_gnss_infra::api::receiver::RtkDdEpochV1 {
                header: header.clone(),
                payload: item.clone(),
            };
            dd_lines.push(serde_json::to_string(&wrapped)?);
        }
        let ref_sig = dd.first().map(|d| d.ref_sig);
        let ref_changed = ref_sig != last_ref;
        if ref_sig.is_some() {
            last_ref = ref_sig;
        }

        let float_baseline = bijux_gnss_infra::api::receiver::solve_float_baseline_dd(
            &dd,
            base_xyz,
            &ephs,
            rover.t_rx_s.0,
        );

        let prereq = bijux_gnss_infra::api::receiver::AdvancedPrerequisites {
            has_base_observations: !base.sats.is_empty(),
            has_rover_observations: !rover.sats.is_empty(),
            has_ephemeris: !ephs.is_empty(),
            has_reference_frame: true,
            has_corrections: true,
            has_min_satellites: dd.len() >= 3,
            has_ambiguity_state: float_baseline.is_some(),
        };
        let prereq_decision = bijux_gnss_infra::api::receiver::evaluate_prerequisites(
            bijux_gnss_infra::api::receiver::AdvancedMode::Rtk,
            &prereq,
        );
        let (refused_status, refused_reason) = refused_execution_status(&prereq_decision);
        let float_ambiguity_state = match float_baseline.as_ref() {
            Some(float_baseline_solution) => {
                Some(
                    bijux_gnss_infra::api::receiver::rtk_float_ambiguity_state_from_baseline_solution(
                        float_baseline_solution,
                    )
                    .ok_or_else(|| {
                        eyre!(
                            "rtk ambiguity state extraction failed for epoch {} despite baseline execution",
                            rover.epoch_idx
                        )
                    })?,
                )
            }
            None => None,
        };
        let fix_input = float_ambiguity_state.clone().unwrap_or_else(|| {
            bijux_gnss_infra::api::receiver::RtkFloatAmbiguityState {
                ids: Vec::new(),
                float_cycles: Vec::new(),
                covariance_cycles2: Vec::new(),
                integer_compatible: false,
            }
        });
        let (fix_result, mut audit) =
            fixer.fix_with_state(rover.epoch_idx, &fix_input, &mut fix_state);

        let mut baseline_fixed = false;
        let mut baseline_payload = None;
        let mut baseline_quality_payload = None;
        if let Some(float_baseline_solution) = float_baseline.as_ref() {
            let mut adjusted = bijux_gnss_infra::api::receiver::BaselineSolution {
                enu_m: float_baseline_solution.enu_m,
                covariance_m2: Some(float_baseline_solution.covariance_enu_m2),
                fixed: false,
            };
            if let Some((fixed_ids, fixed_integers)) =
                bijux_gnss_infra::api::receiver::rtk_ambiguity_state_from_fixed_solution(
                    &fix_result,
                )
            {
                if let Some(conditioned) =
                    bijux_gnss_infra::api::receiver::rtk_conditioned_baseline_from_fixed_ambiguities(
                        float_baseline_solution,
                        &fixed_ids,
                        &fixed_integers,
                    )
                {
                    let conditioned_baseline = bijux_gnss_infra::api::receiver::BaselineSolution {
                        enu_m: conditioned.enu_m,
                        covariance_m2: Some(conditioned.covariance_enu_m2),
                        fixed: true,
                    };
                    let fixed_guard =
                        bijux_gnss_infra::api::receiver::evaluate_rtk_fixed_baseline_guard(
                            &dd,
                            base_xyz,
                            &conditioned_baseline,
                            &ephs,
                            rover.t_rx_s.0,
                            bijux_gnss_infra::api::receiver::RtkFixedBaselineGuardPolicy::default(),
                        );
                    if fixed_guard.accepted {
                        adjusted = conditioned_baseline;
                    } else {
                        audit.status =
                            bijux_gnss_infra::api::receiver::RtkAmbiguityFixStatus::Failed;
                        audit.reason = format!("fixed_guard:{}", fixed_guard.reasons.join(","));
                        audit.fixed_count = 0;
                    }
                } else {
                    audit.status = bijux_gnss_infra::api::receiver::RtkAmbiguityFixStatus::Failed;
                    audit.reason = "baseline_conditioning_failed".to_string();
                    audit.fixed_count = 0;
                }
            }
            baseline_fixed = adjusted.fixed;
            let (rms_obs, rms_pred, used_sats) = if let Some((rms_obs, rms_pred, count)) =
                bijux_gnss_infra::api::receiver::dd_residual_metrics(
                    &dd,
                    base_xyz,
                    adjusted.enu_m,
                    &ephs,
                    rover.t_rx_s.0,
                ) {
                (rms_obs, rms_pred, count)
            } else {
                (0.0, 0.0, 0)
            };
            let separation = bijux_gnss_infra::api::receiver::solution_separation(
                &dd,
                base_xyz,
                &ephs,
                rover.t_rx_s.0,
            );
            let mut sep_sig = None;
            let mut sep_max = None;
            if let Some(seps) = separation {
                if let Some(max) =
                    seps.iter().max_by(|a, b| a.delta_enu_m.total_cmp(&b.delta_enu_m))
                {
                    sep_sig = Some(format!("{:?}", max.sig));
                    sep_max = Some(max.delta_enu_m);
                }
            }
            if let Some(cov) = adjusted.covariance_m2 {
                let sigma_e = cov[0][0].abs().sqrt();
                let sigma_n = cov[1][1].abs().sqrt();
                let sigma_u = cov[2][2].abs().sqrt();
                let sigma_h = (sigma_e * sigma_e + sigma_n * sigma_n).sqrt();
                let hpl = sigma_h * 6.0;
                let vpl = sigma_u * 6.0;
                let quality = bijux_gnss_infra::api::receiver::RtkBaselineQuality {
                    epoch_idx: rover.epoch_idx,
                    fixed: adjusted.fixed,
                    sigma_e,
                    sigma_n,
                    sigma_u,
                    used_sats,
                    residual_rms_m: rms_obs,
                    predicted_rms_m: rms_pred,
                    hpl_m: hpl,
                    vpl_m: vpl,
                    separation_sig: sep_sig,
                    separation_max_m: sep_max,
                };
                let wrapped = bijux_gnss_infra::api::receiver::RtkBaselineQualityV1 {
                    header: header.clone(),
                    payload: bijux_gnss_infra::api::receiver::ExecutionArtifact {
                        epoch_idx: rover.epoch_idx,
                        status: bijux_gnss_infra::api::receiver::ExecutionStatus::Executed,
                        reason: None,
                        value: Some(quality.clone()),
                    },
                };
                baseline_quality_lines.push(serde_json::to_string(&wrapped)?);
                baseline_quality_payload = Some(quality);
                baseline_quality_executed_count = baseline_quality_executed_count.saturating_add(1);
            } else {
                return Err(eyre!(
                    "rtk baseline quality missing covariance for epoch {} after baseline execution",
                    rover.epoch_idx
                ));
            }
            baseline_payload = Some(adjusted);
        } else if prereq_decision.ready {
            return Err(eyre!(
                "rtk baseline solver produced no solution for epoch {} despite ready prerequisites",
                rover.epoch_idx
            ));
        }
        let baseline_wrapped = bijux_gnss_infra::api::receiver::RtkBaselineEpochV1 {
            header: header.clone(),
            payload: bijux_gnss_infra::api::receiver::ExecutionArtifact {
                epoch_idx: rover.epoch_idx,
                status: if baseline_payload.is_some() {
                    bijux_gnss_infra::api::receiver::ExecutionStatus::Executed
                } else {
                    refused_status
                },
                reason: if baseline_payload.is_some() { None } else { refused_reason.clone() },
                value: baseline_payload,
            },
        };
        baseline_lines.push(serde_json::to_string(&baseline_wrapped)?);
        match baseline_wrapped.payload.status {
            bijux_gnss_infra::api::receiver::ExecutionStatus::Executed => {
                baseline_executed_count = baseline_executed_count.saturating_add(1);
            }
            bijux_gnss_infra::api::receiver::ExecutionStatus::NotReady => {
                baseline_not_ready_count = baseline_not_ready_count.saturating_add(1);
            }
            bijux_gnss_infra::api::receiver::ExecutionStatus::Unsupported => {
                baseline_unsupported_count = baseline_unsupported_count.saturating_add(1);
            }
        }
        if baseline_quality_payload.is_none() {
            let baseline_quality_wrapped = bijux_gnss_infra::api::receiver::RtkBaselineQualityV1 {
                header: header.clone(),
                payload: bijux_gnss_infra::api::receiver::ExecutionArtifact {
                    epoch_idx: rover.epoch_idx,
                    status: refused_status,
                    reason: refused_reason.clone(),
                    value: None::<bijux_gnss_infra::api::receiver::RtkBaselineQuality>,
                },
            };
            baseline_quality_lines.push(serde_json::to_string(&baseline_quality_wrapped)?);
        }
        let fix_audit = bijux_gnss_infra::api::receiver::RtkFixAuditV1 {
            header: header.clone(),
            payload: audit.clone(),
        };
        fix_audit_lines.push(serde_json::to_string(&fix_audit)?);

        let correction_input = bijux_gnss_infra::api::receiver::CorrectionInputArtifact {
            epoch_idx: rover.epoch_idx,
            mode: bijux_gnss_infra::api::receiver::AdvancedMode::Rtk,
            ephemeris_count: ephs.len(),
            products_ok: !ephs.is_empty(),
            correction_tags: vec!["broadcast_ephemeris".to_string()],
        };
        let correction_wrapped = bijux_gnss_infra::api::core::ArtifactV1 {
            header: header.clone(),
            payload: correction_input,
        };
        correction_input_lines.push(serde_json::to_string(&correction_wrapped)?);

        let ambiguity_wrapped = bijux_gnss_infra::api::core::ArtifactV1 {
            header: header.clone(),
            payload: if let Some(float_ambiguity_state) = float_ambiguity_state.as_ref() {
                ambiguity_state_executed_count = ambiguity_state_executed_count.saturating_add(1);
                bijux_gnss_infra::api::receiver::ExecutionArtifact {
                    epoch_idx: rover.epoch_idx,
                    status: bijux_gnss_infra::api::receiver::ExecutionStatus::Executed,
                    reason: None,
                    value: Some(bijux_gnss_infra::api::receiver::AmbiguityStateArtifact {
                        epoch_idx: rover.epoch_idx,
                        mode: bijux_gnss_infra::api::receiver::AdvancedMode::Rtk,
                        float_count: float_ambiguity_state.float_cycles.len(),
                        fixed_count: audit.fixed_count,
                    }),
                }
            } else {
                match refused_status {
                    bijux_gnss_infra::api::receiver::ExecutionStatus::NotReady => {
                        ambiguity_state_not_ready_count =
                            ambiguity_state_not_ready_count.saturating_add(1);
                    }
                    bijux_gnss_infra::api::receiver::ExecutionStatus::Unsupported => {
                        ambiguity_state_unsupported_count =
                            ambiguity_state_unsupported_count.saturating_add(1);
                    }
                    bijux_gnss_infra::api::receiver::ExecutionStatus::Executed => {}
                }
                bijux_gnss_infra::api::receiver::ExecutionArtifact {
                    epoch_idx: rover.epoch_idx,
                    status: refused_status,
                    reason: refused_reason.clone(),
                    value: None::<bijux_gnss_infra::api::receiver::AmbiguityStateArtifact>,
                }
            },
        };
        ambiguity_state_lines.push(serde_json::to_string(&ambiguity_wrapped)?);
        let raw_claim = if baseline_fixed {
            bijux_gnss_infra::api::receiver::AdvancedSolutionClaim::Fixed
        } else {
            bijux_gnss_infra::api::receiver::AdvancedSolutionClaim::Float
        };
        let ambiguity_state_count =
            float_ambiguity_state.as_ref().map_or(0usize, |state| state.float_cycles.len());
        let correction_source = "broadcast_ephemeris".to_string();
        let measurements = rtk_advanced_solution_measurements(baseline_quality_payload.as_ref());
        let evidence = bijux_gnss_infra::api::receiver::evaluate_solution_evidence(
            bijux_gnss_infra::api::receiver::AdvancedMode::Rtk,
            raw_claim,
            ambiguity_state_count,
            fix_result.ratio,
            &correction_source,
            &measurements,
        );
        let claim_decision = bijux_gnss_infra::api::receiver::apply_downgrade_policy(
            bijux_gnss_infra::api::receiver::AdvancedMode::Rtk,
            &prereq_decision,
            raw_claim,
            Some(&evidence),
        );
        let source_obs_id = rover
            .manifest
            .as_ref()
            .map(|manifest| manifest.epoch_id.clone())
            .unwrap_or_else(|| format!("obs-epoch-{:010}", rover.epoch_idx));
        let advanced_solution = bijux_gnss_infra::api::receiver::AdvancedSolutionArtifact {
            epoch_idx: rover.epoch_idx,
            mode: bijux_gnss_infra::api::receiver::AdvancedMode::Rtk,
            status: claim_decision.status.clone(),
            downgraded: claim_decision.downgraded,
            downgrade_reason: claim_decision.downgrade_reason.clone(),
            prerequisites: prereq,
            refusal_class: prereq_decision.refusal_class,
            provenance: bijux_gnss_infra::api::receiver::AdvancedSolutionProvenance {
                claim: claim_decision.claim,
                ambiguity_state_count,
                correction_source,
                fallback_from: claim_decision.downgrade_reason.clone(),
                fixed_ratio: fix_result.ratio,
                measurements,
                evidence,
            },
            artifact_id: format!("rtk-advanced-epoch-{:010}", rover.epoch_idx),
            source_observation_epoch_id: source_obs_id,
        };
        let advanced_wrapped = bijux_gnss_infra::api::core::ArtifactV1 {
            header: header.clone(),
            payload: advanced_solution,
        };
        advanced_solution_lines.push(serde_json::to_string(&advanced_wrapped)?);

        let slip_count =
            base.sats.iter().chain(rover.sats.iter()).filter(|s| s.lock_flags.cycle_slip).count();
        let precision = bijux_gnss_infra::api::receiver::RtkPrecision {
            epoch_idx: rover.epoch_idx,
            fix_accepted: evidence_backed_rtk_fix_accepted(claim_decision.claim),
            ratio: fix_result.ratio,
            fixed_count: audit.fixed_count,
            ref_changed,
            slip_count,
        };
        let wrapped = bijux_gnss_infra::api::receiver::RtkPrecisionV1 {
            header: header.clone(),
            payload: precision,
        };
        precision_lines.push(serde_json::to_string(&wrapped)?);
    }

    let sd_path = out_dir.join("rtk_sd.jsonl");
    fs::write(&sd_path, sd_lines.join("\n"))?;
    let dd_path = out_dir.join("rtk_dd.jsonl");
    fs::write(&dd_path, dd_lines.join("\n"))?;
    let baseline_path = out_dir.join("rtk_baseline.jsonl");
    fs::write(&baseline_path, baseline_lines.join("\n"))?;
    let baseline_quality_path = out_dir.join("rtk_baseline_quality.jsonl");
    fs::write(&baseline_quality_path, baseline_quality_lines.join("\n"))?;
    let fix_audit_path = out_dir.join("rtk_fix_audit.jsonl");
    fs::write(&fix_audit_path, fix_audit_lines.join("\n"))?;
    let precision_path = out_dir.join("rtk_precision.jsonl");
    fs::write(&precision_path, precision_lines.join("\n"))?;
    let correction_input_path = out_dir.join("rtk_correction_input.jsonl");
    fs::write(&correction_input_path, correction_input_lines.join("\n"))?;
    let ambiguity_state_path = out_dir.join("rtk_ambiguity_state.jsonl");
    fs::write(&ambiguity_state_path, ambiguity_state_lines.join("\n"))?;
    let advanced_solution_path = out_dir.join("rtk_advanced_solution.jsonl");
    fs::write(&advanced_solution_path, advanced_solution_lines.join("\n"))?;
    let support_matrix_path = out_dir.join("rtk_support_matrix.json");
    fs::write(&support_matrix_path, serde_json::to_string_pretty(&support_matrix)?)?;

    let align_report = aligner.report(base_epochs.len(), rover_epochs.len());
    fs::write(out_dir.join("rtk_align.json"), serde_json::to_string_pretty(&align_report)?)?;

    validate_jsonl_schema(&schema_path("rtk_baseline_v1.schema.json"), &baseline_path, false)?;
    validate_jsonl_schema(&schema_path("rtk_sd_v1.schema.json"), &sd_path, false)?;
    validate_jsonl_schema(
        &schema_path("rtk_baseline_quality_v1.schema.json"),
        &baseline_quality_path,
        false,
    )?;
    validate_jsonl_schema(&schema_path("rtk_fix_audit_v1.schema.json"), &fix_audit_path, false)?;
    validate_jsonl_schema(&schema_path("rtk_precision_v1.schema.json"), &precision_path, false)?;
    validate_jsonl_schema(
        &schema_path("rtk_correction_input_v1.schema.json"),
        &correction_input_path,
        false,
    )?;
    validate_jsonl_schema(
        &schema_path("rtk_ambiguity_state_v1.schema.json"),
        &ambiguity_state_path,
        false,
    )?;
    validate_jsonl_schema(
        &schema_path("rtk_advanced_solution_v1.schema.json"),
        &advanced_solution_path,
        false,
    )?;

    let report = serde_json::json!({
        "aligned_epochs": aligned.len(),
        "sd_count": sd_lines.len(),
        "dd_count": dd_lines.len(),
        "baseline_executed_count": baseline_executed_count,
        "baseline_not_ready_count": baseline_not_ready_count,
        "baseline_unsupported_count": baseline_unsupported_count,
        "baseline_quality_executed_count": baseline_quality_executed_count,
        "ambiguity_state_executed_count": ambiguity_state_executed_count,
        "ambiguity_state_not_ready_count": ambiguity_state_not_ready_count,
        "ambiguity_state_unsupported_count": ambiguity_state_unsupported_count,
        "advanced_solution_count": advanced_solution_lines.len(),
        "support_matrix_path": support_matrix_path.display().to_string()
    });
    write_manifest(&common, "rtk", &profile, dataset.as_ref(), &report)?;

    Ok(())
}
