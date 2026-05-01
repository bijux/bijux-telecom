fn handle_cacode(command: GnssCommand) -> Result<()> {
    let GnssCommand::CaCode { prn, count } = command else {
        bail!("invalid command for handler");
    };

    let code = generate_ca_code(Prn(prn))
        .map_err(|err| eyre!("failed to generate C/A code for PRN {prn}: {err}"))?;
    let count = count.min(code.len());
    for chip in code.iter().take(count) {
        print!("{chip} ");
    }
    println!();

    Ok(())
}

fn handle_nav(command: GnssCommand) -> Result<()> {
    let GnssCommand::Nav { command } = command else {
        bail!("invalid command for handler");
    };
    let common = match &command {
        NavCommand::Decode { common, .. } => common,
    };
    let profile = load_config(common)?;
    let dataset = load_dataset(common)?;

    match command {
                    NavCommand::Decode { common, track, prn } => {
                        let _ = runtime_config_from_env(&common, None);
                        let rows = read_tracking_dump(&track)?;
                        let mut prompt = Vec::new();
                        let target = SatId {
                            constellation: Constellation::Gps,
                            prn,
                        };
                        let mut sorted: Vec<_> = rows.into_iter().filter(|r| r.sat == target).collect();
                        sorted.sort_by_key(|r| r.epoch_idx);
                        for row in sorted {
                            prompt.push(row.prompt_i);
                        }
                        let bits = bijux_gnss_infra::api::nav::bit_sync_from_prompt(&prompt);
                        let (mut ephs, stats) = bijux_gnss_infra::api::nav::decode_subframes(&bits.bits);
                        for eph in ephs.iter_mut() {
                            eph.sat = target;
                        }
                        let report = NavDecodeReport {
                            sat: target,
                            preamble_hits: stats.preamble_hits,
                            parity_pass_rate: stats.parity_pass_rate,
                            ephemerides: ephs.clone(),
                        };
                        emit_report(&common, "nav_decode", &report)?;
                        write_ephemeris(&common, &ephs, &profile, dataset.as_ref())?;
                        write_manifest(&common, "nav_decode", &profile, dataset.as_ref(), &report)?;
                    }
                }

    Ok(())
}

fn handle_rtk(command: GnssCommand) -> Result<()> {
    let GnssCommand::Rtk {
                common,
                base_obs,
                rover_obs,
                eph,
                base_ecef,
                tolerance_s,
                ref_policy,
            } = command else {
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
                    let mut fix_state = bijux_gnss_infra::api::receiver::FixState::default();
                    let fixer = bijux_gnss_infra::api::receiver::NaiveFixer::new(
                        bijux_gnss_infra::api::receiver::FixPolicy::default(),
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
                                let refs =
                                    bijux_gnss_infra::api::receiver::choose_ref_sat_per_constellation(&sd);
                                let mut chosen = std::collections::BTreeMap::new();
                                for (constellation, sig) in refs {
                                    let selector =
                                        ref_selectors.entry(constellation).or_insert_with(|| {
                                            bijux_gnss_infra::api::receiver::RefSatSelector::new(5)
                                        });
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
    
                        let mut baseline = bijux_gnss_infra::api::receiver::solve_baseline_dd(
                            &dd,
                            base_xyz,
                            &ephs,
                            rover.t_rx_s.0,
                        );
    
                        let float = bijux_gnss_infra::api::receiver::FloatAmbiguitySolution {
                            ids: dd
                                .iter()
                                .map(|d| bijux_gnss_infra::api::core::AmbiguityId {
                                    sig: d.sig,
                                    signal: format!("{:?}", d.sig.band),
                                })
                                .collect(),
                            float_cycles: dd.iter().map(|d| d.phase_cycles).collect(),
                            covariance: dd.iter().map(|d| vec![d.variance_phase]).collect(),
                        };
                        let (fix_result, audit) =
                            fixer.fix_with_state(rover.epoch_idx, &float, &mut fix_state);
                        let fix_audit = bijux_gnss_infra::api::receiver::RtkFixAuditV1 {
                            header: header.clone(),
                            payload: audit.clone(),
                        };
                        fix_audit_lines.push(serde_json::to_string(&fix_audit)?);
    
                        if let Some(baseline_val) = baseline.take() {
                            let before_rms = bijux_gnss_infra::api::receiver::dd_residual_metrics(
                                &dd,
                                base_xyz,
                                baseline_val.enu_m,
                                &ephs,
                            rover.t_rx_s.0,
                            )
                            .map(|(rms, _pred, _)| rms);
                            let mut adjusted = bijux_gnss_infra::api::receiver::apply_fix_hold(
                                baseline_val,
                                fix_result.accepted,
                            );
                            let after_rms = bijux_gnss_infra::api::receiver::dd_residual_metrics(
                                &dd,
                                base_xyz,
                                adjusted.enu_m,
                                &ephs,
                            rover.t_rx_s.0,
                            )
                            .map(|(rms, _pred, _)| rms);
                            if let (Some(before), Some(after)) = (before_rms, after_rms) {
                                if after > before * 1.1 {
                                    adjusted.fixed = false;
                                } else {
                                    adjusted.fixed = fix_result.accepted;
                                }
                            } else {
                                adjusted.fixed = fix_result.accepted;
                            }
                            let (rms_obs, rms_pred, used_sats) =
                                if let Some((rms_obs, rms_pred, count)) =
                                    bijux_gnss_infra::api::receiver::dd_residual_metrics(
                                        &dd,
                                        base_xyz,
                                        adjusted.enu_m,
                                        &ephs,
                            rover.t_rx_s.0,
                                    )
                                {
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
                                if let Some(max) = seps
                                    .iter()
                                    .max_by(|a, b| a.delta_enu_m.total_cmp(&b.delta_enu_m))
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
                                    payload: quality,
                                };
                                baseline_quality_lines.push(serde_json::to_string(&wrapped)?);
                            }
                            let wrapped = bijux_gnss_infra::api::receiver::RtkBaselineEpochV1 {
                                header: header.clone(),
                                payload: adjusted,
                            };
                            baseline_lines.push(serde_json::to_string(&wrapped)?);
                        }

                        let correction_input =
                            bijux_gnss_infra::api::receiver::CorrectionInputArtifact {
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

                        let ambiguity_state =
                            bijux_gnss_infra::api::receiver::AmbiguityStateArtifact {
                                epoch_idx: rover.epoch_idx,
                                mode: bijux_gnss_infra::api::receiver::AdvancedMode::Rtk,
                                float_count: float.float_cycles.len(),
                                fixed_count: usize::from(fix_result.accepted),
                            };
                        let ambiguity_wrapped = bijux_gnss_infra::api::core::ArtifactV1 {
                            header: header.clone(),
                            payload: ambiguity_state,
                        };
                        ambiguity_state_lines.push(serde_json::to_string(&ambiguity_wrapped)?);

                        let prereq = bijux_gnss_infra::api::receiver::AdvancedPrerequisites {
                            has_base_observations: !base.sats.is_empty(),
                            has_rover_observations: !rover.sats.is_empty(),
                            has_ephemeris: !ephs.is_empty(),
                            has_reference_frame: true,
                            has_corrections: true,
                            has_min_satellites: dd.len() >= 3,
                            has_ambiguity_state: !float.float_cycles.is_empty(),
                        };
                        let prereq_decision =
                            bijux_gnss_infra::api::receiver::evaluate_prerequisites(
                                bijux_gnss_infra::api::receiver::AdvancedMode::Rtk,
                                &prereq,
                            );
                        let raw_claim = if fix_result.accepted {
                            bijux_gnss_infra::api::receiver::AdvancedSolutionClaim::Fixed
                        } else {
                            bijux_gnss_infra::api::receiver::AdvancedSolutionClaim::Float
                        };
                        let (status, downgraded, downgrade_reason, claim) =
                            bijux_gnss_infra::api::receiver::apply_downgrade_policy(
                                bijux_gnss_infra::api::receiver::AdvancedMode::Rtk,
                                &prereq_decision,
                                raw_claim,
                            );
                        let source_obs_id = rover
                            .manifest
                            .as_ref()
                            .map(|manifest| manifest.epoch_id.clone())
                            .unwrap_or_else(|| format!("obs-epoch-{:010}", rover.epoch_idx));
                        let advanced_solution =
                            bijux_gnss_infra::api::receiver::AdvancedSolutionArtifact {
                                epoch_idx: rover.epoch_idx,
                                mode: bijux_gnss_infra::api::receiver::AdvancedMode::Rtk,
                                status,
                                downgraded,
                                downgrade_reason: downgrade_reason.clone(),
                                prerequisites: prereq,
                                refusal_class: prereq_decision.refusal_class,
                                provenance: bijux_gnss_infra::api::receiver::AdvancedSolutionProvenance {
                                    claim,
                                    ambiguity_state_count: float.float_cycles.len(),
                                    correction_source: "broadcast_ephemeris".to_string(),
                                    fallback_from: downgrade_reason,
                                    fixed_ratio: fix_result.ratio,
                                },
                                artifact_id: format!("rtk-advanced-epoch-{:010}", rover.epoch_idx),
                                source_observation_epoch_id: source_obs_id,
                            };
                        let advanced_wrapped = bijux_gnss_infra::api::core::ArtifactV1 {
                            header: header.clone(),
                            payload: advanced_solution,
                        };
                        advanced_solution_lines.push(serde_json::to_string(&advanced_wrapped)?);
    
                        let slip_count = base
                            .sats
                            .iter()
                            .chain(rover.sats.iter())
                            .filter(|s| s.lock_flags.cycle_slip)
                            .count();
                        let precision = bijux_gnss_infra::api::receiver::RtkPrecision {
                            epoch_idx: rover.epoch_idx,
                            fix_accepted: fix_result.accepted,
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
                    fs::write(
                        &support_matrix_path,
                        serde_json::to_string_pretty(&support_matrix)?,
                    )?;
    
                    let align_report = aligner.report(base_epochs.len(), rover_epochs.len());
                    fs::write(
                        out_dir.join("rtk_align.json"),
                        serde_json::to_string_pretty(&align_report)?,
                    )?;
    
                    validate_json_schema(
                        &schema_path("rtk_baseline_v1.schema.json"),
                        &baseline_path,
                        false,
                    )?;
                    validate_jsonl_schema(
                        &schema_path("rtk_baseline_quality_v1.schema.json"),
                        &baseline_quality_path,
                        false,
                    )?;
                    validate_jsonl_schema(
                        &schema_path("rtk_fix_audit_v1.schema.json"),
                        &fix_audit_path,
                        false,
                    )?;
                    validate_jsonl_schema(
                        &schema_path("rtk_precision_v1.schema.json"),
                        &precision_path,
                        false,
                    )?;

                    let report = serde_json::json!({
                        "aligned_epochs": aligned.len(),
                        "sd_count": sd_lines.len(),
                        "dd_count": dd_lines.len(),
                        "advanced_solution_count": advanced_solution_lines.len(),
                        "support_matrix_path": support_matrix_path.display().to_string()
                    });
                    write_manifest(&common, "rtk", &profile, dataset.as_ref(), &report)?;

    Ok(())
}

fn handle_diagnostics(command: GnssCommand) -> Result<()> {
    let GnssCommand::Diagnostics { command } = command else {
        bail!("invalid command for handler");
    };

    match command {
        DiagnosticsCommand::OperatorMap { common } => {
            let _ = runtime_config_from_env(&common, None);
            let report = operator_map_report();
            match common.report {
                ReportFormat::Table => print_operator_map_table(&report),
                ReportFormat::Json => emit_report(&common, "diagnostics_operator_map", &report)?,
            }
            write_diagnostics_report_artifact(
                &common,
                "diagnostics_operator_map",
                &report,
                "diagnostics_operator_map_report.schema.json",
            )?;
            write_manifest(
                &common,
                "diagnostics_operator_map",
                &ReceiverConfig::default(),
                None,
                &report,
            )?;
        }
        DiagnosticsCommand::Workflow { common } => {
            let _ = runtime_config_from_env(&common, None);
            let report = workflow_map_report();
            match common.report {
                ReportFormat::Table => print_workflow_table(&report),
                ReportFormat::Json => emit_report(&common, "diagnostics_workflow", &report)?,
            }
            write_diagnostics_report_artifact(
                &common,
                "diagnostics_workflow",
                &report,
                "diagnostics_workflow_report.schema.json",
            )?;
            write_manifest(
                &common,
                "diagnostics_workflow",
                &ReceiverConfig::default(),
                None,
                &report,
            )?;
        }
        DiagnosticsCommand::Summarize {
            common,
            run_dir,
            top,
        } => {
            let _ = runtime_config_from_env(&common, None);
            let events = summarize_run_diagnostics(&run_dir)?;
            let summary = bijux_gnss_infra::api::core::aggregate_diagnostics(&events);
            let mut entries = summary.entries.clone();
            entries.sort_by(|a, b| b.count.cmp(&a.count));
            let report = serde_json::json!({
                "schema_version": 1,
                "run_dir": run_dir.display().to_string(),
                "total": summary.total,
                "top": top,
                "entries": entries,
                "layered": layered_report(
                    summarize_critical_entries(&summary.entries),
                    serde_json::json!({
                        "top_entries": summary.entries,
                        "total": summary.total
                    })
                ),
            });
            match common.report {
                ReportFormat::Table => {
                    print_diagnostics_summary_table(
                        report.get("entries").and_then(|v| v.as_array()),
                        top,
                    );
                }
                ReportFormat::Json => emit_report(&common, "diagnostics_summarize", &report)?,
            }
            write_diagnostics_report_artifact(
                &common,
                "diagnostics_summarize",
                &report,
                "diagnostics_summary_report.schema.json",
            )?;
            write_manifest(&common, "diagnostics_summarize", &ReceiverConfig::default(), None, &report)?;
        }
        DiagnosticsCommand::Explain { common, run_dir } => {
            let _ = runtime_config_from_env(&common, None);
            let summary = explain_run_scope(&run_dir)?;
            match common.report {
                ReportFormat::Table => print_diagnostics_explain_table(&summary),
                ReportFormat::Json => emit_report(&common, "diagnostics_explain", &summary)?,
            }
            write_diagnostics_report_artifact(
                &common,
                "diagnostics_explain",
                &summary,
                "diagnostics_explain_report.schema.json",
            )?;
            write_manifest(
                &common,
                "diagnostics_explain",
                &ReceiverConfig::default(),
                None,
                &summary,
            )?;
        }
        DiagnosticsCommand::VerifyRepro { common, run_dir } => {
            let _ = runtime_config_from_env(&common, None);
            let report = verify_repro_bundle(&run_dir)?;
            match common.report {
                ReportFormat::Table => print_verify_repro_table(&report),
                ReportFormat::Json => emit_report(&common, "diagnostics_verify_repro", &report)?,
            }
            write_diagnostics_report_artifact(
                &common,
                "diagnostics_verify_repro",
                &report,
                "diagnostics_verify_repro_report.schema.json",
            )?;
            write_manifest(
                &common,
                "diagnostics_verify_repro",
                &ReceiverConfig::default(),
                None,
                &report,
            )?;
        }
        DiagnosticsCommand::Compare {
            common,
            baseline_run_dir,
            candidate_run_dir,
        } => {
            let _ = runtime_config_from_env(&common, None);
            let report = compare_run_evidence(&baseline_run_dir, &candidate_run_dir)?;
            match common.report {
                ReportFormat::Table => print_compare_report_table(&report),
                ReportFormat::Json => emit_report(&common, "diagnostics_compare", &report)?,
            }
            write_diagnostics_report_artifact(
                &common,
                "diagnostics_compare",
                &report,
                "diagnostics_compare_report.schema.json",
            )?;
            write_manifest(
                &common,
                "diagnostics_compare",
                &ReceiverConfig::default(),
                None,
                &report,
            )?;
        }
        DiagnosticsCommand::ReplayAudit {
            common,
            baseline_run_dir,
            candidate_run_dir,
        } => {
            let _ = runtime_config_from_env(&common, None);
            let report = replay_audit_report(&baseline_run_dir, &candidate_run_dir)?;
            match common.report {
                ReportFormat::Table => print_replay_audit_table(&report),
                ReportFormat::Json => emit_report(&common, "diagnostics_replay_audit", &report)?,
            }
            write_diagnostics_report_artifact(
                &common,
                "diagnostics_replay_audit",
                &report,
                "diagnostics_replay_audit_report.schema.json",
            )?;
            write_manifest(
                &common,
                "diagnostics_replay_audit",
                &ReceiverConfig::default(),
                None,
                &report,
            )?;
        }
        DiagnosticsCommand::AdvancedGate {
            common,
            run_dir,
            mode,
            strict,
        } => {
            let _ = runtime_config_from_env(&common, None);
            let report = advanced_gate_report(&run_dir, mode)?;
            let passed = report
                .get("gate_passed")
                .and_then(|v| v.as_bool())
                .unwrap_or(false);
            if strict && !passed {
                return Err(classified_error(
                    CliErrorClass::UnsupportedScience,
                    format!(
                        "advanced gate failed for mode={:?} run_dir={}",
                        mode,
                        run_dir.display()
                    ),
                ));
            }
            match common.report {
                ReportFormat::Table => print_advanced_gate_table(&report),
                ReportFormat::Json => emit_report(&common, "diagnostics_advanced_gate", &report)?,
            }
            write_diagnostics_report_artifact(
                &common,
                "diagnostics_advanced_gate",
                &report,
                "diagnostics_advanced_gate_report.schema.json",
            )?;
            write_manifest(
                &common,
                "diagnostics_advanced_gate",
                &ReceiverConfig::default(),
                None,
                &report,
            )?;
        }
        DiagnosticsCommand::ArtifactInventory { common, run_dir } => {
            let _ = runtime_config_from_env(&common, None);
            let report = artifact_inventory_report(&run_dir)?;
            match common.report {
                ReportFormat::Table => print_artifact_inventory_table(&report),
                ReportFormat::Json => {
                    emit_report(&common, "diagnostics_artifact_inventory", &report)?
                }
            }
            write_diagnostics_report_artifact(
                &common,
                "diagnostics_artifact_inventory",
                &report,
                "diagnostics_artifact_inventory_report.schema.json",
            )?;
            write_manifest(
                &common,
                "diagnostics_artifact_inventory",
                &ReceiverConfig::default(),
                None,
                &report,
            )?;
        }
        DiagnosticsCommand::DebugPlan { common, run_dir } => {
            let _ = runtime_config_from_env(&common, None);
            let report = debug_plan_report(&run_dir)?;
            match common.report {
                ReportFormat::Table => print_debug_plan_table(&report),
                ReportFormat::Json => emit_report(&common, "diagnostics_debug_plan", &report)?,
            }
            write_diagnostics_report_artifact(
                &common,
                "diagnostics_debug_plan",
                &report,
                "diagnostics_debug_plan_report.schema.json",
            )?;
            write_manifest(
                &common,
                "diagnostics_debug_plan",
                &ReceiverConfig::default(),
                None,
                &report,
            )?;
        }
        DiagnosticsCommand::BenchmarkSummary { common, run_dir } => {
            let _ = runtime_config_from_env(&common, None);
            let report = benchmark_summary_report(&run_dir)?;
            match common.report {
                ReportFormat::Table => print_benchmark_summary_table(&report),
                ReportFormat::Json => {
                    emit_report(&common, "diagnostics_benchmark_summary", &report)?
                }
            }
            write_diagnostics_report_artifact(
                &common,
                "diagnostics_benchmark_summary",
                &report,
                "diagnostics_benchmark_summary_report.schema.json",
            )?;
            write_manifest(
                &common,
                "diagnostics_benchmark_summary",
                &ReceiverConfig::default(),
                None,
                &report,
            )?;
        }
        DiagnosticsCommand::MediumGate {
            common,
            run_dir,
            strict,
        } => {
            let _ = runtime_config_from_env(&common, None);
            let report = medium_gate_report(&run_dir)?;
            let passed = report
                .get("gate_passed")
                .and_then(|v| v.as_bool())
                .unwrap_or(false);
            if strict && !passed {
                return Err(classified_error(
                    CliErrorClass::UnsupportedScience,
                    format!("medium gate failed for run_dir={}", run_dir.display()),
                ));
            }
            match common.report {
                ReportFormat::Table => print_medium_gate_table(&report),
                ReportFormat::Json => emit_report(&common, "diagnostics_medium_gate", &report)?,
            }
            write_diagnostics_report_artifact(
                &common,
                "diagnostics_medium_gate",
                &report,
                "diagnostics_medium_gate_report.schema.json",
            )?;
            write_manifest(
                &common,
                "diagnostics_medium_gate",
                &ReceiverConfig::default(),
                None,
                &report,
            )?;
        }
        DiagnosticsCommand::OperatorStatus { common, run_dir } => {
            let _ = runtime_config_from_env(&common, None);
            let report = operator_status_report(&run_dir)?;
            match common.report {
                ReportFormat::Table => print_operator_status_table(&report),
                ReportFormat::Json => emit_report(&common, "diagnostics_operator_status", &report)?,
            }
            write_diagnostics_report_artifact(
                &common,
                "diagnostics_operator_status",
                &report,
                "diagnostics_operator_status_report.schema.json",
            )?;
            write_manifest(
                &common,
                "diagnostics_operator_status",
                &ReceiverConfig::default(),
                None,
                &report,
            )?;
        }
        DiagnosticsCommand::ChannelSummary { common, run_dir } => {
            let _ = runtime_config_from_env(&common, None);
            let report = channel_summary_report(&run_dir)?;
            match common.report {
                ReportFormat::Table => print_channel_summary_table(&report),
                ReportFormat::Json => emit_report(&common, "diagnostics_channel_summary", &report)?,
            }
            write_diagnostics_report_artifact(
                &common,
                "diagnostics_channel_summary",
                &report,
                "diagnostics_channel_summary_report.schema.json",
            )?;
            write_manifest(
                &common,
                "diagnostics_channel_summary",
                &ReceiverConfig::default(),
                None,
                &report,
            )?;
        }
        DiagnosticsCommand::ExportBundle {
            common,
            run_dir,
            out_dir,
        } => {
            let _ = runtime_config_from_env(&common, None);
            let report = export_bundle_report(&run_dir, out_dir.as_ref())?;
            match common.report {
                ReportFormat::Table => print_export_bundle_table(&report),
                ReportFormat::Json => emit_report(&common, "diagnostics_export_bundle", &report)?,
            }
            write_diagnostics_report_artifact(
                &common,
                "diagnostics_export_bundle",
                &report,
                "diagnostics_export_bundle_report.schema.json",
            )?;
            write_manifest(
                &common,
                "diagnostics_export_bundle",
                &ReceiverConfig::default(),
                None,
                &report,
            )?;
        }
        DiagnosticsCommand::MachineCatalog { common } => {
            let _ = runtime_config_from_env(&common, None);
            let report = machine_catalog_report();
            match common.report {
                ReportFormat::Table => print_machine_catalog_table(&report),
                ReportFormat::Json => emit_report(&common, "diagnostics_machine_catalog", &report)?,
            }
            write_diagnostics_report_artifact(
                &common,
                "diagnostics_machine_catalog",
                &report,
                "diagnostics_machine_catalog_report.schema.json",
            )?;
            write_manifest(
                &common,
                "diagnostics_machine_catalog",
                &ReceiverConfig::default(),
                None,
                &report,
            )?;
        }
        DiagnosticsCommand::ApiParity { common } => {
            let _ = runtime_config_from_env(&common, None);
            let report = api_parity_report();
            match common.report {
                ReportFormat::Table => print_api_parity_table(&report),
                ReportFormat::Json => emit_report(&common, "diagnostics_api_parity", &report)?,
            }
            write_diagnostics_report_artifact(
                &common,
                "diagnostics_api_parity",
                &report,
                "diagnostics_api_parity_report.schema.json",
            )?;
            write_manifest(
                &common,
                "diagnostics_api_parity",
                &ReceiverConfig::default(),
                None,
                &report,
            )?;
        }
        DiagnosticsCommand::ExpertGuide { common } => {
            let _ = runtime_config_from_env(&common, None);
            let report = expert_guide_report();
            match common.report {
                ReportFormat::Table => print_expert_guide_table(&report),
                ReportFormat::Json => emit_report(&common, "diagnostics_expert_guide", &report)?,
            }
            write_diagnostics_report_artifact(
                &common,
                "diagnostics_expert_guide",
                &report,
                "diagnostics_expert_guide_report.schema.json",
            )?;
            write_manifest(
                &common,
                "diagnostics_expert_guide",
                &ReceiverConfig::default(),
                None,
                &report,
            )?;
        }
        DiagnosticsCommand::HistoryBrowse {
            common,
            root_dir,
            limit,
        } => {
            let _ = runtime_config_from_env(&common, None);
            let report = history_browse_report(&root_dir, limit)?;
            match common.report {
                ReportFormat::Table => print_history_browse_table(&report),
                ReportFormat::Json => emit_report(&common, "diagnostics_history_browse", &report)?,
            }
            write_diagnostics_report_artifact(
                &common,
                "diagnostics_history_browse",
                &report,
                "diagnostics_history_browse_report.schema.json",
            )?;
            write_manifest(
                &common,
                "diagnostics_history_browse",
                &ReceiverConfig::default(),
                None,
                &report,
            )?;
        }
        DiagnosticsCommand::RouteExplain { common, topic } => {
            let _ = runtime_config_from_env(&common, None);
            let report = route_explain_report(topic);
            match common.report {
                ReportFormat::Table => print_route_explain_table(&report),
                ReportFormat::Json => emit_report(&common, "diagnostics_route_explain", &report)?,
            }
            write_diagnostics_report_artifact(
                &common,
                "diagnostics_route_explain",
                &report,
                "diagnostics_route_explain_report.schema.json",
            )?;
            write_manifest(
                &common,
                "diagnostics_route_explain",
                &ReceiverConfig::default(),
                None,
                &report,
            )?;
        }
        DiagnosticsCommand::OperatorWorkflow { common, profile } => {
            let _ = runtime_config_from_env(&common, None);
            let report = operator_workflow_report(profile);
            match common.report {
                ReportFormat::Table => print_operator_workflow_table(&report),
                ReportFormat::Json => {
                    emit_report(&common, "diagnostics_operator_workflow", &report)?
                }
            }
            write_diagnostics_report_artifact(
                &common,
                "diagnostics_operator_workflow",
                &report,
                "diagnostics_operator_workflow_report.schema.json",
            )?;
            write_manifest(
                &common,
                "diagnostics_operator_workflow",
                &ReceiverConfig::default(),
                None,
                &report,
            )?;
        }
        DiagnosticsCommand::OperatorErgonomics { common, run_dir } => {
            let _ = runtime_config_from_env(&common, None);
            let report = operator_ergonomics_report(&run_dir)?;
            match common.report {
                ReportFormat::Table => print_operator_ergonomics_table(&report),
                ReportFormat::Json => {
                    emit_report(&common, "diagnostics_operator_ergonomics", &report)?
                }
            }
            write_diagnostics_report_artifact(
                &common,
                "diagnostics_operator_ergonomics",
                &report,
                "diagnostics_operator_ergonomics_report.schema.json",
            )?;
            write_manifest(
                &common,
                "diagnostics_operator_ergonomics",
                &ReceiverConfig::default(),
                None,
                &report,
            )?;
        }
        DiagnosticsCommand::AuditTrail { common, run_dir } => {
            let _ = runtime_config_from_env(&common, None);
            let report = audit_trail_report(&run_dir)?;
            match common.report {
                ReportFormat::Table => print_audit_trail_table(&report),
                ReportFormat::Json => emit_report(&common, "diagnostics_audit_trail", &report)?,
            }
            write_diagnostics_report_artifact(
                &common,
                "diagnostics_audit_trail",
                &report,
                "diagnostics_audit_trail_report.schema.json",
            )?;
            write_manifest(
                &common,
                "diagnostics_audit_trail",
                &ReceiverConfig::default(),
                None,
                &report,
            )?;
        }
        DiagnosticsCommand::DependencyTrace { common, run_dir } => {
            let _ = runtime_config_from_env(&common, None);
            let report = dependency_trace_report(&run_dir)?;
            match common.report {
                ReportFormat::Table => print_dependency_trace_table(&report),
                ReportFormat::Json => {
                    emit_report(&common, "diagnostics_dependency_trace", &report)?
                }
            }
            write_diagnostics_report_artifact(
                &common,
                "diagnostics_dependency_trace",
                &report,
                "diagnostics_dependency_trace_report.schema.json",
            )?;
            write_manifest(
                &common,
                "diagnostics_dependency_trace",
                &ReceiverConfig::default(),
                None,
                &report,
            )?;
        }
    }

    Ok(())
}

fn operator_map_report() -> serde_json::Value {
    serde_json::json!({
        "schema_version": 1,
        "operator_map": [
            {
                "workflow": "run",
                "goal": "execute receiver pipeline",
                "command": "bijux gnss run --dataset <id> --config <profile.toml>"
            },
            {
                "workflow": "diagnose",
                "goal": "inspect replay scope, cache behavior, and artifact integrity",
                "command": "bijux gnss diagnostics explain --run-dir <run_dir>"
            },
            {
                "workflow": "replay_audit",
                "goal": "prove deterministic match or classify drift",
                "command": "bijux gnss diagnostics replay-audit --baseline-run-dir <run_a> --candidate-run-dir <run_b>"
            },
            {
                "workflow": "compare",
                "goal": "compare quality deltas and reproducibility evidence",
                "command": "bijux gnss diagnostics compare --baseline-run-dir <run_a> --candidate-run-dir <run_b>"
            },
            {
                "workflow": "export_bundle",
                "goal": "prepare reproducible review bundle for triage",
                "command": "bijux gnss diagnostics export-bundle --run-dir <run_dir>"
            }
        ]
    })
}

fn layered_report(critical: Vec<String>, details: serde_json::Value) -> serde_json::Value {
    serde_json::json!({
        "critical": critical,
        "details": details
    })
}

fn summarize_critical_entries(
    entries: &[bijux_gnss_infra::api::core::DiagnosticSummaryEntry],
) -> Vec<String> {
    let mut critical = Vec::new();
    for entry in entries {
        let severity = format!("{:?}", entry.severity).to_lowercase();
        if severity == "error" || severity == "fatal" {
            critical.push(format!("{}:{}:count={}", severity, entry.code, entry.count));
        }
    }
    critical
}

fn workflow_map_report() -> serde_json::Value {
    serde_json::json!({
        "schema_version": 1,
        "workflow": [
            {
                "stage": "run",
                "command": "bijux gnss run --dataset <id> --config <profile.toml>",
                "artifacts": ["manifest.json", "run_report.json", "artifacts/"]
            },
            {
                "stage": "inspect",
                "command": "bijux gnss inspect --dataset <id> --report json",
                "artifacts": ["inspect_report.json", "summary.json"]
            },
            {
                "stage": "validate",
                "command": "bijux gnss validate --dataset <id> --eph <eph.json> --reference <ref.jsonl>",
                "artifacts": ["validation_report.json", "validation_evidence_bundle.json"]
            },
            {
                "stage": "diagnose",
                "command": "bijux gnss diagnostics explain --run-dir <run_dir>",
                "artifacts": ["diagnostics_explain_report.json", "summary.json"]
            },
            {
                "stage": "replay",
                "command": "bijux gnss diagnostics verify-repro --run-dir <run_dir>",
                "artifacts": ["diagnostics_verify_repro_report.json", "summary.json"]
            },
            {
                "stage": "compare",
                "command": "bijux gnss diff --run-a <run_a> --run-b <run_b>",
                "artifacts": ["compare summary json on stdout"]
            }
        ]
    })
}

fn print_operator_map_table(report: &serde_json::Value) {
    println!("operator workflow map");
    if let Some(rows) = report.get("operator_map").and_then(|v| v.as_array()) {
        for row in rows {
            let workflow = row.get("workflow").and_then(|v| v.as_str()).unwrap_or("unknown");
            let command = row.get("command").and_then(|v| v.as_str()).unwrap_or("unknown");
            println!("{workflow}\t{command}");
        }
    }
}

fn write_diagnostics_report_artifact(
    common: &CommonArgs,
    command: &str,
    report: &serde_json::Value,
    schema_name: &str,
) -> Result<PathBuf> {
    let out_dir = artifacts_dir(common, command, None)?;
    let report_path = out_dir.join("report.json");
    fs::write(&report_path, serde_json::to_string_pretty(report)?)?;
    let schema = schema_path(schema_name);
    if schema.exists() {
        validate_json_schema(&schema, &report_path, false)?;
    }
    Ok(report_path)
}

fn print_workflow_table(report: &serde_json::Value) {
    println!("GNSS workflow map");
    if let Some(stages) = report.get("workflow").and_then(|v| v.as_array()) {
        for stage in stages {
            let label = stage.get("stage").and_then(|v| v.as_str()).unwrap_or("unknown");
            let command = stage
                .get("command")
                .and_then(|v| v.as_str())
                .unwrap_or("unknown");
            println!("{label}\t{command}");
        }
    }
}

fn print_diagnostics_summary_table(entries: Option<&Vec<serde_json::Value>>, top: usize) {
    println!("Diagnostics summary (top {}):", top);
    if let Some(items) = entries {
        for entry in items.iter().take(top.max(1)) {
            let code = entry.get("code").and_then(|v| v.as_str()).unwrap_or("unknown");
            let severity = entry
                .get("severity")
                .and_then(|v| v.as_str())
                .unwrap_or("unknown");
            let count = entry.get("count").and_then(|v| v.as_u64()).unwrap_or(0);
            println!("{code}\t{severity}\tcount={count}");
        }
    }
}

fn print_diagnostics_explain_table(report: &serde_json::Value) {
    let command = report
        .get("replay_scope")
        .and_then(|v| v.get("command"))
        .and_then(|v| v.as_str())
        .unwrap_or("unknown");
    let deterministic = report
        .get("replay_scope")
        .and_then(|v| v.get("deterministic"))
        .and_then(|v| v.as_bool())
        .unwrap_or(false);
    let artifacts = report
        .get("artifact_integrity")
        .and_then(|v| v.get("artifact_files_scanned"))
        .and_then(|v| v.as_u64())
        .unwrap_or(0);
    let corrupted = report
        .get("artifact_integrity")
        .and_then(|v| v.get("corrupted_files"))
        .and_then(|v| v.as_array())
        .map_or(0usize, Vec::len);
    println!("command\t{command}");
    println!("deterministic\t{deterministic}");
    println!("artifact_files\t{artifacts}");
    println!("corrupted_files\t{corrupted}");
}

fn print_verify_repro_table(report: &serde_json::Value) {
    let audit_ok = report
        .get("audit_ok")
        .and_then(|v| v.as_bool())
        .unwrap_or(false);
    let fingerprint = report
        .get("replay_fingerprint")
        .and_then(|v| v.as_str())
        .unwrap_or("missing");
    let issue_count = report
        .get("issues")
        .and_then(|v| v.as_array())
        .map_or(0usize, Vec::len);
    println!("audit_ok\t{audit_ok}");
    println!("issues\t{issue_count}");
    println!("replay_fingerprint\t{fingerprint}");
}

fn print_compare_report_table(report: &serde_json::Value) {
    let fingerprint_match = report
        .get("reproducibility")
        .and_then(|v| v.get("fingerprint_match"))
        .and_then(|v| v.as_bool())
        .unwrap_or(false);
    let baseline_audit = report
        .get("reproducibility")
        .and_then(|v| v.get("baseline_audit_ok"))
        .and_then(|v| v.as_bool())
        .unwrap_or(false);
    let candidate_audit = report
        .get("reproducibility")
        .and_then(|v| v.get("candidate_audit_ok"))
        .and_then(|v| v.as_bool())
        .unwrap_or(false);
    let rms_delta = report
        .get("quality")
        .and_then(|v| v.get("mean_rms_delta_m"))
        .and_then(|v| v.as_f64())
        .unwrap_or(0.0);
    let cn0_delta = report
        .get("quality")
        .and_then(|v| v.get("mean_cn0_delta_dbhz"))
        .and_then(|v| v.as_f64())
        .unwrap_or(0.0);
    println!("fingerprint_match\t{fingerprint_match}");
    println!("baseline_audit_ok\t{baseline_audit}");
    println!("candidate_audit_ok\t{candidate_audit}");
    println!("mean_rms_delta_m\t{rms_delta:.6}");
    println!("mean_cn0_delta_dbhz\t{cn0_delta:.6}");
}

fn print_replay_audit_table(report: &serde_json::Value) {
    let class = report
        .get("classification")
        .and_then(|v| v.as_str())
        .unwrap_or("unknown");
    let baseline_command = report
        .get("baseline_command")
        .and_then(|v| v.as_str())
        .unwrap_or("unknown");
    let candidate_command = report
        .get("candidate_command")
        .and_then(|v| v.as_str())
        .unwrap_or("unknown");
    let reasons = report
        .get("reasons")
        .and_then(|v| v.as_array())
        .map(|vals| {
            vals.iter()
                .filter_map(|v| v.as_str())
                .collect::<Vec<_>>()
                .join(",")
        })
        .unwrap_or_else(String::new);
    println!("classification\t{class}");
    println!("baseline_command\t{baseline_command}");
    println!("candidate_command\t{candidate_command}");
    println!("reasons\t{reasons}");
}

fn print_advanced_gate_table(report: &serde_json::Value) {
    let mode = report
        .get("mode")
        .and_then(|v| v.as_str())
        .unwrap_or("unknown");
    let passed = report
        .get("gate_passed")
        .and_then(|v| v.as_bool())
        .unwrap_or(false);
    let maturity = report
        .get("support")
        .and_then(|v| v.get("maturity"))
        .and_then(|v| v.as_str())
        .unwrap_or("unknown");
    let evidence_supported = report
        .get("evidence")
        .and_then(|v| v.get("claim_guard_supported"))
        .and_then(|v| v.as_bool())
        .unwrap_or(false);
    println!("mode\t{mode}");
    println!("gate_passed\t{passed}");
    println!("maturity\t{maturity}");
    println!("claim_guard_supported\t{evidence_supported}");
}

fn print_artifact_inventory_table(report: &serde_json::Value) {
    println!("artifact inventory");
    if let Some(groups) = report.get("groups").and_then(|v| v.as_object()) {
        for (group, payload) in groups {
            let files = payload
                .get("file_count")
                .and_then(|v| v.as_u64())
                .unwrap_or(0);
            println!("{group}\tfiles={files}");
        }
    }
}

fn print_debug_plan_table(report: &serde_json::Value) {
    println!("debug plan");
    if let Some(stages) = report.get("stages").and_then(|v| v.as_array()) {
        for stage in stages {
            let name = stage.get("stage").and_then(|v| v.as_str()).unwrap_or("unknown");
            let check = stage
                .get("recommended_check")
                .and_then(|v| v.as_str())
                .unwrap_or("unknown");
            println!("{name}\t{check}");
        }
    }
}

fn print_benchmark_summary_table(report: &serde_json::Value) {
    let nav_epochs = report
        .get("summary")
        .and_then(|v| v.get("nav_epochs"))
        .and_then(|v| v.as_u64())
        .unwrap_or(0);
    let obs_epochs = report
        .get("summary")
        .and_then(|v| v.get("obs_epochs"))
        .and_then(|v| v.as_u64())
        .unwrap_or(0);
    println!("obs_epochs\t{obs_epochs}");
    println!("nav_epochs\t{nav_epochs}");
}

fn print_medium_gate_table(report: &serde_json::Value) {
    let passed = report
        .get("gate_passed")
        .and_then(|v| v.as_bool())
        .unwrap_or(false);
    println!("gate_passed\t{passed}");
    let reasons = report
        .get("reasons")
        .and_then(|v| v.as_array())
        .map(|rows| {
            rows.iter()
                .filter_map(|v| v.as_str())
                .collect::<Vec<_>>()
                .join(",")
        })
        .unwrap_or_default();
    println!("reasons\t{reasons}");
}

fn print_operator_status_table(report: &serde_json::Value) {
    let mode = report
        .get("status")
        .and_then(|v| v.get("run_state"))
        .and_then(|v| v.as_str())
        .unwrap_or("unknown");
    let quality = report
        .get("status")
        .and_then(|v| v.get("quality_state"))
        .and_then(|v| v.as_str())
        .unwrap_or("unknown");
    let evidence = report
        .get("status")
        .and_then(|v| v.get("evidence_state"))
        .and_then(|v| v.as_str())
        .unwrap_or("unknown");
    println!("run_state\t{mode}");
    println!("quality_state\t{quality}");
    println!("evidence_state\t{evidence}");
}

fn print_channel_summary_table(report: &serde_json::Value) {
    let channels = report
        .get("summary")
        .and_then(|v| v.get("channel_count"))
        .and_then(|v| v.as_u64())
        .unwrap_or(0);
    let epochs = report
        .get("summary")
        .and_then(|v| v.get("obs_epochs"))
        .and_then(|v| v.as_u64())
        .unwrap_or(0);
    println!("channel_count\t{channels}");
    println!("obs_epochs\t{epochs}");
}

fn print_export_bundle_table(report: &serde_json::Value) {
    let path = report
        .get("bundle_dir")
        .and_then(|v| v.as_str())
        .unwrap_or("unknown");
    let files = report
        .get("file_count")
        .and_then(|v| v.as_u64())
        .unwrap_or(0);
    println!("bundle_dir\t{path}");
    println!("file_count\t{files}");
}

fn print_machine_catalog_table(report: &serde_json::Value) {
    println!("machine-readable report catalog");
    if let Some(rows) = report.get("reports").and_then(|v| v.as_array()) {
        for row in rows {
            let name = row.get("name").and_then(|v| v.as_str()).unwrap_or("unknown");
            let schema = row
                .get("schema")
                .and_then(|v| v.as_str())
                .unwrap_or("unknown");
            println!("{name}\t{schema}");
        }
    }
}

fn print_api_parity_table(report: &serde_json::Value) {
    println!("cli/api parity");
    if let Some(rows) = report.get("workflows").and_then(|v| v.as_array()) {
        for row in rows {
            let workflow = row
                .get("workflow")
                .and_then(|v| v.as_str())
                .unwrap_or("unknown");
            let cli = row
                .get("cli_supported")
                .and_then(|v| v.as_bool())
                .unwrap_or(false);
            let api = row
                .get("api_supported")
                .and_then(|v| v.as_bool())
                .unwrap_or(false);
            println!("{workflow}\tcli={cli}\tapi={api}");
        }
    }
}

fn print_expert_guide_table(report: &serde_json::Value) {
    println!("expert guide");
    if let Some(flows) = report.get("flows").and_then(|v| v.as_array()) {
        for flow in flows {
            let name = flow.get("name").and_then(|v| v.as_str()).unwrap_or("unknown");
            let first = flow
                .get("steps")
                .and_then(|v| v.as_array())
                .and_then(|rows| rows.first())
                .and_then(|v| v.as_str())
                .unwrap_or("no-steps");
            println!("{name}\t{first}");
        }
    }
}

fn print_history_browse_table(report: &serde_json::Value) {
    println!("history browse");
    if let Some(runs) = report.get("runs").and_then(|v| v.as_array()) {
        for run in runs {
            let dir = run.get("run_dir").and_then(|v| v.as_str()).unwrap_or("unknown");
            let cmd = run.get("command").and_then(|v| v.as_str()).unwrap_or("unknown");
            let dataset = run
                .get("dataset_id")
                .and_then(|v| v.as_str())
                .unwrap_or("unknown");
            println!("{dir}\t{cmd}\t{dataset}");
        }
    }
}

fn print_route_explain_table(report: &serde_json::Value) {
    let topic = report
        .get("topic")
        .and_then(|v| v.as_str())
        .unwrap_or("unknown");
    println!("route topic\t{topic}");
    if let Some(steps) = report.get("steps").and_then(|v| v.as_array()) {
        for step in steps {
            if let Some(cmd) = step.get("command").and_then(|v| v.as_str()) {
                println!("{cmd}");
            }
        }
    }
}

fn print_operator_workflow_table(report: &serde_json::Value) {
    let profile = report
        .get("profile")
        .and_then(|v| v.as_str())
        .unwrap_or("unknown");
    println!("operator workflow\t{profile}");
    if let Some(steps) = report.get("steps").and_then(|v| v.as_array()) {
        for step in steps {
            let label = step
                .get("label")
                .and_then(|v| v.as_str())
                .unwrap_or("unknown");
            let command = step
                .get("command")
                .and_then(|v| v.as_str())
                .unwrap_or("unknown");
            println!("{label}\t{command}");
        }
    }
}

fn print_operator_ergonomics_table(report: &serde_json::Value) {
    let score = report
        .get("ergonomics_score")
        .and_then(|v| v.as_f64())
        .unwrap_or(0.0);
    let gate = report
        .get("rigor")
        .and_then(|v| v.get("medium_gate_passed"))
        .and_then(|v| v.as_bool())
        .unwrap_or(false);
    let evidence = report
        .get("rigor")
        .and_then(|v| v.get("evidence_state"))
        .and_then(|v| v.as_str())
        .unwrap_or("unknown");
    println!("ergonomics_score\t{score:.2}");
    println!("medium_gate_passed\t{gate}");
    println!("evidence_state\t{evidence}");
}

fn print_audit_trail_table(report: &serde_json::Value) {
    let replay = report
        .get("replay_controls")
        .and_then(|v| v.get("deterministic"))
        .and_then(|v| v.as_bool())
        .unwrap_or(false);
    let overrides = report
        .get("override_events")
        .and_then(|v| v.as_array())
        .map_or(0usize, Vec::len);
    let exceptions = report
        .get("policy_exceptions")
        .and_then(|v| v.as_array())
        .map_or(0usize, Vec::len);
    println!("deterministic_replay\t{replay}");
    println!("override_events\t{overrides}");
    println!("policy_exceptions\t{exceptions}");
}

fn print_dependency_trace_table(report: &serde_json::Value) {
    let toolchain = report
        .get("tooling")
        .and_then(|v| v.get("toolchain"))
        .and_then(|v| v.as_str())
        .unwrap_or("unknown");
    let correction_rows = report
        .get("corrections")
        .and_then(|v| v.get("rows"))
        .and_then(|v| v.as_u64())
        .unwrap_or(0);
    let env_refs = report
        .get("environment")
        .and_then(|v| v.get("reference_count"))
        .and_then(|v| v.as_u64())
        .unwrap_or(0);
    println!("toolchain\t{toolchain}");
    println!("correction_rows\t{correction_rows}");
    println!("environment_references\t{env_refs}");
}

fn summarize_run_diagnostics(run_dir: &Path) -> Result<Vec<DiagnosticEvent>> {
    ensure_run_dir_exists(run_dir)?;
    let artifacts_dir = run_dir.join("artifacts");
    if !artifacts_dir.exists() {
        return Err(classified_error(
            CliErrorClass::OperatorMisconfiguration,
            format!("artifacts directory not found: {}", artifacts_dir.display()),
        ));
    }
    let mut events = Vec::new();
    for entry in fs::read_dir(&artifacts_dir)? {
        let entry = entry?;
        let path = entry.path();
        if !path.is_file() {
            continue;
        }
        let Some(ext) = path.extension().and_then(|s| s.to_str()) else {
            continue;
        };
        if ext != "json" && ext != "jsonl" {
            continue;
        }
        if let Ok(result) = artifact_validate(&path, None, false) {
            events.extend(result.diagnostics);
        }
    }
    Ok(events)
}

fn explain_run_scope(run_dir: &Path) -> Result<serde_json::Value> {
    ensure_run_dir_exists(run_dir)?;
    let manifest_path = run_dir.join("manifest.json");
    let run_manifest: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(&manifest_path)?)
        .map_err(|err| {
            classified_error(
                CliErrorClass::InternalFault,
                format!("failed parsing manifest {}: {err}", manifest_path.display()),
            )
        })?;

    let artifacts_dir = run_dir.join("artifacts");
    let mut identity_coverage = serde_json::Map::new();
    let mut total_artifact_files = 0usize;
    let mut total_lines = 0usize;
    let mut corrupted_files = Vec::new();
    let mut cache_events = serde_json::Map::new();
    let mut cache_event_total = 0usize;

    if artifacts_dir.exists() {
        for entry in std::fs::read_dir(&artifacts_dir)? {
            let entry = entry?;
            let path = entry.path();
            if !path.is_file() {
                continue;
            }
            let Some(ext) = path.extension().and_then(|s| s.to_str()) else {
                continue;
            };
            if ext != "json" && ext != "jsonl" {
                continue;
            }
            total_artifact_files = total_artifact_files.saturating_add(1);
            let data = std::fs::read_to_string(&path)?;
            let file_name =
                path.file_name().and_then(|name| name.to_str()).unwrap_or("unknown").to_string();
            let mut has_artifact_id = false;
            let mut has_epoch_id = false;
            let mut has_source_epoch_id = false;
            let mut parsed_lines = 0usize;

            if ext == "jsonl" {
                for (idx, line) in data.lines().enumerate() {
                    if line.trim().is_empty() {
                        continue;
                    }
                    parsed_lines = parsed_lines.saturating_add(1);
                    total_lines = total_lines.saturating_add(1);
                    let parsed: serde_json::Value = match serde_json::from_str(line) {
                        Ok(v) => v,
                        Err(err) => {
                            corrupted_files.push(serde_json::json!({
                                "file": file_name,
                                "line": idx + 1,
                                "error": err.to_string()
                            }));
                            break;
                        }
                    };
                    let payload = parsed.get("payload").unwrap_or(&parsed);
                    has_artifact_id |= payload.get("artifact_id").is_some();
                    has_epoch_id |= payload.get("epoch_id").is_some();
                    has_source_epoch_id |= payload.get("source_observation_epoch_id").is_some();
                }
            } else if ext == "json" {
                total_lines = total_lines.saturating_add(1);
                if let Err(err) = serde_json::from_str::<serde_json::Value>(&data) {
                    corrupted_files.push(serde_json::json!({
                        "file": file_name,
                        "line": 1,
                        "error": err.to_string()
                    }));
                } else {
                    parsed_lines = 1;
                }
            }

            identity_coverage.insert(
                file_name.clone(),
                serde_json::json!({
                    "parsed_entries": parsed_lines,
                    "has_artifact_id": has_artifact_id,
                    "has_epoch_id": has_epoch_id,
                    "has_source_observation_epoch_id": has_source_epoch_id
                }),
            );
        }
    }

    let trace_path = run_dir.join("trace.ndjson");
    if trace_path.exists() {
        let trace = std::fs::read_to_string(trace_path)?;
        for line in trace.lines() {
            if line.trim().is_empty() {
                continue;
            }
            let Ok(row) = serde_json::from_str::<serde_json::Value>(line) else {
                continue;
            };
            let Some(name) = row.get("name").and_then(|v| v.as_str()) else {
                continue;
            };
            if name != "acquisition_code_fft_cache_hit" && name != "acquisition_code_fft_cache_miss"
            {
                continue;
            }
            cache_event_total = cache_event_total.saturating_add(1);
            if name == "acquisition_code_fft_cache_miss" {
                if let Some(reason) = row
                    .get("fields")
                    .and_then(|v| v.as_array())
                    .and_then(|fields| {
                        fields.iter().find_map(|entry| {
                            let key = entry.get(0).and_then(|v| v.as_str())?;
                            if key == "reason" {
                                entry.get(1).and_then(|v| v.as_str()).map(|s| s.to_string())
                            } else {
                                None
                            }
                        })
                    })
                {
                    let next =
                        cache_events.get(&reason).and_then(|v| v.as_u64()).unwrap_or(0) + 1;
                    cache_events.insert(reason, serde_json::Value::from(next));
                }
            }
        }
    }

    let metrics_summary_path = run_dir.join("metrics_summary.json");
    let metrics_summary = if metrics_summary_path.exists() {
        serde_json::from_str::<serde_json::Value>(&std::fs::read_to_string(&metrics_summary_path)?)
            .unwrap_or(serde_json::Value::Null)
    } else {
        serde_json::Value::Null
    };

    Ok(serde_json::json!({
        "schema_version": 1,
        "run_dir": run_dir.display().to_string(),
        "manifest_path": manifest_path.display().to_string(),
        "replay_scope": {
            "command": run_manifest.get("command").cloned().unwrap_or(serde_json::Value::Null),
            "dataset_id": run_manifest.get("dataset_id").cloned().unwrap_or(serde_json::Value::Null),
            "config_hash": run_manifest.get("config_hash").cloned().unwrap_or(serde_json::Value::Null),
            "toolchain": run_manifest.get("toolchain").cloned().unwrap_or(serde_json::Value::Null),
            "features": run_manifest.get("features").cloned().unwrap_or(serde_json::Value::Null),
            "deterministic": run_manifest.get("replay_scope").and_then(|v| v.get("deterministic")).cloned().unwrap_or(serde_json::Value::Null),
            "resume": run_manifest.get("replay_scope").and_then(|v| v.get("resume")).cloned().unwrap_or(serde_json::Value::Null),
            "explicit_output_dir": run_manifest.get("replay_scope").and_then(|v| v.get("explicit_output_dir")).cloned().unwrap_or(serde_json::Value::Null)
        },
        "front_end_provenance": run_manifest
            .get("front_end_provenance")
            .cloned()
            .unwrap_or(serde_json::Value::Null),
        "artifact_integrity": {
            "artifact_files_scanned": total_artifact_files,
            "entries_scanned": total_lines,
            "corrupted_files": corrupted_files
        },
        "artifact_identity_coverage": identity_coverage,
        "cache_behavior": {
            "cache_events_seen": cache_event_total,
            "miss_reason_counts": cache_events,
            "metrics_summary": metrics_summary
        }
    }))
}

fn verify_repro_bundle(run_dir: &Path) -> Result<serde_json::Value> {
    ensure_run_dir_exists(run_dir)?;
    let manifest_path = run_dir.join("manifest.json");
    let manifest_data = std::fs::read_to_string(&manifest_path)?;
    let manifest: serde_json::Value = serde_json::from_str(&manifest_data).map_err(|err| {
        classified_error(
            CliErrorClass::InternalFault,
            format!("failed parsing manifest {}: {err}", manifest_path.display()),
        )
    })?;

    let run_report_path = run_dir.join("run_report.json");
    let run_report_data = if run_report_path.exists() {
        Some(std::fs::read_to_string(&run_report_path)?)
    } else {
        None
    };

    let artifacts_dir = run_dir.join("artifacts");
    let mut artifact_hashes = serde_json::Map::new();
    let mut issues = Vec::new();

    if !artifacts_dir.exists() {
        issues.push("artifacts_directory_missing".to_string());
    } else {
        for entry in std::fs::read_dir(&artifacts_dir)? {
            let entry = entry?;
            let path = entry.path();
            if !path.is_file() {
                continue;
            }
            let file_name = path.file_name().and_then(|n| n.to_str()).unwrap_or("unknown");
            let bytes = std::fs::read(&path)?;
            let hash = sha256_hex(&bytes);
            artifact_hashes.insert(file_name.to_string(), serde_json::Value::String(hash));
        }
    }

    if artifact_hashes.is_empty() {
        issues.push("artifact_hash_set_empty".to_string());
    }

    let mut fingerprint_payload = serde_json::Map::new();
    fingerprint_payload.insert(
        "command".to_string(),
        manifest.get("command").cloned().unwrap_or(serde_json::Value::Null),
    );
    fingerprint_payload.insert(
        "config_hash".to_string(),
        manifest.get("config_hash").cloned().unwrap_or(serde_json::Value::Null),
    );
    fingerprint_payload.insert(
        "dataset_id".to_string(),
        manifest.get("dataset_id").cloned().unwrap_or(serde_json::Value::Null),
    );
    fingerprint_payload.insert(
        "replay_scope".to_string(),
        manifest
            .get("replay_scope")
            .cloned()
            .unwrap_or(serde_json::Value::Null),
    );
    fingerprint_payload.insert(
        "front_end_provenance".to_string(),
        manifest
            .get("front_end_provenance")
            .cloned()
            .unwrap_or(serde_json::Value::Null),
    );
    fingerprint_payload.insert(
        "layout_schema_version".to_string(),
        manifest
            .get("layout_schema_version")
            .cloned()
            .unwrap_or(serde_json::Value::Null),
    );
    let fingerprint_json = serde_json::to_vec(&fingerprint_payload)?;
    let replay_fingerprint = sha256_hex(&fingerprint_json);

    let manifest_hash = sha256_hex(manifest_data.as_bytes());
    let run_report_hash = run_report_data.as_deref().map(|text| sha256_hex(text.as_bytes()));
    let audit_ok = issues.is_empty();

    Ok(serde_json::json!({
        "schema_version": 1,
        "run_dir": run_dir.display().to_string(),
        "audit_ok": audit_ok,
        "issues": issues,
        "replay_fingerprint": replay_fingerprint,
        "manifest_sha256": manifest_hash,
        "run_report_sha256": run_report_hash,
        "artifact_sha256": artifact_hashes
    }))
}

fn compare_run_evidence(baseline_run_dir: &Path, candidate_run_dir: &Path) -> Result<serde_json::Value> {
    ensure_run_dir_exists(baseline_run_dir)?;
    ensure_run_dir_exists(candidate_run_dir)?;
    let baseline_repro = verify_repro_bundle(baseline_run_dir)?;
    let candidate_repro = verify_repro_bundle(candidate_run_dir)?;
    let baseline_explain = explain_run_scope(baseline_run_dir)?;
    let candidate_explain = explain_run_scope(candidate_run_dir)?;
    let quality = diff_runs(baseline_run_dir, candidate_run_dir)?;

    let baseline_corrupted = baseline_explain
        .get("artifact_integrity")
        .and_then(|v| v.get("corrupted_files"))
        .and_then(|v| v.as_array())
        .map_or(0usize, Vec::len);
    let candidate_corrupted = candidate_explain
        .get("artifact_integrity")
        .and_then(|v| v.get("corrupted_files"))
        .and_then(|v| v.as_array())
        .map_or(0usize, Vec::len);

    Ok(serde_json::json!({
        "schema_version": 1,
        "baseline_run_dir": baseline_run_dir.display().to_string(),
        "candidate_run_dir": candidate_run_dir.display().to_string(),
        "reproducibility": {
            "fingerprint_match": baseline_repro.get("replay_fingerprint") == candidate_repro.get("replay_fingerprint"),
            "baseline_audit_ok": baseline_repro.get("audit_ok").and_then(|v| v.as_bool()).unwrap_or(false),
            "candidate_audit_ok": candidate_repro.get("audit_ok").and_then(|v| v.as_bool()).unwrap_or(false),
            "baseline_replay_fingerprint": baseline_repro.get("replay_fingerprint").cloned().unwrap_or(serde_json::Value::Null),
            "candidate_replay_fingerprint": candidate_repro.get("replay_fingerprint").cloned().unwrap_or(serde_json::Value::Null),
            "baseline_issues": baseline_repro.get("issues").cloned().unwrap_or_else(|| serde_json::json!([])),
            "candidate_issues": candidate_repro.get("issues").cloned().unwrap_or_else(|| serde_json::json!([]))
        },
        "artifact_integrity": {
            "baseline_corrupted_files": baseline_corrupted,
            "candidate_corrupted_files": candidate_corrupted,
            "corrupted_file_delta": candidate_corrupted as isize - baseline_corrupted as isize
        },
        "quality": {
            "mean_rms_delta_m": quality.get("mean_rms_m").and_then(|v| v.get("delta")).and_then(|v| v.as_f64()).unwrap_or(0.0),
            "mean_cn0_delta_dbhz": quality.get("mean_cn0_dbhz").and_then(|v| v.get("delta")).and_then(|v| v.as_f64()).unwrap_or(0.0),
            "convergence_epoch_baseline": quality.get("convergence_epoch_idx").and_then(|v| v.get("run_a")).cloned().unwrap_or(serde_json::Value::Null),
            "convergence_epoch_candidate": quality.get("convergence_epoch_idx").and_then(|v| v.get("run_b")).cloned().unwrap_or(serde_json::Value::Null),
            "raw_diff": quality
        },
        "interpretation": {
            "claim_level": if baseline_repro.get("audit_ok").and_then(|v| v.as_bool()).unwrap_or(false)
                && candidate_repro.get("audit_ok").and_then(|v| v.as_bool()).unwrap_or(false)
            {
                "evidence_bound"
            } else {
                "audit_limited"
            },
            "note": "quality deltas are comparative diagnostics and must be interpreted with validation evidence and support maturity"
        },
        "layered": layered_report(
            vec![],
            serde_json::json!({
                "reproducibility": {
                    "fingerprint_match": baseline_repro.get("replay_fingerprint") == candidate_repro.get("replay_fingerprint"),
                    "baseline_audit_ok": baseline_repro.get("audit_ok").and_then(|v| v.as_bool()).unwrap_or(false),
                    "candidate_audit_ok": candidate_repro.get("audit_ok").and_then(|v| v.as_bool()).unwrap_or(false)
                },
                "quality_delta": {
                    "mean_rms_delta_m": quality.get("mean_rms_m").and_then(|v| v.get("delta")).and_then(|v| v.as_f64()).unwrap_or(0.0),
                    "mean_cn0_delta_dbhz": quality.get("mean_cn0_dbhz").and_then(|v| v.get("delta")).and_then(|v| v.as_f64()).unwrap_or(0.0)
                }
            })
        )
    }))
}

fn replay_audit_report(baseline_run_dir: &Path, candidate_run_dir: &Path) -> Result<serde_json::Value> {
    ensure_run_dir_exists(baseline_run_dir)?;
    ensure_run_dir_exists(candidate_run_dir)?;
    let compare = compare_run_evidence(baseline_run_dir, candidate_run_dir)?;
    let baseline_explain = explain_run_scope(baseline_run_dir)?;
    let candidate_explain = explain_run_scope(candidate_run_dir)?;

    let fingerprint_match = compare
        .get("reproducibility")
        .and_then(|v| v.get("fingerprint_match"))
        .and_then(|v| v.as_bool())
        .unwrap_or(false);
    let baseline_audit_ok = compare
        .get("reproducibility")
        .and_then(|v| v.get("baseline_audit_ok"))
        .and_then(|v| v.as_bool())
        .unwrap_or(false);
    let candidate_audit_ok = compare
        .get("reproducibility")
        .and_then(|v| v.get("candidate_audit_ok"))
        .and_then(|v| v.as_bool())
        .unwrap_or(false);
    let corrupted_delta = compare
        .get("artifact_integrity")
        .and_then(|v| v.get("corrupted_file_delta"))
        .and_then(|v| v.as_i64())
        .unwrap_or(0);

    let mut reasons = Vec::new();
    if !baseline_audit_ok {
        reasons.push("baseline_repro_audit_failed".to_string());
    }
    if !candidate_audit_ok {
        reasons.push("candidate_repro_audit_failed".to_string());
    }
    if !fingerprint_match {
        reasons.push("replay_fingerprint_changed".to_string());
    }
    if corrupted_delta != 0 {
        reasons.push("artifact_corruption_delta_detected".to_string());
    }
    let classification = if baseline_audit_ok && candidate_audit_ok && fingerprint_match && corrupted_delta == 0
    {
        "deterministic_match"
    } else if baseline_audit_ok && candidate_audit_ok {
        "scientific_or_configuration_drift"
    } else {
        "integrity_failure"
    };

    Ok(serde_json::json!({
        "schema_version": 1,
        "baseline_run_dir": baseline_run_dir.display().to_string(),
        "candidate_run_dir": candidate_run_dir.display().to_string(),
        "classification": classification,
        "reasons": reasons,
        "baseline_command": baseline_explain
            .get("replay_scope")
            .and_then(|v| v.get("command"))
            .and_then(|v| v.as_str())
            .unwrap_or("unknown"),
        "candidate_command": candidate_explain
            .get("replay_scope")
            .and_then(|v| v.get("command"))
            .and_then(|v| v.as_str())
            .unwrap_or("unknown"),
        "compare": compare,
        "layered": layered_report(
            reasons.clone(),
            serde_json::json!({
                "classification": classification,
                "compare": compare
            })
        )
    }))
}

fn advanced_gate_report(run_dir: &Path, mode: AdvancedGateMode) -> Result<serde_json::Value> {
    ensure_run_dir_exists(run_dir)?;
    let mode_label = match mode {
        AdvancedGateMode::Rtk => "Rtk",
        AdvancedGateMode::Ppp => "Ppp",
    };
    let support_path = run_dir.join("artifacts").join("rtk").join("rtk_support_matrix.json");
    let support_json = if support_path.exists() {
        serde_json::from_str::<serde_json::Value>(&fs::read_to_string(&support_path)?).unwrap_or(serde_json::Value::Null)
    } else {
        serde_json::Value::Null
    };
    let support_row = support_json
        .get("rows")
        .and_then(|v| v.as_array())
        .and_then(|rows| {
            rows.iter().find(|row| {
                row.get("mode")
                    .and_then(|v| v.as_str())
                    .map(|value| value == mode_label)
                    .unwrap_or(false)
            })
        })
        .cloned()
        .unwrap_or(serde_json::Value::Null);

    let maturity = support_row
        .get("maturity")
        .and_then(|v| v.as_str())
        .unwrap_or("unknown");
    let real_solver = support_row
        .get("real_solver")
        .and_then(|v| v.as_bool())
        .unwrap_or(false);

    let evidence_path = run_dir
        .join("artifacts")
        .join("validate")
        .join("validation_evidence_bundle.json");
    let evidence_json = if evidence_path.exists() {
        serde_json::from_str::<serde_json::Value>(&fs::read_to_string(&evidence_path)?).unwrap_or(serde_json::Value::Null)
    } else {
        serde_json::Value::Null
    };
    let claim_guard_supported = evidence_json
        .get("claim_evidence_guard")
        .and_then(|v| v.get("supported"))
        .and_then(|v| v.as_bool())
        .unwrap_or(false);

    let mut reasons = Vec::new();
    if support_row.is_null() {
        reasons.push("support_matrix_row_missing".to_string());
    }
    if maturity == "Scaffolding" {
        reasons.push("mode_is_scaffolding".to_string());
    }
    if !real_solver {
        reasons.push("mode_not_real_solver".to_string());
    }
    if !claim_guard_supported {
        reasons.push("claim_evidence_guard_not_supported".to_string());
    }
    let gate_passed = reasons.is_empty();
    let claim_level = if maturity == "Scaffolding" || !real_solver {
        "scaffolding_only"
    } else if gate_passed {
        "evidence_bound_experimental"
    } else {
        "blocked_by_evidence"
    };

    Ok(serde_json::json!({
        "schema_version": 1,
        "run_dir": run_dir.display().to_string(),
        "mode": mode_label,
        "gate_passed": gate_passed,
        "claim_level": claim_level,
        "reasons": reasons,
        "support": {
            "matrix_path": support_path.display().to_string(),
            "maturity": maturity,
            "real_solver": real_solver,
            "row": support_row
        },
        "evidence": {
            "bundle_path": evidence_path.display().to_string(),
            "claim_guard_supported": claim_guard_supported,
            "claim_guard": evidence_json
                .get("claim_evidence_guard")
                .cloned()
                .unwrap_or(serde_json::Value::Null)
        }
    }))
}

fn artifact_inventory_report(run_dir: &Path) -> Result<serde_json::Value> {
    ensure_run_dir_exists(run_dir)?;
    let artifacts_root = run_dir.join("artifacts");
    let mut groups = serde_json::Map::new();
    if artifacts_root.exists() {
        for entry in fs::read_dir(&artifacts_root)? {
            let entry = entry?;
            let path = entry.path();
            if !path.is_dir() {
                continue;
            }
            let name = entry.file_name().to_string_lossy().to_string();
            let mut files = Vec::new();
            for child in fs::read_dir(&path)? {
                let child = child?;
                let child_path = child.path();
                if child_path.is_file() {
                    files.push(child.file_name().to_string_lossy().to_string());
                }
            }
            files.sort();
            groups.insert(
                name,
                serde_json::json!({
                    "file_count": files.len(),
                    "files": files
                }),
            );
        }
    }
    Ok(serde_json::json!({
        "schema_version": 1,
        "run_dir": run_dir.display().to_string(),
        "groups": groups
    }))
}

fn debug_plan_report(run_dir: &Path) -> Result<serde_json::Value> {
    ensure_run_dir_exists(run_dir)?;
    Ok(serde_json::json!({
        "schema_version": 1,
        "run_dir": run_dir.display().to_string(),
        "stages": [
            {
                "stage": "acquisition",
                "artifact": "artifacts/acquire/acq.jsonl",
                "recommended_check": "bijux gnss artifact validate --file <acq.jsonl>",
                "diagnostic_focus": "peak ratios, support status, threshold provenance"
            },
            {
                "stage": "tracking",
                "artifact": "artifacts/track/track.jsonl",
                "recommended_check": "bijux gnss diagnostics channel-summary --run-dir <run_dir>",
                "diagnostic_focus": "lock transitions, cycle slips, cn0 distribution"
            },
            {
                "stage": "observations",
                "artifact": "artifacts/obs/obs.jsonl",
                "recommended_check": "bijux gnss artifact validate --file <obs.jsonl>",
                "diagnostic_focus": "missing/weak observables, epoch consistency"
            },
            {
                "stage": "pvt",
                "artifact": "artifacts/pvt/pvt.jsonl",
                "recommended_check": "bijux gnss diagnostics medium-gate --run-dir <run_dir>",
                "diagnostic_focus": "residuals, geometry, claim evidence guard"
            }
        ]
    }))
}

fn benchmark_summary_report(run_dir: &Path) -> Result<serde_json::Value> {
    ensure_run_dir_exists(run_dir)?;
    let obs_path = run_dir.join("artifacts").join("obs").join("obs.jsonl");
    let pvt_path = run_dir.join("artifacts").join("pvt").join("pvt.jsonl");
    let obs = if obs_path.exists() {
        read_obs_epochs(&obs_path).unwrap_or_default()
    } else {
        Vec::new()
    };
    let nav = if pvt_path.exists() {
        read_nav_solutions(&pvt_path).unwrap_or_default()
    } else {
        Vec::new()
    };
    let mean_cn0 = {
        let mut sum = 0.0;
        let mut count = 0usize;
        for epoch in &obs {
            for sat in &epoch.sats {
                sum += sat.cn0_dbhz;
                count = count.saturating_add(1);
            }
        }
        if count == 0 {
            None
        } else {
            Some(sum / count as f64)
        }
    };
    let mean_rms = if nav.is_empty() {
        None
    } else {
        Some(nav.iter().map(|v| v.rms_m.0).sum::<f64>() / nav.len() as f64)
    };
    Ok(serde_json::json!({
        "schema_version": 1,
        "run_dir": run_dir.display().to_string(),
        "summary": {
            "obs_epochs": obs.len(),
            "nav_epochs": nav.len(),
            "mean_cn0_dbhz": mean_cn0,
            "mean_nav_rms_m": mean_rms
        },
        "rigor": {
            "requires_validation_evidence_bundle": true,
            "requires_replay_audit": true
        }
    }))
}

fn medium_gate_report(run_dir: &Path) -> Result<serde_json::Value> {
    ensure_run_dir_exists(run_dir)?;
    let explain = explain_run_scope(run_dir)?;
    let repro = verify_repro_bundle(run_dir)?;
    let evidence_path = run_dir
        .join("artifacts")
        .join("validate")
        .join("validation_evidence_bundle.json");
    let evidence = if evidence_path.exists() {
        serde_json::from_str::<serde_json::Value>(&fs::read_to_string(&evidence_path)?)
            .unwrap_or(serde_json::Value::Null)
    } else {
        serde_json::Value::Null
    };
    let claim_supported = evidence
        .get("claim_evidence_guard")
        .and_then(|v| v.get("supported"))
        .and_then(|v| v.as_bool())
        .unwrap_or(false);
    let corruption_count = explain
        .get("artifact_integrity")
        .and_then(|v| v.get("corrupted_files"))
        .and_then(|v| v.as_array())
        .map_or(0usize, Vec::len);
    let repro_ok = repro.get("audit_ok").and_then(|v| v.as_bool()).unwrap_or(false);
    let mut reasons = Vec::new();
    if !repro_ok {
        reasons.push("repro_audit_failed".to_string());
    }
    if corruption_count > 0 {
        reasons.push("artifact_corruption_detected".to_string());
    }
    if !claim_supported {
        reasons.push("claim_evidence_guard_failed".to_string());
    }
    Ok(serde_json::json!({
        "schema_version": 1,
        "run_dir": run_dir.display().to_string(),
        "gate_passed": reasons.is_empty(),
        "reasons": reasons,
        "checks": {
            "replay_audit_ok": repro_ok,
            "artifact_corruption_count": corruption_count,
            "claim_evidence_supported": claim_supported
        }
    }))
}

fn operator_status_report(run_dir: &Path) -> Result<serde_json::Value> {
    ensure_run_dir_exists(run_dir)?;
    let explain = explain_run_scope(run_dir)?;
    let repro = verify_repro_bundle(run_dir)?;
    let evidence_path = run_dir
        .join("artifacts")
        .join("validate")
        .join("validation_evidence_bundle.json");
    let evidence = if evidence_path.exists() {
        serde_json::from_str::<serde_json::Value>(&fs::read_to_string(&evidence_path)?)
            .unwrap_or(serde_json::Value::Null)
    } else {
        serde_json::Value::Null
    };
    let corrupted = explain
        .get("artifact_integrity")
        .and_then(|v| v.get("corrupted_files"))
        .and_then(|v| v.as_array())
        .map_or(0usize, Vec::len);
    let evidence_state = if evidence
        .get("claim_evidence_guard")
        .and_then(|v| v.get("supported"))
        .and_then(|v| v.as_bool())
        .unwrap_or(false)
    {
        "evidence_supported"
    } else {
        "evidence_limited"
    };
    let quality_state = if corrupted == 0 {
        "artifact_clean"
    } else {
        "artifact_corrupted"
    };
    let run_state = if repro.get("audit_ok").and_then(|v| v.as_bool()).unwrap_or(false) {
        "audit_ready"
    } else {
        "audit_alert"
    };
    Ok(serde_json::json!({
        "schema_version": 1,
        "run_dir": run_dir.display().to_string(),
        "status": {
            "run_state": run_state,
            "quality_state": quality_state,
            "evidence_state": evidence_state
        },
        "details": {
            "corrupted_artifact_count": corrupted,
            "replay_fingerprint": repro.get("replay_fingerprint").cloned().unwrap_or(serde_json::Value::Null)
        }
    }))
}

fn channel_summary_report(run_dir: &Path) -> Result<serde_json::Value> {
    ensure_run_dir_exists(run_dir)?;
    let obs_path = run_dir.join("artifacts").join("obs").join("obs.jsonl");
    let obs = if obs_path.exists() {
        read_obs_epochs(&obs_path).unwrap_or_default()
    } else {
        Vec::new()
    };
    let mut per_channel: std::collections::BTreeMap<String, (usize, f64)> =
        std::collections::BTreeMap::new();
    for epoch in &obs {
        for sat in &epoch.sats {
            let key = format!("{:?}-{}", sat.signal_id.sat.constellation, sat.signal_id.sat.prn);
            let entry = per_channel.entry(key).or_insert((0usize, 0.0));
            entry.0 = entry.0.saturating_add(1);
            entry.1 += sat.cn0_dbhz;
        }
    }
    let channels: Vec<serde_json::Value> = per_channel
        .iter()
        .map(|(channel, (count, sum_cn0))| {
            serde_json::json!({
                "channel": channel,
                "epochs_seen": count,
                "mean_cn0_dbhz": if *count == 0 { 0.0 } else { *sum_cn0 / *count as f64 }
            })
        })
        .collect();
    Ok(serde_json::json!({
        "schema_version": 1,
        "run_dir": run_dir.display().to_string(),
        "summary": {
            "obs_epochs": obs.len(),
            "channel_count": channels.len()
        },
        "channels": channels
    }))
}

fn export_bundle_report(run_dir: &Path, out_dir: Option<&PathBuf>) -> Result<serde_json::Value> {
    ensure_run_dir_exists(run_dir)?;
    let bundle_root = out_dir
        .cloned()
        .unwrap_or_else(|| run_dir.join("artifacts").join("diagnostics_export_bundle"));
    fs::create_dir_all(&bundle_root)?;

    let mut included = Vec::new();
    let mut checksums = serde_json::Map::new();
    let candidates = vec![
        run_dir.join("manifest.json"),
        run_dir.join("run_report.json"),
        run_dir
            .join("artifacts")
            .join("validate")
            .join("validation_report.json"),
        run_dir
            .join("artifacts")
            .join("validate")
            .join("validation_evidence_bundle.json"),
        run_dir.join("trace.ndjson"),
    ];
    for src in candidates {
        if !src.exists() || !src.is_file() {
            continue;
        }
        let file_name = src
            .file_name()
            .and_then(|v| v.to_str())
            .unwrap_or("unknown")
            .to_string();
        let dst = bundle_root.join(&file_name);
        fs::copy(&src, &dst)?;
        let bytes = fs::read(&dst)?;
        checksums.insert(file_name.clone(), serde_json::Value::String(sha256_hex(&bytes)));
        included.push(file_name);
    }
    included.sort();

    let bundle_manifest = serde_json::json!({
        "schema_version": 1,
        "source_run_dir": run_dir.display().to_string(),
        "included_files": included,
        "checksums_sha256": checksums
    });
    fs::write(
        bundle_root.join("bundle_manifest.json"),
        serde_json::to_string_pretty(&bundle_manifest)?,
    )?;

    Ok(serde_json::json!({
        "schema_version": 1,
        "run_dir": run_dir.display().to_string(),
        "bundle_dir": bundle_root.display().to_string(),
        "file_count": bundle_manifest
            .get("included_files")
            .and_then(|v| v.as_array())
            .map_or(0usize, Vec::len),
        "bundle_manifest": bundle_manifest
    }))
}

fn machine_catalog_report() -> serde_json::Value {
    let reports = vec![
        serde_json::json!({"name": "diagnostics_operator_map", "schema": "schemas/diagnostics_operator_map_report.schema.json", "schema_version": 1}),
        serde_json::json!({"name": "diagnostics_workflow", "schema": "schemas/diagnostics_workflow_report.schema.json", "schema_version": 1}),
        serde_json::json!({"name": "diagnostics_summarize", "schema": "schemas/diagnostics_summary_report.schema.json", "schema_version": 1}),
        serde_json::json!({"name": "diagnostics_explain", "schema": "schemas/diagnostics_explain_report.schema.json", "schema_version": 1}),
        serde_json::json!({"name": "diagnostics_verify_repro", "schema": "schemas/diagnostics_verify_repro_report.schema.json", "schema_version": 1}),
        serde_json::json!({"name": "diagnostics_compare", "schema": "schemas/diagnostics_compare_report.schema.json", "schema_version": 1}),
        serde_json::json!({"name": "diagnostics_replay_audit", "schema": "schemas/diagnostics_replay_audit_report.schema.json", "schema_version": 1}),
        serde_json::json!({"name": "diagnostics_advanced_gate", "schema": "schemas/diagnostics_advanced_gate_report.schema.json", "schema_version": 1}),
        serde_json::json!({"name": "diagnostics_artifact_inventory", "schema": "schemas/diagnostics_artifact_inventory_report.schema.json", "schema_version": 1}),
        serde_json::json!({"name": "diagnostics_debug_plan", "schema": "schemas/diagnostics_debug_plan_report.schema.json", "schema_version": 1}),
        serde_json::json!({"name": "diagnostics_benchmark_summary", "schema": "schemas/diagnostics_benchmark_summary_report.schema.json", "schema_version": 1}),
        serde_json::json!({"name": "diagnostics_medium_gate", "schema": "schemas/diagnostics_medium_gate_report.schema.json", "schema_version": 1}),
        serde_json::json!({"name": "diagnostics_operator_status", "schema": "schemas/diagnostics_operator_status_report.schema.json", "schema_version": 1}),
        serde_json::json!({"name": "diagnostics_channel_summary", "schema": "schemas/diagnostics_channel_summary_report.schema.json", "schema_version": 1}),
        serde_json::json!({"name": "diagnostics_export_bundle", "schema": "schemas/diagnostics_export_bundle_report.schema.json", "schema_version": 1}),
        serde_json::json!({"name": "diagnostics_api_parity", "schema": "schemas/diagnostics_api_parity_report.schema.json", "schema_version": 1}),
        serde_json::json!({"name": "diagnostics_expert_guide", "schema": "schemas/diagnostics_expert_guide_report.schema.json", "schema_version": 1}),
        serde_json::json!({"name": "diagnostics_history_browse", "schema": "schemas/diagnostics_history_browse_report.schema.json", "schema_version": 1}),
        serde_json::json!({"name": "diagnostics_route_explain", "schema": "schemas/diagnostics_route_explain_report.schema.json", "schema_version": 1}),
        serde_json::json!({"name": "diagnostics_operator_workflow", "schema": "schemas/diagnostics_operator_workflow_report.schema.json", "schema_version": 1}),
        serde_json::json!({"name": "diagnostics_operator_ergonomics", "schema": "schemas/diagnostics_operator_ergonomics_report.schema.json", "schema_version": 1}),
        serde_json::json!({"name": "diagnostics_audit_trail", "schema": "schemas/diagnostics_audit_trail_report.schema.json", "schema_version": 1}),
        serde_json::json!({"name": "diagnostics_dependency_trace", "schema": "schemas/diagnostics_dependency_trace_report.schema.json", "schema_version": 1})
    ];
    serde_json::json!({
        "schema_version": 1,
        "reports": reports
    })
}

fn api_parity_report() -> serde_json::Value {
    let workflows = vec![
        serde_json::json!({
            "workflow": "run",
            "cli_supported": true,
            "api_supported": true,
            "cli_command": "bijux gnss run",
            "api_surface": "prepare_run + receiver runtime pipeline"
        }),
        serde_json::json!({
            "workflow": "inspect",
            "cli_supported": true,
            "api_supported": true,
            "cli_command": "bijux gnss inspect",
            "api_surface": "FileSamples + signal metadata inspectors"
        }),
        serde_json::json!({
            "workflow": "diagnose",
            "cli_supported": true,
            "api_supported": true,
            "cli_command": "bijux gnss diagnostics explain|summarize",
            "api_surface": "artifact_validate + aggregate_diagnostics + run manifest parser"
        }),
        serde_json::json!({
            "workflow": "compare",
            "cli_supported": true,
            "api_supported": true,
            "cli_command": "bijux gnss diagnostics compare",
            "api_surface": "read_nav_solutions + read_obs_epochs + validate_reference"
        }),
        serde_json::json!({
            "workflow": "replay_audit",
            "cli_supported": true,
            "api_supported": true,
            "cli_command": "bijux gnss diagnostics replay-audit",
            "api_surface": "run manifest + artifact hash + replay fingerprint computation"
        }),
        serde_json::json!({
            "workflow": "export",
            "cli_supported": true,
            "api_supported": true,
            "cli_command": "bijux gnss diagnostics export-bundle",
            "api_surface": "run layout paths + managed artifact copy + sha256 manifest"
        }),
    ];
    let parity_ok = workflows.iter().all(|row| {
        row.get("cli_supported")
            .and_then(|v| v.as_bool())
            .unwrap_or(false)
            == row
                .get("api_supported")
                .and_then(|v| v.as_bool())
                .unwrap_or(false)
    });
    serde_json::json!({
        "schema_version": 1,
        "parity_ok": parity_ok,
        "workflows": workflows
    })
}

fn expert_guide_report() -> serde_json::Value {
    let flows = vec![
        serde_json::json!({
            "name": "integrity_triage",
            "intent": "fast integrity diagnosis without losing rigor",
            "steps": [
                "bijux gnss diagnostics medium-gate --run-dir <run_dir> --report table",
                "bijux gnss diagnostics replay-audit --baseline-run-dir <a> --candidate-run-dir <b> --report json",
                "bijux gnss diagnostics advanced-gate --run-dir <run_dir> --mode rtk --strict --report json"
            ]
        }),
        serde_json::json!({
            "name": "artifact_forensics",
            "intent": "explain artifact state before deep debugging",
            "steps": [
                "bijux gnss diagnostics artifact-inventory --run-dir <run_dir> --report json",
                "bijux gnss diagnostics explain --run-dir <run_dir> --report json",
                "bijux gnss diagnostics export-bundle --run-dir <run_dir>"
            ]
        }),
        serde_json::json!({
            "name": "solver_regression",
            "intent": "compare navigation behavior with evidence context",
            "steps": [
                "bijux gnss diagnostics compare --baseline-run-dir <a> --candidate-run-dir <b> --report json",
                "bijux gnss diagnostics channel-summary --run-dir <b> --report json",
                "bijux gnss diagnostics benchmark-summary --run-dir <b> --report json"
            ]
        }),
    ];
    serde_json::json!({
        "schema_version": 1,
        "flows": flows
    })
}

fn history_browse_report(root_dir: &Path, limit: usize) -> Result<serde_json::Value> {
    if !root_dir.exists() || !root_dir.is_dir() {
        return Err(classified_error(
            CliErrorClass::OperatorMisconfiguration,
            format!("history root not found: {}", root_dir.display()),
        ));
    }
    let mut runs = Vec::new();
    for entry in fs::read_dir(root_dir)? {
        let entry = entry?;
        let path = entry.path();
        if !path.is_dir() {
            continue;
        }
        let manifest_path = path.join("manifest.json");
        if !manifest_path.exists() {
            continue;
        }
        let manifest = serde_json::from_str::<serde_json::Value>(&fs::read_to_string(&manifest_path)?)
            .unwrap_or(serde_json::Value::Null);
        let artifact_files = path.join("artifacts");
        let artifact_file_count = if artifact_files.exists() {
            let mut count = 0usize;
            for group in fs::read_dir(&artifact_files)? {
                let group = group?;
                let group_path = group.path();
                if group_path.is_dir() {
                    for child in fs::read_dir(group_path)? {
                        let child = child?;
                        if child.path().is_file() {
                            count = count.saturating_add(1);
                        }
                    }
                }
            }
            count
        } else {
            0
        };
        runs.push(serde_json::json!({
            "run_dir": path.display().to_string(),
            "command": manifest.get("command").cloned().unwrap_or(serde_json::Value::Null),
            "dataset_id": manifest.get("dataset_id").cloned().unwrap_or(serde_json::Value::Null),
            "config_hash": manifest.get("config_hash").cloned().unwrap_or(serde_json::Value::Null),
            "artifact_file_count": artifact_file_count
        }));
    }
    runs.sort_by(|a, b| {
        b.get("run_dir")
            .and_then(|v| v.as_str())
            .unwrap_or("")
            .cmp(a.get("run_dir").and_then(|v| v.as_str()).unwrap_or(""))
    });
    runs.truncate(limit.max(1));
    Ok(serde_json::json!({
        "schema_version": 1,
        "root_dir": root_dir.display().to_string(),
        "limit": limit,
        "runs": runs
    }))
}

fn route_explain_report(topic: RouteTopic) -> serde_json::Value {
    let topic_name = format!("{topic:?}").to_lowercase();
    let steps = match topic {
        RouteTopic::Integrity => vec![
            serde_json::json!({"command": "bijux gnss diagnostics medium-gate --run-dir <run_dir> --report table", "reason": "surface critical pass/fail first"}),
            serde_json::json!({"command": "bijux gnss diagnostics advanced-gate --run-dir <run_dir> --mode rtk --report json", "reason": "separate support maturity from evidence failures"}),
            serde_json::json!({"command": "bijux gnss diagnostics explain --run-dir <run_dir> --report json", "reason": "inspect replay scope, cache, and artifact integrity"})
        ],
        RouteTopic::Replay => vec![
            serde_json::json!({"command": "bijux gnss diagnostics verify-repro --run-dir <run_dir> --report json", "reason": "establish bundle and fingerprint baseline"}),
            serde_json::json!({"command": "bijux gnss diagnostics replay-audit --baseline-run-dir <a> --candidate-run-dir <b> --report table", "reason": "classify deterministic match vs drift"}),
            serde_json::json!({"command": "bijux gnss diagnostics compare --baseline-run-dir <a> --candidate-run-dir <b> --report json", "reason": "attach quality deltas to replay outcome"})
        ],
        RouteTopic::Compare => vec![
            serde_json::json!({"command": "bijux gnss diagnostics compare --baseline-run-dir <a> --candidate-run-dir <b> --report json", "reason": "compute reproducibility and quality delta summary"}),
            serde_json::json!({"command": "bijux gnss diagnostics channel-summary --run-dir <b> --report json", "reason": "inspect channel-level context for differences"}),
            serde_json::json!({"command": "bijux gnss diagnostics benchmark-summary --run-dir <b> --report json", "reason": "review digestible benchmark metrics with rigor markers"})
        ],
        RouteTopic::Export => vec![
            serde_json::json!({"command": "bijux gnss diagnostics export-bundle --run-dir <run_dir>", "reason": "create reproducible review package"}),
            serde_json::json!({"command": "bijux gnss diagnostics machine-catalog --report json", "reason": "declare report contracts for downstream systems"}),
            serde_json::json!({"command": "bijux gnss diagnostics history-browse --root-dir runs --report json", "reason": "find neighboring runs for triage context"})
        ],
    };
    serde_json::json!({
        "schema_version": 1,
        "topic": topic_name,
        "steps": steps
    })
}

fn operator_workflow_report(profile: WorkflowProfile) -> serde_json::Value {
    let profile_name = format!("{profile:?}").to_lowercase();
    let steps = match profile {
        WorkflowProfile::Run => vec![
            serde_json::json!({"label": "execute", "command": "bijux gnss run --dataset <id> --config <profile.toml>", "output": "manifest + run_report + artifacts"}),
            serde_json::json!({"label": "status", "command": "bijux gnss diagnostics operator-status --run-dir <run_dir> --report table", "output": "run/quality/evidence states"}),
            serde_json::json!({"label": "gate", "command": "bijux gnss diagnostics medium-gate --run-dir <run_dir> --strict --report json", "output": "integration gate result"})
        ],
        WorkflowProfile::Triage => vec![
            serde_json::json!({"label": "inventory", "command": "bijux gnss diagnostics artifact-inventory --run-dir <run_dir> --report table", "output": "artifact group counts"}),
            serde_json::json!({"label": "route", "command": "bijux gnss diagnostics route-explain --topic integrity --report table", "output": "debugging command path"}),
            serde_json::json!({"label": "bundle", "command": "bijux gnss diagnostics export-bundle --run-dir <run_dir>", "output": "reproducible review package"})
        ],
        WorkflowProfile::Compare => vec![
            serde_json::json!({"label": "replay", "command": "bijux gnss diagnostics replay-audit --baseline-run-dir <a> --candidate-run-dir <b> --report table", "output": "determinism classification"}),
            serde_json::json!({"label": "quality", "command": "bijux gnss diagnostics compare --baseline-run-dir <a> --candidate-run-dir <b> --report json", "output": "quality and integrity deltas"}),
            serde_json::json!({"label": "channel", "command": "bijux gnss diagnostics channel-summary --run-dir <b> --report table", "output": "channel-level context for drift"})
        ],
    };
    serde_json::json!({
        "schema_version": 1,
        "profile": profile_name,
        "steps": steps
    })
}

fn operator_ergonomics_report(run_dir: &Path) -> Result<serde_json::Value> {
    ensure_run_dir_exists(run_dir)?;
    let status = operator_status_report(run_dir)?;
    let gate = medium_gate_report(run_dir)?;
    let gate_passed = gate
        .get("gate_passed")
        .and_then(|v| v.as_bool())
        .unwrap_or(false);
    let evidence_state = status
        .get("status")
        .and_then(|v| v.get("evidence_state"))
        .and_then(|v| v.as_str())
        .unwrap_or("unknown");
    let quality_state = status
        .get("status")
        .and_then(|v| v.get("quality_state"))
        .and_then(|v| v.as_str())
        .unwrap_or("unknown");
    let run_state = status
        .get("status")
        .and_then(|v| v.get("run_state"))
        .and_then(|v| v.as_str())
        .unwrap_or("unknown");
    let score = match (gate_passed, quality_state, evidence_state, run_state) {
        (true, "artifact_clean", "evidence_supported", "audit_ready") => 1.0,
        (true, "artifact_clean", _, _) => 0.8,
        (true, _, _, _) => 0.6,
        (false, _, _, _) => 0.3,
    };
    let quick_actions = if gate_passed {
        vec![
            "route-explain --topic compare".to_string(),
            "operator-workflow --profile compare".to_string(),
        ]
    } else {
        vec![
            "medium-gate --strict".to_string(),
            "route-explain --topic integrity".to_string(),
            "export-bundle".to_string(),
        ]
    };

    Ok(serde_json::json!({
        "schema_version": 1,
        "run_dir": run_dir.display().to_string(),
        "ergonomics_score": score,
        "quick_actions": quick_actions,
        "rigor": {
            "medium_gate_passed": gate_passed,
            "evidence_state": evidence_state,
            "quality_state": quality_state,
            "run_state": run_state
        }
    }))
}

fn audit_trail_report(run_dir: &Path) -> Result<serde_json::Value> {
    ensure_run_dir_exists(run_dir)?;
    let manifest_path = run_dir.join("manifest.json");
    let manifest: serde_json::Value = serde_json::from_str(&fs::read_to_string(&manifest_path)?)
        .unwrap_or(serde_json::Value::Null);
    let replay_controls = serde_json::json!({
        "deterministic": manifest
            .get("replay_scope")
            .and_then(|v| v.get("deterministic"))
            .cloned()
            .unwrap_or(serde_json::Value::Null),
        "resume": manifest
            .get("replay_scope")
            .and_then(|v| v.get("resume"))
            .cloned()
            .unwrap_or(serde_json::Value::Null),
        "explicit_output_dir": manifest
            .get("replay_scope")
            .and_then(|v| v.get("explicit_output_dir"))
            .cloned()
            .unwrap_or(serde_json::Value::Null),
    });

    let mut override_events = Vec::new();
    let trace_path = run_dir.join("trace.ndjson");
    if trace_path.exists() {
        let trace = fs::read_to_string(&trace_path)?;
        for line in trace.lines() {
            if line.trim().is_empty() {
                continue;
            }
            let Ok(row) = serde_json::from_str::<serde_json::Value>(line) else {
                continue;
            };
            let name = row
                .get("name")
                .and_then(|v| v.as_str())
                .unwrap_or_default()
                .to_lowercase();
            if name.contains("override") || name.contains("exception") || name.contains("replay")
            {
                override_events.push(row);
            }
        }
    }

    let mut policy_exceptions = Vec::new();
    let evidence_path = run_dir
        .join("artifacts")
        .join("validate")
        .join("validation_evidence_bundle.json");
    if evidence_path.exists() {
        let evidence = serde_json::from_str::<serde_json::Value>(&fs::read_to_string(&evidence_path)?)
            .unwrap_or(serde_json::Value::Null);
        if let Some(violations) = evidence
            .get("claim_evidence_guard")
            .and_then(|v| v.get("violations"))
            .and_then(|v| v.as_array())
        {
            policy_exceptions.extend(violations.iter().cloned());
        }
        if let Some(refusals) = evidence.get("refusal_reasons").and_then(|v| v.as_array()) {
            policy_exceptions.extend(refusals.iter().cloned());
        }
    }

    Ok(serde_json::json!({
        "schema_version": 1,
        "run_dir": run_dir.display().to_string(),
        "manifest_path": manifest_path.display().to_string(),
        "replay_controls": replay_controls,
        "override_events": override_events,
        "policy_exceptions": policy_exceptions
    }))
}

fn dependency_trace_report(run_dir: &Path) -> Result<serde_json::Value> {
    ensure_run_dir_exists(run_dir)?;
    let manifest_path = run_dir.join("manifest.json");
    let manifest: serde_json::Value = serde_json::from_str(&fs::read_to_string(&manifest_path)?)
        .unwrap_or(serde_json::Value::Null);

    let correction_path = run_dir
        .join("artifacts")
        .join("rtk")
        .join("rtk_correction_input.jsonl");
    let mut correction_rows = 0u64;
    let mut correction_tags = std::collections::BTreeSet::new();
    if correction_path.exists() {
        let text = fs::read_to_string(&correction_path)?;
        for line in text.lines() {
            if line.trim().is_empty() {
                continue;
            }
            let Ok(row) = serde_json::from_str::<serde_json::Value>(line) else {
                continue;
            };
            correction_rows = correction_rows.saturating_add(1);
            if let Some(tags) = row
                .get("payload")
                .and_then(|v| v.get("correction_tags"))
                .and_then(|v| v.as_array())
            {
                for tag in tags {
                    if let Some(value) = tag.as_str() {
                        correction_tags.insert(value.to_string());
                    }
                }
            }
        }
    }

    let mut env_references = Vec::new();
    if let Some(command) = manifest.get("command").and_then(|v| v.as_str()) {
        if command.contains("--config") {
            env_references.push("config_file".to_string());
        }
        if command.contains("--dataset") {
            env_references.push("dataset_reference".to_string());
        }
    }
    if manifest.get("toolchain").is_some() {
        env_references.push("toolchain_metadata".to_string());
    }
    if manifest.get("features").is_some() {
        env_references.push("feature_set".to_string());
    }
    env_references.sort();
    env_references.dedup();
    let env_reference_count = env_references.len();

    Ok(serde_json::json!({
        "schema_version": 1,
        "run_dir": run_dir.display().to_string(),
        "tooling": {
            "toolchain": manifest
                .get("toolchain")
                .and_then(|v| v.as_str())
                .unwrap_or("unknown"),
            "features": manifest
                .get("features")
                .cloned()
                .unwrap_or(serde_json::json!([]))
        },
        "corrections": {
            "artifact_path": correction_path.display().to_string(),
            "rows": correction_rows,
            "tags": correction_tags.into_iter().collect::<Vec<_>>()
        },
        "environment": {
            "dataset_id": manifest
                .get("dataset_id")
                .cloned()
                .unwrap_or(serde_json::Value::Null),
            "config_hash": manifest
                .get("config_hash")
                .cloned()
                .unwrap_or(serde_json::Value::Null),
            "references": env_references,
            "reference_count": env_reference_count
        }
    }))
}

fn ensure_run_dir_exists(run_dir: &Path) -> Result<()> {
    if run_dir.exists() && run_dir.is_dir() {
        return Ok(());
    }
    Err(classified_error(
        CliErrorClass::OperatorMisconfiguration,
        format!("run directory not found: {}", run_dir.display()),
    ))
}

fn sha256_hex(bytes: &[u8]) -> String {
    use sha2::Digest;
    let mut hasher = sha2::Sha256::new();
    hasher.update(bytes);
    hex::encode(hasher.finalize())
}

fn handle_doctor(command: GnssCommand) -> Result<()> {
    let GnssCommand::Doctor { common, file } = command else {
        bail!("invalid command for handler");
    };

    let _ = runtime_config_from_env(&common, None);
    println!("bijux gnss doctor");
    println!("build: {}", env!("CARGO_PKG_VERSION"));
    println!("features: tracing={}", cfg!(feature = "tracing"));

    #[cfg(any(target_arch = "x86", target_arch = "x86_64"))]
    {
        println!(
            "cpu: sse2={} avx2={} fma={}",
            std::is_x86_feature_detected!("sse2"),
            std::is_x86_feature_detected!("avx2"),
            std::is_x86_feature_detected!("fma")
        );
    }
    #[cfg(not(any(target_arch = "x86", target_arch = "x86_64")))]
    {
        println!("cpu: feature detection not available on this arch");
    }

    if let Some(config_path) = &common.config {
        match load_config_from_path(config_path) {
            Ok(_) => println!("config: ok ({})", config_path.display()),
            Err(err) => println!("config: invalid ({}) - {}", config_path.display(), err),
        }
    } else {
        println!("config: default");
    }

    if let Some(path) = file {
        if path.exists() {
            println!("input: ok ({})", path.display());
        } else {
            println!("input: missing ({})", path.display());
        }
    }

    Ok(())
}

#[cfg(test)]
mod diagnostics_tests {
    use super::{
        advanced_gate_report, artifact_inventory_report, compare_run_evidence, explain_run_scope,
        export_bundle_report, machine_catalog_report, medium_gate_report, operator_status_report,
        replay_audit_report, verify_repro_bundle, AdvancedGateMode,
    };
    use std::fs;
    use std::path::PathBuf;

    fn create_base_run(name: &str, config_hash: &str) -> PathBuf {
        let base = std::env::temp_dir().join(format!(
            "bijux_{name}_{}_{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .expect("duration")
                .as_nanos()
        ));
        fs::create_dir_all(base.join("artifacts")).expect("artifacts dir");
        let manifest = serde_json::json!({
            "command": "run",
            "dataset_id": "demo",
            "config_hash": config_hash,
            "layout_schema_version": 1,
            "replay_scope": {"deterministic": true},
            "front_end_provenance": {"sample_rate_hz": 5000000.0}
        });
        fs::write(
            base.join("manifest.json"),
            serde_json::to_string_pretty(&manifest).expect("manifest json"),
        )
        .expect("manifest write");
        fs::write(base.join("run_report.json"), "{\"schema_version\":1}\n").expect("report write");
        fs::write(base.join("artifacts").join("obs.jsonl"), "{\"payload\":{\"artifact_id\":\"obs-1\"}}\n")
            .expect("artifact write");
        base
    }

    #[test]
    fn explain_run_scope_reports_cache_replay_and_identity() {
        let base = std::env::temp_dir().join(format!(
            "bijux_diagnostics_explain_{}_{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .expect("duration")
                .as_nanos()
        ));
        fs::create_dir_all(base.join("artifacts")).expect("artifacts dir");

        let manifest = serde_json::json!({
            "command": "run",
            "dataset_id": "demo",
            "config_hash": "abc123",
            "toolchain": "rustc",
            "features": [],
            "replay_scope": {
                "deterministic": true,
                "resume": false,
                "explicit_output_dir": true
            }
        });
        fs::write(
            base.join("manifest.json"),
            serde_json::to_string_pretty(&manifest).expect("manifest json"),
        )
        .expect("manifest write");

        let obs_line = serde_json::json!({
            "header": {"schema_version":1},
            "payload": {"artifact_id":"obs-epoch-0001","epoch_id":"epoch-0001"}
        });
        fs::write(
            base.join("artifacts").join("obs.jsonl"),
            format!("{}\n", serde_json::to_string(&obs_line).expect("obs line")),
        )
        .expect("obs write");

        let trace_line = serde_json::json!({
            "name":"acquisition_code_fft_cache_miss",
            "fields":[["reason","incompatible_assumptions"]]
        });
        fs::write(
            base.join("trace.ndjson"),
            format!("{}\n", serde_json::to_string(&trace_line).expect("trace line")),
        )
        .expect("trace write");

        let summary = explain_run_scope(PathBuf::as_path(&base)).expect("summary");
        assert_eq!(
            summary
                .get("replay_scope")
                .and_then(|v| v.get("deterministic"))
                .and_then(|v| v.as_bool()),
            Some(true)
        );
        assert_eq!(
            summary
                .get("cache_behavior")
                .and_then(|v| v.get("miss_reason_counts"))
                .and_then(|v| v.get("incompatible_assumptions"))
                .and_then(|v| v.as_u64()),
            Some(1)
        );
        assert_eq!(
            summary
                .get("artifact_identity_coverage")
                .and_then(|v| v.get("obs.jsonl"))
                .and_then(|v| v.get("has_artifact_id"))
                .and_then(|v| v.as_bool()),
            Some(true)
        );

        let _ = fs::remove_dir_all(base);
    }

    #[test]
    fn verify_repro_bundle_reports_hashes_and_fingerprint() {
        let base = std::env::temp_dir().join(format!(
            "bijux_verify_repro_{}_{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .expect("duration")
                .as_nanos()
        ));
        fs::create_dir_all(base.join("artifacts")).expect("artifacts dir");
        let manifest = serde_json::json!({
            "command": "validate",
            "config_hash": "cfg",
            "dataset_id": "demo",
            "layout_schema_version": 1,
            "replay_scope": {"deterministic": true},
            "front_end_provenance": {"sample_rate_hz": 5000000.0}
        });
        fs::write(
            base.join("manifest.json"),
            serde_json::to_string_pretty(&manifest).expect("manifest"),
        )
        .expect("manifest write");
        fs::write(base.join("run_report.json"), "{\"schema_version\":1}\n").expect("report write");
        fs::write(base.join("artifacts").join("obs.jsonl"), "{\"payload\":{}}\n").expect("artifact");

        let report = verify_repro_bundle(PathBuf::as_path(&base)).expect("report");
        assert_eq!(report.get("audit_ok").and_then(|v| v.as_bool()), Some(true));
        assert!(report.get("replay_fingerprint").and_then(|v| v.as_str()).is_some());
        assert!(report.get("manifest_sha256").and_then(|v| v.as_str()).is_some());

        let _ = fs::remove_dir_all(base);
    }

    #[test]
    fn compare_run_evidence_reports_repro_and_quality_deltas() {
        let baseline = create_base_run("compare_base", "cfg-a");
        let candidate = create_base_run("compare_cand", "cfg-a");
        let report =
            compare_run_evidence(PathBuf::as_path(&baseline), PathBuf::as_path(&candidate))
                .expect("compare");
        assert_eq!(
            report
                .get("reproducibility")
                .and_then(|v| v.get("fingerprint_match"))
                .and_then(|v| v.as_bool()),
            Some(true)
        );
        assert!(report.get("quality").is_some());
        let _ = fs::remove_dir_all(baseline);
        let _ = fs::remove_dir_all(candidate);
    }

    #[test]
    fn replay_audit_report_classifies_fingerprint_drift() {
        let baseline = create_base_run("audit_base", "cfg-a");
        let candidate = create_base_run("audit_cand", "cfg-b");
        let report = replay_audit_report(PathBuf::as_path(&baseline), PathBuf::as_path(&candidate))
            .expect("replay audit");
        assert_eq!(
            report.get("classification").and_then(|v| v.as_str()),
            Some("scientific_or_configuration_drift")
        );
        let _ = fs::remove_dir_all(baseline);
        let _ = fs::remove_dir_all(candidate);
    }

    #[test]
    fn advanced_gate_report_passes_for_supported_rtk_claims() {
        let run_dir = create_base_run("advanced_gate", "cfg-gate");
        fs::create_dir_all(run_dir.join("artifacts").join("rtk")).expect("rtk dir");
        fs::create_dir_all(run_dir.join("artifacts").join("validate")).expect("validate dir");

        let support = serde_json::json!({
            "schema_version": 1,
            "rows": [
                {
                    "mode": "Rtk",
                    "maturity": "Experimental",
                    "real_solver": true,
                    "required_inputs": ["base_obs", "rover_obs", "ephemeris", "base_ecef"],
                    "notes": "supported"
                }
            ]
        });
        fs::write(
            run_dir.join("artifacts").join("rtk").join("rtk_support_matrix.json"),
            serde_json::to_string_pretty(&support).expect("support json"),
        )
        .expect("support write");

        let evidence = serde_json::json!({
            "schema_version": 1,
            "claim_evidence_guard": {
                "supported": true,
                "violations": []
            }
        });
        fs::write(
            run_dir
                .join("artifacts")
                .join("validate")
                .join("validation_evidence_bundle.json"),
            serde_json::to_string_pretty(&evidence).expect("evidence json"),
        )
        .expect("evidence write");

        let report = advanced_gate_report(PathBuf::as_path(&run_dir), AdvancedGateMode::Rtk)
            .expect("advanced gate");
        assert_eq!(report.get("gate_passed").and_then(|v| v.as_bool()), Some(true));

        let _ = fs::remove_dir_all(run_dir);
    }

    #[test]
    fn artifact_inventory_report_lists_groups() {
        let run_dir = create_base_run("artifact_inventory", "cfg-artifacts");
        fs::create_dir_all(run_dir.join("artifacts").join("obs")).expect("obs dir");
        fs::write(
            run_dir.join("artifacts").join("obs").join("obs.jsonl"),
            "{\"payload\":{}}\n",
        )
        .expect("obs write");
        let report = artifact_inventory_report(PathBuf::as_path(&run_dir)).expect("inventory report");
        assert!(report
            .get("groups")
            .and_then(|v| v.get("obs"))
            .is_some());
        let _ = fs::remove_dir_all(run_dir);
    }

    #[test]
    fn medium_gate_report_exposes_gate_flag() {
        let run_dir = create_base_run("medium_gate", "cfg-medium");
        fs::create_dir_all(run_dir.join("artifacts").join("validate")).expect("validate dir");
        fs::write(
            run_dir
                .join("artifacts")
                .join("validate")
                .join("validation_evidence_bundle.json"),
            "{\"claim_evidence_guard\":{\"supported\":true}}\n",
        )
        .expect("evidence write");
        let report = medium_gate_report(PathBuf::as_path(&run_dir)).expect("medium gate");
        assert!(report.get("gate_passed").is_some());
        let _ = fs::remove_dir_all(run_dir);
    }

    #[test]
    fn operator_status_report_emits_state_labels() {
        let run_dir = create_base_run("operator_status", "cfg-status");
        fs::create_dir_all(run_dir.join("artifacts").join("validate")).expect("validate dir");
        fs::write(
            run_dir
                .join("artifacts")
                .join("validate")
                .join("validation_evidence_bundle.json"),
            "{\"claim_evidence_guard\":{\"supported\":false}}\n",
        )
        .expect("evidence write");
        let report = operator_status_report(PathBuf::as_path(&run_dir)).expect("status");
        assert!(report.get("status").is_some());
        let _ = fs::remove_dir_all(run_dir);
    }

    #[test]
    fn export_bundle_report_writes_bundle_manifest() {
        let run_dir = create_base_run("bundle_export", "cfg-bundle");
        let report = export_bundle_report(PathBuf::as_path(&run_dir), None).expect("bundle");
        let bundle_dir = report
            .get("bundle_dir")
            .and_then(|v| v.as_str())
            .expect("bundle dir");
        let manifest = PathBuf::from(bundle_dir).join("bundle_manifest.json");
        assert!(manifest.exists());
        let _ = fs::remove_dir_all(run_dir);
    }

    #[test]
    fn machine_catalog_report_contains_compare_contract() {
        let report = machine_catalog_report();
        let has_compare = report
            .get("reports")
            .and_then(|v| v.as_array())
            .map(|rows| {
                rows.iter().any(|row| {
                    row.get("name")
                        .and_then(|v| v.as_str())
                        .map(|name| name == "diagnostics_compare")
                        .unwrap_or(false)
                })
            })
            .unwrap_or(false);
        assert!(has_compare);
    }
}
