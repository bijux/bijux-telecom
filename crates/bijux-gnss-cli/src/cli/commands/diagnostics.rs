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
    }

    Ok(())
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
        }
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
        "compare": compare
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
    use super::{explain_run_scope, verify_repro_bundle};
    use std::fs;
    use std::path::PathBuf;

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
}
