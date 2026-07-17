use super::*;

pub(crate) fn handle_diagnostics(command: GnssCommand) -> Result<()> {
    let GnssCommand::Diagnostics { command } = command else {
        bail!("invalid command for handler");
    };

    match command {
        DiagnosticsCommand::OperatorMap { common } => {
            workflow_dispatch::handle_operator_map(common)?
        }
        DiagnosticsCommand::Workflow { common } => workflow_dispatch::handle_workflow(common)?,
        DiagnosticsCommand::Summarize { common, run_dir, top } => {
            replay_dispatch::handle_summarize(common, run_dir, top)?
        }
        DiagnosticsCommand::Explain { common, run_dir } => {
            replay_dispatch::handle_explain(common, run_dir)?
        }
        DiagnosticsCommand::VerifyRepro { common, run_dir } => {
            replay_dispatch::handle_verify_repro(common, run_dir)?
        }
        DiagnosticsCommand::Compare { common, baseline_run_dir, candidate_run_dir } => {
            replay_dispatch::handle_compare(common, baseline_run_dir, candidate_run_dir)?
        }
        DiagnosticsCommand::ReplayAudit { common, baseline_run_dir, candidate_run_dir } => {
            replay_dispatch::handle_replay_audit(common, baseline_run_dir, candidate_run_dir)?
        }
        DiagnosticsCommand::AdvancedGate { common, run_dir, mode, strict } => {
            let _ = runtime_config_from_env(&common, None);
            let report = advanced_gate_report(&run_dir, mode)?;
            let passed =
                report.get("gate_passed").and_then(|value| value.as_bool()).unwrap_or(false);
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
            report_publishing::publish_diagnostics_report(
                &common,
                "diagnostics_advanced_gate",
                &report,
                "diagnostics_advanced_gate_report.schema.json",
                print_advanced_gate_table,
            )?;
        }
        DiagnosticsCommand::ArtifactInventory { common, run_dir } => {
            let report = artifact_inventory_report(&run_dir)?;
            report_publishing::publish_diagnostics_report(
                &common,
                "diagnostics_artifact_inventory",
                &report,
                "diagnostics_artifact_inventory_report.schema.json",
                print_artifact_inventory_table,
            )?;
        }
        DiagnosticsCommand::DebugPlan { common, run_dir } => {
            let report = debug_plan_report(&run_dir)?;
            report_publishing::publish_diagnostics_report(
                &common,
                "diagnostics_debug_plan",
                &report,
                "diagnostics_debug_plan_report.schema.json",
                print_debug_plan_table,
            )?;
        }
        DiagnosticsCommand::BenchmarkSummary { common, run_dir } => {
            let report = benchmark_summary_report(&run_dir)?;
            report_publishing::publish_diagnostics_report(
                &common,
                "diagnostics_benchmark_summary",
                &report,
                "diagnostics_benchmark_summary_report.schema.json",
                print_benchmark_summary_table,
            )?;
        }
        DiagnosticsCommand::MediumGate { common, run_dir, strict } => {
            let _ = runtime_config_from_env(&common, None);
            let report = medium_gate_report(&run_dir)?;
            let passed =
                report.get("gate_passed").and_then(|value| value.as_bool()).unwrap_or(false);
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
        DiagnosticsCommand::ExportBundle { common, run_dir, out_dir } => {
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
        DiagnosticsCommand::HistoryBrowse { common, root_dir, limit } => {
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
        DiagnosticsCommand::TrustClass { common, run_dir } => {
            let _ = runtime_config_from_env(&common, None);
            let report = trust_class_report(&run_dir)?;
            match common.report {
                ReportFormat::Table => print_trust_class_table(&report),
                ReportFormat::Json => emit_report(&common, "diagnostics_trust_class", &report)?,
            }
            write_diagnostics_report_artifact(
                &common,
                "diagnostics_trust_class",
                &report,
                "diagnostics_trust_class_report.schema.json",
            )?;
            write_manifest(
                &common,
                "diagnostics_trust_class",
                &ReceiverConfig::default(),
                None,
                &report,
            )?;
        }
        DiagnosticsCommand::IntegrityFocus { common, run_dir } => {
            let _ = runtime_config_from_env(&common, None);
            let report = integrity_focus_report(&run_dir)?;
            match common.report {
                ReportFormat::Table => print_integrity_focus_table(&report),
                ReportFormat::Json => emit_report(&common, "diagnostics_integrity_focus", &report)?,
            }
            write_diagnostics_report_artifact(
                &common,
                "diagnostics_integrity_focus",
                &report,
                "diagnostics_integrity_focus_report.schema.json",
            )?;
            write_manifest(
                &common,
                "diagnostics_integrity_focus",
                &ReceiverConfig::default(),
                None,
                &report,
            )?;
        }
    }

    Ok(())
}
