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
            quality_dispatch::handle_advanced_gate(common, run_dir, mode, strict)?
        }
        DiagnosticsCommand::ArtifactInventory { common, run_dir } => {
            quality_dispatch::handle_artifact_inventory(common, run_dir)?
        }
        DiagnosticsCommand::DebugPlan { common, run_dir } => {
            quality_dispatch::handle_debug_plan(common, run_dir)?
        }
        DiagnosticsCommand::BenchmarkSummary { common, run_dir } => {
            quality_dispatch::handle_benchmark_summary(common, run_dir)?
        }
        DiagnosticsCommand::MediumGate { common, run_dir, strict } => {
            quality_dispatch::handle_medium_gate(common, run_dir, strict)?
        }
        DiagnosticsCommand::OperatorStatus { common, run_dir } => {
            quality_dispatch::handle_operator_status(common, run_dir)?
        }
        DiagnosticsCommand::ChannelSummary { common, run_dir } => {
            quality_dispatch::handle_channel_summary(common, run_dir)?
        }
        DiagnosticsCommand::ExportBundle { common, run_dir, out_dir } => {
            quality_dispatch::handle_export_bundle(common, run_dir, out_dir)?
        }
        DiagnosticsCommand::MachineCatalog { common } => {
            guidance_dispatch::handle_machine_catalog(common)?
        }
        DiagnosticsCommand::ApiParity { common } => guidance_dispatch::handle_api_parity(common)?,
        DiagnosticsCommand::ExpertGuide { common } => {
            guidance_dispatch::handle_expert_guide(common)?
        }
        DiagnosticsCommand::HistoryBrowse { common, root_dir, limit } => {
            guidance_dispatch::handle_history_browse(common, root_dir, limit)?
        }
        DiagnosticsCommand::RouteExplain { common, topic } => {
            guidance_dispatch::handle_route_explain(common, topic)?
        }
        DiagnosticsCommand::OperatorWorkflow { common, profile } => {
            guidance_dispatch::handle_operator_workflow(common, profile)?
        }
        DiagnosticsCommand::OperatorErgonomics { common, run_dir } => {
            guidance_dispatch::handle_operator_ergonomics(common, run_dir)?
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
