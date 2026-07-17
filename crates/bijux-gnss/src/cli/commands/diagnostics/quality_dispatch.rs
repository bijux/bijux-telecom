use super::*;

pub(super) fn handle_advanced_gate(
    common: CommonArgs,
    run_dir: PathBuf,
    mode: AdvancedGateMode,
    strict: bool,
) -> Result<()> {
    let _ = runtime_config_from_env(&common, None);
    let report = advanced_gate_report(&run_dir, mode)?;
    let passed = report.get("gate_passed").and_then(|value| value.as_bool()).unwrap_or(false);
    if strict && !passed {
        return Err(classified_error(
            CliErrorClass::UnsupportedScience,
            format!("advanced gate failed for mode={:?} run_dir={}", mode, run_dir.display()),
        ));
    }
    report_publishing::publish_diagnostics_report(
        &common,
        "diagnostics_advanced_gate",
        &report,
        "diagnostics_advanced_gate_report.schema.json",
        print_advanced_gate_table,
    )
}

pub(super) fn handle_artifact_inventory(common: CommonArgs, run_dir: PathBuf) -> Result<()> {
    let report = artifact_inventory_report(&run_dir)?;
    report_publishing::publish_diagnostics_report(
        &common,
        "diagnostics_artifact_inventory",
        &report,
        "diagnostics_artifact_inventory_report.schema.json",
        print_artifact_inventory_table,
    )
}

pub(super) fn handle_debug_plan(common: CommonArgs, run_dir: PathBuf) -> Result<()> {
    let report = debug_plan_report(&run_dir)?;
    report_publishing::publish_diagnostics_report(
        &common,
        "diagnostics_debug_plan",
        &report,
        "diagnostics_debug_plan_report.schema.json",
        print_debug_plan_table,
    )
}

pub(super) fn handle_benchmark_summary(common: CommonArgs, run_dir: PathBuf) -> Result<()> {
    let report = benchmark_summary_report(&run_dir)?;
    report_publishing::publish_diagnostics_report(
        &common,
        "diagnostics_benchmark_summary",
        &report,
        "diagnostics_benchmark_summary_report.schema.json",
        print_benchmark_summary_table,
    )
}

pub(super) fn handle_medium_gate(common: CommonArgs, run_dir: PathBuf, strict: bool) -> Result<()> {
    let _ = runtime_config_from_env(&common, None);
    let report = medium_gate_report(&run_dir)?;
    let passed = report.get("gate_passed").and_then(|value| value.as_bool()).unwrap_or(false);
    if strict && !passed {
        return Err(classified_error(
            CliErrorClass::UnsupportedScience,
            format!("medium gate failed for run_dir={}", run_dir.display()),
        ));
    }
    report_publishing::publish_diagnostics_report(
        &common,
        "diagnostics_medium_gate",
        &report,
        "diagnostics_medium_gate_report.schema.json",
        print_medium_gate_table,
    )
}

pub(super) fn handle_operator_status(common: CommonArgs, run_dir: PathBuf) -> Result<()> {
    let report = operator_status_report(&run_dir)?;
    report_publishing::publish_diagnostics_report(
        &common,
        "diagnostics_operator_status",
        &report,
        "diagnostics_operator_status_report.schema.json",
        print_operator_status_table,
    )
}

pub(super) fn handle_channel_summary(common: CommonArgs, run_dir: PathBuf) -> Result<()> {
    let report = channel_summary_report(&run_dir)?;
    report_publishing::publish_diagnostics_report(
        &common,
        "diagnostics_channel_summary",
        &report,
        "diagnostics_channel_summary_report.schema.json",
        print_channel_summary_table,
    )
}

pub(super) fn handle_export_bundle(
    common: CommonArgs,
    run_dir: PathBuf,
    out_dir: Option<PathBuf>,
) -> Result<()> {
    let report = export_bundle_report(&run_dir, out_dir.as_ref())?;
    report_publishing::publish_diagnostics_report(
        &common,
        "diagnostics_export_bundle",
        &report,
        "diagnostics_export_bundle_report.schema.json",
        print_export_bundle_table,
    )
}
