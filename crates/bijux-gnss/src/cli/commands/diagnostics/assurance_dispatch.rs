use super::*;

pub(super) fn handle_audit_trail(common: CommonArgs, run_dir: PathBuf) -> Result<()> {
    let report = audit_trail_report(&run_dir)?;
    report_publishing::publish_diagnostics_report(
        &common,
        "diagnostics_audit_trail",
        &report,
        "diagnostics_audit_trail_report.schema.json",
        print_audit_trail_table,
    )
}

pub(super) fn handle_dependency_trace(common: CommonArgs, run_dir: PathBuf) -> Result<()> {
    let report = dependency_trace_report(&run_dir)?;
    report_publishing::publish_diagnostics_report(
        &common,
        "diagnostics_dependency_trace",
        &report,
        "diagnostics_dependency_trace_report.schema.json",
        print_dependency_trace_table,
    )
}

pub(super) fn handle_trust_class(common: CommonArgs, run_dir: PathBuf) -> Result<()> {
    let report = trust_class_report(&run_dir)?;
    report_publishing::publish_diagnostics_report(
        &common,
        "diagnostics_trust_class",
        &report,
        "diagnostics_trust_class_report.schema.json",
        print_trust_class_table,
    )
}

pub(super) fn handle_integrity_focus(common: CommonArgs, run_dir: PathBuf) -> Result<()> {
    let report = integrity_focus_report(&run_dir)?;
    report_publishing::publish_diagnostics_report(
        &common,
        "diagnostics_integrity_focus",
        &report,
        "diagnostics_integrity_focus_report.schema.json",
        print_integrity_focus_table,
    )
}
