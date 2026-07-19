use super::*;

pub(super) fn handle_summarize(common: CommonArgs, run_dir: PathBuf, top: usize) -> Result<()> {
    let events = summarize_run_diagnostics(&run_dir)?;
    let summary = bijux_gnss_infra::api::core::aggregate_diagnostics(&events);
    let mut entries = summary.entries.clone();
    entries.sort_by(|left, right| right.count.cmp(&left.count));
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
    report_publishing::publish_diagnostics_report(
        &common,
        "diagnostics_summarize",
        &report,
        "diagnostics_summary_report.schema.json",
        |report| {
            print_diagnostics_summary_table(
                report.get("entries").and_then(|value| value.as_array()),
                top,
            )
        },
    )
}

pub(super) fn handle_explain(common: CommonArgs, run_dir: PathBuf) -> Result<()> {
    let summary = explain_run_scope(&run_dir)?;
    report_publishing::publish_diagnostics_report(
        &common,
        "diagnostics_explain",
        &summary,
        "diagnostics_explain_report.schema.json",
        print_diagnostics_explain_table,
    )
}

pub(super) fn handle_verify_repro(common: CommonArgs, run_dir: PathBuf) -> Result<()> {
    let report = verify_repro_bundle(&run_dir)?;
    report_publishing::publish_diagnostics_report(
        &common,
        "diagnostics_verify_repro",
        &report,
        "diagnostics_verify_repro_report.schema.json",
        print_verify_repro_table,
    )
}

pub(super) fn handle_compare(
    common: CommonArgs,
    baseline_run_dir: PathBuf,
    candidate_run_dir: PathBuf,
) -> Result<()> {
    let report = compare_run_evidence(&baseline_run_dir, &candidate_run_dir)?;
    report_publishing::publish_diagnostics_report(
        &common,
        "diagnostics_compare",
        &report,
        "diagnostics_compare_report.schema.json",
        print_compare_report_table,
    )
}

pub(super) fn handle_replay_audit(
    common: CommonArgs,
    baseline_run_dir: PathBuf,
    candidate_run_dir: PathBuf,
) -> Result<()> {
    let report = replay_audit_report(&baseline_run_dir, &candidate_run_dir)?;
    report_publishing::publish_diagnostics_report(
        &common,
        "diagnostics_replay_audit",
        &report,
        "diagnostics_replay_audit_report.schema.json",
        print_replay_audit_table,
    )
}
