use super::*;

pub(super) fn handle_machine_catalog(common: CommonArgs) -> Result<()> {
    let report = machine_catalog_report();
    report_publishing::publish_diagnostics_report(
        &common,
        "diagnostics_machine_catalog",
        &report,
        "diagnostics_machine_catalog_report.schema.json",
        print_machine_catalog_table,
    )
}

pub(super) fn handle_api_parity(common: CommonArgs) -> Result<()> {
    let report = api_parity_report();
    report_publishing::publish_diagnostics_report(
        &common,
        "diagnostics_api_parity",
        &report,
        "diagnostics_api_parity_report.schema.json",
        print_api_parity_table,
    )
}

pub(super) fn handle_expert_guide(common: CommonArgs) -> Result<()> {
    let report = expert_guide_report();
    report_publishing::publish_diagnostics_report(
        &common,
        "diagnostics_expert_guide",
        &report,
        "diagnostics_expert_guide_report.schema.json",
        print_expert_guide_table,
    )
}

pub(super) fn handle_history_browse(
    common: CommonArgs,
    root_dir: PathBuf,
    limit: usize,
) -> Result<()> {
    let report = history_browse_report(&root_dir, limit)?;
    report_publishing::publish_diagnostics_report(
        &common,
        "diagnostics_history_browse",
        &report,
        "diagnostics_history_browse_report.schema.json",
        print_history_browse_table,
    )
}

pub(super) fn handle_route_explain(common: CommonArgs, topic: RouteTopic) -> Result<()> {
    let report = route_explain_report(topic);
    report_publishing::publish_diagnostics_report(
        &common,
        "diagnostics_route_explain",
        &report,
        "diagnostics_route_explain_report.schema.json",
        print_route_explain_table,
    )
}

pub(super) fn handle_operator_workflow(common: CommonArgs, profile: WorkflowProfile) -> Result<()> {
    let report = operator_workflow_report(profile);
    report_publishing::publish_diagnostics_report(
        &common,
        "diagnostics_operator_workflow",
        &report,
        "diagnostics_operator_workflow_report.schema.json",
        print_operator_workflow_table,
    )
}

pub(super) fn handle_operator_ergonomics(common: CommonArgs, run_dir: PathBuf) -> Result<()> {
    let report = operator_ergonomics_report(&run_dir)?;
    report_publishing::publish_diagnostics_report(
        &common,
        "diagnostics_operator_ergonomics",
        &report,
        "diagnostics_operator_ergonomics_report.schema.json",
        print_operator_ergonomics_table,
    )
}
