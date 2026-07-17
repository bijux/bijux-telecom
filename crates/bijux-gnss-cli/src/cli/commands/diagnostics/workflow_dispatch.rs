use super::*;

pub(super) fn handle_operator_map(common: CommonArgs) -> Result<()> {
    let report = operator_map_report();
    report_publishing::publish_diagnostics_report(
        &common,
        "diagnostics_operator_map",
        &report,
        "diagnostics_operator_map_report.schema.json",
        print_operator_map_table,
    )
}

pub(super) fn handle_workflow(common: CommonArgs) -> Result<()> {
    let report = workflow_map_report();
    report_publishing::publish_diagnostics_report(
        &common,
        "diagnostics_workflow",
        &report,
        "diagnostics_workflow_report.schema.json",
        print_workflow_table,
    )
}
