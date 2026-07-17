use super::*;

pub(super) fn publish_diagnostics_report<T, F>(
    common: &CommonArgs,
    command: &str,
    report: &T,
    schema_name: &str,
    print_table: F,
) -> Result<()>
where
    T: serde::Serialize,
    F: FnOnce(&T),
{
    let _ = runtime_config_from_env(common, None);
    match common.report {
        ReportFormat::Table => print_table(report),
        ReportFormat::Json => emit_report(common, command, report)?,
    }
    let report_value = serde_json::to_value(report)?;
    write_diagnostics_report_artifact(common, command, &report_value, schema_name)?;
    write_manifest(common, command, &ReceiverConfig::default(), None, report)?;
    Ok(())
}
