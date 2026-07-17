fn execute_gnss_command(command: GnssCommand) -> Result<()> {
    match command {
        cmd @ GnssCommand::CaCode { .. } => handle_ca_code(cmd),
        cmd @ GnssCommand::Acquire { .. } => handle_acquire(cmd),
        cmd @ GnssCommand::Track { .. } => handle_track(cmd),
        cmd @ GnssCommand::Nav { .. } => handle_nav(cmd),
        cmd @ GnssCommand::Pvt { .. } => handle_pvt(cmd),
        cmd @ GnssCommand::Inspect { .. } => handle_inspect(cmd),
        cmd @ GnssCommand::Rtk { .. } => handle_rtk(cmd),
        cmd @ GnssCommand::Experiment { .. } => handle_experiment(cmd),
        cmd @ GnssCommand::ExportSyntheticIq { .. } => handle_export_synthetic_iq(cmd),
        cmd @ GnssCommand::ValidateSyntheticIq { .. } => handle_validate_synthetic_iq(cmd),
        cmd @ GnssCommand::ValidateSyntheticNavigation { .. } => {
            handle_validate_synthetic_navigation(cmd)
        }
        cmd @ GnssCommand::MeasureSyntheticQuantization { .. } => {
            handle_measure_synthetic_quantization(cmd)
        }
        cmd @ GnssCommand::ValidateConfig { .. } => handle_validate_config(cmd),
        cmd @ GnssCommand::Config { .. } => handle_config(cmd),
        cmd @ GnssCommand::ValidateArtifacts { .. } => handle_validate_artifacts(cmd),
        cmd @ GnssCommand::ValidateSidecar { .. } => handle_validate_sidecar(cmd),
        cmd @ GnssCommand::Analyze { .. } => handle_analyze(cmd),
        cmd @ GnssCommand::Diff { .. } => handle_diff(cmd),
        cmd @ GnssCommand::Artifact { .. } => handle_artifact(cmd),
        cmd @ GnssCommand::Diagnostics { .. } => handle_diagnostics(cmd),
        cmd @ GnssCommand::ConfigUpgrade { .. } => handle_config_upgrade(cmd),
        cmd @ GnssCommand::ConfigSchema { .. } => handle_config_schema(cmd),
        cmd @ GnssCommand::Validate { .. } => handle_validate(cmd),
        cmd @ GnssCommand::ValidateReference { .. } => handle_validate_reference(cmd),
        cmd @ GnssCommand::ValidateCapture { .. } => handle_validate_capture(cmd),
        cmd @ GnssCommand::Run { .. } => handle_run(cmd),
        cmd @ GnssCommand::Rinex { .. } => handle_rinex(cmd),
        cmd @ GnssCommand::Doctor { .. } => handle_doctor(cmd),
    }
}

mod runtime_environment {
    use super::*;

    include!("command_runtime/runtime_environment.rs");
}

#[cfg(test)]
pub(crate) use runtime_environment::capture_start_gps_time;
pub(crate) use runtime_environment::{runtime_config_from_capture_start, runtime_config_from_env};

mod dataset_inspection {
    use super::*;

    include!("command_runtime/dataset_inspection.rs");
}

pub(crate) use dataset_inspection::{inspect_dataset, print_inspect_table};

mod acquisition_reporting {
    use super::*;

    include!("command_runtime/acquisition_reporting.rs");
}

pub(crate) use acquisition_reporting::print_acquisition_table;

mod synthetic_reporting {
    use super::*;

    include!("command_runtime/synthetic_reporting.rs");
}

pub(crate) use synthetic_reporting::{
    print_synthetic_iq_export_table, print_synthetic_iq_validation_table,
    print_synthetic_navigation_validation_table, print_synthetic_quantization_measurement_table,
};

fn format_optional_degrees(value: Option<f64>) -> String {
    value.map(|degrees| format!("{degrees:.6}")).unwrap_or_else(|| "n/a".to_string())
}

fn format_optional_percent(value: Option<f64>) -> String {
    value.map(|pct| format!("{pct:.3}")).unwrap_or_else(|| "n/a".to_string())
}

fn format_optional_reason(value: Option<&str>) -> String {
    value.unwrap_or("n/a").to_string()
}

#[cfg(feature = "tracing")]
fn init_tracing() {
    let _ = tracing_subscriber::fmt().with_env_filter("info").with_target(false).try_init();
}

#[cfg(not(feature = "tracing"))]
fn init_tracing() {}

fn main() -> Result<()> {
    init_tracing();
    let command_line = CommandLine::parse();
    match command_line.command {
        ApplicationCommand::Gnss { command } => execute_gnss_command(command),
    }
}
