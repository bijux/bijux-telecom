#![forbid(unsafe_code)]

include!("cli/mod.rs");
include!("cli/command_catalog.rs");
include!("cli/command_line.rs");
include!("cli/report.rs");
#[path = "cli/command_support/mod.rs"]
mod command_support;
use command_support::*;
include!("cli/commands/ingest.rs");
include!("cli/commands/run_pipeline.rs");
include!("cli/commands/synthetic.rs");
mod diagnostics {
    use super::*;

    include!("cli/commands/diagnostics/mod.rs");
}
use diagnostics::{handle_cacode, handle_diagnostics, handle_doctor, handle_nav, handle_rtk};
include!("cli/commands/validate.rs");
include!("cli/commands/analyze.rs");
include!("cli/command_runtime.rs");
include!("cli/commands/artifact.rs");
