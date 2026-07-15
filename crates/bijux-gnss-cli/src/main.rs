#![forbid(unsafe_code)]

include!("cli/mod.rs");
include!("cli/args_commands.rs");
include!("cli/args.rs");
include!("cli/report.rs");
include!("cli/output.rs");
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
include!("cli/run_command.rs");
include!("cli/commands/artifact.rs");
