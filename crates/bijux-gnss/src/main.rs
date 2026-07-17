#![forbid(unsafe_code)]

include!("cli/execution_support.rs");
include!("cli/command_catalog/mod.rs");
include!("cli/command_line.rs");
include!("cli/report.rs");
#[path = "cli/command_support/mod.rs"]
mod command_support;
use command_support::*;
#[path = "cli/commands/mod.rs"]
mod commands;
use commands::*;
include!("cli/command_runtime.rs");
