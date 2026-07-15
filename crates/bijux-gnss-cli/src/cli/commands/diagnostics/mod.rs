mod ca_code;
mod doctor;
mod filesystem_support;
mod navigation_decode;
mod report_dispatch;
mod report_rendering;
mod rtk_processing;

pub(super) use ca_code::handle_cacode;
pub(super) use doctor::handle_doctor;
pub(super) use filesystem_support::{ensure_run_dir_exists, sha256_hex};
pub(super) use navigation_decode::handle_nav;
pub(super) use report_dispatch::handle_diagnostics;
pub(super) use report_rendering::*;
pub(super) use rtk_processing::handle_rtk;

include!("replay_evidence.rs");
include!("run_quality.rs");
include!("operator_guidance.rs");
include!("tests.rs");
