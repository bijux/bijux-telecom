mod ca_code;
mod doctor;
mod filesystem_support;
mod navigation_decode;
mod operator_guidance;
mod report_dispatch;
mod report_rendering;
mod replay_evidence;
mod rtk_processing;
mod run_quality;

pub(super) use ca_code::handle_cacode;
pub(super) use doctor::handle_doctor;
pub(super) use filesystem_support::{ensure_run_dir_exists, sha256_hex};
pub(super) use navigation_decode::handle_nav;
pub(super) use operator_guidance::{
    api_parity_report, audit_trail_report, dependency_trace_report, expert_guide_report,
    history_browse_report, integrity_focus_report, machine_catalog_report,
    operator_ergonomics_report, operator_workflow_report, route_explain_report,
    trust_class_report,
};
pub(super) use report_dispatch::handle_diagnostics;
pub(super) use report_rendering::*;
pub(super) use replay_evidence::{
    compare_run_evidence, explain_run_scope, replay_audit_report, summarize_run_diagnostics,
    verify_repro_bundle,
};
pub(super) use rtk_processing::handle_rtk;
pub(super) use run_quality::{
    advanced_gate_report, artifact_inventory_report, benchmark_summary_report,
    channel_summary_report, debug_plan_report, export_bundle_report, medium_gate_report,
    operator_status_report,
};

include!("tests.rs");
