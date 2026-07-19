use super::*;

pub(crate) fn operator_map_report() -> serde_json::Value {
    serde_json::json!({
        "schema_version": 1,
        "operator_map": [
            {
                "workflow": "run",
                "goal": "execute receiver pipeline",
                "command": "bijux gnss run --dataset <id> --config <profile.toml>"
            },
            {
                "workflow": "diagnose",
                "goal": "inspect replay scope, cache behavior, and artifact integrity",
                "command": "bijux gnss diagnostics explain --run-dir <run_dir>"
            },
            {
                "workflow": "replay_audit",
                "goal": "prove deterministic match or classify drift",
                "command": "bijux gnss diagnostics replay-audit --baseline-run-dir <run_a> --candidate-run-dir <run_b>"
            },
            {
                "workflow": "compare",
                "goal": "compare quality deltas and reproducibility evidence",
                "command": "bijux gnss diagnostics compare --baseline-run-dir <run_a> --candidate-run-dir <run_b>"
            },
            {
                "workflow": "export_bundle",
                "goal": "prepare reproducible review bundle for triage",
                "command": "bijux gnss diagnostics export-bundle --run-dir <run_dir>"
            }
        ]
    })
}

pub(crate) fn layered_report(
    critical: Vec<String>,
    details: serde_json::Value,
) -> serde_json::Value {
    serde_json::json!({
        "critical": critical,
        "details": details
    })
}

pub(crate) fn summarize_critical_entries(
    entries: &[bijux_gnss_infra::api::core::DiagnosticSummaryEntry],
) -> Vec<String> {
    let mut critical = Vec::new();
    for entry in entries {
        let severity = format!("{:?}", entry.severity).to_lowercase();
        if severity == "error" || severity == "fatal" {
            critical.push(format!("{}:{}:count={}", severity, entry.code, entry.count));
        }
    }
    critical
}

pub(crate) fn workflow_map_report() -> serde_json::Value {
    serde_json::json!({
        "schema_version": 1,
        "workflow": [
            {
                "stage": "run",
                "command": "bijux gnss run --dataset <id> --config <profile.toml>",
                "artifacts": ["manifest.json", "run_report.json", "artifacts/"]
            },
            {
                "stage": "inspect",
                "command": "bijux gnss inspect --dataset <id> --report json",
                "artifacts": ["inspect_report.json", "summary.json"]
            },
            {
                "stage": "validate",
                "command": "bijux gnss validate --dataset <id> --eph <eph.json> --reference <ref.jsonl>",
                "artifacts": ["validation_report.json", "validation_evidence_bundle.json"]
            },
            {
                "stage": "diagnose",
                "command": "bijux gnss diagnostics explain --run-dir <run_dir>",
                "artifacts": ["diagnostics_explain_report.json", "summary.json"]
            },
            {
                "stage": "replay",
                "command": "bijux gnss diagnostics verify-repro --run-dir <run_dir>",
                "artifacts": ["diagnostics_verify_repro_report.json", "summary.json"]
            },
            {
                "stage": "compare",
                "command": "bijux gnss diff --run-a <run_a> --run-b <run_b>",
                "artifacts": ["compare summary json on stdout"]
            }
        ]
    })
}

pub(crate) fn print_operator_map_table(report: &serde_json::Value) {
    println!("operator workflow map");
    if let Some(rows) = report.get("operator_map").and_then(|v| v.as_array()) {
        for row in rows {
            let workflow = row.get("workflow").and_then(|v| v.as_str()).unwrap_or("unknown");
            let command = row.get("command").and_then(|v| v.as_str()).unwrap_or("unknown");
            println!("{workflow}\t{command}");
        }
    }
}

pub(crate) fn write_diagnostics_report_artifact(
    common: &CommonArgs,
    command: &str,
    report: &serde_json::Value,
    schema_name: &str,
) -> Result<PathBuf> {
    let out_dir = artifacts_dir(common, command, None)?;
    let report_path = out_dir.join("report.json");
    fs::write(&report_path, serde_json::to_string_pretty(report)?)?;
    let schema = schema_path(schema_name);
    if schema.exists() {
        validate_json_schema(&schema, &report_path, false)?;
    }
    Ok(report_path)
}

pub(crate) fn print_workflow_table(report: &serde_json::Value) {
    println!("GNSS workflow map");
    if let Some(stages) = report.get("workflow").and_then(|v| v.as_array()) {
        for stage in stages {
            let label = stage.get("stage").and_then(|v| v.as_str()).unwrap_or("unknown");
            let command = stage.get("command").and_then(|v| v.as_str()).unwrap_or("unknown");
            println!("{label}\t{command}");
        }
    }
}

pub(crate) fn print_diagnostics_summary_table(
    entries: Option<&Vec<serde_json::Value>>,
    top: usize,
) {
    println!("Diagnostics summary (top {}):", top);
    if let Some(items) = entries {
        for entry in items.iter().take(top.max(1)) {
            let code = entry.get("code").and_then(|v| v.as_str()).unwrap_or("unknown");
            let severity = entry.get("severity").and_then(|v| v.as_str()).unwrap_or("unknown");
            let count = entry.get("count").and_then(|v| v.as_u64()).unwrap_or(0);
            println!("{code}\t{severity}\tcount={count}");
        }
    }
}

pub(crate) fn print_diagnostics_explain_table(report: &serde_json::Value) {
    let command = report
        .get("replay_scope")
        .and_then(|v| v.get("command"))
        .and_then(|v| v.as_str())
        .unwrap_or("unknown");
    let deterministic = report
        .get("replay_scope")
        .and_then(|v| v.get("deterministic"))
        .and_then(|v| v.as_bool())
        .unwrap_or(false);
    let artifacts = report
        .get("artifact_integrity")
        .and_then(|v| v.get("artifact_files_scanned"))
        .and_then(|v| v.as_u64())
        .unwrap_or(0);
    let corrupted = report
        .get("artifact_integrity")
        .and_then(|v| v.get("corrupted_files"))
        .and_then(|v| v.as_array())
        .map_or(0usize, Vec::len);
    println!("command\t{command}");
    println!("deterministic\t{deterministic}");
    println!("artifact_files\t{artifacts}");
    println!("corrupted_files\t{corrupted}");
}

pub(crate) fn print_verify_repro_table(report: &serde_json::Value) {
    let audit_ok = report.get("audit_ok").and_then(|v| v.as_bool()).unwrap_or(false);
    let fingerprint =
        report.get("replay_fingerprint").and_then(|v| v.as_str()).unwrap_or("missing");
    let issue_count = report.get("issues").and_then(|v| v.as_array()).map_or(0usize, Vec::len);
    println!("audit_ok\t{audit_ok}");
    println!("issues\t{issue_count}");
    println!("replay_fingerprint\t{fingerprint}");
}

pub(crate) fn print_compare_report_table(report: &serde_json::Value) {
    let fingerprint_match = report
        .get("reproducibility")
        .and_then(|v| v.get("fingerprint_match"))
        .and_then(|v| v.as_bool())
        .unwrap_or(false);
    let baseline_audit = report
        .get("reproducibility")
        .and_then(|v| v.get("baseline_audit_ok"))
        .and_then(|v| v.as_bool())
        .unwrap_or(false);
    let candidate_audit = report
        .get("reproducibility")
        .and_then(|v| v.get("candidate_audit_ok"))
        .and_then(|v| v.as_bool())
        .unwrap_or(false);
    let rms_delta = report
        .get("quality")
        .and_then(|v| v.get("mean_rms_delta_m"))
        .and_then(|v| v.as_f64())
        .unwrap_or(0.0);
    let cn0_delta = report
        .get("quality")
        .and_then(|v| v.get("mean_cn0_delta_dbhz"))
        .and_then(|v| v.as_f64())
        .unwrap_or(0.0);
    println!("fingerprint_match\t{fingerprint_match}");
    println!("baseline_audit_ok\t{baseline_audit}");
    println!("candidate_audit_ok\t{candidate_audit}");
    println!("mean_rms_delta_m\t{rms_delta:.6}");
    println!("mean_cn0_delta_dbhz\t{cn0_delta:.6}");
}

pub(crate) fn print_replay_audit_table(report: &serde_json::Value) {
    let class = report.get("classification").and_then(|v| v.as_str()).unwrap_or("unknown");
    let baseline_command =
        report.get("baseline_command").and_then(|v| v.as_str()).unwrap_or("unknown");
    let candidate_command =
        report.get("candidate_command").and_then(|v| v.as_str()).unwrap_or("unknown");
    let reasons = report
        .get("reasons")
        .and_then(|v| v.as_array())
        .map(|vals| vals.iter().filter_map(|v| v.as_str()).collect::<Vec<_>>().join(","))
        .unwrap_or_default();
    println!("classification\t{class}");
    println!("baseline_command\t{baseline_command}");
    println!("candidate_command\t{candidate_command}");
    println!("reasons\t{reasons}");
}

pub(crate) fn print_advanced_gate_table(report: &serde_json::Value) {
    let mode = report.get("mode").and_then(|v| v.as_str()).unwrap_or("unknown");
    let passed = report.get("gate_passed").and_then(|v| v.as_bool()).unwrap_or(false);
    let maturity = report
        .get("support")
        .and_then(|v| v.get("maturity"))
        .and_then(|v| v.as_str())
        .unwrap_or("unknown");
    let evidence_supported = report
        .get("evidence")
        .and_then(|v| v.get("claim_guard_supported"))
        .and_then(|v| v.as_bool())
        .unwrap_or(false);
    println!("mode\t{mode}");
    println!("gate_passed\t{passed}");
    println!("maturity\t{maturity}");
    println!("claim_guard_supported\t{evidence_supported}");
}

pub(crate) fn print_artifact_inventory_table(report: &serde_json::Value) {
    println!("artifact inventory");
    if let Some(groups) = report.get("groups").and_then(|v| v.as_object()) {
        for (group, payload) in groups {
            let files = payload.get("file_count").and_then(|v| v.as_u64()).unwrap_or(0);
            println!("{group}\tfiles={files}");
        }
    }
}

pub(crate) fn print_debug_plan_table(report: &serde_json::Value) {
    println!("debug plan");
    if let Some(stages) = report.get("stages").and_then(|v| v.as_array()) {
        for stage in stages {
            let name = stage.get("stage").and_then(|v| v.as_str()).unwrap_or("unknown");
            let check =
                stage.get("recommended_check").and_then(|v| v.as_str()).unwrap_or("unknown");
            println!("{name}\t{check}");
        }
    }
}

pub(crate) fn print_benchmark_summary_table(report: &serde_json::Value) {
    let nav_epochs = report
        .get("summary")
        .and_then(|v| v.get("nav_epochs"))
        .and_then(|v| v.as_u64())
        .unwrap_or(0);
    let obs_epochs = report
        .get("summary")
        .and_then(|v| v.get("obs_epochs"))
        .and_then(|v| v.as_u64())
        .unwrap_or(0);
    println!("obs_epochs\t{obs_epochs}");
    println!("nav_epochs\t{nav_epochs}");
}

pub(crate) fn print_medium_gate_table(report: &serde_json::Value) {
    let passed = report.get("gate_passed").and_then(|v| v.as_bool()).unwrap_or(false);
    println!("gate_passed\t{passed}");
    let reasons = report
        .get("reasons")
        .and_then(|v| v.as_array())
        .map(|rows| rows.iter().filter_map(|v| v.as_str()).collect::<Vec<_>>().join(","))
        .unwrap_or_default();
    println!("reasons\t{reasons}");
}

pub(crate) fn print_operator_status_table(report: &serde_json::Value) {
    let mode = report
        .get("status")
        .and_then(|v| v.get("run_state"))
        .and_then(|v| v.as_str())
        .unwrap_or("unknown");
    let quality = report
        .get("status")
        .and_then(|v| v.get("quality_state"))
        .and_then(|v| v.as_str())
        .unwrap_or("unknown");
    let evidence = report
        .get("status")
        .and_then(|v| v.get("evidence_state"))
        .and_then(|v| v.as_str())
        .unwrap_or("unknown");
    println!("run_state\t{mode}");
    println!("quality_state\t{quality}");
    println!("evidence_state\t{evidence}");
}

pub(crate) fn print_channel_summary_table(report: &serde_json::Value) {
    let channels = report
        .get("summary")
        .and_then(|v| v.get("channel_count"))
        .and_then(|v| v.as_u64())
        .unwrap_or(0);
    let epochs = report
        .get("summary")
        .and_then(|v| v.get("obs_epochs"))
        .and_then(|v| v.as_u64())
        .unwrap_or(0);
    println!("channel_count\t{channels}");
    println!("obs_epochs\t{epochs}");
}

pub(crate) fn print_export_bundle_table(report: &serde_json::Value) {
    let path = report.get("bundle_dir").and_then(|v| v.as_str()).unwrap_or("unknown");
    let files = report.get("file_count").and_then(|v| v.as_u64()).unwrap_or(0);
    println!("bundle_dir\t{path}");
    println!("file_count\t{files}");
}

pub(crate) fn print_machine_catalog_table(report: &serde_json::Value) {
    println!("machine-readable report catalog");
    if let Some(rows) = report.get("reports").and_then(|v| v.as_array()) {
        for row in rows {
            let name = row.get("name").and_then(|v| v.as_str()).unwrap_or("unknown");
            let schema = row.get("schema").and_then(|v| v.as_str()).unwrap_or("unknown");
            println!("{name}\t{schema}");
        }
    }
}

pub(crate) fn print_api_parity_table(report: &serde_json::Value) {
    println!("cli/api parity");
    if let Some(rows) = report.get("workflows").and_then(|v| v.as_array()) {
        for row in rows {
            let workflow = row.get("workflow").and_then(|v| v.as_str()).unwrap_or("unknown");
            let cli = row.get("cli_supported").and_then(|v| v.as_bool()).unwrap_or(false);
            let api = row.get("api_supported").and_then(|v| v.as_bool()).unwrap_or(false);
            println!("{workflow}\tcli={cli}\tapi={api}");
        }
    }
}

pub(crate) fn print_expert_guide_table(report: &serde_json::Value) {
    println!("expert guide");
    if let Some(flows) = report.get("flows").and_then(|v| v.as_array()) {
        for flow in flows {
            let name = flow.get("name").and_then(|v| v.as_str()).unwrap_or("unknown");
            let first = flow
                .get("steps")
                .and_then(|v| v.as_array())
                .and_then(|rows| rows.first())
                .and_then(|v| v.as_str())
                .unwrap_or("no-steps");
            println!("{name}\t{first}");
        }
    }
}

pub(crate) fn print_history_browse_table(report: &serde_json::Value) {
    println!("history browse");
    if let Some(runs) = report.get("runs").and_then(|v| v.as_array()) {
        for run in runs {
            let dir = run.get("run_dir").and_then(|v| v.as_str()).unwrap_or("unknown");
            let cmd = run.get("command").and_then(|v| v.as_str()).unwrap_or("unknown");
            let dataset = run.get("dataset_id").and_then(|v| v.as_str()).unwrap_or("unknown");
            println!("{dir}\t{cmd}\t{dataset}");
        }
    }
}

pub(crate) fn print_route_explain_table(report: &serde_json::Value) {
    let topic = report.get("topic").and_then(|v| v.as_str()).unwrap_or("unknown");
    println!("route topic\t{topic}");
    if let Some(steps) = report.get("steps").and_then(|v| v.as_array()) {
        for step in steps {
            if let Some(cmd) = step.get("command").and_then(|v| v.as_str()) {
                println!("{cmd}");
            }
        }
    }
}

pub(crate) fn print_operator_workflow_table(report: &serde_json::Value) {
    let profile = report.get("profile").and_then(|v| v.as_str()).unwrap_or("unknown");
    println!("operator workflow\t{profile}");
    if let Some(steps) = report.get("steps").and_then(|v| v.as_array()) {
        for step in steps {
            let label = step.get("label").and_then(|v| v.as_str()).unwrap_or("unknown");
            let command = step.get("command").and_then(|v| v.as_str()).unwrap_or("unknown");
            println!("{label}\t{command}");
        }
    }
}

pub(crate) fn print_operator_ergonomics_table(report: &serde_json::Value) {
    let score = report.get("ergonomics_score").and_then(|v| v.as_f64()).unwrap_or(0.0);
    let gate = report
        .get("rigor")
        .and_then(|v| v.get("medium_gate_passed"))
        .and_then(|v| v.as_bool())
        .unwrap_or(false);
    let evidence = report
        .get("rigor")
        .and_then(|v| v.get("evidence_state"))
        .and_then(|v| v.as_str())
        .unwrap_or("unknown");
    println!("ergonomics_score\t{score:.2}");
    println!("medium_gate_passed\t{gate}");
    println!("evidence_state\t{evidence}");
}

pub(crate) fn print_audit_trail_table(report: &serde_json::Value) {
    let replay = report
        .get("replay_controls")
        .and_then(|v| v.get("deterministic"))
        .and_then(|v| v.as_bool())
        .unwrap_or(false);
    let overrides =
        report.get("override_events").and_then(|v| v.as_array()).map_or(0usize, Vec::len);
    let exceptions =
        report.get("policy_exceptions").and_then(|v| v.as_array()).map_or(0usize, Vec::len);
    println!("deterministic_replay\t{replay}");
    println!("override_events\t{overrides}");
    println!("policy_exceptions\t{exceptions}");
}

pub(crate) fn print_dependency_trace_table(report: &serde_json::Value) {
    let toolchain = report
        .get("tooling")
        .and_then(|v| v.get("toolchain"))
        .and_then(|v| v.as_str())
        .unwrap_or("unknown");
    let correction_rows =
        report.get("corrections").and_then(|v| v.get("rows")).and_then(|v| v.as_u64()).unwrap_or(0);
    let env_refs = report
        .get("environment")
        .and_then(|v| v.get("reference_count"))
        .and_then(|v| v.as_u64())
        .unwrap_or(0);
    println!("toolchain\t{toolchain}");
    println!("correction_rows\t{correction_rows}");
    println!("environment_references\t{env_refs}");
}

pub(crate) fn print_trust_class_table(report: &serde_json::Value) {
    let class = report.get("trust_class").and_then(|v| v.as_str()).unwrap_or("unknown");
    let rationale = report
        .get("rationale")
        .and_then(|v| v.as_array())
        .map(|rows| rows.iter().filter_map(|v| v.as_str()).collect::<Vec<_>>().join(","))
        .unwrap_or_default();
    println!("trust_class\t{class}");
    println!("rationale\t{rationale}");
}

pub(crate) fn print_integrity_focus_table(report: &serde_json::Value) {
    let class = report.get("trust_class").and_then(|v| v.as_str()).unwrap_or("unknown");
    let score = report.get("engineering_trust_score").and_then(|v| v.as_f64()).unwrap_or(0.0);
    let blockers =
        report.get("blocking_findings").and_then(|v| v.as_array()).map_or(0usize, Vec::len);
    println!("trust_class\t{class}");
    println!("engineering_trust_score\t{score:.2}");
    println!("blocking_findings\t{blockers}");
}
