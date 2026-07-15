fn machine_catalog_report() -> serde_json::Value {
    let reports = vec![
        serde_json::json!({"name": "diagnostics_operator_map", "schema": "schemas/diagnostics_operator_map_report.schema.json", "schema_version": 1}),
        serde_json::json!({"name": "diagnostics_workflow", "schema": "schemas/diagnostics_workflow_report.schema.json", "schema_version": 1}),
        serde_json::json!({"name": "diagnostics_summarize", "schema": "schemas/diagnostics_summary_report.schema.json", "schema_version": 1}),
        serde_json::json!({"name": "diagnostics_explain", "schema": "schemas/diagnostics_explain_report.schema.json", "schema_version": 1}),
        serde_json::json!({"name": "diagnostics_verify_repro", "schema": "schemas/diagnostics_verify_repro_report.schema.json", "schema_version": 1}),
        serde_json::json!({"name": "diagnostics_compare", "schema": "schemas/diagnostics_compare_report.schema.json", "schema_version": 1}),
        serde_json::json!({"name": "diagnostics_replay_audit", "schema": "schemas/diagnostics_replay_audit_report.schema.json", "schema_version": 1}),
        serde_json::json!({"name": "diagnostics_advanced_gate", "schema": "schemas/diagnostics_advanced_gate_report.schema.json", "schema_version": 1}),
        serde_json::json!({"name": "diagnostics_artifact_inventory", "schema": "schemas/diagnostics_artifact_inventory_report.schema.json", "schema_version": 1}),
        serde_json::json!({"name": "diagnostics_debug_plan", "schema": "schemas/diagnostics_debug_plan_report.schema.json", "schema_version": 1}),
        serde_json::json!({"name": "diagnostics_benchmark_summary", "schema": "schemas/diagnostics_benchmark_summary_report.schema.json", "schema_version": 1}),
        serde_json::json!({"name": "diagnostics_medium_gate", "schema": "schemas/diagnostics_medium_gate_report.schema.json", "schema_version": 1}),
        serde_json::json!({"name": "diagnostics_operator_status", "schema": "schemas/diagnostics_operator_status_report.schema.json", "schema_version": 1}),
        serde_json::json!({"name": "diagnostics_channel_summary", "schema": "schemas/diagnostics_channel_summary_report.schema.json", "schema_version": 1}),
        serde_json::json!({"name": "diagnostics_export_bundle", "schema": "schemas/diagnostics_export_bundle_report.schema.json", "schema_version": 1}),
        serde_json::json!({"name": "diagnostics_api_parity", "schema": "schemas/diagnostics_api_parity_report.schema.json", "schema_version": 1}),
        serde_json::json!({"name": "diagnostics_expert_guide", "schema": "schemas/diagnostics_expert_guide_report.schema.json", "schema_version": 1}),
        serde_json::json!({"name": "diagnostics_history_browse", "schema": "schemas/diagnostics_history_browse_report.schema.json", "schema_version": 1}),
        serde_json::json!({"name": "diagnostics_route_explain", "schema": "schemas/diagnostics_route_explain_report.schema.json", "schema_version": 1}),
        serde_json::json!({"name": "diagnostics_operator_workflow", "schema": "schemas/diagnostics_operator_workflow_report.schema.json", "schema_version": 1}),
        serde_json::json!({"name": "diagnostics_operator_ergonomics", "schema": "schemas/diagnostics_operator_ergonomics_report.schema.json", "schema_version": 1}),
        serde_json::json!({"name": "diagnostics_audit_trail", "schema": "schemas/diagnostics_audit_trail_report.schema.json", "schema_version": 1}),
        serde_json::json!({"name": "diagnostics_dependency_trace", "schema": "schemas/diagnostics_dependency_trace_report.schema.json", "schema_version": 1}),
        serde_json::json!({"name": "diagnostics_trust_class", "schema": "schemas/diagnostics_trust_class_report.schema.json", "schema_version": 1}),
        serde_json::json!({"name": "diagnostics_integrity_focus", "schema": "schemas/diagnostics_integrity_focus_report.schema.json", "schema_version": 1}),
    ];
    serde_json::json!({
        "schema_version": 1,
        "reports": reports
    })
}

fn api_parity_report() -> serde_json::Value {
    let workflows = vec![
        serde_json::json!({
            "workflow": "run",
            "cli_supported": true,
            "api_supported": true,
            "cli_command": "bijux gnss run",
            "api_surface": "prepare_run + receiver runtime pipeline"
        }),
        serde_json::json!({
            "workflow": "inspect",
            "cli_supported": true,
            "api_supported": true,
            "cli_command": "bijux gnss inspect",
            "api_surface": "FileSamples + signal metadata inspectors"
        }),
        serde_json::json!({
            "workflow": "diagnose",
            "cli_supported": true,
            "api_supported": true,
            "cli_command": "bijux gnss diagnostics explain|summarize",
            "api_surface": "artifact_validate + aggregate_diagnostics + run manifest parser"
        }),
        serde_json::json!({
            "workflow": "compare",
            "cli_supported": true,
            "api_supported": true,
            "cli_command": "bijux gnss diagnostics compare",
            "api_surface": "read_nav_solutions + read_obs_epochs + validate_reference"
        }),
        serde_json::json!({
            "workflow": "replay_audit",
            "cli_supported": true,
            "api_supported": true,
            "cli_command": "bijux gnss diagnostics replay-audit",
            "api_surface": "run manifest + artifact hash + replay fingerprint computation"
        }),
        serde_json::json!({
            "workflow": "export",
            "cli_supported": true,
            "api_supported": true,
            "cli_command": "bijux gnss diagnostics export-bundle",
            "api_surface": "run layout paths + managed artifact copy + sha256 manifest"
        }),
    ];
    let parity_ok = workflows.iter().all(|row| {
        row.get("cli_supported").and_then(|v| v.as_bool()).unwrap_or(false)
            == row.get("api_supported").and_then(|v| v.as_bool()).unwrap_or(false)
    });
    serde_json::json!({
        "schema_version": 1,
        "parity_ok": parity_ok,
        "workflows": workflows
    })
}

fn expert_guide_report() -> serde_json::Value {
    let flows = vec![
        serde_json::json!({
            "name": "integrity_triage",
            "intent": "fast integrity diagnosis without losing rigor",
            "steps": [
                "bijux gnss diagnostics medium-gate --run-dir <run_dir> --report table",
                "bijux gnss diagnostics replay-audit --baseline-run-dir <a> --candidate-run-dir <b> --report json",
                "bijux gnss diagnostics advanced-gate --run-dir <run_dir> --mode rtk --strict --report json"
            ]
        }),
        serde_json::json!({
            "name": "artifact_forensics",
            "intent": "explain artifact state before deep debugging",
            "steps": [
                "bijux gnss diagnostics artifact-inventory --run-dir <run_dir> --report json",
                "bijux gnss diagnostics explain --run-dir <run_dir> --report json",
                "bijux gnss diagnostics export-bundle --run-dir <run_dir>"
            ]
        }),
        serde_json::json!({
            "name": "solver_regression",
            "intent": "compare navigation behavior with evidence context",
            "steps": [
                "bijux gnss diagnostics compare --baseline-run-dir <a> --candidate-run-dir <b> --report json",
                "bijux gnss diagnostics channel-summary --run-dir <b> --report json",
                "bijux gnss diagnostics benchmark-summary --run-dir <b> --report json"
            ]
        }),
    ];
    serde_json::json!({
        "schema_version": 1,
        "flows": flows
    })
}

fn history_browse_report(root_dir: &Path, limit: usize) -> Result<serde_json::Value> {
    if !root_dir.exists() || !root_dir.is_dir() {
        return Err(classified_error(
            CliErrorClass::OperatorMisconfiguration,
            format!("history root not found: {}", root_dir.display()),
        ));
    }
    let mut runs = Vec::new();
    for entry in fs::read_dir(root_dir)? {
        let entry = entry?;
        let path = entry.path();
        if !path.is_dir() {
            continue;
        }
        let manifest_path = path.join("manifest.json");
        if !manifest_path.exists() {
            continue;
        }
        let manifest =
            serde_json::from_str::<serde_json::Value>(&fs::read_to_string(&manifest_path)?)
                .unwrap_or(serde_json::Value::Null);
        let artifact_files = path.join("artifacts");
        let artifact_file_count = if artifact_files.exists() {
            let mut count = 0usize;
            for group in fs::read_dir(&artifact_files)? {
                let group = group?;
                let group_path = group.path();
                if group_path.is_dir() {
                    for child in fs::read_dir(group_path)? {
                        let child = child?;
                        if child.path().is_file() {
                            count = count.saturating_add(1);
                        }
                    }
                }
            }
            count
        } else {
            0
        };
        runs.push(serde_json::json!({
            "run_dir": path.display().to_string(),
            "command": manifest.get("command").cloned().unwrap_or(serde_json::Value::Null),
            "dataset_id": manifest.get("dataset_id").cloned().unwrap_or(serde_json::Value::Null),
            "config_hash": manifest.get("config_hash").cloned().unwrap_or(serde_json::Value::Null),
            "artifact_file_count": artifact_file_count
        }));
    }
    runs.sort_by(|a, b| {
        b.get("run_dir")
            .and_then(|v| v.as_str())
            .unwrap_or("")
            .cmp(a.get("run_dir").and_then(|v| v.as_str()).unwrap_or(""))
    });
    runs.truncate(limit.max(1));
    Ok(serde_json::json!({
        "schema_version": 1,
        "root_dir": root_dir.display().to_string(),
        "limit": limit,
        "runs": runs
    }))
}

fn route_explain_report(topic: RouteTopic) -> serde_json::Value {
    let topic_name = format!("{topic:?}").to_lowercase();
    let steps = match topic {
        RouteTopic::Integrity => vec![
            serde_json::json!({"command": "bijux gnss diagnostics medium-gate --run-dir <run_dir> --report table", "reason": "surface critical pass/fail first"}),
            serde_json::json!({"command": "bijux gnss diagnostics advanced-gate --run-dir <run_dir> --mode rtk --report json", "reason": "separate support maturity from evidence failures"}),
            serde_json::json!({"command": "bijux gnss diagnostics explain --run-dir <run_dir> --report json", "reason": "inspect replay scope, cache, and artifact integrity"}),
        ],
        RouteTopic::Replay => vec![
            serde_json::json!({"command": "bijux gnss diagnostics verify-repro --run-dir <run_dir> --report json", "reason": "establish bundle and fingerprint baseline"}),
            serde_json::json!({"command": "bijux gnss diagnostics replay-audit --baseline-run-dir <a> --candidate-run-dir <b> --report table", "reason": "classify deterministic match vs drift"}),
            serde_json::json!({"command": "bijux gnss diagnostics compare --baseline-run-dir <a> --candidate-run-dir <b> --report json", "reason": "attach quality deltas to replay outcome"}),
        ],
        RouteTopic::Compare => vec![
            serde_json::json!({"command": "bijux gnss diagnostics compare --baseline-run-dir <a> --candidate-run-dir <b> --report json", "reason": "compute reproducibility and quality delta summary"}),
            serde_json::json!({"command": "bijux gnss diagnostics channel-summary --run-dir <b> --report json", "reason": "inspect channel-level context for differences"}),
            serde_json::json!({"command": "bijux gnss diagnostics benchmark-summary --run-dir <b> --report json", "reason": "review digestible benchmark metrics with rigor markers"}),
        ],
        RouteTopic::Export => vec![
            serde_json::json!({"command": "bijux gnss diagnostics export-bundle --run-dir <run_dir>", "reason": "create reproducible review package"}),
            serde_json::json!({"command": "bijux gnss diagnostics machine-catalog --report json", "reason": "declare report contracts for downstream systems"}),
            serde_json::json!({"command": "bijux gnss diagnostics history-browse --root-dir runs --report json", "reason": "find neighboring runs for triage context"}),
        ],
    };
    serde_json::json!({
        "schema_version": 1,
        "topic": topic_name,
        "steps": steps
    })
}

fn operator_workflow_report(profile: WorkflowProfile) -> serde_json::Value {
    let profile_name = format!("{profile:?}").to_lowercase();
    let steps = match profile {
        WorkflowProfile::Run => vec![
            serde_json::json!({"label": "execute", "command": "bijux gnss run --dataset <id> --config <profile.toml>", "output": "manifest + run_report + artifacts"}),
            serde_json::json!({"label": "status", "command": "bijux gnss diagnostics operator-status --run-dir <run_dir> --report table", "output": "run/quality/evidence states"}),
            serde_json::json!({"label": "gate", "command": "bijux gnss diagnostics medium-gate --run-dir <run_dir> --strict --report json", "output": "integration gate result"}),
        ],
        WorkflowProfile::Triage => vec![
            serde_json::json!({"label": "inventory", "command": "bijux gnss diagnostics artifact-inventory --run-dir <run_dir> --report table", "output": "artifact group counts"}),
            serde_json::json!({"label": "route", "command": "bijux gnss diagnostics route-explain --topic integrity --report table", "output": "debugging command path"}),
            serde_json::json!({"label": "bundle", "command": "bijux gnss diagnostics export-bundle --run-dir <run_dir>", "output": "reproducible review package"}),
        ],
        WorkflowProfile::Compare => vec![
            serde_json::json!({"label": "replay", "command": "bijux gnss diagnostics replay-audit --baseline-run-dir <a> --candidate-run-dir <b> --report table", "output": "determinism classification"}),
            serde_json::json!({"label": "quality", "command": "bijux gnss diagnostics compare --baseline-run-dir <a> --candidate-run-dir <b> --report json", "output": "quality and integrity deltas"}),
            serde_json::json!({"label": "channel", "command": "bijux gnss diagnostics channel-summary --run-dir <b> --report table", "output": "channel-level context for drift"}),
        ],
    };
    serde_json::json!({
        "schema_version": 1,
        "profile": profile_name,
        "steps": steps
    })
}

fn operator_ergonomics_report(run_dir: &Path) -> Result<serde_json::Value> {
    ensure_run_dir_exists(run_dir)?;
    let status = operator_status_report(run_dir)?;
    let gate = medium_gate_report(run_dir)?;
    let gate_passed = gate.get("gate_passed").and_then(|v| v.as_bool()).unwrap_or(false);
    let evidence_state = status
        .get("status")
        .and_then(|v| v.get("evidence_state"))
        .and_then(|v| v.as_str())
        .unwrap_or("unknown");
    let quality_state = status
        .get("status")
        .and_then(|v| v.get("quality_state"))
        .and_then(|v| v.as_str())
        .unwrap_or("unknown");
    let run_state = status
        .get("status")
        .and_then(|v| v.get("run_state"))
        .and_then(|v| v.as_str())
        .unwrap_or("unknown");
    let score = match (gate_passed, quality_state, evidence_state, run_state) {
        (true, "artifact_clean", "evidence_supported", "audit_ready") => 1.0,
        (true, "artifact_clean", _, _) => 0.8,
        (true, _, _, _) => 0.6,
        (false, _, _, _) => 0.3,
    };
    let quick_actions = if gate_passed {
        vec![
            "route-explain --topic compare".to_string(),
            "operator-workflow --profile compare".to_string(),
        ]
    } else {
        vec![
            "medium-gate --strict".to_string(),
            "route-explain --topic integrity".to_string(),
            "export-bundle".to_string(),
        ]
    };

    Ok(serde_json::json!({
        "schema_version": 1,
        "run_dir": run_dir.display().to_string(),
        "ergonomics_score": score,
        "quick_actions": quick_actions,
        "rigor": {
            "medium_gate_passed": gate_passed,
            "evidence_state": evidence_state,
            "quality_state": quality_state,
            "run_state": run_state
        }
    }))
}

fn audit_trail_report(run_dir: &Path) -> Result<serde_json::Value> {
    ensure_run_dir_exists(run_dir)?;
    let manifest_path = run_dir.join("manifest.json");
    let manifest: serde_json::Value = serde_json::from_str(&fs::read_to_string(&manifest_path)?)
        .unwrap_or(serde_json::Value::Null);
    let replay_controls = serde_json::json!({
        "deterministic": manifest
            .get("replay_scope")
            .and_then(|v| v.get("deterministic"))
            .cloned()
            .unwrap_or(serde_json::Value::Null),
        "resume": manifest
            .get("replay_scope")
            .and_then(|v| v.get("resume"))
            .cloned()
            .unwrap_or(serde_json::Value::Null),
        "explicit_output_dir": manifest
            .get("replay_scope")
            .and_then(|v| v.get("explicit_output_dir"))
            .cloned()
            .unwrap_or(serde_json::Value::Null),
    });

    let mut override_events = Vec::new();
    let trace_path = run_dir.join("trace.ndjson");
    if trace_path.exists() {
        let trace = fs::read_to_string(&trace_path)?;
        for line in trace.lines() {
            if line.trim().is_empty() {
                continue;
            }
            let Ok(row) = serde_json::from_str::<serde_json::Value>(line) else {
                continue;
            };
            let name = row.get("name").and_then(|v| v.as_str()).unwrap_or_default().to_lowercase();
            if name.contains("override") || name.contains("exception") || name.contains("replay") {
                override_events.push(row);
            }
        }
    }

    let mut policy_exceptions = Vec::new();
    let evidence_path =
        run_dir.join("artifacts").join("validate").join("validation_evidence_bundle.json");
    if evidence_path.exists() {
        let evidence =
            serde_json::from_str::<serde_json::Value>(&fs::read_to_string(&evidence_path)?)
                .unwrap_or(serde_json::Value::Null);
        if let Some(violations) = evidence
            .get("claim_evidence_guard")
            .and_then(|v| v.get("violations"))
            .and_then(|v| v.as_array())
        {
            policy_exceptions.extend(violations.iter().cloned());
        }
        if let Some(refusals) = evidence.get("refusal_reasons").and_then(|v| v.as_array()) {
            policy_exceptions.extend(refusals.iter().cloned());
        }
    }

    Ok(serde_json::json!({
        "schema_version": 1,
        "run_dir": run_dir.display().to_string(),
        "manifest_path": manifest_path.display().to_string(),
        "replay_controls": replay_controls,
        "override_events": override_events,
        "policy_exceptions": policy_exceptions
    }))
}

fn dependency_trace_report(run_dir: &Path) -> Result<serde_json::Value> {
    ensure_run_dir_exists(run_dir)?;
    let manifest_path = run_dir.join("manifest.json");
    let manifest: serde_json::Value = serde_json::from_str(&fs::read_to_string(&manifest_path)?)
        .unwrap_or(serde_json::Value::Null);

    let correction_path = run_dir.join("artifacts").join("rtk").join("rtk_correction_input.jsonl");
    let mut correction_rows = 0u64;
    let mut correction_tags = std::collections::BTreeSet::new();
    if correction_path.exists() {
        let text = fs::read_to_string(&correction_path)?;
        for line in text.lines() {
            if line.trim().is_empty() {
                continue;
            }
            let Ok(row) = serde_json::from_str::<serde_json::Value>(line) else {
                continue;
            };
            correction_rows = correction_rows.saturating_add(1);
            if let Some(tags) =
                row.get("payload").and_then(|v| v.get("correction_tags")).and_then(|v| v.as_array())
            {
                for tag in tags {
                    if let Some(value) = tag.as_str() {
                        correction_tags.insert(value.to_string());
                    }
                }
            }
        }
    }

    let mut env_references = Vec::new();
    if let Some(command) = manifest.get("command").and_then(|v| v.as_str()) {
        if command.contains("--config") {
            env_references.push("config_file".to_string());
        }
        if command.contains("--dataset") {
            env_references.push("dataset_reference".to_string());
        }
    }
    if manifest.get("toolchain").is_some() {
        env_references.push("toolchain_metadata".to_string());
    }
    if manifest.get("features").is_some() {
        env_references.push("feature_set".to_string());
    }
    env_references.sort();
    env_references.dedup();
    let env_reference_count = env_references.len();

    Ok(serde_json::json!({
        "schema_version": 1,
        "run_dir": run_dir.display().to_string(),
        "tooling": {
            "toolchain": manifest
                .get("toolchain")
                .and_then(|v| v.as_str())
                .unwrap_or("unknown"),
            "features": manifest
                .get("features")
                .cloned()
                .unwrap_or(serde_json::json!([]))
        },
        "corrections": {
            "artifact_path": correction_path.display().to_string(),
            "rows": correction_rows,
            "tags": correction_tags.into_iter().collect::<Vec<_>>()
        },
        "environment": {
            "dataset_id": manifest
                .get("dataset_id")
                .cloned()
                .unwrap_or(serde_json::Value::Null),
            "config_hash": manifest
                .get("config_hash")
                .cloned()
                .unwrap_or(serde_json::Value::Null),
            "references": env_references,
            "reference_count": env_reference_count
        }
    }))
}

fn trust_class_report(run_dir: &Path) -> Result<serde_json::Value> {
    ensure_run_dir_exists(run_dir)?;
    let medium_gate = medium_gate_report(run_dir)?;
    let repro = verify_repro_bundle(run_dir)?;
    let benchmark = benchmark_summary_report(run_dir)?;

    let gate_passed = medium_gate.get("gate_passed").and_then(|v| v.as_bool()).unwrap_or(false);
    let repro_ok = repro.get("audit_ok").and_then(|v| v.as_bool()).unwrap_or(false);
    let nav_epochs = benchmark
        .get("summary")
        .and_then(|v| v.get("nav_epochs"))
        .and_then(|v| v.as_u64())
        .unwrap_or(0);
    let obs_epochs = benchmark
        .get("summary")
        .and_then(|v| v.get("obs_epochs"))
        .and_then(|v| v.as_u64())
        .unwrap_or(0);

    let mut rationale = Vec::new();
    if gate_passed {
        rationale.push("medium_gate_passed".to_string());
    } else {
        rationale.push("medium_gate_failed".to_string());
    }
    if repro_ok {
        rationale.push("repro_audit_ok".to_string());
    } else {
        rationale.push("repro_audit_failed".to_string());
    }
    if nav_epochs > 0 && obs_epochs > 0 {
        rationale.push("benchmark_epochs_present".to_string());
    } else {
        rationale.push("benchmark_epochs_missing".to_string());
    }

    let trust_class = if gate_passed && repro_ok && nav_epochs >= 100 && obs_epochs >= 100 {
        "certification_grade"
    } else if gate_passed && repro_ok {
        "benchmarked"
    } else if repro_ok {
        "operational"
    } else {
        "draft"
    };

    Ok(serde_json::json!({
        "schema_version": 1,
        "run_dir": run_dir.display().to_string(),
        "trust_class": trust_class,
        "rationale": rationale,
        "checks": {
            "medium_gate_passed": gate_passed,
            "repro_audit_ok": repro_ok,
            "obs_epochs": obs_epochs,
            "nav_epochs": nav_epochs
        }
    }))
}

fn integrity_focus_report(run_dir: &Path) -> Result<serde_json::Value> {
    ensure_run_dir_exists(run_dir)?;
    let trust = trust_class_report(run_dir)?;
    let audit = audit_trail_report(run_dir)?;
    let dependency = dependency_trace_report(run_dir)?;
    let medium_gate = medium_gate_report(run_dir)?;

    let mut blocking_findings = Vec::new();
    if !medium_gate.get("gate_passed").and_then(|v| v.as_bool()).unwrap_or(false) {
        blocking_findings.push("medium_gate_failed".to_string());
    }
    if audit.get("policy_exceptions").and_then(|v| v.as_array()).map_or(0usize, Vec::len) > 0 {
        blocking_findings.push("policy_exceptions_present".to_string());
    }
    if dependency
        .get("corrections")
        .and_then(|v| v.get("rows"))
        .and_then(|v| v.as_u64())
        .unwrap_or(0)
        == 0
    {
        blocking_findings.push("correction_trace_missing".to_string());
    }

    let trust_class = trust.get("trust_class").and_then(|v| v.as_str()).unwrap_or("draft");
    let base_score = match trust_class {
        "certification_grade" => 1.0,
        "benchmarked" => 0.8,
        "operational" => 0.6,
        _ => 0.3,
    };
    let penalty = 0.15 * blocking_findings.len() as f64;
    let engineering_trust_score = (base_score - penalty).clamp(0.0, 1.0);

    let prioritized_actions = if blocking_findings.is_empty() {
        vec![
            "replay-audit baseline/candidate for drift watch".to_string(),
            "export-bundle for durable review package".to_string(),
        ]
    } else {
        vec![
            "medium-gate --strict to force critical gate closure".to_string(),
            "audit-trail and dependency-trace to close integrity gaps".to_string(),
            "trust-class rerun after fixes".to_string(),
        ]
    };

    Ok(serde_json::json!({
        "schema_version": 1,
        "run_dir": run_dir.display().to_string(),
        "trust_class": trust_class,
        "engineering_trust_score": engineering_trust_score,
        "blocking_findings": blocking_findings,
        "prioritized_actions": prioritized_actions,
        "signals": {
            "trust_class_report": trust,
            "audit_trail_report": audit,
            "dependency_trace_report": dependency,
            "medium_gate_report": medium_gate
        }
    }))
}

