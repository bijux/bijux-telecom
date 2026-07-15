use super::*;

pub(crate) fn summarize_run_diagnostics(run_dir: &Path) -> Result<Vec<DiagnosticEvent>> {
    ensure_run_dir_exists(run_dir)?;
    let artifacts_dir = run_dir.join("artifacts");
    if !artifacts_dir.exists() {
        return Err(classified_error(
            CliErrorClass::OperatorMisconfiguration,
            format!("artifacts directory not found: {}", artifacts_dir.display()),
        ));
    }
    let mut events = Vec::new();
    for entry in fs::read_dir(&artifacts_dir)? {
        let entry = entry?;
        let path = entry.path();
        if !path.is_file() {
            continue;
        }
        let Some(ext) = path.extension().and_then(|s| s.to_str()) else {
            continue;
        };
        if ext != "json" && ext != "jsonl" {
            continue;
        }
        if let Ok(result) = artifact_validate(&path, None, false) {
            events.extend(result.diagnostics);
        }
    }
    Ok(events)
}

pub(crate) fn explain_run_scope(run_dir: &Path) -> Result<serde_json::Value> {
    ensure_run_dir_exists(run_dir)?;
    let manifest_path = run_dir.join("manifest.json");
    let run_manifest: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(&manifest_path)?).map_err(|err| {
            classified_error(
                CliErrorClass::InternalFault,
                format!("failed parsing manifest {}: {err}", manifest_path.display()),
            )
        })?;

    let artifacts_dir = run_dir.join("artifacts");
    let mut identity_coverage = serde_json::Map::new();
    let mut total_artifact_files = 0usize;
    let mut total_lines = 0usize;
    let mut corrupted_files = Vec::new();
    let mut cache_events = serde_json::Map::new();
    let mut cache_event_total = 0usize;

    if artifacts_dir.exists() {
        for entry in std::fs::read_dir(&artifacts_dir)? {
            let entry = entry?;
            let path = entry.path();
            if !path.is_file() {
                continue;
            }
            let Some(ext) = path.extension().and_then(|s| s.to_str()) else {
                continue;
            };
            if ext != "json" && ext != "jsonl" {
                continue;
            }
            total_artifact_files = total_artifact_files.saturating_add(1);
            let data = std::fs::read_to_string(&path)?;
            let file_name =
                path.file_name().and_then(|name| name.to_str()).unwrap_or("unknown").to_string();
            let mut has_artifact_id = false;
            let mut has_epoch_id = false;
            let mut has_source_epoch_id = false;
            let mut parsed_lines = 0usize;

            if ext == "jsonl" {
                for (idx, line) in data.lines().enumerate() {
                    if line.trim().is_empty() {
                        continue;
                    }
                    parsed_lines = parsed_lines.saturating_add(1);
                    total_lines = total_lines.saturating_add(1);
                    let parsed: serde_json::Value = match serde_json::from_str(line) {
                        Ok(v) => v,
                        Err(err) => {
                            corrupted_files.push(serde_json::json!({
                                "file": file_name,
                                "line": idx + 1,
                                "error": err.to_string()
                            }));
                            break;
                        }
                    };
                    let payload = parsed.get("payload").unwrap_or(&parsed);
                    has_artifact_id |= payload.get("artifact_id").is_some();
                    has_epoch_id |= payload.get("epoch_id").is_some();
                    has_source_epoch_id |= payload.get("source_observation_epoch_id").is_some();
                }
            } else if ext == "json" {
                total_lines = total_lines.saturating_add(1);
                if let Err(err) = serde_json::from_str::<serde_json::Value>(&data) {
                    corrupted_files.push(serde_json::json!({
                        "file": file_name,
                        "line": 1,
                        "error": err.to_string()
                    }));
                } else {
                    parsed_lines = 1;
                }
            }

            identity_coverage.insert(
                file_name.clone(),
                serde_json::json!({
                    "parsed_entries": parsed_lines,
                    "has_artifact_id": has_artifact_id,
                    "has_epoch_id": has_epoch_id,
                    "has_source_observation_epoch_id": has_source_epoch_id
                }),
            );
        }
    }

    let trace_path = run_dir.join("trace.ndjson");
    if trace_path.exists() {
        let trace = std::fs::read_to_string(trace_path)?;
        for line in trace.lines() {
            if line.trim().is_empty() {
                continue;
            }
            let Ok(row) = serde_json::from_str::<serde_json::Value>(line) else {
                continue;
            };
            let Some(name) = row.get("name").and_then(|v| v.as_str()) else {
                continue;
            };
            if name != "acquisition_code_fft_cache_hit" && name != "acquisition_code_fft_cache_miss"
            {
                continue;
            }
            cache_event_total = cache_event_total.saturating_add(1);
            if name == "acquisition_code_fft_cache_miss" {
                if let Some(reason) =
                    row.get("fields").and_then(|v| v.as_array()).and_then(|fields| {
                        fields.iter().find_map(|entry| {
                            let key = entry.get(0).and_then(|v| v.as_str())?;
                            if key == "reason" {
                                entry.get(1).and_then(|v| v.as_str()).map(|s| s.to_string())
                            } else {
                                None
                            }
                        })
                    })
                {
                    let next = cache_events.get(&reason).and_then(|v| v.as_u64()).unwrap_or(0) + 1;
                    cache_events.insert(reason, serde_json::Value::from(next));
                }
            }
        }
    }

    let metrics_summary_path = run_dir.join("metrics_summary.json");
    let metrics_summary = if metrics_summary_path.exists() {
        serde_json::from_str::<serde_json::Value>(&std::fs::read_to_string(&metrics_summary_path)?)
            .unwrap_or(serde_json::Value::Null)
    } else {
        serde_json::Value::Null
    };

    Ok(serde_json::json!({
        "schema_version": 1,
        "run_dir": run_dir.display().to_string(),
        "manifest_path": manifest_path.display().to_string(),
        "replay_scope": {
            "command": run_manifest.get("command").cloned().unwrap_or(serde_json::Value::Null),
            "dataset_id": run_manifest.get("dataset_id").cloned().unwrap_or(serde_json::Value::Null),
            "config_hash": run_manifest.get("config_hash").cloned().unwrap_or(serde_json::Value::Null),
            "toolchain": run_manifest.get("toolchain").cloned().unwrap_or(serde_json::Value::Null),
            "features": run_manifest.get("features").cloned().unwrap_or(serde_json::Value::Null),
            "deterministic": run_manifest.get("replay_scope").and_then(|v| v.get("deterministic")).cloned().unwrap_or(serde_json::Value::Null),
            "resume": run_manifest.get("replay_scope").and_then(|v| v.get("resume")).cloned().unwrap_or(serde_json::Value::Null),
            "explicit_output_dir": run_manifest.get("replay_scope").and_then(|v| v.get("explicit_output_dir")).cloned().unwrap_or(serde_json::Value::Null)
        },
        "front_end_provenance": run_manifest
            .get("front_end_provenance")
            .cloned()
            .unwrap_or(serde_json::Value::Null),
        "artifact_integrity": {
            "artifact_files_scanned": total_artifact_files,
            "entries_scanned": total_lines,
            "corrupted_files": corrupted_files
        },
        "artifact_identity_coverage": identity_coverage,
        "cache_behavior": {
            "cache_events_seen": cache_event_total,
            "miss_reason_counts": cache_events,
            "metrics_summary": metrics_summary
        }
    }))
}

pub(crate) fn verify_repro_bundle(run_dir: &Path) -> Result<serde_json::Value> {
    ensure_run_dir_exists(run_dir)?;
    let manifest_path = run_dir.join("manifest.json");
    let manifest_data = std::fs::read_to_string(&manifest_path)?;
    let manifest: serde_json::Value = serde_json::from_str(&manifest_data).map_err(|err| {
        classified_error(
            CliErrorClass::InternalFault,
            format!("failed parsing manifest {}: {err}", manifest_path.display()),
        )
    })?;

    let run_report_path = run_dir.join("run_report.json");
    let run_report_data = if run_report_path.exists() {
        Some(std::fs::read_to_string(&run_report_path)?)
    } else {
        None
    };

    let artifacts_dir = run_dir.join("artifacts");
    let mut artifact_hashes = serde_json::Map::new();
    let mut issues = Vec::new();

    if !artifacts_dir.exists() {
        issues.push("artifacts_directory_missing".to_string());
    } else {
        for entry in std::fs::read_dir(&artifacts_dir)? {
            let entry = entry?;
            let path = entry.path();
            if !path.is_file() {
                continue;
            }
            let file_name = path.file_name().and_then(|n| n.to_str()).unwrap_or("unknown");
            let bytes = std::fs::read(&path)?;
            let hash = sha256_hex(&bytes);
            artifact_hashes.insert(file_name.to_string(), serde_json::Value::String(hash));
        }
    }

    if artifact_hashes.is_empty() {
        issues.push("artifact_hash_set_empty".to_string());
    }

    let mut fingerprint_payload = serde_json::Map::new();
    fingerprint_payload.insert(
        "command".to_string(),
        manifest.get("command").cloned().unwrap_or(serde_json::Value::Null),
    );
    fingerprint_payload.insert(
        "config_hash".to_string(),
        manifest.get("config_hash").cloned().unwrap_or(serde_json::Value::Null),
    );
    fingerprint_payload.insert(
        "dataset_id".to_string(),
        manifest.get("dataset_id").cloned().unwrap_or(serde_json::Value::Null),
    );
    fingerprint_payload.insert(
        "replay_scope".to_string(),
        manifest.get("replay_scope").cloned().unwrap_or(serde_json::Value::Null),
    );
    fingerprint_payload.insert(
        "front_end_provenance".to_string(),
        manifest.get("front_end_provenance").cloned().unwrap_or(serde_json::Value::Null),
    );
    fingerprint_payload.insert(
        "layout_schema_version".to_string(),
        manifest.get("layout_schema_version").cloned().unwrap_or(serde_json::Value::Null),
    );
    let fingerprint_json = serde_json::to_vec(&fingerprint_payload)?;
    let replay_fingerprint = sha256_hex(&fingerprint_json);

    let manifest_hash = sha256_hex(manifest_data.as_bytes());
    let run_report_hash = run_report_data.as_deref().map(|text| sha256_hex(text.as_bytes()));
    let audit_ok = issues.is_empty();

    Ok(serde_json::json!({
        "schema_version": 1,
        "run_dir": run_dir.display().to_string(),
        "audit_ok": audit_ok,
        "issues": issues,
        "replay_fingerprint": replay_fingerprint,
        "manifest_sha256": manifest_hash,
        "run_report_sha256": run_report_hash,
        "artifact_sha256": artifact_hashes
    }))
}

pub(crate) fn compare_run_evidence(
    baseline_run_dir: &Path,
    candidate_run_dir: &Path,
) -> Result<serde_json::Value> {
    ensure_run_dir_exists(baseline_run_dir)?;
    ensure_run_dir_exists(candidate_run_dir)?;
    let baseline_repro = verify_repro_bundle(baseline_run_dir)?;
    let candidate_repro = verify_repro_bundle(candidate_run_dir)?;
    let baseline_explain = explain_run_scope(baseline_run_dir)?;
    let candidate_explain = explain_run_scope(candidate_run_dir)?;
    let quality = diff_runs(baseline_run_dir, candidate_run_dir)?;

    let baseline_corrupted = baseline_explain
        .get("artifact_integrity")
        .and_then(|v| v.get("corrupted_files"))
        .and_then(|v| v.as_array())
        .map_or(0usize, Vec::len);
    let candidate_corrupted = candidate_explain
        .get("artifact_integrity")
        .and_then(|v| v.get("corrupted_files"))
        .and_then(|v| v.as_array())
        .map_or(0usize, Vec::len);

    Ok(serde_json::json!({
        "schema_version": 1,
        "baseline_run_dir": baseline_run_dir.display().to_string(),
        "candidate_run_dir": candidate_run_dir.display().to_string(),
        "reproducibility": {
            "fingerprint_match": baseline_repro.get("replay_fingerprint") == candidate_repro.get("replay_fingerprint"),
            "baseline_audit_ok": baseline_repro.get("audit_ok").and_then(|v| v.as_bool()).unwrap_or(false),
            "candidate_audit_ok": candidate_repro.get("audit_ok").and_then(|v| v.as_bool()).unwrap_or(false),
            "baseline_replay_fingerprint": baseline_repro.get("replay_fingerprint").cloned().unwrap_or(serde_json::Value::Null),
            "candidate_replay_fingerprint": candidate_repro.get("replay_fingerprint").cloned().unwrap_or(serde_json::Value::Null),
            "baseline_issues": baseline_repro.get("issues").cloned().unwrap_or_else(|| serde_json::json!([])),
            "candidate_issues": candidate_repro.get("issues").cloned().unwrap_or_else(|| serde_json::json!([]))
        },
        "artifact_integrity": {
            "baseline_corrupted_files": baseline_corrupted,
            "candidate_corrupted_files": candidate_corrupted,
            "corrupted_file_delta": candidate_corrupted as isize - baseline_corrupted as isize
        },
        "quality": {
            "mean_rms_delta_m": quality.get("mean_rms_m").and_then(|v| v.get("delta")).and_then(|v| v.as_f64()).unwrap_or(0.0),
            "mean_cn0_delta_dbhz": quality.get("mean_cn0_dbhz").and_then(|v| v.get("delta")).and_then(|v| v.as_f64()).unwrap_or(0.0),
            "convergence_epoch_baseline": quality.get("convergence_epoch_idx").and_then(|v| v.get("run_a")).cloned().unwrap_or(serde_json::Value::Null),
            "convergence_epoch_candidate": quality.get("convergence_epoch_idx").and_then(|v| v.get("run_b")).cloned().unwrap_or(serde_json::Value::Null),
            "raw_diff": quality
        },
        "interpretation": {
            "claim_level": if baseline_repro.get("audit_ok").and_then(|v| v.as_bool()).unwrap_or(false)
                && candidate_repro.get("audit_ok").and_then(|v| v.as_bool()).unwrap_or(false)
            {
                "evidence_bound"
            } else {
                "audit_limited"
            },
            "note": "quality deltas are comparative diagnostics and must be interpreted with validation evidence and support maturity"
        },
        "layered": layered_report(
            vec![],
            serde_json::json!({
                "reproducibility": {
                    "fingerprint_match": baseline_repro.get("replay_fingerprint") == candidate_repro.get("replay_fingerprint"),
                    "baseline_audit_ok": baseline_repro.get("audit_ok").and_then(|v| v.as_bool()).unwrap_or(false),
                    "candidate_audit_ok": candidate_repro.get("audit_ok").and_then(|v| v.as_bool()).unwrap_or(false)
                },
                "quality_delta": {
                    "mean_rms_delta_m": quality.get("mean_rms_m").and_then(|v| v.get("delta")).and_then(|v| v.as_f64()).unwrap_or(0.0),
                    "mean_cn0_delta_dbhz": quality.get("mean_cn0_dbhz").and_then(|v| v.get("delta")).and_then(|v| v.as_f64()).unwrap_or(0.0)
                }
            })
        )
    }))
}

pub(crate) fn replay_audit_report(
    baseline_run_dir: &Path,
    candidate_run_dir: &Path,
) -> Result<serde_json::Value> {
    ensure_run_dir_exists(baseline_run_dir)?;
    ensure_run_dir_exists(candidate_run_dir)?;
    let compare = compare_run_evidence(baseline_run_dir, candidate_run_dir)?;
    let baseline_explain = explain_run_scope(baseline_run_dir)?;
    let candidate_explain = explain_run_scope(candidate_run_dir)?;

    let fingerprint_match = compare
        .get("reproducibility")
        .and_then(|v| v.get("fingerprint_match"))
        .and_then(|v| v.as_bool())
        .unwrap_or(false);
    let baseline_audit_ok = compare
        .get("reproducibility")
        .and_then(|v| v.get("baseline_audit_ok"))
        .and_then(|v| v.as_bool())
        .unwrap_or(false);
    let candidate_audit_ok = compare
        .get("reproducibility")
        .and_then(|v| v.get("candidate_audit_ok"))
        .and_then(|v| v.as_bool())
        .unwrap_or(false);
    let corrupted_delta = compare
        .get("artifact_integrity")
        .and_then(|v| v.get("corrupted_file_delta"))
        .and_then(|v| v.as_i64())
        .unwrap_or(0);

    let mut reasons = Vec::new();
    if !baseline_audit_ok {
        reasons.push("baseline_repro_audit_failed".to_string());
    }
    if !candidate_audit_ok {
        reasons.push("candidate_repro_audit_failed".to_string());
    }
    if !fingerprint_match {
        reasons.push("replay_fingerprint_changed".to_string());
    }
    if corrupted_delta != 0 {
        reasons.push("artifact_corruption_delta_detected".to_string());
    }
    let classification =
        if baseline_audit_ok && candidate_audit_ok && fingerprint_match && corrupted_delta == 0 {
            "deterministic_match"
        } else if baseline_audit_ok && candidate_audit_ok {
            "scientific_or_configuration_drift"
        } else {
            "integrity_failure"
        };

    Ok(serde_json::json!({
        "schema_version": 1,
        "baseline_run_dir": baseline_run_dir.display().to_string(),
        "candidate_run_dir": candidate_run_dir.display().to_string(),
        "classification": classification,
        "reasons": reasons,
        "baseline_command": baseline_explain
            .get("replay_scope")
            .and_then(|v| v.get("command"))
            .and_then(|v| v.as_str())
            .unwrap_or("unknown"),
        "candidate_command": candidate_explain
            .get("replay_scope")
            .and_then(|v| v.get("command"))
            .and_then(|v| v.as_str())
            .unwrap_or("unknown"),
        "compare": compare,
        "layered": layered_report(
            reasons.clone(),
            serde_json::json!({
                "classification": classification,
                "compare": compare
            })
        )
    }))
}
