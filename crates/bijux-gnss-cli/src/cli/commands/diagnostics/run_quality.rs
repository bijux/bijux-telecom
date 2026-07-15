fn advanced_gate_report(run_dir: &Path, mode: AdvancedGateMode) -> Result<serde_json::Value> {
    ensure_run_dir_exists(run_dir)?;
    let mode_label = match mode {
        AdvancedGateMode::Rtk => "Rtk",
        AdvancedGateMode::Ppp => "Ppp",
    };
    let support_path = run_dir.join("artifacts").join("rtk").join("rtk_support_matrix.json");
    let support_json = if support_path.exists() {
        serde_json::from_str::<serde_json::Value>(&fs::read_to_string(&support_path)?)
            .unwrap_or(serde_json::Value::Null)
    } else {
        serde_json::Value::Null
    };
    let support_row = support_json
        .get("rows")
        .and_then(|v| v.as_array())
        .and_then(|rows| {
            rows.iter().find(|row| {
                row.get("mode")
                    .and_then(|v| v.as_str())
                    .map(|value| value == mode_label)
                    .unwrap_or(false)
            })
        })
        .cloned()
        .unwrap_or(serde_json::Value::Null);

    let maturity = support_row.get("maturity").and_then(|v| v.as_str()).unwrap_or("unknown");
    let real_solver = support_row.get("real_solver").and_then(|v| v.as_bool()).unwrap_or(false);

    let evidence_path =
        run_dir.join("artifacts").join("validate").join("validation_evidence_bundle.json");
    let evidence_json = if evidence_path.exists() {
        serde_json::from_str::<serde_json::Value>(&fs::read_to_string(&evidence_path)?)
            .unwrap_or(serde_json::Value::Null)
    } else {
        serde_json::Value::Null
    };
    let claim_guard_supported = evidence_json
        .get("claim_evidence_guard")
        .and_then(|v| v.get("supported"))
        .and_then(|v| v.as_bool())
        .unwrap_or(false);

    let mut reasons = Vec::new();
    if support_row.is_null() {
        reasons.push("support_matrix_row_missing".to_string());
    }
    if maturity == "NotReady" {
        reasons.push("mode_not_ready".to_string());
    }
    if !real_solver {
        reasons.push("mode_not_real_solver".to_string());
    }
    if !claim_guard_supported {
        reasons.push("claim_evidence_guard_not_supported".to_string());
    }
    let gate_passed = reasons.is_empty();
    let claim_level = if maturity == "NotReady" || !real_solver {
        "not_ready"
    } else if gate_passed {
        "evidence_bound_experimental"
    } else {
        "blocked_by_evidence"
    };

    Ok(serde_json::json!({
        "schema_version": 1,
        "run_dir": run_dir.display().to_string(),
        "mode": mode_label,
        "gate_passed": gate_passed,
        "claim_level": claim_level,
        "reasons": reasons,
        "support": {
            "matrix_path": support_path.display().to_string(),
            "maturity": maturity,
            "real_solver": real_solver,
            "row": support_row
        },
        "evidence": {
            "bundle_path": evidence_path.display().to_string(),
            "claim_guard_supported": claim_guard_supported,
            "claim_guard": evidence_json
                .get("claim_evidence_guard")
                .cloned()
                .unwrap_or(serde_json::Value::Null)
        }
    }))
}

fn artifact_inventory_report(run_dir: &Path) -> Result<serde_json::Value> {
    ensure_run_dir_exists(run_dir)?;
    let artifacts_root = run_dir.join("artifacts");
    let mut groups = serde_json::Map::new();
    if artifacts_root.exists() {
        for entry in fs::read_dir(&artifacts_root)? {
            let entry = entry?;
            let path = entry.path();
            if !path.is_dir() {
                continue;
            }
            let name = entry.file_name().to_string_lossy().to_string();
            let mut files = Vec::new();
            for child in fs::read_dir(&path)? {
                let child = child?;
                let child_path = child.path();
                if child_path.is_file() {
                    files.push(child.file_name().to_string_lossy().to_string());
                }
            }
            files.sort();
            groups.insert(
                name,
                serde_json::json!({
                    "file_count": files.len(),
                    "files": files
                }),
            );
        }
    }
    Ok(serde_json::json!({
        "schema_version": 1,
        "run_dir": run_dir.display().to_string(),
        "groups": groups
    }))
}

fn debug_plan_report(run_dir: &Path) -> Result<serde_json::Value> {
    ensure_run_dir_exists(run_dir)?;
    Ok(serde_json::json!({
        "schema_version": 1,
        "run_dir": run_dir.display().to_string(),
        "stages": [
            {
                "stage": "acquisition",
                "artifact": "artifacts/acquire/acq.jsonl",
                "recommended_check": "bijux gnss artifact validate --file <acq.jsonl>",
                "diagnostic_focus": "peak ratios, support status, threshold provenance"
            },
            {
                "stage": "tracking",
                "artifact": "artifacts/track/track.jsonl",
                "recommended_check": "bijux gnss diagnostics channel-summary --run-dir <run_dir>",
                "diagnostic_focus": "lock transitions, cycle slips, cn0 distribution"
            },
            {
                "stage": "observations",
                "artifact": "artifacts/obs/obs.jsonl",
                "recommended_check": "bijux gnss artifact validate --file <obs.jsonl>",
                "diagnostic_focus": "missing/weak observables, epoch consistency"
            },
            {
                "stage": "pvt",
                "artifact": "artifacts/pvt/pvt.jsonl",
                "recommended_check": "bijux gnss diagnostics medium-gate --run-dir <run_dir>",
                "diagnostic_focus": "residuals, geometry, claim evidence guard"
            }
        ]
    }))
}

fn benchmark_summary_report(run_dir: &Path) -> Result<serde_json::Value> {
    ensure_run_dir_exists(run_dir)?;
    let obs_path = run_dir.join("artifacts").join("obs").join("obs.jsonl");
    let pvt_path = run_dir.join("artifacts").join("pvt").join("pvt.jsonl");
    let obs =
        if obs_path.exists() { read_obs_epochs(&obs_path).unwrap_or_default() } else { Vec::new() };
    let nav = if pvt_path.exists() {
        read_nav_solutions(&pvt_path).unwrap_or_default()
    } else {
        Vec::new()
    };
    let mean_cn0 = {
        let mut sum = 0.0;
        let mut count = 0usize;
        for epoch in &obs {
            for sat in &epoch.sats {
                sum += sat.cn0_dbhz;
                count = count.saturating_add(1);
            }
        }
        if count == 0 {
            None
        } else {
            Some(sum / count as f64)
        }
    };
    let mean_rms = if nav.is_empty() {
        None
    } else {
        Some(nav.iter().map(|v| v.rms_m.0).sum::<f64>() / nav.len() as f64)
    };
    Ok(serde_json::json!({
        "schema_version": 1,
        "run_dir": run_dir.display().to_string(),
        "summary": {
            "obs_epochs": obs.len(),
            "nav_epochs": nav.len(),
            "mean_cn0_dbhz": mean_cn0,
            "mean_nav_rms_m": mean_rms
        },
        "rigor": {
            "requires_validation_evidence_bundle": true,
            "requires_replay_audit": true
        }
    }))
}

fn medium_gate_report(run_dir: &Path) -> Result<serde_json::Value> {
    ensure_run_dir_exists(run_dir)?;
    let explain = explain_run_scope(run_dir)?;
    let repro = verify_repro_bundle(run_dir)?;
    let evidence_path =
        run_dir.join("artifacts").join("validate").join("validation_evidence_bundle.json");
    let evidence = if evidence_path.exists() {
        serde_json::from_str::<serde_json::Value>(&fs::read_to_string(&evidence_path)?)
            .unwrap_or(serde_json::Value::Null)
    } else {
        serde_json::Value::Null
    };
    let claim_supported = evidence
        .get("claim_evidence_guard")
        .and_then(|v| v.get("supported"))
        .and_then(|v| v.as_bool())
        .unwrap_or(false);
    let corruption_count = explain
        .get("artifact_integrity")
        .and_then(|v| v.get("corrupted_files"))
        .and_then(|v| v.as_array())
        .map_or(0usize, Vec::len);
    let repro_ok = repro.get("audit_ok").and_then(|v| v.as_bool()).unwrap_or(false);
    let mut reasons = Vec::new();
    if !repro_ok {
        reasons.push("repro_audit_failed".to_string());
    }
    if corruption_count > 0 {
        reasons.push("artifact_corruption_detected".to_string());
    }
    if !claim_supported {
        reasons.push("claim_evidence_guard_failed".to_string());
    }
    Ok(serde_json::json!({
        "schema_version": 1,
        "run_dir": run_dir.display().to_string(),
        "gate_passed": reasons.is_empty(),
        "reasons": reasons,
        "checks": {
            "replay_audit_ok": repro_ok,
            "artifact_corruption_count": corruption_count,
            "claim_evidence_supported": claim_supported
        }
    }))
}

fn operator_status_report(run_dir: &Path) -> Result<serde_json::Value> {
    ensure_run_dir_exists(run_dir)?;
    let explain = explain_run_scope(run_dir)?;
    let repro = verify_repro_bundle(run_dir)?;
    let evidence_path =
        run_dir.join("artifacts").join("validate").join("validation_evidence_bundle.json");
    let evidence = if evidence_path.exists() {
        serde_json::from_str::<serde_json::Value>(&fs::read_to_string(&evidence_path)?)
            .unwrap_or(serde_json::Value::Null)
    } else {
        serde_json::Value::Null
    };
    let corrupted = explain
        .get("artifact_integrity")
        .and_then(|v| v.get("corrupted_files"))
        .and_then(|v| v.as_array())
        .map_or(0usize, Vec::len);
    let evidence_state = if evidence
        .get("claim_evidence_guard")
        .and_then(|v| v.get("supported"))
        .and_then(|v| v.as_bool())
        .unwrap_or(false)
    {
        "evidence_supported"
    } else {
        "evidence_limited"
    };
    let quality_state = if corrupted == 0 { "artifact_clean" } else { "artifact_corrupted" };
    let run_state = if repro.get("audit_ok").and_then(|v| v.as_bool()).unwrap_or(false) {
        "audit_ready"
    } else {
        "audit_alert"
    };
    Ok(serde_json::json!({
        "schema_version": 1,
        "run_dir": run_dir.display().to_string(),
        "status": {
            "run_state": run_state,
            "quality_state": quality_state,
            "evidence_state": evidence_state
        },
        "details": {
            "corrupted_artifact_count": corrupted,
            "replay_fingerprint": repro.get("replay_fingerprint").cloned().unwrap_or(serde_json::Value::Null)
        }
    }))
}

fn channel_summary_report(run_dir: &Path) -> Result<serde_json::Value> {
    ensure_run_dir_exists(run_dir)?;
    let obs_path = run_dir.join("artifacts").join("obs").join("obs.jsonl");
    let obs =
        if obs_path.exists() { read_obs_epochs(&obs_path).unwrap_or_default() } else { Vec::new() };
    let mut per_channel: std::collections::BTreeMap<String, (usize, f64)> =
        std::collections::BTreeMap::new();
    for epoch in &obs {
        for sat in &epoch.sats {
            let key = format!("{:?}-{}", sat.signal_id.sat.constellation, sat.signal_id.sat.prn);
            let entry = per_channel.entry(key).or_insert((0usize, 0.0));
            entry.0 = entry.0.saturating_add(1);
            entry.1 += sat.cn0_dbhz;
        }
    }
    let channels: Vec<serde_json::Value> = per_channel
        .iter()
        .map(|(channel, (count, sum_cn0))| {
            serde_json::json!({
                "channel": channel,
                "epochs_seen": count,
                "mean_cn0_dbhz": if *count == 0 { 0.0 } else { *sum_cn0 / *count as f64 }
            })
        })
        .collect();
    Ok(serde_json::json!({
        "schema_version": 1,
        "run_dir": run_dir.display().to_string(),
        "summary": {
            "obs_epochs": obs.len(),
            "channel_count": channels.len()
        },
        "channels": channels
    }))
}

fn export_bundle_report(run_dir: &Path, out_dir: Option<&PathBuf>) -> Result<serde_json::Value> {
    ensure_run_dir_exists(run_dir)?;
    let bundle_root = out_dir
        .cloned()
        .unwrap_or_else(|| run_dir.join("artifacts").join("diagnostics_export_bundle"));
    fs::create_dir_all(&bundle_root)?;

    let mut included = Vec::new();
    let mut checksums = serde_json::Map::new();
    let candidates = vec![
        run_dir.join("manifest.json"),
        run_dir.join("run_report.json"),
        run_dir.join("artifacts").join("validate").join("validation_report.json"),
        run_dir.join("artifacts").join("validate").join("validation_evidence_bundle.json"),
        run_dir.join("trace.ndjson"),
    ];
    for src in candidates {
        if !src.exists() || !src.is_file() {
            continue;
        }
        let file_name = src.file_name().and_then(|v| v.to_str()).unwrap_or("unknown").to_string();
        let dst = bundle_root.join(&file_name);
        fs::copy(&src, &dst)?;
        let bytes = fs::read(&dst)?;
        checksums.insert(file_name.clone(), serde_json::Value::String(sha256_hex(&bytes)));
        included.push(file_name);
    }
    included.sort();

    let bundle_manifest = serde_json::json!({
        "schema_version": 1,
        "source_run_dir": run_dir.display().to_string(),
        "included_files": included,
        "checksums_sha256": checksums
    });
    fs::write(
        bundle_root.join("bundle_manifest.json"),
        serde_json::to_string_pretty(&bundle_manifest)?,
    )?;

    Ok(serde_json::json!({
        "schema_version": 1,
        "run_dir": run_dir.display().to_string(),
        "bundle_dir": bundle_root.display().to_string(),
        "file_count": bundle_manifest
            .get("included_files")
            .and_then(|v| v.as_array())
            .map_or(0usize, Vec::len),
        "bundle_manifest": bundle_manifest
    }))
}

