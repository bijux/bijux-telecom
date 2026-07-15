#[cfg(test)]
mod diagnostics_tests {
    use super::{
        advanced_gate_report, artifact_inventory_report, compare_run_evidence, explain_run_scope,
        evidence_backed_rtk_fix_accepted, export_bundle_report, machine_catalog_report,
        medium_gate_report, navigation_decode::nav_reference_week, operator_status_report,
        refused_execution_status,
        replay_audit_report, rtk_advanced_solution_measurements, verify_repro_bundle,
        AdvancedGateMode,
    };
    use crate::schema_path;
    use std::fs;
    use std::path::PathBuf;

    fn create_base_run(name: &str, config_hash: &str) -> PathBuf {
        let base = std::env::temp_dir().join(format!(
            "bijux_{name}_{}_{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .expect("duration")
                .as_nanos()
        ));
        fs::create_dir_all(base.join("artifacts")).expect("artifacts dir");
        let manifest = serde_json::json!({
            "command": "run",
            "dataset_id": "demo",
            "config_hash": config_hash,
            "layout_schema_version": 1,
            "replay_scope": {"deterministic": true},
            "front_end_provenance": {"sample_rate_hz": 5000000.0}
        });
        fs::write(
            base.join("manifest.json"),
            serde_json::to_string_pretty(&manifest).expect("manifest json"),
        )
        .expect("manifest write");
        fs::write(base.join("run_report.json"), "{\"schema_version\":1}\n").expect("report write");
        fs::write(
            base.join("artifacts").join("obs.jsonl"),
            "{\"payload\":{\"artifact_id\":\"obs-1\"}}\n",
        )
        .expect("artifact write");
        base
    }

    fn dataset_entry_with_capture_start(
        capture_start_utc: &str,
    ) -> bijux_gnss_infra::api::DatasetEntry {
        bijux_gnss_infra::api::DatasetEntry {
            id: "demo".to_string(),
            path: "datasets/demo.iq16".to_string(),
            format: bijux_gnss_infra::api::signal::IqSampleFormat::Iq16Le,
            sample_rate_hz: Some(4_092_000.0),
            intermediate_freq_hz: Some(0.0),
            capture_start_utc: Some(capture_start_utc.to_string()),
            expected_sats: vec![1],
            expected_region: None,
            expected_time_utc: None,
            sidecar: None,
            recorded_capture: None,
        }
    }

    #[test]
    fn nav_reference_week_prefers_explicit_value() {
        let dataset = dataset_entry_with_capture_start("2022-05-08T00:00:00Z");

        let reference_week =
            nav_reference_week(Some(2209), Some(&dataset)).expect("reference week");

        assert_eq!(reference_week, Some(2209));
    }

    #[test]
    fn nav_reference_week_derives_full_week_from_dataset_capture_time() {
        let dataset = dataset_entry_with_capture_start("2022-05-08T00:00:00Z");

        let reference_week = nav_reference_week(None, Some(&dataset)).expect("reference week");

        assert_eq!(reference_week, Some(2209));
    }

    #[test]
    fn explain_run_scope_reports_cache_replay_and_identity() {
        let base = std::env::temp_dir().join(format!(
            "bijux_diagnostics_explain_{}_{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .expect("duration")
                .as_nanos()
        ));
        fs::create_dir_all(base.join("artifacts")).expect("artifacts dir");

        let manifest = serde_json::json!({
            "command": "run",
            "dataset_id": "demo",
            "config_hash": "abc123",
            "toolchain": "rustc",
            "features": [],
            "replay_scope": {
                "deterministic": true,
                "resume": false,
                "explicit_output_dir": true
            }
        });
        fs::write(
            base.join("manifest.json"),
            serde_json::to_string_pretty(&manifest).expect("manifest json"),
        )
        .expect("manifest write");

        let obs_line = serde_json::json!({
            "header": {"schema_version":1},
            "payload": {"artifact_id":"obs-epoch-0001","epoch_id":"epoch-0001"}
        });
        fs::write(
            base.join("artifacts").join("obs.jsonl"),
            format!("{}\n", serde_json::to_string(&obs_line).expect("obs line")),
        )
        .expect("obs write");

        let trace_line = serde_json::json!({
            "name":"acquisition_code_fft_cache_miss",
            "fields":[["reason","incompatible_assumptions"]]
        });
        fs::write(
            base.join("trace.ndjson"),
            format!("{}\n", serde_json::to_string(&trace_line).expect("trace line")),
        )
        .expect("trace write");

        let summary = explain_run_scope(PathBuf::as_path(&base)).expect("summary");
        assert_eq!(
            summary
                .get("replay_scope")
                .and_then(|v| v.get("deterministic"))
                .and_then(|v| v.as_bool()),
            Some(true)
        );
        assert_eq!(
            summary
                .get("cache_behavior")
                .and_then(|v| v.get("miss_reason_counts"))
                .and_then(|v| v.get("incompatible_assumptions"))
                .and_then(|v| v.as_u64()),
            Some(1)
        );
        assert_eq!(
            summary
                .get("artifact_identity_coverage")
                .and_then(|v| v.get("obs.jsonl"))
                .and_then(|v| v.get("has_artifact_id"))
                .and_then(|v| v.as_bool()),
            Some(true)
        );

        let _ = fs::remove_dir_all(base);
    }

    #[test]
    fn verify_repro_bundle_reports_hashes_and_fingerprint() {
        let base = std::env::temp_dir().join(format!(
            "bijux_verify_repro_{}_{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .expect("duration")
                .as_nanos()
        ));
        fs::create_dir_all(base.join("artifacts")).expect("artifacts dir");
        let manifest = serde_json::json!({
            "command": "validate",
            "config_hash": "cfg",
            "dataset_id": "demo",
            "layout_schema_version": 1,
            "replay_scope": {"deterministic": true},
            "front_end_provenance": {"sample_rate_hz": 5000000.0}
        });
        fs::write(
            base.join("manifest.json"),
            serde_json::to_string_pretty(&manifest).expect("manifest"),
        )
        .expect("manifest write");
        fs::write(base.join("run_report.json"), "{\"schema_version\":1}\n").expect("report write");
        fs::write(base.join("artifacts").join("obs.jsonl"), "{\"payload\":{}}\n")
            .expect("artifact");

        let report = verify_repro_bundle(PathBuf::as_path(&base)).expect("report");
        assert_eq!(report.get("audit_ok").and_then(|v| v.as_bool()), Some(true));
        assert!(report.get("replay_fingerprint").and_then(|v| v.as_str()).is_some());
        assert!(report.get("manifest_sha256").and_then(|v| v.as_str()).is_some());

        let _ = fs::remove_dir_all(base);
    }

    #[test]
    fn compare_run_evidence_reports_repro_and_quality_deltas() {
        let baseline = create_base_run("compare_base", "cfg-a");
        let candidate = create_base_run("compare_cand", "cfg-a");
        let report =
            compare_run_evidence(PathBuf::as_path(&baseline), PathBuf::as_path(&candidate))
                .expect("compare");
        assert_eq!(
            report
                .get("reproducibility")
                .and_then(|v| v.get("fingerprint_match"))
                .and_then(|v| v.as_bool()),
            Some(true)
        );
        assert!(report.get("quality").is_some());
        let _ = fs::remove_dir_all(baseline);
        let _ = fs::remove_dir_all(candidate);
    }

    #[test]
    fn replay_audit_report_classifies_fingerprint_drift() {
        let baseline = create_base_run("audit_base", "cfg-a");
        let candidate = create_base_run("audit_cand", "cfg-b");
        let report = replay_audit_report(PathBuf::as_path(&baseline), PathBuf::as_path(&candidate))
            .expect("replay audit");
        assert_eq!(
            report.get("classification").and_then(|v| v.as_str()),
            Some("scientific_or_configuration_drift")
        );
        let _ = fs::remove_dir_all(baseline);
        let _ = fs::remove_dir_all(candidate);
    }

    #[test]
    fn advanced_gate_report_passes_for_supported_rtk_claims() {
        let run_dir = create_base_run("advanced_gate", "cfg-gate");
        fs::create_dir_all(run_dir.join("artifacts").join("rtk")).expect("rtk dir");
        fs::create_dir_all(run_dir.join("artifacts").join("validate")).expect("validate dir");

        let support = serde_json::json!({
            "schema_version": 1,
            "rows": [
                {
                    "mode": "Rtk",
                    "maturity": "Experimental",
                    "real_solver": true,
                    "required_inputs": ["base_obs", "rover_obs", "ephemeris", "base_ecef"],
                    "notes": "supported"
                }
            ]
        });
        fs::write(
            run_dir.join("artifacts").join("rtk").join("rtk_support_matrix.json"),
            serde_json::to_string_pretty(&support).expect("support json"),
        )
        .expect("support write");

        let evidence = serde_json::json!({
            "schema_version": 1,
            "claim_evidence_guard": {
                "supported": true,
                "violations": []
            }
        });
        fs::write(
            run_dir.join("artifacts").join("validate").join("validation_evidence_bundle.json"),
            serde_json::to_string_pretty(&evidence).expect("evidence json"),
        )
        .expect("evidence write");

        let report = advanced_gate_report(PathBuf::as_path(&run_dir), AdvancedGateMode::Rtk)
            .expect("advanced gate");
        assert_eq!(report.get("gate_passed").and_then(|v| v.as_bool()), Some(true));

        let _ = fs::remove_dir_all(run_dir);
    }

    #[test]
    fn advanced_gate_report_marks_ppp_support_as_not_ready() {
        let run_dir = create_base_run("advanced_gate_ppp", "cfg-gate-ppp");
        fs::create_dir_all(run_dir.join("artifacts").join("rtk")).expect("rtk dir");
        fs::create_dir_all(run_dir.join("artifacts").join("validate")).expect("validate dir");

        let support = serde_json::json!({
            "schema_version": 2,
            "rows": [
                {
                    "mode": "Ppp",
                    "maturity": "NotReady",
                    "real_solver": false,
                    "required_inputs": ["multi_frequency_obs", "products_or_broadcast", "time_consistent_epochs"],
                    "notes": "not ready"
                }
            ]
        });
        fs::write(
            run_dir.join("artifacts").join("rtk").join("rtk_support_matrix.json"),
            serde_json::to_string_pretty(&support).expect("support json"),
        )
        .expect("support write");

        let evidence = serde_json::json!({
            "schema_version": 1,
            "claim_evidence_guard": {
                "supported": false,
                "violations": []
            }
        });
        fs::write(
            run_dir.join("artifacts").join("validate").join("validation_evidence_bundle.json"),
            serde_json::to_string_pretty(&evidence).expect("evidence json"),
        )
        .expect("evidence write");

        let report = advanced_gate_report(PathBuf::as_path(&run_dir), AdvancedGateMode::Ppp)
            .expect("advanced gate");
        assert_eq!(report.get("gate_passed").and_then(|v| v.as_bool()), Some(false));
        assert_eq!(report.get("claim_level").and_then(|v| v.as_str()), Some("not_ready"));
        let reasons = report.get("reasons").and_then(|v| v.as_array()).expect("reasons");
        assert!(reasons.iter().any(|value| value.as_str() == Some("mode_not_ready")));

        let _ = fs::remove_dir_all(run_dir);
    }

    #[test]
    fn artifact_inventory_report_lists_groups() {
        let run_dir = create_base_run("artifact_inventory", "cfg-artifacts");
        fs::create_dir_all(run_dir.join("artifacts").join("obs")).expect("obs dir");
        fs::write(run_dir.join("artifacts").join("obs").join("obs.jsonl"), "{\"payload\":{}}\n")
            .expect("obs write");
        let report =
            artifact_inventory_report(PathBuf::as_path(&run_dir)).expect("inventory report");
        assert!(report.get("groups").and_then(|v| v.get("obs")).is_some());
        let _ = fs::remove_dir_all(run_dir);
    }

    #[test]
    fn medium_gate_report_exposes_gate_flag() {
        let run_dir = create_base_run("medium_gate", "cfg-medium");
        fs::create_dir_all(run_dir.join("artifacts").join("validate")).expect("validate dir");
        fs::write(
            run_dir.join("artifacts").join("validate").join("validation_evidence_bundle.json"),
            "{\"claim_evidence_guard\":{\"supported\":true}}\n",
        )
        .expect("evidence write");
        let report = medium_gate_report(PathBuf::as_path(&run_dir)).expect("medium gate");
        assert!(report.get("gate_passed").is_some());
        let _ = fs::remove_dir_all(run_dir);
    }

    #[test]
    fn operator_status_report_emits_state_labels() {
        let run_dir = create_base_run("operator_status", "cfg-status");
        fs::create_dir_all(run_dir.join("artifacts").join("validate")).expect("validate dir");
        fs::write(
            run_dir.join("artifacts").join("validate").join("validation_evidence_bundle.json"),
            "{\"claim_evidence_guard\":{\"supported\":false}}\n",
        )
        .expect("evidence write");
        let report = operator_status_report(PathBuf::as_path(&run_dir)).expect("status");
        assert!(report.get("status").is_some());
        let _ = fs::remove_dir_all(run_dir);
    }

    #[test]
    fn export_bundle_report_writes_bundle_manifest() {
        let run_dir = create_base_run("bundle_export", "cfg-bundle");
        let report = export_bundle_report(PathBuf::as_path(&run_dir), None).expect("bundle");
        let bundle_dir = report.get("bundle_dir").and_then(|v| v.as_str()).expect("bundle dir");
        let manifest = PathBuf::from(bundle_dir).join("bundle_manifest.json");
        assert!(manifest.exists());
        let _ = fs::remove_dir_all(run_dir);
    }

    #[test]
    fn machine_catalog_report_contains_compare_contract() {
        let report = machine_catalog_report();
        let has_compare = report
            .get("reports")
            .and_then(|v| v.as_array())
            .map(|rows| {
                rows.iter().any(|row| {
                    row.get("name")
                        .and_then(|v| v.as_str())
                        .map(|name| name == "diagnostics_compare")
                        .unwrap_or(false)
                })
            })
            .unwrap_or(false);
        assert!(has_compare);
    }

    #[test]
    fn rtk_single_difference_schema_requires_receiver_provenance() {
        let schema = fs::read_to_string(schema_path("rtk_sd_v1.schema.json")).expect("schema");
        let schema: serde_json::Value = serde_json::from_str(&schema).expect("schema json");
        let required = schema
            .get("definitions")
            .and_then(|defs| defs.get("RtkSingleDifference"))
            .and_then(|payload| payload.get("required"))
            .and_then(|required| required.as_array())
            .expect("required fields");
        let required_fields: Vec<&str> =
            required.iter().filter_map(|field| field.as_str()).collect();

        assert!(required_fields.contains(&"rover_pseudorange_m"));
        assert!(required_fields.contains(&"rover_signal_timing"));
        assert!(required_fields.contains(&"base_pseudorange_m"));
        assert!(required_fields.contains(&"base_signal_timing"));
    }

    #[test]
    fn refused_execution_status_maps_unsupported_model_explicitly() {
        let decision = bijux_gnss_infra::api::receiver::AdvancedPrereqDecision {
            ready: false,
            refusal_class: Some(
                bijux_gnss_infra::api::receiver::AdvancedRefusalClass::UnsupportedModel,
            ),
            reasons: vec!["unsupported_model".to_string()],
        };

        let (status, reason) = refused_execution_status(&decision);
        assert_eq!(status, bijux_gnss_infra::api::receiver::ExecutionStatus::Unsupported);
        assert_eq!(reason.as_deref(), Some("unsupported_model"));
    }

    #[test]
    fn rtk_advanced_solution_measurements_track_quality_payload() {
        let quality = bijux_gnss_infra::api::receiver::RtkBaselineQuality {
            epoch_idx: 9,
            fixed: true,
            sigma_e: 0.03,
            sigma_n: 0.04,
            sigma_u: 0.05,
            used_sats: 7,
            residual_rms_m: 0.02,
            predicted_rms_m: 0.01,
            hpl_m: 0.30,
            vpl_m: 0.45,
            separation_sig: None,
            separation_max_m: None,
        };

        let measurements = rtk_advanced_solution_measurements(Some(&quality));
        assert_eq!(measurements.sigma_h_m, Some(0.05));
        assert_eq!(measurements.sigma_v_m, Some(0.05));
        assert_eq!(measurements.residual_rms_m, Some(0.02));
        assert_eq!(measurements.predicted_rms_m, Some(0.01));
        assert_eq!(measurements.hpl_m, Some(0.30));
        assert_eq!(measurements.vpl_m, Some(0.45));
    }

    #[test]
    fn evidence_backed_rtk_fix_accepted_only_reports_final_fixed_claims() {
        assert!(evidence_backed_rtk_fix_accepted(
            bijux_gnss_infra::api::receiver::AdvancedSolutionClaim::Fixed
        ));
        assert!(!evidence_backed_rtk_fix_accepted(
            bijux_gnss_infra::api::receiver::AdvancedSolutionClaim::Float
        ));
        assert!(!evidence_backed_rtk_fix_accepted(
            bijux_gnss_infra::api::receiver::AdvancedSolutionClaim::FallbackNav
        ));
    }

    #[test]
    fn rtk_baseline_schema_requires_execution_envelope() {
        let schema =
            fs::read_to_string(schema_path("rtk_baseline_v1.schema.json")).expect("schema");
        let schema: serde_json::Value = serde_json::from_str(&schema).expect("schema json");
        let required = schema
            .get("definitions")
            .and_then(|defs| defs.get("RtkBaseline"))
            .and_then(|payload| payload.get("required"))
            .and_then(|required| required.as_array())
            .expect("required fields");
        let required_fields: Vec<&str> =
            required.iter().filter_map(|field| field.as_str()).collect();

        assert!(required_fields.contains(&"epoch_idx"));
        assert!(required_fields.contains(&"status"));
        assert!(required_fields.contains(&"value"));

        let baseline_solution_required = schema
            .get("definitions")
            .and_then(|defs| defs.get("BaselineSolution"))
            .and_then(|payload| payload.get("required"))
            .and_then(|required| required.as_array())
            .expect("baseline solution required");
        let baseline_solution_fields: Vec<&str> =
            baseline_solution_required.iter().filter_map(|field| field.as_str()).collect();
        assert!(baseline_solution_fields.contains(&"enu_m"));
        assert!(baseline_solution_fields.contains(&"fixed"));
    }

    #[test]
    fn rtk_ambiguity_state_schema_requires_execution_envelope() {
        let schema =
            fs::read_to_string(schema_path("rtk_ambiguity_state_v1.schema.json")).expect("schema");
        let schema: serde_json::Value = serde_json::from_str(&schema).expect("schema json");
        let required = schema
            .get("definitions")
            .and_then(|defs| defs.get("RtkAmbiguityState"))
            .and_then(|payload| payload.get("required"))
            .and_then(|required| required.as_array())
            .expect("required fields");
        let required_fields: Vec<&str> =
            required.iter().filter_map(|field| field.as_str()).collect();

        assert!(required_fields.contains(&"epoch_idx"));
        assert!(required_fields.contains(&"status"));
        assert!(required_fields.contains(&"value"));

        let ambiguity_payload_required = schema
            .get("definitions")
            .and_then(|defs| defs.get("AmbiguityState"))
            .and_then(|payload| payload.get("required"))
            .and_then(|required| required.as_array())
            .expect("ambiguity payload required");
        let ambiguity_payload_fields: Vec<&str> =
            ambiguity_payload_required.iter().filter_map(|field| field.as_str()).collect();
        assert!(ambiguity_payload_fields.contains(&"float_count"));
        assert!(ambiguity_payload_fields.contains(&"fixed_count"));
    }
}
