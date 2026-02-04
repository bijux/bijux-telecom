fn handle_artifact(command: GnssCommand) -> Result<()> {
    let GnssCommand::Artifact { command } = command else {
        bail!("invalid command for handler");
    };

    match command {
        ArtifactCommand::Validate {
            common,
            file,
            kind,
            strict,
            report,
            fail_on,
        } => {
            set_trace_dir(&common);
            let result = artifact_validate(&file, kind.as_deref(), strict)?;
            let summary = bijux_gnss_infra::api::core::aggregate_diagnostics(&result.diagnostics);
            if let Some(path) = report {
                fs::write(&path, serde_json::to_string_pretty(&summary)?)?;
            }
            enforce_fail_policy(&result.diagnostics, fail_on)?;
            let summary = serde_json::json!({
                "file": file.display().to_string(),
                "kind": result.kind,
                "strict": strict,
                "diagnostics_total": summary.total
            });
            write_manifest(
                &common,
                "artifact_validate",
                &ReceiverConfig::default(),
                load_dataset(&common)?.as_ref(),
                &summary,
            )?;
        }
        ArtifactCommand::Explain { common, file } => {
            set_trace_dir(&common);
            explain_artifact(&common, &file)?;
        }
        ArtifactCommand::Convert {
            common,
            input,
            output,
            to,
        } => {
            set_trace_dir(&common);
            convert_artifact(&input, &output, &to)?;
            let summary = serde_json::json!({
                "input": input.display().to_string(),
                "output": output.display().to_string(),
                "to": to
            });
            write_manifest(
                &common,
                "artifact_convert",
                &ReceiverConfig::default(),
                load_dataset(&common)?.as_ref(),
                &summary,
            )?;
        }
    }
    Ok(())
}

fn explain_artifact(common: &CommonArgs, path: &Path) -> Result<()> {
    let result = artifact_explain(path)?;
    let output = render_artifact_explain(path, &result);
    print!("{output}");

    write_manifest(
        common,
        "artifact_explain",
        &ReceiverConfig::default(),
        load_dataset(common)?.as_ref(),
        &serde_json::json!({
            "file": path.display().to_string(),
            "kind": result.kind,
            "schema_version": result.header.schema_version,
            "diagnostics_total": result.diagnostics_total
        }),
    )?;

    Ok(())
}

fn convert_artifact(input: &Path, output: &Path, to: &str) -> Result<()> {
    if to != "v1" {
        bail!("only v1 conversion is supported for now");
    }
    let data = fs::read_to_string(input)?;
    fs::write(output, data)?;
    Ok(())
}

fn render_artifact_explain(path: &Path, result: &bijux_gnss_infra::api::ArtifactExplainResult) -> String {
    let header = &result.header;
    let mut out = String::new();
    out.push_str(&format!("artifact: {}\n", path.display()));
    out.push_str(&format!("kind: {}\n", result.kind));
    out.push_str(&format!("schema_version: {}\n", header.schema_version));
    out.push_str(&format!("producer: {}\n", header.producer));
    out.push_str(&format!(
        "producer_version: {}\n",
        header.producer_version
    ));
    out.push_str(&format!("created_at_unix_ms: {}\n", header.created_at_unix_ms));
    out.push_str(&format!("git_sha: {}\n", header.git_sha));
    out.push_str(&format!("git_dirty: {}\n", header.git_dirty));
    out.push_str(&format!("config_hash: {}\n", header.config_hash));
    out.push_str(&format!(
        "dataset_id: {}\n",
        header.dataset_id.as_deref().unwrap_or("none")
    ));
    out.push_str(&format!("toolchain: {}\n", header.toolchain));
    out.push_str(&format!("features: {}\n", header.features.join(", ")));
    out.push_str(&format!("deterministic: {}\n", header.deterministic));
    if result.entries > 0 {
        out.push_str(&format!("entries: {}\n", result.entries));
    }
    out.push_str(&format!(
        "diagnostics: total={} error={} warn={}\n",
        result.diagnostics_total, result.diagnostics_error, result.diagnostics_warn
    ));
    out
}

fn enforce_fail_policy(events: &[DiagnosticEvent], fail_on: DiagnosticFailOn) -> Result<()> {
    let mut has_error = false;
    let mut has_warn = false;
    for event in events {
        if matches!(event.severity, DiagnosticSeverity::Error) {
            has_error = true;
        }
        if matches!(event.severity, DiagnosticSeverity::Warning) {
            has_warn = true;
        }
        eprintln!("diagnostic {:?} {}: {}", event.severity, event.code, event.message);
    }
    match fail_on {
        DiagnosticFailOn::None => Ok(()),
        DiagnosticFailOn::Warn => {
            if has_error || has_warn {
                bail!("artifact validation failed due to diagnostics");
            }
            Ok(())
        }
        DiagnosticFailOn::Error => {
            if has_error {
                bail!("artifact validation failed with error severity diagnostics");
            }
            Ok(())
        }
    }
}

#[cfg(test)]
mod artifact_tests {
    use super::*;
    use std::path::PathBuf;

    #[test]
    fn explain_obs_fixture_prints_fields() {
        let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../bijux-gnss-core/tests/data/obs_fixture.jsonl");
        let result = bijux_gnss_infra::api::artifact_explain(&path).expect("explain fixture");
        let output = render_artifact_explain(&path, &result);
        assert!(output.contains("schema_version: 1"));
        assert!(output.contains("producer: bijux-gnss-core-fixture"));
        assert!(output.contains("producer_version: 0.1.0"));
        assert!(output.contains("entries: 1"));
    }
}
