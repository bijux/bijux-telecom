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
            let summary = bijux_gnss_core::aggregate_diagnostics(&result.diagnostics);
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
                &ReceiverProfile::default(),
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
                &ReceiverProfile::default(),
                load_dataset(&common)?.as_ref(),
                &summary,
            )?;
        }
    }
    Ok(())
}

fn explain_artifact(common: &CommonArgs, path: &Path) -> Result<()> {
    let result = artifact_explain(path)?;
    let header = result.header;
    let kind = result.kind;
    let line_count = result.entries;
    println!("artifact: {}", path.display());
    println!("kind: {kind}");
    println!("schema_version: {}", header.schema_version);
    println!("created_at_unix_ms: {}", header.created_at_unix_ms);
    println!("git_sha: {}", header.git_sha);
    println!("git_dirty: {}", header.git_dirty);
    println!("config_hash: {}", header.config_hash);
    println!(
        "dataset_id: {}",
        header.dataset_id.as_deref().unwrap_or("none")
    );
    println!("toolchain: {}", header.toolchain);
    println!("features: {}", header.features.join(", "));
    println!("deterministic: {}", header.deterministic);
    if line_count > 0 {
        println!("entries: {line_count}");
    }
    let error_count = result.diagnostics_error;
    let warn_count = result.diagnostics_warn;
    let total = result.diagnostics_total;
    println!(
        "diagnostics: total={} error={} warn={}",
        total, error_count, warn_count
    );

    write_manifest(
        common,
        "artifact_explain",
        &ReceiverProfile::default(),
        load_dataset(common)?.as_ref(),
        &serde_json::json!({
            "file": path.display().to_string(),
            "kind": kind,
            "schema_version": header.schema_version,
            "diagnostics_total": total
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
