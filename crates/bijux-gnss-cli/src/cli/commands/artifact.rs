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
            let events = validate_artifact_file(&file, kind.as_deref(), strict)?;
            let summary = bijux_gnss_core::aggregate_diagnostics(&events);
            if let Some(path) = report {
                fs::write(&path, serde_json::to_string_pretty(&summary)?)?;
            }
            enforce_fail_policy(&events, fail_on)?;
            let summary = serde_json::json!({
                "file": file.display().to_string(),
                "kind": kind.clone().unwrap_or_else(|| "auto".to_string()),
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

fn validate_artifact_file(
    path: &Path,
    kind: Option<&str>,
    strict: bool,
) -> Result<Vec<DiagnosticEvent>> {
    let data = fs::read_to_string(path)?;
    if strict && data.trim().is_empty() {
        bail!("artifact is empty: {}", path.display());
    }
    let kind = kind
        .map(|k| k.to_lowercase())
        .or_else(|| detect_kind_from_path(path))
        .unwrap_or_else(|| "unknown".to_string());

    let events = match kind.as_str() {
        "obs" => validate_obs_artifact(&data)?,
        "track" => validate_track_artifact(&data)?,
        "acq" => validate_acq_artifact(&data)?,
        "eph" | "ephemeris" => validate_ephemeris_artifact(&data)?,
        "pvt" | "nav" => validate_pvt_artifact(&data)?,
        "ppp" => validate_generic_jsonl(&data, "ppp")?,
        "rtk" => validate_generic_jsonl(&data, "rtk")?,
        _ => bail!("unknown artifact kind; pass --kind"),
    };
    println!("artifact ok: {}", path.display());
    Ok(events)
}

fn convert_artifact(input: &Path, output: &Path, to: &str) -> Result<()> {
    if to != "v1" {
        bail!("only v1 conversion is supported for now");
    }
    let data = fs::read_to_string(input)?;
    fs::write(output, data)?;
    Ok(())
}

fn detect_kind_from_path(path: &Path) -> Option<String> {
    let name = path.file_name()?.to_string_lossy().to_lowercase();
    if name.contains("obs") {
        return Some("obs".to_string());
    }
    if name.contains("track") {
        return Some("track".to_string());
    }
    if name.contains("acq") {
        return Some("acq".to_string());
    }
    if name.contains("eph") {
        return Some("eph".to_string());
    }
    if name.contains("pvt") || name.contains("nav") {
        return Some("pvt".to_string());
    }
    if name.contains("ppp") {
        return Some("ppp".to_string());
    }
    if name.contains("rtk") {
        return Some("rtk".to_string());
    }
    None
}

fn validate_obs_artifact(data: &str) -> Result<Vec<DiagnosticEvent>> {
    let mut epochs = Vec::new();
    let mut events = Vec::new();
    for line in data.lines() {
        if line.trim().is_empty() {
            continue;
        }
        let wrapped: ObsEpochV1 = serde_json::from_str(line)?;
        if !ArtifactReadPolicy::is_supported(wrapped.header.schema_version) {
            bail!(
                "unsupported obs schema_version {}",
                wrapped.header.schema_version
            );
        }
        events.extend(wrapped.validate());
        epochs.push(wrapped.epoch);
    }
    if let Err(err) = validate_obs_epochs(&epochs) {
        events.push(DiagnosticEvent::new(
            DiagnosticSeverity::Error,
            "GNSS_OBS_VALIDATE_FAILED",
            format!("obs epoch validation failed: {err}"),
        ));
    }
    Ok(events)
}

fn validate_track_artifact(data: &str) -> Result<Vec<DiagnosticEvent>> {
    let mut events = Vec::new();
    for line in data.lines() {
        if line.trim().is_empty() {
            continue;
        }
        let wrapped: TrackEpochV1 = serde_json::from_str(line)?;
        if !ArtifactReadPolicy::is_supported(wrapped.header.schema_version) {
            bail!(
                "unsupported track schema_version {}",
                wrapped.header.schema_version
            );
        }
        events.extend(wrapped.validate());
    }
    Ok(events)
}

fn validate_acq_artifact(data: &str) -> Result<Vec<DiagnosticEvent>> {
    let mut events = Vec::new();
    for line in data.lines() {
        if line.trim().is_empty() {
            continue;
        }
        let wrapped: AcqResultV1 = serde_json::from_str(line)?;
        if !ArtifactReadPolicy::is_supported(wrapped.header.schema_version) {
            bail!(
                "unsupported acq schema_version {}",
                wrapped.header.schema_version
            );
        }
        events.extend(wrapped.validate());
    }
    Ok(events)
}

fn validate_ephemeris_artifact(data: &str) -> Result<Vec<DiagnosticEvent>> {
    let wrapped: GpsEphemerisV1 = serde_json::from_str(data)?;
    if !ArtifactReadPolicy::is_supported(wrapped.header.schema_version) {
        bail!(
            "unsupported ephemeris schema_version {}",
            wrapped.header.schema_version
        );
    }
    Ok(Vec::new())
}

fn validate_pvt_artifact(data: &str) -> Result<Vec<DiagnosticEvent>> {
    let mut events = Vec::new();
    for line in data.lines() {
        if line.trim().is_empty() {
            continue;
        }
        let wrapped: NavSolutionEpochV1 = serde_json::from_str(line)?;
        if !ArtifactReadPolicy::is_supported(wrapped.header.schema_version) {
            bail!(
                "unsupported pvt schema_version {}",
                wrapped.header.schema_version
            );
        }
        events.extend(wrapped.validate());
    }
    Ok(events)
}

fn validate_generic_jsonl(data: &str, label: &str) -> Result<Vec<DiagnosticEvent>> {
    for line in data.lines() {
        if line.trim().is_empty() {
            continue;
        }
        let value: serde_json::Value = serde_json::from_str(line)?;
        let header = value
            .get("header")
            .ok_or_else(|| eyre!("{label} artifact missing header"))?;
        let schema_version = header
            .get("schema_version")
            .and_then(|v| v.as_u64())
            .ok_or_else(|| eyre!("{label} artifact missing schema_version"))?;
        if !ArtifactReadPolicy::is_supported(schema_version as u32) {
            bail!("{label} unsupported schema_version {schema_version}");
        }
    }
    Ok(Vec::new())
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
