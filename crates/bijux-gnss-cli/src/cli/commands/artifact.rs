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
        } => {
            set_trace_dir(&common);
            validate_artifact_file(&file, kind.as_deref(), strict)?;
            let summary = serde_json::json!({
                "file": file.display().to_string(),
                "kind": kind.clone().unwrap_or_else(|| "auto".to_string()),
                "strict": strict
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

fn validate_artifact_file(path: &Path, kind: Option<&str>, strict: bool) -> Result<()> {
    let data = fs::read_to_string(path)?;
    if strict && data.trim().is_empty() {
        bail!("artifact is empty: {}", path.display());
    }
    let kind = kind
        .map(|k| k.to_lowercase())
        .or_else(|| detect_kind_from_path(path))
        .unwrap_or_else(|| "unknown".to_string());

    match kind.as_str() {
        "obs" => validate_obs_artifact(&data)?,
        "track" => validate_track_artifact(&data)?,
        "acq" => validate_acq_artifact(&data)?,
        "eph" | "ephemeris" => validate_ephemeris_artifact(&data)?,
        "pvt" | "nav" => validate_pvt_artifact(&data)?,
        "ppp" => validate_generic_jsonl(&data, "ppp")?,
        "rtk" => validate_generic_jsonl(&data, "rtk")?,
        _ => bail!("unknown artifact kind; pass --kind"),
    }
    println!("artifact ok: {}", path.display());
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

fn validate_obs_artifact(data: &str) -> Result<()> {
    let mut epochs = Vec::new();
    for line in data.lines() {
        if line.trim().is_empty() {
            continue;
        }
        let wrapped: ObsEpochV1 = serde_json::from_str(line)?;
        if !ArtifactCompatibility::is_supported(wrapped.header.schema_version) {
            bail!(
                "unsupported obs schema_version {}",
                wrapped.header.schema_version
            );
        }
        for event in check_obs_epoch_finite(&wrapped.epoch) {
            eprintln!("diagnostic {:?}: {}", event.severity, event.message);
        }
        epochs.push(wrapped.epoch);
    }
    validate_obs_epochs(&epochs).map_err(|err| eyre!("obs epoch validation failed: {err}"))?;
    Ok(())
}

fn validate_track_artifact(data: &str) -> Result<()> {
    for line in data.lines() {
        if line.trim().is_empty() {
            continue;
        }
        let wrapped: TrackEpochV1 = serde_json::from_str(line)?;
        if !ArtifactCompatibility::is_supported(wrapped.header.schema_version) {
            bail!(
                "unsupported track schema_version {}",
                wrapped.header.schema_version
            );
        }
    }
    Ok(())
}

fn validate_acq_artifact(data: &str) -> Result<()> {
    for line in data.lines() {
        if line.trim().is_empty() {
            continue;
        }
        let wrapped: AcqResultV1 = serde_json::from_str(line)?;
        if !ArtifactCompatibility::is_supported(wrapped.header.schema_version) {
            bail!(
                "unsupported acq schema_version {}",
                wrapped.header.schema_version
            );
        }
    }
    Ok(())
}

fn validate_ephemeris_artifact(data: &str) -> Result<()> {
    let wrapped: GpsEphemerisV1 = serde_json::from_str(data)?;
    if !ArtifactCompatibility::is_supported(wrapped.header.schema_version) {
        bail!(
            "unsupported ephemeris schema_version {}",
            wrapped.header.schema_version
        );
    }
    Ok(())
}

fn validate_pvt_artifact(data: &str) -> Result<()> {
    for line in data.lines() {
        if line.trim().is_empty() {
            continue;
        }
        let wrapped: NavSolutionEpochV1 = serde_json::from_str(line)?;
        if !ArtifactCompatibility::is_supported(wrapped.header.schema_version) {
            bail!(
                "unsupported pvt schema_version {}",
                wrapped.header.schema_version
            );
        }
        for event in check_nav_solution_finite(&wrapped.epoch) {
            eprintln!("diagnostic {:?}: {}", event.severity, event.message);
        }
    }
    Ok(())
}

fn validate_generic_jsonl(data: &str, label: &str) -> Result<()> {
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
        if !ArtifactCompatibility::is_supported(schema_version as u32) {
            bail!("{label} unsupported schema_version {schema_version}");
        }
    }
    Ok(())
}
