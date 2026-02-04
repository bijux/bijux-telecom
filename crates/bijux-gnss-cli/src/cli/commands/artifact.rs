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
    let data = fs::read_to_string(path)?;
    let mut header: Option<ArtifactHeaderV1> = None;
    let mut kind = detect_kind_from_path(path).unwrap_or_else(|| "unknown".to_string());
    let mut line_count = 0usize;

    if data.trim_start().starts_with('{') && !data.contains('\n') {
        if let Ok(wrapped) = serde_json::from_str::<GpsEphemerisV1>(&data) {
            header = Some(wrapped.header);
            kind = "ephemeris".to_string();
            line_count = wrapped.payload.len();
        } else if let Ok(wrapped) = serde_json::from_str::<serde_json::Value>(&data) {
            if let Some(value) = wrapped.get("header") {
                header = Some(serde_json::from_value(value.clone())?);
            }
        }
    } else {
        for line in data.lines() {
            if line.trim().is_empty() {
                continue;
            }
            if header.is_none() {
                let value: serde_json::Value = serde_json::from_str(line)?;
                if let Some(value) = value.get("header") {
                    header = Some(serde_json::from_value(value.clone())?);
                }
            }
            line_count += 1;
        }
    }

    let header = header.ok_or_else(|| eyre!("artifact header not found"))?;
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

    let events = validate_artifact_file(path, Some(&kind), false)?;
    let summary = bijux_gnss_core::aggregate_diagnostics(&events);
    let mut error_count = 0usize;
    let mut warn_count = 0usize;
    for entry in &summary.entries {
        match entry.severity {
            DiagnosticSeverity::Error => error_count += entry.count,
            DiagnosticSeverity::Warning => warn_count += entry.count,
            DiagnosticSeverity::Info => {}
        }
    }
    println!(
        "diagnostics: total={} error={} warn={}",
        summary.total, error_count, warn_count
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
            "diagnostics_total": summary.total
        }),
    )?;

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
        "ppp" => validate_ppp_artifact(&data)?,
        "rtk" => validate_rtk_artifact(&data)?,
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
        epochs.push(wrapped.payload);
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

fn validate_ppp_artifact(data: &str) -> Result<Vec<DiagnosticEvent>> {
    let mut events = Vec::new();
    for line in data.lines() {
        if line.trim().is_empty() {
            continue;
        }
        let wrapped: bijux_gnss_nav::PppEpochV1 = serde_json::from_str(line)?;
        if !ArtifactReadPolicy::is_supported(wrapped.header.schema_version) {
            bail!(
                "unsupported ppp schema_version {}",
                wrapped.header.schema_version
            );
        }
        events.extend(wrapped.validate());
    }
    Ok(events)
}

fn validate_rtk_artifact(data: &str) -> Result<Vec<DiagnosticEvent>> {
    let mut events = Vec::new();
    for line in data.lines() {
        if line.trim().is_empty() {
            continue;
        }
        let value: serde_json::Value = serde_json::from_str(line)?;
        let header: ArtifactHeaderV1 = serde_json::from_value(
            value
                .get("header")
                .cloned()
                .ok_or_else(|| eyre!("rtk artifact missing header"))?,
        )?;
        if !ArtifactReadPolicy::is_supported(header.schema_version) {
            bail!("unsupported rtk schema_version {}", header.schema_version);
        }
        let payload = value
            .get("payload")
            .cloned()
            .ok_or_else(|| eyre!("rtk artifact missing payload"))?;
        if payload.get("code_m").is_some() && payload.get("ref_sig").is_none() {
            let wrapped: bijux_gnss_receiver::rtk::RtkSdEpochV1 = serde_json::from_value(value)?;
            events.extend(wrapped.validate());
            continue;
        }
        if payload.get("ref_sig").is_some() {
            let wrapped: bijux_gnss_receiver::rtk::RtkDdEpochV1 = serde_json::from_value(value)?;
            events.extend(wrapped.validate());
            continue;
        }
        if payload.get("enu_m").is_some() {
            let wrapped: bijux_gnss_receiver::rtk::RtkBaselineEpochV1 =
                serde_json::from_value(value)?;
            events.extend(wrapped.validate());
            continue;
        }
        if payload.get("sigma_e").is_some() {
            let wrapped: bijux_gnss_receiver::rtk::RtkBaselineQualityV1 =
                serde_json::from_value(value)?;
            events.extend(wrapped.validate());
            continue;
        }
        if payload.get("fix_accepted").is_some() {
            let wrapped: bijux_gnss_receiver::rtk::RtkPrecisionV1 =
                serde_json::from_value(value)?;
            events.extend(wrapped.validate());
            continue;
        }
        if payload.get("fixed_count").is_some() {
            let wrapped: bijux_gnss_receiver::rtk::RtkFixAuditV1 =
                serde_json::from_value(value)?;
            events.extend(wrapped.validate());
            continue;
        }
    }
    Ok(events)
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
