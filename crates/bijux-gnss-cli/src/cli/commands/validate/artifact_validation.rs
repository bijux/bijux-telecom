use super::*;

pub(crate) fn handle_validate_artifacts(command: GnssCommand) -> Result<()> {
    let GnssCommand::ValidateArtifacts { common, obs, eph, strict } = command else {
        bail!("invalid command for handler");
    };

    let _ = runtime_config_from_env(&common, None);
    if obs.is_none() && eph.is_none() {
        bail!("--obs and/or --eph is required");
    }
    let dataset = load_dataset(&common)?;
    let mut checked = Vec::new();
    if let Some(path) = obs {
        validate_jsonl_schema(&schema_path("obs_epoch_v1.schema.json"), &path, strict)?;
        println!("obs ok: {}", path.display());
        checked.push("obs".to_string());
    }
    if let Some(path) = eph {
        validate_json_schema(&schema_path("gps_ephemeris_v1.schema.json"), &path, strict)?;
        println!("ephemeris ok: {}", path.display());
        checked.push("ephemeris".to_string());
    }
    let report = serde_json::json!({ "checked": checked });
    write_manifest(
        &common,
        "validate_artifacts",
        &ReceiverConfig::default(),
        dataset.as_ref(),
        &report,
    )?;

    Ok(())
}
