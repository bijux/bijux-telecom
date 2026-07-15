fn load_config(common: &CommonArgs) -> Result<ReceiverConfig> {
    match &common.config {
        Some(path) => load_config_from_path(path),
        None => Ok(ReceiverConfig::default()),
    }
}

fn load_config_from_path(path: &Path) -> Result<ReceiverConfig> {
    let contents = fs::read_to_string(path)
        .with_context(|| format!("failed to read config {}", path.display()))?;
    let profile: ReceiverConfig = toml::from_str(&contents)
        .with_context(|| format!("failed to parse config {}", path.display()))?;
    if profile.schema_version.0 != SchemaVersion::CURRENT.0 {
        return Err(eyre!(
            "unsupported schema_version {}, expected {}",
            profile.schema_version.0,
            SchemaVersion::CURRENT.0
        ));
    }
    Ok(profile)
}

fn load_dataset(common: &CommonArgs) -> Result<Option<DatasetEntry>> {
    let Some(id) = &common.dataset else {
        if common.unregistered_dataset {
            return Ok(None);
        }
        return Err(classified_error(
            CliErrorClass::OperatorMisconfiguration,
            "dataset id is required (use --dataset or --unregistered-dataset)",
        ));
    };
    let registry_path = PathBuf::from("datasets/registry.toml");
    let registry = DatasetRegistry::load(&registry_path)
        .with_context(|| format!("failed to parse {}", registry_path.display()))?;
    let entry = registry.find(id).ok_or_else(|| {
        classified_error(
            CliErrorClass::OperatorMisconfiguration,
            format!("dataset not found: {id}"),
        )
    })?;
    Ok(Some(entry))
}

fn resolve_input_file(file: Option<&PathBuf>, dataset: Option<&DatasetEntry>) -> Result<PathBuf> {
    if let Some(file) = file {
        return Ok(file.clone());
    }
    if let Some(dataset) = dataset {
        return Ok(PathBuf::from(&dataset.path));
    }
    Err(classified_error(
        CliErrorClass::OperatorMisconfiguration,
        "no input file provided; use --file or --dataset",
    ))
}

fn resolve_raw_iq_metadata(
    common: &CommonArgs,
    dataset: Option<&DatasetEntry>,
) -> Result<RawIqMetadata> {
    let metadata =
        bijux_gnss_infra::api::resolve_raw_iq_metadata(dataset, common.sidecar.as_deref())?;
    validate_sidecar_schema(&metadata)?;
    Ok(metadata)
}

fn apply_raw_iq_metadata(
    profile: &mut ReceiverConfig,
    metadata: &RawIqMetadata,
    sampling_hz: Option<f64>,
    if_hz: Option<f64>,
) -> Result<()> {
    enforce_locked_capture_value("sample_rate_hz", sampling_hz, metadata.sample_rate_hz)?;
    enforce_locked_capture_value("intermediate_freq_hz", if_hz, metadata.intermediate_freq_hz)?;
    profile.sample_rate_hz = metadata.sample_rate_hz;
    profile.intermediate_freq_hz = metadata.intermediate_freq_hz;
    if let Some(bits) = metadata.quantization_bits {
        profile.quantization_bits = bits;
    }
    Ok(())
}

fn enforce_locked_capture_value(
    field: &str,
    cli_value: Option<f64>,
    metadata_value: f64,
) -> Result<()> {
    let Some(cli_value) = cli_value else {
        return Ok(());
    };
    if (cli_value - metadata_value).abs() > 1e-9 {
        return Err(classified_error(
            CliErrorClass::OperatorMisconfiguration,
            format!(
                "{field} must come from explicit raw IQ metadata; update the dataset registry or sidecar instead of overriding it on the command line"
            ),
        ));
    }
    Ok(())
}
