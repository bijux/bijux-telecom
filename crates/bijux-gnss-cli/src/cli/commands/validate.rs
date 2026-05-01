#[cfg(feature = "schema-validate")]
use jsonschema::validator_for;

#[derive(Copy, Clone)]
enum CsvType {
    U64,
    U8,
    F64,
    Bool,
    Str,
}

fn validate_csv_schema(path: &Path, expected: &str, types: &[CsvType]) -> Result<()> {
    let data = fs::read_to_string(path)?;
    let mut lines = data.lines();
    let header = lines.next().unwrap_or_default();
    if header.trim() != expected {
        bail!(
            "csv schema mismatch in {}: expected {}, got {}",
            path.display(),
            expected,
            header
        );
    }
    for (idx, line) in lines.enumerate() {
        if line.trim().is_empty() {
            continue;
        }
        let cols: Vec<_> = line.split(',').collect();
        if cols.len() != types.len() {
            bail!(
                "csv schema mismatch in {} line {}: expected {} columns, got {}",
                path.display(),
                idx + 1,
                types.len(),
                cols.len()
            );
        }
        for (value, kind) in cols.iter().zip(types.iter()) {
            match kind {
                CsvType::U64 => {
                    value.parse::<u64>().map_err(|_| {
                        eyre!("csv parse error in {} line {}", path.display(), idx + 1)
                    })?;
                }
                CsvType::U8 => {
                    value.parse::<u8>().map_err(|_| {
                        eyre!("csv parse error in {} line {}", path.display(), idx + 1)
                    })?;
                }
                CsvType::F64 => {
                    value.parse::<f64>().map_err(|_| {
                        eyre!("csv parse error in {} line {}", path.display(), idx + 1)
                    })?;
                }
                CsvType::Bool => {
                    value.parse::<bool>().map_err(|_| {
                        eyre!("csv parse error in {} line {}", path.display(), idx + 1)
                    })?;
                }
                CsvType::Str => {
                    if value.trim().is_empty() {
                        bail!(
                            "csv parse error in {} line {}: empty string",
                            path.display(),
                            idx + 1
                        );
                    }
                }
            }
        }
    }
    Ok(())
}

fn validate_json_schema(schema_path: &Path, data_path: &Path, strict: bool) -> Result<()> {
    #[cfg(feature = "schema-validate")]
    {
    let schema_data = fs::read_to_string(schema_path)?;
    let schema_json: serde_json::Value = serde_json::from_str(&schema_data)?;
    let compiled = validator_for(&schema_json)
        .map_err(|e| eyre!("invalid schema {}: {}", schema_path.display(), e))?;
    let data = fs::read_to_string(data_path)?;
    let json: serde_json::Value = serde_json::from_str(&data)?;
    if strict {
        match &json {
            serde_json::Value::Array(items) if items.is_empty() => {
                bail!("{} is empty", data_path.display());
            }
            _ => {}
        }
    }
    if !compiled.is_valid(&json) {
        let mut messages = Vec::new();
        for error in compiled.iter_errors(&json) {
            messages.push(error.to_string());
        }
        bail!(
            "schema validation failed for {}: {}",
            data_path.display(),
            messages.join(", ")
        );
    }
    Ok(())
    }
    #[cfg(not(feature = "schema-validate"))]
    {
        let _ = schema_path;
        if strict {
            bail!(
                "schema validation disabled; enable --features schema-validate to validate {}",
                data_path.display()
            );
        }
        Ok(())
    }
}

fn validate_jsonl_schema(schema_path: &Path, data_path: &Path, strict: bool) -> Result<()> {
    #[cfg(feature = "schema-validate")]
    {
    let schema_data = fs::read_to_string(schema_path)?;
    let schema_json: serde_json::Value = serde_json::from_str(&schema_data)?;
    let compiled = validator_for(&schema_json)
        .map_err(|e| eyre!("invalid schema {}: {}", schema_path.display(), e))?;
    let data = fs::read_to_string(data_path)?;
    let mut count = 0usize;
    for (idx, line) in data.lines().enumerate() {
        if line.trim().is_empty() {
            continue;
        }
        let json: serde_json::Value = serde_json::from_str(line)?;
        count += 1;
        let errors: Vec<String> = compiled.iter_errors(&json).map(|e| e.to_string()).collect();
        if !errors.is_empty() {
            bail!(
                "schema validation failed for {} line {}: {}",
                data_path.display(),
                idx + 1,
                errors.join(", ")
            );
        }
    }
    if strict && count == 0 {
        bail!("{} is empty", data_path.display());
    }
    Ok(())
    }
    #[cfg(not(feature = "schema-validate"))]
    {
        let _ = schema_path;
        if strict {
            bail!(
                "schema validation disabled; enable --features schema-validate to validate {}",
                data_path.display()
            );
        }
        Ok(())
    }
}

fn validate_config_schema(profile: &ReceiverConfig) -> Result<()> {
    #[cfg(feature = "schema-validate")]
    {
    let schema_path = schema_path("receiver_profile.schema.json");
    let schema_json: serde_json::Value = if schema_path.exists() {
        let file_json: serde_json::Value =
            serde_json::from_str(&fs::read_to_string(&schema_path)?)?;
        if file_json.get("properties").is_some() {
            file_json
        } else {
            serde_json::to_value(schemars::schema_for!(ReceiverConfig))?
        }
    } else {
        serde_json::to_value(schemars::schema_for!(ReceiverConfig))?
    };
    let compiled = validator_for(&schema_json).map_err(|e| eyre!("invalid schema: {}", e))?;
    let json = serde_json::to_value(profile)?;
    if !compiled.is_valid(&json) {
        let mut messages = Vec::new();
        for error in compiled.iter_errors(&json) {
            messages.push(error.to_string());
        }
        bail!("config schema validation failed: {}", messages.join(", "));
    }
    Ok(())
    }
    #[cfg(not(feature = "schema-validate"))]
    {
        let _ = profile;
        Ok(())
    }
}

fn validate_sidecar_schema(sidecar: &SidecarSpec) -> Result<()> {
    #[cfg(feature = "schema-validate")]
    {
    let schema_path = schema_path("sidecar.schema.json");
    let schema_data = fs::read_to_string(schema_path)?;
    let schema_json: serde_json::Value = serde_json::from_str(&schema_data)?;
    let compiled =
        validator_for(&schema_json).map_err(|e| eyre!("invalid sidecar schema: {}", e))?;
    let json = serde_json::to_value(sidecar)?;
    if !compiled.is_valid(&json) {
        let mut messages = Vec::new();
        for error in compiled.iter_errors(&json) {
            messages.push(error.to_string());
        }
        bail!("sidecar schema validation failed: {}", messages.join(", "));
    }
    Ok(())
    }
    #[cfg(not(feature = "schema-validate"))]
    {
        let _ = sidecar;
        Ok(())
    }
}

fn handle_validateartifacts(command: GnssCommand) -> Result<()> {
    let GnssCommand::ValidateArtifacts {
        common,
        obs,
        eph,
        strict,
    } = command
    else {
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

fn handle_validate(command: GnssCommand) -> Result<()> {
    let GnssCommand::Validate {
        common,
        file,
        eph,
        reference,
        prn,
        sp3,
        clk,
    } = command
    else {
        bail!("invalid command for handler");
    };

    let _ = runtime_config_from_env(&common, None);
    let file = file.context("--file is required for validation")?;
    let profile = load_config(&common)?;
    let dataset = load_dataset(&common)?;
    if profile.schema_version.0 != SchemaVersion::CURRENT.0 {
        bail!(
            "unsupported schema_version {}, expected {}",
            profile.schema_version.0,
            SchemaVersion::CURRENT.0
        );
    }

    validate_config_schema(&profile)?;

    let report = <ReceiverConfig as ValidateConfig>::validate(&profile);
    if !report.errors.is_empty() {
        bail!(
            "config invalid: {}",
            report
                .errors
                .iter()
                .map(|e| e.message.as_str())
                .collect::<Vec<_>>()
                .join(", ")
        );
    }

    let mut obs = read_obs_epochs(&file)?;
    let nav = read_ephemeris(&eph)?;
    let reference_epochs = read_reference_epochs(&reference)?;

    if !prn.is_empty() {
        obs.iter_mut().for_each(|e| {
            e.sats.retain(|sat| prn.contains(&sat.signal_id.sat.prn));
        });
    }

    let mut solutions = Vec::new();
    let mut nav_solver =
        bijux_gnss_infra::api::receiver::Navigation::new(
            profile.to_pipeline_config(),
            runtime_config_from_env(&common, None),
        );
    for obs_epoch in &obs {
        if let Some(sol) = nav_solver.solve_epoch(obs_epoch, &nav) {
            solutions.push(sol);
        }
    }

    #[cfg(feature = "precise-products")]
    let (products_ok, product_fallbacks) = {
        let mut products = bijux_gnss_infra::api::nav::Products::new(
            bijux_gnss_infra::api::nav::BroadcastProductsProvider::new(nav.clone()),
        );
        if let Some(path) = sp3 {
            let data = fs::read_to_string(path)?;
            let sp3 = data
                .parse::<bijux_gnss_infra::api::nav::Sp3Provider>()
                .map_err(|e| eyre!("sp3 parse error: {}", e))?;
            products = products.with_sp3(sp3);
        }
        if let Some(path) = clk {
            let data = fs::read_to_string(path)?;
            let clk = data
                .parse::<bijux_gnss_infra::api::nav::ClkProvider>()
                .map_err(|e| eyre!("clk parse error: {}", e))?;
            products = products.with_clk(clk);
        }
        let ok = products.sp3.is_some() || products.clk.is_some();
        let fallbacks = if ok {
            Vec::new()
        } else {
            vec!["broadcast_only".to_string()]
        };
        (ok, fallbacks)
    };
    #[cfg(not(feature = "precise-products"))]
    let (products_ok, product_fallbacks) = {
        if sp3.is_some() || clk.is_some() {
            bail!("precise-products feature disabled; recompile with feature to use SP3/CLK");
        }
        (false, vec!["precise_products_disabled".to_string()])
    };

    let report = build_validation_report(
        &[],
        &obs,
        &solutions,
        &reference_epochs,
        profile.sample_rate_hz,
        products_ok,
        product_fallbacks,
        bijux_gnss_infra::api::receiver::ValidationSciencePolicy {
            min_mean_cn0_dbhz: profile.navigation.science_thresholds.min_mean_cn0_dbhz,
            max_pdop: profile.navigation.science_thresholds.max_pdop,
            max_residual_rms_m: profile.navigation.science_thresholds.max_residual_rms_m,
            min_used_satellites: profile.navigation.science_thresholds.min_used_satellites,
            min_lock_ratio: profile.navigation.science_thresholds.min_lock_ratio,
        },
    )?;
    let out_dir = artifacts_dir(&common, "validate", dataset.as_ref())?;
    let out = out_dir.join("validation_report.json");
    fs::write(&out, serde_json::to_string_pretty(&report)?)?;
    println!("wrote {}", out.display());
    write_manifest(&common, "validate", &profile, dataset.as_ref(), &report)?;

    Ok(())
}

fn handle_validate_reference(command: GnssCommand) -> Result<()> {
    let GnssCommand::ValidateReference {
        common,
        run_dir,
        reference,
        align,
    } = command
    else {
        bail!("invalid command for handler");
    };

    let _ = runtime_config_from_env(&common, None);
    let artifacts = run_dir.join("artifacts");
    let obs_path = artifacts.join("obs.jsonl");
    let nav_path = artifacts.join("pvt.jsonl");
    let obs = read_obs_epochs(&obs_path)?;
    let solutions = read_nav_solutions(&nav_path)?;
    let reference_epochs = read_reference_epochs(&reference)?;
    let align_policy = match align {
        ReferenceAlign::Nearest => bijux_gnss_infra::api::ReferenceAlign::Nearest,
        ReferenceAlign::Linear => bijux_gnss_infra::api::ReferenceAlign::Linear,
    };
    let aligned = bijux_gnss_infra::api::validate_reference(&solutions, &reference_epochs, align_policy)?;

    let report = build_validation_report(
        &[],
        &obs,
        &solutions,
        &aligned,
        0.0,
        false,
        vec!["run_dir_only".to_string()],
        bijux_gnss_infra::api::receiver::ValidationSciencePolicy::default(),
    )?;
    let out_dir = artifacts_dir(&common, "validate_reference", None)?;
    let out = out_dir.join("validation_report.json");
    fs::write(&out, serde_json::to_string_pretty(&report)?)?;
    let summary = serde_json::json!({ "report": out.display().to_string() });
    write_manifest(
        &common,
        "validate_reference",
        &ReceiverConfig::default(),
        None,
        &summary,
    )?;
    Ok(())
}
