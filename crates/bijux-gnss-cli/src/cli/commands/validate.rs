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
    let schema_data = fs::read_to_string(schema_path)?;
    let schema_json: serde_json::Value = serde_json::from_str(&schema_data)?;
    let compiled = JSONSchema::compile(&schema_json)
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
    if let Err(errors) = compiled.validate(&json) {
        let mut messages = Vec::new();
        for error in errors {
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

fn validate_jsonl_schema(schema_path: &Path, data_path: &Path, strict: bool) -> Result<()> {
    let schema_data = fs::read_to_string(schema_path)?;
    let schema_json: serde_json::Value = serde_json::from_str(&schema_data)?;
    let compiled = JSONSchema::compile(&schema_json)
        .map_err(|e| eyre!("invalid schema {}: {}", schema_path.display(), e))?;
    let data = fs::read_to_string(data_path)?;
    let mut count = 0usize;
    for (idx, line) in data.lines().enumerate() {
        if line.trim().is_empty() {
            continue;
        }
        let json: serde_json::Value = serde_json::from_str(line)?;
        count += 1;
        let errors: Vec<String> = match compiled.validate(&json) {
            Ok(()) => Vec::new(),
            Err(errors) => errors.map(|e| e.to_string()).collect(),
        };
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

fn validate_config_schema(profile: &ReceiverProfile) -> Result<()> {
    let schema_path = schema_path("receiver_profile.schema.json");
    let schema_json: serde_json::Value = if schema_path.exists() {
        let file_json: serde_json::Value =
            serde_json::from_str(&fs::read_to_string(&schema_path)?)?;
        if file_json.get("properties").is_some() {
            file_json
        } else {
            serde_json::to_value(schema_for!(ReceiverProfile))?
        }
    } else {
        serde_json::to_value(schema_for!(ReceiverProfile))?
    };
    let compiled = JSONSchema::compile(&schema_json).map_err(|e| eyre!("invalid schema: {}", e))?;
    let json = serde_json::to_value(profile)?;
    if let Err(errors) = compiled.validate(&json) {
        let mut messages = Vec::new();
        for error in errors {
            messages.push(error.to_string());
        }
        bail!("config schema validation failed: {}", messages.join(", "));
    }
    Ok(())
}

fn validate_sidecar_schema(sidecar: &SidecarSpec) -> Result<()> {
    let schema_path = schema_path("sidecar.schema.json");
    let schema_data = fs::read_to_string(schema_path)?;
    let schema_json: serde_json::Value = serde_json::from_str(&schema_data)?;
    let compiled =
        JSONSchema::compile(&schema_json).map_err(|e| eyre!("invalid sidecar schema: {}", e))?;
    let json = serde_json::to_value(sidecar)?;
    if let Err(errors) = compiled.validate(&json) {
        let mut messages = Vec::new();
        for error in errors {
            messages.push(error.to_string());
        }
        bail!("sidecar schema validation failed: {}", messages.join(", "));
    }
    Ok(())
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

    set_trace_dir(&common);
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
        &ReceiverProfile::default(),
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

    set_trace_dir(&common);
    let file = file.context("--file is required for validation")?;
    let profile = load_profile(&common)?;
    let dataset = load_dataset(&common)?;
    if profile.schema_version.0 != SchemaVersion::CURRENT.0 {
        bail!(
            "unsupported schema_version {}, expected {}",
            profile.schema_version.0,
            SchemaVersion::CURRENT.0
        );
    }

    validate_config_schema(&profile)?;

    let report = <ReceiverProfile as ValidateConfig>::validate(&profile);
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
        bijux_gnss_receiver::navigation::Navigation::new(profile.to_receiver_config());
    for obs_epoch in &obs {
        if let Some(sol) = nav_solver.solve_epoch(obs_epoch, &nav) {
            solutions.push(sol);
        }
    }

    #[cfg(feature = "precise-products")]
    let (products_ok, product_fallbacks) = {
        let mut products = bijux_gnss_nav::Products::new(
            bijux_gnss_nav::BroadcastProductsProvider::new(nav.clone()),
        );
        if let Some(path) = sp3 {
            let data = fs::read_to_string(path)?;
            let sp3 = data
                .parse::<bijux_gnss_nav::Sp3Provider>()
                .map_err(|e| eyre!("sp3 parse error: {}", e))?;
            products = products.with_sp3(sp3);
        }
        if let Some(path) = clk {
            let data = fs::read_to_string(path)?;
            let clk = data
                .parse::<bijux_gnss_nav::ClkProvider>()
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
    )?;
    let out_dir = artifacts_dir(&common, "validate", dataset.as_ref())?;
    let out = out_dir.join("validation_report.json");
    fs::write(&out, serde_json::to_string_pretty(&report)?)?;
    println!("wrote {}", out.display());
    write_manifest(&common, "validate", &profile, dataset.as_ref(), &report)?;

    Ok(())
}
