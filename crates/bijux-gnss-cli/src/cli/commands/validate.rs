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
