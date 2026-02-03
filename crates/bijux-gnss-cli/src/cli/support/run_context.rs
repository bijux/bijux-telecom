use std::sync::OnceLock;

#[derive(Debug, Clone)]
#[allow(dead_code)]
struct RunContext {
    run_dir: PathBuf,
    artifacts_dir: PathBuf,
    logs_dir: PathBuf,
    command: String,
}

static RUN_CONTEXT: OnceLock<RunContext> = OnceLock::new();

fn now_unix_ms() -> u128 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_millis()
}

fn resolve_run_context(
    common: &CommonArgs,
    command: &str,
    dataset: Option<&DatasetEntry>,
) -> Result<&'static RunContext> {
    if let Some(ctx) = RUN_CONTEXT.get() {
        return Ok(ctx);
    }
    let run_dir = if let Some(resume) = &common.resume {
        resume.clone()
    } else if let Some(out) = &common.out {
        out.clone()
    } else {
        let dataset_tag = dataset
            .map(|d| d.id.clone())
            .unwrap_or_else(|| "unknown".to_string());
        let stamp = now_unix_ms();
        PathBuf::from("runs").join(format!("{stamp}_{dataset_tag}_{command}"))
    };
    let artifacts_dir = run_dir.join("artifacts");
    let logs_dir = run_dir.join("logs");
    fs::create_dir_all(&artifacts_dir)?;
    fs::create_dir_all(&logs_dir)?;
    let ctx = RunContext {
        run_dir,
        artifacts_dir,
        logs_dir,
        command: command.to_string(),
    };
    let _ = RUN_CONTEXT.set(ctx);
    Ok(RUN_CONTEXT.get().expect("run context set"))
}

fn run_dir(common: &CommonArgs, command: &str, dataset: Option<&DatasetEntry>) -> Result<PathBuf> {
    Ok(resolve_run_context(common, command, dataset)?.run_dir.clone())
}

fn artifacts_dir(
    common: &CommonArgs,
    command: &str,
    dataset: Option<&DatasetEntry>,
) -> Result<PathBuf> {
    Ok(resolve_run_context(common, command, dataset)?.artifacts_dir.clone())
}

fn append_run_index(run_dir: &Path, manifest: &RunManifest) -> Result<()> {
    let index_path = PathBuf::from("runs").join("index.jsonl");
    fs::create_dir_all(index_path.parent().unwrap_or(Path::new("runs")))?;
    let entry = RunIndexEntry {
        run_dir: run_dir.display().to_string(),
        command: manifest.command.clone(),
        timestamp_unix_ms: manifest.timestamp_unix_ms,
        git_hash: manifest.git_hash.clone(),
        dataset_id: manifest.dataset_id.clone(),
        config_hash: manifest.config_hash.clone(),
        summary: manifest.summary.clone(),
    };
    let line = serde_json::to_string(&entry)?;
    let mut file = fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open(index_path)?;
    writeln!(file, "{line}")?;
    Ok(())
}

fn artifact_header(
    common: &CommonArgs,
    profile: &ReceiverProfile,
    dataset: Option<&DatasetEntry>,
) -> Result<ArtifactHeaderV1> {
    let config_hash = hash_config(common.config.as_ref(), profile)?;
    Ok(ArtifactHeaderV1 {
        schema_version: ArtifactReadPolicy::LATEST,
        created_at_unix_ms: now_unix_ms(),
        git_sha: git_hash().unwrap_or_else(|| "unknown".to_string()),
        config_hash,
        dataset_id: dataset.map(|d| d.id.clone()),
        toolchain: std::env::var("RUSTC_VERSION").unwrap_or_else(|_| "unknown".to_string()),
        features: enabled_features(),
        deterministic: common.deterministic,
        git_dirty: git_dirty(),
    })
}

fn enabled_features() -> Vec<String> {
    let mut features = Vec::new();
    if let Ok(value) = std::env::var("BIJUX_GNSS_FEATURES") {
        for item in value.split(',') {
            let trimmed = item.trim();
            if !trimmed.is_empty() {
                features.push(trimmed.to_string());
            }
        }
    }
    features
}
