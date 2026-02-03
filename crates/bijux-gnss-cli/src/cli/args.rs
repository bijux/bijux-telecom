#[derive(Parser)]
#[command(name = "bijux", version, about = "bijux-gnss CLI")]
struct Cli {
    #[command(subcommand)]
    command: AppCommand,
}

#[derive(Subcommand)]
enum AppCommand {
    /// GNSS-related commands
    Gnss {
        #[command(subcommand)]
        command: GnssCommand,
    },
}

#[derive(Args, Clone)]
struct CommonArgs {
    /// Receiver profile config (TOML)
    #[arg(long)]
    config: Option<PathBuf>,

    /// Dataset ID from datasets/registry.toml
    #[arg(long)]
    dataset: Option<String>,

    /// Allow runs without a registered dataset id
    #[arg(long)]
    unregistered_dataset: bool,

    /// Output directory for artifacts (run.json, reports)
    #[arg(long, alias = "output")]
    out: Option<PathBuf>,

    /// Report output format
    #[arg(long, value_enum, default_value_t = ReportFormat::Table)]
    report: ReportFormat,

    /// Override deterministic seed
    #[arg(long)]
    seed: Option<u64>,

    /// Force deterministic execution
    #[arg(long)]
    deterministic: bool,

    /// Dump trace artifacts (requires receiver feature `trace-dump`)
    #[arg(long)]
    dump: Option<PathBuf>,

    /// Sidecar metadata file for raw IQ
    #[arg(long)]
    sidecar: Option<PathBuf>,

    /// Resume from a previous run directory
    #[arg(long)]
    resume: Option<PathBuf>,
}

#[derive(Copy, Clone, ValueEnum)]
enum ReportFormat {
    Json,
    Table,
}

#[derive(Copy, Clone, ValueEnum)]
enum RefPolicy {
    Global,
    PerConstellation,
}

#[derive(Debug, Deserialize)]
#[allow(dead_code)]
struct DatasetRegistry {
    version: u32,
    entries: Vec<DatasetEntry>,
}

#[derive(Debug, Serialize, Deserialize, Clone)]
#[allow(dead_code)]
struct DatasetEntry {
    id: String,
    path: String,
    format: String,
    sample_rate_hz: f64,
    intermediate_freq_hz: f64,
    expected_sats: Vec<u8>,
    expected_region: Option<String>,
    expected_time_utc: Option<String>,
    sidecar: Option<String>,
}

#[derive(Debug, Serialize, Deserialize)]
struct SidecarSpec {
    sample_rate_hz: f64,
    #[serde(default)]
    offset_bytes: u64,
}

#[derive(Debug, Serialize)]
struct RunManifest {
    command: String,
    timestamp_unix_ms: u128,
    git_hash: String,
    git_dirty: bool,
    config_hash: String,
    config_snapshot: Option<String>,
    dataset_id: Option<String>,
    dataset_metadata: Option<DatasetEntry>,
    build_profile: String,
    cpu_features: Vec<String>,
    toolchain: String,
    features: Vec<String>,
    summary: serde_json::Value,
}

#[derive(Debug, Serialize)]
struct RunIndexEntry {
    run_dir: String,
    command: String,
    timestamp_unix_ms: u128,
    git_hash: String,
    dataset_id: Option<String>,
    config_hash: String,
    summary: serde_json::Value,
}
