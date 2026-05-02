#[derive(Parser)]
#[command(
    name = "bijux",
    version,
    about = "GNSS receiver workflows: run, inspect, validate, diagnose, replay, and compare"
)]
struct Cli {
    #[command(subcommand)]
    command: AppCommand,
}

#[derive(Subcommand)]
enum AppCommand {
    /// GNSS receiver commands for operation, validation, diagnostics, and replay
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

#[derive(Debug, Serialize, Deserialize)]
struct SidecarSpec {
    sample_rate_hz: f64,
    #[serde(default)]
    offset_bytes: u64,
}
