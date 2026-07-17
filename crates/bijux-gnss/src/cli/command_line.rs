#[derive(Parser)]
#[command(
    name = "bijux",
    version,
    about = "GNSS receiver workflows: run, inspect, validate, diagnose, replay, and compare"
)]
struct CommandLine {
    #[command(subcommand)]
    command: ApplicationCommand,
}

#[derive(Subcommand)]
enum ApplicationCommand {
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

#[derive(Copy, Clone, ValueEnum)]
enum SyntheticQuantizationArg {
    Float32,
    Bipolar1Bit,
    Signed2Bit,
    Signed4Bit,
    Signed8Bit,
    Signed16Bit,
}

impl SyntheticQuantizationArg {
    fn into_quantization(self) -> bijux_gnss_infra::api::signal::IqQuantization {
        match self {
            Self::Float32 => bijux_gnss_infra::api::signal::IqQuantization::Float32,
            Self::Bipolar1Bit => bijux_gnss_infra::api::signal::IqQuantization::Bipolar1Bit,
            Self::Signed2Bit => bijux_gnss_infra::api::signal::IqQuantization::Signed2Bit,
            Self::Signed4Bit => bijux_gnss_infra::api::signal::IqQuantization::Signed4Bit,
            Self::Signed8Bit => bijux_gnss_infra::api::signal::IqQuantization::Signed8Bit,
            Self::Signed16Bit => bijux_gnss_infra::api::signal::IqQuantization::Signed16Bit,
        }
    }
}
