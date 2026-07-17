use super::*;

#[derive(Args)]
pub(crate) struct ValidateArtifactsArgs {
    #[command(flatten)]
    pub(crate) common: CommonArgs,

    /// ObsEpoch JSONL file
    #[arg(long, value_name = "FILE")]
    pub(crate) obs: Option<PathBuf>,

    /// Ephemeris JSON file
    #[arg(long, value_name = "FILE")]
    pub(crate) eph: Option<PathBuf>,

    /// Require non-empty files
    #[arg(long)]
    pub(crate) strict: bool,
}

#[derive(Args)]
pub(crate) struct ValidateSidecarArgs {
    #[command(flatten)]
    pub(crate) common: CommonArgs,

    #[arg(long = "sidecar-file", value_name = "FILE")]
    pub(crate) sidecar_file: PathBuf,
}

#[derive(Args)]
pub(crate) struct ObservationValidationArgs {
    #[command(flatten)]
    pub(crate) common: CommonArgs,

    #[command(flatten)]
    pub(crate) input: RawCaptureInputArgs,

    /// Ephemeris JSON file (required for PVT)
    #[arg(long, value_name = "FILE")]
    pub(crate) eph: PathBuf,

    /// Reference solution JSONL for comparison
    #[arg(long, value_name = "FILE")]
    pub(crate) reference: PathBuf,

    /// PRN list for acquisition/tracking
    #[arg(
        long,
        value_delimiter = ',',
        default_value = "1,2,3,4,5",
        value_parser = clap::value_parser!(u8).range(1..=32)
    )]
    pub(crate) prn: Vec<u8>,

    /// Precise ephemeris SP3 file (optional)
    #[arg(long, value_name = "FILE")]
    pub(crate) sp3: Option<PathBuf>,

    /// Precise clock CLK file (optional)
    #[arg(long, value_name = "FILE")]
    pub(crate) clk: Option<PathBuf>,

    /// Bias-SINEX code bias file for dual-frequency code corrections (optional)
    #[arg(long = "bias-sinex", alias = "dcb", value_name = "FILE")]
    pub(crate) bias_sinex: Option<PathBuf>,
}

#[derive(Args)]
pub(crate) struct ValidateReferenceArgs {
    #[command(flatten)]
    pub(crate) common: CommonArgs,

    /// Run directory containing artifacts/
    #[arg(long, value_name = "DIR")]
    pub(crate) run_dir: PathBuf,

    /// Reference trajectory file (JSONL or CSV)
    #[arg(long, value_name = "FILE")]
    pub(crate) reference: PathBuf,

    /// Alignment policy for reference (nearest or linear)
    #[arg(long, value_enum, default_value_t = ReferenceAlign::Nearest)]
    pub(crate) align: ReferenceAlign,
}
