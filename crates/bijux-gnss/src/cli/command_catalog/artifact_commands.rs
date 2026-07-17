use super::*;

#[derive(Subcommand)]
pub(crate) enum ArtifactCommand {
    /// Validate an artifact against schema and invariants
    Validate {
        #[command(flatten)]
        common: CommonArgs,
        /// Artifact file path
        #[arg(long, value_name = "FILE")]
        file: PathBuf,
        /// Artifact kind override (obs, track, acq, eph, pvt, rtk, ppp)
        #[arg(long)]
        kind: Option<String>,
        /// Require non-empty artifacts
        #[arg(long)]
        strict: bool,
        /// Write diagnostics summary report (JSON)
        #[arg(long, value_name = "FILE")]
        report: Option<PathBuf>,
        /// Fail on diagnostics at or above this severity
        #[arg(long, value_enum, default_value = "error")]
        fail_on: DiagnosticFailOn,
    },

    /// Convert an artifact to a target version
    Convert {
        #[command(flatten)]
        common: CommonArgs,
        /// Input artifact
        #[arg(long, value_name = "FILE")]
        input: PathBuf,
        /// Output artifact
        #[arg(long, value_name = "FILE")]
        output: PathBuf,
        /// Target version (e.g. v1)
        #[arg(long)]
        to: String,
    },

    /// Explain an artifact (header + quick stats)
    Explain {
        #[command(flatten)]
        common: CommonArgs,

        /// Artifact file path
        #[arg(long, value_name = "FILE")]
        file: PathBuf,
    },
}

#[derive(ValueEnum, Clone, Copy, Debug)]
pub(crate) enum DiagnosticFailOn {
    None,
    Warn,
    Error,
}
