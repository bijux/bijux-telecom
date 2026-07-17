use super::*;

#[derive(Args)]
pub(crate) struct ValidateConfigArgs {
    #[command(flatten)]
    pub(crate) common: CommonArgs,
}

#[derive(Args)]
pub(crate) struct ConfigSchemaArgs {
    #[command(flatten)]
    pub(crate) common: CommonArgs,

    #[arg(long, value_name = "FILE")]
    pub(crate) out: PathBuf,
}

#[derive(Args)]
pub(crate) struct ConfigUpgradeArgs {
    #[command(flatten)]
    pub(crate) common: CommonArgs,

    /// Input config file
    #[arg(long, value_name = "FILE")]
    pub(crate) config: PathBuf,

    /// Optional output path (defaults to overwriting input)
    #[arg(long, value_name = "FILE")]
    pub(crate) out: Option<PathBuf>,
}

#[derive(Subcommand)]
pub(crate) enum ConfigCommand {
    /// Validate a configuration file (prints warnings unless --strict)
    Validate {
        #[command(flatten)]
        common: CommonArgs,

        #[arg(long, value_name = "FILE")]
        file: PathBuf,

        /// Treat warnings as errors
        #[arg(long)]
        strict: bool,
    },

    /// Print default receiver profile configuration
    PrintDefaults {
        #[command(flatten)]
        common: CommonArgs,

        /// Output path for defaults (TOML). If omitted, prints to stdout.
        #[arg(long, value_name = "FILE")]
        out: Option<PathBuf>,
    },
}
