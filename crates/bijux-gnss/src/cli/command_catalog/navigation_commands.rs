use super::*;

#[derive(ValueEnum, Clone, Copy, Debug)]
pub(crate) enum ReferenceAlign {
    Nearest,
    Linear,
}

#[derive(Subcommand)]
pub(crate) enum NavCommand {
    /// Decode GPS LNAV from tracking dump
    Decode {
        #[command(flatten)]
        common: CommonArgs,

        /// Tracking JSONL dump from `gnss track`
        #[arg(long, value_name = "FILE")]
        track: PathBuf,

        /// PRN to decode
        #[arg(long)]
        prn: u8,

        /// Full GPS week used to resolve the LNAV 10-bit week field
        #[arg(long)]
        reference_week: Option<u32>,
    },
}
