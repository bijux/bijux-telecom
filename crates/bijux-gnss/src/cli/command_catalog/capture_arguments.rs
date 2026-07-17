use super::*;

#[derive(Args)]
pub(crate) struct RawCaptureInputArgs {
    #[arg(long, alias = "path", value_name = "FILE")]
    pub(crate) file: Option<PathBuf>,
}

#[derive(Args)]
pub(crate) struct SamplingRateOverrideArgs {
    #[arg(long)]
    pub(crate) sampling_hz: Option<f64>,
}

#[derive(Args)]
pub(crate) struct IntermediateFrequencyArgs {
    #[arg(long)]
    pub(crate) if_hz: Option<f64>,
}

#[derive(Args)]
pub(crate) struct CodeReplicaArgs {
    #[arg(long)]
    pub(crate) code_hz: Option<f64>,

    #[arg(long)]
    pub(crate) code_length: Option<usize>,
}

#[derive(Args)]
pub(crate) struct AcquisitionSearchArgs {
    /// Override the receiver profile's maximum Doppler search range, in Hz.
    #[arg(long, alias = "doppler")]
    pub(crate) doppler_search_hz: Option<i32>,

    /// Override the receiver profile's Doppler bin width, in Hz.
    #[arg(long)]
    pub(crate) doppler_step_hz: Option<i32>,
}

#[derive(Args)]
pub(crate) struct CaptureWindowArgs {
    #[arg(long, default_value_t = 0)]
    pub(crate) offset_bytes: u64,
}

#[derive(Args)]
pub(crate) struct DefaultPrnSelectionArgs {
    /// Comma-separated PRN list, e.g. "1,3,8"
    #[arg(
        long,
        value_delimiter = ',',
        default_value = "1,2,3,4,5",
        value_parser = clap::value_parser!(u8).range(1..=32)
    )]
    pub(crate) prn: Vec<u8>,
}

#[derive(Args)]
pub(crate) struct RequiredPrnSelectionArgs {
    /// Comma-separated PRN list, e.g. "11,12,25,31,32"
    #[arg(long, value_delimiter = ',', value_parser = clap::value_parser!(u8).range(1..=32))]
    pub(crate) prn: Vec<u8>,
}
