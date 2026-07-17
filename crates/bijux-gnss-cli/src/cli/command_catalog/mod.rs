mod diagnostics_commands;
mod artifact_commands;
mod navigation_commands;
mod configuration_commands;

pub(crate) use artifact_commands::{ArtifactCommand, DiagnosticFailOn};
pub(crate) use configuration_commands::{
    ConfigCommand, ConfigSchemaArgs, ConfigUpgradeArgs, ValidateConfigArgs,
};
pub(crate) use diagnostics_commands::{AdvancedGateMode, DiagnosticsCommand, RouteTopic, WorkflowProfile};
pub(crate) use navigation_commands::{NavCommand, ReferenceAlign};

#[derive(Subcommand)]
pub(crate) enum GnssCommand {
    /// Generate GPS L1 C/A code for a PRN
    CaCode {
        #[arg(long)]
        prn: u8,

        /// Zero-based chip offset within the repeating C/A sequence
        #[arg(long, default_value_t = 0)]
        start_chip: usize,

        /// Number of chips to print
        #[arg(long, default_value_t = 16)]
        count: usize,

        /// Print published PRN assignment metadata before the chip sequence
        #[arg(long)]
        with_reference: bool,

        /// Print periodic autocorrelation summary before the chip sequence
        #[arg(long)]
        with_autocorrelation: bool,

        /// Print periodic cross-correlation summary against another PRN before the chip sequence
        #[arg(long, value_name = "PRN", value_parser = clap::value_parser!(u8).range(1..=32))]
        cross_correlation_prn: Option<u8>,
    },

    /// Acquire satellites from a raw IQ file with explicit metadata
    Acquire {
        #[command(flatten)]
        common: CommonArgs,

        #[arg(long, alias = "path", value_name = "FILE")]
        file: Option<PathBuf>,

        #[arg(long)]
        sampling_hz: Option<f64>,

        #[arg(long)]
        if_hz: Option<f64>,

        #[arg(long)]
        code_hz: Option<f64>,

        #[arg(long)]
        code_length: Option<usize>,

        /// Override the receiver profile's maximum Doppler search range, in Hz.
        #[arg(long, alias = "doppler")]
        doppler_search_hz: Option<i32>,

        /// Override the receiver profile's Doppler bin width, in Hz.
        #[arg(long)]
        doppler_step_hz: Option<i32>,

        #[arg(long, default_value_t = 0)]
        offset_bytes: u64,

        /// Number of top candidates per PRN
        #[arg(long, default_value_t = 3)]
        top: usize,

        /// Comma-separated PRN list, e.g. "1,3,8"
        #[arg(
            long,
            value_delimiter = ',',
            default_value = "1,2,3,4,5",
            value_parser = clap::value_parser!(u8).range(1..=32)
        )]
        prn: Vec<u8>,
    },

    /// Track satellites from a raw IQ file with explicit metadata
    Track {
        #[command(flatten)]
        common: CommonArgs,

        #[arg(long, alias = "path", value_name = "FILE")]
        file: Option<PathBuf>,

        #[arg(long)]
        sampling_hz: Option<f64>,

        #[arg(long)]
        if_hz: Option<f64>,

        #[arg(long)]
        code_hz: Option<f64>,

        #[arg(long)]
        code_length: Option<usize>,

        /// Override the receiver profile's maximum Doppler search range, in Hz.
        #[arg(long, alias = "doppler")]
        doppler_search_hz: Option<i32>,

        /// Override the receiver profile's Doppler bin width, in Hz.
        #[arg(long)]
        doppler_step_hz: Option<i32>,

        #[arg(long, default_value_t = 0)]
        offset_bytes: u64,

        #[arg(
            long,
            value_delimiter = ',',
            default_value = "1,2,3,4,5",
            value_parser = clap::value_parser!(u8).range(1..=32)
        )]
        prn: Vec<u8>,
    },

    /// Navigation-related commands
    Nav {
        #[command(subcommand)]
        command: NavCommand,
    },

    /// Solve PVT from a dataset
    Pvt {
        #[command(flatten)]
        common: CommonArgs,
        /// Observation JSONL (ObsEpoch)
        #[arg(long, value_name = "FILE")]
        obs: PathBuf,

        /// Broadcast navigation JSON, broadcast ephemeris JSON, RINEX NAV, or nav-decode report
        #[arg(long, value_name = "FILE")]
        eph: PathBuf,

        /// Use the receiver-owned navigation filter solver
        #[arg(long)]
        ekf: bool,
    },

    /// Inspect raw IQ dataset statistics from explicit metadata
    Inspect {
        #[command(flatten)]
        common: CommonArgs,

        #[arg(long, alias = "path", value_name = "FILE")]
        file: Option<PathBuf>,

        #[arg(long)]
        sampling_hz: Option<f64>,

        #[arg(long, default_value_t = 0)]
        max_samples: usize,
    },

    /// RTK alignment and SD/DD artifacts
    Rtk {
        #[command(flatten)]
        common: CommonArgs,

        /// Base observation JSONL
        #[arg(long, value_name = "FILE")]
        base_obs: PathBuf,

        /// Rover observation JSONL
        #[arg(long, value_name = "FILE")]
        rover_obs: PathBuf,

        /// Broadcast ephemeris JSON, RINEX NAV, or nav-decode report
        #[arg(long, value_name = "FILE")]
        eph: PathBuf,

        /// Base ECEF position "x,y,z"
        #[arg(long, value_name = "ECEF")]
        base_ecef: String,

        /// Alignment tolerance in seconds
        #[arg(long, default_value_t = 0.0005)]
        tolerance_s: f64,

        /// Reference satellite policy
        #[arg(long, value_enum, default_value_t = RefPolicy::Global)]
        ref_policy: RefPolicy,
    },

    /// Run parameter sweeps over synthetic scenarios
    Experiment {
        #[command(flatten)]
        common: CommonArgs,

        #[arg(long, value_name = "FILE")]
        scenario: PathBuf,

        /// Sweep parameters like "tracking.dll_bw_hz=1.0,2.0"
        #[arg(long, value_name = "PARAM=VALS")]
        sweep: Vec<String>,
    },

    /// Export a deterministic synthetic raw IQ bundle and matching truth artifact
    ExportSyntheticIq {
        /// Synthetic scenario TOML file
        #[arg(long, value_name = "FILE")]
        scenario: PathBuf,

        /// Output directory for the generated run bundle
        #[arg(long, alias = "output", value_name = "DIR")]
        out: Option<PathBuf>,

        /// Report output format
        #[arg(long, value_enum, default_value_t = ReportFormat::Table)]
        report: ReportFormat,

        /// Synthetic capture start timestamp written into the sidecar
        #[arg(long, default_value = "2026-07-09T00:00:00Z")]
        capture_start_utc: String,
    },

    /// Validate a synthetic IQ bundle against truth-guided C/N0 expectations
    ValidateSyntheticIq {
        #[command(flatten)]
        common: CommonArgs,

        #[arg(long, alias = "path", value_name = "FILE")]
        file: PathBuf,

        /// Synthetic truth JSON emitted alongside the IQ capture
        #[arg(long, value_name = "FILE")]
        truth: PathBuf,

        /// Maximum allowed absolute C/N0 error in dB-Hz
        #[arg(long, default_value_t = 5.0)]
        tolerance_db_hz: f64,

        /// Maximum allowed wrapped acquisition code-phase error in samples
        #[arg(long, default_value_t = 2)]
        acquisition_code_phase_tolerance_samples: usize,

        /// Maximum allowed acquisition Doppler error in Doppler bins
        #[arg(long, default_value_t = 1)]
        acquisition_doppler_tolerance_bins: usize,
    },

    /// Validate a full synthetic navigation scenario from acquisition through PVT
    ValidateSyntheticNavigation {
        #[command(flatten)]
        common: CommonArgs,

        /// Truth-complete synthetic navigation scenario TOML file
        #[arg(long, value_name = "FILE")]
        scenario: PathBuf,
    },

    /// Measure synthetic quantization loss against a float32 reference capture
    MeasureSyntheticQuantization {
        #[command(flatten)]
        common: CommonArgs,

        /// Synthetic scenario TOML file
        #[arg(long, value_name = "FILE")]
        scenario: PathBuf,

        /// Quantization profiles to measure; defaults to the canonical float32-to-1-bit sweep
        #[arg(long, value_enum, value_delimiter = ',')]
        quantization: Vec<SyntheticQuantizationArg>,

        /// Synthetic capture start timestamp used in generated metadata
        #[arg(long, default_value = "2026-07-14T00:00:00Z")]
        capture_start_utc: String,
    },

    /// Artifact validation and conversion
    Artifact {
        #[command(subcommand)]
        command: ArtifactCommand,
    },

    /// Validate a receiver profile configuration file
    ValidateConfig {
        #[command(flatten)]
        args: ValidateConfigArgs,
    },

    /// Configuration utilities
    Config {
        #[command(subcommand)]
        command: ConfigCommand,
    },

    /// Validate observation or ephemeris artifacts against schemas
    ValidateArtifacts {
        #[command(flatten)]
        common: CommonArgs,

        /// ObsEpoch JSONL file
        #[arg(long, value_name = "FILE")]
        obs: Option<PathBuf>,

        /// Ephemeris JSON file
        #[arg(long, value_name = "FILE")]
        eph: Option<PathBuf>,

        /// Require non-empty files
        #[arg(long)]
        strict: bool,
    },

    /// Diagnostics and audit workflows for receiver evidence
    Diagnostics {
        #[command(subcommand)]
        command: DiagnosticsCommand,
    },

    /// Validate sidecar file against schema
    ValidateSidecar {
        #[command(flatten)]
        common: CommonArgs,

        #[arg(long = "sidecar-file", value_name = "FILE")]
        sidecar_file: PathBuf,
    },

    /// Analyze a GNSS run directory and emit evidence-oriented summaries
    Analyze {
        /// Run directory produced by bijux-gnss
        #[arg(long, value_name = "DIR")]
        run_dir: PathBuf,

        /// Optional reference JSONL for position error plots
        #[arg(long, value_name = "FILE")]
        reference: Option<PathBuf>,
    },

    /// Compare two GNSS run directories for quality deltas
    Diff {
        /// First run directory
        #[arg(long, value_name = "DIR")]
        run_a: PathBuf,

        /// Second run directory
        #[arg(long, value_name = "DIR")]
        run_b: PathBuf,
    },

    /// Write JSON schema for receiver config
    ConfigSchema {
        #[command(flatten)]
        args: ConfigSchemaArgs,
    },

    /// Upgrade a receiver config to the current schema version
    ConfigUpgrade {
        #[command(flatten)]
        args: ConfigUpgradeArgs,
    },

    /// Run a streaming pipeline with optional replay rate
    Run {
        #[command(flatten)]
        common: CommonArgs,

        #[arg(long, alias = "path", value_name = "FILE")]
        file: Option<PathBuf>,

        /// Replay mode; if set, uses --rate to pace output
        #[arg(long)]
        replay: bool,

        /// Replay rate multiplier (1.0 = real-time, 0 = as fast as possible)
        #[arg(long, default_value_t = 1.0)]
        rate: f64,
    },

    /// Write RINEX-like observation and navigation files
    Rinex {
        #[command(flatten)]
        common: CommonArgs,

        /// ObsEpoch JSONL
        #[arg(long, value_name = "FILE")]
        obs: PathBuf,

        /// Ephemeris JSON
        #[arg(long, value_name = "FILE")]
        eph: PathBuf,

        /// Enforce strict formatting checks
        #[arg(long)]
        strict: bool,
    },

    /// Print receiver runtime readiness diagnostics
    Doctor {
        #[command(flatten)]
        common: CommonArgs,

        #[arg(long, alias = "path", value_name = "FILE")]
        file: Option<PathBuf>,
    },

    /// Run a full validation pipeline and emit validation_report.json
    Validate {
        #[command(flatten)]
        common: CommonArgs,
        #[arg(long, alias = "path", value_name = "FILE")]
        file: Option<PathBuf>,
        /// Ephemeris JSON file (required for PVT)
        #[arg(long, value_name = "FILE")]
        eph: PathBuf,
        /// Reference solution JSONL for comparison
        #[arg(long, value_name = "FILE")]
        reference: PathBuf,
        /// PRN list for acquisition/tracking
        #[arg(
            long,
            value_delimiter = ',',
            default_value = "1,2,3,4,5",
            value_parser = clap::value_parser!(u8).range(1..=32)
        )]
        prn: Vec<u8>,
        /// Precise ephemeris SP3 file (optional)
        #[arg(long, value_name = "FILE")]
        sp3: Option<PathBuf>,
        /// Precise clock CLK file (optional)
        #[arg(long, value_name = "FILE")]
        clk: Option<PathBuf>,
        /// Bias-SINEX code bias file for dual-frequency code corrections (optional)
        #[arg(long = "bias-sinex", alias = "dcb", value_name = "FILE")]
        bias_sinex: Option<PathBuf>,
    },

    /// Validate a run directory against a reference trajectory
    ValidateReference {
        #[command(flatten)]
        common: CommonArgs,

        /// Run directory containing artifacts/
        #[arg(long, value_name = "DIR")]
        run_dir: PathBuf,

        /// Reference trajectory file (JSONL or CSV)
        #[arg(long, value_name = "FILE")]
        reference: PathBuf,

        /// Alignment policy for reference (nearest or linear)
        #[arg(long, value_enum, default_value_t = ReferenceAlign::Nearest)]
        align: ReferenceAlign,
    },

    /// Validate a raw capture end to end from acquisition through navigation attempts
    ValidateCapture {
        #[command(flatten)]
        common: CommonArgs,

        #[arg(long, alias = "path", value_name = "FILE")]
        file: Option<PathBuf>,

        #[arg(long)]
        sampling_hz: Option<f64>,

        #[arg(long)]
        if_hz: Option<f64>,

        #[arg(long)]
        code_hz: Option<f64>,

        #[arg(long)]
        code_length: Option<usize>,

        /// Override the receiver profile's maximum Doppler search range, in Hz.
        #[arg(long, alias = "doppler")]
        doppler_search_hz: Option<i32>,

        /// Override the receiver profile's Doppler bin width, in Hz.
        #[arg(long)]
        doppler_step_hz: Option<i32>,

        /// Broadcast navigation JSON, broadcast ephemeris JSON, or RINEX NAV file
        #[arg(long, value_name = "FILE")]
        eph: PathBuf,

        /// Comma-separated PRN list, e.g. "11,12,25,31,32"
        #[arg(long, value_delimiter = ',', value_parser = clap::value_parser!(u8).range(1..=32))]
        prn: Vec<u8>,
    },
}
