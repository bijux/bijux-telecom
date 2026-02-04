#[derive(Subcommand)]
pub(crate) enum GnssCommand {
    /// Generate GPS L1 C/A code for a PRN
    CaCode {
        #[arg(long)]
        prn: u8,

        /// Number of chips to print
        #[arg(long, default_value_t = 16)]
        count: usize,
    },

    /// Acquire satellites from a raw i16 sample file
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

        #[arg(long, alias = "doppler", default_value_t = 10_000)]
        doppler_search_hz: i32,

        #[arg(long, default_value_t = 500)]
        doppler_step_hz: i32,

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

    /// Track satellites from a raw i16 sample file
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

        #[arg(long, alias = "doppler", default_value_t = 10_000)]
        doppler_search_hz: i32,

        #[arg(long, default_value_t = 500)]
        doppler_step_hz: i32,

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

        /// Ephemeris JSON (from nav decode)
        #[arg(long, value_name = "FILE")]
        eph: PathBuf,

        /// Use EKF-based solver (scaffold)
        #[arg(long)]
        ekf: bool,
    },

    /// Inspect dataset statistics
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

        /// Ephemeris JSON
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

    /// Artifact validation and conversion
    Artifact {
        #[command(subcommand)]
        command: ArtifactCommand,
    },

    /// Validate a receiver profile configuration file
    ValidateConfig {
        #[command(flatten)]
        common: CommonArgs,

        #[arg(long, value_name = "FILE")]
        config: Option<PathBuf>,
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

    /// Diagnostics utilities
    Diagnostics {
        #[command(subcommand)]
        command: DiagnosticsCommand,
    },

    /// Validate sidecar file against schema
    ValidateSidecar {
        #[command(flatten)]
        common: CommonArgs,

        #[arg(long, value_name = "FILE")]
        sidecar: PathBuf,
    },

    /// Write JSON schema for receiver config
    ConfigSchema {
        #[command(flatten)]
        common: CommonArgs,

        #[arg(long, value_name = "FILE")]
        out: PathBuf,
    },

    /// Upgrade a receiver config to the current schema version
    ConfigUpgrade {
        #[command(flatten)]
        common: CommonArgs,

        /// Input config file
        #[arg(long, value_name = "FILE")]
        config: PathBuf,

        /// Optional output path (defaults to overwriting input)
        #[arg(long, value_name = "FILE")]
        out: Option<PathBuf>,
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

    /// Print build and runtime diagnostics
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
}

#[derive(ValueEnum, Clone, Copy, Debug)]
pub(crate) enum ReferenceAlign {
    Nearest,
    Linear,
}

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

    /// Convert an artifact to a target version (scaffold)
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
    },
}

#[derive(Subcommand)]
pub(crate) enum DiagnosticsCommand {
    /// Summarize diagnostics from a run directory
    Summarize {
        #[command(flatten)]
        common: CommonArgs,

        /// Run directory (expects artifacts/ inside)
        #[arg(long, value_name = "DIR")]
        run_dir: PathBuf,

        /// Max number of codes to print
        #[arg(long, default_value_t = 10)]
        top: usize,
    },
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
