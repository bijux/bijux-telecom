use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command as ProcessCommand;
use std::time::{SystemTime, UNIX_EPOCH};

use bijux_gnss_core::{validate_obs_epochs, Constellation, ObsEpoch, SamplesFrame, SatId};
use bijux_gnss_nav::{
    elevation_azimuth_deg, sat_state_gps_l1ca, CodeBiasProvider, GpsEphemeris, Matrix,
    NavClockModel, PhaseBiasProvider, PppConfig, PppFilter, PppProcessNoise, ProcessNoiseConfig,
    ProductsProvider, PseudorangeMeasurement, WeightingConfig,
};
use bijux_gnss_receiver::{
    acquisition::Acquisition, ca_code::generate_ca_code, ca_code::Prn, data::FileSamples,
    data::SampleSource, signal::samples_per_code, ReceiverConfig, ReceiverProfile,
};
use clap::{Args, Parser, Subcommand, ValueEnum};
use eyre::{bail, eyre, Context, ContextCompat, Result};
use jsonschema::JSONSchema;
use schemars::schema_for;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

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

#[derive(Subcommand)]
enum GnssCommand {
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

    /// Validate a receiver profile configuration file
    ValidateConfig {
        #[command(flatten)]
        common: CommonArgs,

        #[arg(long, value_name = "FILE")]
        config: Option<PathBuf>,
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
}

#[derive(Subcommand)]
enum NavCommand {
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

#[derive(Args, Clone)]
struct CommonArgs {
    /// Receiver profile config (TOML)
    #[arg(long)]
    config: Option<PathBuf>,

    /// Dataset ID from datasets/registry.yaml
    #[arg(long)]
    dataset: Option<String>,

    /// Output directory for artifacts (run.json, reports)
    #[arg(long)]
    out: Option<PathBuf>,

    /// Report output format
    #[arg(long, value_enum, default_value_t = ReportFormat::Table)]
    report: ReportFormat,

    /// Override deterministic seed
    #[arg(long)]
    seed: Option<u64>,

    /// Dump trace artifacts (requires receiver feature `trace-dump`)
    #[arg(long)]
    dump: Option<PathBuf>,

    /// Sidecar metadata file for raw IQ
    #[arg(long)]
    sidecar: Option<PathBuf>,
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

#[derive(Debug, Deserialize, Clone)]
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
    config_hash: String,
    dataset_id: Option<String>,
    build_profile: String,
    cpu_features: Vec<String>,
    results: serde_json::Value,
}

#[derive(Debug, Serialize)]
struct AcquisitionReport {
    sats: Vec<SatId>,
    results: Vec<AcquisitionRow>,
}

#[derive(Debug, Serialize)]
struct AcquisitionRow {
    sat: SatId,
    carrier_hz: f64,
    code_phase_samples: usize,
    peak: f32,
    peak_mean_ratio: f32,
    peak_second_ratio: f32,
}

#[derive(Debug, Serialize)]
struct InspectReport {
    sample_rate_hz: f64,
    total_samples: usize,
    dc_offset_i: f64,
    dc_offset_q: f64,
    clip_rate: f64,
    noise_floor_db: f64,
    power_histogram: Vec<u64>,
}

#[derive(Debug, Serialize, Clone)]
struct ExperimentRunResult {
    run_index: usize,
    config_hash: String,
    scenario_id: String,
    overrides: Vec<(String, String)>,
    lock_pct: f64,
    pvt_rms_m: f64,
    residual_rms_m: f64,
    rejected_count: usize,
    ms_per_epoch: f64,
}

#[derive(Debug, Serialize)]
struct ExperimentSummary {
    runs: Vec<ExperimentRunResult>,
}

struct EkfContext {
    ekf: bijux_gnss_nav::Ekf,
    model: NavClockModel,
    last_t_rx_s: Option<f64>,
    ambiguity: bijux_gnss_nav::AmbiguityManager,
    isb: bijux_gnss_nav::InterSystemBiasManager,
    ztd_index: Option<usize>,
    atmosphere: bijux_gnss_nav::AtmosphereConfig,
    code_bias: bijux_gnss_nav::ZeroBiases,
    phase_bias: bijux_gnss_nav::ZeroBiases,
    corrections: bijux_gnss_nav::CorrectionContext,
}

impl EkfContext {
    fn new() -> Self {
        let x = vec![0.0_f64; 8];
        let p = Matrix::identity(8);
        let mut ekf = bijux_gnss_nav::Ekf::new(
            x,
            p,
            bijux_gnss_nav::EkfConfig {
                gating_chi2_code: Some(200.0),
                gating_chi2_phase: Some(200.0),
                gating_chi2_doppler: Some(200.0),
                huber_k: Some(10.0),
                square_root: true,
                covariance_epsilon: 1e-6,
                divergence_max_variance: 1e12,
            },
        );
        let ztd_index = {
            let idx = ekf.x.len();
            ekf.add_state("ztd_m", 2.3, 10.0);
            Some(idx)
        };
        Self {
            ekf,
            model: NavClockModel::new(ProcessNoiseConfig {
                pos_m: 1.0,
                vel_mps: 1.0,
                clock_bias_s: 1e-4,
                clock_drift_s: 1e-5,
                ztd_m: 0.01,
            }),
            last_t_rx_s: None,
            ambiguity: bijux_gnss_nav::AmbiguityManager::new(),
            isb: bijux_gnss_nav::InterSystemBiasManager::new(),
            ztd_index,
            atmosphere: bijux_gnss_nav::AtmosphereConfig::default(),
            code_bias: bijux_gnss_nav::ZeroBiases,
            phase_bias: bijux_gnss_nav::ZeroBiases,
            corrections: bijux_gnss_nav::CorrectionContext,
        }
    }
}

#[derive(Debug, Serialize, Deserialize)]
struct TrackingReport {
    sats: Vec<SatId>,
    epochs: Vec<TrackingRow>,
}

#[derive(Debug, Serialize, Deserialize, Clone)]
struct TrackingRow {
    epoch_idx: u64,
    sample_index: u64,
    sat: SatId,
    carrier_hz: f64,
    code_rate_hz: f64,
    code_phase_samples: f64,
    prompt_i: f32,
    prompt_q: f32,
    lock: bool,
    cn0_dbhz: f64,
    pll_lock: bool,
    dll_lock: bool,
    fll_lock: bool,
    cycle_slip: bool,
    nav_bit_lock: bool,
    dll_err: f32,
    pll_err: f32,
    fll_err: f32,
}

#[derive(Debug, Serialize)]
struct NavDecodeReport {
    sat: SatId,
    preamble_hits: usize,
    parity_pass_rate: f64,
    ephemerides: Vec<bijux_gnss_nav::GpsEphemeris>,
}

#[derive(Debug, Serialize, Deserialize)]
struct ValidationReferenceEpoch {
    epoch_idx: u64,
    latitude_deg: f64,
    longitude_deg: f64,
    altitude_m: f64,
}

#[derive(Debug, Serialize)]
struct ValidationErrorStats {
    count: usize,
    mean: f64,
    median: f64,
    rms: f64,
    p95: f64,
}

#[derive(Debug, Serialize)]
struct ValidationReport {
    samples: usize,
    epochs: usize,
    horiz_error_m: ValidationErrorStats,
    vert_error_m: ValidationErrorStats,
    residuals: Vec<NavResidualReport>,
    time_consistency: TimeConsistencyReport,
    consistency: SolutionConsistencyReport,
    budgets: ValidationBudgets,
    budget_violations: Vec<String>,
    nis_mean: Option<f64>,
    nees_mean: Option<f64>,
    inter_frequency_alignment: bijux_gnss_core::InterFrequencyAlignmentReport,
    ppp_readiness: PppReadinessReport,
}

#[derive(Debug, Serialize)]
struct PppReadinessReport {
    multi_freq_present: bool,
    combinations_valid: bool,
    products_ok: bool,
    product_fallbacks: Vec<String>,
}

#[derive(Debug, Serialize)]
struct PppEvaluationReport {
    epochs: usize,
    horiz_rms_m: Option<f64>,
    vert_rms_m: Option<f64>,
    time_to_first_meter_s: Option<f64>,
    time_to_decimeter_s: Option<f64>,
    time_to_centimeter_s: Option<f64>,
    residual_rms_m: f64,
}

#[derive(Debug, Serialize)]
struct NavResidualReport {
    epoch_idx: u64,
    rms_m: f64,
    pdop: f64,
    residuals: Vec<(SatId, f64)>,
    rejected: Vec<SatId>,
}

#[derive(Debug, Serialize)]
struct SolutionConsistencyReport {
    position_jump_count: usize,
    clock_jump_count: usize,
    pdop_spike_count: usize,
    warnings: Vec<String>,
}

#[derive(Debug, Clone, Serialize)]
struct ValidationBudgets {
    acq_doppler_hz: f64,
    acq_code_phase_samples: f64,
    tracking_carrier_jitter_hz: f64,
    ephemeris_parity_rate_min: f64,
    pvt_max_iterations: usize,
}

#[derive(Debug, Serialize)]
struct TimeConsistencyReport {
    channels: usize,
    epochs_checked: usize,
    epoch_backward: usize,
    epoch_gaps: usize,
    sample_backward: usize,
    sample_step_mismatch: usize,
    expected_step: Option<u64>,
    observed_step_mean: Option<f64>,
    warnings: Vec<String>,
}

fn main() -> Result<()> {
    init_tracing();
    let cli = Cli::parse();

    match cli.command {
        AppCommand::Gnss { command } => match command {
            GnssCommand::CaCode { prn, count } => {
                let code = generate_ca_code(Prn(prn));
                let count = count.min(code.len());
                for chip in code.iter().take(count) {
                    print!("{chip} ");
                }
                println!();
            }
            GnssCommand::Acquire {
                common,
                file,
                sampling_hz,
                if_hz,
                code_hz,
                code_length,
                doppler_search_hz,
                doppler_step_hz,
                offset_bytes,
                top,
                prn,
            } => {
                set_trace_dir(&common);
                let dataset = load_dataset(&common)?;
                let mut profile = load_profile(&common)?;
                apply_common_overrides(&mut profile, &common);
                apply_overrides(&mut profile, sampling_hz, if_hz, code_hz, code_length);
                if let Some(entry) = &dataset {
                    profile.sample_rate_hz = entry.sample_rate_hz;
                    profile.intermediate_freq_hz = entry.intermediate_freq_hz;
                }
                profile
                    .validate()
                    .map_err(|errs| eyre!("invalid config: {}", errs.join(", ")))?;
                let config = profile.to_receiver_config();

                let input_file = resolve_input_file(file.as_ref(), dataset.as_ref())?;

                let sidecar = load_sidecar(common.sidecar.as_ref())?;
                let frame = load_frame(&input_file, &config, offset_bytes, sidecar.as_ref())?;

                let acquisition =
                    Acquisition::new(config).with_doppler(doppler_search_hz, doppler_step_hz);
                let sats = prns_to_sats(&prn);
                let results = acquisition.run_fft_topn(
                    &frame,
                    &sats,
                    top,
                    profile.acquisition.integration_ms,
                    1,
                );

                let mut rows = Vec::new();
                for candidates in &results {
                    for r in candidates {
                        rows.push(AcquisitionRow {
                            sat: r.sat,
                            carrier_hz: r.carrier_hz,
                            code_phase_samples: r.code_phase_samples,
                            peak: r.peak,
                            peak_mean_ratio: r.peak_mean_ratio,
                            peak_second_ratio: r.peak_second_ratio,
                        });
                    }
                }

                let report = AcquisitionReport {
                    sats,
                    results: rows,
                };
                match common.report {
                    ReportFormat::Table => print_acquisition_table(&report),
                    ReportFormat::Json => emit_report(&common, "acquire", &report)?,
                }
                write_manifest(&common, "acquire", &profile, dataset.as_ref(), &report)?;
            }
            GnssCommand::Track {
                common,
                file,
                sampling_hz,
                if_hz,
                code_hz,
                code_length,
                doppler_search_hz,
                doppler_step_hz,
                offset_bytes,
                prn,
            } => {
                set_trace_dir(&common);
                let dataset = load_dataset(&common)?;
                let mut profile = load_profile(&common)?;
                apply_common_overrides(&mut profile, &common);
                apply_overrides(&mut profile, sampling_hz, if_hz, code_hz, code_length);
                if let Some(entry) = &dataset {
                    profile.sample_rate_hz = entry.sample_rate_hz;
                    profile.intermediate_freq_hz = entry.intermediate_freq_hz;
                }
                profile
                    .validate()
                    .map_err(|errs| eyre!("invalid config: {}", errs.join(", ")))?;
                let config = profile.to_receiver_config();

                let input_file = resolve_input_file(file.as_ref(), dataset.as_ref())?;
                let sidecar = load_sidecar(common.sidecar.as_ref())?;
                let frame = load_frame(&input_file, &config, offset_bytes, sidecar.as_ref())?;

                let acquisition = Acquisition::new(config.clone())
                    .with_doppler(doppler_search_hz, doppler_step_hz);
                let sats = prns_to_sats(&prn);
                let acquisitions = acquisition.run_fft(&frame, &sats);

                let tracking = bijux_gnss_receiver::tracking::Tracking::new(config.clone());
                let tracks = tracking.track_from_acquisition(
                    &frame,
                    &acquisitions,
                    bijux_gnss_core::SignalBand::L1,
                );

                let report = TrackingReport {
                    sats: sats.clone(),
                    epochs: tracks
                        .iter()
                        .flat_map(|t| {
                            t.epochs.iter().map(move |e| TrackingRow {
                                epoch_idx: e.epoch.index,
                                sample_index: e.sample_index,
                                sat: t.sat,
                                carrier_hz: t.carrier_hz,
                                code_rate_hz: e.code_rate_hz,
                                code_phase_samples: t.code_phase_samples,
                                prompt_i: e.prompt_i,
                                prompt_q: e.prompt_q,
                                lock: e.lock,
                                cn0_dbhz: e.cn0_dbhz,
                                pll_lock: e.pll_lock,
                                dll_lock: e.dll_lock,
                                fll_lock: e.fll_lock,
                                cycle_slip: e.cycle_slip,
                                nav_bit_lock: e.nav_bit_lock,
                                dll_err: e.dll_err,
                                pll_err: e.pll_err,
                                fll_err: e.fll_err,
                            })
                        })
                        .collect(),
                };

                emit_report(&common, "track", &report)?;
                write_track_timeseries(&common, &report)?;
                write_obs_timeseries(&common, &config, &tracks, profile.navigation.hatch_window)?;
                write_manifest(&common, "track", &profile, dataset.as_ref(), &report)?;
            }
            GnssCommand::Nav { command } => match command {
                NavCommand::Decode { common, track, prn } => {
                    set_trace_dir(&common);
                    let rows = read_tracking_dump(&track)?;
                    let mut prompt = Vec::new();
                    let target = SatId {
                        constellation: Constellation::Gps,
                        prn,
                    };
                    let mut sorted: Vec<_> = rows.into_iter().filter(|r| r.sat == target).collect();
                    sorted.sort_by_key(|r| r.epoch_idx);
                    for row in sorted {
                        prompt.push(row.prompt_i);
                    }
                    let bits = bijux_gnss_nav::bit_sync_from_prompt(&prompt);
                    let (mut ephs, stats) = bijux_gnss_nav::decode_subframes(&bits.bits);
                    for eph in ephs.iter_mut() {
                        eph.sat = target;
                    }
                    let report = NavDecodeReport {
                        sat: target,
                        preamble_hits: stats.preamble_hits,
                        parity_pass_rate: stats.parity_pass_rate,
                        ephemerides: ephs.clone(),
                    };
                    emit_report(&common, "nav_decode", &report)?;
                    write_ephemeris(&common, &ephs)?;
                }
            },
            GnssCommand::Pvt {
                common,
                obs,
                eph,
                ekf,
            } => {
                set_trace_dir(&common);
                let obs_epochs = read_obs_epochs(&obs)?;
                let ephs = read_ephemeris(&eph)?;
                let mut lines = Vec::new();
                let mut nav = if !ekf {
                    Some(bijux_gnss_receiver::navigation::Navigation::new(
                        ReceiverProfile::default().to_receiver_config(),
                    ))
                } else {
                    None
                };
                let mut ekf_ctx = if ekf { Some(EkfContext::new()) } else { None };
                for obs_epoch in obs_epochs {
                    let solution = if ekf {
                        solve_epoch_ekf(&mut ekf_ctx, &obs_epoch, &ephs)?
                    } else {
                        nav.as_mut()
                            .and_then(|nav| nav.solve_epoch(&obs_epoch, &ephs))
                    };
                    if let Some(solution) = solution {
                        let line = serde_json::to_string(&solution)?;
                        lines.push(line);
                    }
                }
                let Some(out_dir) = &common.out else {
                    return Ok(());
                };
                fs::create_dir_all(out_dir)?;
                let path = out_dir.join("pvt.jsonl");
                fs::write(path, lines.join("\n"))?;
            }
            GnssCommand::Inspect {
                common,
                file,
                sampling_hz,
                max_samples,
            } => {
                set_trace_dir(&common);
                let dataset = load_dataset(&common)?;
                let input_file = resolve_input_file(file.as_ref(), dataset.as_ref())?;
                let sample_rate_hz = sampling_hz
                    .or_else(|| dataset.as_ref().map(|d| d.sample_rate_hz))
                    .unwrap_or(5_000_000.0);
                let report = inspect_dataset(&input_file, sample_rate_hz, max_samples)?;
                match common.report {
                    ReportFormat::Table => print_inspect_table(&report),
                    ReportFormat::Json => emit_report(&common, "inspect", &report)?,
                }
                write_manifest(
                    &common,
                    "inspect",
                    &ReceiverProfile::default(),
                    dataset.as_ref(),
                    &report,
                )?;
            }
            GnssCommand::Rtk {
                common,
                base_obs,
                rover_obs,
                eph,
                base_ecef,
                tolerance_s,
                ref_policy,
            } => {
                set_trace_dir(&common);
                let base_epochs = read_obs_epochs(&base_obs)?;
                let rover_epochs = read_obs_epochs(&rover_obs)?;
                let ephs = read_ephemeris(&eph)?;
                let base_xyz = parse_ecef(&base_ecef)?;

                let mut aligner = bijux_gnss_receiver::rtk::EpochAligner::new(tolerance_s);
                let aligned = aligner.align(&base_epochs, &rover_epochs);

                let out_dir = common.out.clone().context("--out is required for rtk")?;
                fs::create_dir_all(&out_dir)?;

                let mut sd_lines = Vec::new();
                let mut dd_lines = Vec::new();
                let mut baseline_lines = Vec::new();
                let mut baseline_quality_lines = Vec::new();
                let mut fix_audit_lines = Vec::new();
                let mut precision_lines = Vec::new();
                let mut fix_state = bijux_gnss_receiver::ambiguity::FixState::default();
                let fixer = bijux_gnss_receiver::ambiguity::NaiveFixer::new(
                    bijux_gnss_receiver::ambiguity::FixPolicy::default(),
                );
                let mut last_ref: Option<bijux_gnss_core::SigId> = None;
                let mut ref_selector = bijux_gnss_receiver::rtk::RefSatSelector::new(5);
                let mut ref_selectors: std::collections::BTreeMap<
                    Constellation,
                    bijux_gnss_receiver::rtk::RefSatSelector,
                > = std::collections::BTreeMap::new();

                for (base, rover) in aligned {
                    let sd = bijux_gnss_receiver::rtk::build_sd(&base, &rover);
                    for item in &sd {
                        sd_lines.push(serde_json::to_string(item)?);
                    }
                    let dd = match ref_policy {
                        RefPolicy::Global => {
                            if let Some(ref_sig) = ref_selector.choose(&sd) {
                                bijux_gnss_receiver::rtk::build_dd(&sd, ref_sig)
                            } else {
                                Vec::new()
                            }
                        }
                        RefPolicy::PerConstellation => {
                            let refs =
                                bijux_gnss_receiver::rtk::choose_ref_sat_per_constellation(&sd);
                            let mut chosen = std::collections::BTreeMap::new();
                            for (constellation, sig) in refs {
                                let selector =
                                    ref_selectors.entry(constellation).or_insert_with(|| {
                                        bijux_gnss_receiver::rtk::RefSatSelector::new(5)
                                    });
                                let subset: Vec<_> = sd
                                    .iter()
                                    .filter(|s| s.sig.sat.constellation == constellation)
                                    .cloned()
                                    .collect();
                                if let Some(picked) = selector.choose(&subset) {
                                    chosen.insert(constellation, picked);
                                } else {
                                    chosen.insert(constellation, sig);
                                }
                            }
                            bijux_gnss_receiver::rtk::build_dd_per_constellation(&sd, &chosen)
                        }
                    };
                    for item in &dd {
                        dd_lines.push(serde_json::to_string(item)?);
                    }
                    let ref_sig = dd.first().map(|d| d.ref_sig);
                    let ref_changed = ref_sig != last_ref;
                    if ref_sig.is_some() {
                        last_ref = ref_sig;
                    }

                    let mut baseline = bijux_gnss_receiver::rtk::solve_baseline_dd(
                        &dd,
                        base_xyz,
                        &ephs,
                        rover.t_rx_s,
                    );

                    let float = bijux_gnss_receiver::ambiguity::FloatAmbiguitySolution {
                        ids: dd
                            .iter()
                            .map(|d| bijux_gnss_core::AmbiguityId {
                                sig: d.sig,
                                signal: format!("{:?}", d.sig.band),
                            })
                            .collect(),
                        float_cycles: dd.iter().map(|d| d.phase_cycles).collect(),
                        covariance: dd.iter().map(|d| vec![d.variance_phase]).collect(),
                    };
                    let (fix_result, audit) =
                        fixer.fix_with_state(rover.epoch_idx, &float, &mut fix_state);
                    fix_audit_lines.push(serde_json::to_string(&audit)?);

                    if let Some(baseline_val) = baseline.take() {
                        let before_rms = bijux_gnss_receiver::rtk::dd_residual_metrics(
                            &dd,
                            base_xyz,
                            baseline_val.enu_m,
                            &ephs,
                            rover.t_rx_s,
                        )
                        .map(|(rms, _pred, _)| rms);
                        let mut adjusted = bijux_gnss_receiver::rtk::apply_fix_hold(
                            baseline_val,
                            fix_result.accepted,
                        );
                        let after_rms = bijux_gnss_receiver::rtk::dd_residual_metrics(
                            &dd,
                            base_xyz,
                            adjusted.enu_m,
                            &ephs,
                            rover.t_rx_s,
                        )
                        .map(|(rms, _pred, _)| rms);
                        if let (Some(before), Some(after)) = (before_rms, after_rms) {
                            if after > before * 1.1 {
                                adjusted.fixed = false;
                            } else {
                                adjusted.fixed = fix_result.accepted;
                            }
                        } else {
                            adjusted.fixed = fix_result.accepted;
                        }
                        let (rms_obs, rms_pred, used_sats) =
                            if let Some((rms_obs, rms_pred, count)) =
                                bijux_gnss_receiver::rtk::dd_residual_metrics(
                                    &dd,
                                    base_xyz,
                                    adjusted.enu_m,
                                    &ephs,
                                    rover.t_rx_s,
                                )
                            {
                                (rms_obs, rms_pred, count)
                            } else {
                                (0.0, 0.0, 0)
                            };
                        let separation = bijux_gnss_receiver::rtk::solution_separation(
                            &dd,
                            base_xyz,
                            &ephs,
                            rover.t_rx_s,
                        );
                        let mut sep_sig = None;
                        let mut sep_max = None;
                        if let Some(seps) = separation {
                            if let Some(max) = seps
                                .iter()
                                .max_by(|a, b| a.delta_enu_m.partial_cmp(&b.delta_enu_m).unwrap())
                            {
                                sep_sig = Some(format!("{:?}", max.sig));
                                sep_max = Some(max.delta_enu_m);
                            }
                        }
                        if let Some(cov) = adjusted.covariance_m2 {
                            let sigma_e = cov[0][0].abs().sqrt();
                            let sigma_n = cov[1][1].abs().sqrt();
                            let sigma_u = cov[2][2].abs().sqrt();
                            let sigma_h = (sigma_e * sigma_e + sigma_n * sigma_n).sqrt();
                            let hpl = sigma_h * 6.0;
                            let vpl = sigma_u * 6.0;
                            let mut obj = serde_json::json!({
                                "epoch_idx": rover.epoch_idx,
                                "fixed": adjusted.fixed,
                                "sigma_e": sigma_e,
                                "sigma_n": sigma_n,
                                "sigma_u": sigma_u,
                                "used_sats": used_sats,
                                "residual_rms_m": rms_obs,
                                "predicted_rms_m": rms_pred,
                                "hpl_m": hpl,
                                "vpl_m": vpl
                            });
                            if let serde_json::Value::Object(map) = &mut obj {
                                if let Some(sig) = sep_sig {
                                    map.insert(
                                        "separation_sig".to_string(),
                                        serde_json::Value::String(sig),
                                    );
                                }
                                if let Some(val) = sep_max {
                                    map.insert(
                                        "separation_max_m".to_string(),
                                        serde_json::Value::Number(
                                            serde_json::Number::from_f64(val)
                                                .unwrap_or_else(|| serde_json::Number::from(0)),
                                        ),
                                    );
                                }
                            }
                            baseline_quality_lines.push(serde_json::to_string(&obj)?);
                        }
                        baseline_lines.push(serde_json::to_string(&adjusted)?);
                    }

                    let slip_count = base
                        .sats
                        .iter()
                        .chain(rover.sats.iter())
                        .filter(|s| s.lock_flags.cycle_slip)
                        .count();
                    precision_lines.push(serde_json::to_string(&serde_json::json!({
                        "epoch_idx": rover.epoch_idx,
                        "fix_accepted": fix_result.accepted,
                        "ratio": fix_result.ratio,
                        "fixed_count": audit.fixed_count,
                        "ref_changed": ref_changed,
                        "slip_count": slip_count
                    }))?);
                }

                let sd_path = out_dir.join("rtk_sd.jsonl");
                fs::write(&sd_path, sd_lines.join("\n"))?;
                let dd_path = out_dir.join("rtk_dd.jsonl");
                fs::write(&dd_path, dd_lines.join("\n"))?;
                let baseline_path = out_dir.join("rtk_baseline.jsonl");
                fs::write(&baseline_path, baseline_lines.join("\n"))?;
                let baseline_quality_path = out_dir.join("rtk_baseline_quality.jsonl");
                fs::write(&baseline_quality_path, baseline_quality_lines.join("\n"))?;
                let fix_audit_path = out_dir.join("rtk_fix_audit.jsonl");
                fs::write(&fix_audit_path, fix_audit_lines.join("\n"))?;
                let precision_path = out_dir.join("rtk_precision.jsonl");
                fs::write(&precision_path, precision_lines.join("\n"))?;

                let align_report = aligner.report(base_epochs.len(), rover_epochs.len());
                fs::write(
                    out_dir.join("rtk_align.json"),
                    serde_json::to_string_pretty(&align_report)?,
                )?;

                validate_json_schema(
                    &schema_path("rtk_baseline.schema.json"),
                    &baseline_path,
                    false,
                )?;
                validate_jsonl_schema(
                    &schema_path("rtk_baseline_quality.schema.json"),
                    &baseline_quality_path,
                    false,
                )?;
                validate_jsonl_schema(
                    &schema_path("rtk_fix_audit.schema.json"),
                    &fix_audit_path,
                    false,
                )?;
                validate_jsonl_schema(
                    &schema_path("rtk_precision.schema.json"),
                    &precision_path,
                    false,
                )?;
            }
            GnssCommand::Experiment {
                common,
                scenario,
                sweep,
            } => {
                set_trace_dir(&common);
                let mut profile = load_profile(&common)?;
                apply_common_overrides(&mut profile, &common);
                profile
                    .validate()
                    .map_err(|errs| eyre!("invalid config before sweep: {}", errs.join(", ")))?;

                let scenario_contents = fs::read_to_string(&scenario)
                    .with_context(|| format!("failed to read scenario {}", scenario.display()))?;
                let scenario_def: bijux_gnss_receiver::synthetic::SyntheticScenario =
                    toml::from_str(&scenario_contents)?;

                let sweep_spec = parse_sweep(&sweep)?;
                let runs = expand_sweep(&sweep_spec);
                let Some(out_dir) = &common.out else {
                    bail!("--out is required for experiment");
                };
                fs::create_dir_all(out_dir)?;

                let mut results = Vec::new();
                for (idx, overrides) in runs.iter().enumerate() {
                    let mut run_profile = profile.clone();
                    for (key, value) in overrides {
                        apply_sweep_value(&mut run_profile, key, value)?;
                    }
                    run_profile
                        .validate()
                        .map_err(|errs| eyre!("invalid config in sweep: {}", errs.join(", ")))?;

                    let config = run_profile.to_receiver_config();
                    let start = std::time::Instant::now();
                    let frame = bijux_gnss_receiver::synthetic::generate_l1_ca_multi(
                        &config,
                        &scenario_def,
                    );
                    let sats: Vec<SatId> = scenario_def.satellites.iter().map(|s| s.sat).collect();
                    let acquisition = Acquisition::new(config.clone())
                        .with_doppler(10_000, run_profile.acquisition.doppler_step_hz);
                    let acquisitions = acquisition.run_fft(&frame, &sats);
                    let tracking = bijux_gnss_receiver::tracking::Tracking::new(config.clone());
                    let tracks = tracking.track_from_acquisition(
                        &frame,
                        &acquisitions,
                        bijux_gnss_core::SignalBand::L1,
                    );
                    let obs = bijux_gnss_receiver::observations::observations_from_tracking_results(
                        &config,
                        &tracks,
                        run_profile.navigation.hatch_window,
                    );
                    let mut nav = bijux_gnss_receiver::navigation::Navigation::new(config.clone());
                    let mut solutions = Vec::new();
                    for epoch in &obs {
                        if let Some(solution) = nav.solve_epoch(epoch, &scenario_def.ephemerides) {
                            solutions.push(solution);
                        }
                    }
                    let elapsed = start.elapsed().as_secs_f64();
                    let epochs = obs.len().max(1) as f64;
                    let ms_per_epoch = (elapsed * 1000.0) / epochs;

                    let lock_total: usize = tracks.iter().map(|t| t.epochs.len()).sum();
                    let lock_good: usize = tracks
                        .iter()
                        .map(|t| t.epochs.iter().filter(|e| e.lock).count())
                        .sum();
                    let lock_pct = if lock_total > 0 {
                        lock_good as f64 / lock_total as f64
                    } else {
                        0.0
                    };

                    let mean_pvt_rms = if solutions.is_empty() {
                        0.0
                    } else {
                        solutions.iter().map(|s| s.rms_m).sum::<f64>() / solutions.len() as f64
                    };
                    let mean_residual_rms = mean_pvt_rms;
                    let rejected_count = solutions
                        .iter()
                        .flat_map(|s| s.residuals.iter().filter(|r| r.rejected))
                        .count();

                    let config_hash = hash_config(common.config.as_ref(), &run_profile)?;
                    let result = ExperimentRunResult {
                        run_index: idx,
                        config_hash,
                        scenario_id: scenario_def.id.clone(),
                        overrides: overrides.clone(),
                        lock_pct,
                        pvt_rms_m: mean_pvt_rms,
                        residual_rms_m: mean_residual_rms,
                        rejected_count,
                        ms_per_epoch,
                    };
                    write_experiment_run(out_dir, idx, &result, &obs, &solutions)?;
                    results.push(result);
                }

                let summary = ExperimentSummary {
                    runs: results.clone(),
                };
                let summary_path = out_dir.join("experiment_summary.json");
                fs::write(&summary_path, serde_json::to_string_pretty(&summary)?)?;
                let schema_path = schema_path("experiment_summary.schema.json");
                if schema_path.exists() {
                    validate_json_schema(&schema_path, &summary_path, false)?;
                }
            }
            GnssCommand::ValidateConfig { common, config } => {
                set_trace_dir(&common);
                let path = config
                    .or(common.config.clone())
                    .context("--config is required")?;
                let profile = load_profile_from_path(&path)?;
                validate_config_schema(&profile)?;
                match profile.validate() {
                    Ok(()) => {
                        println!("config valid: {}", path.display());
                    }
                    Err(errors) => {
                        bail!("config invalid: {}", errors.join(", "));
                    }
                }
            }
            GnssCommand::ValidateArtifacts {
                common,
                obs,
                eph,
                strict,
            } => {
                set_trace_dir(&common);
                if obs.is_none() && eph.is_none() {
                    bail!("--obs and/or --eph is required");
                }
                if let Some(path) = obs {
                    validate_jsonl_schema(&schema_path("obs_epoch.schema.json"), &path, strict)?;
                    println!("obs ok: {}", path.display());
                }
                if let Some(path) = eph {
                    validate_json_schema(&schema_path("gps_ephemeris.schema.json"), &path, strict)?;
                    println!("ephemeris ok: {}", path.display());
                }
            }
            GnssCommand::ValidateSidecar { common, sidecar } => {
                set_trace_dir(&common);
                let spec = load_sidecar(Some(&sidecar))?;
                if spec.is_some() {
                    println!("sidecar ok: {}", sidecar.display());
                }
            }
            GnssCommand::ConfigSchema { common, out } => {
                set_trace_dir(&common);
                let schema = schema_for!(ReceiverProfile);
                fs::write(&out, serde_json::to_string_pretty(&schema)?)?;
                println!("wrote {}", out.display());
            }
            GnssCommand::Validate {
                common,
                file,
                eph,
                reference,
                prn,
                sp3,
                clk,
            } => {
                set_trace_dir(&common);
                let dataset = load_dataset(&common)?;
                let mut profile = load_profile(&common)?;
                apply_common_overrides(&mut profile, &common);
                if let Some(entry) = &dataset {
                    profile.sample_rate_hz = entry.sample_rate_hz;
                    profile.intermediate_freq_hz = entry.intermediate_freq_hz;
                }
                profile
                    .validate()
                    .map_err(|errs| eyre!("invalid config: {}", errs.join(", ")))?;
                validate_config_schema(&profile)?;
                let config = profile.to_receiver_config();
                let input_file = resolve_input_file(file.as_ref(), dataset.as_ref())?;
                let sidecar = load_sidecar(common.sidecar.as_ref())?;
                let frame = load_frame(&input_file, &config, 0, sidecar.as_ref())?;

                let acquisition = Acquisition::new(config.clone());
                let sats = prns_to_sats(&prn);
                let acquisitions = acquisition.run_fft(&frame, &sats);
                let tracking = bijux_gnss_receiver::tracking::Tracking::new(config.clone());
                let tracks = tracking.track_from_acquisition(
                    &frame,
                    &acquisitions,
                    bijux_gnss_core::SignalBand::L1,
                );
                let obs = bijux_gnss_receiver::observations::observations_from_tracking_results(
                    &config,
                    &tracks,
                    profile.navigation.hatch_window,
                );
                let ephs = read_ephemeris(&eph)?;
                let mut products_diag = bijux_gnss_nav::ProductDiagnostics::new();
                let mut products_ok = false;
                let mut products = bijux_gnss_nav::Products::new(
                    bijux_gnss_nav::BroadcastProductsProvider::new(ephs.clone()),
                );
                if sp3.is_some() || clk.is_some() {
                    if let Some(sp3_path) = sp3 {
                        let data = fs::read_to_string(sp3_path)?;
                        let sp3: bijux_gnss_nav::Sp3Provider = data
                            .parse()
                            .map_err(|err| eyre!("SP3 parse failed: {err}"))?;
                        products = products.with_sp3(sp3);
                    } else {
                        products_diag.fallback("SP3 not provided");
                    }
                    if let Some(clk_path) = clk {
                        let data = fs::read_to_string(clk_path)?;
                        let clk: bijux_gnss_nav::ClkProvider = data
                            .parse()
                            .map_err(|err| eyre!("CLK parse failed: {err}"))?;
                        products = products.with_clk(clk);
                    } else {
                        products_diag.fallback("CLK not provided");
                    }
                    if let Some(first_epoch) = obs.first() {
                        if let Some(first_sat) = first_epoch.sats.first() {
                            let sat = first_sat.signal_id.sat;
                            let t = first_epoch.t_rx_s;
                            let state = products.sat_state(sat, t, &mut products_diag);
                            let clock = products.clock_bias_s(sat, t, &mut products_diag);
                            products_ok = state.is_some()
                                && clock.is_some()
                                && products_diag.fallbacks.is_empty();
                        }
                    }
                } else {
                    products_diag.fallback("products not provided");
                }
                let mut nav = bijux_gnss_receiver::navigation::Navigation::new(config.clone());

                let mut solutions = Vec::new();
                for epoch in &obs {
                    if let Some(solution) = nav.solve_epoch(epoch, &ephs) {
                        solutions.push(solution);
                    }
                }

                let reference_epochs = read_reference_epochs(&reference)?;
                let report = build_validation_report(
                    &tracks,
                    &obs,
                    &solutions,
                    &reference_epochs,
                    profile.sample_rate_hz,
                    products_ok,
                    products_diag.fallbacks.clone(),
                )?;

                let out_dir = common
                    .out
                    .clone()
                    .context("--out is required for validate")?;
                fs::create_dir_all(&out_dir)?;
                let path = out_dir.join("validation_report.json");
                fs::write(&path, serde_json::to_string_pretty(&report)?)?;
                println!("wrote {}", path.display());

                let (rows, stats) = reference_compare(&solutions, &reference_epochs);
                let compare_path = out_dir.join("reference_compare.json");
                fs::write(&compare_path, serde_json::to_string_pretty(&stats)?)?;
                let compare_csv = out_dir.join("reference_compare.csv");
                fs::write(&compare_csv, rows.join("\n"))?;

                if profile.navigation.ppp.enabled {
                    let ppp_config = build_ppp_config(&profile);
                    let mut ppp = PppFilter::new(ppp_config);
                    let mut ppp_solutions = Vec::new();
                    for epoch in &obs {
                        if let Some(sol) = ppp.solve_epoch(epoch, &ephs, &products) {
                            ppp_solutions.push(sol);
                        }
                    }
                    let ppp_report = ppp_evaluation_report(&ppp_solutions, &reference_epochs);
                    let ppp_path = out_dir.join("ppp_report.json");
                    fs::write(&ppp_path, serde_json::to_string_pretty(&ppp_report)?)?;
                }
            }
            GnssCommand::Run {
                common,
                file,
                replay,
                rate,
            } => {
                set_trace_dir(&common);
                let dataset = load_dataset(&common)?;
                let mut profile = load_profile(&common)?;
                apply_common_overrides(&mut profile, &common);
                if let Some(entry) = &dataset {
                    profile.sample_rate_hz = entry.sample_rate_hz;
                    profile.intermediate_freq_hz = entry.intermediate_freq_hz;
                }
                profile
                    .validate()
                    .map_err(|errs| eyre!("invalid config: {}", errs.join(", ")))?;
                let config = profile.to_receiver_config();
                let input_file = resolve_input_file(file.as_ref(), dataset.as_ref())?;
                let sidecar = load_sidecar(common.sidecar.as_ref())?;
                let mut source = FileSamples::open(
                    input_file.to_str().unwrap_or_default(),
                    sidecar.as_ref().map(|s| s.offset_bytes).unwrap_or(0),
                    sidecar
                        .as_ref()
                        .map(|s| s.sample_rate_hz)
                        .unwrap_or(config.sampling_freq_hz),
                )?;
                let samples_per_code = samples_per_code(
                    config.sampling_freq_hz,
                    config.code_freq_basis_hz,
                    config.code_length,
                );
                let mut epoch = 0u64;
                while let Some(frame) = source.next_frame(samples_per_code)? {
                    let _ = frame;
                    epoch += 1;
                    if epoch.is_multiple_of(100) {
                        println!("processed epochs: {}", epoch);
                    }
                    if replay && rate > 0.0 {
                        let dt = 0.001 / rate;
                        std::thread::sleep(std::time::Duration::from_secs_f64(dt));
                    }
                }
            }
            GnssCommand::Rinex {
                common,
                obs,
                eph,
                strict,
            } => {
                set_trace_dir(&common);
                let obs_epochs = read_obs_epochs(&obs)?;
                let ephs = read_ephemeris(&eph)?;
                let out_dir = common.out.clone().context("--out is required for rinex")?;
                fs::create_dir_all(&out_dir)?;
                let obs_path = out_dir.join("obs.rnx");
                let nav_path = out_dir.join("nav.rnx");
                write_rinex_obs(&obs_path, &obs_epochs, strict)?;
                write_rinex_nav(&nav_path, &ephs, strict)?;
                println!("wrote {}", obs_path.display());
                println!("wrote {}", nav_path.display());
            }
            GnssCommand::Doctor { common, file } => {
                set_trace_dir(&common);
                println!("bijux-gnss doctor");
                #[cfg(any(target_arch = "x86", target_arch = "x86_64"))]
                {
                    println!("sse2: {}", std::is_x86_feature_detected!("sse2"));
                    println!("avx2: {}", std::is_x86_feature_detected!("avx2"));
                }
                let mut sample_rate_hz = 5_000_000.0;
                match load_profile(&common) {
                    Ok(config) => {
                        let _ = validate_config_schema(&config);
                        match config.validate() {
                            Ok(()) => println!("config: ok"),
                            Err(errs) => println!("config errors: {}", errs.join(", ")),
                        }
                        sample_rate_hz = config.sample_rate_hz;
                    }
                    Err(err) => println!("config load error: {}", err),
                }
                let leap = bijux_gnss_core::LeapSeconds::default_table();
                match leap.validate() {
                    Ok(()) => println!("leap seconds: ok (offset {})", leap.latest_offset()),
                    Err(err) => println!("leap seconds: {err}"),
                }
                if let Some(path) = file {
                    let readable =
                        FileSamples::open(path.to_str().unwrap_or_default(), 0, sample_rate_hz);
                    println!("dataset readable: {}", readable.is_ok());
                }
                println!("fft backend: rustfft");
            }
        },
    }

    Ok(())
}

fn apply_overrides(
    profile: &mut ReceiverProfile,
    sampling_hz: Option<f64>,
    if_hz: Option<f64>,
    code_hz: Option<f64>,
    code_length: Option<usize>,
) {
    if let Some(value) = sampling_hz {
        profile.sample_rate_hz = value;
    }
    if let Some(value) = if_hz {
        profile.intermediate_freq_hz = value;
    }
    if let Some(value) = code_hz {
        profile.code_freq_basis_hz = value;
    }
    if let Some(value) = code_length {
        profile.code_length = value;
    }
}

fn apply_common_overrides(profile: &mut ReceiverProfile, common: &CommonArgs) {
    if let Some(seed) = common.seed {
        profile.seed = seed;
    }
}

fn load_profile(common: &CommonArgs) -> Result<ReceiverProfile> {
    match &common.config {
        Some(path) => load_profile_from_path(path),
        None => Ok(ReceiverProfile::default()),
    }
}

fn load_profile_from_path(path: &Path) -> Result<ReceiverProfile> {
    let contents = fs::read_to_string(path)
        .with_context(|| format!("failed to read config {}", path.display()))?;
    let profile: ReceiverProfile = toml::from_str(&contents)
        .with_context(|| format!("failed to parse config {}", path.display()))?;
    Ok(profile)
}

fn load_dataset(common: &CommonArgs) -> Result<Option<DatasetEntry>> {
    let Some(id) = &common.dataset else {
        return Ok(None);
    };
    let registry_path = PathBuf::from("datasets/registry.yaml");
    let contents = fs::read_to_string(&registry_path)
        .with_context(|| format!("failed to read {}", registry_path.display()))?;
    let registry: DatasetRegistry = serde_yaml::from_str(&contents)
        .with_context(|| format!("failed to parse {}", registry_path.display()))?;
    let entry = registry
        .entries
        .into_iter()
        .find(|e| e.id == *id)
        .with_context(|| format!("dataset not found: {id}"))?;
    Ok(Some(entry))
}

fn resolve_input_file(file: Option<&PathBuf>, dataset: Option<&DatasetEntry>) -> Result<PathBuf> {
    if let Some(file) = file {
        return Ok(file.clone());
    }
    if let Some(dataset) = dataset {
        return Ok(PathBuf::from(&dataset.path));
    }
    bail!("no input file provided; use --file or --dataset");
}

fn load_sidecar(path: Option<&PathBuf>) -> Result<Option<SidecarSpec>> {
    let Some(path) = path else {
        return Ok(None);
    };
    let data = fs::read_to_string(path)?;
    let spec: SidecarSpec = toml::from_str(&data)?;
    validate_sidecar_schema(&spec)?;
    Ok(Some(spec))
}

fn emit_report<T: Serialize>(common: &CommonArgs, command: &str, report: &T) -> Result<()> {
    match common.report {
        ReportFormat::Json => {
            let json = serde_json::to_string_pretty(report)?;
            if let Some(out_dir) = &common.out {
                fs::create_dir_all(out_dir)?;
                let path = out_dir.join(format!("{command}_report.json"));
                fs::write(&path, json)?;
                println!("wrote {}", path.display());
            } else {
                println!("{json}");
            }
        }
        ReportFormat::Table => {
            let json = serde_json::to_string_pretty(report)?;
            println!("{json}");
        }
    }
    Ok(())
}

fn load_frame(
    path: &Path,
    config: &ReceiverConfig,
    offset_bytes: u64,
    sidecar: Option<&SidecarSpec>,
) -> Result<SamplesFrame> {
    let samples_per_code = samples_per_code(
        sidecar
            .map(|s| s.sample_rate_hz)
            .unwrap_or(config.sampling_freq_hz),
        config.code_freq_basis_hz,
        config.code_length,
    );
    let offset = sidecar.map(|s| s.offset_bytes).unwrap_or(offset_bytes);
    let mut source = FileSamples::open(
        path.to_str().unwrap_or_default(),
        offset,
        sidecar
            .map(|s| s.sample_rate_hz)
            .unwrap_or(config.sampling_freq_hz),
    )
    .with_context(|| format!("failed to open {}", path.display()))?;

    let frame = match source.next_frame(samples_per_code)? {
        Some(frame) => frame,
        None => bail!("no samples available in {}", path.display()),
    };
    if frame.len() < samples_per_code {
        bail!(
            "not enough samples: need {samples_per_code}, got {}",
            frame.len()
        );
    }
    Ok(frame)
}

fn write_manifest<T: Serialize>(
    common: &CommonArgs,
    command: &str,
    profile: &ReceiverProfile,
    dataset: Option<&DatasetEntry>,
    report: &T,
) -> Result<()> {
    let Some(out_dir) = &common.out else {
        return Ok(());
    };
    fs::create_dir_all(out_dir)?;

    let config_hash = hash_config(common.config.as_ref(), profile)?;
    let git_hash = git_hash().unwrap_or_else(|| "unknown".to_string());
    let manifest = RunManifest {
        command: command.to_string(),
        timestamp_unix_ms: SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_millis(),
        git_hash,
        config_hash,
        dataset_id: dataset.map(|d| d.id.clone()),
        build_profile: std::env::var("PROFILE").unwrap_or_else(|_| "unknown".to_string()),
        cpu_features: cpu_features(),
        results: serde_json::to_value(report)?,
    };

    let json = serde_json::to_string_pretty(&manifest)?;
    fs::write(out_dir.join("run.json"), json)?;
    Ok(())
}

fn hash_config(path: Option<&PathBuf>, profile: &ReceiverProfile) -> Result<String> {
    let mut hasher = Sha256::new();
    if let Some(path) = path {
        let bytes = fs::read(path)?;
        hasher.update(bytes);
    } else {
        let serialized = toml::to_string(profile)?;
        hasher.update(serialized.as_bytes());
    }
    Ok(hex::encode(hasher.finalize()))
}

fn git_hash() -> Option<String> {
    let output = ProcessCommand::new("git")
        .args(["rev-parse", "HEAD"])
        .output()
        .ok()?;
    if !output.status.success() {
        return None;
    }
    let hash = String::from_utf8_lossy(&output.stdout).trim().to_string();
    Some(hash)
}

fn cpu_features() -> Vec<String> {
    let mut features = Vec::new();
    #[cfg(target_arch = "x86_64")]
    {
        if std::is_x86_feature_detected!("avx2") {
            features.push("avx2".to_string());
        }
        if std::is_x86_feature_detected!("sse4.2") {
            features.push("sse4.2".to_string());
        }
        if std::is_x86_feature_detected!("fma") {
            features.push("fma".to_string());
        }
    }
    features
}

fn inspect_dataset(path: &Path, sample_rate_hz: f64, max_samples: usize) -> Result<InspectReport> {
    let bytes = fs::read(path).with_context(|| format!("failed to read {}", path.display()))?;
    let mut samples = Vec::with_capacity(bytes.len() / 2);
    for chunk in bytes.chunks_exact(2) {
        samples.push(i16::from_le_bytes([chunk[0], chunk[1]]));
    }

    let total_iq = samples.len() / 2;
    let limit = if max_samples == 0 {
        total_iq
    } else {
        total_iq.min(max_samples)
    };

    let mut sum_i = 0.0f64;
    let mut sum_q = 0.0f64;
    let mut clip = 0u64;
    let mut power_hist = vec![0u64; 8];
    let mut power_sum = 0.0f64;

    for idx in 0..limit {
        let i = samples[2 * idx] as f64;
        let q = samples[2 * idx + 1] as f64;
        sum_i += i;
        sum_q += q;
        if samples[2 * idx].abs() == i16::MAX || samples[2 * idx + 1].abs() == i16::MAX {
            clip += 1;
        }
        let power = i * i + q * q;
        power_sum += power;
        let bin = ((power.sqrt() / 10000.0).min(7.0)) as usize;
        power_hist[bin] += 1;
    }

    let dc_i = sum_i / limit.max(1) as f64;
    let dc_q = sum_q / limit.max(1) as f64;
    let mean_power = power_sum / limit.max(1) as f64;
    let noise_floor_db = 10.0 * mean_power.max(1e-9).log10();

    Ok(InspectReport {
        sample_rate_hz,
        total_samples: limit,
        dc_offset_i: dc_i,
        dc_offset_q: dc_q,
        clip_rate: clip as f64 / limit.max(1) as f64,
        noise_floor_db,
        power_histogram: power_hist,
    })
}

fn set_trace_dir(common: &CommonArgs) {
    if let Some(dir) = &common.dump {
        std::env::set_var("BIJUX_TRACE_DIR", dir);
    }
}

fn print_acquisition_table(report: &AcquisitionReport) {
    println!("Sat\tCarrier(Hz)\tCodePhase\tPeak\tPeak/Mean\tPeak/2nd");
    for row in &report.results {
        println!(
            "{}\t{:.1}\t{}\t{:.3}\t{:.2}\t{:.2}",
            format_sat(row.sat),
            row.carrier_hz,
            row.code_phase_samples,
            row.peak,
            row.peak_mean_ratio,
            row.peak_second_ratio
        );
    }
}

fn print_inspect_table(report: &InspectReport) {
    println!("SampleRate(Hz)\tSamples\tDC_I\tDC_Q\tClipRate\tNoiseFloor(dB)");
    println!(
        "{:.1}\t{}\t{:.3}\t{:.3}\t{:.6}\t{:.2}",
        report.sample_rate_hz,
        report.total_samples,
        report.dc_offset_i,
        report.dc_offset_q,
        report.clip_rate,
        report.noise_floor_db
    );
    println!("Power histogram bins: {:?}", report.power_histogram);
}

fn format_sat(sat: SatId) -> String {
    format!("{:?}-{}", sat.constellation, sat.prn)
}

fn prns_to_sats(prns: &[u8]) -> Vec<SatId> {
    prns.iter()
        .map(|&prn| SatId {
            constellation: Constellation::Gps,
            prn,
        })
        .collect()
}

fn parse_sweep(values: &[String]) -> Result<Vec<(String, Vec<String>)>> {
    let mut out = Vec::new();
    for item in values {
        let Some((key, vals)) = item.split_once('=') else {
            bail!("invalid sweep format: {item}");
        };
        let vals: Vec<String> = vals.split(',').map(|v| v.trim().to_string()).collect();
        if vals.is_empty() {
            bail!("sweep values empty for {key}");
        }
        out.push((key.trim().to_string(), vals));
    }
    Ok(out)
}

fn expand_sweep(spec: &[(String, Vec<String>)]) -> Vec<Vec<(String, String)>> {
    fn expand(
        idx: usize,
        spec: &[(String, Vec<String>)],
        current: &mut Vec<(String, String)>,
        out: &mut Vec<Vec<(String, String)>>,
    ) {
        if idx == spec.len() {
            out.push(current.clone());
            return;
        }
        let (key, vals) = &spec[idx];
        for val in vals {
            current.push((key.clone(), val.clone()));
            expand(idx + 1, spec, current, out);
            current.pop();
        }
    }
    let mut out = Vec::new();
    expand(0, spec, &mut Vec::new(), &mut out);
    if out.is_empty() {
        out.push(Vec::new());
    }
    out
}

fn apply_sweep_value(profile: &mut ReceiverProfile, key: &str, value: &str) -> Result<()> {
    match key {
        "tracking.dll_bw_hz" => profile.tracking.dll_bw_hz = value.parse()?,
        "tracking.pll_bw_hz" => profile.tracking.pll_bw_hz = value.parse()?,
        "tracking.fll_bw_hz" => profile.tracking.fll_bw_hz = value.parse()?,
        "tracking.early_late_spacing_chips" => {
            profile.tracking.early_late_spacing_chips = value.parse()?
        }
        "acquisition.integration_ms" => profile.acquisition.integration_ms = value.parse()?,
        "acquisition.doppler_step_hz" => profile.acquisition.doppler_step_hz = value.parse()?,
        "acquisition.doppler_search_hz" => profile.acquisition.doppler_search_hz = value.parse()?,
        "acquisition.peak_mean_threshold" => {
            profile.acquisition.peak_mean_threshold = value.parse()?
        }
        "acquisition.peak_second_threshold" => {
            profile.acquisition.peak_second_threshold = value.parse()?
        }
        "navigation.hatch_window" => profile.navigation.hatch_window = value.parse()?,
        "navigation.weighting.elev_mask_deg" => {
            profile.navigation.weighting.elev_mask_deg = value.parse()?
        }
        "navigation.weighting.elev_exponent" => {
            profile.navigation.weighting.elev_exponent = value.parse()?
        }
        "navigation.weighting.cn0_ref_dbhz" => {
            profile.navigation.weighting.cn0_ref_dbhz = value.parse()?
        }
        "navigation.weighting.min_weight" => {
            profile.navigation.weighting.min_weight = value.parse()?
        }
        "navigation.weighting.tracking_mode_scalar_weight" => {
            profile.navigation.weighting.tracking_mode_scalar_weight = value.parse()?
        }
        "navigation.weighting.tracking_mode_vector_weight" => {
            profile.navigation.weighting.tracking_mode_vector_weight = value.parse()?
        }
        "navigation.robust_solver" => profile.navigation.robust_solver = value.parse()?,
        "navigation.raim" => profile.navigation.raim = value.parse()?,
        _ => bail!("unsupported sweep parameter: {key}"),
    }
    Ok(())
}

fn write_experiment_run(
    out_dir: &Path,
    idx: usize,
    result: &ExperimentRunResult,
    obs: &[ObsEpoch],
    solutions: &[bijux_gnss_core::NavSolutionEpoch],
) -> Result<()> {
    let run_dir = out_dir.join(format!("run_{idx:03}"));
    fs::create_dir_all(&run_dir)?;
    fs::write(
        run_dir.join("result.json"),
        serde_json::to_string_pretty(result)?,
    )?;
    let schema_path = schema_path("experiment_run.schema.json");
    if schema_path.exists() {
        validate_json_schema(&schema_path, &run_dir.join("result.json"), false)?;
    }

    let mut cn0_lines = Vec::new();
    cn0_lines.push("epoch_idx,constellation,prn,cn0_dbhz".to_string());
    for epoch in obs {
        for sat in &epoch.sats {
            cn0_lines.push(format!(
                "{},{:?},{},{}",
                epoch.epoch_idx,
                sat.signal_id.sat.constellation,
                sat.signal_id.sat.prn,
                sat.cn0_dbhz
            ));
        }
    }
    fs::write(run_dir.join("cn0.csv"), cn0_lines.join("\n"))?;
    validate_csv_schema(
        &run_dir.join("cn0.csv"),
        "epoch_idx,constellation,prn,cn0_dbhz",
        &[CsvType::U64, CsvType::Str, CsvType::U8, CsvType::F64],
    )?;

    let mut residual_lines = Vec::new();
    residual_lines.push("epoch_idx,constellation,prn,residual_m,rejected".to_string());
    for sol in solutions {
        for res in &sol.residuals {
            residual_lines.push(format!(
                "{},{:?},{},{},{}",
                sol.epoch.index, res.sat.constellation, res.sat.prn, res.residual_m, res.rejected
            ));
        }
    }
    fs::write(run_dir.join("residuals.csv"), residual_lines.join("\n"))?;
    validate_csv_schema(
        &run_dir.join("residuals.csv"),
        "epoch_idx,constellation,prn,residual_m,rejected",
        &[
            CsvType::U64,
            CsvType::Str,
            CsvType::U8,
            CsvType::F64,
            CsvType::Bool,
        ],
    )?;

    let mut ambiguity_lines = Vec::new();
    ambiguity_lines.push("epoch_idx,constellation,prn,carrier_phase_cycles".to_string());
    for epoch in obs {
        for sat in &epoch.sats {
            ambiguity_lines.push(format!(
                "{},{:?},{},{}",
                epoch.epoch_idx,
                sat.signal_id.sat.constellation,
                sat.signal_id.sat.prn,
                sat.carrier_phase_cycles
            ));
        }
    }
    fs::write(run_dir.join("ambiguities.csv"), ambiguity_lines.join("\n"))?;
    validate_csv_schema(
        &run_dir.join("ambiguities.csv"),
        "epoch_idx,constellation,prn,carrier_phase_cycles",
        &[CsvType::U64, CsvType::Str, CsvType::U8, CsvType::F64],
    )?;
    Ok(())
}

#[derive(Clone, Copy)]
enum CsvType {
    U64,
    U8,
    F64,
    Bool,
    Str,
}

fn validate_csv_schema(path: &Path, expected: &str, types: &[CsvType]) -> Result<()> {
    let contents = fs::read_to_string(path)?;
    let header = contents.lines().next().unwrap_or_default().trim();
    if header != expected {
        bail!(
            "csv header mismatch for {}: expected `{}`, got `{}`",
            path.display(),
            expected,
            header
        );
    }
    for (idx, line) in contents.lines().enumerate().skip(1) {
        if line.trim().is_empty() {
            continue;
        }
        let cols: Vec<&str> = line.split(',').collect();
        if cols.len() != types.len() {
            bail!(
                "csv column count mismatch for {} line {}",
                path.display(),
                idx + 1
            );
        }
        for (value, ty) in cols.iter().zip(types.iter()) {
            match ty {
                CsvType::U64 => {
                    value.parse::<u64>().map_err(|_| {
                        eyre!("csv parse error in {} line {}", path.display(), idx + 1)
                    })?;
                }
                CsvType::U8 => {
                    value.parse::<u8>().map_err(|_| {
                        eyre!("csv parse error in {} line {}", path.display(), idx + 1)
                    })?;
                }
                CsvType::F64 => {
                    value.parse::<f64>().map_err(|_| {
                        eyre!("csv parse error in {} line {}", path.display(), idx + 1)
                    })?;
                }
                CsvType::Bool => {
                    value.parse::<bool>().map_err(|_| {
                        eyre!("csv parse error in {} line {}", path.display(), idx + 1)
                    })?;
                }
                CsvType::Str => {
                    if value.trim().is_empty() {
                        bail!(
                            "csv parse error in {} line {}: empty string",
                            path.display(),
                            idx + 1
                        );
                    }
                }
            }
        }
    }
    Ok(())
}

fn solve_epoch_ekf(
    ctx: &mut Option<EkfContext>,
    obs: &ObsEpoch,
    ephs: &[GpsEphemeris],
) -> Result<Option<bijux_gnss_core::NavSolutionEpoch>> {
    let Some(ctx) = ctx.as_mut() else {
        return Ok(None);
    };
    let dt_s = if let Some(prev) = ctx.last_t_rx_s {
        (obs.t_rx_s - prev).max(1e-3)
    } else {
        0.001
    };
    ctx.last_t_rx_s = Some(obs.t_rx_s);
    ctx.ekf.predict(&ctx.model, dt_s);

    let mut used = 0;
    let mut sats: Vec<&bijux_gnss_core::ObsSatellite> = obs.sats.iter().collect();
    sats.sort_by_key(|s| s.signal_id);
    for sat in sats {
        let eph = match ephs.iter().find(|e| e.sat == sat.signal_id.sat) {
            Some(e) => e,
            None => continue,
        };
        let _corr = bijux_gnss_nav::compute_corrections(&ctx.corrections);
        let state = sat_state_gps_l1ca(eph, obs.t_rx_s, 0.0);
        let state_next = sat_state_gps_l1ca(eph, obs.t_rx_s + 0.1, 0.0);
        let sat_vel = [
            (state_next.x_m - state.x_m) / 0.1,
            (state_next.y_m - state.y_m) / 0.1,
            (state_next.z_m - state.z_m) / 0.1,
        ];
        let rx_x = ctx.ekf.x[0];
        let rx_y = ctx.ekf.x[1];
        let rx_z = ctx.ekf.x[2];
        let (_az, el) = elevation_azimuth_deg(rx_x, rx_y, rx_z, state.x_m, state.y_m, state.z_m);
        let weight =
            bijux_gnss_nav::weight_from_cn0_elev(sat.cn0_dbhz, el, WeightingConfig::default());
        let sigma_m = (5.0 / weight.max(0.1)).max(1.0);
        let isb_index = if sat.signal_id.sat.constellation != Constellation::Gps {
            let key = format!("isb_{:?}", sat.signal_id.sat.constellation);
            Some(ctx.isb.get_or_add(&mut ctx.ekf, &key, 0.0, 1e-6))
        } else {
            None
        };
        let code_bias_m = ctx.code_bias.code_bias_m(sat.signal_id).unwrap_or(0.0);
        let meas = PseudorangeMeasurement {
            sig: sat.signal_id,
            z_m: sat.pseudorange_m - code_bias_m,
            sat_pos_m: [state.x_m, state.y_m, state.z_m],
            sat_clock_s: state.clock_bias_s,
            tropo_m: 0.0,
            iono_m: 0.0,
            sigma_m,
            elevation_deg: Some(el),
            ztd_index: ctx.ztd_index,
            isb_index,
        };
        if ctx.ekf.update(&meas) {
            used += 1;
        }

        let doppler_meas = bijux_gnss_nav::DopplerMeasurement {
            sig: sat.signal_id,
            z_hz: sat.doppler_hz,
            sat_pos_m: [state.x_m, state.y_m, state.z_m],
            sat_vel_mps: sat_vel,
            wavelength_m: 299_792_458.0 / sat.metadata.signal.carrier_hz.value(),
            sigma_hz: 2.0,
        };
        ctx.ekf.update(&doppler_meas);

        let amb_key = format!(
            "{:?}:{}:{:?}",
            sat.metadata.signal.constellation, sat.signal_id.sat.prn, sat.metadata.signal.band
        );
        let amb_idx = ctx.ambiguity.get_or_add(&mut ctx.ekf, &amb_key, 0.0, 100.0);
        let phase_bias_cycles = ctx
            .phase_bias
            .phase_bias_cycles(sat.signal_id)
            .unwrap_or(0.0);
        let carrier_meas = bijux_gnss_nav::CarrierPhaseMeasurement {
            sig: sat.signal_id,
            z_cycles: sat.carrier_phase_cycles - phase_bias_cycles,
            sat_pos_m: [state.x_m, state.y_m, state.z_m],
            sat_clock_s: state.clock_bias_s,
            tropo_m: 0.0,
            iono_m: 0.0,
            wavelength_m: 299_792_458.0 / sat.metadata.signal.carrier_hz.value(),
            ambiguity_index: Some(amb_idx),
            sigma_cycles: 0.05,
            elevation_deg: Some(el),
            ztd_index: ctx.ztd_index,
            isb_index,
        };
        ctx.ekf.update(&carrier_meas);
    }

    if let Some(idx) = ctx.ztd_index {
        if idx < ctx.ekf.x.len() {
            let before = ctx.ekf.x[idx];
            let after = bijux_gnss_nav::clamp_ztd(before, &ctx.atmosphere);
            if (after - before).abs() > 1e-6 {
                ctx.ekf.x[idx] = after;
                ctx.ekf
                    .health
                    .events
                    .push(bijux_gnss_core::NavHealthEvent::ZtdClamped {
                        before_m: before,
                        after_m: after,
                    });
            }
        }
    }

    if used < 4 {
        return Ok(None);
    }
    let (lat, lon, alt) =
        bijux_gnss_nav::ecef_to_geodetic(ctx.ekf.x[0], ctx.ekf.x[1], ctx.ekf.x[2]);
    Ok(Some(bijux_gnss_core::NavSolutionEpoch {
        epoch: bijux_gnss_core::Epoch {
            index: obs.epoch_idx,
        },
        ecef_x_m: ctx.ekf.x[0],
        ecef_y_m: ctx.ekf.x[1],
        ecef_z_m: ctx.ekf.x[2],
        latitude_deg: lat,
        longitude_deg: lon,
        altitude_m: alt,
        clock_bias_s: ctx.ekf.x[6],
        pdop: 0.0,
        rms_m: ctx.ekf.health.innovation_rms,
        residuals: Vec::new(),
        isb: Vec::new(),
        sigma_h_m: None,
        sigma_v_m: None,
        ekf_innovation_rms: Some(ctx.ekf.health.innovation_rms),
        ekf_condition_number: ctx.ekf.health.condition_number,
        ekf_whiteness_ratio: ctx.ekf.health.whiteness_ratio,
        ekf_predicted_variance: ctx.ekf.health.predicted_variance,
        ekf_observed_variance: ctx.ekf.health.observed_variance,
    }))
}

fn write_track_timeseries(common: &CommonArgs, report: &TrackingReport) -> Result<()> {
    let Some(out_dir) = &common.out else {
        return Ok(());
    };
    fs::create_dir_all(out_dir)?;
    let path = out_dir.join("track.jsonl");
    let mut lines = Vec::new();
    for epoch in &report.epochs {
        let line = serde_json::to_string(epoch)?;
        lines.push(line);
    }
    fs::write(&path, lines.join("\n"))?;
    Ok(())
}

fn write_obs_timeseries(
    common: &CommonArgs,
    config: &ReceiverConfig,
    tracks: &[bijux_gnss_receiver::tracking::TrackingResult],
    hatch_window: u32,
) -> Result<()> {
    let Some(out_dir) = &common.out else {
        return Ok(());
    };
    fs::create_dir_all(out_dir)?;
    let obs = bijux_gnss_receiver::observations::observations_from_tracking_results(
        config,
        tracks,
        hatch_window,
    );
    let path = out_dir.join("obs.jsonl");
    let mut lines = Vec::new();
    for epoch in &obs {
        let line = serde_json::to_string(epoch)?;
        lines.push(line);
    }
    fs::write(&path, lines.join("\n"))?;
    validate_jsonl_schema(&schema_path("obs_epoch.schema.json"), &path, false)?;
    let combos = bijux_gnss_receiver::combinations::combinations_from_obs_epochs(
        &obs,
        bijux_gnss_core::SignalBand::L1,
        bijux_gnss_core::SignalBand::L2,
    );
    if !combos.is_empty() {
        let combo_path = out_dir.join("combinations.jsonl");
        let mut combo_lines = Vec::new();
        for combo in combos {
            combo_lines.push(serde_json::to_string(&combo)?);
        }
        fs::write(&combo_path, combo_lines.join("\n"))?;
        validate_jsonl_schema(&schema_path("combinations.schema.json"), &combo_path, false)?;
    }
    Ok(())
}

fn read_tracking_dump(path: &Path) -> Result<Vec<TrackingRow>> {
    let data = fs::read_to_string(path)?;
    let mut rows = Vec::new();
    for line in data.lines() {
        if line.trim().is_empty() {
            continue;
        }
        let row: TrackingRow = serde_json::from_str(line)?;
        rows.push(row);
    }
    Ok(rows)
}

fn read_obs_epochs(path: &Path) -> Result<Vec<ObsEpoch>> {
    let data = fs::read_to_string(path)?;
    let mut epochs = Vec::new();
    for line in data.lines() {
        if line.trim().is_empty() {
            continue;
        }
        let epoch: ObsEpoch = serde_json::from_str(line)?;
        epochs.push(epoch);
    }
    validate_obs_epochs(&epochs).map_err(|err| eyre!("obs epoch validation failed: {err}"))?;
    Ok(epochs)
}

fn parse_ecef(text: &str) -> Result<[f64; 3]> {
    let parts: Vec<&str> = text.split(',').collect();
    if parts.len() != 3 {
        bail!("invalid ECEF format, expected x,y,z");
    }
    Ok([
        parts[0].trim().parse()?,
        parts[1].trim().parse()?,
        parts[2].trim().parse()?,
    ])
}

fn read_ephemeris(path: &Path) -> Result<Vec<GpsEphemeris>> {
    let data = fs::read_to_string(path)?;
    let ephs: Vec<GpsEphemeris> = serde_json::from_str(&data)?;
    Ok(ephs)
}

fn read_reference_epochs(path: &Path) -> Result<Vec<ValidationReferenceEpoch>> {
    let data = fs::read_to_string(path)?;
    let mut epochs = Vec::new();
    for line in data.lines() {
        if line.trim().is_empty() {
            continue;
        }
        let epoch: ValidationReferenceEpoch = serde_json::from_str(line)?;
        epochs.push(epoch);
    }
    Ok(epochs)
}

fn write_ephemeris(common: &CommonArgs, ephs: &[GpsEphemeris]) -> Result<()> {
    let Some(out_dir) = &common.out else {
        return Ok(());
    };
    fs::create_dir_all(out_dir)?;
    let path = out_dir.join("ephemeris.json");
    let data = serde_json::to_string_pretty(ephs)?;
    fs::write(&path, data)?;
    validate_json_schema(&schema_path("gps_ephemeris.schema.json"), &path, false)?;
    Ok(())
}

fn build_validation_report(
    tracks: &[bijux_gnss_receiver::tracking::TrackingResult],
    obs: &[ObsEpoch],
    solutions: &[bijux_gnss_core::NavSolutionEpoch],
    reference: &[ValidationReferenceEpoch],
    sample_rate_hz: f64,
    products_ok: bool,
    product_fallbacks: Vec<String>,
) -> Result<ValidationReport> {
    let mut ref_map = std::collections::BTreeMap::new();
    for r in reference {
        ref_map.insert(r.epoch_idx, r);
    }

    let mut horiz_errors = Vec::new();
    let mut vert_errors = Vec::new();
    let mut residuals = Vec::new();
    let mut nees_values = Vec::new();

    for sol in solutions {
        if let Some(r) = ref_map.get(&sol.epoch.index) {
            let (x, y, z) = lla_to_ecef(r.latitude_deg, r.longitude_deg, r.altitude_m);
            let dx = sol.ecef_x_m - x;
            let dy = sol.ecef_y_m - y;
            let dz = sol.ecef_z_m - z;
            let horiz = (dx * dx + dy * dy).sqrt();
            let vert = dz.abs();
            horiz_errors.push(horiz);
            vert_errors.push(vert);
            if let (Some(sig_h), Some(sig_v)) = (sol.sigma_h_m, sol.sigma_v_m) {
                if sig_h > 0.0 && sig_v > 0.0 {
                    let (e, n, u) = bijux_gnss_nav::ecef_to_enu(
                        sol.ecef_x_m,
                        sol.ecef_y_m,
                        sol.ecef_z_m,
                        r.latitude_deg,
                        r.longitude_deg,
                        r.altitude_m,
                    );
                    let nees = (e * e + n * n) / (sig_h * sig_h) + (u * u) / (sig_v * sig_v);
                    nees_values.push(nees);
                }
            }
        }
        let mut per_sat = Vec::new();
        let mut rejected = Vec::new();
        for r in &sol.residuals {
            if r.rejected {
                rejected.push(r.sat);
            } else {
                per_sat.push((r.sat, r.residual_m));
            }
        }
        residuals.push(NavResidualReport {
            epoch_idx: sol.epoch.index,
            rms_m: sol.rms_m,
            pdop: sol.pdop,
            residuals: per_sat,
            rejected,
        });
    }

    let horiz_stats = stats(&horiz_errors);
    let vert_stats = stats(&vert_errors);
    let budgets = ValidationBudgets::default();
    let violations = check_budgets(tracks, solutions, &budgets);
    let time_consistency = check_time_consistency(tracks, sample_rate_hz);
    let consistency = check_solution_consistency(solutions);
    let inter_frequency_alignment = bijux_gnss_core::check_inter_frequency_alignment(obs);
    let multi_freq_present = obs.iter().any(|e| {
        let mut by_sat: std::collections::BTreeMap<SatId, std::collections::BTreeSet<_>> =
            std::collections::BTreeMap::new();
        for sat in &e.sats {
            by_sat
                .entry(sat.signal_id.sat)
                .or_default()
                .insert(sat.signal_id.band);
        }
        by_sat.values().any(|bands| bands.len() > 1)
    });
    let combos = bijux_gnss_receiver::combinations::combinations_from_obs_epochs(
        obs,
        bijux_gnss_core::SignalBand::L1,
        bijux_gnss_core::SignalBand::L2,
    );
    let combinations_valid = combos.iter().all(|c| c.status == "ok") && !combos.is_empty();
    let nis_values: Vec<f64> = solutions
        .iter()
        .filter_map(|s| {
            let pred = s.ekf_predicted_variance?;
            if pred > 0.0 {
                Some((s.ekf_innovation_rms.unwrap_or(0.0).powi(2)) / pred)
            } else {
                None
            }
        })
        .collect();
    let nis_mean = if nis_values.is_empty() {
        None
    } else {
        Some(nis_values.iter().sum::<f64>() / nis_values.len() as f64)
    };
    let nees_mean = if nees_values.is_empty() {
        None
    } else {
        Some(nees_values.iter().sum::<f64>() / nees_values.len() as f64)
    };

    Ok(ValidationReport {
        samples: tracks.iter().map(|t| t.epochs.len()).sum(),
        epochs: solutions.len(),
        horiz_error_m: horiz_stats,
        vert_error_m: vert_stats,
        residuals,
        time_consistency,
        consistency,
        budgets,
        budget_violations: violations,
        nis_mean,
        nees_mean,
        inter_frequency_alignment,
        ppp_readiness: PppReadinessReport {
            multi_freq_present,
            combinations_valid,
            products_ok,
            product_fallbacks,
        },
    })
}

fn build_ppp_config(profile: &ReceiverProfile) -> PppConfig {
    let p = &profile.navigation.ppp;
    PppConfig {
        enable_iono_state: p.enable_iono_state,
        use_iono_free: p.use_iono_free,
        use_doppler: p.use_doppler,
        prune_after_epochs: p.prune_after_epochs,
        reset_gap_s: p.reset_gap_s,
        residual_gate_m: p.residual_gate_m,
        drift_window_epochs: p.drift_window_epochs as usize,
        drift_threshold_m: p.drift_threshold_m,
        process_noise: PppProcessNoise {
            clock_drift_s: p.noise_clock_drift,
            ztd_m: p.noise_ztd,
            iono_m: p.noise_iono,
            ambiguity_cycles: p.noise_ambiguity,
        },
        weighting: WeightingConfig {
            enabled: profile.navigation.weighting.enabled,
            min_elev_deg: profile.navigation.weighting.min_elev_deg,
            elev_exponent: profile.navigation.weighting.elev_exponent,
            cn0_ref_dbhz: profile.navigation.weighting.cn0_ref_dbhz,
            min_weight: profile.navigation.weighting.min_weight,
        },
        convergence: bijux_gnss_nav::PppConvergenceConfig {
            min_time_s: p.convergence_min_time_s,
            pos_rate_mps: p.convergence_pos_rate_mps,
            sigma_h_m: p.convergence_sigma_h_m,
            sigma_v_m: p.convergence_sigma_v_m,
        },
    }
}

fn ppp_evaluation_report(
    solutions: &[bijux_gnss_nav::PppSolutionEpoch],
    reference: &[ValidationReferenceEpoch],
) -> PppEvaluationReport {
    let mut ref_map = std::collections::BTreeMap::new();
    for r in reference {
        ref_map.insert(r.epoch_idx, r);
    }
    let mut horiz = Vec::new();
    let mut vert = Vec::new();
    let mut residual_rms = Vec::new();
    let mut last_conv = None;
    for sol in solutions {
        residual_rms.push(sol.rms_m);
        if let Some(r) = ref_map.get(&sol.epoch_idx) {
            let (x, y, z) = lla_to_ecef(r.latitude_deg, r.longitude_deg, r.altitude_m);
            let dx = sol.ecef_x_m - x;
            let dy = sol.ecef_y_m - y;
            let dz = sol.ecef_z_m - z;
            horiz.push((dx * dx + dy * dy).sqrt());
            vert.push(dz.abs());
        }
        last_conv = Some(sol.convergence.clone());
    }
    let horiz_rms = if horiz.is_empty() {
        None
    } else {
        Some((horiz.iter().map(|v| v * v).sum::<f64>() / horiz.len() as f64).sqrt())
    };
    let vert_rms = if vert.is_empty() {
        None
    } else {
        Some((vert.iter().map(|v| v * v).sum::<f64>() / vert.len() as f64).sqrt())
    };
    let residual_rms_m = if residual_rms.is_empty() {
        0.0
    } else {
        (residual_rms.iter().map(|v| v * v).sum::<f64>() / residual_rms.len() as f64).sqrt()
    };
    let (t1, t10, t1cm) = last_conv
        .map(|c| {
            (
                c.time_to_first_meter_s,
                c.time_to_decimeter_s,
                c.time_to_centimeter_s,
            )
        })
        .unwrap_or((None, None, None));
    PppEvaluationReport {
        epochs: solutions.len(),
        horiz_rms_m: horiz_rms,
        vert_rms_m: vert_rms,
        time_to_first_meter_s: t1,
        time_to_decimeter_s: t10,
        time_to_centimeter_s: t1cm,
        residual_rms_m,
    }
}

#[derive(Debug, Serialize)]
struct ReferenceCompareStats {
    count: usize,
    horiz_rms_m: f64,
    vert_rms_m: f64,
}

fn reference_compare(
    solutions: &[bijux_gnss_core::NavSolutionEpoch],
    reference: &[ValidationReferenceEpoch],
) -> (Vec<String>, ReferenceCompareStats) {
    let mut ref_map = std::collections::BTreeMap::new();
    for r in reference {
        ref_map.insert(r.epoch_idx, r);
    }
    let mut rows = Vec::new();
    rows.push("epoch_idx,dx_m,dy_m,dz_m,horiz_m,vert_m".to_string());
    let mut horiz = Vec::new();
    let mut vert = Vec::new();
    for sol in solutions {
        if let Some(r) = ref_map.get(&sol.epoch.index) {
            let (x, y, z) = lla_to_ecef(r.latitude_deg, r.longitude_deg, r.altitude_m);
            let dx = sol.ecef_x_m - x;
            let dy = sol.ecef_y_m - y;
            let dz = sol.ecef_z_m - z;
            let h = (dx * dx + dy * dy).sqrt();
            let v = dz.abs();
            horiz.push(h);
            vert.push(v);
            rows.push(format!(
                "{},{:.4},{:.4},{:.4},{:.4},{:.4}",
                sol.epoch.index, dx, dy, dz, h, v
            ));
        }
    }
    let horiz_rms = if horiz.is_empty() {
        0.0
    } else {
        (horiz.iter().map(|v| v * v).sum::<f64>() / horiz.len() as f64).sqrt()
    };
    let vert_rms = if vert.is_empty() {
        0.0
    } else {
        (vert.iter().map(|v| v * v).sum::<f64>() / vert.len() as f64).sqrt()
    };
    (
        rows,
        ReferenceCompareStats {
            count: horiz.len(),
            horiz_rms_m: horiz_rms,
            vert_rms_m: vert_rms,
        },
    )
}

fn check_solution_consistency(
    solutions: &[bijux_gnss_core::NavSolutionEpoch],
) -> SolutionConsistencyReport {
    let mut position_jump_count = 0;
    let mut clock_jump_count = 0;
    let mut pdop_spike_count = 0;
    let mut warnings = Vec::new();
    let mut prev: Option<&bijux_gnss_core::NavSolutionEpoch> = None;
    let mut prev_pdop: Option<f64> = None;
    for sol in solutions {
        if let Some(prev_sol) = prev {
            let dx = sol.ecef_x_m - prev_sol.ecef_x_m;
            let dy = sol.ecef_y_m - prev_sol.ecef_y_m;
            let dz = sol.ecef_z_m - prev_sol.ecef_z_m;
            let dist = (dx * dx + dy * dy + dz * dz).sqrt();
            if dist > 50.0 {
                position_jump_count += 1;
            }
            let clock_jump = (sol.clock_bias_s - prev_sol.clock_bias_s).abs();
            if clock_jump > 1e-3 {
                clock_jump_count += 1;
            }
        }
        if let Some(prev_pdop) = prev_pdop {
            if sol.pdop > prev_pdop * 2.5 && sol.pdop > 6.0 {
                pdop_spike_count += 1;
            }
        }
        prev_pdop = Some(sol.pdop);
        prev = Some(sol);
    }
    if position_jump_count > 0 {
        warnings.push("position jumps detected".to_string());
    }
    if clock_jump_count > 0 {
        warnings.push("clock jumps detected".to_string());
    }
    if pdop_spike_count > 0 {
        warnings.push("pdop spikes detected".to_string());
    }
    SolutionConsistencyReport {
        position_jump_count,
        clock_jump_count,
        pdop_spike_count,
        warnings,
    }
}

fn check_time_consistency(
    tracks: &[bijux_gnss_receiver::tracking::TrackingResult],
    sample_rate_hz: f64,
) -> TimeConsistencyReport {
    let mut epoch_backward = 0;
    let mut epoch_gaps = 0;
    let mut sample_backward = 0;
    let mut warnings = Vec::new();
    let mut epochs_checked = 0;
    let mut sample_step_mismatch = 0;
    let configured_step = if sample_rate_hz > 0.0 {
        Some((sample_rate_hz * 0.001).round().max(1.0) as u64)
    } else {
        None
    };
    let mut expected_step: Option<u64> = configured_step;
    let mut sample_step_total = 0u64;
    let mut sample_step_count = 0u64;

    for track in tracks {
        let mut prev_epoch: Option<u64> = None;
        let mut prev_sample: Option<u64> = None;
        for epoch in &track.epochs {
            epochs_checked += 1;
            if let Some(prev) = prev_epoch {
                if epoch.epoch.index < prev {
                    epoch_backward += 1;
                } else if epoch.epoch.index > prev + 1 {
                    epoch_gaps += 1;
                }
            }
            if let Some(prev) = prev_sample {
                if epoch.sample_index < prev {
                    sample_backward += 1;
                } else {
                    let step = epoch.sample_index - prev;
                    if let Some(expected) = expected_step {
                        if step != expected {
                            sample_step_mismatch += 1;
                        }
                    } else {
                        expected_step = Some(step);
                    }
                    sample_step_total = sample_step_total.saturating_add(step);
                    sample_step_count = sample_step_count.saturating_add(1);
                }
            }
            prev_epoch = Some(epoch.epoch.index);
            prev_sample = Some(epoch.sample_index);
        }
    }

    if epoch_backward > 0 {
        warnings.push("epoch index went backwards".to_string());
    }
    if epoch_gaps > 0 {
        warnings.push("epoch index gaps detected".to_string());
    }
    if sample_backward > 0 {
        warnings.push("sample index went backwards".to_string());
    }
    if sample_step_mismatch > 0 {
        warnings.push("sample cadence mismatch detected".to_string());
    }

    TimeConsistencyReport {
        channels: tracks.len(),
        epochs_checked,
        epoch_backward,
        epoch_gaps,
        sample_backward,
        sample_step_mismatch,
        expected_step,
        observed_step_mean: if sample_step_count > 0 {
            Some(sample_step_total as f64 / sample_step_count as f64)
        } else {
            None
        },
        warnings,
    }
}

fn stats(values: &[f64]) -> ValidationErrorStats {
    if values.is_empty() {
        return ValidationErrorStats {
            count: 0,
            mean: 0.0,
            median: 0.0,
            rms: 0.0,
            p95: 0.0,
        };
    }
    let mut sorted = values.to_vec();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let count = values.len();
    let mean = values.iter().sum::<f64>() / count as f64;
    let rms = (values.iter().map(|v| v * v).sum::<f64>() / count as f64).sqrt();
    let median = sorted[count / 2];
    let p95 = sorted[(count as f64 * 0.95).floor().min((count - 1) as f64) as usize];
    ValidationErrorStats {
        count,
        mean,
        median,
        rms,
        p95,
    }
}

impl Default for ValidationBudgets {
    fn default() -> Self {
        Self {
            acq_doppler_hz: 500.0,
            acq_code_phase_samples: 5.0,
            tracking_carrier_jitter_hz: 50.0,
            ephemeris_parity_rate_min: 0.9,
            pvt_max_iterations: 10,
        }
    }
}

fn check_budgets(
    tracks: &[bijux_gnss_receiver::tracking::TrackingResult],
    solutions: &[bijux_gnss_core::NavSolutionEpoch],
    budgets: &ValidationBudgets,
) -> Vec<String> {
    let mut violations = Vec::new();
    for track in tracks {
        let mut carriers = Vec::new();
        for e in &track.epochs {
            carriers.push(e.carrier_hz);
        }
        if carriers.len() > 1 {
            let mean = carriers.iter().sum::<f64>() / carriers.len() as f64;
            let var = carriers
                .iter()
                .map(|v| (v - mean) * (v - mean))
                .sum::<f64>()
                / carriers.len() as f64;
            let std = var.sqrt();
            if std > budgets.tracking_carrier_jitter_hz {
                violations.push(format!(
                    "tracking jitter too high for PRN {}: {:.1} Hz",
                    track.sat.prn, std
                ));
            }
        }
    }
    for sol in solutions {
        if sol.residuals.len() >= 4 && sol.rms_m.is_nan() {
            violations.push(format!("PVT RMS is NaN at epoch {}", sol.epoch.index));
        }
    }
    violations
}

fn lla_to_ecef(lat_deg: f64, lon_deg: f64, alt_m: f64) -> (f64, f64, f64) {
    let a = 6378137.0;
    let f = 1.0 / 298.257_223_563;
    let e2 = f * (2.0 - f);
    let lat = lat_deg.to_radians();
    let lon = lon_deg.to_radians();
    let sin_lat = lat.sin();
    let cos_lat = lat.cos();
    let n = a / (1.0 - e2 * sin_lat * sin_lat).sqrt();
    let x = (n + alt_m) * cos_lat * lon.cos();
    let y = (n + alt_m) * cos_lat * lon.sin();
    let z = (n * (1.0 - e2) + alt_m) * sin_lat;
    (x, y, z)
}

fn write_rinex_obs(path: &Path, epochs: &[ObsEpoch], strict: bool) -> Result<()> {
    let mut out = String::new();
    push_rinex_line(
        &mut out,
        "     3.03           OBSERVATION DATA    G                   RINEX VERSION / TYPE",
    );
    push_rinex_line(
        &mut out,
        "BIJUX GNSS                               MARKER NAME",
    );
    push_rinex_line(
        &mut out,
        "G    4 C1C L1C D1C S1C                                      SYS / # / OBS TYPES",
    );
    push_rinex_line(
        &mut out,
        "                                                            END OF HEADER",
    );
    for epoch in epochs {
        let (year, month, day, hour, min, sec) = gps_epoch_to_ymdhms(epoch);
        push_rinex_line(
            &mut out,
            &format!(
                "> {:04} {:02} {:02} {:02} {:02} {:011.7}  0 {:2}",
                year,
                month,
                day,
                hour,
                min,
                sec,
                epoch.sats.len()
            ),
        );
        for sat in &epoch.sats {
            let line = format!(
                "G{:02}{}{}{}{}",
                sat.signal_id.sat.prn,
                format_f14_3(sat.pseudorange_m),
                format_f14_6(sat.carrier_phase_cycles),
                format_f14_3(sat.doppler_hz),
                format_f14_3(sat.cn0_dbhz)
            );
            push_rinex_line(&mut out, &line);
        }
    }
    fs::write(path, out)?;
    if strict {
        validate_rinex_file(path)?;
    }
    Ok(())
}

fn write_rinex_nav(path: &Path, ephs: &[GpsEphemeris], strict: bool) -> Result<()> {
    let mut out = String::new();
    push_rinex_line(
        &mut out,
        "     3.03           NAVIGATION DATA     G                   RINEX VERSION / TYPE",
    );
    push_rinex_line(
        &mut out,
        "                                                            END OF HEADER",
    );
    for eph in ephs {
        let (year, month, day, hour, min, sec) = gps_week_tow_to_ymdhms(eph.week, eph.toc_s);
        let line1 = format!(
            "G{:02} {:04} {:02} {:02} {:02} {:02} {:02}{}{}{}",
            eph.sat.prn,
            year,
            month,
            day,
            hour,
            min,
            sec,
            format_e19_12(eph.af0),
            format_e19_12(eph.af1),
            format_e19_12(eph.af2)
        );
        push_rinex_line(&mut out, &line1);
        let line2 = format!(
            "    {}{}{}{}",
            format_e19_12(eph.iode as f64),
            format_e19_12(eph.crs),
            format_e19_12(eph.delta_n),
            format_e19_12(eph.m0)
        );
        push_rinex_line(&mut out, &line2);
        let line3 = format!(
            "    {}{}{}{}",
            format_e19_12(eph.cuc),
            format_e19_12(eph.e),
            format_e19_12(eph.cus),
            format_e19_12(eph.sqrt_a)
        );
        push_rinex_line(&mut out, &line3);
        let line4 = format!(
            "    {}{}{}{}",
            format_e19_12(eph.toe_s),
            format_e19_12(eph.cic),
            format_e19_12(eph.omega0),
            format_e19_12(eph.cis)
        );
        push_rinex_line(&mut out, &line4);
        let line5 = format!(
            "    {}{}{}{}",
            format_e19_12(eph.i0),
            format_e19_12(eph.crc),
            format_e19_12(eph.w),
            format_e19_12(eph.omegadot)
        );
        push_rinex_line(&mut out, &line5);
        let line6 = format!(
            "    {}{}{}{}",
            format_e19_12(eph.idot),
            format_e19_12(0.0_f64),
            format_e19_12(eph.week as f64),
            format_e19_12(0.0_f64)
        );
        push_rinex_line(&mut out, &line6);
        let line7 = format!(
            "    {}{}{}{}",
            format_e19_12(eph.tgd),
            format_e19_12(eph.iodc as f64),
            format_e19_12(0.0_f64),
            format_e19_12(0.0_f64)
        );
        push_rinex_line(&mut out, &line7);
        push_rinex_line(
            &mut out,
            "    0.000000000000E+00 0.000000000000E+00 0.000000000000E+00 0.000000000000E+00",
        );
    }
    fs::write(path, out)?;
    if strict {
        validate_rinex_file(path)?;
    }
    Ok(())
}

fn push_rinex_line(out: &mut String, line: &str) {
    if line.len() >= 80 {
        out.push_str(&line[..80]);
    } else {
        out.push_str(&format!("{:<80}", line));
    }
    out.push('\n');
}

fn format_e19_12(value: f64) -> String {
    format!("{:>19.12E}", value)
}

fn format_f14_3(value: f64) -> String {
    format!("{:>14.3}", value)
}

fn format_f14_6(value: f64) -> String {
    format!("{:>14.6}", value)
}

fn validate_rinex_file(path: &Path) -> Result<()> {
    let data = fs::read_to_string(path)?;
    for (idx, line) in data.lines().enumerate() {
        if line.len() != 80 {
            bail!("RINEX line length != 80 at {}:{}", path.display(), idx + 1);
        }
    }
    Ok(())
}

fn gps_epoch_to_ymdhms(epoch: &ObsEpoch) -> (i32, u32, u32, u32, u32, f64) {
    let week = epoch.gps_week.unwrap_or(0);
    let tow = epoch.tow_s.unwrap_or(epoch.t_rx_s);
    gps_week_tow_to_ymdhms(week, tow)
}

fn gps_week_tow_to_ymdhms(week: u16, tow_s: f64) -> (i32, u32, u32, u32, u32, f64) {
    let gps_epoch_days = 365 * 10 + 2 + 5; // 1980-01-06 offset in days from 1970-01-01
    let total_days = gps_epoch_days as f64 + week as f64 * 7.0 + (tow_s / 86400.0);
    let (year, month, day) = days_to_ymd(total_days.floor() as i64);
    let sec_of_day = tow_s % 86400.0;
    let hour = (sec_of_day / 3600.0).floor() as u32;
    let min = ((sec_of_day - hour as f64 * 3600.0) / 60.0).floor() as u32;
    let sec = sec_of_day - hour as f64 * 3600.0 - min as f64 * 60.0;
    (year, month, day, hour, min, sec)
}

fn days_to_ymd(days_since_1970: i64) -> (i32, u32, u32) {
    let mut days = days_since_1970;
    let mut year = 1970;
    loop {
        let leap = is_leap(year);
        let year_days = if leap { 366 } else { 365 };
        if days >= year_days {
            days -= year_days;
            year += 1;
        } else {
            break;
        }
    }
    let month_days = [
        31,
        if is_leap(year) { 29 } else { 28 },
        31,
        30,
        31,
        30,
        31,
        31,
        30,
        31,
        30,
        31,
    ];
    let mut month = 1;
    for md in month_days {
        if days >= md {
            days -= md;
            month += 1;
        } else {
            break;
        }
    }
    (year, month as u32, (days + 1) as u32)
}

fn is_leap(year: i32) -> bool {
    (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0)
}

fn schema_path(name: &str) -> PathBuf {
    let base = Path::new(env!("CARGO_MANIFEST_DIR"));
    base.join("../../schemas").join(name)
}

fn validate_json_schema(schema_path: &Path, data_path: &Path, strict: bool) -> Result<()> {
    let schema_data = fs::read_to_string(schema_path)?;
    let schema_json: serde_json::Value = serde_json::from_str(&schema_data)?;
    let compiled = JSONSchema::compile(&schema_json)
        .map_err(|e| eyre!("invalid schema {}: {}", schema_path.display(), e))?;
    let data = fs::read_to_string(data_path)?;
    let json: serde_json::Value = serde_json::from_str(&data)?;
    if strict {
        match &json {
            serde_json::Value::Array(items) if items.is_empty() => {
                bail!("{} is empty", data_path.display());
            }
            _ => {}
        }
    }
    if let Err(errors) = compiled.validate(&json) {
        let mut messages = Vec::new();
        for error in errors {
            messages.push(error.to_string());
        }
        bail!(
            "schema validation failed for {}: {}",
            data_path.display(),
            messages.join(", ")
        );
    }
    Ok(())
}

fn validate_jsonl_schema(schema_path: &Path, data_path: &Path, strict: bool) -> Result<()> {
    let schema_data = fs::read_to_string(schema_path)?;
    let schema_json: serde_json::Value = serde_json::from_str(&schema_data)?;
    let compiled = JSONSchema::compile(&schema_json)
        .map_err(|e| eyre!("invalid schema {}: {}", schema_path.display(), e))?;
    let data = fs::read_to_string(data_path)?;
    let mut count = 0usize;
    for (idx, line) in data.lines().enumerate() {
        if line.trim().is_empty() {
            continue;
        }
        let json: serde_json::Value = serde_json::from_str(line)?;
        count += 1;
        let errors: Vec<String> = match compiled.validate(&json) {
            Ok(()) => Vec::new(),
            Err(errors) => errors.map(|e| e.to_string()).collect(),
        };
        if !errors.is_empty() {
            bail!(
                "schema validation failed for {} line {}: {}",
                data_path.display(),
                idx + 1,
                errors.join(", ")
            );
        }
    }
    if strict && count == 0 {
        bail!("{} is empty", data_path.display());
    }
    Ok(())
}

fn validate_config_schema(profile: &ReceiverProfile) -> Result<()> {
    let schema_path = schema_path("receiver_profile.schema.json");
    let schema_json: serde_json::Value = if schema_path.exists() {
        let file_json: serde_json::Value =
            serde_json::from_str(&fs::read_to_string(&schema_path)?)?;
        if file_json.get("properties").is_some() {
            file_json
        } else {
            serde_json::to_value(schema_for!(ReceiverProfile))?
        }
    } else {
        serde_json::to_value(schema_for!(ReceiverProfile))?
    };
    let compiled = JSONSchema::compile(&schema_json).map_err(|e| eyre!("invalid schema: {}", e))?;
    let json = serde_json::to_value(profile)?;
    if let Err(errors) = compiled.validate(&json) {
        let mut messages = Vec::new();
        for error in errors {
            messages.push(error.to_string());
        }
        bail!("config schema validation failed: {}", messages.join(", "));
    }
    Ok(())
}

fn validate_sidecar_schema(sidecar: &SidecarSpec) -> Result<()> {
    let schema_path = schema_path("sidecar.schema.json");
    let schema_data = fs::read_to_string(schema_path)?;
    let schema_json: serde_json::Value = serde_json::from_str(&schema_data)?;
    let compiled =
        JSONSchema::compile(&schema_json).map_err(|e| eyre!("invalid sidecar schema: {}", e))?;
    let json = serde_json::to_value(sidecar)?;
    if let Err(errors) = compiled.validate(&json) {
        let mut messages = Vec::new();
        for error in errors {
            messages.push(error.to_string());
        }
        bail!("sidecar schema validation failed: {}", messages.join(", "));
    }
    Ok(())
}

#[cfg(feature = "tracing")]
fn init_tracing() {
    let _ = tracing_subscriber::fmt()
        .with_env_filter("info")
        .with_target(false)
        .try_init();
}

#[cfg(not(feature = "tracing"))]
fn init_tracing() {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rinex_obs_has_header() {
        let epoch = ObsEpoch {
            t_rx_s: 0.0,
            gps_week: Some(0),
            tow_s: Some(0.0),
            epoch_idx: 0,
            discontinuity: false,
            role: bijux_gnss_core::ReceiverRole::Rover,
            sats: vec![bijux_gnss_core::ObsSatellite {
                signal_id: bijux_gnss_core::SigId {
                    sat: bijux_gnss_core::SatId {
                        constellation: bijux_gnss_core::Constellation::Gps,
                        prn: 1,
                    },
                    band: bijux_gnss_core::SignalBand::L1,
                    code: bijux_gnss_core::SignalCode::Ca,
                },
                pseudorange_m: 20_000_000.0,
                pseudorange_var_m2: 1.0,
                carrier_phase_cycles: 1000.0,
                carrier_phase_var_cycles2: 0.01,
                doppler_hz: -500.0,
                doppler_var_hz2: 4.0,
                cn0_dbhz: 40.0,
                lock_flags: bijux_gnss_core::LockFlags {
                    code_lock: true,
                    carrier_lock: true,
                    bit_lock: false,
                    cycle_slip: false,
                },
                multipath_suspect: false,
                elevation_deg: None,
                azimuth_deg: None,
                weight: None,
                error_model: None,
                metadata: bijux_gnss_core::ObsMetadata {
                    tracking_mode: "test".to_string(),
                    integration_ms: 1,
                    lock_quality: 1.0,
                    smoothing_window: 0,
                    smoothing_age: 0,
                    smoothing_resets: 0,
                    signal: bijux_gnss_core::SignalSpec {
                        constellation: bijux_gnss_core::Constellation::Gps,
                        band: bijux_gnss_core::SignalBand::L1,
                        code: bijux_gnss_core::SignalCode::Ca,
                        code_rate_hz: 1_023_000.0,
                        carrier_hz: bijux_gnss_core::GPS_L1_CA_CARRIER_HZ,
                    },
                },
            }],
        };
        let path = std::path::Path::new("/tmp/rinex_obs_test.rnx");
        write_rinex_obs(path, &[epoch], true).expect("write obs");
        let data = std::fs::read_to_string(path).expect("read obs");
        assert!(data.contains("RINEX VERSION / TYPE"));
        assert!(data.contains("END OF HEADER"));
        for line in data.lines() {
            assert!(line.len() <= 80, "line too long: {}", line.len());
        }
    }

    #[test]
    fn rinex_nav_has_header() {
        let eph = GpsEphemeris {
            sat: SatId {
                constellation: Constellation::Gps,
                prn: 1,
            },
            iodc: 1,
            iode: 1,
            week: 0,
            toe_s: 0.0,
            toc_s: 0.0,
            sqrt_a: 5153.7954775,
            e: 0.0,
            i0: 0.0,
            idot: 0.0,
            omega0: 0.0,
            omegadot: 0.0,
            w: 0.0,
            m0: 0.0,
            delta_n: 0.0,
            cuc: 0.0,
            cus: 0.0,
            crc: 0.0,
            crs: 0.0,
            cic: 0.0,
            cis: 0.0,
            af0: 0.0,
            af1: 0.0,
            af2: 0.0,
            tgd: 0.0,
        };
        let path = std::path::Path::new("/tmp/rinex_nav_test.rnx");
        write_rinex_nav(path, &[eph], true).expect("write nav");
        let data = std::fs::read_to_string(path).expect("read nav");
        assert!(data.contains("NAVIGATION DATA"));
        assert!(data.contains("END OF HEADER"));
        for line in data.lines() {
            assert!(line.len() <= 80, "line too long: {}", line.len());
        }
    }
}
