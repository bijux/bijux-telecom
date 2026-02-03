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
    checkpoint_path: Option<String>,
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

