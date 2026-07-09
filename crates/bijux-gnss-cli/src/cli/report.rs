#[derive(Debug, Serialize)]
struct AcquisitionReport {
    sats: Vec<SatId>,
    front_end_metrics: bijux_gnss_infra::api::signal::IqFrontEndMetrics,
    reported_prns: Vec<ReportedPrn>,
    results: Vec<AcquisitionRow>,
}

#[derive(Debug, Serialize, Clone)]
struct AcquisitionRow {
    sat: SatId,
    carrier_hz: f64,
    code_phase_samples: usize,
    peak: f32,
    peak_mean_ratio: f32,
    peak_second_ratio: f32,
    hypothesis: String,
    selection_reason: Option<String>,
}

#[derive(Debug, Serialize, Clone)]
struct ReportedPrn {
    sat: SatId,
    classification: String,
    carrier_hz: f64,
    peak_mean_ratio: f32,
    peak_second_ratio: f32,
}

fn summarize_reported_prns(rows: &[AcquisitionRow]) -> Vec<ReportedPrn> {
    let mut by_prn = std::collections::BTreeMap::<SatId, ReportedPrn>::new();
    for row in rows {
        let classification = match row.hypothesis.as_str() {
            "accepted" => "accepted",
            "ambiguous" => "candidate",
            _ => continue,
        };
        let candidate = ReportedPrn {
            sat: row.sat,
            classification: classification.to_string(),
            carrier_hz: row.carrier_hz,
            peak_mean_ratio: row.peak_mean_ratio,
            peak_second_ratio: row.peak_second_ratio,
        };
        match by_prn.get(&row.sat) {
            Some(existing)
                if reported_prn_sort_key(existing) <= reported_prn_sort_key(&candidate) => {}
            _ => {
                by_prn.insert(row.sat, candidate);
            }
        }
    }
    let mut reported_prns: Vec<_> = by_prn.into_values().collect();
    reported_prns
        .sort_by(|left, right| reported_prn_sort_key(left).cmp(&reported_prn_sort_key(right)));
    reported_prns
}

fn reported_prn_sort_key(entry: &ReportedPrn) -> (u8, std::cmp::Reverse<u32>, u8) {
    let class_rank = if entry.classification == "accepted" { 0 } else { 1 };
    (class_rank, std::cmp::Reverse(entry.peak_mean_ratio.to_bits()), entry.sat.prn)
}

#[derive(Debug, Serialize)]
struct InspectReport {
    format: String,
    sample_rate_hz: f64,
    intermediate_freq_hz: f64,
    capture_start_utc: String,
    total_samples: usize,
    usable_duration_s: f64,
    front_end_metrics: bijux_gnss_infra::api::signal::IqFrontEndMetrics,
    noise_floor_db: f64,
    power_histogram: Vec<u64>,
    signal_quality: RawIqSignalQualityReport,
}

#[derive(Debug, Serialize, Deserialize, Clone)]
struct RawIqSignalQualityReport {
    format: String,
    sample_rate_hz: f64,
    intermediate_freq_hz: f64,
    capture_start_utc: String,
    analyzed_samples: usize,
    usable_duration_s: f64,
    estimated_noise_floor_db: f64,
    front_end_metrics: bijux_gnss_infra::api::signal::IqFrontEndMetrics,
}

#[derive(Debug, Serialize)]
struct StreamingRunReport {
    epochs: u64,
    processed_input_samples: u64,
    acquisitions: usize,
    tracked_channels: usize,
    observation_epochs: usize,
    front_end_metrics: bijux_gnss_infra::api::signal::IqFrontEndMetrics,
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

#[derive(Debug, Serialize)]
struct SyntheticIqExportReport {
    scenario_id: String,
    seed: u64,
    sample_count: usize,
    sample_rate_hz: f64,
    output_iq: String,
    output_sidecar: String,
    output_truth: String,
    satellites: Vec<SatId>,
}

#[derive(Debug, Serialize)]
struct SyntheticIqValidationReport {
    input_iq: String,
    input_sidecar: String,
    input_truth: String,
    validation: bijux_gnss_infra::api::receiver::sim::SyntheticCn0ValidationReport,
}

#[cfg(test)]
mod report_tests {
    use super::*;
    use bijux_gnss_infra::api::core::Constellation;

    fn gps_row(prn: u8, hypothesis: &str, peak_mean_ratio: f32) -> AcquisitionRow {
        AcquisitionRow {
            sat: SatId { constellation: Constellation::Gps, prn },
            carrier_hz: 500.0 * prn as f64,
            code_phase_samples: 42,
            peak: peak_mean_ratio * 10.0,
            peak_mean_ratio,
            peak_second_ratio: 1.1,
            hypothesis: hypothesis.to_string(),
            selection_reason: None,
        }
    }

    #[test]
    fn summarize_reported_prns_prefers_accepted_rows_and_filters_rejections() {
        let rows = vec![
            gps_row(31, "ambiguous", 8.0),
            gps_row(31, "accepted", 7.0),
            gps_row(12, "ambiguous", 6.0),
            gps_row(25, "deferred", 9.0),
            gps_row(32, "rejected", 10.0),
        ];

        let reported_prns = summarize_reported_prns(&rows);

        assert_eq!(reported_prns.len(), 2);
        assert_eq!(reported_prns[0].sat.prn, 31);
        assert_eq!(reported_prns[0].classification, "accepted");
        assert_eq!(reported_prns[1].sat.prn, 12);
        assert_eq!(reported_prns[1].classification, "candidate");
    }
}

struct EkfContext {
    ekf: bijux_gnss_infra::api::nav::Ekf,
    model: NavClockModel,
    last_t_rx_s: Option<f64>,
    ambiguity: bijux_gnss_infra::api::nav::AmbiguityManager,
    isb: bijux_gnss_infra::api::nav::InterSystemBiasManager,
    ztd_index: Option<usize>,
    atmosphere: bijux_gnss_infra::api::nav::AtmosphereConfig,
    code_bias: bijux_gnss_infra::api::nav::ZeroBiases,
    phase_bias: bijux_gnss_infra::api::nav::ZeroBiases,
    corrections: bijux_gnss_infra::api::nav::CorrectionContext,
}

impl EkfContext {
    fn new() -> Self {
        let x = vec![0.0_f64; 8];
        let p = Matrix::identity(8);
        let mut ekf = bijux_gnss_infra::api::nav::Ekf::new(
            x,
            p,
            bijux_gnss_infra::api::nav::EkfConfig {
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
            ambiguity: bijux_gnss_infra::api::nav::AmbiguityManager::new(),
            isb: bijux_gnss_infra::api::nav::InterSystemBiasManager::new(),
            ztd_index,
            atmosphere: bijux_gnss_infra::api::nav::AtmosphereConfig::default(),
            code_bias: bijux_gnss_infra::api::nav::ZeroBiases,
            phase_bias: bijux_gnss_infra::api::nav::ZeroBiases,
            corrections: bijux_gnss_infra::api::nav::CorrectionContext,
        }
    }
}

#[derive(Debug, Serialize, Deserialize)]
struct TrackingReport {
    sats: Vec<SatId>,
    front_end_metrics: bijux_gnss_infra::api::signal::IqFrontEndMetrics,
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
    #[serde(default)]
    early_i: f32,
    #[serde(default)]
    early_q: f32,
    #[serde(default)]
    late_i: f32,
    #[serde(default)]
    late_q: f32,
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
    #[serde(default)]
    anti_false_lock: bool,
    #[serde(default)]
    cycle_slip_reason: Option<String>,
    #[serde(default)]
    lock_state: String,
    #[serde(default)]
    lock_state_reason: Option<String>,
}

#[derive(Debug, Serialize)]
struct NavDecodeReport {
    sat: SatId,
    preamble_hits: usize,
    parity_pass_rate: f64,
    ephemerides: Vec<bijux_gnss_infra::api::nav::GpsEphemeris>,
}
