/// Summary of one observation-error distribution.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticObservationErrorStats {
    /// Count of comparable observation rows.
    pub count: usize,
    /// Signed mean error.
    pub mean_error: f64,
    /// Median absolute error.
    pub median_abs_error: f64,
    /// Root-mean-square error magnitude.
    pub rms_error: f64,
    /// 95th percentile absolute error.
    pub p95_abs_error: f64,
    /// Maximum absolute error.
    pub max_abs_error: f64,
}

/// Per-satellite observation truth comparison summary.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticObservationValidationSatellite {
    /// Satellite identifier.
    pub sat: SatId,
    /// Aggregated pseudorange error statistics in meters.
    pub pseudorange_error_m: Option<SyntheticObservationErrorStats>,
    /// Aggregated ambiguity-aligned carrier-phase residual statistics in cycles.
    pub carrier_phase_error_cycles: Option<SyntheticObservationErrorStats>,
    /// Number of carrier-phase arcs aligned before residual aggregation.
    pub carrier_phase_arcs_evaluated: usize,
    /// Aggregated Doppler error statistics in Hz.
    pub doppler_error_hz: Option<SyntheticObservationErrorStats>,
    /// Aggregated C/N0 error statistics in dB-Hz.
    pub cn0_error_db_hz: Option<SyntheticObservationErrorStats>,
    /// Reasons a metric could not be evaluated for this satellite.
    pub notes: Vec<String>,
}

/// Truth-guided observation validation report for a synthetic scenario.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticObservationValidationReport {
    /// Stable scenario identifier for this validation run.
    pub scenario_id: String,
    /// Capture sample rate in Hz.
    pub sample_rate_hz: f64,
    /// Hatch-smoothing window applied before observation comparison.
    pub hatch_window: u32,
    /// Absolute receive-time anchor used for geometric pseudorange truth.
    pub reference_receive_time_s: f64,
    /// Per-satellite observation truth summaries.
    pub satellites: Vec<SyntheticObservationValidationSatellite>,
}

