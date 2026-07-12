/// Absolute synthetic reference used when comparing emitted observations against truth.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct SyntheticObservationTruthReference {
    /// Absolute receiver receive time at capture sample zero, in seconds.
    pub receive_time_s: f64,
    /// Receiver truth position in ECEF meters.
    pub receiver_ecef_m: [f64; 3],
    /// Optional first-order ionosphere model applied when deriving signal-specific code truth.
    #[serde(default)]
    pub ionosphere_delay_model: Option<SyntheticIonosphereDelayModel>,
}

/// Truth-guided value recorded for one observation observable.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticObservationTruthTableValue {
    /// Expected synthetic truth value for this observable.
    pub truth: Option<f64>,
    /// Measured receiver value for this observable.
    pub measured: f64,
    /// Reported one-sigma uncertainty for this observable, when the receiver exposes one.
    pub sigma: Option<f64>,
    /// Signed residual between the measured value and the truth value, when comparable.
    pub residual: Option<f64>,
}

/// Per-epoch observation truth-table row for synthetic validation.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticObservationTruthTableEpoch {
    /// Stable artifact identifier for the source observation epoch.
    pub artifact_id: String,
    /// Stable identity key for the source observation epoch.
    pub epoch_id: String,
    /// Receiver observation epoch index.
    pub epoch_index: u64,
    /// Absolute sample index at the start of the observation epoch.
    pub sample_index: u64,
    /// Observation status reported by the receiver.
    pub observation_status: ObservationStatus,
    /// Receiver reasons for degraded or rejected observation handling.
    pub observation_reject_reasons: Vec<String>,
    /// Pseudorange truth row in meters.
    pub pseudorange_m: SyntheticObservationTruthTableValue,
    /// Carrier-phase truth row in cycles.
    pub carrier_phase_cycles: SyntheticObservationTruthTableValue,
    /// Carrier-phase arc anchor for ambiguity alignment, when one exists.
    pub carrier_phase_arc_start_sample_index: Option<u64>,
    /// Ambiguity bias removed from the carrier-phase truth comparison, in cycles.
    pub carrier_phase_arc_bias_cycles: Option<f64>,
    /// Doppler truth row in Hz.
    pub doppler_hz: SyntheticObservationTruthTableValue,
    /// C/N0 truth row in dB-Hz.
    pub cn0_db_hz: SyntheticObservationTruthTableValue,
}

/// Per-satellite observation truth table for synthetic validation.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticObservationTruthTableSatellite {
    /// Satellite identifier.
    pub sat: SatId,
    /// Full signal identifier for this truth row.
    pub signal_id: SigId,
    /// Injected Doppler shift in Hz.
    pub injected_doppler_hz: f64,
    /// Expected measured Doppler after the receiver clock bias is applied, in Hz.
    pub expected_measured_doppler_hz: f64,
    /// Injected code phase at sample zero, in chips.
    pub injected_code_phase_chips: f64,
    /// Injected carrier-to-noise density ratio in dB-Hz.
    pub injected_cn0_db_hz: f32,
    /// Number of truth-table rows recorded for this satellite.
    pub epoch_count: usize,
    /// Number of carrier-phase arcs aligned before residual evaluation.
    pub carrier_phase_arcs_evaluated: usize,
    /// Reasons one or more observables could not be compared against synthetic truth.
    pub notes: Vec<String>,
    /// Per-epoch truth-table rows.
    pub epochs: Vec<SyntheticObservationTruthTableEpoch>,
}

/// Truth-guided observation truth table for a synthetic scenario.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticObservationTruthTableReport {
    /// Stable scenario identifier for this validation run.
    pub scenario_id: String,
    /// Capture sample rate in Hz.
    pub sample_rate_hz: f64,
    /// Hatch-smoothing window applied before observation comparison.
    pub hatch_window: u32,
    /// Absolute receive-time anchor used for geometric pseudorange truth.
    pub reference_receive_time_s: f64,
    /// Per-satellite truth-table rows.
    pub satellites: Vec<SyntheticObservationTruthTableSatellite>,
}

/// Synthetic truth reference for one PVT solution epoch.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SyntheticPvtTruthReferenceEpoch {
    /// Reference position and time for this navigation epoch.
    pub position: ValidationReferenceEpoch,
    /// Receiver clock bias truth in seconds for this navigation epoch.
    pub clock_bias_s: f64,
}

/// ECEF coordinates recorded in a synthetic PVT truth table.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct SyntheticPvtTruthTableEcef {
    /// X coordinate in meters.
    pub x_m: f64,
    /// Y coordinate in meters.
    pub y_m: f64,
    /// Z coordinate in meters.
    pub z_m: f64,
}

/// Geodetic coordinates recorded in a synthetic PVT truth table.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct SyntheticPvtTruthTableGeodetic {
    /// Latitude in degrees.
    pub latitude_deg: f64,
    /// Longitude in degrees.
    pub longitude_deg: f64,
    /// Altitude above the ellipsoid in meters.
    pub altitude_m: f64,
}

/// ENU error components recorded in a synthetic PVT truth table.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct SyntheticPvtTruthTableEnuError {
    /// East error in meters.
    pub east_m: f64,
    /// North error in meters.
    pub north_m: f64,
    /// Up error in meters.
    pub up_m: f64,
    /// Horizontal error magnitude in meters.
    pub horiz_m: f64,
    /// Vertical error magnitude in meters.
    pub vert_m: f64,
    /// 3D position error magnitude in meters.
    pub error_3d_m: f64,
}

/// Receiver clock-bias truth comparison recorded in a synthetic PVT truth table.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct SyntheticPvtTruthTableClockBias {
    /// Truth clock bias in seconds.
    pub truth_s: f64,
    /// Measured clock bias in seconds.
    pub measured_s: f64,
    /// Measured minus truth clock bias in seconds.
    pub error_s: f64,
    /// Truth clock bias in meters.
    pub truth_m: f64,
    /// Measured clock bias in meters.
    pub measured_m: f64,
    /// Measured minus truth clock bias in meters.
    pub error_m: f64,
}

/// DOP values recorded in a synthetic PVT truth table.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct SyntheticPvtTruthTableDop {
    /// Position dilution of precision.
    pub pdop: f64,
    /// Horizontal dilution of precision.
    pub hdop: Option<f64>,
    /// Vertical dilution of precision.
    pub vdop: Option<f64>,
    /// Geometric dilution of precision.
    pub gdop: Option<f64>,
    /// Time dilution of precision.
    pub tdop: Option<f64>,
}

/// Per-epoch synthetic PVT truth-table row.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticPvtTruthTableEpoch {
    /// Stable artifact identifier for the navigation solution epoch.
    pub artifact_id: String,
    /// Source observation epoch identifier used to produce this navigation solution.
    pub source_observation_epoch_id: String,
    /// Receiver navigation epoch index.
    pub epoch_index: u64,
    /// Receiver receive time for this solution, in seconds.
    pub receive_time_s: f64,
    /// Truth ECEF position.
    pub truth_ecef_m: SyntheticPvtTruthTableEcef,
    /// Measured ECEF position.
    pub measured_ecef_m: SyntheticPvtTruthTableEcef,
    /// Measured minus truth ECEF error, in meters.
    pub ecef_error_m: SyntheticPvtTruthTableEcef,
    /// Truth geodetic position.
    pub truth_geodetic: SyntheticPvtTruthTableGeodetic,
    /// Measured geodetic position.
    pub measured_geodetic: SyntheticPvtTruthTableGeodetic,
    /// ENU error components.
    pub enu_error_m: SyntheticPvtTruthTableEnuError,
    /// Clock-bias truth comparison.
    pub clock_bias: SyntheticPvtTruthTableClockBias,
    /// Residual RMS after the navigation solve, in meters.
    pub residual_rms_m: f64,
    /// Pre-fit residual RMS, in meters, when available.
    pub pre_fit_residual_rms_m: Option<f64>,
    /// Post-fit residual RMS, in meters, when available.
    pub post_fit_residual_rms_m: Option<f64>,
    /// Dilution-of-precision values reported for this solution.
    pub dop: SyntheticPvtTruthTableDop,
    /// Receiver solution status.
    pub solution_status: SolutionStatus,
    /// Receiver solution quality classification.
    pub solution_quality: NavQualityFlag,
    /// Receiver solution validity classification.
    pub solution_validity: SolutionValidity,
    /// Whether the receiver marked this navigation solution valid.
    pub valid: bool,
    /// Number of satellites carried in the solution artifact.
    pub sat_count: usize,
    /// Number of satellites used in the final solve.
    pub used_sat_count: usize,
    /// Number of rejected satellites reported by the receiver.
    pub rejected_sat_count: usize,
}

/// Truth-guided PVT truth table for a synthetic navigation run.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticPvtTruthTableReport {
    /// Stable scenario identifier for this validation run.
    pub scenario_id: String,
    /// Count of navigation solutions supplied to the builder.
    pub solution_count: usize,
    /// Count of rows with matching synthetic truth.
    pub matched_epoch_count: usize,
    /// Solution epochs that had no matching synthetic truth reference.
    pub unmatched_solution_epochs: Vec<u64>,
    /// Truth epochs that were not consumed by any navigation solution row.
    pub unused_reference_epochs: Vec<u64>,
    /// Per-epoch PVT truth-table rows.
    pub epochs: Vec<SyntheticPvtTruthTableEpoch>,
}
