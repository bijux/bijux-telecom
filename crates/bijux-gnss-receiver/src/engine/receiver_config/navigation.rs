use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

use bijux_gnss_core::api::Constellation;

/// Receiver constellation-selection policy.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, JsonSchema, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum ConstellationSelectionPolicy {
    /// Allow only GPS.
    GpsOnly,
    /// Allow only Galileo.
    GalileoOnly,
    /// Allow only GLONASS.
    GlonassOnly,
    /// Allow only BeiDou.
    BeidouOnly,
    /// Allow every supported constellation.
    Mixed,
}

impl ConstellationSelectionPolicy {
    /// Whether this policy allows a given constellation.
    pub fn allows(self, constellation: Constellation) -> bool {
        self.selected_constellations().contains(&constellation)
    }

    /// Selected constellations enabled by this policy.
    pub fn selected_constellations(self) -> &'static [Constellation] {
        match self {
            Self::GpsOnly => &[Constellation::Gps],
            Self::GalileoOnly => &[Constellation::Galileo],
            Self::GlonassOnly => &[Constellation::Glonass],
            Self::BeidouOnly => &[Constellation::Beidou],
            Self::Mixed => &[
                Constellation::Gps,
                Constellation::Galileo,
                Constellation::Glonass,
                Constellation::Beidou,
            ],
        }
    }
}

impl Default for ConstellationSelectionPolicy {
    fn default() -> Self {
        Self::Mixed
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, JsonSchema, PartialEq, Eq, Default)]
#[serde(rename_all = "snake_case")]
pub enum NavigationWeightingMode {
    #[default]
    Elevation,
    Cn0,
    ElevationCn0,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, JsonSchema, PartialEq, Eq, Default)]
#[serde(rename_all = "snake_case")]
/// Motion class used to tune navigation position-solution smoothing.
pub enum NavigationMotionClass {
    /// Receiver is expected to remain stationary.
    Static,
    /// Receiver is expected to move at pedestrian speeds.
    Pedestrian,
    /// Receiver is expected to move at ground-vehicle speeds.
    #[default]
    Vehicle,
    /// Receiver is expected to move at airborne speeds.
    Airborne,
}

fn default_position_solution_smoothing() -> bool {
    true
}

/// Navigation configuration parameters.
#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
pub struct NavigationConfig {
    /// Enable robust solver.
    pub robust_solver: bool,
    /// Huber loss parameter.
    pub huber_k: f64,
    /// Enable RAIM-like checks.
    pub raim: bool,
    /// Smooth navigation position solutions across epochs.
    #[serde(default = "default_position_solution_smoothing")]
    pub position_solution_smoothing: bool,
    /// Motion class used to tune navigation position-solution smoothing.
    #[serde(default)]
    pub position_solution_motion_class: NavigationMotionClass,
    /// Hatch smoothing window.
    pub hatch_window: u32,
    /// Weighting configuration.
    pub weighting: NavigationWeightingConfig,
    /// Ionosphere model mode identifier.
    pub iono_mode: String,
    /// Enable troposphere modeling.
    pub tropo_enable: bool,
    /// Default ZTD, in meters.
    pub tropo_ztd_m: f64,
    /// PPP configuration.
    #[serde(default)]
    pub ppp: PppConfig,
    /// Scientific threshold policy configuration.
    #[serde(default)]
    pub science_thresholds: ScienceThresholdsConfig,
    /// Constellation selection policy for receiver acquisition and navigation.
    #[serde(default)]
    pub constellation_policy: ConstellationSelectionPolicy,
}

/// Scientific threshold policy parameters.
#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
pub struct ScienceThresholdsConfig {
    /// Minimum mean C/N0 for accepted navigation solutions.
    pub min_mean_cn0_dbhz: f64,
    /// Maximum PDOP for accepted navigation solutions.
    pub max_pdop: f64,
    /// Maximum GDOP for accepted navigation solutions.
    #[serde(default = "default_science_threshold_max_gdop")]
    pub max_gdop: f64,
    /// Maximum residual RMS (meters) for accepted navigation solutions.
    pub max_residual_rms_m: f64,
    /// Minimum used satellites for accepted navigation solutions.
    pub min_used_satellites: usize,
    /// Minimum lock quality ratio for stable integrity classification.
    pub min_lock_ratio: f64,
}

fn default_science_threshold_max_gdop() -> f64 {
    12.0
}

pub(crate) const DEFAULT_PPP_NOISE_POSITION: f64 = 0.02;
pub(crate) const DEFAULT_PPP_NOISE_VELOCITY: f64 = 0.005;
pub(crate) const DEFAULT_PPP_NOISE_CLOCK_BIAS: f64 = 1e-7;
pub(crate) const DEFAULT_PPP_NOISE_CLOCK_DRIFT: f64 = 1e-5;
pub(crate) const DEFAULT_PPP_NOISE_INTER_SYSTEM_BIAS: f64 = 1e-9;
pub(crate) const DEFAULT_PPP_NOISE_ZTD: f64 = 0.01;
pub(crate) const DEFAULT_PPP_NOISE_IONO: f64 = 0.1;
pub(crate) const DEFAULT_PPP_NOISE_AMBIGUITY: f64 = 0.05;
pub(crate) const DEFAULT_PPP_MEASUREMENT_CODE_FLOOR_M: f64 = 0.3;
pub(crate) const DEFAULT_PPP_MEASUREMENT_PHASE_FLOOR_CYCLES: f64 = 0.01;
pub(crate) const DEFAULT_PPP_MEASUREMENT_ORBIT_SIGMA_SCALE: f64 = 1.0;
pub(crate) const DEFAULT_PPP_MEASUREMENT_CLOCK_SIGMA_SCALE: f64 = 1.0;
pub(crate) const DEFAULT_PPP_MEASUREMENT_TROPOSPHERE_RESIDUAL_M: f64 = 0.05;
pub(crate) const DEFAULT_PPP_MEASUREMENT_ANTENNA_RESIDUAL_M: f64 = 0.01;
pub(crate) const DEFAULT_PPP_PRECISE_PRODUCT_BRIDGE_ACTION: &str = "bridge_with_broadcast";
pub(crate) const DEFAULT_PPP_PRECISE_PRODUCT_RESET_ACTION: &str = "reset_satellite_state";
pub(crate) const DEFAULT_PPP_PRECISE_PRODUCT_REFUSE_ACTION: &str = "refuse_satellite";
pub(crate) const DEFAULT_PPP_PRECISE_PRODUCT_STATE_INFLATION: f64 = 100.0;

fn default_ppp_noise_position() -> f64 {
    DEFAULT_PPP_NOISE_POSITION
}

fn default_ppp_noise_velocity() -> f64 {
    DEFAULT_PPP_NOISE_VELOCITY
}

fn default_ppp_noise_clock_bias() -> f64 {
    DEFAULT_PPP_NOISE_CLOCK_BIAS
}

fn default_ppp_noise_clock_drift() -> f64 {
    DEFAULT_PPP_NOISE_CLOCK_DRIFT
}

fn default_ppp_noise_inter_system_bias() -> f64 {
    DEFAULT_PPP_NOISE_INTER_SYSTEM_BIAS
}

fn default_ppp_noise_ztd() -> f64 {
    DEFAULT_PPP_NOISE_ZTD
}

fn default_ppp_noise_iono() -> f64 {
    DEFAULT_PPP_NOISE_IONO
}

fn default_ppp_noise_ambiguity() -> f64 {
    DEFAULT_PPP_NOISE_AMBIGUITY
}

fn default_ppp_measurement_code_floor_m() -> f64 {
    DEFAULT_PPP_MEASUREMENT_CODE_FLOOR_M
}

fn default_ppp_measurement_phase_floor_cycles() -> f64 {
    DEFAULT_PPP_MEASUREMENT_PHASE_FLOOR_CYCLES
}

fn default_ppp_measurement_orbit_sigma_scale() -> f64 {
    DEFAULT_PPP_MEASUREMENT_ORBIT_SIGMA_SCALE
}

fn default_ppp_measurement_clock_sigma_scale() -> f64 {
    DEFAULT_PPP_MEASUREMENT_CLOCK_SIGMA_SCALE
}

fn default_ppp_measurement_troposphere_residual_m() -> f64 {
    DEFAULT_PPP_MEASUREMENT_TROPOSPHERE_RESIDUAL_M
}

fn default_ppp_measurement_antenna_residual_m() -> f64 {
    DEFAULT_PPP_MEASUREMENT_ANTENNA_RESIDUAL_M
}

fn default_ppp_precise_product_bridge_action() -> String {
    DEFAULT_PPP_PRECISE_PRODUCT_BRIDGE_ACTION.to_string()
}

fn default_ppp_precise_product_reset_action() -> String {
    DEFAULT_PPP_PRECISE_PRODUCT_RESET_ACTION.to_string()
}

fn default_ppp_precise_product_refuse_action() -> String {
    DEFAULT_PPP_PRECISE_PRODUCT_REFUSE_ACTION.to_string()
}

fn default_ppp_precise_product_state_inflation() -> f64 {
    DEFAULT_PPP_PRECISE_PRODUCT_STATE_INFLATION
}

/// PPP configuration parameters.
#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
pub struct PppConfig {
    /// Enable PPP processing.
    pub enabled: bool,
    /// Use ionosphere-free combinations.
    pub use_iono_free: bool,
    /// Use Doppler measurements.
    pub use_doppler: bool,
    /// Enable ionosphere state estimation.
    pub enable_iono_state: bool,
    /// Ambiguity resolution mode.
    pub ar_mode: String,
    /// Ambiguity ratio test threshold.
    pub ar_ratio_threshold: f64,
    /// Consecutive epochs required for AR acceptance.
    pub ar_stability_epochs: u32,
    /// Maximum satellites to attempt AR on.
    pub ar_max_sats: usize,
    /// Prefer elevation-based selection.
    pub ar_use_elevation: bool,
    /// Prune ambiguity states after this many epochs.
    pub prune_after_epochs: u64,
    /// Reset PPP state after a gap of this many seconds.
    pub reset_gap_s: f64,
    /// Residual gate threshold, in meters.
    pub residual_gate_m: f64,
    /// Drift detection window, in epochs.
    pub drift_window_epochs: u64,
    /// Drift detection threshold, in meters.
    pub drift_threshold_m: f64,
    /// Checkpoint interval, in epochs.
    pub checkpoint_interval_epochs: u64,
    /// Receiver antenna type used to select ANTEX phase-center corrections.
    #[serde(default)]
    pub receiver_antenna_type: Option<String>,
    /// Process noise for receiver position.
    #[serde(default = "default_ppp_noise_position")]
    pub noise_position: f64,
    /// Process noise for receiver velocity.
    #[serde(default = "default_ppp_noise_velocity")]
    pub noise_velocity: f64,
    /// Process noise for receiver clock bias.
    #[serde(default = "default_ppp_noise_clock_bias")]
    pub noise_clock_bias: f64,
    /// Process noise for clock drift.
    #[serde(default = "default_ppp_noise_clock_drift")]
    pub noise_clock_drift: f64,
    /// Process noise for constellation clock offsets.
    #[serde(default = "default_ppp_noise_inter_system_bias")]
    pub noise_inter_system_bias: f64,
    /// Process noise for ZTD.
    #[serde(default = "default_ppp_noise_ztd")]
    pub noise_ztd: f64,
    /// Process noise for ionosphere.
    #[serde(default = "default_ppp_noise_iono")]
    pub noise_iono: f64,
    /// Process noise for ambiguities.
    #[serde(default = "default_ppp_noise_ambiguity")]
    pub noise_ambiguity: f64,
    /// PPP code measurement noise floor, in meters.
    #[serde(default = "default_ppp_measurement_code_floor_m")]
    pub measurement_code_floor_m: f64,
    /// PPP phase measurement noise floor, in cycles.
    #[serde(default = "default_ppp_measurement_phase_floor_cycles")]
    pub measurement_phase_floor_cycles: f64,
    /// Scale factor applied to satellite orbit uncertainty.
    #[serde(default = "default_ppp_measurement_orbit_sigma_scale")]
    pub measurement_orbit_sigma_scale: f64,
    /// Scale factor applied to satellite clock uncertainty.
    #[serde(default = "default_ppp_measurement_clock_sigma_scale")]
    pub measurement_clock_sigma_scale: f64,
    /// Residual zenith troposphere uncertainty, in meters.
    #[serde(default = "default_ppp_measurement_troposphere_residual_m")]
    pub measurement_troposphere_residual_m: f64,
    /// Residual antenna correction uncertainty, in meters.
    #[serde(default = "default_ppp_measurement_antenna_residual_m")]
    pub measurement_antenna_residual_m: f64,
    /// Action applied when precise products omit a satellite.
    #[serde(default = "default_ppp_precise_product_bridge_action")]
    pub precise_product_missing_satellite_action: String,
    /// Action applied when precise products do not cover the requested epoch.
    #[serde(default = "default_ppp_precise_product_bridge_action")]
    pub precise_product_out_of_coverage_action: String,
    /// Action applied when precise products have too little interpolation support.
    #[serde(default = "default_ppp_precise_product_bridge_action")]
    pub precise_product_insufficient_support_action: String,
    /// Action applied when precise orbit products contain an interpolation gap.
    #[serde(default = "default_ppp_precise_product_reset_action")]
    pub precise_product_orbit_gap_action: String,
    /// Action applied when precise orbit products flag an unusable record.
    #[serde(default = "default_ppp_precise_product_refuse_action")]
    pub precise_product_orbit_flag_action: String,
    /// Action applied when precise clock products contain an interpolation gap.
    #[serde(default = "default_ppp_precise_product_reset_action")]
    pub precise_product_clock_gap_action: String,
    /// Action applied when precise clock products contain a clock jump.
    #[serde(default = "default_ppp_precise_product_reset_action")]
    pub precise_product_clock_jump_action: String,
    /// Covariance multiplier for satellite-scoped PPP state inflation.
    #[serde(default = "default_ppp_precise_product_state_inflation")]
    pub precise_product_state_inflation: f64,
    /// Minimum convergence time, in seconds.
    pub convergence_min_time_s: f64,
    /// Convergence position rate threshold, in m/s.
    pub convergence_pos_rate_mps: f64,
    /// Horizontal sigma threshold for convergence, in meters.
    pub convergence_sigma_h_m: f64,
    /// Vertical sigma threshold for convergence, in meters.
    pub convergence_sigma_v_m: f64,
}

/// Navigation measurement weighting parameters.
#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
pub struct NavigationWeightingConfig {
    /// Enable weighting.
    pub enabled: bool,
    /// Measurement weighting model.
    #[serde(default)]
    pub mode: NavigationWeightingMode,
    /// Minimum elevation, in degrees.
    pub min_elev_deg: f64,
    /// Elevation exponent for weighting.
    pub elev_exponent: f64,
    /// Reference C/N0, in dB-Hz.
    pub cn0_ref_dbhz: f64,
    /// Minimum weight floor.
    pub min_weight: f64,
    /// Elevation mask, in degrees.
    pub elev_mask_deg: f64,
    /// Scalar tracking mode weight.
    pub tracking_mode_scalar_weight: f64,
    /// Vector tracking mode weight.
    pub tracking_mode_vector_weight: f64,
}
