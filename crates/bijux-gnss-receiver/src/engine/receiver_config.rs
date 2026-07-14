use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

use bijux_gnss_core::api::{
    AcqError, ConfigError, Constellation, InputError, NavError, SchemaVersion, SignalBand,
    SignalError, TrackError,
};
use bijux_gnss_signal::api::FrontEndFilterSpec;
use thiserror::Error;

/// On-disk receiver configuration.
///
/// For a combined receiver+navigation config, see `bijux_gnss_core::api::BijuxGnssConfig`.
#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
pub struct ReceiverConfig {
    /// Schema version for config compatibility.
    #[schemars(with = "u32")]
    pub schema_version: SchemaVersion,
    /// Sample rate of the IF signal, in Hz.
    pub sample_rate_hz: f64,
    /// Intermediate frequency, in Hz.
    pub intermediate_freq_hz: f64,
    /// Quantization bits per sample.
    pub quantization_bits: u8,
    /// Code frequency basis, in Hz.
    pub code_freq_basis_hz: f64,
    /// Code length in chips.
    pub code_length: usize,
    /// Front-end conditioning configuration.
    #[serde(default)]
    pub front_end: FrontEndConfig,
    /// RNG seed for deterministic operations.
    pub seed: u64,
    /// Acquisition configuration.
    pub acquisition: AcquisitionConfig,
    /// Tracking configuration.
    pub tracking: TrackingConfig,
    /// Navigation configuration.
    pub navigation: NavigationConfig,
}

/// Derived receiver configuration used at runtime.
#[derive(Debug, Clone)]
pub struct ReceiverPipelineConfig {
    /// Sample rate of the IF signal, in Hz.
    pub sampling_freq_hz: f64,
    /// Intermediate frequency, in Hz.
    pub intermediate_freq_hz: f64,
    /// Whether mean I/Q offset should be removed before acquisition consumes a frame.
    pub remove_dc_offset: bool,
    /// Optional complex front-end FIR filter applied before acquisition and tracking consume input.
    pub front_end_filter: Option<FrontEndFilterSpec>,
    /// Code frequency basis, in Hz.
    pub code_freq_basis_hz: f64,
    /// Code length in chips.
    pub code_length: usize,
    /// Maximum tracking channels.
    pub channels: usize,
    /// Doppler search range used by acquisition, in Hz.
    pub acquisition_doppler_search_hz: i32,
    /// Doppler bin spacing used by acquisition, in Hz.
    pub acquisition_doppler_step_hz: i32,
    /// Doppler-rate search range used by acquisition, in Hz/s.
    pub acquisition_doppler_rate_search_hz_per_s: i32,
    /// Doppler-rate bin spacing used by acquisition, in Hz/s.
    pub acquisition_doppler_rate_step_hz_per_s: i32,
    /// Coherent integration used by acquisition, in milliseconds.
    pub acquisition_integration_ms: u32,
    /// Noncoherent integration used by acquisition.
    pub acquisition_noncoherent: u32,
    /// Minimum accepted peak-to-mean acquisition ratio.
    pub acquisition_peak_mean_threshold: f32,
    /// Minimum accepted peak-to-second-peak acquisition ratio.
    pub acquisition_peak_second_threshold: f32,
    /// Acquisition threshold policy configuration.
    pub acquisition_threshold_policy: AcquisitionThresholdPolicyConfig,
    /// Default early/late spacing, in chips.
    pub early_late_spacing_chips: f64,
    /// DLL noise bandwidth, in Hz.
    pub dll_bw_hz: f64,
    /// PLL noise bandwidth, in Hz.
    pub pll_bw_hz: f64,
    /// FLL noise bandwidth, in Hz.
    pub fll_bw_hz: f64,
    /// Whether tracking loop and coherent integration adaptation are enabled.
    pub adaptive_tracking_enabled: bool,
    /// Integration time for tracking, in milliseconds.
    pub tracking_integration_ms: u32,
    /// Target per-epoch budget, in milliseconds.
    pub tracking_budget_ms: f64,
    /// Action when budget exceeded (drop_epochs or continue).
    pub tracking_over_budget_action: String,
    /// Per-band tracking overrides.
    pub tracking_per_band: Vec<BandTrackingSpec>,
    /// Whether to use a robust navigation solver.
    pub robust_solver: bool,
    /// Huber loss parameter for robust solver.
    pub huber_k: f64,
    /// Whether RAIM-like checks are enabled.
    pub raim: bool,
    /// Whether navigation position solutions should be smoothed across epochs.
    pub position_solution_smoothing: bool,
    /// Receiver motion class used to tune position-solution smoothing.
    pub position_solution_motion_class: NavigationMotionClass,
    /// Hatch smoothing window, in epochs.
    pub hatch_window: u32,
    /// Navigation weighting configuration.
    pub weighting: NavigationWeightingConfig,
    /// Ionosphere model mode identifier.
    pub iono_mode: String,
    /// Whether to enable troposphere modeling.
    pub tropo_enable: bool,
    /// Default zenith tropospheric delay, in meters.
    pub tropo_ztd_m: f64,
    /// PPP configuration.
    pub ppp: PppConfig,
    /// Scientific threshold policy configuration.
    pub science_thresholds: ScienceThresholdsConfig,
    /// Constellation selection policy for acquisition and navigation.
    pub constellation_policy: ConstellationSelectionPolicy,
}

impl Default for ReceiverPipelineConfig {
    fn default() -> Self {
        Self {
            sampling_freq_hz: 5_000_000.0,
            intermediate_freq_hz: 0.0,
            remove_dc_offset: false,
            front_end_filter: None,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            channels: 12,
            acquisition_doppler_search_hz: 10_000,
            acquisition_doppler_step_hz: 500,
            acquisition_doppler_rate_search_hz_per_s: 0,
            acquisition_doppler_rate_step_hz_per_s: 250,
            acquisition_integration_ms: 1,
            acquisition_noncoherent: 1,
            acquisition_peak_mean_threshold: 2.5,
            acquisition_peak_second_threshold: 1.5,
            acquisition_threshold_policy: AcquisitionThresholdPolicyConfig::default(),
            early_late_spacing_chips: 0.5,
            dll_bw_hz: 2.0,
            pll_bw_hz: 15.0,
            fll_bw_hz: 10.0,
            adaptive_tracking_enabled: default_adaptive_tracking_enabled(),
            tracking_integration_ms: 1,
            tracking_budget_ms: 1.0,
            tracking_over_budget_action: "drop_epochs".to_string(),
            tracking_per_band: Vec::new(),
            robust_solver: true,
            huber_k: 30.0,
            raim: true,
            position_solution_smoothing: true,
            position_solution_motion_class: NavigationMotionClass::Vehicle,
            hatch_window: 100,
            weighting: NavigationWeightingConfig::default(),
            iono_mode: "broadcast".to_string(),
            tropo_enable: true,
            tropo_ztd_m: 2.3,
            ppp: PppConfig::default(),
            science_thresholds: ScienceThresholdsConfig::default(),
            constellation_policy: ConstellationSelectionPolicy::Mixed,
        }
    }
}

/// Front-end conditioning options applied before acquisition.
#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema, Default)]
pub struct FrontEndConfig {
    /// Remove the mean I/Q offset from the acquisition frame before processing.
    #[serde(default)]
    pub remove_dc_offset: bool,
    /// Optional complex front-end FIR filter applied before acquisition and tracking.
    #[serde(default)]
    pub filter: Option<FrontEndFilterSpec>,
}

/// Tracking parameters for a specific band.
#[derive(Debug, Clone, Copy)]
pub struct TrackingParams {
    /// Early/late spacing, in chips.
    pub early_late_spacing_chips: f64,
    /// DLL noise bandwidth, in Hz.
    pub dll_bw_hz: f64,
    /// PLL noise bandwidth, in Hz.
    pub pll_bw_hz: f64,
    /// FLL noise bandwidth, in Hz.
    pub fll_bw_hz: f64,
    /// Integration time, in milliseconds.
    pub integration_ms: u32,
}

/// Per-band tracking specification in runtime config.
#[derive(Debug, Clone)]
pub struct BandTrackingSpec {
    /// Signal band.
    pub band: SignalBand,
    /// Early/late spacing, in chips.
    pub early_late_spacing_chips: f64,
    /// DLL noise bandwidth, in Hz.
    pub dll_bw_hz: f64,
    /// PLL noise bandwidth, in Hz.
    pub pll_bw_hz: f64,
    /// FLL noise bandwidth, in Hz.
    pub fll_bw_hz: f64,
    /// Integration time, in milliseconds.
    pub integration_ms: u32,
}

impl ReceiverPipelineConfig {
    /// Resolve tracking parameters for a given band, falling back to defaults.
    pub fn tracking_params(&self, band: SignalBand) -> TrackingParams {
        if let Some(profile) = self.tracking_per_band.iter().find(|p| p.band == band) {
            return TrackingParams {
                early_late_spacing_chips: profile.early_late_spacing_chips,
                dll_bw_hz: profile.dll_bw_hz,
                pll_bw_hz: profile.pll_bw_hz,
                fll_bw_hz: profile.fll_bw_hz,
                integration_ms: profile.integration_ms,
            };
        }
        TrackingParams {
            early_late_spacing_chips: self.early_late_spacing_chips,
            dll_bw_hz: self.dll_bw_hz,
            pll_bw_hz: self.pll_bw_hz,
            fll_bw_hz: self.fll_bw_hz,
            integration_ms: self.tracking_integration_ms,
        }
    }

    /// Whether this runtime configuration allows a given constellation.
    pub fn allows_constellation(&self, constellation: Constellation) -> bool {
        self.constellation_policy.allows(constellation)
    }

    /// Selected constellations enabled by this runtime configuration.
    pub fn selected_constellations(&self) -> &'static [Constellation] {
        self.constellation_policy.selected_constellations()
    }
}

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

impl Default for ConstellationSelectionPolicy {
    fn default() -> Self {
        Self::Mixed
    }
}

#[derive(Debug, Error)]
/// Receiver pipeline error wrapper.
pub enum ReceiverError {
    /// Invalid input.
    #[error(transparent)]
    Input(#[from] InputError),
    /// Invalid configuration.
    #[error(transparent)]
    Config(#[from] ConfigError),
    /// Signal processing error.
    #[error(transparent)]
    Signal(#[from] SignalError),
    /// Acquisition error.
    #[error(transparent)]
    Acquisition(#[from] AcqError),
    /// Tracking error.
    #[error(transparent)]
    Tracking(#[from] TrackError),
    /// Navigation error.
    #[error(transparent)]
    Navigation(#[from] NavError),
}

/// Acquisition configuration parameters.
#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
pub struct AcquisitionConfig {
    /// Doppler search range, in Hz.
    pub doppler_search_hz: i32,
    /// Doppler bin spacing, in Hz.
    pub doppler_step_hz: i32,
    /// Doppler-rate search range, in Hz/s.
    #[serde(default)]
    pub doppler_rate_search_hz_per_s: i32,
    /// Doppler-rate bin spacing, in Hz/s.
    #[serde(default)]
    pub doppler_rate_step_hz_per_s: i32,
    /// Coherent integration length, in ms.
    pub integration_ms: u32,
    /// Noncoherent integration count.
    #[serde(default)]
    pub noncoherent_integration: u32,
    /// Peak-to-mean threshold.
    pub peak_mean_threshold: f32,
    /// Peak-to-second-peak threshold.
    pub peak_second_threshold: f32,
    /// Threshold policy used to derive acceptance thresholds.
    #[serde(default)]
    pub threshold_policy: AcquisitionThresholdPolicyConfig,
}

/// Threshold-derivation policy for acquisition acceptance.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, JsonSchema, PartialEq, Eq, Default)]
#[serde(rename_all = "snake_case")]
pub enum AcquisitionThresholdMode {
    /// Use the configured ratio thresholds directly.
    #[default]
    FixedRatio,
    /// Calibrate the peak-to-mean threshold against a declared false-alarm probability.
    CalibratedFalseAlarm,
}

/// Acquisition threshold policy parameters.
#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
pub struct AcquisitionThresholdPolicyConfig {
    /// Threshold derivation mode.
    #[serde(default)]
    pub mode: AcquisitionThresholdMode,
    /// Target accepted false-alarm probability for calibrated acquisition thresholds.
    #[serde(default = "default_acquisition_false_alarm_probability")]
    pub false_alarm_probability: f64,
    /// Number of deterministic noise-only trials used during calibration.
    #[serde(default = "default_acquisition_threshold_calibration_trial_count")]
    pub calibration_trial_count: usize,
    /// Confidence level used to report the calibration interval.
    #[serde(default = "default_acquisition_threshold_confidence_level")]
    pub confidence_level: f64,
}

impl Default for AcquisitionThresholdPolicyConfig {
    fn default() -> Self {
        Self {
            mode: AcquisitionThresholdMode::FixedRatio,
            false_alarm_probability: default_acquisition_false_alarm_probability(),
            calibration_trial_count: default_acquisition_threshold_calibration_trial_count(),
            confidence_level: default_acquisition_threshold_confidence_level(),
        }
    }
}

pub fn default_acquisition_false_alarm_probability() -> f64 {
    0.01
}

pub fn default_acquisition_threshold_calibration_trial_count() -> usize {
    128
}

pub fn default_acquisition_threshold_confidence_level() -> f64 {
    0.95
}

/// Tracking configuration parameters.
#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
pub struct TrackingConfig {
    /// Early/late spacing, in chips.
    pub early_late_spacing_chips: f64,
    /// DLL noise bandwidth, in Hz.
    pub dll_bw_hz: f64,
    /// PLL noise bandwidth, in Hz.
    pub pll_bw_hz: f64,
    /// FLL noise bandwidth, in Hz.
    pub fll_bw_hz: f64,
    /// Whether tracking loop and coherent integration adaptation are enabled.
    #[serde(default = "default_adaptive_tracking_enabled")]
    pub adaptive_tracking_enabled: bool,
    /// Maximum tracking channels.
    pub max_channels: usize,
    /// Per-epoch CPU budget, in milliseconds.
    pub per_epoch_budget_ms: f64,
    /// Action when over budget (drop_epochs or continue).
    #[serde(default = "default_over_budget_action")]
    pub over_budget_action: String,
    /// Default integration time, in milliseconds.
    #[serde(default = "default_tracking_integration_ms")]
    pub integration_ms: u32,
    /// Per-band overrides.
    #[serde(default)]
    pub per_band: Vec<BandTrackingConfig>,
}

/// Tracking overrides for a specific band.
#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
pub struct BandTrackingConfig {
    /// Band identifier (e.g. L1, L2).
    pub band: String,
    /// Early/late spacing, in chips.
    pub early_late_spacing_chips: f64,
    /// DLL noise bandwidth, in Hz.
    pub dll_bw_hz: f64,
    /// PLL noise bandwidth, in Hz.
    pub pll_bw_hz: f64,
    /// FLL noise bandwidth, in Hz.
    pub fll_bw_hz: f64,
    /// Integration time, in milliseconds.
    #[serde(default = "default_tracking_integration_ms")]
    pub integration_ms: u32,
}

pub fn default_tracking_integration_ms() -> u32 {
    1
}

pub fn default_adaptive_tracking_enabled() -> bool {
    true
}

pub const SUPPORTED_ACQUISITION_INTEGRATION_MS: [u32; 5] = [1, 2, 5, 10, 20];

pub fn acquisition_integration_ms_is_supported(integration_ms: u32) -> bool {
    SUPPORTED_ACQUISITION_INTEGRATION_MS.contains(&integration_ms)
}

pub fn supported_acquisition_integration_ms_csv() -> String {
    SUPPORTED_ACQUISITION_INTEGRATION_MS.iter().map(u32::to_string).collect::<Vec<_>>().join(", ")
}

pub fn default_over_budget_action() -> String {
    "drop_epochs".to_string()
}

pub fn default_position_solution_smoothing() -> bool {
    true
}

pub fn parse_band(text: &str) -> Option<SignalBand> {
    match text.to_lowercase().as_str() {
        "l1" => Some(SignalBand::L1),
        "l2" => Some(SignalBand::L2),
        "l5" => Some(SignalBand::L5),
        "e1" => Some(SignalBand::E1),
        "e5" => Some(SignalBand::E5),
        "b1" => Some(SignalBand::B1),
        "b2" => Some(SignalBand::B2),
        _ => None,
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
/// Navigation configuration parameters.
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

#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
/// Scientific threshold policy parameters.
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

#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
/// PPP configuration parameters.
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
    /// Process noise for clock drift.
    pub noise_clock_drift: f64,
    /// Process noise for ZTD.
    pub noise_ztd: f64,
    /// Process noise for ionosphere.
    pub noise_iono: f64,
    /// Process noise for ambiguities.
    pub noise_ambiguity: f64,
    /// Minimum convergence time, in seconds.
    pub convergence_min_time_s: f64,
    /// Convergence position rate threshold, in m/s.
    pub convergence_pos_rate_mps: f64,
    /// Horizontal sigma threshold for convergence, in meters.
    pub convergence_sigma_h_m: f64,
    /// Vertical sigma threshold for convergence, in meters.
    pub convergence_sigma_v_m: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
/// Navigation measurement weighting parameters.
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
