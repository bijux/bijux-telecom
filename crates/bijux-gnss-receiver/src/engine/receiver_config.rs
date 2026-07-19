use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

use bijux_gnss_core::api::{
    AcqError, ConfigError, Constellation, InputError, NavError, SchemaVersion, SignalBand,
    SignalError, TrackError,
};
use bijux_gnss_signal::api::FrontEndFilterSpec;
use thiserror::Error;

pub(crate) mod navigation;

use self::navigation::{
    ConstellationSelectionPolicy, NavigationConfig, NavigationMotionClass,
    NavigationWeightingConfig, PppConfig, ScienceThresholdsConfig,
};

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
    /// Receiver clock model used when forming observations from receiver time.
    #[serde(default)]
    pub receiver_clock: ReceiverClockConfig,
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
    /// Receiver clock bias applied to code and carrier observables, in seconds.
    pub receiver_clock_bias_s: f64,
    /// Receiver oscillator frequency bias applied to Doppler observations, in Hz.
    pub receiver_clock_frequency_bias_hz: f64,
    /// One-sigma receiver clock bias uncertainty represented in observation error models, in seconds.
    pub receiver_clock_bias_sigma_s: f64,
    /// Source label for receiver clock terms.
    pub receiver_clock_source: String,
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
    /// Whether receiver-wide vector tracking aid is enabled.
    pub vector_tracking_enabled: bool,
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
            receiver_clock_bias_s: 0.0,
            receiver_clock_frequency_bias_hz: 0.0,
            receiver_clock_bias_sigma_s: 0.0,
            receiver_clock_source: "config".to_string(),
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
            vector_tracking_enabled: default_vector_tracking_enabled(),
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

/// Receiver clock terms applied when observations are generated.
#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema, PartialEq)]
pub struct ReceiverClockConfig {
    /// Receiver clock bias relative to GNSS time, in seconds.
    #[serde(default)]
    pub bias_s: f64,
    /// Receiver oscillator frequency bias added to Doppler observations, in Hz.
    #[serde(default)]
    pub frequency_bias_hz: f64,
    /// One-sigma receiver clock bias uncertainty, in seconds.
    #[serde(default)]
    pub bias_sigma_s: f64,
    /// Source label for configured clock terms.
    #[serde(default = "default_receiver_clock_source")]
    pub source: String,
}

impl Default for ReceiverClockConfig {
    fn default() -> Self {
        Self {
            bias_s: 0.0,
            frequency_bias_hz: 0.0,
            bias_sigma_s: 0.0,
            source: default_receiver_clock_source(),
        }
    }
}

pub fn default_receiver_clock_source() -> String {
    "config".to_string()
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
    /// Whether receiver-wide vector tracking aid is enabled.
    #[serde(default = "default_vector_tracking_enabled")]
    pub vector_tracking_enabled: bool,
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

pub fn default_vector_tracking_enabled() -> bool {
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
