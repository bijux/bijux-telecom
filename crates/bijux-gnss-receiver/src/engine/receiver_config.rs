use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

use bijux_gnss_core::api::{
    AcqError, ConfigError, InputError, NavError, SchemaVersion, SignalBand, SignalError, TrackError,
};
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
    /// Coherent integration used by acquisition, in milliseconds.
    pub acquisition_integration_ms: u32,
    /// Noncoherent integration used by acquisition.
    pub acquisition_noncoherent: u32,
    /// Minimum accepted peak-to-mean acquisition ratio.
    pub acquisition_peak_mean_threshold: f32,
    /// Minimum accepted peak-to-second-peak acquisition ratio.
    pub acquisition_peak_second_threshold: f32,
    /// Default early/late spacing, in chips.
    pub early_late_spacing_chips: f64,
    /// DLL noise bandwidth, in Hz.
    pub dll_bw_hz: f64,
    /// PLL noise bandwidth, in Hz.
    pub pll_bw_hz: f64,
    /// FLL noise bandwidth, in Hz.
    pub fll_bw_hz: f64,
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
}

impl Default for ReceiverPipelineConfig {
    fn default() -> Self {
        Self {
            sampling_freq_hz: 5_000_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            channels: 12,
            acquisition_doppler_search_hz: 10_000,
            acquisition_doppler_step_hz: 500,
            acquisition_integration_ms: 1,
            acquisition_noncoherent: 1,
            acquisition_peak_mean_threshold: 2.5,
            acquisition_peak_second_threshold: 1.5,
            early_late_spacing_chips: 0.5,
            dll_bw_hz: 2.0,
            pll_bw_hz: 15.0,
            fll_bw_hz: 10.0,
            tracking_integration_ms: 1,
            tracking_budget_ms: 1.0,
            tracking_over_budget_action: "drop_epochs".to_string(),
            tracking_per_band: Vec::new(),
            robust_solver: true,
            huber_k: 30.0,
            raim: true,
            hatch_window: 100,
            weighting: NavigationWeightingConfig::default(),
            iono_mode: "broadcast".to_string(),
            tropo_enable: true,
            tropo_ztd_m: 2.3,
            ppp: PppConfig::default(),
            science_thresholds: ScienceThresholdsConfig::default(),
        }
    }
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
    /// Coherent integration length, in ms.
    pub integration_ms: u32,
    /// Noncoherent integration count.
    #[serde(default)]
    pub noncoherent_integration: u32,
    /// Peak-to-mean threshold.
    pub peak_mean_threshold: f32,
    /// Peak-to-second-peak threshold.
    pub peak_second_threshold: f32,
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

#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
/// Navigation configuration parameters.
pub struct NavigationConfig {
    /// Enable robust solver.
    pub robust_solver: bool,
    /// Huber loss parameter.
    pub huber_k: f64,
    /// Enable RAIM-like checks.
    pub raim: bool,
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
}

#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
/// Scientific threshold policy parameters.
pub struct ScienceThresholdsConfig {
    /// Minimum mean C/N0 for accepted navigation solutions.
    pub min_mean_cn0_dbhz: f64,
    /// Maximum PDOP for accepted navigation solutions.
    pub max_pdop: f64,
    /// Maximum residual RMS (meters) for accepted navigation solutions.
    pub max_residual_rms_m: f64,
    /// Minimum used satellites for accepted navigation solutions.
    pub min_used_satellites: usize,
    /// Minimum lock quality ratio for stable integrity classification.
    pub min_lock_ratio: f64,
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
