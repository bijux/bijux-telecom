use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

use bijux_gnss_core::{
    AcqError, ConfigError, InputError, NavError, SchemaVersion, SignalBand, SignalError, TrackError,
};
use thiserror::Error;

/// On-disk receiver profile configuration.
#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
pub struct ReceiverProfile {
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
    pub acquisition: AcquisitionProfile,
    /// Tracking configuration.
    pub tracking: TrackingProfile,
    /// Navigation configuration.
    pub navigation: NavigationProfile,
}

/// Derived receiver configuration used at runtime.
#[derive(Debug, Clone)]
pub struct ReceiverConfig {
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
    pub weighting: NavigationWeightingProfile,
    /// Ionosphere model mode identifier.
    pub iono_mode: String,
    /// Whether to enable troposphere modeling.
    pub tropo_enable: bool,
    /// Default zenith tropospheric delay, in meters.
    pub tropo_ztd_m: f64,
    /// PPP configuration.
    pub ppp: PppProfile,
}

impl Default for ReceiverConfig {
    fn default() -> Self {
        Self {
            sampling_freq_hz: 5_000_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            channels: 12,
            early_late_spacing_chips: 0.5,
            dll_bw_hz: 2.0,
            pll_bw_hz: 15.0,
            fll_bw_hz: 10.0,
            tracking_integration_ms: 1,
            tracking_per_band: Vec::new(),
            robust_solver: true,
            huber_k: 30.0,
            raim: true,
            hatch_window: 100,
            weighting: NavigationWeightingProfile::default(),
            iono_mode: "broadcast".to_string(),
            tropo_enable: true,
            tropo_ztd_m: 2.3,
            ppp: PppProfile::default(),
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

impl ReceiverConfig {
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
pub struct AcquisitionProfile {
    /// Doppler search range, in Hz.
    pub doppler_search_hz: i32,
    /// Doppler bin spacing, in Hz.
    pub doppler_step_hz: i32,
    /// Coherent integration length, in ms.
    pub integration_ms: u32,
    /// Peak-to-mean threshold.
    pub peak_mean_threshold: f32,
    /// Peak-to-second-peak threshold.
    pub peak_second_threshold: f32,
}

/// Tracking configuration parameters.
#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
pub struct TrackingProfile {
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
    /// Default integration time, in milliseconds.
    #[serde(default = "default_tracking_integration_ms")]
    pub integration_ms: u32,
    /// Per-band overrides.
    #[serde(default)]
    pub per_band: Vec<BandTrackingProfile>,
}

/// Tracking overrides for a specific band.
#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
pub struct BandTrackingProfile {
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

pub(crate) fn default_tracking_integration_ms() -> u32 {
    1
}

pub(crate) fn parse_band(text: &str) -> Option<SignalBand> {
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
pub struct NavigationProfile {
    /// Enable robust solver.
    pub robust_solver: bool,
    /// Huber loss parameter.
    pub huber_k: f64,
    /// Enable RAIM-like checks.
    pub raim: bool,
    /// Hatch smoothing window.
    pub hatch_window: u32,
    /// Weighting configuration.
    pub weighting: NavigationWeightingProfile,
    /// Ionosphere model mode identifier.
    pub iono_mode: String,
    /// Enable troposphere modeling.
    pub tropo_enable: bool,
    /// Default ZTD, in meters.
    pub tropo_ztd_m: f64,
    /// PPP configuration.
    #[serde(default)]
    pub ppp: PppProfile,
}

#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
/// PPP configuration parameters.
pub struct PppProfile {
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
pub struct NavigationWeightingProfile {
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
