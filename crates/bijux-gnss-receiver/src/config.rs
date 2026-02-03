use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

use bijux_gnss_core::{
    AcqError, ConfigError, InputError, NavError, SignalBand, SignalError, TrackError,
};
use thiserror::Error;

#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
pub struct ReceiverProfile {
    pub sample_rate_hz: f64,
    pub intermediate_freq_hz: f64,
    pub quantization_bits: u8,
    pub code_freq_basis_hz: f64,
    pub code_length: usize,
    pub seed: u64,
    pub acquisition: AcquisitionProfile,
    pub tracking: TrackingProfile,
    pub navigation: NavigationProfile,
}

#[derive(Debug, Clone)]
pub struct ReceiverConfig {
    pub sampling_freq_hz: f64,
    pub intermediate_freq_hz: f64,
    pub code_freq_basis_hz: f64,
    pub code_length: usize,
    pub channels: usize,
    pub early_late_spacing_chips: f64,
    pub dll_bw_hz: f64,
    pub pll_bw_hz: f64,
    pub fll_bw_hz: f64,
    pub tracking_integration_ms: u32,
    pub tracking_per_band: Vec<BandTrackingSpec>,
    pub robust_solver: bool,
    pub huber_k: f64,
    pub raim: bool,
    pub hatch_window: u32,
    pub weighting: NavigationWeightingProfile,
    pub iono_mode: String,
    pub tropo_enable: bool,
    pub tropo_ztd_m: f64,
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
            weighting: NavigationWeightingProfile {
                enabled: true,
                min_elev_deg: 5.0,
                elev_exponent: 2.0,
                cn0_ref_dbhz: 50.0,
                min_weight: 0.1,
                elev_mask_deg: 5.0,
                tracking_mode_scalar_weight: 1.0,
                tracking_mode_vector_weight: 1.2,
            },
            iono_mode: "broadcast".to_string(),
            tropo_enable: true,
            tropo_ztd_m: 2.3,
            ppp: PppProfile::default(),
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct TrackingParams {
    pub early_late_spacing_chips: f64,
    pub dll_bw_hz: f64,
    pub pll_bw_hz: f64,
    pub fll_bw_hz: f64,
    pub integration_ms: u32,
}

#[derive(Debug, Clone)]
pub struct BandTrackingSpec {
    pub band: SignalBand,
    pub early_late_spacing_chips: f64,
    pub dll_bw_hz: f64,
    pub pll_bw_hz: f64,
    pub fll_bw_hz: f64,
    pub integration_ms: u32,
}

impl ReceiverConfig {
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
pub enum ReceiverError {
    #[error(transparent)]
    Input(#[from] InputError),
    #[error(transparent)]
    Config(#[from] ConfigError),
    #[error(transparent)]
    Signal(#[from] SignalError),
    #[error(transparent)]
    Acquisition(#[from] AcqError),
    #[error(transparent)]
    Tracking(#[from] TrackError),
    #[error(transparent)]
    Navigation(#[from] NavError),
}

#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
pub struct AcquisitionProfile {
    pub doppler_search_hz: i32,
    pub doppler_step_hz: i32,
    pub integration_ms: u32,
    pub peak_mean_threshold: f32,
    pub peak_second_threshold: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
pub struct TrackingProfile {
    pub early_late_spacing_chips: f64,
    pub dll_bw_hz: f64,
    pub pll_bw_hz: f64,
    pub fll_bw_hz: f64,
    pub max_channels: usize,
    pub per_epoch_budget_ms: f64,
    #[serde(default = "default_tracking_integration_ms")]
    pub integration_ms: u32,
    #[serde(default)]
    pub per_band: Vec<BandTrackingProfile>,
}

#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
pub struct BandTrackingProfile {
    pub band: String,
    pub early_late_spacing_chips: f64,
    pub dll_bw_hz: f64,
    pub pll_bw_hz: f64,
    pub fll_bw_hz: f64,
    #[serde(default = "default_tracking_integration_ms")]
    pub integration_ms: u32,
}

fn default_tracking_integration_ms() -> u32 {
    1
}

fn parse_band(text: &str) -> Option<SignalBand> {
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
pub struct NavigationProfile {
    pub robust_solver: bool,
    pub huber_k: f64,
    pub raim: bool,
    pub hatch_window: u32,
    pub weighting: NavigationWeightingProfile,
    pub iono_mode: String,
    pub tropo_enable: bool,
    pub tropo_ztd_m: f64,
    #[serde(default)]
    pub ppp: PppProfile,
}

#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
pub struct PppProfile {
    pub enabled: bool,
    pub use_iono_free: bool,
    pub use_doppler: bool,
    pub enable_iono_state: bool,
    pub ar_mode: String,
    pub ar_ratio_threshold: f64,
    pub ar_stability_epochs: u32,
    pub ar_max_sats: usize,
    pub ar_use_elevation: bool,
    pub prune_after_epochs: u64,
    pub reset_gap_s: f64,
    pub residual_gate_m: f64,
    pub drift_window_epochs: u64,
    pub drift_threshold_m: f64,
    pub checkpoint_interval_epochs: u64,
    pub noise_clock_drift: f64,
    pub noise_ztd: f64,
    pub noise_iono: f64,
    pub noise_ambiguity: f64,
    pub convergence_min_time_s: f64,
    pub convergence_pos_rate_mps: f64,
    pub convergence_sigma_h_m: f64,
    pub convergence_sigma_v_m: f64,
}

impl Default for PppProfile {
    fn default() -> Self {
        Self {
            enabled: false,
            use_iono_free: false,
            use_doppler: false,
            enable_iono_state: false,
            ar_mode: "float_ppp".to_string(),
            ar_ratio_threshold: 3.0,
            ar_stability_epochs: 3,
            ar_max_sats: 8,
            ar_use_elevation: true,
            prune_after_epochs: 200,
            reset_gap_s: 2.0,
            residual_gate_m: 200.0,
            drift_window_epochs: 100,
            drift_threshold_m: 10.0,
            checkpoint_interval_epochs: 0,
            noise_clock_drift: 1e-5,
            noise_ztd: 0.01,
            noise_iono: 0.1,
            noise_ambiguity: 0.05,
            convergence_min_time_s: 60.0,
            convergence_pos_rate_mps: 0.1,
            convergence_sigma_h_m: 1.0,
            convergence_sigma_v_m: 2.0,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
pub struct NavigationWeightingProfile {
    pub enabled: bool,
    pub min_elev_deg: f64,
    pub elev_exponent: f64,
    pub cn0_ref_dbhz: f64,
    pub min_weight: f64,
    pub elev_mask_deg: f64,
    pub tracking_mode_scalar_weight: f64,
    pub tracking_mode_vector_weight: f64,
}

impl ReceiverProfile {
    pub fn validate(&self) -> Result<(), Vec<String>> {
        let mut errors = Vec::new();
        if self.sample_rate_hz <= 0.0 {
            errors.push("sample_rate_hz must be > 0".to_string());
        }
        if self.code_length == 0 {
            errors.push("code_length must be > 0".to_string());
        }
        if self.quantization_bits == 0 {
            errors.push("quantization_bits must be > 0".to_string());
        }
        if self.seed == 0 {
            errors.push("seed must be > 0".to_string());
        }
        if self.acquisition.doppler_step_hz <= 0 {
            errors.push("acquisition.doppler_step_hz must be > 0".to_string());
        }
        if self.acquisition.integration_ms == 0 {
            errors.push("acquisition.integration_ms must be > 0".to_string());
        }
        if self.tracking.early_late_spacing_chips <= 0.0 {
            errors.push("tracking.early_late_spacing_chips must be > 0".to_string());
        }
        if self.tracking.max_channels == 0 {
            errors.push("tracking.max_channels must be > 0".to_string());
        }
        if self.tracking.per_epoch_budget_ms <= 0.0 {
            errors.push("tracking.per_epoch_budget_ms must be > 0".to_string());
        }
        if self.tracking.integration_ms == 0 {
            errors.push("tracking.integration_ms must be > 0".to_string());
        }
        let mut seen = std::collections::BTreeSet::new();
        for band in &self.tracking.per_band {
            let parsed = match parse_band(&band.band) {
                Some(band) => band,
                None => {
                    errors.push(format!("tracking.per_band has unknown band {}", band.band));
                    continue;
                }
            };
            if !seen.insert(parsed) {
                errors.push(format!(
                    "tracking.per_band has duplicate entry for {:?}",
                    parsed
                ));
            }
            if band.early_late_spacing_chips <= 0.0 {
                errors.push(format!(
                    "tracking.per_band.{:?}.early_late_spacing_chips must be > 0",
                    parsed
                ));
            }
            if band.dll_bw_hz <= 0.0 || band.pll_bw_hz <= 0.0 || band.fll_bw_hz <= 0.0 {
                errors.push(format!(
                    "tracking.per_band.{:?}.loop bandwidths must be > 0",
                    parsed
                ));
            }
            if band.integration_ms == 0 {
                errors.push(format!(
                    "tracking.per_band.{:?}.integration_ms must be > 0",
                    parsed
                ));
            }
        }
        if self.navigation.huber_k <= 0.0 {
            errors.push("navigation.huber_k must be > 0".to_string());
        }
        if self.navigation.hatch_window == 0 {
            errors.push("navigation.hatch_window must be > 0".to_string());
        }
        if self.navigation.tropo_ztd_m < 0.0 {
            errors.push("navigation.tropo_ztd_m must be >= 0".to_string());
        }
        if self.navigation.weighting.min_elev_deg < 0.0
            || self.navigation.weighting.min_elev_deg > 90.0
        {
            errors.push("navigation.weighting.min_elev_deg must be within [0, 90]".to_string());
        }
        if self.navigation.weighting.elev_exponent <= 0.0 {
            errors.push("navigation.weighting.elev_exponent must be > 0".to_string());
        }
        if self.navigation.weighting.cn0_ref_dbhz <= 0.0 {
            errors.push("navigation.weighting.cn0_ref_dbhz must be > 0".to_string());
        }
        if self.navigation.weighting.min_weight <= 0.0 {
            errors.push("navigation.weighting.min_weight must be > 0".to_string());
        }
        if self.navigation.weighting.elev_mask_deg < 0.0
            || self.navigation.weighting.elev_mask_deg > 90.0
        {
            errors.push("navigation.weighting.elev_mask_deg must be within [0, 90]".to_string());
        }
        if self.navigation.weighting.tracking_mode_scalar_weight <= 0.0 {
            errors.push("navigation.weighting.tracking_mode_scalar_weight must be > 0".to_string());
        }
        if self.navigation.weighting.tracking_mode_vector_weight <= 0.0 {
            errors.push("navigation.weighting.tracking_mode_vector_weight must be > 0".to_string());
        }
        if self.navigation.ppp.reset_gap_s <= 0.0 {
            errors.push("navigation.ppp.reset_gap_s must be > 0".to_string());
        }
        if self.navigation.ppp.prune_after_epochs == 0 {
            errors.push("navigation.ppp.prune_after_epochs must be > 0".to_string());
        }
        if self.navigation.ppp.residual_gate_m <= 0.0 {
            errors.push("navigation.ppp.residual_gate_m must be > 0".to_string());
        }
        if self.navigation.ppp.drift_window_epochs == 0 {
            errors.push("navigation.ppp.drift_window_epochs must be > 0".to_string());
        }
        if self.navigation.ppp.drift_threshold_m <= 0.0 {
            errors.push("navigation.ppp.drift_threshold_m must be > 0".to_string());
        }
        if errors.is_empty() {
            Ok(())
        } else {
            Err(errors)
        }
    }

    pub fn to_receiver_config(&self) -> ReceiverConfig {
        ReceiverConfig {
            sampling_freq_hz: self.sample_rate_hz,
            intermediate_freq_hz: self.intermediate_freq_hz,
            code_freq_basis_hz: self.code_freq_basis_hz,
            code_length: self.code_length,
            channels: self.tracking.max_channels,
            early_late_spacing_chips: self.tracking.early_late_spacing_chips,
            dll_bw_hz: self.tracking.dll_bw_hz,
            pll_bw_hz: self.tracking.pll_bw_hz,
            fll_bw_hz: self.tracking.fll_bw_hz,
            tracking_per_band: self
                .tracking
                .per_band
                .iter()
                .filter_map(|entry| {
                    parse_band(&entry.band).map(|band| BandTrackingSpec {
                        band,
                        early_late_spacing_chips: entry.early_late_spacing_chips,
                        dll_bw_hz: entry.dll_bw_hz,
                        pll_bw_hz: entry.pll_bw_hz,
                        fll_bw_hz: entry.fll_bw_hz,
                        integration_ms: entry.integration_ms,
                    })
                })
                .collect(),
            tracking_integration_ms: self.tracking.integration_ms,
            robust_solver: self.navigation.robust_solver,
            huber_k: self.navigation.huber_k,
            raim: self.navigation.raim,
            hatch_window: self.navigation.hatch_window,
            weighting: self.navigation.weighting.clone(),
            iono_mode: self.navigation.iono_mode.clone(),
            tropo_enable: self.navigation.tropo_enable,
            tropo_ztd_m: self.navigation.tropo_ztd_m,
            ppp: self.navigation.ppp.clone(),
        }
    }
}

impl Default for ReceiverProfile {
    fn default() -> Self {
        Self {
            sample_rate_hz: 5_000_000.0,
            intermediate_freq_hz: 0.0,
            quantization_bits: 16,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            seed: 1,
            acquisition: AcquisitionProfile {
                doppler_search_hz: 10_000,
                doppler_step_hz: 500,
                integration_ms: 1,
                peak_mean_threshold: 2.5,
                peak_second_threshold: 1.5,
            },
            tracking: TrackingProfile {
                early_late_spacing_chips: 0.5,
                dll_bw_hz: 2.0,
                pll_bw_hz: 15.0,
                fll_bw_hz: 10.0,
                max_channels: 8,
                per_epoch_budget_ms: 0.7,
                integration_ms: 1,
                per_band: Vec::new(),
            },
            navigation: NavigationProfile {
                robust_solver: true,
                huber_k: 30.0,
                raim: true,
                hatch_window: 100,
                weighting: NavigationWeightingProfile {
                    enabled: true,
                    min_elev_deg: 5.0,
                    elev_exponent: 2.0,
                    cn0_ref_dbhz: 50.0,
                    min_weight: 0.1,
                    elev_mask_deg: 5.0,
                    tracking_mode_scalar_weight: 1.0,
                    tracking_mode_vector_weight: 1.2,
                },
                iono_mode: "broadcast".to_string(),
                tropo_enable: true,
                tropo_ztd_m: 2.3,
                ppp: PppProfile::default(),
            },
        }
    }
}
