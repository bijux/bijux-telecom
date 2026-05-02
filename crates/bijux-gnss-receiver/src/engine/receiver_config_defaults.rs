#![allow(missing_docs)]

use bijux_gnss_core::api::SchemaVersion;

use crate::engine::receiver_config::{
    default_tracking_integration_ms, AcquisitionConfig, NavigationConfig,
    NavigationWeightingConfig, PppConfig, ReceiverConfig, ScienceThresholdsConfig, TrackingConfig,
};

impl Default for ReceiverConfig {
    fn default() -> Self {
        Self {
            schema_version: SchemaVersion::CURRENT,
            sample_rate_hz: 5_000_000.0,
            intermediate_freq_hz: 0.0,
            quantization_bits: 16,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            seed: 1,
            acquisition: AcquisitionConfig::default(),
            tracking: TrackingConfig::default(),
            navigation: NavigationConfig::default(),
        }
    }
}

impl Default for AcquisitionConfig {
    fn default() -> Self {
        Self {
            doppler_search_hz: 10000,
            doppler_step_hz: 500,
            integration_ms: 1,
            noncoherent_integration: 1,
            peak_mean_threshold: 2.5,
            peak_second_threshold: 1.5,
        }
    }
}

impl Default for TrackingConfig {
    fn default() -> Self {
        Self {
            early_late_spacing_chips: 0.5,
            dll_bw_hz: 2.0,
            pll_bw_hz: 15.0,
            fll_bw_hz: 10.0,
            max_channels: 8,
            per_epoch_budget_ms: 0.7,
            over_budget_action: "drop_epochs".to_string(),
            integration_ms: default_tracking_integration_ms(),
            per_band: Vec::new(),
        }
    }
}

impl Default for NavigationConfig {
    fn default() -> Self {
        Self {
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

impl Default for NavigationWeightingConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            min_elev_deg: 5.0,
            elev_exponent: 2.0,
            cn0_ref_dbhz: 50.0,
            min_weight: 0.1,
            elev_mask_deg: 5.0,
            tracking_mode_scalar_weight: 1.0,
            tracking_mode_vector_weight: 1.2,
        }
    }
}

impl Default for PppConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            use_iono_free: true,
            use_doppler: true,
            enable_iono_state: true,
            ar_mode: "float_ppp".to_string(),
            ar_ratio_threshold: 2.5,
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

impl Default for ScienceThresholdsConfig {
    fn default() -> Self {
        Self {
            min_mean_cn0_dbhz: 28.0,
            max_pdop: 8.0,
            max_residual_rms_m: 25.0,
            min_used_satellites: 4,
            min_lock_ratio: 0.7,
        }
    }
}
