#![allow(missing_docs)]

use bijux_gnss_core::api::SchemaVersion;

use crate::engine::receiver_config::navigation::{
    ConstellationSelectionPolicy, NavigationConfig, NavigationMotionClass,
    NavigationWeightingConfig, NavigationWeightingMode, PppConfig, ScienceThresholdsConfig,
    DEFAULT_PPP_MEASUREMENT_ANTENNA_RESIDUAL_M, DEFAULT_PPP_MEASUREMENT_CLOCK_SIGMA_SCALE,
    DEFAULT_PPP_MEASUREMENT_CODE_FLOOR_M, DEFAULT_PPP_MEASUREMENT_ORBIT_SIGMA_SCALE,
    DEFAULT_PPP_MEASUREMENT_PHASE_FLOOR_CYCLES, DEFAULT_PPP_MEASUREMENT_TROPOSPHERE_RESIDUAL_M,
    DEFAULT_PPP_NOISE_AMBIGUITY, DEFAULT_PPP_NOISE_CLOCK_BIAS, DEFAULT_PPP_NOISE_CLOCK_DRIFT,
    DEFAULT_PPP_NOISE_INTER_SYSTEM_BIAS, DEFAULT_PPP_NOISE_IONO, DEFAULT_PPP_NOISE_POSITION,
    DEFAULT_PPP_NOISE_VELOCITY, DEFAULT_PPP_NOISE_ZTD, DEFAULT_PPP_PRECISE_PRODUCT_BRIDGE_ACTION,
    DEFAULT_PPP_PRECISE_PRODUCT_REFUSE_ACTION, DEFAULT_PPP_PRECISE_PRODUCT_RESET_ACTION,
    DEFAULT_PPP_PRECISE_PRODUCT_STATE_INFLATION,
};
use crate::engine::receiver_config::{
    default_acquisition_false_alarm_probability,
    default_acquisition_threshold_calibration_trial_count,
    default_acquisition_threshold_confidence_level, default_adaptive_tracking_enabled,
    default_tracking_integration_ms, default_vector_tracking_enabled, AcquisitionConfig,
    AcquisitionThresholdMode, AcquisitionThresholdPolicyConfig, FrontEndConfig,
    ReceiverClockConfig, ReceiverConfig, TrackingConfig,
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
            front_end: FrontEndConfig::default(),
            receiver_clock: ReceiverClockConfig::default(),
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
            doppler_rate_search_hz_per_s: 0,
            doppler_rate_step_hz_per_s: 250,
            integration_ms: 1,
            noncoherent_integration: 1,
            peak_mean_threshold: 2.5,
            peak_second_threshold: 1.5,
            threshold_policy: AcquisitionThresholdPolicyConfig {
                mode: AcquisitionThresholdMode::FixedRatio,
                false_alarm_probability: default_acquisition_false_alarm_probability(),
                calibration_trial_count: default_acquisition_threshold_calibration_trial_count(),
                confidence_level: default_acquisition_threshold_confidence_level(),
            },
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
            adaptive_tracking_enabled: default_adaptive_tracking_enabled(),
            vector_tracking_enabled: default_vector_tracking_enabled(),
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

impl Default for NavigationWeightingConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            mode: NavigationWeightingMode::Elevation,
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
            receiver_antenna_type: None,
            noise_position: DEFAULT_PPP_NOISE_POSITION,
            noise_velocity: DEFAULT_PPP_NOISE_VELOCITY,
            noise_clock_bias: DEFAULT_PPP_NOISE_CLOCK_BIAS,
            noise_clock_drift: DEFAULT_PPP_NOISE_CLOCK_DRIFT,
            noise_inter_system_bias: DEFAULT_PPP_NOISE_INTER_SYSTEM_BIAS,
            noise_ztd: DEFAULT_PPP_NOISE_ZTD,
            noise_iono: DEFAULT_PPP_NOISE_IONO,
            noise_ambiguity: DEFAULT_PPP_NOISE_AMBIGUITY,
            measurement_code_floor_m: DEFAULT_PPP_MEASUREMENT_CODE_FLOOR_M,
            measurement_phase_floor_cycles: DEFAULT_PPP_MEASUREMENT_PHASE_FLOOR_CYCLES,
            measurement_orbit_sigma_scale: DEFAULT_PPP_MEASUREMENT_ORBIT_SIGMA_SCALE,
            measurement_clock_sigma_scale: DEFAULT_PPP_MEASUREMENT_CLOCK_SIGMA_SCALE,
            measurement_troposphere_residual_m: DEFAULT_PPP_MEASUREMENT_TROPOSPHERE_RESIDUAL_M,
            measurement_antenna_residual_m: DEFAULT_PPP_MEASUREMENT_ANTENNA_RESIDUAL_M,
            precise_product_missing_satellite_action: DEFAULT_PPP_PRECISE_PRODUCT_BRIDGE_ACTION
                .to_string(),
            precise_product_out_of_coverage_action: DEFAULT_PPP_PRECISE_PRODUCT_BRIDGE_ACTION
                .to_string(),
            precise_product_insufficient_support_action: DEFAULT_PPP_PRECISE_PRODUCT_BRIDGE_ACTION
                .to_string(),
            precise_product_orbit_gap_action: DEFAULT_PPP_PRECISE_PRODUCT_RESET_ACTION.to_string(),
            precise_product_orbit_flag_action: DEFAULT_PPP_PRECISE_PRODUCT_REFUSE_ACTION
                .to_string(),
            precise_product_clock_gap_action: DEFAULT_PPP_PRECISE_PRODUCT_RESET_ACTION.to_string(),
            precise_product_clock_jump_action: DEFAULT_PPP_PRECISE_PRODUCT_RESET_ACTION.to_string(),
            precise_product_state_inflation: DEFAULT_PPP_PRECISE_PRODUCT_STATE_INFLATION,
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
            max_gdop: 12.0,
            max_residual_rms_m: 25.0,
            min_used_satellites: 4,
            min_lock_ratio: 0.7,
        }
    }
}
