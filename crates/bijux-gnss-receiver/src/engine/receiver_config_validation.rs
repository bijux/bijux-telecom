#![allow(missing_docs)]

use crate::engine::receiver_config::navigation::{
    DEFAULT_PPP_PRECISE_PRODUCT_BRIDGE_ACTION, DEFAULT_PPP_PRECISE_PRODUCT_REFUSE_ACTION,
    DEFAULT_PPP_PRECISE_PRODUCT_RESET_ACTION,
};
use crate::engine::receiver_config::{
    acquisition_integration_ms_is_supported, parse_band, supported_acquisition_integration_ms_csv,
    BandTrackingSpec, ReceiverConfig, ReceiverPipelineConfig,
};
use bijux_gnss_core::api::{ConfigError, SchemaVersion, ValidateConfig, ValidationReport};

fn validate_finite_non_negative(report: &mut ValidationReport, field: &str, value: f64) {
    if !value.is_finite() || value < 0.0 {
        report.errors.push(ConfigError { message: format!("{field} must be finite and >= 0") });
    }
}

fn validate_ppp_precise_product_action(report: &mut ValidationReport, field: &str, value: &str) {
    if value != DEFAULT_PPP_PRECISE_PRODUCT_BRIDGE_ACTION
        && value != "inflate_satellite_state"
        && value != DEFAULT_PPP_PRECISE_PRODUCT_RESET_ACTION
        && value != DEFAULT_PPP_PRECISE_PRODUCT_REFUSE_ACTION
    {
        report.errors.push(ConfigError {
            message: format!(
                "{field} must be one of [bridge_with_broadcast, inflate_satellite_state, reset_satellite_state, refuse_satellite]"
            ),
        });
    }
}

fn validate_ppp_ar_mode(report: &mut ValidationReport, value: &str) {
    if value != "float_ppp" && value != "ppp_ar_wide_lane" && value != "ppp_ar_narrow_lane" {
        report.errors.push(ConfigError {
            message: "navigation.ppp.ar_mode must be one of [float_ppp, ppp_ar_wide_lane, ppp_ar_narrow_lane]"
                .to_string(),
        });
    }
}

impl ValidateConfig for ReceiverConfig {
    fn validate(&self) -> ValidationReport {
        let mut report = ValidationReport::default();
        if self.schema_version.0 != SchemaVersion::CURRENT.0 {
            report.errors.push(ConfigError {
                message: format!(
                    "schema_version {} does not match current {}",
                    self.schema_version.0,
                    SchemaVersion::CURRENT.0
                ),
            });
        }
        if self.sample_rate_hz <= 0.0 {
            report.errors.push(ConfigError { message: "sample_rate_hz must be > 0".to_string() });
        }
        if let Some(filter) = &self.front_end.filter {
            if let Err(error) = filter.validate(self.sample_rate_hz) {
                report.errors.push(ConfigError { message: format!("front_end.filter {error}") });
            }
        }
        if !self.receiver_clock.bias_s.is_finite() {
            report
                .errors
                .push(ConfigError { message: "receiver_clock.bias_s must be finite".to_string() });
        }
        if !self.receiver_clock.frequency_bias_hz.is_finite() {
            report.errors.push(ConfigError {
                message: "receiver_clock.frequency_bias_hz must be finite".to_string(),
            });
        }
        if !self.receiver_clock.bias_sigma_s.is_finite() || self.receiver_clock.bias_sigma_s < 0.0 {
            report.errors.push(ConfigError {
                message: "receiver_clock.bias_sigma_s must be finite and >= 0".to_string(),
            });
        }
        if self.receiver_clock.source.trim().is_empty() {
            report.errors.push(ConfigError {
                message: "receiver_clock.source must not be empty".to_string(),
            });
        }
        if self.code_length == 0 {
            report.errors.push(ConfigError { message: "code_length must be > 0".to_string() });
        }
        if self.quantization_bits == 0 {
            report
                .errors
                .push(ConfigError { message: "quantization_bits must be > 0".to_string() });
        }
        if self.seed == 0 {
            report.errors.push(ConfigError { message: "seed must be > 0".to_string() });
        }
        if self.acquisition.doppler_step_hz <= 0 {
            report.errors.push(ConfigError {
                message: "acquisition.doppler_step_hz must be > 0".to_string(),
            });
        }
        if self.acquisition.doppler_search_hz < 0 {
            report.errors.push(ConfigError {
                message: "acquisition.doppler_search_hz must be >= 0".to_string(),
            });
        }
        if self.acquisition.doppler_step_hz > 0
            && self.acquisition.doppler_search_hz > 0
            && self.acquisition.doppler_search_hz % self.acquisition.doppler_step_hz != 0
        {
            report.errors.push(ConfigError {
                message: "acquisition.doppler_search_hz must be an integer multiple of acquisition.doppler_step_hz".to_string(),
            });
        }
        if self.acquisition.integration_ms == 0 {
            report.errors.push(ConfigError {
                message: "acquisition.integration_ms must be > 0".to_string(),
            });
        } else if !acquisition_integration_ms_is_supported(self.acquisition.integration_ms) {
            report.errors.push(ConfigError {
                message: format!(
                    "acquisition.integration_ms must be one of [{}]",
                    supported_acquisition_integration_ms_csv()
                ),
            });
        }
        if self.acquisition.noncoherent_integration == 0 {
            report.errors.push(ConfigError {
                message: "acquisition.noncoherent_integration must be > 0".to_string(),
            });
        }
        if self.acquisition.peak_mean_threshold <= 0.0 {
            report.errors.push(ConfigError {
                message: "acquisition.peak_mean_threshold must be > 0".to_string(),
            });
        }
        if self.acquisition.peak_second_threshold <= 1.0 {
            report.errors.push(ConfigError {
                message: "acquisition.peak_second_threshold must be > 1".to_string(),
            });
        }
        if self.acquisition.threshold_policy.false_alarm_probability <= 0.0
            || self.acquisition.threshold_policy.false_alarm_probability >= 1.0
        {
            report.errors.push(ConfigError {
                message:
                    "acquisition.threshold_policy.false_alarm_probability must be within (0, 1)"
                        .to_string(),
            });
        }
        if self.acquisition.threshold_policy.calibration_trial_count == 0 {
            report.errors.push(ConfigError {
                message: "acquisition.threshold_policy.calibration_trial_count must be > 0"
                    .to_string(),
            });
        }
        if self.acquisition.threshold_policy.confidence_level <= 0.0
            || self.acquisition.threshold_policy.confidence_level >= 1.0
        {
            report.errors.push(ConfigError {
                message: "acquisition.threshold_policy.confidence_level must be within (0, 1)"
                    .to_string(),
            });
        }
        if self.tracking.early_late_spacing_chips <= 0.0 {
            report.errors.push(ConfigError {
                message: "tracking.early_late_spacing_chips must be > 0".to_string(),
            });
        }
        if self.tracking.max_channels == 0 {
            report
                .errors
                .push(ConfigError { message: "tracking.max_channels must be > 0".to_string() });
        }
        if self.tracking.per_epoch_budget_ms <= 0.0 {
            report.errors.push(ConfigError {
                message: "tracking.per_epoch_budget_ms must be > 0".to_string(),
            });
        }
        if self.tracking.over_budget_action != "drop_epochs"
            && self.tracking.over_budget_action != "continue"
        {
            report.errors.push(ConfigError {
                message: "tracking.over_budget_action must be drop_epochs or continue".to_string(),
            });
        }
        if self.tracking.integration_ms == 0 {
            report
                .errors
                .push(ConfigError { message: "tracking.integration_ms must be > 0".to_string() });
        }
        let mut seen = std::collections::BTreeSet::new();
        for band in &self.tracking.per_band {
            let parsed = match parse_band(&band.band) {
                Some(band) => band,
                None => {
                    report.errors.push(ConfigError {
                        message: format!("tracking.per_band has unknown band {}", band.band),
                    });
                    continue;
                }
            };
            if !seen.insert(parsed) {
                report.errors.push(ConfigError {
                    message: format!("tracking.per_band has duplicate entry for {:?}", parsed),
                });
            }
            if band.early_late_spacing_chips <= 0.0 {
                report.errors.push(ConfigError {
                    message: format!(
                        "tracking.per_band.{:?}.early_late_spacing_chips must be > 0",
                        parsed
                    ),
                });
            }
            if band.dll_bw_hz <= 0.0 || band.pll_bw_hz <= 0.0 || band.fll_bw_hz <= 0.0 {
                report.errors.push(ConfigError {
                    message: format!("tracking.per_band.{:?}.loop bandwidths must be > 0", parsed),
                });
            }
            if band.integration_ms == 0 {
                report.errors.push(ConfigError {
                    message: format!("tracking.per_band.{:?}.integration_ms must be > 0", parsed),
                });
            }
        }
        if self.navigation.huber_k <= 0.0 {
            report
                .errors
                .push(ConfigError { message: "navigation.huber_k must be > 0".to_string() });
        }
        if self.navigation.hatch_window == 0 {
            report
                .errors
                .push(ConfigError { message: "navigation.hatch_window must be > 0".to_string() });
        }
        if self.navigation.tropo_ztd_m < 0.0 {
            report
                .errors
                .push(ConfigError { message: "navigation.tropo_ztd_m must be >= 0".to_string() });
        }
        if self.navigation.weighting.min_elev_deg < 0.0
            || self.navigation.weighting.min_elev_deg > 90.0
        {
            report.errors.push(ConfigError {
                message: "navigation.weighting.min_elev_deg must be within [0, 90]".to_string(),
            });
        }
        if self.navigation.weighting.elev_exponent <= 0.0 {
            report.errors.push(ConfigError {
                message: "navigation.weighting.elev_exponent must be > 0".to_string(),
            });
        }
        if self.navigation.weighting.cn0_ref_dbhz <= 0.0 {
            report.errors.push(ConfigError {
                message: "navigation.weighting.cn0_ref_dbhz must be > 0".to_string(),
            });
        }
        if self.navigation.weighting.min_weight <= 0.0 {
            report.errors.push(ConfigError {
                message: "navigation.weighting.min_weight must be > 0".to_string(),
            });
        }
        if self.navigation.weighting.elev_mask_deg < 0.0
            || self.navigation.weighting.elev_mask_deg > 90.0
        {
            report.errors.push(ConfigError {
                message: "navigation.weighting.elev_mask_deg must be within [0, 90]".to_string(),
            });
        }
        if self.navigation.weighting.tracking_mode_scalar_weight <= 0.0 {
            report.errors.push(ConfigError {
                message: "navigation.weighting.tracking_mode_scalar_weight must be > 0".to_string(),
            });
        }
        if self.navigation.weighting.tracking_mode_vector_weight <= 0.0 {
            report.errors.push(ConfigError {
                message: "navigation.weighting.tracking_mode_vector_weight must be > 0".to_string(),
            });
        }
        if self.navigation.ppp.reset_gap_s <= 0.0 {
            report.errors.push(ConfigError {
                message: "navigation.ppp.reset_gap_s must be > 0".to_string(),
            });
        }
        if self.navigation.ppp.prune_after_epochs == 0 {
            report.errors.push(ConfigError {
                message: "navigation.ppp.prune_after_epochs must be > 0".to_string(),
            });
        }
        if self.navigation.ppp.residual_gate_m <= 0.0 {
            report.errors.push(ConfigError {
                message: "navigation.ppp.residual_gate_m must be > 0".to_string(),
            });
        }
        if self.navigation.ppp.drift_window_epochs == 0 {
            report.errors.push(ConfigError {
                message: "navigation.ppp.drift_window_epochs must be > 0".to_string(),
            });
        }
        if self.navigation.ppp.drift_threshold_m <= 0.0 {
            report.errors.push(ConfigError {
                message: "navigation.ppp.drift_threshold_m must be > 0".to_string(),
            });
        }
        let ppp = &self.navigation.ppp;
        validate_ppp_ar_mode(&mut report, &ppp.ar_mode);
        if !ppp.ar_ratio_threshold.is_finite() || ppp.ar_ratio_threshold <= 0.0 {
            report.errors.push(ConfigError {
                message: "navigation.ppp.ar_ratio_threshold must be finite and > 0".to_string(),
            });
        }
        if ppp.ar_stability_epochs == 0 {
            report.errors.push(ConfigError {
                message: "navigation.ppp.ar_stability_epochs must be > 0".to_string(),
            });
        }
        if ppp.ar_max_sats == 0 {
            report.errors.push(ConfigError {
                message: "navigation.ppp.ar_max_sats must be > 0".to_string(),
            });
        }
        for (field, value) in [
            ("navigation.ppp.noise_position", ppp.noise_position),
            ("navigation.ppp.noise_velocity", ppp.noise_velocity),
            ("navigation.ppp.noise_clock_bias", ppp.noise_clock_bias),
            ("navigation.ppp.noise_clock_drift", ppp.noise_clock_drift),
            ("navigation.ppp.noise_inter_system_bias", ppp.noise_inter_system_bias),
            ("navigation.ppp.noise_ztd", ppp.noise_ztd),
            ("navigation.ppp.noise_iono", ppp.noise_iono),
            ("navigation.ppp.noise_ambiguity", ppp.noise_ambiguity),
            ("navigation.ppp.measurement_code_floor_m", ppp.measurement_code_floor_m),
            ("navigation.ppp.measurement_phase_floor_cycles", ppp.measurement_phase_floor_cycles),
            ("navigation.ppp.measurement_orbit_sigma_scale", ppp.measurement_orbit_sigma_scale),
            ("navigation.ppp.measurement_clock_sigma_scale", ppp.measurement_clock_sigma_scale),
            (
                "navigation.ppp.measurement_troposphere_residual_m",
                ppp.measurement_troposphere_residual_m,
            ),
            ("navigation.ppp.measurement_antenna_residual_m", ppp.measurement_antenna_residual_m),
        ] {
            validate_finite_non_negative(&mut report, field, value);
        }
        for (field, value) in [
            (
                "navigation.ppp.precise_product_missing_satellite_action",
                ppp.precise_product_missing_satellite_action.as_str(),
            ),
            (
                "navigation.ppp.precise_product_out_of_coverage_action",
                ppp.precise_product_out_of_coverage_action.as_str(),
            ),
            (
                "navigation.ppp.precise_product_insufficient_support_action",
                ppp.precise_product_insufficient_support_action.as_str(),
            ),
            (
                "navigation.ppp.precise_product_orbit_gap_action",
                ppp.precise_product_orbit_gap_action.as_str(),
            ),
            (
                "navigation.ppp.precise_product_orbit_flag_action",
                ppp.precise_product_orbit_flag_action.as_str(),
            ),
            (
                "navigation.ppp.precise_product_clock_gap_action",
                ppp.precise_product_clock_gap_action.as_str(),
            ),
            (
                "navigation.ppp.precise_product_clock_jump_action",
                ppp.precise_product_clock_jump_action.as_str(),
            ),
        ] {
            validate_ppp_precise_product_action(&mut report, field, value);
        }
        if !ppp.precise_product_state_inflation.is_finite()
            || ppp.precise_product_state_inflation < 1.0
        {
            report.errors.push(ConfigError {
                message: "navigation.ppp.precise_product_state_inflation must be finite and >= 1"
                    .to_string(),
            });
        }
        if self.navigation.science_thresholds.min_mean_cn0_dbhz <= 0.0 {
            report.errors.push(ConfigError {
                message: "navigation.science_thresholds.min_mean_cn0_dbhz must be > 0".to_string(),
            });
        }
        if self.navigation.science_thresholds.max_pdop <= 0.0 {
            report.errors.push(ConfigError {
                message: "navigation.science_thresholds.max_pdop must be > 0".to_string(),
            });
        }
        if self.navigation.science_thresholds.max_gdop <= 0.0 {
            report.errors.push(ConfigError {
                message: "navigation.science_thresholds.max_gdop must be > 0".to_string(),
            });
        }
        if self.navigation.science_thresholds.max_residual_rms_m <= 0.0 {
            report.errors.push(ConfigError {
                message: "navigation.science_thresholds.max_residual_rms_m must be > 0".to_string(),
            });
        }
        if self.navigation.science_thresholds.min_used_satellites < 4 {
            report.errors.push(ConfigError {
                message: "navigation.science_thresholds.min_used_satellites must be >= 4"
                    .to_string(),
            });
        }
        if !(0.0..=1.0).contains(&self.navigation.science_thresholds.min_lock_ratio) {
            report.errors.push(ConfigError {
                message: "navigation.science_thresholds.min_lock_ratio must be within [0, 1]"
                    .to_string(),
            });
        }
        report
    }
}

impl ReceiverConfig {
    pub fn validate(&self) -> Result<(), Vec<String>> {
        let report = <Self as ValidateConfig>::validate(self);
        if report.errors.is_empty() {
            Ok(())
        } else {
            Err(report.errors.into_iter().map(|e| e.message).collect())
        }
    }

    pub fn to_pipeline_config(&self) -> ReceiverPipelineConfig {
        ReceiverPipelineConfig {
            sampling_freq_hz: self.sample_rate_hz,
            intermediate_freq_hz: self.intermediate_freq_hz,
            remove_dc_offset: self.front_end.remove_dc_offset,
            front_end_filter: self.front_end.filter.clone(),
            receiver_clock_bias_s: self.receiver_clock.bias_s,
            receiver_clock_frequency_bias_hz: self.receiver_clock.frequency_bias_hz,
            receiver_clock_bias_sigma_s: self.receiver_clock.bias_sigma_s,
            receiver_clock_source: self.receiver_clock.source.clone(),
            code_freq_basis_hz: self.code_freq_basis_hz,
            code_length: self.code_length,
            channels: self.tracking.max_channels,
            acquisition_doppler_search_hz: self.acquisition.doppler_search_hz,
            acquisition_doppler_step_hz: self.acquisition.doppler_step_hz,
            acquisition_doppler_rate_search_hz_per_s: self.acquisition.doppler_rate_search_hz_per_s,
            acquisition_doppler_rate_step_hz_per_s: self.acquisition.doppler_rate_step_hz_per_s,
            acquisition_integration_ms: self.acquisition.integration_ms,
            acquisition_noncoherent: self.acquisition.noncoherent_integration,
            acquisition_peak_mean_threshold: self.acquisition.peak_mean_threshold,
            acquisition_peak_second_threshold: self.acquisition.peak_second_threshold,
            acquisition_threshold_policy: self.acquisition.threshold_policy.clone(),
            early_late_spacing_chips: self.tracking.early_late_spacing_chips,
            dll_bw_hz: self.tracking.dll_bw_hz,
            pll_bw_hz: self.tracking.pll_bw_hz,
            fll_bw_hz: self.tracking.fll_bw_hz,
            adaptive_tracking_enabled: self.tracking.adaptive_tracking_enabled,
            vector_tracking_enabled: self.tracking.vector_tracking_enabled,
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
            tracking_budget_ms: self.tracking.per_epoch_budget_ms,
            tracking_over_budget_action: self.tracking.over_budget_action.clone(),
            robust_solver: self.navigation.robust_solver,
            huber_k: self.navigation.huber_k,
            raim: self.navigation.raim,
            position_solution_smoothing: self.navigation.position_solution_smoothing,
            position_solution_motion_class: self.navigation.position_solution_motion_class,
            hatch_window: self.navigation.hatch_window,
            weighting: self.navigation.weighting.clone(),
            iono_mode: self.navigation.iono_mode.clone(),
            tropo_enable: self.navigation.tropo_enable,
            tropo_ztd_m: self.navigation.tropo_ztd_m,
            ppp: self.navigation.ppp.clone(),
            science_thresholds: self.navigation.science_thresholds.clone(),
            constellation_policy: self.navigation.constellation_policy,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::receiver_config::navigation::{
        ConstellationSelectionPolicy, NavigationMotionClass, NavigationWeightingMode,
    };
    use crate::engine::receiver_config::AcquisitionThresholdMode;
    use bijux_gnss_core::api::Constellation;
    use schemars::schema_for;

    #[test]
    fn validation_rejects_negative_doppler_search_range() {
        let mut config = ReceiverConfig::default();
        config.acquisition.doppler_search_hz = -500;

        let report = <ReceiverConfig as ValidateConfig>::validate(&config);

        assert!(report
            .errors
            .iter()
            .any(|error| { error.message == "acquisition.doppler_search_hz must be >= 0" }));
    }

    #[test]
    fn validation_rejects_unaligned_doppler_grid() {
        let mut config = ReceiverConfig::default();
        config.acquisition.doppler_search_hz = 1_250;
        config.acquisition.doppler_step_hz = 500;

        let report = <ReceiverConfig as ValidateConfig>::validate(&config);

        assert!(report.errors.iter().any(|error| {
            error.message
                == "acquisition.doppler_search_hz must be an integer multiple of acquisition.doppler_step_hz"
        }));
    }

    #[test]
    fn validation_allows_zero_doppler_search_range() {
        let mut config = ReceiverConfig::default();
        config.acquisition.doppler_search_hz = 0;
        config.acquisition.doppler_step_hz = 500;

        let report = <ReceiverConfig as ValidateConfig>::validate(&config);

        assert!(
            !report
                .errors
                .iter()
                .any(|error| error.message.contains("acquisition.doppler_search_hz")),
            "unexpected Doppler-search validation error: {:?}",
            report.errors
        );
    }

    #[test]
    fn validation_rejects_unsupported_acquisition_coherent_integration() {
        let mut config = ReceiverConfig::default();
        config.acquisition.integration_ms = 3;

        let report = <ReceiverConfig as ValidateConfig>::validate(&config);

        assert!(report.errors.iter().any(|error| {
            error.message == "acquisition.integration_ms must be one of [1, 2, 5, 10, 20]"
        }));
    }

    #[test]
    fn validation_rejects_invalid_calibrated_acquisition_threshold_policy() {
        let mut config = ReceiverConfig::default();
        config.acquisition.threshold_policy.mode = AcquisitionThresholdMode::CalibratedFalseAlarm;
        config.acquisition.threshold_policy.false_alarm_probability = 1.0;
        config.acquisition.threshold_policy.calibration_trial_count = 0;
        config.acquisition.threshold_policy.confidence_level = 0.0;

        let report = <ReceiverConfig as ValidateConfig>::validate(&config);

        assert!(report.errors.iter().any(|error| {
            error.message
                == "acquisition.threshold_policy.false_alarm_probability must be within (0, 1)"
        }));
        assert!(report.errors.iter().any(|error| {
            error.message == "acquisition.threshold_policy.calibration_trial_count must be > 0"
        }));
        assert!(report.errors.iter().any(|error| {
            error.message == "acquisition.threshold_policy.confidence_level must be within (0, 1)"
        }));
    }

    #[test]
    fn validation_rejects_invalid_ppp_stochastic_configuration() {
        let mut config = ReceiverConfig::default();
        config.navigation.ppp.noise_position = -0.1;
        config.navigation.ppp.measurement_phase_floor_cycles = f64::NAN;
        config.navigation.ppp.measurement_clock_sigma_scale = f64::INFINITY;

        let report = <ReceiverConfig as ValidateConfig>::validate(&config);

        assert!(report.errors.iter().any(|error| {
            error.message == "navigation.ppp.noise_position must be finite and >= 0"
        }));
        assert!(report.errors.iter().any(|error| {
            error.message == "navigation.ppp.measurement_phase_floor_cycles must be finite and >= 0"
        }));
        assert!(report.errors.iter().any(|error| {
            error.message == "navigation.ppp.measurement_clock_sigma_scale must be finite and >= 0"
        }));
    }

    #[test]
    fn validation_rejects_invalid_ppp_precise_product_policy() {
        let mut config = ReceiverConfig::default();
        config.navigation.ppp.precise_product_clock_jump_action = "consume_anyway".to_string();
        config.navigation.ppp.precise_product_state_inflation = 0.5;

        let report = <ReceiverConfig as ValidateConfig>::validate(&config);

        assert!(report.errors.iter().any(|error| {
            error.message
                == "navigation.ppp.precise_product_clock_jump_action must be one of [bridge_with_broadcast, inflate_satellite_state, reset_satellite_state, refuse_satellite]"
        }));
        assert!(report.errors.iter().any(|error| {
            error.message
                == "navigation.ppp.precise_product_state_inflation must be finite and >= 1"
        }));
    }

    #[test]
    fn validation_rejects_invalid_ppp_ambiguity_resolution_configuration() {
        let mut config = ReceiverConfig::default();
        config.navigation.ppp.ar_mode = "integer_now".to_string();
        config.navigation.ppp.ar_ratio_threshold = f64::NAN;
        config.navigation.ppp.ar_stability_epochs = 0;
        config.navigation.ppp.ar_max_sats = 0;

        let report = <ReceiverConfig as ValidateConfig>::validate(&config);

        assert!(report.errors.iter().any(|error| {
            error.message
                == "navigation.ppp.ar_mode must be one of [float_ppp, ppp_ar_wide_lane, ppp_ar_narrow_lane]"
        }));
        assert!(report.errors.iter().any(|error| {
            error.message == "navigation.ppp.ar_ratio_threshold must be finite and > 0"
        }));
        assert!(report
            .errors
            .iter()
            .any(|error| { error.message == "navigation.ppp.ar_stability_epochs must be > 0" }));
        assert!(report
            .errors
            .iter()
            .any(|error| { error.message == "navigation.ppp.ar_max_sats must be > 0" }));
    }

    #[test]
    fn ppp_stochastic_configuration_round_trips_through_toml() {
        let mut config = ReceiverConfig::default();
        config.navigation.ppp.noise_position = 0.12;
        config.navigation.ppp.noise_velocity = 0.034;
        config.navigation.ppp.noise_clock_bias = 2.0e-7;
        config.navigation.ppp.noise_inter_system_bias = 3.0e-9;
        config.navigation.ppp.measurement_code_floor_m = 0.42;
        config.navigation.ppp.measurement_phase_floor_cycles = 0.002;
        config.navigation.ppp.measurement_orbit_sigma_scale = 1.4;
        config.navigation.ppp.measurement_clock_sigma_scale = 1.6;
        config.navigation.ppp.measurement_troposphere_residual_m = 0.08;
        config.navigation.ppp.measurement_antenna_residual_m = 0.02;

        let raw = toml::to_string(&config).expect("serialize receiver config");
        let reparsed: ReceiverConfig = toml::from_str(&raw).expect("parse receiver config");

        assert_eq!(reparsed.navigation.ppp.noise_position, 0.12);
        assert_eq!(reparsed.navigation.ppp.noise_velocity, 0.034);
        assert_eq!(reparsed.navigation.ppp.noise_clock_bias, 2.0e-7);
        assert_eq!(reparsed.navigation.ppp.noise_inter_system_bias, 3.0e-9);
        assert_eq!(reparsed.navigation.ppp.measurement_code_floor_m, 0.42);
        assert_eq!(reparsed.navigation.ppp.measurement_phase_floor_cycles, 0.002);
        assert_eq!(reparsed.navigation.ppp.measurement_orbit_sigma_scale, 1.4);
        assert_eq!(reparsed.navigation.ppp.measurement_clock_sigma_scale, 1.6);
        assert_eq!(reparsed.navigation.ppp.measurement_troposphere_residual_m, 0.08);
        assert_eq!(reparsed.navigation.ppp.measurement_antenna_residual_m, 0.02);
        assert!(raw.contains("measurement_code_floor_m = 0.42"));
    }

    #[test]
    fn ppp_precise_product_policy_round_trips_through_toml() {
        let mut config = ReceiverConfig::default();
        config.navigation.ppp.precise_product_missing_satellite_action =
            "inflate_satellite_state".to_string();
        config.navigation.ppp.precise_product_out_of_coverage_action =
            "bridge_with_broadcast".to_string();
        config.navigation.ppp.precise_product_insufficient_support_action =
            "reset_satellite_state".to_string();
        config.navigation.ppp.precise_product_orbit_gap_action =
            "inflate_satellite_state".to_string();
        config.navigation.ppp.precise_product_orbit_flag_action = "refuse_satellite".to_string();
        config.navigation.ppp.precise_product_clock_gap_action =
            "reset_satellite_state".to_string();
        config.navigation.ppp.precise_product_clock_jump_action = "refuse_satellite".to_string();
        config.navigation.ppp.precise_product_state_inflation = 25.0;

        let raw = toml::to_string(&config).expect("serialize receiver config");
        let reparsed: ReceiverConfig = toml::from_str(&raw).expect("parse receiver config");

        assert_eq!(
            reparsed.navigation.ppp.precise_product_missing_satellite_action,
            "inflate_satellite_state"
        );
        assert_eq!(
            reparsed.navigation.ppp.precise_product_insufficient_support_action,
            "reset_satellite_state"
        );
        assert_eq!(
            reparsed.navigation.ppp.precise_product_orbit_gap_action,
            "inflate_satellite_state"
        );
        assert_eq!(reparsed.navigation.ppp.precise_product_orbit_flag_action, "refuse_satellite");
        assert_eq!(reparsed.navigation.ppp.precise_product_clock_jump_action, "refuse_satellite");
        assert_eq!(reparsed.navigation.ppp.precise_product_state_inflation, 25.0);
        assert!(raw.contains("precise_product_clock_jump_action = \"refuse_satellite\""));
    }

    #[test]
    fn ppp_stochastic_defaults_apply_to_existing_toml_profiles() {
        let raw = toml::to_string(&ReceiverConfig::default()).expect("serialize receiver config");
        let omitted_stochastic_fields = [
            "noise_position",
            "noise_velocity",
            "noise_clock_bias",
            "noise_clock_drift",
            "noise_inter_system_bias",
            "noise_ztd",
            "noise_iono",
            "noise_ambiguity",
            "measurement_code_floor_m",
            "measurement_phase_floor_cycles",
            "measurement_orbit_sigma_scale",
            "measurement_clock_sigma_scale",
            "measurement_troposphere_residual_m",
            "measurement_antenna_residual_m",
        ];
        let legacy_raw = raw
            .lines()
            .filter(|line| {
                let trimmed = line.trim_start();
                !omitted_stochastic_fields
                    .iter()
                    .any(|field| trimmed.starts_with(&format!("{field} =")))
            })
            .collect::<Vec<_>>()
            .join("\n");

        let reparsed: ReceiverConfig =
            toml::from_str(&legacy_raw).expect("parse receiver config without stochastic fields");

        assert_eq!(
            reparsed.navigation.ppp.noise_position,
            ReceiverConfig::default().navigation.ppp.noise_position
        );
        assert_eq!(
            reparsed.navigation.ppp.noise_velocity,
            ReceiverConfig::default().navigation.ppp.noise_velocity
        );
        assert_eq!(
            reparsed.navigation.ppp.noise_clock_bias,
            ReceiverConfig::default().navigation.ppp.noise_clock_bias
        );
        assert_eq!(
            reparsed.navigation.ppp.noise_inter_system_bias,
            ReceiverConfig::default().navigation.ppp.noise_inter_system_bias
        );
        assert_eq!(
            reparsed.navigation.ppp.measurement_code_floor_m,
            ReceiverConfig::default().navigation.ppp.measurement_code_floor_m
        );
        assert_eq!(
            reparsed.navigation.ppp.measurement_phase_floor_cycles,
            ReceiverConfig::default().navigation.ppp.measurement_phase_floor_cycles
        );
        assert_eq!(
            reparsed.navigation.ppp.measurement_orbit_sigma_scale,
            ReceiverConfig::default().navigation.ppp.measurement_orbit_sigma_scale
        );
        assert_eq!(
            reparsed.navigation.ppp.measurement_clock_sigma_scale,
            ReceiverConfig::default().navigation.ppp.measurement_clock_sigma_scale
        );
        assert_eq!(
            reparsed.navigation.ppp.measurement_troposphere_residual_m,
            ReceiverConfig::default().navigation.ppp.measurement_troposphere_residual_m
        );
        assert_eq!(
            reparsed.navigation.ppp.measurement_antenna_residual_m,
            ReceiverConfig::default().navigation.ppp.measurement_antenna_residual_m
        );
    }

    #[test]
    fn ppp_precise_product_policy_defaults_apply_to_existing_toml_profiles() {
        let raw = toml::to_string(&ReceiverConfig::default()).expect("serialize receiver config");
        let omitted_policy_fields = [
            "precise_product_missing_satellite_action",
            "precise_product_out_of_coverage_action",
            "precise_product_insufficient_support_action",
            "precise_product_orbit_gap_action",
            "precise_product_orbit_flag_action",
            "precise_product_clock_gap_action",
            "precise_product_clock_jump_action",
            "precise_product_state_inflation",
        ];
        let legacy_raw = raw
            .lines()
            .filter(|line| {
                let trimmed = line.trim_start();
                !omitted_policy_fields
                    .iter()
                    .any(|field| trimmed.starts_with(&format!("{field} =")))
            })
            .collect::<Vec<_>>()
            .join("\n");

        let reparsed: ReceiverConfig =
            toml::from_str(&legacy_raw).expect("parse receiver config without product policy");
        let defaults = ReceiverConfig::default();

        assert_eq!(
            reparsed.navigation.ppp.precise_product_missing_satellite_action,
            defaults.navigation.ppp.precise_product_missing_satellite_action
        );
        assert_eq!(
            reparsed.navigation.ppp.precise_product_orbit_flag_action,
            defaults.navigation.ppp.precise_product_orbit_flag_action
        );
        assert_eq!(
            reparsed.navigation.ppp.precise_product_clock_jump_action,
            defaults.navigation.ppp.precise_product_clock_jump_action
        );
        assert_eq!(
            reparsed.navigation.ppp.precise_product_state_inflation,
            defaults.navigation.ppp.precise_product_state_inflation
        );
    }

    #[test]
    fn default_navigation_constellation_policy_is_mixed() {
        let config = ReceiverConfig::default();

        assert_eq!(config.navigation.constellation_policy, ConstellationSelectionPolicy::Mixed);
        assert!(config.navigation.position_solution_smoothing);
        assert_eq!(
            config.navigation.position_solution_motion_class,
            NavigationMotionClass::Vehicle
        );
    }

    #[test]
    fn pipeline_config_carries_constellation_policy_helpers() {
        let mut config = ReceiverConfig::default();
        config.navigation.constellation_policy = ConstellationSelectionPolicy::GlonassOnly;

        let pipeline = config.to_pipeline_config();

        assert_eq!(pipeline.constellation_policy, ConstellationSelectionPolicy::GlonassOnly);
        assert!(pipeline.allows_constellation(Constellation::Glonass));
        assert!(!pipeline.allows_constellation(Constellation::Gps));
        assert_eq!(pipeline.selected_constellations(), &[Constellation::Glonass]);
    }

    #[test]
    fn constellation_policy_round_trips_through_toml() {
        let mut config = ReceiverConfig::default();
        config.navigation.constellation_policy = ConstellationSelectionPolicy::GalileoOnly;

        let raw = toml::to_string(&config).expect("serialize receiver config");
        let reparsed: ReceiverConfig = toml::from_str(&raw).expect("parse receiver config");

        assert_eq!(
            reparsed.navigation.constellation_policy,
            ConstellationSelectionPolicy::GalileoOnly
        );
        assert!(raw.contains("constellation_policy = \"galileo_only\""));
    }

    #[test]
    fn adaptive_tracking_toggle_round_trips_through_toml() {
        let mut config = ReceiverConfig::default();
        config.tracking.adaptive_tracking_enabled = false;

        let raw = toml::to_string(&config).expect("serialize receiver config");
        let reparsed: ReceiverConfig = toml::from_str(&raw).expect("parse receiver config");

        assert!(!reparsed.tracking.adaptive_tracking_enabled);
        assert!(raw.contains("adaptive_tracking_enabled = false"));
    }

    #[test]
    fn vector_tracking_toggle_round_trips_through_toml() {
        let mut config = ReceiverConfig::default();
        config.tracking.vector_tracking_enabled = false;

        let raw = toml::to_string(&config).expect("serialize receiver config");
        let reparsed: ReceiverConfig = toml::from_str(&raw).expect("parse receiver config");
        let pipeline = reparsed.to_pipeline_config();

        assert!(!reparsed.tracking.vector_tracking_enabled);
        assert!(!pipeline.vector_tracking_enabled);
        assert!(raw.contains("vector_tracking_enabled = false"));
    }

    #[test]
    fn receiver_clock_model_round_trips_into_pipeline_config() {
        let mut config = ReceiverConfig::default();
        config.receiver_clock.bias_s = 1.25e-6;
        config.receiver_clock.frequency_bias_hz = -42.0;
        config.receiver_clock.bias_sigma_s = 2.5e-8;
        config.receiver_clock.source = "rinex_reference_clock".to_string();

        let raw = toml::to_string(&config).expect("serialize receiver config");
        let reparsed: ReceiverConfig = toml::from_str(&raw).expect("parse receiver config");
        let pipeline = reparsed.to_pipeline_config();

        assert!((pipeline.receiver_clock_bias_s - 1.25e-6).abs() <= f64::EPSILON);
        assert!((pipeline.receiver_clock_frequency_bias_hz + 42.0).abs() <= f64::EPSILON);
        assert!((pipeline.receiver_clock_bias_sigma_s - 2.5e-8).abs() <= f64::EPSILON);
        assert_eq!(pipeline.receiver_clock_source, "rinex_reference_clock");
        assert!(raw.contains("[receiver_clock]"));
        assert!(raw.contains("frequency_bias_hz = -42.0"));
    }

    #[test]
    fn receiver_clock_model_rejects_invalid_terms() {
        let mut config = ReceiverConfig::default();
        config.receiver_clock.bias_s = f64::NAN;
        config.receiver_clock.frequency_bias_hz = f64::INFINITY;
        config.receiver_clock.bias_sigma_s = -1.0;
        config.receiver_clock.source = " ".to_string();

        let report = <ReceiverConfig as ValidateConfig>::validate(&config);
        let messages = report.errors.iter().map(|error| error.message.as_str()).collect::<Vec<_>>();

        assert!(messages.iter().any(|message| *message == "receiver_clock.bias_s must be finite"));
        assert!(messages
            .iter()
            .any(|message| *message == "receiver_clock.frequency_bias_hz must be finite"));
        assert!(messages
            .iter()
            .any(|message| *message == "receiver_clock.bias_sigma_s must be finite and >= 0"));
        assert!(messages
            .iter()
            .any(|message| *message == "receiver_clock.source must not be empty"));
    }

    #[test]
    fn receiver_config_schema_lists_constellation_policy_modes() {
        let schema = schema_for!(ReceiverConfig);
        let raw = serde_json::to_string(&schema).expect("serialize receiver config schema");

        for value in ["gps_only", "galileo_only", "glonass_only", "beidou_only", "mixed"] {
            assert!(
                raw.contains(value),
                "receiver config schema missing constellation policy value {value}: {raw}"
            );
        }
    }

    #[test]
    fn navigation_weighting_mode_round_trips_through_toml() {
        let mut config = ReceiverConfig::default();
        config.navigation.weighting.mode = NavigationWeightingMode::ElevationCn0;

        let raw = toml::to_string(&config).expect("serialize receiver config");
        let reparsed: ReceiverConfig = toml::from_str(&raw).expect("parse receiver config");

        assert_eq!(reparsed.navigation.weighting.mode, NavigationWeightingMode::ElevationCn0);
        assert!(raw.contains("mode = \"elevation_cn0\""));
    }

    #[test]
    fn receiver_config_schema_lists_navigation_weighting_modes() {
        let schema = schema_for!(ReceiverConfig);
        let raw = serde_json::to_string(&schema).expect("serialize receiver config schema");

        for value in ["elevation", "cn0", "elevation_cn0"] {
            assert!(
                raw.contains(value),
                "receiver config schema missing navigation weighting mode value {value}: {raw}"
            );
        }
    }

    #[test]
    fn acquisition_threshold_mode_round_trips_through_toml() {
        let mut config = ReceiverConfig::default();
        config.acquisition.threshold_policy.mode = AcquisitionThresholdMode::CalibratedFalseAlarm;

        let raw = toml::to_string(&config).expect("serialize receiver config");
        let reparsed: ReceiverConfig = toml::from_str(&raw).expect("parse receiver config");

        assert_eq!(
            reparsed.acquisition.threshold_policy.mode,
            AcquisitionThresholdMode::CalibratedFalseAlarm
        );
        assert!(raw.contains("mode = \"calibrated_false_alarm\""));
    }

    #[test]
    fn receiver_config_schema_lists_acquisition_threshold_modes() {
        let schema = schema_for!(ReceiverConfig);
        let raw = serde_json::to_string(&schema).expect("serialize receiver config schema");

        for value in ["fixed_ratio", "calibrated_false_alarm"] {
            assert!(
                raw.contains(value),
                "receiver config schema missing acquisition threshold mode value {value}: {raw}"
            );
        }
    }

    #[test]
    fn position_solution_smoothing_round_trips_through_toml() {
        let mut config = ReceiverConfig::default();
        config.navigation.position_solution_smoothing = false;
        config.navigation.position_solution_motion_class = NavigationMotionClass::Pedestrian;

        let raw = toml::to_string(&config).expect("serialize receiver config");
        let reparsed: ReceiverConfig = toml::from_str(&raw).expect("parse receiver config");

        assert!(!reparsed.navigation.position_solution_smoothing);
        assert_eq!(
            reparsed.navigation.position_solution_motion_class,
            NavigationMotionClass::Pedestrian
        );
        assert!(raw.contains("position_solution_smoothing = false"));
        assert!(raw.contains("position_solution_motion_class = \"pedestrian\""));
    }
}
