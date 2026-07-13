#![allow(missing_docs)]

use crate::engine::receiver_config::{
    acquisition_integration_ms_is_supported, parse_band, supported_acquisition_integration_ms_csv,
    BandTrackingSpec, ReceiverConfig, ReceiverPipelineConfig,
};
use bijux_gnss_core::api::{ConfigError, SchemaVersion, ValidateConfig, ValidationReport};

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
            code_freq_basis_hz: self.code_freq_basis_hz,
            code_length: self.code_length,
            channels: self.tracking.max_channels,
            acquisition_doppler_search_hz: self.acquisition.doppler_search_hz,
            acquisition_doppler_step_hz: self.acquisition.doppler_step_hz,
            acquisition_integration_ms: self.acquisition.integration_ms,
            acquisition_noncoherent: self.acquisition.noncoherent_integration,
            acquisition_peak_mean_threshold: self.acquisition.peak_mean_threshold,
            acquisition_peak_second_threshold: self.acquisition.peak_second_threshold,
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
    use crate::engine::receiver_config::{
        ConstellationSelectionPolicy, NavigationMotionClass, NavigationWeightingMode,
    };
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
