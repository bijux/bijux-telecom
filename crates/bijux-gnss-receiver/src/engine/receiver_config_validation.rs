#![allow(missing_docs)]

use bijux_gnss_core::api::{ConfigError, SchemaVersion, ValidateConfig, ValidationReport};

use crate::engine::receiver_config::{
    parse_band, BandTrackingSpec, ReceiverConfig, ReceiverPipelineConfig,
};

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
        if self.acquisition.integration_ms == 0 {
            report.errors.push(ConfigError {
                message: "acquisition.integration_ms must be > 0".to_string(),
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
            hatch_window: self.navigation.hatch_window,
            weighting: self.navigation.weighting.clone(),
            iono_mode: self.navigation.iono_mode.clone(),
            tropo_enable: self.navigation.tropo_enable,
            tropo_ztd_m: self.navigation.tropo_ztd_m,
            ppp: self.navigation.ppp.clone(),
        }
    }
}
