//! Sweep-parameter override helpers.

use bijux_gnss_receiver::api::core::InputError;
use bijux_gnss_receiver::api::{NavigationWeightingMode, ReceiverConfig};

/// Apply sweep parameter overrides.
pub(crate) fn apply_sweep_value(
    profile: &mut ReceiverConfig,
    key: &str,
    value: &str,
) -> Result<(), InputError> {
    match key {
        "tracking.dll_bw_hz" => profile.tracking.dll_bw_hz = value.parse().map_err(map_err)?,
        "tracking.pll_bw_hz" => profile.tracking.pll_bw_hz = value.parse().map_err(map_err)?,
        "tracking.fll_bw_hz" => profile.tracking.fll_bw_hz = value.parse().map_err(map_err)?,
        "tracking.early_late_spacing_chips" => {
            profile.tracking.early_late_spacing_chips = value.parse().map_err(map_err)?
        }
        "acquisition.integration_ms" => {
            profile.acquisition.integration_ms = value.parse().map_err(map_err)?
        }
        "acquisition.doppler_step_hz" => {
            profile.acquisition.doppler_step_hz = value.parse().map_err(map_err)?
        }
        "acquisition.doppler_search_hz" => {
            profile.acquisition.doppler_search_hz = value.parse().map_err(map_err)?
        }
        "acquisition.peak_mean_threshold" => {
            profile.acquisition.peak_mean_threshold = value.parse().map_err(map_err)?
        }
        "acquisition.peak_second_threshold" => {
            profile.acquisition.peak_second_threshold = value.parse().map_err(map_err)?
        }
        "navigation.hatch_window" => {
            profile.navigation.hatch_window = value.parse().map_err(map_err)?
        }
        "navigation.weighting.elev_mask_deg" => {
            profile.navigation.weighting.elev_mask_deg = value.parse().map_err(map_err)?
        }
        "navigation.weighting.mode" => {
            profile.navigation.weighting.mode = match value {
                "elevation" => NavigationWeightingMode::Elevation,
                "cn0" => NavigationWeightingMode::Cn0,
                "elevation_cn0" => NavigationWeightingMode::ElevationCn0,
                _ => {
                    return Err(InputError {
                        message: format!("unsupported navigation.weighting.mode: {value}"),
                    });
                }
            }
        }
        "navigation.weighting.elev_exponent" => {
            profile.navigation.weighting.elev_exponent = value.parse().map_err(map_err)?
        }
        "navigation.weighting.cn0_ref_dbhz" => {
            profile.navigation.weighting.cn0_ref_dbhz = value.parse().map_err(map_err)?
        }
        "navigation.weighting.min_weight" => {
            profile.navigation.weighting.min_weight = value.parse().map_err(map_err)?
        }
        "navigation.weighting.tracking_mode_scalar_weight" => {
            profile.navigation.weighting.tracking_mode_scalar_weight =
                value.parse().map_err(map_err)?
        }
        "navigation.weighting.tracking_mode_vector_weight" => {
            profile.navigation.weighting.tracking_mode_vector_weight =
                value.parse().map_err(map_err)?
        }
        "navigation.robust_solver" => {
            profile.navigation.robust_solver = value.parse().map_err(map_err)?
        }
        "navigation.raim" => profile.navigation.raim = value.parse().map_err(map_err)?,
        _ => return Err(InputError { message: format!("unsupported sweep parameter: {key}") }),
    }
    Ok(())
}

fn map_err(err: impl std::fmt::Display) -> InputError {
    InputError { message: err.to_string() }
}

#[cfg(test)]
mod tests {
    use super::apply_sweep_value;
    use bijux_gnss_receiver::api::ReceiverConfig;

    #[test]
    fn unsupported_sweep_parameter_returns_an_explicit_error() {
        let mut profile = ReceiverConfig::default();
        let err = apply_sweep_value(&mut profile, "navigation.unknown", "1")
            .expect_err("unsupported parameter must fail");
        assert!(err.message.contains("unsupported sweep parameter"));
    }
}
