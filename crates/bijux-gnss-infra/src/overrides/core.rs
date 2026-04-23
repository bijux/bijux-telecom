//! Configuration override helpers.

use bijux_gnss_receiver::api::core::InputError;
use bijux_gnss_receiver::api::ReceiverConfig;

use crate::overrides::CommonOverrides;

/// Apply config overrides from CLI options.
pub(crate) fn apply_overrides(
    profile: &mut ReceiverConfig,
    sampling_hz: Option<f64>,
    if_hz: Option<f64>,
    code_hz: Option<f64>,
    code_length: Option<usize>,
) {
    if let Some(value) = sampling_hz {
        profile.sample_rate_hz = value;
    }
    if let Some(value) = if_hz {
        profile.intermediate_freq_hz = value;
    }
    if let Some(value) = code_hz {
        profile.code_freq_basis_hz = value;
    }
    if let Some(value) = code_length {
        profile.code_length = value;
    }
}

/// Apply seed/determinism overrides.
pub(crate) fn apply_common_overrides(profile: &mut ReceiverConfig, common: CommonOverrides) {
    if let Some(seed) = common.seed {
        profile.seed = seed;
    }
    if common.deterministic && common.seed.is_none() {
        profile.seed = 1;
    }
}

/// Apply sweep parameter overrides.
pub(crate) fn apply_sweep_value(
    profile: &mut ReceiverConfig,
    key: &str,
    value: &str,
) -> Result<(), InputError> {
    match key {
        "tracking.dll_bw_hz" => profile.tracking.dll_bw_hz = value.parse().map_err(map)?,
        "tracking.pll_bw_hz" => profile.tracking.pll_bw_hz = value.parse().map_err(map)?,
        "tracking.fll_bw_hz" => profile.tracking.fll_bw_hz = value.parse().map_err(map)?,
        "tracking.early_late_spacing_chips" => {
            profile.tracking.early_late_spacing_chips = value.parse().map_err(map)?
        }
        "acquisition.integration_ms" => {
            profile.acquisition.integration_ms = value.parse().map_err(map)?
        }
        "acquisition.doppler_step_hz" => {
            profile.acquisition.doppler_step_hz = value.parse().map_err(map)?
        }
        "acquisition.doppler_search_hz" => {
            profile.acquisition.doppler_search_hz = value.parse().map_err(map)?
        }
        "acquisition.peak_mean_threshold" => {
            profile.acquisition.peak_mean_threshold = value.parse().map_err(map)?
        }
        "acquisition.peak_second_threshold" => {
            profile.acquisition.peak_second_threshold = value.parse().map_err(map)?
        }
        "navigation.hatch_window" => {
            profile.navigation.hatch_window = value.parse().map_err(map)?
        }
        "navigation.weighting.elev_mask_deg" => {
            profile.navigation.weighting.elev_mask_deg = value.parse().map_err(map)?
        }
        "navigation.weighting.elev_exponent" => {
            profile.navigation.weighting.elev_exponent = value.parse().map_err(map)?
        }
        "navigation.weighting.cn0_ref_dbhz" => {
            profile.navigation.weighting.cn0_ref_dbhz = value.parse().map_err(map)?
        }
        "navigation.weighting.min_weight" => {
            profile.navigation.weighting.min_weight = value.parse().map_err(map)?
        }
        "navigation.weighting.tracking_mode_scalar_weight" => {
            profile.navigation.weighting.tracking_mode_scalar_weight = value.parse().map_err(map)?
        }
        "navigation.weighting.tracking_mode_vector_weight" => {
            profile.navigation.weighting.tracking_mode_vector_weight = value.parse().map_err(map)?
        }
        "navigation.robust_solver" => {
            profile.navigation.robust_solver = value.parse().map_err(map)?
        }
        "navigation.raim" => profile.navigation.raim = value.parse().map_err(map)?,
        _ => return Err(InputError { message: format!("unsupported sweep parameter: {key}") }),
    }
    Ok(())
}

fn map(err: impl std::fmt::Display) -> InputError {
    InputError { message: err.to_string() }
}
