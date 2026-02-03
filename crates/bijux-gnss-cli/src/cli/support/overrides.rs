
fn apply_overrides(
    profile: &mut ReceiverProfile,
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

fn apply_common_overrides(profile: &mut ReceiverProfile, common: &CommonArgs) {
    if let Some(seed) = common.seed {
        profile.seed = seed;
    }
    if common.deterministic && common.seed.is_none() {
        profile.seed = 1;
    }
}

fn apply_sweep_value(profile: &mut ReceiverProfile, key: &str, value: &str) -> Result<()> {
    match key {
        "tracking.dll_bw_hz" => profile.tracking.dll_bw_hz = value.parse()?,
        "tracking.pll_bw_hz" => profile.tracking.pll_bw_hz = value.parse()?,
        "tracking.fll_bw_hz" => profile.tracking.fll_bw_hz = value.parse()?,
        "tracking.early_late_spacing_chips" => {
            profile.tracking.early_late_spacing_chips = value.parse()?
        }
        "acquisition.integration_ms" => profile.acquisition.integration_ms = value.parse()?,
        "acquisition.doppler_step_hz" => profile.acquisition.doppler_step_hz = value.parse()?,
        "acquisition.doppler_search_hz" => profile.acquisition.doppler_search_hz = value.parse()?,
        "acquisition.peak_mean_threshold" => {
            profile.acquisition.peak_mean_threshold = value.parse()?
        }
        "acquisition.peak_second_threshold" => {
            profile.acquisition.peak_second_threshold = value.parse()?
        }
        "navigation.hatch_window" => profile.navigation.hatch_window = value.parse()?,
        "navigation.weighting.elev_mask_deg" => {
            profile.navigation.weighting.elev_mask_deg = value.parse()?
        }
        "navigation.weighting.elev_exponent" => {
            profile.navigation.weighting.elev_exponent = value.parse()?
        }
        "navigation.weighting.cn0_ref_dbhz" => {
            profile.navigation.weighting.cn0_ref_dbhz = value.parse()?
        }
        "navigation.weighting.min_weight" => {
            profile.navigation.weighting.min_weight = value.parse()?
        }
        "navigation.weighting.tracking_mode_scalar_weight" => {
            profile.navigation.weighting.tracking_mode_scalar_weight = value.parse()?
        }
        "navigation.weighting.tracking_mode_vector_weight" => {
            profile.navigation.weighting.tracking_mode_vector_weight = value.parse()?
        }
        "navigation.robust_solver" => profile.navigation.robust_solver = value.parse()?,
        "navigation.raim" => profile.navigation.raim = value.parse()?,
        _ => bail!("unsupported sweep parameter: {key}"),
    }
    Ok(())
}
