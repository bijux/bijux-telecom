//! Receiver profile override helpers.

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

#[cfg(test)]
mod tests {
    use super::apply_common_overrides;
    use crate::overrides::CommonOverrides;
    use bijux_gnss_receiver::api::ReceiverConfig;

    #[test]
    fn deterministic_mode_supplies_a_default_seed() {
        let mut profile = ReceiverConfig::default();
        apply_common_overrides(&mut profile, CommonOverrides { seed: None, deterministic: true });
        assert_eq!(profile.seed, 1);
    }
}
