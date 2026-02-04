pub(crate) mod core;

/// Common override values from CLI.
#[derive(Debug, Clone, Copy)]
pub struct CommonOverrides {
    /// Deterministic seed override.
    pub seed: Option<u64>,
    /// Force deterministic run.
    pub deterministic: bool,
}

/// Apply config overrides from CLI options.
pub fn apply_overrides(
    profile: &mut bijux_gnss_receiver::api::ReceiverProfile,
    sampling_hz: Option<f64>,
    if_hz: Option<f64>,
    code_hz: Option<f64>,
    code_length: Option<usize>,
) {
    core::apply_overrides(profile, sampling_hz, if_hz, code_hz, code_length);
}

/// Apply seed/determinism overrides.
pub fn apply_common_overrides(
    profile: &mut bijux_gnss_receiver::api::ReceiverProfile,
    common: CommonOverrides,
) {
    core::apply_common_overrides(profile, common);
}

/// Apply sweep parameter overrides.
pub fn apply_sweep_value(
    profile: &mut bijux_gnss_receiver::api::ReceiverProfile,
    key: &str,
    value: &str,
) -> Result<(), bijux_gnss_receiver::api::core::InputError> {
    core::apply_sweep_value(profile, key, value)
}
