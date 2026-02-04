pub(crate) mod core;

/// Hash a config file or profile snapshot.
pub fn hash_config(
    path: Option<&std::path::PathBuf>,
    profile: &bijux_gnss_receiver::api::ReceiverProfile,
) -> Result<String, bijux_gnss_receiver::api::core::InputError> {
    core::hash_config(path, profile)
}

/// Return current git hash if available.
pub fn git_hash() -> Option<String> {
    core::git_hash()
}

/// Return true when git workspace is dirty.
pub fn git_dirty() -> bool {
    core::git_dirty()
}

/// CPU feature detection summary.
pub fn cpu_features() -> Vec<String> {
    core::cpu_features()
}
