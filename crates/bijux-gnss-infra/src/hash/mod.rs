mod provenance;

/// Hash a config file or profile snapshot.
pub fn hash_config(
    path: Option<&std::path::PathBuf>,
    profile: &bijux_gnss_receiver::api::ReceiverConfig,
) -> Result<String, bijux_gnss_receiver::api::core::InputError> {
    provenance::hash_config(path, profile)
}

/// Return current git hash if available.
pub fn git_hash() -> Option<String> {
    provenance::git_hash()
}

/// Return true when git workspace is dirty.
pub fn git_dirty() -> bool {
    provenance::git_dirty()
}

/// CPU feature detection summary.
pub fn cpu_features() -> Vec<String> {
    provenance::cpu_features()
}
