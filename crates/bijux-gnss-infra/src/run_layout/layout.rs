//! Run directory layout helpers.

/// Run directory layout.
#[derive(Debug, Clone)]
pub struct RunDirLayout {
    /// Root run directory.
    pub run_dir: std::path::PathBuf,
    /// Artifacts directory.
    pub artifacts_dir: std::path::PathBuf,
    /// Logs directory.
    pub logs_dir: std::path::PathBuf,
    /// Summary output path.
    pub summary_path: std::path::PathBuf,
    /// Manifest output path.
    pub manifest_path: std::path::PathBuf,
}

impl RunDirLayout {
    /// Create a layout from a run directory.
    pub fn new(run_dir: std::path::PathBuf) -> Self {
        let artifacts_dir = run_dir.join("artifacts");
        let logs_dir = run_dir.join("logs");
        let summary_path = run_dir.join("summary.json");
        let manifest_path = run_dir.join("manifest.json");
        Self { run_dir, artifacts_dir, logs_dir, summary_path, manifest_path }
    }

    /// Create directories on disk.
    pub fn create(&self) -> Result<(), bijux_gnss_receiver::api::core::InputError> {
        std::fs::create_dir_all(&self.artifacts_dir).map_err(map_err)?;
        std::fs::create_dir_all(&self.logs_dir).map_err(map_err)?;
        Ok(())
    }
}

fn map_err(err: impl std::fmt::Display) -> bijux_gnss_receiver::api::core::InputError {
    bijux_gnss_receiver::api::core::InputError { message: err.to_string() }
}
