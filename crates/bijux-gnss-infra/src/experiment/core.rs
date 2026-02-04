//! Experiment and sweep specifications.

use serde::{Deserialize, Serialize};

/// Sweep parameter definition.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SweepParameter {
    /// Parameter key.
    pub key: String,
    /// Values to sweep.
    pub values: Vec<String>,
}

/// Experiment specification for batch runs.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExperimentSpec {
    /// Dataset or scenario id.
    pub dataset_id: String,
    /// Optional config path.
    pub config_path: Option<String>,
    /// Sweep parameters.
    pub sweep: Vec<SweepParameter>,
    /// Deterministic seed.
    pub seed: Option<u64>,
    /// Output directory.
    pub out_dir: Option<String>,
}
