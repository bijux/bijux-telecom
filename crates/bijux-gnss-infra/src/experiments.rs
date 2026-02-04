//! Experiment-related helpers for the infra API.
#![allow(missing_docs)]

/// Sweep parameter definition.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct SweepParameter {
    /// Parameter key.
    pub key: String,
    /// Values to sweep.
    pub values: Vec<String>,
}

/// Experiment specification for batch runs.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
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
