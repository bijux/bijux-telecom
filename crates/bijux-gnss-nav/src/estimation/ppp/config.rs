#![allow(missing_docs)]

use std::collections::BTreeMap;

use bijux_gnss_core::{Constellation, SatId, SigId};
use serde::{Deserialize, Serialize};

use bijux_gnss_core::{ArtifactPayloadValidate, DiagnosticEvent, DiagnosticSeverity};

use crate::corrections::biases::{CodeBiasProvider, PhaseBiasProvider};
use crate::corrections::CorrectionContext;
use crate::estimation::ekf::state::Ekf;
use crate::estimation::position::solver::WeightingConfig;

pub const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;

#[derive(Debug, Clone)]
pub struct PppProcessNoise {
    pub clock_drift_s: f64,
    pub ztd_m: f64,
    pub iono_m: f64,
    pub ambiguity_cycles: f64,
}

#[derive(Debug, Clone)]
pub struct PppConvergenceConfig {
    pub min_time_s: f64,
    pub pos_rate_mps: f64,
    pub sigma_h_m: f64,
    pub sigma_v_m: f64,
}

#[derive(Debug, Clone)]
pub struct PppConfig {
    pub enable_iono_state: bool,
    pub use_iono_free: bool,
    pub use_doppler: bool,
    pub ar_mode: PppArMode,
    pub ar_ratio_threshold: f64,
    pub ar_stability_epochs: u32,
    pub ar_max_sats: usize,
    pub ar_use_elevation: bool,
    pub prune_after_epochs: u64,
    pub reset_gap_s: f64,
    pub residual_gate_m: f64,
    pub drift_window_epochs: usize,
    pub drift_threshold_m: f64,
    pub checkpoint_interval_epochs: u64,
    pub process_noise: PppProcessNoise,
    pub weighting: WeightingConfig,
    pub convergence: PppConvergenceConfig,
}

impl Default for PppConfig {
    fn default() -> Self {
        Self {
            enable_iono_state: false,
            use_iono_free: false,
            use_doppler: false,
            ar_mode: PppArMode::FloatPpp,
            ar_ratio_threshold: 3.0,
            ar_stability_epochs: 3,
            ar_max_sats: 8,
            ar_use_elevation: true,
            prune_after_epochs: 200,
            reset_gap_s: 2.0,
            residual_gate_m: 200.0,
            drift_window_epochs: 100,
            drift_threshold_m: 10.0,
            checkpoint_interval_epochs: 0,
            process_noise: PppProcessNoise {
                clock_drift_s: 1e-5,
                ztd_m: 0.01,
                iono_m: 0.1,
                ambiguity_cycles: 0.05,
            },
            weighting: WeightingConfig::default(),
            convergence: PppConvergenceConfig {
                min_time_s: 60.0,
                pos_rate_mps: 0.1,
                sigma_h_m: 1.0,
                sigma_v_m: 2.0,
            },
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PppSolutionEpoch {
    pub epoch_idx: u64,
    pub t_rx_s: f64,
    pub ecef_x_m: f64,
    pub ecef_y_m: f64,
    pub ecef_z_m: f64,
    pub clock_bias_s: f64,
    pub rms_m: f64,
    pub sigma_h_m: Option<f64>,
    pub sigma_v_m: Option<f64>,
    pub innovation_rms: f64,
    pub convergence: PppConvergenceState,
    pub residuals: Vec<(SigId, f64)>,
    pub nis_mean: Option<f64>,
    pub ar_mode: PppArMode,
    pub fixed_wl: usize,
}

impl ArtifactPayloadValidate for PppSolutionEpoch {
    fn validate_payload(&self) -> Vec<DiagnosticEvent> {
        let mut events = Vec::new();
        if !self.ecef_x_m.is_finite()
            || !self.ecef_y_m.is_finite()
            || !self.ecef_z_m.is_finite()
            || !self.clock_bias_s.is_finite()
        {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Error,
                "PPP_EPOCH_INVALID",
                "PPP solution contains NaN/Inf",
            ));
        }
        events
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PppConvergenceState {
    pub converged: bool,
    pub time_to_first_meter_s: Option<f64>,
    pub time_to_decimeter_s: Option<f64>,
    pub time_to_centimeter_s: Option<f64>,
    pub last_position_change_m: Option<f64>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum PppArMode {
    FloatPpp,
    PppArWideLane,
    PppArNarrowLane,
}

#[derive(Debug, Clone)]
pub struct PppHealth {
    pub last_reset_reason: Option<String>,
    pub pruned_states: usize,
    pub convergence: PppConvergenceState,
    pub warnings: Vec<String>,
    pub nis_mean: Option<f64>,
    pub ar_events: Vec<String>,
}

#[derive(Debug, Clone)]
pub struct PppIndices {
    pub pos: [usize; 3],
    pub vel: [usize; 3],
    pub clock_bias: usize,
    pub clock_drift: usize,
    pub ztd: usize,
    pub isb: BTreeMap<Constellation, usize>,
    pub iono: BTreeMap<SatId, usize>,
    pub ambiguity: BTreeMap<SigId, usize>,
}

pub struct PppFilter {
    pub ekf: Ekf,
    pub config: PppConfig,
    pub indices: PppIndices,
    pub last_t_rx_s: Option<f64>,
    pub last_pos: Option<[f64; 3]>,
    pub epoch0_t_s: Option<f64>,
    pub last_seen_iono: BTreeMap<SatId, u64>,
    pub last_seen_amb: BTreeMap<SigId, u64>,
    pub residual_history: BTreeMap<SigId, Vec<f64>>,
    pub drift_history: Vec<[f64; 3]>,
    pub wl_state: BTreeMap<SatId, WlAmbiguity>,
    pub ar_stable_epochs: u32,
    pub health: PppHealth,
    pub code_bias: Box<dyn CodeBiasProvider + Send + Sync>,
    pub phase_bias: Box<dyn PhaseBiasProvider + Send + Sync>,
    pub corrections: CorrectionContext,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct PppCheckpoint {
    pub x: Vec<f64>,
    pub p: Vec<Vec<f64>>,
    pub indices_isb: Vec<(Constellation, usize)>,
    pub indices_iono: Vec<(SatId, usize)>,
    pub indices_amb: Vec<(SigId, usize)>,
    pub last_t_rx_s: Option<f64>,
    pub epoch0_t_s: Option<f64>,
    pub last_pos: Option<[f64; 3]>,
}

#[derive(Debug, Clone)]
pub struct WlAmbiguity {
    pub float_cycles: f64,
    pub variance: f64,
    pub fixed: bool,
    pub last_update_epoch: u64,
}
