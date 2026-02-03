use std::collections::{BTreeMap, BTreeSet};

use bijux_gnss_core::{Constellation, ObsEpoch, ObsSatellite, SatId, SigId, SignalBand};

use crate::ekf::{Ekf, EkfConfig, MeasurementKind, MeasurementModel, StateModel};
use crate::linalg::Matrix;
use crate::{
    elevation_azimuth_deg, sat_state_gps_l1ca, weight_from_cn0_elev, CodeBiasProvider,
    CorrectionContext, Corrections, GpsEphemeris, GpsSatState, PhaseBiasProvider, ProductsProvider,
    WeightingConfig, ZeroBiases,
};

const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;

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

#[derive(Debug, Clone)]
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

#[derive(Debug, Clone)]
pub struct PppConvergenceState {
    pub converged: bool,
    pub time_to_first_meter_s: Option<f64>,
    pub time_to_decimeter_s: Option<f64>,
    pub time_to_centimeter_s: Option<f64>,
    pub last_position_change_m: Option<f64>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
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
struct PppIndices {
    pos: [usize; 3],
    vel: [usize; 3],
    clock_bias: usize,
    clock_drift: usize,
    ztd: usize,
    isb: BTreeMap<Constellation, usize>,
    iono: BTreeMap<SatId, usize>,
    ambiguity: BTreeMap<SigId, usize>,
}

pub struct PppFilter {
    pub ekf: Ekf,
    pub config: PppConfig,
    indices: PppIndices,
    last_t_rx_s: Option<f64>,
    last_pos: Option<[f64; 3]>,
    epoch0_t_s: Option<f64>,
    last_seen_iono: BTreeMap<SatId, u64>,
    last_seen_amb: BTreeMap<SigId, u64>,
    residual_history: BTreeMap<SigId, Vec<f64>>,
    drift_history: Vec<[f64; 3]>,
    wl_state: BTreeMap<SatId, WlAmbiguity>,
    ar_stable_epochs: u32,
    pub health: PppHealth,
    code_bias: Box<dyn CodeBiasProvider + Send + Sync>,
    phase_bias: Box<dyn PhaseBiasProvider + Send + Sync>,
    corrections: CorrectionContext,
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
struct WlAmbiguity {
    float_cycles: f64,
    variance: f64,
    fixed: bool,
    last_update_epoch: u64,
}

