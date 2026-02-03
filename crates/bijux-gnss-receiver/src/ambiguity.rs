use std::collections::BTreeMap;

use bijux_gnss_core::{AmbiguityId, AmbiguityState, AmbiguityStatus, ObsSatellite};
use bijux_gnss_nav::Matrix;

#[derive(Debug, Clone)]
pub struct AmbiguityManager {
    pub states: BTreeMap<AmbiguityId, AmbiguityState>,
}

impl AmbiguityManager {
    pub fn new() -> Self {
        Self {
            states: BTreeMap::new(),
        }
    }

    pub fn update_from_obs(&mut self, epoch_idx: u64, sats: &[ObsSatellite]) {
        for sat in sats {
            let id = AmbiguityId {
                constellation: sat.metadata.signal.constellation,
                prn: sat.prn,
                band: sat.metadata.signal.band,
                signal: format!("{:?}", sat.metadata.signal.band),
            };
            let state = self.states.entry(id.clone()).or_insert(AmbiguityState {
                id,
                float_cycles: sat.carrier_phase_cycles,
                variance: 100.0,
                status: AmbiguityStatus::Float,
                last_update_epoch: epoch_idx,
            });
            if sat.lock_flags.cycle_slip || !sat.lock_flags.carrier_lock {
                state.status = AmbiguityStatus::Unknown;
                state.float_cycles = sat.carrier_phase_cycles;
                state.variance = 100.0;
            } else {
                state.float_cycles = sat.carrier_phase_cycles;
                state.last_update_epoch = epoch_idx;
            }
        }
    }
}

impl Default for AmbiguityManager {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Debug, Clone)]
pub struct FloatAmbiguitySolution {
    pub ids: Vec<AmbiguityId>,
    pub float_cycles: Vec<f64>,
    pub covariance: Vec<Vec<f64>>,
}

pub fn float_from_state(
    ids: Vec<AmbiguityId>,
    indices: Vec<usize>,
    state: &[f64],
    covariance: &Matrix,
) -> FloatAmbiguitySolution {
    let mut float_cycles = Vec::new();
    for &idx in &indices {
        float_cycles.push(state.get(idx).copied().unwrap_or(0.0));
    }
    let sub = covariance.submatrix(&indices, &indices);
    let mut cov = Vec::new();
    for r in 0..sub.rows() {
        let mut row = Vec::new();
        for c in 0..sub.cols() {
            row.push(sub[(r, c)]);
        }
        cov.push(row);
    }
    FloatAmbiguitySolution {
        ids,
        float_cycles,
        covariance: cov,
    }
}

pub trait AmbiguityFixer {
    fn fix(&self, float: &FloatAmbiguitySolution) -> AmbiguityFixResult;
}

#[derive(Debug, Clone)]
pub struct AmbiguityFixResult {
    pub candidates: Vec<Vec<i64>>,
    pub ratio: Option<f64>,
    pub accepted: bool,
}

#[derive(Debug, Clone)]
pub struct FixPolicy {
    pub ratio_threshold: f64,
    pub consecutive_required: usize,
}

impl Default for FixPolicy {
    fn default() -> Self {
        Self {
            ratio_threshold: 3.0,
            consecutive_required: 3,
        }
    }
}

pub struct DummyFixer;

impl AmbiguityFixer for DummyFixer {
    fn fix(&self, _float: &FloatAmbiguitySolution) -> AmbiguityFixResult {
        AmbiguityFixResult {
            candidates: Vec::new(),
            ratio: None,
            accepted: false,
        }
    }
}

#[derive(Debug, Clone, Default)]
pub struct FixState {
    pub consecutive_accepts: usize,
    pub fixed: bool,
}

pub fn ratio_test(ratio: f64, policy: &FixPolicy, state: &mut FixState) -> bool {
    if ratio >= policy.ratio_threshold {
        state.consecutive_accepts += 1;
    } else {
        state.consecutive_accepts = 0;
        state.fixed = false;
    }
    if state.consecutive_accepts >= policy.consecutive_required {
        state.fixed = true;
    }
    state.fixed
}
