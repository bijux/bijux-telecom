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
                sig: sat.signal_id,
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

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct FixAuditEvent {
    pub epoch_idx: u64,
    pub ratio: Option<f64>,
    pub accepted: bool,
    pub reason: String,
    pub fixed_count: usize,
}

pub struct NaiveFixer {
    pub policy: FixPolicy,
}

impl NaiveFixer {
    pub fn new(policy: FixPolicy) -> Self {
        Self { policy }
    }

    pub fn fix_with_state(
        &self,
        epoch_idx: u64,
        float: &FloatAmbiguitySolution,
        state: &mut FixState,
    ) -> (AmbiguityFixResult, FixAuditEvent) {
        if float.float_cycles.is_empty() {
            let audit = FixAuditEvent {
                epoch_idx,
                ratio: None,
                accepted: false,
                reason: "no_ambiguities".to_string(),
                fixed_count: 0,
            };
            return (
                AmbiguityFixResult {
                    candidates: Vec::new(),
                    ratio: None,
                    accepted: false,
                },
                audit,
            );
        }

        let mut candidate = Vec::new();
        let mut residuals = Vec::new();
        for &val in &float.float_cycles {
            let rounded = val.round();
            candidate.push(rounded as i64);
            residuals.push(val - rounded);
        }
        let mut weights = Vec::new();
        for i in 0..float.covariance.len() {
            let var = float.covariance[i][i].abs().max(1e-6);
            weights.push(1.0 / var);
        }
        let cost1: f64 = residuals
            .iter()
            .zip(weights.iter())
            .map(|(r, w)| r * r * w)
            .sum();
        let mut candidate2 = candidate.clone();
        if let Some((idx, _)) = residuals.iter().enumerate().max_by(|a, b| {
            a.1.abs()
                .partial_cmp(&b.1.abs())
                .unwrap_or(std::cmp::Ordering::Equal)
        }) {
            candidate2[idx] += if residuals[idx] >= 0.0 { 1 } else { -1 };
        }
        let mut residuals2 = Vec::new();
        for (val, &c) in float.float_cycles.iter().zip(candidate2.iter()) {
            residuals2.push(val - c as f64);
        }
        let cost2: f64 = residuals2
            .iter()
            .zip(weights.iter())
            .map(|(r, w)| r * r * w)
            .sum();
        let ratio = if cost1 > 0.0 {
            Some(cost2 / cost1)
        } else {
            None
        };

        let accepted = if let Some(r) = ratio {
            ratio_test(r, &self.policy, state)
        } else {
            false
        };

        let reason = if accepted {
            "accepted"
        } else if ratio.is_none() {
            "no_ratio"
        } else {
            "ratio_fail"
        };

        let result = AmbiguityFixResult {
            candidates: vec![candidate, candidate2],
            ratio,
            accepted,
        };
        let audit = FixAuditEvent {
            epoch_idx,
            ratio,
            accepted,
            reason: reason.to_string(),
            fixed_count: if accepted {
                float.float_cycles.len()
            } else {
                0
            },
        };
        (result, audit)
    }
}
