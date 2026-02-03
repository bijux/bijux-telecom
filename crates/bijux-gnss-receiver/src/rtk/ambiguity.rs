#![allow(missing_docs)]

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

#[derive(Debug, Clone)]
pub struct DecorrelatedAmbiguities {
    pub z: Vec<Vec<i64>>,
    pub n_prime: Vec<f64>,
    pub q_prime: Vec<Vec<f64>>,
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

pub fn decorrelate_lambda(float: &FloatAmbiguitySolution) -> DecorrelatedAmbiguities {
    let n = float.float_cycles.len();
    let mut z = vec![vec![0_i64; n]; n];
    for (i, row) in z.iter_mut().enumerate() {
        row[i] = 1;
    }
    DecorrelatedAmbiguities {
        z,
        n_prime: float.float_cycles.clone(),
        q_prime: float.covariance.clone(),
    }
}

#[derive(Debug, Clone)]
pub struct IntegerCandidate {
    pub integers: Vec<i64>,
    pub cost: f64,
}

pub fn search_integer_candidates(
    n_prime: &[f64],
    q_prime: &[Vec<f64>],
    top_k: usize,
) -> Vec<IntegerCandidate> {
    if n_prime.is_empty() {
        return Vec::new();
    }
    let mut best = Vec::new();
    let mut base = Vec::new();
    for &v in n_prime {
        base.push(v.round() as i64);
    }
    let cost1 = candidate_cost(n_prime, q_prime, &base);
    best.push(IntegerCandidate {
        integers: base.clone(),
        cost: cost1,
    });
    let mut neighbor = base.clone();
    let mut max_idx = 0;
    let mut max_res = 0.0;
    for (i, &v) in n_prime.iter().enumerate() {
        let res = (v - v.round()).abs();
        if res > max_res {
            max_res = res;
            max_idx = i;
        }
    }
    if max_res > 0.0 {
        let dir = if n_prime[max_idx] - n_prime[max_idx].round() >= 0.0 {
            1
        } else {
            -1
        };
        neighbor[max_idx] += dir;
        let cost2 = candidate_cost(n_prime, q_prime, &neighbor);
        best.push(IntegerCandidate {
            integers: neighbor,
            cost: cost2,
        });
    }
    best.sort_by(|a, b| {
        a.cost
            .partial_cmp(&b.cost)
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    best.truncate(top_k.max(1));
    best
}

fn candidate_cost(n_prime: &[f64], q_prime: &[Vec<f64>], integers: &[i64]) -> f64 {
    let mut cost = 0.0;
    for i in 0..n_prime.len() {
        let var = q_prime
            .get(i)
            .and_then(|row| row.get(i))
            .copied()
            .unwrap_or(1.0)
            .max(1e-6);
        let res = n_prime[i] - integers[i] as f64;
        cost += res * res / var;
    }
    cost
}

pub fn ratio_from_candidates(cands: &[IntegerCandidate]) -> Option<f64> {
    if cands.len() < 2 {
        return None;
    }
    let c1 = cands[0].cost;
    let c2 = cands[1].cost;
    if c1 <= 0.0 {
        None
    } else {
        Some(c2 / c1)
    }
}

pub fn select_partial_fix(
    float: &FloatAmbiguitySolution,
    max_count: usize,
) -> FloatAmbiguitySolution {
    if float.float_cycles.is_empty() {
        return float.clone();
    }
    let mut idxs: Vec<usize> = (0..float.float_cycles.len()).collect();
    idxs.sort_by(|a, b| {
        let va = float.covariance[*a][*a];
        let vb = float.covariance[*b][*b];
        va.partial_cmp(&vb).unwrap_or(std::cmp::Ordering::Equal)
    });
    let keep = max_count.max(1).min(float.float_cycles.len());
    let mut ids = Vec::new();
    let mut floats = Vec::new();
    let mut cov = Vec::new();
    for &i in idxs.iter().take(keep) {
        ids.push(float.ids[i].clone());
        floats.push(float.float_cycles[i]);
    }
    for &i in idxs.iter().take(keep) {
        let mut row = Vec::new();
        for &j in idxs.iter().take(keep) {
            row.push(float.covariance[i][j]);
        }
        cov.push(row);
    }
    FloatAmbiguitySolution {
        ids,
        float_cycles: floats,
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

        let decor = decorrelate_lambda(float);
        let mut candidates = search_integer_candidates(&decor.n_prime, &decor.q_prime, 2);
        let mut ratio = ratio_from_candidates(&candidates);
        let mut accepted = ratio
            .map(|r| ratio_test(r, &self.policy, state))
            .unwrap_or(false);
        let mut fixed_count = if accepted { decor.n_prime.len() } else { 0 };
        let mut reason = if accepted { "accepted" } else { "ratio_fail" };

        if !accepted && float.float_cycles.len() > 1 {
            let partial = select_partial_fix(float, float.float_cycles.len() / 2);
            let decor_p = decorrelate_lambda(&partial);
            candidates = search_integer_candidates(&decor_p.n_prime, &decor_p.q_prime, 2);
            ratio = ratio_from_candidates(&candidates);
            if let Some(r) = ratio {
                accepted = ratio_test(r, &self.policy, state);
                if accepted {
                    fixed_count = decor_p.n_prime.len();
                    reason = "partial_fix";
                }
            }
        }

        if ratio.is_none() {
            reason = "no_ratio";
        }

        let result = AmbiguityFixResult {
            candidates: candidates.iter().map(|c| c.integers.clone()).collect(),
            ratio,
            accepted,
        };
        let audit = FixAuditEvent {
            epoch_idx,
            ratio,
            accepted,
            reason: reason.to_string(),
            fixed_count,
        };
        (result, audit)
    }
}
