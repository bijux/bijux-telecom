use std::collections::BTreeMap;

use bijux_gnss_core::api::{
    AmbiguityId, AmbiguityState, AmbiguityStatus, ArtifactPayloadValidate, DiagnosticEvent,
    DiagnosticSeverity, ObsSatellite, SigId,
};
use serde::{Deserialize, Serialize};

use super::baseline::RtkFloatBaselineSolution;
use crate::linalg::Matrix;

/// One double-difference ambiguity identifier keyed by signal and reference signal.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub struct RtkDoubleDifferenceAmbiguityId {
    /// Signal identity for the non-reference satellite.
    pub sig: SigId,
    /// Signal identity for the reference satellite.
    pub ref_sig: SigId,
}

/// Float ambiguity state ordered for integer candidate search.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RtkFloatAmbiguityState {
    /// Ordered ambiguity identifiers.
    pub ids: Vec<RtkDoubleDifferenceAmbiguityId>,
    /// Float ambiguity estimates in cycles.
    pub float_cycles: Vec<f64>,
    /// Full covariance matrix in square cycles.
    pub covariance_cycles2: Vec<Vec<f64>>,
}

impl ArtifactPayloadValidate for RtkFloatAmbiguityState {
    fn validate_payload(&self) -> Vec<DiagnosticEvent> {
        let mut events = Vec::new();
        if self.ids.len() != self.float_cycles.len()
            || self.covariance_cycles2.len() != self.ids.len()
            || self.covariance_cycles2.iter().any(|row| row.len() != self.ids.len())
        {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Error,
                "RTK_FLOAT_STATE_SHAPE_INVALID",
                "float ambiguity state dimensions do not match",
            ));
        }
        if !self.float_cycles.iter().all(|value| value.is_finite())
            || !self
                .covariance_cycles2
                .iter()
                .flat_map(|row| row.iter())
                .all(|value| value.is_finite())
        {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Error,
                "RTK_FLOAT_STATE_NUMERIC_INVALID",
                "float ambiguity state contains NaN/Inf",
            ));
        }
        events
    }
}

/// Decorrelated ambiguity state used during integer candidate search.
#[derive(Debug, Clone)]
pub struct RtkDecorrelatedAmbiguityState {
    pub z: Vec<Vec<i64>>,
    pub n_prime: Vec<f64>,
    pub q_prime: Vec<Vec<f64>>,
}

/// One integer ambiguity candidate and its quadratic cost.
#[derive(Debug, Clone, PartialEq)]
pub struct RtkIntegerAmbiguityCandidate {
    pub integers: Vec<i64>,
    pub cost: f64,
}

/// Ratio-test policy for RTK ambiguity fixing.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RtkAmbiguityFixPolicy {
    pub ratio_threshold: f64,
    pub consecutive_required: usize,
}

impl Default for RtkAmbiguityFixPolicy {
    fn default() -> Self {
        Self { ratio_threshold: 3.0, consecutive_required: 3 }
    }
}

/// Stateful acceptance history for consecutive ratio-test decisions.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct RtkAmbiguityFixState {
    pub consecutive_accepts: usize,
    pub fixed: bool,
}

/// Fix outcome class emitted by the ambiguity fixer.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum RtkAmbiguityFixStatus {
    Float,
    Fixed,
    Failed,
}

/// Result of one ambiguity-fixing attempt.
#[derive(Debug, Clone, PartialEq)]
pub struct RtkAmbiguityFixResult {
    pub candidates: Vec<RtkIntegerAmbiguityCandidate>,
    pub ratio: Option<f64>,
    pub status: RtkAmbiguityFixStatus,
    pub fixed_count: usize,
    pub selected_ids: Option<Vec<RtkDoubleDifferenceAmbiguityId>>,
    pub selected_integers: Option<Vec<i64>>,
}

impl RtkAmbiguityFixResult {
    pub fn accepted(&self) -> bool {
        self.status == RtkAmbiguityFixStatus::Fixed
    }
}

/// Serializable audit event for one ambiguity-fixing attempt.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct RtkAmbiguityFixAudit {
    pub epoch_idx: u64,
    pub ratio: Option<f64>,
    pub status: RtkAmbiguityFixStatus,
    pub reason: String,
    pub fixed_count: usize,
}

impl ArtifactPayloadValidate for RtkAmbiguityFixAudit {
    fn validate_payload(&self) -> Vec<DiagnosticEvent> {
        let mut events = Vec::new();
        if let Some(ratio) = self.ratio {
            if !ratio.is_finite() {
                events.push(DiagnosticEvent::new(
                    DiagnosticSeverity::Error,
                    "RTK_FIX_AUDIT_NUMERIC_INVALID",
                    "rtk ambiguity fix audit ratio contains NaN/Inf",
                ));
            }
        }
        events
    }
}

/// Ratio-test ambiguity fixer operating on the float baseline solution.
#[derive(Debug, Clone)]
pub struct RtkRatioTestFixer {
    pub policy: RtkAmbiguityFixPolicy,
}

impl RtkRatioTestFixer {
    pub fn new(policy: RtkAmbiguityFixPolicy) -> Self {
        Self { policy }
    }

    pub fn fix_with_state(
        &self,
        epoch_idx: u64,
        float: &RtkFloatAmbiguityState,
        state: &mut RtkAmbiguityFixState,
    ) -> (RtkAmbiguityFixResult, RtkAmbiguityFixAudit) {
        if float.float_cycles.is_empty() {
            let result = RtkAmbiguityFixResult {
                candidates: Vec::new(),
                ratio: None,
                status: RtkAmbiguityFixStatus::Failed,
                fixed_count: 0,
                selected_ids: None,
                selected_integers: None,
            };
            let audit = RtkAmbiguityFixAudit {
                epoch_idx,
                ratio: None,
                status: result.status,
                reason: "no_ambiguities".to_string(),
                fixed_count: 0,
            };
            return (result, audit);
        }

        let decorrelated = rtk_lambda_decorrelate(float);
        let mut candidates =
            rtk_integer_ambiguity_candidates(&decorrelated.n_prime, &decorrelated.q_prime, 2);
        let mut ratio = rtk_candidate_ratio(&candidates);
        let mut status = if ratio
            .map(|candidate_ratio| rtk_ratio_test_acceptance(candidate_ratio, &self.policy, state))
            .unwrap_or(false)
        {
            RtkAmbiguityFixStatus::Fixed
        } else {
            RtkAmbiguityFixStatus::Failed
        };
        let mut fixed_count =
            if status == RtkAmbiguityFixStatus::Fixed { float.float_cycles.len() } else { 0 };
        let mut reason =
            if status == RtkAmbiguityFixStatus::Fixed { "accepted" } else { "ratio_fail" };

        if status != RtkAmbiguityFixStatus::Fixed && float.float_cycles.len() > 1 {
            let partial = rtk_select_partial_ambiguity_fix(float, float.float_cycles.len() / 2);
            let decorrelated_partial = rtk_lambda_decorrelate(&partial);
            candidates = rtk_integer_ambiguity_candidates(
                &decorrelated_partial.n_prime,
                &decorrelated_partial.q_prime,
                2,
            );
            ratio = rtk_candidate_ratio(&candidates);
            if let Some(candidate_ratio) = ratio {
                if rtk_ratio_test_acceptance(candidate_ratio, &self.policy, state) {
                    status = RtkAmbiguityFixStatus::Fixed;
                    fixed_count = partial.float_cycles.len();
                    reason = "partial_fix";
                }
            }
        }

        if ratio.is_none() {
            reason = "no_ratio";
        }

        let selected_ids = if status == RtkAmbiguityFixStatus::Fixed {
            if fixed_count == float.ids.len() {
                Some(float.ids.clone())
            } else {
                Some(rtk_select_partial_ambiguity_fix(float, fixed_count).ids)
            }
        } else {
            None
        };
        let selected_integers = if status == RtkAmbiguityFixStatus::Fixed {
            candidates.first().map(|candidate| candidate.integers.clone())
        } else {
            None
        };
        let result = RtkAmbiguityFixResult {
            candidates,
            ratio,
            status,
            fixed_count,
            selected_ids,
            selected_integers,
        };
        let audit = RtkAmbiguityFixAudit {
            epoch_idx,
            ratio: result.ratio,
            status: result.status,
            reason: reason.to_string(),
            fixed_count,
        };
        (result, audit)
    }
}

/// Baseline estimate conditioned on accepted integer ambiguities.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct RtkConditionedBaselineSolution {
    pub enu_m: [f64; 3],
    pub covariance_enu_m2: [[f64; 3]; 3],
}

/// Receiver-owned tracker for per-signal ambiguity state before double differencing.
#[derive(Debug, Clone)]
pub struct RtkAmbiguityTracker {
    pub states: BTreeMap<AmbiguityId, AmbiguityState>,
}

impl RtkAmbiguityTracker {
    pub fn new() -> Self {
        Self { states: BTreeMap::new() }
    }

    pub fn update_from_obs(&mut self, epoch_idx: u64, sats: &[ObsSatellite]) {
        for sat in sats {
            let id = AmbiguityId {
                sig: sat.signal_id,
                signal: format!("{:?}", sat.metadata.signal.band),
            };
            let valid_arc_id = sat
                .metadata
                .carrier_phase_arc
                .as_ref()
                .filter(|arc| arc.valid_for_ambiguity)
                .map(|arc| arc.id.clone());
            let carrier_phase_usable =
                sat.lock_flags.carrier_lock && !sat.lock_flags.cycle_slip && valid_arc_id.is_some();
            let state = self.states.entry(id.clone()).or_insert(AmbiguityState {
                id,
                float_cycles: sat.carrier_phase_cycles,
                variance: 100.0,
                status: if carrier_phase_usable {
                    AmbiguityStatus::Float
                } else {
                    AmbiguityStatus::Unknown
                },
                last_update_epoch: epoch_idx,
                carrier_phase_arc_id: valid_arc_id.clone(),
                valid_for_carrier_phase_arc: carrier_phase_usable,
            });
            let arc_changed = state.carrier_phase_arc_id != valid_arc_id;
            if !carrier_phase_usable {
                state.status = AmbiguityStatus::Unknown;
                state.float_cycles = sat.carrier_phase_cycles;
                state.variance = 100.0;
                state.carrier_phase_arc_id = valid_arc_id;
                state.valid_for_carrier_phase_arc = false;
                state.last_update_epoch = epoch_idx;
            } else if arc_changed {
                state.status = AmbiguityStatus::Float;
                state.float_cycles = sat.carrier_phase_cycles;
                state.variance = 100.0;
                state.carrier_phase_arc_id = valid_arc_id;
                state.valid_for_carrier_phase_arc = true;
                state.last_update_epoch = epoch_idx;
            } else {
                if state.status == AmbiguityStatus::Unknown {
                    state.status = AmbiguityStatus::Float;
                    state.variance = 100.0;
                }
                state.float_cycles = sat.carrier_phase_cycles;
                state.valid_for_carrier_phase_arc = true;
                state.last_update_epoch = epoch_idx;
            }
        }
    }
}

impl Default for RtkAmbiguityTracker {
    fn default() -> Self {
        Self::new()
    }
}

pub fn rtk_float_ambiguity_state_from_baseline_solution(
    solution: &RtkFloatBaselineSolution,
) -> Option<RtkFloatAmbiguityState> {
    if solution.ambiguity_covariance_cycles2.len() != solution.float_ambiguities.len()
        || solution
            .ambiguity_covariance_cycles2
            .iter()
            .any(|row| row.len() != solution.float_ambiguities.len())
    {
        return None;
    }
    Some(RtkFloatAmbiguityState {
        ids: solution
            .float_ambiguities
            .iter()
            .map(|ambiguity| RtkDoubleDifferenceAmbiguityId {
                sig: ambiguity.sig,
                ref_sig: ambiguity.ref_sig,
            })
            .collect(),
        float_cycles: solution
            .float_ambiguities
            .iter()
            .map(|ambiguity| ambiguity.float_cycles)
            .collect(),
        covariance_cycles2: solution.ambiguity_covariance_cycles2.clone(),
    })
}

pub fn rtk_ambiguity_state_from_fixed_solution(
    result: &RtkAmbiguityFixResult,
) -> Option<(Vec<RtkDoubleDifferenceAmbiguityId>, Vec<i64>)> {
    Some((result.selected_ids.clone()?, result.selected_integers.clone()?))
}

pub fn rtk_float_ambiguity_state_from_filter_state(
    ids: Vec<RtkDoubleDifferenceAmbiguityId>,
    indices: Vec<usize>,
    state: &[f64],
    covariance: &Matrix,
) -> RtkFloatAmbiguityState {
    let mut float_cycles = Vec::new();
    for &idx in &indices {
        float_cycles.push(state.get(idx).copied().unwrap_or(0.0));
    }
    let sub = covariance.submatrix(&indices, &indices);
    let mut covariance_cycles2 = Vec::new();
    for row in 0..sub.rows() {
        let mut out_row = Vec::new();
        for col in 0..sub.cols() {
            out_row.push(sub[(row, col)]);
        }
        covariance_cycles2.push(out_row);
    }
    RtkFloatAmbiguityState { ids, float_cycles, covariance_cycles2 }
}

/// Transform a float double-difference ambiguity state to a different reference signal.
pub fn rtk_transform_float_ambiguity_reference(
    float: &RtkFloatAmbiguityState,
    new_ref_sig: SigId,
) -> Option<RtkFloatAmbiguityState> {
    let transform = reference_switch_transform(float, new_ref_sig)?;
    let old_float = column_matrix(&float.float_cycles);
    let new_float = transform.coefficients.mul(&old_float);
    let old_covariance = matrix_from_rows(&float.covariance_cycles2);
    let new_covariance =
        transform.coefficients.mul(&old_covariance).mul(&transform.coefficients.transpose());

    Some(RtkFloatAmbiguityState {
        ids: transform.ids,
        float_cycles: matrix_column_values(&new_float),
        covariance_cycles2: matrix_rows(&new_covariance),
    })
}

pub fn rtk_lambda_decorrelate(float: &RtkFloatAmbiguityState) -> RtkDecorrelatedAmbiguityState {
    let n = float.float_cycles.len();
    let mut z = vec![vec![0_i64; n]; n];
    for (index, row) in z.iter_mut().enumerate() {
        row[index] = 1;
    }
    RtkDecorrelatedAmbiguityState {
        z,
        n_prime: float.float_cycles.clone(),
        q_prime: float.covariance_cycles2.clone(),
    }
}

#[derive(Debug, Clone)]
struct ReferenceSwitchTransform {
    ids: Vec<RtkDoubleDifferenceAmbiguityId>,
    coefficients: Matrix,
}

fn reference_switch_transform(
    float: &RtkFloatAmbiguityState,
    new_ref_sig: SigId,
) -> Option<ReferenceSwitchTransform> {
    if !float.validate_payload().is_empty() {
        return None;
    }
    let old_ref_sig = common_reference_signal(&float.ids)?;
    if new_ref_sig == old_ref_sig {
        return Some(ReferenceSwitchTransform {
            ids: float.ids.clone(),
            coefficients: Matrix::identity(float.ids.len()),
        });
    }
    let old_index_by_sig =
        float.ids.iter().enumerate().map(|(index, id)| (id.sig, index)).collect::<BTreeMap<_, _>>();
    let Some(&new_ref_old_index) = old_index_by_sig.get(&new_ref_sig) else {
        return None;
    };

    let mut output_signals = float.ids.iter().map(|id| id.sig).collect::<Vec<_>>();
    output_signals.push(old_ref_sig);
    output_signals.retain(|sig| *sig != new_ref_sig);
    output_signals.sort();
    output_signals.dedup();

    let mut coefficients = Matrix::new(output_signals.len(), float.ids.len(), 0.0);
    let mut ids = Vec::with_capacity(output_signals.len());
    for (row, sig) in output_signals.iter().copied().enumerate() {
        ids.push(RtkDoubleDifferenceAmbiguityId { sig, ref_sig: new_ref_sig });
        if sig == old_ref_sig {
            coefficients[(row, new_ref_old_index)] = -1.0;
        } else {
            let &old_index = old_index_by_sig.get(&sig)?;
            coefficients[(row, old_index)] = 1.0;
            coefficients[(row, new_ref_old_index)] -= 1.0;
        }
    }

    Some(ReferenceSwitchTransform { ids, coefficients })
}

fn common_reference_signal(ids: &[RtkDoubleDifferenceAmbiguityId]) -> Option<SigId> {
    let first = ids.first()?.ref_sig;
    ids.iter().all(|id| id.ref_sig == first && id.sig != id.ref_sig).then_some(first)
}

pub fn rtk_integer_ambiguity_candidates(
    n_prime: &[f64],
    q_prime: &[Vec<f64>],
    top_k: usize,
) -> Vec<RtkIntegerAmbiguityCandidate> {
    if n_prime.is_empty() {
        return Vec::new();
    }
    let mut best = Vec::new();
    let mut base = Vec::new();
    for &value in n_prime {
        base.push(value.round() as i64);
    }
    let cost = candidate_cost(n_prime, q_prime, &base);
    best.push(RtkIntegerAmbiguityCandidate { integers: base.clone(), cost });
    let mut neighbor = base;
    let mut max_index = 0;
    let mut max_residual = 0.0;
    for (index, &value) in n_prime.iter().enumerate() {
        let residual = (value - value.round()).abs();
        if residual > max_residual {
            max_residual = residual;
            max_index = index;
        }
    }
    if max_residual > 0.0 {
        let direction = if n_prime[max_index] - n_prime[max_index].round() >= 0.0 { 1 } else { -1 };
        neighbor[max_index] += direction;
        let cost = candidate_cost(n_prime, q_prime, &neighbor);
        best.push(RtkIntegerAmbiguityCandidate { integers: neighbor, cost });
    }
    best.sort_by(|left, right| left.cost.total_cmp(&right.cost));
    best.truncate(top_k.max(1));
    best
}

pub fn rtk_candidate_ratio(candidates: &[RtkIntegerAmbiguityCandidate]) -> Option<f64> {
    if candidates.len() < 2 {
        return None;
    }
    let first_cost = candidates[0].cost;
    let second_cost = candidates[1].cost;
    if first_cost <= 1.0e-12 {
        if second_cost <= 1.0e-12 {
            None
        } else {
            Some(second_cost / 1.0e-12)
        }
    } else if !first_cost.is_finite() || !second_cost.is_finite() {
        None
    } else {
        Some(second_cost / first_cost)
    }
}

pub fn rtk_select_partial_ambiguity_fix(
    float: &RtkFloatAmbiguityState,
    max_count: usize,
) -> RtkFloatAmbiguityState {
    if float.float_cycles.is_empty() {
        return float.clone();
    }
    let mut indices: Vec<usize> = (0..float.float_cycles.len()).collect();
    indices.sort_by(|left, right| {
        float.covariance_cycles2[*left][*left].total_cmp(&float.covariance_cycles2[*right][*right])
    });
    let keep = max_count.max(1).min(float.float_cycles.len());
    let mut ids = Vec::new();
    let mut float_cycles = Vec::new();
    let mut covariance_cycles2 = Vec::new();
    for &index in indices.iter().take(keep) {
        ids.push(float.ids[index]);
        float_cycles.push(float.float_cycles[index]);
    }
    for &row_index in indices.iter().take(keep) {
        let mut row = Vec::new();
        for &col_index in indices.iter().take(keep) {
            row.push(float.covariance_cycles2[row_index][col_index]);
        }
        covariance_cycles2.push(row);
    }
    RtkFloatAmbiguityState { ids, float_cycles, covariance_cycles2 }
}

pub fn rtk_ratio_test_acceptance(
    ratio: f64,
    policy: &RtkAmbiguityFixPolicy,
    state: &mut RtkAmbiguityFixState,
) -> bool {
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

pub fn rtk_conditioned_baseline_from_fixed_ambiguities(
    solution: &RtkFloatBaselineSolution,
    fixed_ids: &[RtkDoubleDifferenceAmbiguityId],
    fixed_integers: &[i64],
) -> Option<RtkConditionedBaselineSolution> {
    if fixed_ids.is_empty() || fixed_ids.len() != fixed_integers.len() {
        return None;
    }

    let solution_ids = solution
        .float_ambiguities
        .iter()
        .map(|ambiguity| RtkDoubleDifferenceAmbiguityId {
            sig: ambiguity.sig,
            ref_sig: ambiguity.ref_sig,
        })
        .collect::<Vec<_>>();
    let mut selected_indices = Vec::with_capacity(fixed_ids.len());
    for fixed_id in fixed_ids {
        let index = solution_ids.iter().position(|candidate| candidate == fixed_id)?;
        selected_indices.push(index);
    }

    let mut float_vector = Matrix::new(selected_indices.len(), 1, 0.0);
    for (row, &index) in selected_indices.iter().enumerate() {
        float_vector[(row, 0)] = solution.float_ambiguities[index].float_cycles;
    }
    let mut fixed_vector = Matrix::new(selected_indices.len(), 1, 0.0);
    for (row, &integer) in fixed_integers.iter().enumerate() {
        fixed_vector[(row, 0)] = integer as f64;
    }
    let innovation = float_vector.sub(&fixed_vector);

    let ambiguity_covariance = matrix_from_rows(
        &selected_indices
            .iter()
            .map(|&row_index| {
                selected_indices
                    .iter()
                    .map(|&col_index| solution.ambiguity_covariance_cycles2[row_index][col_index])
                    .collect::<Vec<_>>()
            })
            .collect::<Vec<_>>(),
    );
    let ambiguity_covariance_inverse = ambiguity_covariance.invert()?;
    let baseline_ambiguity_covariance = matrix_from_rows(
        &solution
            .enu_ambiguity_covariance_m_cycles
            .iter()
            .map(|row| selected_indices.iter().map(|&index| row[index]).collect::<Vec<_>>())
            .collect::<Vec<_>>(),
    );
    let gain = baseline_ambiguity_covariance.mul(&ambiguity_covariance_inverse);
    let baseline_delta = gain.mul(&innovation);
    let conditioned_covariance = matrix_from_covariance_3x3(solution.covariance_enu_m2)
        .sub(&gain.mul(&baseline_ambiguity_covariance.transpose()));

    Some(RtkConditionedBaselineSolution {
        enu_m: [
            solution.enu_m[0] - baseline_delta[(0, 0)],
            solution.enu_m[1] - baseline_delta[(1, 0)],
            solution.enu_m[2] - baseline_delta[(2, 0)],
        ],
        covariance_enu_m2: covariance_3x3_from_matrix(&conditioned_covariance),
    })
}

fn candidate_cost(n_prime: &[f64], q_prime: &[Vec<f64>], integers: &[i64]) -> f64 {
    let mut cost = 0.0;
    for index in 0..n_prime.len() {
        let variance =
            q_prime.get(index).and_then(|row| row.get(index)).copied().unwrap_or(1.0).max(1.0e-6);
        let residual = n_prime[index] - integers[index] as f64;
        cost += residual * residual / variance;
    }
    cost
}

fn matrix_from_rows(rows: &[Vec<f64>]) -> Matrix {
    let row_count = rows.len();
    let col_count = rows.first().map_or(0, Vec::len);
    let data = rows.iter().flat_map(|row| row.iter().copied()).collect::<Vec<_>>();
    Matrix::from_parts(row_count, col_count, data)
}

fn column_matrix(values: &[f64]) -> Matrix {
    Matrix::from_parts(values.len(), 1, values.to_vec())
}

fn matrix_column_values(matrix: &Matrix) -> Vec<f64> {
    (0..matrix.rows()).map(|row| matrix[(row, 0)]).collect()
}

fn matrix_rows(matrix: &Matrix) -> Vec<Vec<f64>> {
    (0..matrix.rows())
        .map(|row| (0..matrix.cols()).map(|col| matrix[(row, col)]).collect())
        .collect()
}

fn matrix_from_covariance_3x3(covariance: [[f64; 3]; 3]) -> Matrix {
    Matrix::from_parts(
        3,
        3,
        covariance.iter().flat_map(|row| row.iter().copied()).collect::<Vec<_>>(),
    )
}

fn covariance_3x3_from_matrix(matrix: &Matrix) -> [[f64; 3]; 3] {
    let mut covariance = [[0.0; 3]; 3];
    for row in 0..3 {
        for col in 0..3 {
            covariance[row][col] = matrix[(row, col)];
        }
    }
    covariance
}
