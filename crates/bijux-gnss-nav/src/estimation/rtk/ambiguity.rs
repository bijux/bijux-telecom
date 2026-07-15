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
    /// Integer basis mapping decorrelated coordinates back to original ambiguity coordinates.
    pub z: Vec<Vec<i64>>,
    /// Integer transform mapping original ambiguity coordinates into decorrelated coordinates.
    pub z_inverse: Vec<Vec<i64>>,
    pub n_prime: Vec<f64>,
    pub q_prime: Vec<Vec<f64>>,
}

/// One integer ambiguity candidate and its quadratic cost.
#[derive(Debug, Clone, PartialEq)]
pub struct RtkIntegerAmbiguityCandidate {
    pub integers: Vec<i64>,
    pub cost: f64,
}

/// Criterion used to choose statistically supportable partial ambiguity subsets.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum RtkPartialAmbiguitySelectionCriterion {
    LowestVariance,
}

/// Evidence describing which ambiguities were included in a partial integer fix.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RtkPartialAmbiguitySelection {
    pub criterion: RtkPartialAmbiguitySelectionCriterion,
    pub requested_count: usize,
    pub selected_count: usize,
    pub excluded_count: usize,
    pub selected_indices: Vec<usize>,
    pub excluded_indices: Vec<usize>,
    pub selected_ids: Vec<RtkDoubleDifferenceAmbiguityId>,
    pub excluded_ids: Vec<RtkDoubleDifferenceAmbiguityId>,
    pub selected_variance_cycles2: Vec<f64>,
    pub excluded_variance_cycles2: Vec<f64>,
    pub max_selected_variance_cycles2: Option<f64>,
    pub min_excluded_variance_cycles2: Option<f64>,
}

impl ArtifactPayloadValidate for RtkPartialAmbiguitySelection {
    fn validate_payload(&self) -> Vec<DiagnosticEvent> {
        let mut events = Vec::new();
        if self.selected_count != self.selected_ids.len()
            || self.selected_count != self.selected_indices.len()
            || self.selected_count != self.selected_variance_cycles2.len()
            || self.excluded_count != self.excluded_ids.len()
            || self.excluded_count != self.excluded_indices.len()
            || self.excluded_count != self.excluded_variance_cycles2.len()
        {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Error,
                "RTK_PARTIAL_FIX_SELECTION_SHAPE_INVALID",
                "partial ambiguity selection dimensions do not match",
            ));
        }
        if self.selected_count == 0 {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Error,
                "RTK_PARTIAL_FIX_SELECTION_EMPTY",
                "partial ambiguity selection must include at least one ambiguity",
            ));
        }
        if !self
            .selected_variance_cycles2
            .iter()
            .chain(self.excluded_variance_cycles2.iter())
            .all(|value| value.is_finite() && *value >= 0.0)
        {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Error,
                "RTK_PARTIAL_FIX_SELECTION_VARIANCE_INVALID",
                "partial ambiguity selection variance contains invalid values",
            ));
        }
        events
    }
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
    pub partial_selection: Option<RtkPartialAmbiguitySelection>,
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
    pub partial_selection: Option<RtkPartialAmbiguitySelection>,
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
        if let Some(selection) = &self.partial_selection {
            events.extend(selection.validate_payload());
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
                partial_selection: None,
            };
            let audit = RtkAmbiguityFixAudit {
                epoch_idx,
                ratio: None,
                status: result.status,
                reason: "no_ambiguities".to_string(),
                fixed_count: 0,
                partial_selection: None,
            };
            return (result, audit);
        }

        let mut candidates = rtk_lambda_integer_ambiguity_candidates(float, 2);
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
        let mut partial_selection = None;

        if status != RtkAmbiguityFixStatus::Fixed && float.float_cycles.len() > 1 {
            if let Some((partial, selection)) =
                rtk_select_partial_ambiguity_fix_with_evidence(float, float.float_cycles.len() / 2)
            {
                candidates = rtk_lambda_integer_ambiguity_candidates(&partial, 2);
                ratio = rtk_candidate_ratio(&candidates);
                partial_selection = Some(selection);
                if let Some(candidate_ratio) = ratio {
                    if rtk_ratio_test_acceptance(candidate_ratio, &self.policy, state) {
                        status = RtkAmbiguityFixStatus::Fixed;
                        fixed_count = partial.float_cycles.len();
                        reason = "partial_fix";
                    }
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
                partial_selection.as_ref().map(|selection| selection.selected_ids.clone())
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
            partial_selection: partial_selection.clone(),
        };
        let audit = RtkAmbiguityFixAudit {
            epoch_idx,
            ratio: result.ratio,
            status: result.status,
            reason: reason.to_string(),
            fixed_count,
            partial_selection,
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
    if result.status != RtkAmbiguityFixStatus::Fixed {
        return None;
    }
    let selected_ids = result.selected_ids.clone()?;
    let selected_integers = result.selected_integers.clone()?;
    if selected_ids.is_empty()
        || selected_ids.len() != selected_integers.len()
        || selected_ids.len() != result.fixed_count
    {
        return None;
    }
    if let Some(selection) = &result.partial_selection {
        if selection.selected_ids != selected_ids || selection.selected_count != selected_ids.len()
        {
            return None;
        }
    }
    Some((selected_ids, selected_integers))
}

/// Transform fixed double-difference integer ambiguities to a different reference signal.
pub fn rtk_transform_fixed_ambiguity_reference(
    fixed_ids: &[RtkDoubleDifferenceAmbiguityId],
    fixed_integers: &[i64],
    new_ref_sig: SigId,
) -> Option<(Vec<RtkDoubleDifferenceAmbiguityId>, Vec<i64>)> {
    if fixed_ids.len() != fixed_integers.len() || fixed_ids.is_empty() {
        return None;
    }
    let transform = reference_switch_transform_from_ids(fixed_ids, new_ref_sig)?;
    let fixed_vector =
        column_matrix(&fixed_integers.iter().map(|value| *value as f64).collect::<Vec<_>>());
    let transformed = transform.coefficients.mul(&fixed_vector);
    let mut integers = Vec::with_capacity(transform.ids.len());
    for value in matrix_column_values(&transformed) {
        let integer = value.round();
        if (value - integer).abs() > 1.0e-9 {
            return None;
        }
        integers.push(integer as i64);
    }
    Some((transform.ids, integers))
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

/// Transform a float baseline solution to a different double-difference reference signal.
pub fn rtk_transform_float_baseline_reference(
    solution: &RtkFloatBaselineSolution,
    new_ref_sig: SigId,
) -> Option<RtkFloatBaselineSolution> {
    let float = rtk_float_ambiguity_state_from_baseline_solution(solution)?;
    let transform = reference_switch_transform(&float, new_ref_sig)?;
    let transformed_float = rtk_transform_float_ambiguity_reference(&float, new_ref_sig)?;
    let old_cross_covariance = matrix_from_rows(&solution.enu_ambiguity_covariance_m_cycles);
    let transformed_cross_covariance =
        old_cross_covariance.mul(&transform.coefficients.transpose());

    Some(RtkFloatBaselineSolution {
        enu_m: solution.enu_m,
        covariance_enu_m2: solution.covariance_enu_m2,
        enu_ambiguity_covariance_m_cycles: matrix_rows(&transformed_cross_covariance),
        float_ambiguities: transformed_float
            .ids
            .iter()
            .zip(transformed_float.float_cycles.iter().copied())
            .enumerate()
            .map(|(index, (id, float_cycles))| super::baseline::RtkFloatAmbiguityEstimate {
                sig: id.sig,
                ref_sig: id.ref_sig,
                float_cycles,
                variance_cycles2: transformed_float.covariance_cycles2[index][index].max(0.0),
            })
            .collect(),
        ambiguity_covariance_cycles2: transformed_float.covariance_cycles2,
    })
}

pub fn rtk_lambda_decorrelate(float: &RtkFloatAmbiguityState) -> RtkDecorrelatedAmbiguityState {
    if !float.validate_payload().is_empty()
        || !valid_integer_search_input(&float.float_cycles, &float.covariance_cycles2)
    {
        return identity_decorrelated_state(float);
    }
    let mut reduced = LambdaReduction::new(&float.float_cycles, &float.covariance_cycles2);
    reduced.reduce();
    reduced.into_state()
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
    reference_switch_transform_from_ids(&float.ids, new_ref_sig)
}

fn reference_switch_transform_from_ids(
    ids: &[RtkDoubleDifferenceAmbiguityId],
    new_ref_sig: SigId,
) -> Option<ReferenceSwitchTransform> {
    let old_ref_sig = common_reference_signal(ids)?;
    if new_ref_sig == old_ref_sig {
        return Some(ReferenceSwitchTransform {
            ids: ids.to_vec(),
            coefficients: Matrix::identity(ids.len()),
        });
    }
    let old_index_by_sig =
        ids.iter().enumerate().map(|(index, id)| (id.sig, index)).collect::<BTreeMap<_, _>>();
    let Some(&new_ref_old_index) = old_index_by_sig.get(&new_ref_sig) else {
        return None;
    };

    let mut output_signals = ids.iter().map(|id| id.sig).collect::<Vec<_>>();
    output_signals.push(old_ref_sig);
    output_signals.retain(|sig| *sig != new_ref_sig);
    output_signals.sort();
    output_signals.dedup();

    let mut coefficients = Matrix::new(output_signals.len(), ids.len(), 0.0);
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

#[derive(Debug, Clone)]
struct LambdaReduction {
    z: Vec<Vec<i64>>,
    z_inverse: Vec<Vec<i64>>,
    float: Vec<f64>,
    covariance: Vec<Vec<f64>>,
}

impl LambdaReduction {
    fn new(float: &[f64], covariance: &[Vec<f64>]) -> Self {
        let size = float.len();
        Self {
            z: integer_identity(size),
            z_inverse: integer_identity(size),
            float: float.to_vec(),
            covariance: covariance.to_vec(),
        }
    }

    fn reduce(&mut self) {
        let size = self.float.len();
        if size < 2 {
            return;
        }
        for _ in 0..(size * size * 8) {
            let mut changed = false;
            for target in 0..size {
                for source in 0..size {
                    if target == source {
                        continue;
                    }
                    let variance = self.covariance[source][source];
                    if !variance.is_finite() || variance.abs() < 1.0e-12 {
                        continue;
                    }
                    let coefficient = (self.covariance[target][source] / variance).round();
                    if coefficient.abs() >= 1.0 {
                        changed |= self.apply_integer_gauss(target, source, -(coefficient as i64));
                    }
                }
            }
            for left in 0..(size - 1) {
                if self.covariance[left + 1][left + 1] + 1.0e-12 < self.covariance[left][left] {
                    self.apply_permutation(left, left + 1);
                    changed = true;
                }
            }
            if !changed {
                break;
            }
        }
    }

    fn apply_integer_gauss(&mut self, target: usize, source: usize, coefficient: i64) -> bool {
        if coefficient == 0 {
            return false;
        }
        let size = self.float.len();
        let mut transform = integer_identity(size);
        transform[target][source] += coefficient;
        let mut inverse = integer_identity(size);
        inverse[target][source] -= coefficient;
        self.apply_coordinate_transform(&transform, &inverse);
        true
    }

    fn apply_permutation(&mut self, left: usize, right: usize) {
        let size = self.float.len();
        let mut transform = integer_identity(size);
        transform.swap(left, right);
        self.apply_coordinate_transform(&transform, &transform);
    }

    fn apply_coordinate_transform(&mut self, transform: &[Vec<i64>], inverse: &[Vec<i64>]) {
        self.float = integer_matrix_vector_mul(transform, &self.float);
        self.covariance = covariance_transform(transform, &self.covariance);
        self.z_inverse = integer_matrix_mul(transform, &self.z_inverse);
        self.z = integer_matrix_mul(&self.z, inverse);
    }

    fn into_state(self) -> RtkDecorrelatedAmbiguityState {
        RtkDecorrelatedAmbiguityState {
            z: self.z,
            z_inverse: self.z_inverse,
            n_prime: self.float,
            q_prime: symmetrize_rows(self.covariance),
        }
    }
}

fn identity_decorrelated_state(float: &RtkFloatAmbiguityState) -> RtkDecorrelatedAmbiguityState {
    let size = float.float_cycles.len();
    RtkDecorrelatedAmbiguityState {
        z: integer_identity(size),
        z_inverse: integer_identity(size),
        n_prime: float.float_cycles.clone(),
        q_prime: float.covariance_cycles2.clone(),
    }
}

pub fn rtk_integer_ambiguity_candidates(
    n_prime: &[f64],
    q_prime: &[Vec<f64>],
    top_k: usize,
) -> Vec<RtkIntegerAmbiguityCandidate> {
    if !valid_integer_search_input(n_prime, q_prime) {
        return Vec::new();
    }
    let requested = top_k.max(1);
    let covariance = matrix_from_rows(q_prime);
    let Some(information) = covariance.invert() else {
        return Vec::new();
    };
    let Some(cholesky) = information.cholesky() else {
        return Vec::new();
    };
    let upper = cholesky.transpose();
    let rounded = n_prime.iter().map(|value| round_to_i64(*value)).collect::<Option<Vec<_>>>();
    let Some(rounded) = rounded else {
        return Vec::new();
    };
    let rounded_cost = candidate_cost_with_information(n_prime, &information, &rounded);
    if !rounded_cost.is_finite() {
        return Vec::new();
    }

    let mut radius = rounded_cost.max(1.0e-9) + 1.0;
    let mut best = Vec::new();
    for _ in 0..40 {
        let mut candidates = Vec::new();
        let mut integers = vec![0_i64; n_prime.len()];
        enumerate_integer_candidates(
            n_prime.len(),
            &upper,
            n_prime,
            radius,
            0.0,
            &mut integers,
            &mut candidates,
        );
        candidates.sort_by(|left, right| {
            left.cost.total_cmp(&right.cost).then_with(|| left.integers.cmp(&right.integers))
        });
        candidates.dedup_by(|left, right| left.integers == right.integers);
        if candidates.len() >= requested {
            candidates.truncate(requested);
            return candidates;
        }
        best = candidates;
        radius = radius * 2.0 + 1.0;
    }
    best.truncate(requested);
    best
}

pub fn rtk_lambda_integer_ambiguity_candidates(
    float: &RtkFloatAmbiguityState,
    top_k: usize,
) -> Vec<RtkIntegerAmbiguityCandidate> {
    if !float.validate_payload().is_empty()
        || !valid_integer_search_input(&float.float_cycles, &float.covariance_cycles2)
    {
        return Vec::new();
    }
    let decorrelated = rtk_lambda_decorrelate(float);
    let requested = top_k.max(1);
    let reduced_candidates =
        rtk_integer_ambiguity_candidates(&decorrelated.n_prime, &decorrelated.q_prime, requested);
    let mut candidates = Vec::with_capacity(reduced_candidates.len());
    for candidate in reduced_candidates {
        let Some(integers) = integer_matrix_i64_vector_mul(&decorrelated.z, &candidate.integers)
        else {
            return Vec::new();
        };
        let cost = candidate_cost(&float.float_cycles, &float.covariance_cycles2, &integers);
        if cost.is_finite() {
            candidates.push(RtkIntegerAmbiguityCandidate { integers, cost });
        }
    }
    candidates.sort_by(|left, right| {
        left.cost.total_cmp(&right.cost).then_with(|| left.integers.cmp(&right.integers))
    });
    candidates.dedup_by(|left, right| left.integers == right.integers);
    candidates.truncate(requested);
    candidates
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
    rtk_select_partial_ambiguity_fix_with_evidence(float, max_count)
        .map(|(partial, _selection)| partial)
        .unwrap_or_else(|| float.clone())
}

pub fn rtk_select_partial_ambiguity_fix_with_evidence(
    float: &RtkFloatAmbiguityState,
    max_count: usize,
) -> Option<(RtkFloatAmbiguityState, RtkPartialAmbiguitySelection)> {
    if !float.validate_payload().is_empty() || float.float_cycles.is_empty() {
        return None;
    }
    let mut indices: Vec<usize> = (0..float.float_cycles.len()).collect();
    indices.sort_by(|left, right| {
        float.covariance_cycles2[*left][*left]
            .total_cmp(&float.covariance_cycles2[*right][*right])
            .then_with(|| float.ids[*left].cmp(&float.ids[*right]))
    });
    let keep = max_count.max(1).min(float.float_cycles.len());
    let selected_indices = indices.iter().take(keep).copied().collect::<Vec<_>>();
    let excluded_indices = indices.iter().skip(keep).copied().collect::<Vec<_>>();
    let mut ids = Vec::new();
    let mut float_cycles = Vec::new();
    let mut covariance_cycles2 = Vec::new();
    for &index in &selected_indices {
        ids.push(float.ids[index]);
        float_cycles.push(float.float_cycles[index]);
    }
    for &row_index in &selected_indices {
        let mut row = Vec::new();
        for &col_index in &selected_indices {
            row.push(float.covariance_cycles2[row_index][col_index]);
        }
        covariance_cycles2.push(row);
    }
    let selected_variance_cycles2 = selected_indices
        .iter()
        .map(|index| float.covariance_cycles2[*index][*index])
        .collect::<Vec<_>>();
    let excluded_variance_cycles2 = excluded_indices
        .iter()
        .map(|index| float.covariance_cycles2[*index][*index])
        .collect::<Vec<_>>();
    let selection = RtkPartialAmbiguitySelection {
        criterion: RtkPartialAmbiguitySelectionCriterion::LowestVariance,
        requested_count: max_count,
        selected_count: selected_indices.len(),
        excluded_count: excluded_indices.len(),
        selected_indices: selected_indices.clone(),
        excluded_indices: excluded_indices.clone(),
        selected_ids: selected_indices.iter().map(|index| float.ids[*index]).collect(),
        excluded_ids: excluded_indices.iter().map(|index| float.ids[*index]).collect(),
        max_selected_variance_cycles2: selected_variance_cycles2.iter().copied().reduce(f64::max),
        min_excluded_variance_cycles2: excluded_variance_cycles2.iter().copied().reduce(f64::min),
        selected_variance_cycles2,
        excluded_variance_cycles2,
    };
    if !selection.validate_payload().is_empty() {
        return None;
    }
    Some((RtkFloatAmbiguityState { ids, float_cycles, covariance_cycles2 }, selection))
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
    let covariance = matrix_from_rows(q_prime);
    let Some(information) = covariance.invert() else {
        return f64::INFINITY;
    };
    candidate_cost_with_information(n_prime, &information, integers)
}

fn candidate_cost_with_information(n_prime: &[f64], information: &Matrix, integers: &[i64]) -> f64 {
    if n_prime.len() != integers.len()
        || information.rows() != n_prime.len()
        || information.cols() != n_prime.len()
    {
        return f64::INFINITY;
    }
    let mut cost = 0.0;
    for row in 0..n_prime.len() {
        let row_residual = integers[row] as f64 - n_prime[row];
        for col in 0..n_prime.len() {
            let col_residual = integers[col] as f64 - n_prime[col];
            cost += row_residual * information[(row, col)] * col_residual;
        }
    }
    cost
}

fn valid_integer_search_input(n_prime: &[f64], q_prime: &[Vec<f64>]) -> bool {
    !n_prime.is_empty()
        && q_prime.len() == n_prime.len()
        && q_prime.iter().all(|row| row.len() == n_prime.len())
        && n_prime.iter().all(|value| value.is_finite())
        && q_prime.iter().flat_map(|row| row.iter()).all(|value| value.is_finite())
}

fn enumerate_integer_candidates(
    level: usize,
    upper_information_factor: &Matrix,
    float: &[f64],
    radius: f64,
    partial_cost: f64,
    integers: &mut [i64],
    candidates: &mut Vec<RtkIntegerAmbiguityCandidate>,
) {
    if level == 0 {
        candidates
            .push(RtkIntegerAmbiguityCandidate { integers: integers.to_vec(), cost: partial_cost });
        return;
    }
    let index = level - 1;
    let mut future_term = 0.0;
    for col in (index + 1)..float.len() {
        future_term += upper_information_factor[(index, col)] * (integers[col] as f64 - float[col]);
    }
    let diagonal = upper_information_factor[(index, index)];
    if !diagonal.is_finite() || diagonal.abs() < 1.0e-12 {
        return;
    }
    let remaining = radius - partial_cost;
    if remaining < -1.0e-12 {
        return;
    }
    let center = float[index] - future_term / diagonal;
    let bound = remaining.max(0.0).sqrt() / diagonal.abs();
    let lower = (center - bound).ceil();
    let upper = (center + bound).floor();
    if !lower.is_finite() || !upper.is_finite() || lower > upper {
        return;
    }
    let mut values = Vec::new();
    let mut value = lower as i64;
    while value <= upper as i64 {
        values.push(value);
        if value == i64::MAX {
            break;
        }
        value += 1;
    }
    values.sort_by(|left, right| {
        ((*left as f64 - center).abs())
            .total_cmp(&(*right as f64 - center).abs())
            .then_with(|| left.cmp(right))
    });
    for value in values {
        let term = diagonal * (value as f64 - float[index]) + future_term;
        let next_cost = partial_cost + term * term;
        if next_cost <= radius + 1.0e-9 {
            integers[index] = value;
            enumerate_integer_candidates(
                index,
                upper_information_factor,
                float,
                radius,
                next_cost,
                integers,
                candidates,
            );
        }
    }
}

fn round_to_i64(value: f64) -> Option<i64> {
    if !value.is_finite() || value < i64::MIN as f64 || value > i64::MAX as f64 {
        return None;
    }
    Some(value.round() as i64)
}

fn integer_identity(size: usize) -> Vec<Vec<i64>> {
    (0..size).map(|row| (0..size).map(|col| if row == col { 1 } else { 0 }).collect()).collect()
}

fn integer_matrix_mul(left: &[Vec<i64>], right: &[Vec<i64>]) -> Vec<Vec<i64>> {
    if left.is_empty() || right.is_empty() {
        return Vec::new();
    }
    let rows = left.len();
    let inner = right.len();
    let cols = right[0].len();
    let mut out = vec![vec![0_i64; cols]; rows];
    for row in 0..rows {
        for col in 0..cols {
            let mut sum = 0_i64;
            for index in 0..inner {
                sum += left[row][index] * right[index][col];
            }
            out[row][col] = sum;
        }
    }
    out
}

fn integer_matrix_vector_mul(matrix: &[Vec<i64>], vector: &[f64]) -> Vec<f64> {
    matrix
        .iter()
        .map(|row| {
            row.iter()
                .zip(vector.iter())
                .map(|(coefficient, value)| *coefficient as f64 * value)
                .sum()
        })
        .collect()
}

fn integer_matrix_i64_vector_mul(matrix: &[Vec<i64>], vector: &[i64]) -> Option<Vec<i64>> {
    let mut out = Vec::with_capacity(matrix.len());
    for row in matrix {
        if row.len() != vector.len() {
            return None;
        }
        let mut sum = 0_i64;
        for (coefficient, value) in row.iter().zip(vector.iter()) {
            sum = sum.checked_add(coefficient.checked_mul(*value)?)?;
        }
        out.push(sum);
    }
    Some(out)
}

fn covariance_transform(transform: &[Vec<i64>], covariance: &[Vec<f64>]) -> Vec<Vec<f64>> {
    let transform_matrix = matrix_from_integer_rows(transform);
    let covariance_matrix = matrix_from_rows(covariance);
    matrix_rows(&transform_matrix.mul(&covariance_matrix).mul(&transform_matrix.transpose()))
}

fn matrix_from_integer_rows(rows: &[Vec<i64>]) -> Matrix {
    let row_count = rows.len();
    let col_count = rows.first().map_or(0, Vec::len);
    let data =
        rows.iter().flat_map(|row| row.iter().map(|value| *value as f64)).collect::<Vec<_>>();
    Matrix::from_parts(row_count, col_count, data)
}

fn symmetrize_rows(rows: Vec<Vec<f64>>) -> Vec<Vec<f64>> {
    let mut out = rows;
    for row in 0..out.len() {
        for col in 0..row {
            let value = 0.5 * (out[row][col] + out[col][row]);
            out[row][col] = value;
            out[col][row] = value;
        }
    }
    out
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
