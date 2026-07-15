use std::collections::HashMap;

use bijux_gnss_core::api::{
    CodeCarrierDivergence, CycleSlipDetector, CycleSlipDetectorEvidence, Meters, ObsEpoch,
    ObsSatellite, SigId,
};
use bijux_gnss_signal::api::signal_cycles_to_meters;

use crate::pipeline::hatch::HatchFilterState;

use super::apply_cycle_slip_surface;
use super::carrier_phase::cycle_slip_decision_evidence;
use super::residual_reports::{observation_snapshot_key, RawObservationSnapshot};
use super::SPEED_OF_LIGHT_MPS;

const GEOMETRY_FREE_IONOSPHERE_DELTA_M: f64 = 0.01;
const GEOMETRY_FREE_CYCLE_SLIP_JUMP_M: f64 = 0.10;
const MELBOURNE_WUBBENA_WIDE_LANE_SLIP_JUMP_CYCLES: f64 = 0.5;

pub(super) fn apply_dual_frequency_cycle_slip_fusion(
    epoch: &mut ObsEpoch,
    previous_epoch: Option<&ObsEpoch>,
    raw_snapshots: &HashMap<String, RawObservationSnapshot>,
    hatch: &mut HashMap<SigId, HatchFilterState>,
) {
    let pair_indices = same_satellite_dual_frequency_pair_indices(epoch);
    for (left_index, right_index) in pair_indices {
        let left = &epoch.sats[left_index];
        let right = &epoch.sats[right_index];
        let previous_pair = previous_epoch.and_then(|previous| {
            let previous_left = previous.sats.iter().find(|sat| sat.signal_id == left.signal_id)?;
            let previous_right =
                previous.sats.iter().find(|sat| sat.signal_id == right.signal_id)?;
            Some((previous_left, previous_right))
        });

        let geometry_free_contributor =
            geometry_free_cycle_slip_contributor(left, right, previous_pair);
        apply_dual_frequency_cycle_slip_contributor(
            epoch,
            left_index,
            right_index,
            geometry_free_contributor,
            raw_snapshots,
            hatch,
        );

        let left = &epoch.sats[left_index];
        let right = &epoch.sats[right_index];
        let previous_pair = previous_epoch.and_then(|previous| {
            let previous_left = previous.sats.iter().find(|sat| sat.signal_id == left.signal_id)?;
            let previous_right =
                previous.sats.iter().find(|sat| sat.signal_id == right.signal_id)?;
            Some((previous_left, previous_right))
        });
        let melbourne_wubbena_contributor =
            melbourne_wubbena_cycle_slip_contributor(left, right, previous_pair);
        apply_dual_frequency_cycle_slip_contributor(
            epoch,
            left_index,
            right_index,
            melbourne_wubbena_contributor,
            raw_snapshots,
            hatch,
        );
    }
}

fn same_satellite_dual_frequency_pair_indices(epoch: &ObsEpoch) -> Vec<(usize, usize)> {
    let mut pairs = Vec::new();
    for left_index in 0..epoch.sats.len() {
        for right_index in (left_index + 1)..epoch.sats.len() {
            let left = &epoch.sats[left_index];
            let right = &epoch.sats[right_index];
            if left.signal_id.sat == right.signal_id.sat
                && left.signal_id.band != right.signal_id.band
                && left.lock_flags.carrier_lock
                && right.lock_flags.carrier_lock
            {
                pairs.push((left_index, right_index));
            }
        }
    }
    pairs
}

fn geometry_free_cycle_slip_contributor(
    left: &ObsSatellite,
    right: &ObsSatellite,
    previous_pair: Option<(&ObsSatellite, &ObsSatellite)>,
) -> CycleSlipDetectorEvidence {
    let Some(current_m) = geometry_free_phase_pair_m(left, right) else {
        return CycleSlipDetectorEvidence::new(
            CycleSlipDetector::GeometryFreePhase,
            false,
            None,
            Some(GEOMETRY_FREE_CYCLE_SLIP_JUMP_M),
            "m",
            "geometry_free_unavailable",
        );
    };
    let Some((previous_left, previous_right)) = previous_pair else {
        return CycleSlipDetectorEvidence::new(
            CycleSlipDetector::GeometryFreePhase,
            false,
            Some(0.0),
            Some(GEOMETRY_FREE_CYCLE_SLIP_JUMP_M),
            "m",
            "geometry_free_insufficient_history",
        );
    };
    let Some(previous_m) = geometry_free_phase_pair_m(previous_left, previous_right) else {
        return CycleSlipDetectorEvidence::new(
            CycleSlipDetector::GeometryFreePhase,
            false,
            None,
            Some(GEOMETRY_FREE_CYCLE_SLIP_JUMP_M),
            "m",
            "geometry_free_unavailable",
        );
    };
    let delta_m = current_m - previous_m;
    let abs_delta_m = delta_m.abs();
    let reason = if abs_delta_m >= GEOMETRY_FREE_CYCLE_SLIP_JUMP_M {
        "geometry_free_phase"
    } else if abs_delta_m >= GEOMETRY_FREE_IONOSPHERE_DELTA_M {
        "geometry_free_ionosphere_drift"
    } else {
        "geometry_free_nominal"
    };
    CycleSlipDetectorEvidence::new(
        CycleSlipDetector::GeometryFreePhase,
        abs_delta_m >= GEOMETRY_FREE_CYCLE_SLIP_JUMP_M,
        Some(abs_delta_m),
        Some(GEOMETRY_FREE_CYCLE_SLIP_JUMP_M),
        "m",
        reason,
    )
}

fn melbourne_wubbena_cycle_slip_contributor(
    left: &ObsSatellite,
    right: &ObsSatellite,
    previous_pair: Option<(&ObsSatellite, &ObsSatellite)>,
) -> CycleSlipDetectorEvidence {
    let Some(current_m) = melbourne_wubbena_pair_m(left, right) else {
        return CycleSlipDetectorEvidence::new(
            CycleSlipDetector::MelbourneWubbena,
            false,
            None,
            Some(MELBOURNE_WUBBENA_WIDE_LANE_SLIP_JUMP_CYCLES),
            "wide_lane_cycles",
            "melbourne_wubbena_unavailable",
        );
    };
    let Some((previous_left, previous_right)) = previous_pair else {
        return CycleSlipDetectorEvidence::new(
            CycleSlipDetector::MelbourneWubbena,
            false,
            Some(0.0),
            Some(MELBOURNE_WUBBENA_WIDE_LANE_SLIP_JUMP_CYCLES),
            "wide_lane_cycles",
            "melbourne_wubbena_insufficient_history",
        );
    };
    let Some(previous_m) = melbourne_wubbena_pair_m(previous_left, previous_right) else {
        return CycleSlipDetectorEvidence::new(
            CycleSlipDetector::MelbourneWubbena,
            false,
            None,
            Some(MELBOURNE_WUBBENA_WIDE_LANE_SLIP_JUMP_CYCLES),
            "wide_lane_cycles",
            "melbourne_wubbena_unavailable",
        );
    };
    let Some(wide_lane_wavelength_m) = wide_lane_wavelength_m(left, right) else {
        return CycleSlipDetectorEvidence::new(
            CycleSlipDetector::MelbourneWubbena,
            false,
            None,
            Some(MELBOURNE_WUBBENA_WIDE_LANE_SLIP_JUMP_CYCLES),
            "wide_lane_cycles",
            "melbourne_wubbena_wavelength_unavailable",
        );
    };
    let delta_cycles = (current_m - previous_m) / wide_lane_wavelength_m;
    let abs_delta_cycles = delta_cycles.abs();
    CycleSlipDetectorEvidence::new(
        CycleSlipDetector::MelbourneWubbena,
        abs_delta_cycles >= MELBOURNE_WUBBENA_WIDE_LANE_SLIP_JUMP_CYCLES,
        Some(abs_delta_cycles),
        Some(MELBOURNE_WUBBENA_WIDE_LANE_SLIP_JUMP_CYCLES),
        "wide_lane_cycles",
        if abs_delta_cycles >= MELBOURNE_WUBBENA_WIDE_LANE_SLIP_JUMP_CYCLES {
            "melbourne_wubbena"
        } else {
            "melbourne_wubbena_nominal"
        },
    )
}

fn apply_dual_frequency_cycle_slip_contributor(
    epoch: &mut ObsEpoch,
    left_index: usize,
    right_index: usize,
    contributor: CycleSlipDetectorEvidence,
    raw_snapshots: &HashMap<String, RawObservationSnapshot>,
    hatch: &mut HashMap<SigId, HatchFilterState>,
) {
    let triggered = contributor.triggered;
    let reason = contributor.reason.clone();
    for index in [left_index, right_index] {
        let sat = &mut epoch.sats[index];
        upsert_cycle_slip_contributor(sat, contributor.clone());
        if triggered {
            apply_cycle_slip_surface(sat, Some(&reason));
            reset_smoothed_observation_after_fused_slip(epoch.epoch_idx, sat, raw_snapshots, hatch);
        }
    }
}

pub(super) fn upsert_cycle_slip_contributor(
    sat: &mut ObsSatellite,
    contributor: CycleSlipDetectorEvidence,
) {
    let evidence = sat
        .metadata
        .cycle_slip_evidence
        .get_or_insert_with(|| cycle_slip_decision_evidence(Vec::new()));
    evidence.upsert_contributor(contributor);
}

fn reset_smoothed_observation_after_fused_slip(
    epoch_idx: u64,
    sat: &mut ObsSatellite,
    raw_snapshots: &HashMap<String, RawObservationSnapshot>,
    hatch: &mut HashMap<SigId, HatchFilterState>,
) {
    if let Some(snapshot) = raw_snapshots.get(&observation_snapshot_key(epoch_idx, sat.signal_id)) {
        sat.pseudorange_m = Meters(snapshot.raw_pseudorange_m);
    }
    let state = hatch.entry(sat.signal_id).or_default();
    state.clear_arc();
    sat.metadata.smoothing_age = 1;
    sat.metadata.smoothing_resets = state.reset_count();
    if let Some(divergence) = sat.metadata.code_carrier_divergence {
        sat.metadata.code_carrier_divergence = Some(CodeCarrierDivergence::from_terms(
            divergence.raw_m,
            divergence.jump_m,
            divergence.expected_ionosphere_m,
            divergence.receiver_clock_m,
            divergence.oscillator_m,
            0.0,
            divergence.multipath_m,
        ));
    }
}

fn geometry_free_phase_pair_m(left: &ObsSatellite, right: &ObsSatellite) -> Option<f64> {
    Some(
        signal_cycles_to_meters(left.carrier_phase_cycles, left.metadata.signal).0
            - signal_cycles_to_meters(right.carrier_phase_cycles, right.metadata.signal).0,
    )
    .filter(|value| value.is_finite())
}

fn melbourne_wubbena_pair_m(left: &ObsSatellite, right: &ObsSatellite) -> Option<f64> {
    Some(geometry_free_phase_pair_m(left, right)? - (left.pseudorange_m.0 - right.pseudorange_m.0))
        .filter(|value| value.is_finite())
}

fn wide_lane_wavelength_m(left: &ObsSatellite, right: &ObsSatellite) -> Option<f64> {
    let left_hz = left.metadata.signal.carrier_hz.value();
    let right_hz = right.metadata.signal.carrier_hz.value();
    let delta_hz = (left_hz - right_hz).abs();
    (delta_hz.is_finite() && delta_hz > f64::EPSILON).then_some(SPEED_OF_LIGHT_MPS / delta_hz)
}
