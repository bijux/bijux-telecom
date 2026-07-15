use std::collections::HashMap;

use bijux_gnss_core::api::{CodeCarrierDivergence, Meters, ObsSatellite, SigId};
use bijux_gnss_signal::api::{signal_cycles_to_meters, signal_wavelength_m};

use crate::pipeline::hatch::HatchFilterState;

use super::code_carrier_divergence::{
    classify_code_carrier_divergence, code_carrier_divergence_evidence,
    receiver_clock_divergence_drift_m, slip_threshold_m,
};
use super::cycle_slip_fusion::upsert_cycle_slip_contributor;
use super::labels::pseudorange_model_has_resolved_alignment;
use super::lock_state::apply_cycle_slip_surface;
use super::residual_reports::{raw_observation_snapshot, RawObservationSnapshot};
use super::status::apply_pseudorange_physics_rejection;
use super::variance::observation_error_model;

pub(super) fn apply_hatch_smoothing(
    sat: &mut ObsSatellite,
    snapshot_key: String,
    hatch_window: u32,
    raw_snapshots: &mut HashMap<String, RawObservationSnapshot>,
    hatch: &mut HashMap<SigId, HatchFilterState>,
) {
    let lambda_m = signal_wavelength_m(sat.metadata.signal).0;
    let state = hatch.entry(sat.signal_id).or_default();
    if !sat.lock_flags.code_lock {
        raw_snapshots.insert(snapshot_key, raw_observation_snapshot(sat, sat.pseudorange_m.0));
        state.clear_arc();
        sat.metadata.smoothing_window = hatch_window;
        sat.metadata.smoothing_age = 0;
        sat.metadata.smoothing_resets = state.reset_count();
        sat.error_model = observation_error_model(sat, 0.0);
        apply_pseudorange_physics_rejection(sat);
        return;
    }
    let supports_code_carrier_slip_detection =
        pseudorange_model_has_resolved_alignment(&sat.metadata.pseudorange_model);
    let raw_pseudorange_m = sat.pseudorange_m.0;
    raw_snapshots.insert(snapshot_key, raw_observation_snapshot(sat, raw_pseudorange_m));
    let Some(carrier_phase_arc_id) = sat
        .metadata
        .carrier_phase_arc
        .as_ref()
        .filter(|arc| arc.valid_for_smoothing)
        .map(|arc| arc.id.clone())
    else {
        state.clear_arc();
        sat.metadata.smoothing_window = hatch_window;
        sat.metadata.smoothing_age = 0;
        sat.metadata.smoothing_resets = state.reset_count();
        sat.error_model = observation_error_model(sat, 0.0);
        apply_pseudorange_physics_rejection(sat);
        return;
    };
    state.align_carrier_phase_arc(&carrier_phase_arc_id);
    let raw_divergence_m = raw_pseudorange_m
        - signal_cycles_to_meters(sat.carrier_phase_cycles, sat.metadata.signal).0;
    let threshold_m = slip_threshold_m(sat.cn0_dbhz, sat.elevation_deg);
    let divergence_delta_m = state.divergence_delta_m(raw_divergence_m).unwrap_or(0.0);
    let divergence_jump = divergence_delta_m.abs();
    let mut smoothing_cycle_slip_reason = None;
    if supports_code_carrier_slip_detection && divergence_jump > threshold_m {
        smoothing_cycle_slip_reason = Some("code_carrier_divergence");
    }
    if supports_code_carrier_slip_detection {
        upsert_cycle_slip_contributor(
            sat,
            code_carrier_divergence_evidence(
                smoothing_cycle_slip_reason.is_some(),
                divergence_jump,
                threshold_m,
            ),
        );
    }
    apply_cycle_slip_surface(sat, smoothing_cycle_slip_reason);
    if sat.lock_flags.cycle_slip {
        state.clear_arc();
        state.align_carrier_phase_arc(&carrier_phase_arc_id);
    }
    let smoothing = state.observe(
        raw_pseudorange_m,
        sat.carrier_phase_cycles.0,
        raw_divergence_m,
        lambda_m,
        hatch_window,
    );
    sat.pseudorange_m = Meters(smoothing.smoothed_pseudorange_m);
    sat.metadata.smoothing_window = hatch_window;
    sat.metadata.smoothing_age = smoothing.smoothing_age_epochs;
    sat.metadata.smoothing_resets = smoothing.reset_count;
    let smoothing_transient_m = smoothing.smoothed_pseudorange_m - raw_pseudorange_m;
    sat.metadata.code_carrier_divergence = Some(CodeCarrierDivergence::from_terms(
        raw_divergence_m,
        divergence_delta_m,
        0.0,
        0.0,
        receiver_clock_divergence_drift_m(sat),
        smoothing_transient_m,
        0.0,
    ));
    classify_code_carrier_divergence(sat, 0.0);
    apply_pseudorange_physics_rejection(sat);
}
