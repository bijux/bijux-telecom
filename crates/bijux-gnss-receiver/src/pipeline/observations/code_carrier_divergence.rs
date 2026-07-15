use std::collections::{BTreeMap, HashMap};

use bijux_gnss_core::api::{
    CodeCarrierDivergence, CycleSlipDetector, CycleSlipDetectorEvidence, ObsEpoch, ObsSatellite,
    SatId, SigId,
};

use super::variance::observation_error_model;
use super::SPEED_OF_LIGHT_MPS;

#[derive(Debug, Clone)]
pub(super) struct IonosphereDelayEvidence {
    pub(super) signal_id: SigId,
    pub(super) delay_m: f64,
}

#[derive(Default, Debug, Clone)]
pub(super) struct CodeCarrierDivergenceState {
    ionosphere_delay_m_by_signal: HashMap<SigId, f64>,
}

pub(super) fn code_carrier_divergence_evidence(
    triggered: bool,
    value_m: f64,
    threshold_m: f64,
) -> CycleSlipDetectorEvidence {
    CycleSlipDetectorEvidence::new(
        CycleSlipDetector::CodeCarrierDivergence,
        triggered,
        Some(value_m),
        Some(threshold_m),
        "m",
        if triggered { "code_carrier_divergence" } else { "code_carrier_divergence_nominal" },
    )
}

pub(super) fn slip_threshold_m(cn0_dbhz: f64, elevation_deg: Option<f64>) -> f64 {
    let cn0 = cn0_dbhz.clamp(20.0, 60.0);
    let cn0_factor = 45.0 / cn0;
    let elev = elevation_deg.unwrap_or(30.0).clamp(0.0, 90.0);
    let elev_factor = 1.0 + (30.0 - elev).max(0.0) / 30.0;
    5.0 * cn0_factor * elev_factor
}

pub(super) fn receiver_clock_divergence_drift_m(sat: &ObsSatellite) -> f64 {
    let carrier_hz = sat.metadata.signal.carrier_hz.value();
    let integration_s = sat.metadata.integration_ms as f64 / 1_000.0;
    let frequency_bias_hz = sat.metadata.receiver_clock_frequency_bias_hz;
    if !carrier_hz.is_finite()
        || carrier_hz <= 0.0
        || !integration_s.is_finite()
        || integration_s <= 0.0
        || !frequency_bias_hz.is_finite()
    {
        return 0.0;
    }
    let wavelength_m = SPEED_OF_LIGHT_MPS / carrier_hz;
    -frequency_bias_hz * wavelength_m * integration_s
}

pub(super) fn apply_code_carrier_divergence_decomposition(
    epoch: &mut ObsEpoch,
    state: &mut CodeCarrierDivergenceState,
) {
    let mut expected_ionosphere_by_signal = HashMap::<SigId, f64>::new();
    for evidence in ionosphere_delay_evidence_from_epoch(epoch) {
        if let Some(previous_delay_m) =
            state.ionosphere_delay_m_by_signal.insert(evidence.signal_id, evidence.delay_m)
        {
            expected_ionosphere_by_signal
                .insert(evidence.signal_id, 2.0 * (evidence.delay_m - previous_delay_m));
        }
    }

    for sat in &mut epoch.sats {
        let expected_ionosphere_m =
            expected_ionosphere_by_signal.get(&sat.signal_id).copied().unwrap_or(0.0);
        classify_code_carrier_divergence(sat, expected_ionosphere_m);
    }
}

pub(super) fn classify_code_carrier_divergence(sat: &mut ObsSatellite, expected_ionosphere_m: f64) {
    let Some(current) = sat.metadata.code_carrier_divergence else {
        return;
    };
    let without_multipath = CodeCarrierDivergence::from_terms(
        current.raw_m,
        current.jump_m,
        expected_ionosphere_m,
        current.receiver_clock_m,
        current.oscillator_m,
        current.smoothing_transient_m,
        0.0,
    );
    let threshold_m = slip_threshold_m(sat.cn0_dbhz, sat.elevation_deg) * 0.8;
    let multipath_m = if without_multipath.unexplained_abs_m() > threshold_m {
        without_multipath.unexplained_m
    } else {
        0.0
    };
    let decomposed = CodeCarrierDivergence::from_terms(
        current.raw_m,
        current.jump_m,
        expected_ionosphere_m,
        current.receiver_clock_m,
        current.oscillator_m,
        current.smoothing_transient_m,
        multipath_m,
    );
    sat.multipath_suspect = multipath_m.abs() > threshold_m;
    sat.error_model = observation_error_model(sat, multipath_m.abs());
    sat.metadata.code_carrier_divergence = Some(decomposed);
}

pub(super) fn ionosphere_delay_evidence_from_epoch(
    epoch: &ObsEpoch,
) -> Vec<IonosphereDelayEvidence> {
    let mut by_sat = BTreeMap::<SatId, Vec<&ObsSatellite>>::new();
    for sat in &epoch.sats {
        by_sat.entry(sat.signal_id.sat).or_default().push(sat);
    }

    let mut out = Vec::new();
    for sats in by_sat.values() {
        for left_index in 0..sats.len() {
            for right_index in (left_index + 1)..sats.len() {
                let left = sats[left_index];
                let right = sats[right_index];
                if let Some((left_delay_m, right_delay_m)) =
                    dual_frequency_code_delay_evidence(left, right)
                {
                    out.push(IonosphereDelayEvidence {
                        signal_id: left.signal_id,
                        delay_m: left_delay_m,
                    });
                    out.push(IonosphereDelayEvidence {
                        signal_id: right.signal_id,
                        delay_m: right_delay_m,
                    });
                }
            }
        }
    }
    out
}

pub(super) fn dual_frequency_code_delay_evidence(
    first: &ObsSatellite,
    second: &ObsSatellite,
) -> Option<(f64, f64)> {
    if first.signal_id.sat != second.signal_id.sat
        || first.signal_id.band == second.signal_id.band
        || !first.lock_flags.code_lock
        || !second.lock_flags.code_lock
    {
        return None;
    }
    let first_frequency_hz = first.metadata.signal.carrier_hz.value();
    let second_frequency_hz = second.metadata.signal.carrier_hz.value();
    if !first_frequency_hz.is_finite()
        || !second_frequency_hz.is_finite()
        || first_frequency_hz <= 0.0
        || second_frequency_hz <= 0.0
        || (first_frequency_hz - second_frequency_hz).abs() <= f64::EPSILON
        || !first.pseudorange_m.0.is_finite()
        || !second.pseudorange_m.0.is_finite()
    {
        return None;
    }

    let first_inverse_frequency2 = 1.0 / first_frequency_hz.powi(2);
    let second_inverse_frequency2 = 1.0 / second_frequency_hz.powi(2);
    let denominator = second_inverse_frequency2 - first_inverse_frequency2;
    if !denominator.is_finite() || denominator == 0.0 {
        return None;
    }
    let code_geometry_free_m = second.pseudorange_m.0 - first.pseudorange_m.0;
    let ionosphere_constant = code_geometry_free_m / denominator;
    Some((
        ionosphere_constant * first_inverse_frequency2,
        ionosphere_constant * second_inverse_frequency2,
    ))
}
