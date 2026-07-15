use crate::api::{KlobucharCoefficients, PositionBroadcastNavigation};
use bijux_gnss_core::api::{Constellation, NavSolutionEpoch, ObsEpoch};

pub(super) fn apply_atmosphere_explainability(
    mut solution: NavSolutionEpoch,
    obs: &ObsEpoch,
    navigation: &[PositionBroadcastNavigation],
    klobuchar: Option<&KlobucharCoefficients>,
    tropo_enabled: bool,
) -> NavSolutionEpoch {
    apply_atmosphere_explainability_in_place(
        &mut solution,
        obs,
        navigation,
        klobuchar,
        tropo_enabled,
    );
    solution
}

pub(super) fn apply_atmosphere_explainability_in_place(
    solution: &mut NavSolutionEpoch,
    obs: &ObsEpoch,
    navigation: &[PositionBroadcastNavigation],
    klobuchar: Option<&KlobucharCoefficients>,
    tropo_enabled: bool,
) {
    super::apply_refusal_cause_explainability_in_place(solution, Some(obs));
    for ionosphere_reason in ionosphere_explain_reasons(obs, navigation, klobuchar) {
        if !solution.explain_reasons.iter().any(|existing| existing == ionosphere_reason) {
            solution.explain_reasons.push(ionosphere_reason.to_string());
        }
    }
    let troposphere_reason = if tropo_enabled {
        "troposphere_correction=saastamoinen"
    } else {
        "troposphere_uncorrected"
    };
    if !solution.explain_reasons.iter().any(|existing| existing == troposphere_reason) {
        solution.explain_reasons.push(troposphere_reason.to_string());
    }
}

fn ionosphere_explain_reasons(
    obs: &ObsEpoch,
    navigation: &[PositionBroadcastNavigation],
    klobuchar: Option<&KlobucharCoefficients>,
) -> Vec<&'static str> {
    let has_gps_observations = obs
        .sats
        .iter()
        .any(|satellite| satellite.signal_id.sat.constellation == Constellation::Gps);
    let has_galileo_observations = obs
        .sats
        .iter()
        .any(|satellite| satellite.signal_id.sat.constellation == Constellation::Galileo);
    let has_galileo_navigation =
        navigation.iter().any(|entry| matches!(entry, PositionBroadcastNavigation::Galileo(_)));

    let mut reasons = Vec::new();
    if klobuchar.is_some() && has_gps_observations {
        reasons.push("ionosphere_correction=klobuchar_broadcast");
    }
    if has_galileo_observations && has_galileo_navigation {
        reasons.push("ionosphere_correction=galileo_nequick");
    }
    if reasons.is_empty() {
        reasons.push("ionosphere_uncorrected");
    }
    reasons
}
