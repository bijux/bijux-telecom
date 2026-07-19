use bijux_gnss_core::api::{
    obs_epoch_stability_key, NavAssumptions, NavSolutionEpoch, ObsEpoch,
    NAV_OUTPUT_STABILITY_SIGNATURE_VERSION,
};

pub(super) fn nav_output_stability_signature(solution: &NavSolutionEpoch) -> String {
    let refusal = solution
        .refusal_class
        .map(|value| format!("{value:?}"))
        .unwrap_or_else(|| "None".to_string());
    let source_descriptor = format!(
        "{}@{}",
        short_id(&solution.source_observation_epoch_id),
        solution.source_time.sample_index
    );
    format!(
        "navsig:v{}:epoch={}:src={}:status={:?}:lifecycle={:?}:valid={}:sat={}:used={}:rej={}:pdop={:.3}:hdop={}:vdop={}:gdop={}:tdop={}:rms={:.3}:refusal={}:decision={}",
        NAV_OUTPUT_STABILITY_SIGNATURE_VERSION,
        solution.epoch.index,
        source_descriptor,
        solution.status,
        solution.lifecycle_state,
        solution.valid,
        solution.sat_count,
        solution.used_sat_count,
        solution.rejected_sat_count,
        solution.pdop,
        format_optional_nav_dop(solution.hdop),
        format_optional_nav_dop(solution.vdop),
        format_optional_nav_dop(solution.gdop),
        format_optional_nav_dop(solution.tdop),
        solution.rms_m.0,
        refusal,
        solution.explain_decision
    )
}

fn format_optional_nav_dop(value: Option<f64>) -> String {
    value.map(|dop| format!("{dop:.3}")).unwrap_or_else(|| "na".to_string())
}

pub(super) fn source_observation_epoch_id(obs: &ObsEpoch) -> String {
    obs.manifest
        .as_ref()
        .map(|manifest| manifest.epoch_id.clone())
        .unwrap_or_else(|| obs_epoch_stability_key(obs))
}

pub(super) fn nav_artifact_id(epoch_idx: u64, source_observation_epoch_id: &str) -> String {
    format!("nav-epoch-{epoch_idx:010}-{}", short_id(source_observation_epoch_id))
}

fn short_id(value: &str) -> String {
    value.chars().take(16).collect()
}

pub(super) fn nav_assumptions(ephemeris_count: usize) -> NavAssumptions {
    let ephemeris_completeness = if ephemeris_count == 0 {
        "none"
    } else if ephemeris_count < 4 {
        "partial"
    } else {
        "sufficient"
    };
    NavAssumptions {
        time_system: "gps".to_string(),
        reference_frame: "ecef_wgs84".to_string(),
        clock_model: "receiver_clock_bias_drift_linear".to_string(),
        ephemeris_source: "broadcast_lnav".to_string(),
        frame_decode_mode: "lnav".to_string(),
        ephemeris_completeness: ephemeris_completeness.to_string(),
        ephemeris_count,
    }
}
