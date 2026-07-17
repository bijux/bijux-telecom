use bijux_gnss_receiver::api::core::{
    ArtifactValidate, DiagnosticEvent, DiagnosticSeverity, InputError, ObsEpochV1,
};

pub(super) fn validate_obs_artifact(data: &str) -> Result<Vec<DiagnosticEvent>, InputError> {
    let mut epochs = Vec::new();
    let mut events = Vec::new();
    let mut last_t_rx_s: Option<bijux_gnss_receiver::api::core::Seconds> = None;

    for line in data.lines() {
        if line.trim().is_empty() {
            continue;
        }

        let wrapped: ObsEpochV1 = serde_json::from_str(line).map_err(super::map_err)?;
        super::validate_schema_version(wrapped.header.schema_version, "obs")?;
        if let Some(previous) = last_t_rx_s {
            if wrapped.payload.t_rx_s.0 < previous.0 {
                events.push(DiagnosticEvent::new(
                    DiagnosticSeverity::Error,
                    "GNSS_OBS_TIME_NON_MONOTONIC",
                    "obs t_rx_s is not monotonic",
                ));
            }
        }
        last_t_rx_s = Some(wrapped.payload.t_rx_s);
        events.extend(wrapped.validate());
        epochs.push(wrapped.payload);
    }

    if let Err(err) = bijux_gnss_receiver::api::signal::validate_obs_epochs(&epochs) {
        events.push(DiagnosticEvent::new(
            DiagnosticSeverity::Error,
            "GNSS_OBS_VALIDATE_FAILED",
            format!("obs epoch validation failed: {err}"),
        ));
    }

    Ok(events)
}
