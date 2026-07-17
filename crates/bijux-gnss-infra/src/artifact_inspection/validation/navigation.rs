use bijux_gnss_receiver::api::core::{
    ArtifactPayloadValidate, DiagnosticEvent, DiagnosticSeverity, InputError, NavSolutionEpochV1,
};

pub(super) fn validate_nav_artifact(data: &str) -> Result<Vec<DiagnosticEvent>, InputError> {
    let mut last_t_rx_s: Option<bijux_gnss_receiver::api::core::Seconds> = None;
    let mut events = Vec::new();

    for line in data.lines() {
        if line.trim().is_empty() {
            continue;
        }

        let wrapped: NavSolutionEpochV1 = serde_json::from_str(line).map_err(super::map_err)?;
        super::validate_schema_version(wrapped.header.schema_version, "pvt")?;
        if let Some(previous) = last_t_rx_s {
            if wrapped.payload.t_rx_s.0 < previous.0 {
                events.push(DiagnosticEvent::new(
                    DiagnosticSeverity::Error,
                    "GNSS_NAV_TIME_NON_MONOTONIC",
                    "nav t_rx_s is not monotonic",
                ));
            }
        }
        last_t_rx_s = Some(wrapped.payload.t_rx_s);
        events.extend(wrapped.payload.validate_payload());
    }

    Ok(events)
}
