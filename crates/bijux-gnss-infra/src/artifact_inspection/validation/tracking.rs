use bijux_gnss_receiver::api::core::{
    ArtifactPayloadValidate, DiagnosticEvent, DiagnosticSeverity, InputError, TrackEpochV1,
};

pub(super) fn validate_track_artifact(data: &str) -> Result<Vec<DiagnosticEvent>, InputError> {
    let mut last_sample_index: Option<u64> = None;
    let mut events = Vec::new();

    for line in data.lines() {
        if line.trim().is_empty() {
            continue;
        }

        let wrapped: TrackEpochV1 = serde_json::from_str(line).map_err(super::map_err)?;
        super::schema_policy::validate_schema_version(wrapped.header.schema_version, "track")?;
        if let Some(previous) = last_sample_index {
            if wrapped.payload.sample_index < previous {
                events.push(DiagnosticEvent::new(
                    DiagnosticSeverity::Error,
                    "GNSS_TRACK_SAMPLE_NON_MONOTONIC",
                    "track sample index is not monotonic",
                ));
            }
        }
        last_sample_index = Some(wrapped.payload.sample_index);
        events.extend(wrapped.payload.validate_payload());
    }

    Ok(events)
}
