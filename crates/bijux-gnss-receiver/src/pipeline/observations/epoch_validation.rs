use bijux_gnss_core::api::{DiagnosticEvent, DiagnosticSeverity, ObsEpoch};

pub(super) fn apply_observation_epoch_sanity(
    epoch: &mut ObsEpoch,
    diagnostics: &mut Vec<DiagnosticEvent>,
) {
    let events = bijux_gnss_core::api::check_obs_epoch_sanity(epoch);
    if events.iter().any(|event| matches!(event.severity, DiagnosticSeverity::Error)) {
        epoch.valid = false;
        diagnostics.extend(events);
    }
}

pub(super) fn validate_observation_epoch_sequence(
    _epochs: &[ObsEpoch],
    _diagnostics: &mut Vec<DiagnosticEvent>,
) {
    #[cfg(feature = "reference-checks")]
    {
        if let Err(err) = bijux_gnss_signal::api::validate_obs_epochs(_epochs) {
            _diagnostics.push(
                DiagnosticEvent::new(DiagnosticSeverity::Error, "OBS_EPOCH_SEQUENCE_INVALID", err)
                    .with_context("stage", "observations"),
            );
        }
    }
}
