use bijux_gnss_receiver::api::core::{AcqResultV1, DiagnosticEvent, InputError};

pub(super) fn validate_acq_artifact(data: &str) -> Result<Vec<DiagnosticEvent>, InputError> {
    super::validate_wrapped_payloads::<AcqResultV1>(data, "acq")
}
