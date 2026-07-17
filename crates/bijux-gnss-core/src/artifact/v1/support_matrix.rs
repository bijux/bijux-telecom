use crate::artifact::{ArtifactPayloadValidate, ArtifactV1};
use crate::api::{DiagnosticEvent, DiagnosticSeverity, SupportMatrix};

/// Support matrix artifact v1.
pub type SupportMatrixV1 = ArtifactV1<SupportMatrix>;

impl ArtifactPayloadValidate for SupportMatrix {
    fn validate_payload(&self) -> Vec<DiagnosticEvent> {
        if self.rows.is_empty() {
            return vec![DiagnosticEvent::new(
                DiagnosticSeverity::Warning,
                "GNSS_SUPPORT_MATRIX_EMPTY",
                "support matrix is empty",
            )];
        }
        Vec::new()
    }
}
