//! Reference validation helpers for CLI integrations.

use bijux_gnss_receiver::api::core::{
    align_reference_by_time, InputError, ReferenceAlign, ValidationReferenceEpoch,
};

/// Align reference epochs to solution epochs and validate alignment.
pub fn validate_reference(
    solutions: &[bijux_gnss_receiver::api::core::NavSolutionEpoch],
    reference_epochs: &[ValidationReferenceEpoch],
    align: ReferenceAlign,
) -> Result<Vec<ValidationReferenceEpoch>, InputError> {
    let aligned = align_reference_by_time(solutions, reference_epochs, align);
    if aligned.is_empty() {
        return Err(InputError {
            message: "no reference epochs aligned; check t_rx_s in reference file".to_string(),
        });
    }
    Ok(aligned)
}
