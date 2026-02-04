//! Reference validation helpers for CLI integrations.

use crate::errors::{InfraError, InfraResult};
use crate::reference_validation::{
    align_reference_by_time, ReferenceAlign, ValidationReferenceEpoch,
};

/// Align reference epochs to solution epochs and validate alignment.
pub fn validate_reference(
    solutions: &[bijux_gnss_core::NavSolutionEpoch],
    reference_epochs: &[ValidationReferenceEpoch],
    align: ReferenceAlign,
) -> InfraResult<Vec<ValidationReferenceEpoch>> {
    let aligned = align_reference_by_time(solutions, reference_epochs, align);
    if aligned.is_empty() {
        return Err(InfraError::InvalidInput(
            "no reference epochs aligned; check t_rx_s in reference file".to_string(),
        ));
    }
    Ok(aligned)
}
