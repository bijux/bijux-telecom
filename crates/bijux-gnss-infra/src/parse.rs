//! Parsing helpers.

use crate::errors::{InfraError, InfraResult};

/// Parse ECEF coordinates in x,y,z format.
pub fn parse_ecef(text: &str) -> InfraResult<[f64; 3]> {
    let parts: Vec<&str> = text.split(',').collect();
    if parts.len() != 3 {
        return Err(InfraError::InvalidInput(
            "invalid ECEF format, expected x,y,z".to_string(),
        ));
    }
    Ok([
        parts[0].trim().parse().map_err(map)?,
        parts[1].trim().parse().map_err(map)?,
        parts[2].trim().parse().map_err(map)?,
    ])
}

fn map(err: impl std::fmt::Display) -> InfraError {
    InfraError::InvalidInput(err.to_string())
}
