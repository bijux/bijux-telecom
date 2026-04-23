//! Parsing helpers.

use bijux_gnss_receiver::api::core::InputError;

/// Parse ECEF coordinates in x,y,z format.
pub(crate) fn parse_ecef(text: &str) -> Result<[f64; 3], InputError> {
    let parts: Vec<&str> = text.split(',').collect();
    if parts.len() != 3 {
        return Err(InputError { message: "invalid ECEF format, expected x,y,z".to_string() });
    }
    Ok([
        parts[0].trim().parse().map_err(map)?,
        parts[1].trim().parse().map_err(map)?,
        parts[2].trim().parse().map_err(map)?,
    ])
}

fn map(err: impl std::fmt::Display) -> InputError {
    InputError { message: err.to_string() }
}
