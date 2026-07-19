//! Coordinate parsing helpers.

use bijux_gnss_receiver::api::core::InputError;

/// Parse ECEF coordinates in `x,y,z` format.
pub(crate) fn parse_ecef(text: &str) -> Result<[f64; 3], InputError> {
    let parts = text.split(',').map(str::trim).collect::<Vec<_>>();
    if parts.len() != 3 {
        return Err(InputError { message: "invalid ECEF format, expected x,y,z".to_string() });
    }
    Ok([
        parts[0].parse().map_err(map_err)?,
        parts[1].parse().map_err(map_err)?,
        parts[2].parse().map_err(map_err)?,
    ])
}

fn map_err(err: impl std::fmt::Display) -> InputError {
    InputError { message: err.to_string() }
}

#[cfg(test)]
mod tests {
    use super::parse_ecef;

    #[test]
    fn parse_ecef_accepts_trimmed_coordinates() {
        let coordinates = parse_ecef("1.0, 2.5, -3.25").expect("parse coordinates");
        assert_eq!(coordinates, [1.0, 2.5, -3.25]);
    }

    #[test]
    fn parse_ecef_rejects_incomplete_coordinates() {
        let err = parse_ecef("1.0,2.5").expect_err("incomplete coordinates must fail");
        assert!(err.message.contains("expected x,y,z"));
    }
}
