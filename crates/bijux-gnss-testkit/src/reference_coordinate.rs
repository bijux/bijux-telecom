//! Trusted reference-coordinate fixtures for public and surveyed validation cases.

use std::error::Error;
use std::fmt;
use std::fs;
use std::path::Path;

/// Stable CSV header for trusted reference-coordinate catalogs.
pub const TRUSTED_REFERENCE_COORDINATE_HEADER: &str =
    "marker_name,fixture_name,lat_deg,lon_deg,alt_m,source";

/// Trusted survey or published reference coordinate for one validation fixture.
#[derive(Debug, Clone, PartialEq)]
pub struct TrustedReferenceCoordinate {
    /// Stable station or marker name.
    pub marker_name: String,
    /// Fixture filename associated with this coordinate.
    pub fixture_name: String,
    /// Geodetic latitude in degrees.
    pub lat_deg: f64,
    /// Geodetic longitude in degrees.
    pub lon_deg: f64,
    /// Geodetic altitude in meters.
    pub alt_m: f64,
    /// Human-readable provenance for the coordinate.
    pub source: String,
}

/// Error raised while loading or requiring trusted reference coordinates.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum TrustedReferenceCoordinateError {
    ReadFailure(String),
    EmptyCatalog,
    UnexpectedHeader(String),
    InvalidFieldCount { line_number: usize, found: usize },
    InvalidLatitude { line_number: usize, value: String },
    InvalidLongitude { line_number: usize, value: String },
    InvalidAltitude { line_number: usize, value: String },
    MissingFixture(String),
}

impl fmt::Display for TrustedReferenceCoordinateError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::ReadFailure(message) => write!(f, "failed to read trusted reference coordinates: {message}"),
            Self::EmptyCatalog => write!(f, "trusted reference coordinate catalog is empty"),
            Self::UnexpectedHeader(header) => {
                write!(f, "unexpected trusted reference coordinate header '{header}'")
            }
            Self::InvalidFieldCount { line_number, found } => write!(
                f,
                "trusted reference coordinate line {line_number} expected 6 fields, found {found}"
            ),
            Self::InvalidLatitude { line_number, value } => {
                write!(f, "invalid latitude on line {line_number}: {value}")
            }
            Self::InvalidLongitude { line_number, value } => {
                write!(f, "invalid longitude on line {line_number}: {value}")
            }
            Self::InvalidAltitude { line_number, value } => {
                write!(f, "invalid altitude on line {line_number}: {value}")
            }
            Self::MissingFixture(fixture_name) => write!(
                f,
                "missing trusted reference coordinate for fixture {fixture_name}"
            ),
        }
    }
}

impl Error for TrustedReferenceCoordinateError {}

/// Load trusted reference coordinates from a CSV file.
pub fn load_trusted_reference_coordinates(
    path: &Path,
) -> Result<Vec<TrustedReferenceCoordinate>, TrustedReferenceCoordinateError> {
    let contents = fs::read_to_string(path)
        .map_err(|err| TrustedReferenceCoordinateError::ReadFailure(err.to_string()))?;
    parse_trusted_reference_coordinates_csv(&contents)
}

/// Parse trusted reference coordinates from CSV data.
pub fn parse_trusted_reference_coordinates_csv(
    data: &str,
) -> Result<Vec<TrustedReferenceCoordinate>, TrustedReferenceCoordinateError> {
    let mut lines = data.lines();
    let Some(header) = lines.next() else {
        return Err(TrustedReferenceCoordinateError::EmptyCatalog);
    };
    if header.trim() != TRUSTED_REFERENCE_COORDINATE_HEADER {
        return Err(TrustedReferenceCoordinateError::UnexpectedHeader(header.trim().to_string()));
    }

    let coordinates = lines
        .enumerate()
        .filter(|(_, line)| !line.trim().is_empty())
        .map(|(index, line)| parse_trusted_reference_coordinate_line(index + 2, line))
        .collect::<Result<Vec<_>, _>>()?;

    if coordinates.is_empty() {
        return Err(TrustedReferenceCoordinateError::EmptyCatalog);
    }

    Ok(coordinates)
}

/// Require one trusted reference coordinate by fixture name.
pub fn trusted_reference_coordinate_by_fixture(
    catalog: &[TrustedReferenceCoordinate],
    fixture_name: &str,
) -> Result<TrustedReferenceCoordinate, TrustedReferenceCoordinateError> {
    catalog
        .iter()
        .find(|coordinate| coordinate.fixture_name == fixture_name)
        .cloned()
        .ok_or_else(|| TrustedReferenceCoordinateError::MissingFixture(fixture_name.to_string()))
}

fn parse_trusted_reference_coordinate_line(
    line_number: usize,
    line: &str,
) -> Result<TrustedReferenceCoordinate, TrustedReferenceCoordinateError> {
    let fields = line.splitn(6, ',').collect::<Vec<_>>();
    if fields.len() != 6 {
        return Err(TrustedReferenceCoordinateError::InvalidFieldCount {
            line_number,
            found: fields.len(),
        });
    }

    let latitude_text = fields[2].trim().to_string();
    let longitude_text = fields[3].trim().to_string();
    let altitude_text = fields[4].trim().to_string();

    Ok(TrustedReferenceCoordinate {
        marker_name: fields[0].trim().to_string(),
        fixture_name: fields[1].trim().to_string(),
        lat_deg: latitude_text.parse::<f64>().map_err(|_| {
            TrustedReferenceCoordinateError::InvalidLatitude {
                line_number,
                value: latitude_text.clone(),
            }
        })?,
        lon_deg: longitude_text.parse::<f64>().map_err(|_| {
            TrustedReferenceCoordinateError::InvalidLongitude {
                line_number,
                value: longitude_text.clone(),
            }
        })?,
        alt_m: altitude_text.parse::<f64>().map_err(|_| {
            TrustedReferenceCoordinateError::InvalidAltitude {
                line_number,
                value: altitude_text.clone(),
            }
        })?,
        source: fields[5].trim().to_string(),
    })
}
