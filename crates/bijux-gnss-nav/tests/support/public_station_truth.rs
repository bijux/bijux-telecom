#![allow(dead_code)]
#![allow(missing_docs)]

use std::path::PathBuf;
use std::sync::OnceLock;

use bijux_gnss_nav::api::{ecef_to_enu, geodetic_to_ecef};

#[derive(Debug, Clone, PartialEq)]
pub struct PublicStationTruth {
    pub marker_name: String,
    pub fixture_name: String,
    pub lat_deg: f64,
    pub lon_deg: f64,
    pub alt_m: f64,
    pub source: String,
}

impl PublicStationTruth {
    pub fn truth_ecef_m(&self) -> (f64, f64, f64) {
        geodetic_to_ecef(self.lat_deg, self.lon_deg, self.alt_m)
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EnuError {
    pub east_m: f64,
    pub north_m: f64,
    pub up_m: f64,
    pub horizontal_m: f64,
    pub three_dimensional_m: f64,
}

pub fn public_station_truth_by_fixture(fixture_name: &str) -> PublicStationTruth {
    load_public_station_truths()
        .iter()
        .find(|truth| truth.fixture_name == fixture_name)
        .cloned()
        .unwrap_or_else(|| panic!("missing public station truth for fixture {fixture_name}"))
}

pub fn station_enu_error_m(
    solution_ecef_m: (f64, f64, f64),
    truth: &PublicStationTruth,
) -> EnuError {
    let (east_m, north_m, up_m) = ecef_to_enu(
        solution_ecef_m.0,
        solution_ecef_m.1,
        solution_ecef_m.2,
        truth.lat_deg,
        truth.lon_deg,
        truth.alt_m,
    );
    EnuError {
        east_m,
        north_m,
        up_m,
        horizontal_m: (east_m * east_m + north_m * north_m).sqrt(),
        three_dimensional_m: (east_m * east_m + north_m * north_m + up_m * up_m).sqrt(),
    }
}

fn load_public_station_truths() -> &'static Vec<PublicStationTruth> {
    static TRUTHS: OnceLock<Vec<PublicStationTruth>> = OnceLock::new();
    TRUTHS.get_or_init(|| {
        parse_public_station_truth_csv(&fixture("public_station_truth.csv"))
            .expect("parse public station truth manifest")
    })
}

fn fixture(name: &str) -> String {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/data").join(name);
    std::fs::read_to_string(&path)
        .unwrap_or_else(|_| panic!("read public station truth fixture {}", path.display()))
}

fn parse_public_station_truth_csv(data: &str) -> Result<Vec<PublicStationTruth>, String> {
    let mut lines = data.lines();
    let Some(header) = lines.next() else {
        return Err("public station truth manifest is empty".to_string());
    };
    if header.trim() != "marker_name,fixture_name,lat_deg,lon_deg,alt_m,source" {
        return Err(format!("unexpected public station truth header '{header}'"));
    }

    lines
        .enumerate()
        .filter(|(_, line)| !line.trim().is_empty())
        .map(|(index, line)| parse_public_station_truth_line(index + 2, line))
        .collect()
}

fn parse_public_station_truth_line(
    line_number: usize,
    line: &str,
) -> Result<PublicStationTruth, String> {
    let fields = line.splitn(6, ',').collect::<Vec<_>>();
    if fields.len() != 6 {
        return Err(format!(
            "public station truth line {line_number} expected 6 fields, found {}",
            fields.len()
        ));
    }

    Ok(PublicStationTruth {
        marker_name: fields[0].trim().to_string(),
        fixture_name: fields[1].trim().to_string(),
        lat_deg: fields[2]
            .trim()
            .parse::<f64>()
            .map_err(|err| format!("invalid latitude on line {line_number}: {err}"))?,
        lon_deg: fields[3]
            .trim()
            .parse::<f64>()
            .map_err(|err| format!("invalid longitude on line {line_number}: {err}"))?,
        alt_m: fields[4]
            .trim()
            .parse::<f64>()
            .map_err(|err| format!("invalid altitude on line {line_number}: {err}"))?,
        source: fields[5].trim().to_string(),
    })
}
