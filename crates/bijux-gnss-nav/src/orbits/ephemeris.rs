#![allow(missing_docs)]
#![allow(dead_code)]

use bijux_gnss_core::api::{Constellation, SatId};
use thiserror::Error;

#[derive(Debug, Error)]
pub enum EphemerisError {
    #[error("unsupported ephemeris format")]
    UnsupportedFormat,
}

/// Broadcast ephemeris parameters.
#[derive(Debug, Clone)]
pub struct Ephemeris {
    pub sat: SatId,
    pub toe_s: f64,
}

pub trait EphemerisProvider {
    fn ephemeris(&self, sat: SatId) -> Option<Ephemeris>;
}

#[derive(Debug, Clone)]
pub struct CsvEphemerisProvider {
    entries: Vec<Ephemeris>,
}

impl CsvEphemerisProvider {
    pub fn from_csv(path: &std::path::Path) -> Result<Self, EphemerisError> {
        let data = std::fs::read_to_string(path).map_err(|_| EphemerisError::UnsupportedFormat)?;
        let mut entries = Vec::new();
        for line in data.lines() {
            let line = line.trim();
            if line.is_empty() || line.starts_with('#') {
                continue;
            }
            let mut parts = line.splitn(2, ',');
            let prn_str = parts.next().unwrap_or("");
            let toe_str = parts.next().unwrap_or("");
            let prn: u8 = match prn_str.trim().parse() {
                Ok(val) => val,
                Err(_) => continue,
            };
            let toe_s: f64 = match toe_str.trim().parse() {
                Ok(val) => val,
                Err(_) => continue,
            };
            entries.push(Ephemeris {
                sat: SatId {
                    constellation: Constellation::Gps,
                    prn,
                },
                toe_s,
            });
        }
        if entries.is_empty() {
            return Err(EphemerisError::UnsupportedFormat);
        }
        Ok(Self { entries })
    }
}

impl EphemerisProvider for CsvEphemerisProvider {
    fn ephemeris(&self, sat: SatId) -> Option<Ephemeris> {
        self.entries.iter().find(|e| e.sat == sat).cloned()
    }
}
