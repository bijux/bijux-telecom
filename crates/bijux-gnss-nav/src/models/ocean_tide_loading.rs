//! Ocean tide loading displacement model.

use std::f64::consts::TAU;

use bijux_gnss_core::api::GpsTime;
use serde::{Deserialize, Serialize};

use crate::estimation::position::solver::geodesy::ecef_to_geodetic;

/// Site-specific ocean tide loading model for a static receiver monument.
///
/// The model sums harmonic east, north, and up displacements for a set of
/// standard ocean-tide constituents relative to a reference GPS epoch.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OceanTideLoadingModel {
    /// Reference epoch for constituent phases.
    pub reference_time: GpsTime,
    /// Harmonic constituent definitions for the site.
    pub constituents: Vec<OceanTideLoadingConstituent>,
}

impl OceanTideLoadingModel {
    /// Returns whether the model contains only finite parameters.
    pub fn is_physical(&self) -> bool {
        self.reference_time.tow_s.is_finite()
            && self.constituents.iter().all(OceanTideLoadingConstituent::is_physical)
    }

    /// Computes the total ENU displacement at a GPS epoch.
    pub fn displacement_enu_m(&self, time: GpsTime) -> [f64; 3] {
        self.constituents.iter().fold([0.0_f64; 3], |mut total, constituent| {
            let displacement = constituent.displacement_enu_m(self.reference_time, time);
            total[0] += displacement[0];
            total[1] += displacement[1];
            total[2] += displacement[2];
            total
        })
    }

    /// Computes the total ECEF displacement for a receiver site at a GPS epoch.
    pub fn displacement_ecef_m(
        &self,
        receiver_ecef_m: [f64; 3],
        time: GpsTime,
    ) -> Option<[f64; 3]> {
        if !self.is_physical() {
            return None;
        }

        let receiver_radius_m =
            receiver_ecef_m.iter().map(|component| component * component).sum::<f64>().sqrt();
        if !receiver_radius_m.is_finite() || receiver_radius_m <= 0.0 {
            return None;
        }

        let (lat_deg, lon_deg, _alt_m) =
            ecef_to_geodetic(receiver_ecef_m[0], receiver_ecef_m[1], receiver_ecef_m[2]);
        if !lat_deg.is_finite() || !lon_deg.is_finite() {
            return None;
        }

        Some(enu_delta_to_ecef_m(receiver_ecef_m, self.displacement_enu_m(time), lat_deg, lon_deg))
    }
}

/// Harmonic displacement definition for one standard ocean-tide constituent.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OceanTideLoadingConstituent {
    /// Standard constituent identifier.
    pub constituent: OceanTideConstituent,
    /// East displacement amplitude, in meters.
    pub east_amplitude_m: f64,
    /// East displacement phase at the reference epoch, in degrees.
    pub east_phase_deg: f64,
    /// North displacement amplitude, in meters.
    pub north_amplitude_m: f64,
    /// North displacement phase at the reference epoch, in degrees.
    pub north_phase_deg: f64,
    /// Up displacement amplitude, in meters.
    pub up_amplitude_m: f64,
    /// Up displacement phase at the reference epoch, in degrees.
    pub up_phase_deg: f64,
}

impl OceanTideLoadingConstituent {
    /// Creates a constituent from ENU amplitudes and phases.
    pub fn new(
        constituent: OceanTideConstituent,
        east_amplitude_m: f64,
        east_phase_deg: f64,
        north_amplitude_m: f64,
        north_phase_deg: f64,
        up_amplitude_m: f64,
        up_phase_deg: f64,
    ) -> Self {
        Self {
            constituent,
            east_amplitude_m,
            east_phase_deg,
            north_amplitude_m,
            north_phase_deg,
            up_amplitude_m,
            up_phase_deg,
        }
    }

    /// Returns whether the constituent contains finite harmonic parameters.
    pub fn is_physical(&self) -> bool {
        [
            self.east_amplitude_m,
            self.east_phase_deg,
            self.north_amplitude_m,
            self.north_phase_deg,
            self.up_amplitude_m,
            self.up_phase_deg,
        ]
        .into_iter()
        .all(f64::is_finite)
    }

    /// Computes ENU displacement for the constituent at a GPS epoch.
    pub fn displacement_enu_m(&self, reference_time: GpsTime, time: GpsTime) -> [f64; 3] {
        let delta_s = time.to_seconds() - reference_time.to_seconds();
        let angular_phase_rad = TAU * self.constituent.cycles_per_day() * delta_s / 86_400.0;
        [
            harmonic_component_m(self.east_amplitude_m, self.east_phase_deg, angular_phase_rad),
            harmonic_component_m(self.north_amplitude_m, self.north_phase_deg, angular_phase_rad),
            harmonic_component_m(self.up_amplitude_m, self.up_phase_deg, angular_phase_rad),
        ]
    }
}

/// Standard ocean-tide constituent frequency identifiers.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum OceanTideConstituent {
    /// Principal lunar semidiurnal constituent.
    M2,
    /// Principal solar semidiurnal constituent.
    S2,
    /// Larger lunar elliptic semidiurnal constituent.
    N2,
    /// Lunisolar semidiurnal constituent.
    K2,
    /// Lunisolar diurnal constituent.
    K1,
    /// Principal lunar diurnal constituent.
    O1,
    /// Principal solar diurnal constituent.
    P1,
    /// Larger lunar elliptic diurnal constituent.
    Q1,
    /// Fortnightly lunar constituent.
    Mf,
    /// Monthly lunar constituent.
    Mm,
    /// Solar semiannual constituent.
    Ssa,
}

impl OceanTideConstituent {
    /// Returns the constituent frequency in cycles per solar day.
    pub fn cycles_per_day(self) -> f64 {
        match self {
            Self::M2 => 1.932_273_6,
            Self::S2 => 2.0,
            Self::N2 => 1.895_981_86,
            Self::K2 => 2.005_475_8,
            Self::K1 => 1.002_737_9,
            Self::O1 => 0.929_535_7,
            Self::P1 => 0.997_262_1,
            Self::Q1 => 0.893_244,
            Self::Mf => 0.073_202_2,
            Self::Mm => 0.036_291_6,
            Self::Ssa => 0.005_475_8,
        }
    }
}

fn harmonic_component_m(amplitude_m: f64, phase_deg: f64, angular_phase_rad: f64) -> f64 {
    amplitude_m * (angular_phase_rad + phase_deg.to_radians()).cos()
}

fn enu_delta_to_ecef_m(
    receiver_ecef_m: [f64; 3],
    enu_m: [f64; 3],
    lat_deg: f64,
    lon_deg: f64,
) -> [f64; 3] {
    let _ = receiver_ecef_m;
    let (sin_lat, cos_lat) = lat_deg.to_radians().sin_cos();
    let (sin_lon, cos_lon) = lon_deg.to_radians().sin_cos();
    let east_m = enu_m[0];
    let north_m = enu_m[1];
    let up_m = enu_m[2];
    let dx = -sin_lon * east_m - sin_lat * cos_lon * north_m + cos_lat * cos_lon * up_m;
    let dy = cos_lon * east_m - sin_lat * sin_lon * north_m + cos_lat * sin_lon * up_m;
    let dz = cos_lat * north_m + sin_lat * up_m;
    [dx, dy, dz]
}

#[cfg(test)]
mod tests {
    use bijux_gnss_core::api::GpsTime;

    use super::{OceanTideConstituent, OceanTideLoadingConstituent, OceanTideLoadingModel};
    use crate::estimation::position::solver::geodesy::geodetic_to_ecef;

    #[test]
    fn constituent_advances_with_harmonic_frequency() {
        let constituent = OceanTideLoadingConstituent::new(
            OceanTideConstituent::S2,
            0.12,
            0.0,
            0.08,
            90.0,
            0.25,
            180.0,
        );
        let reference_time = GpsTime { week: 2200, tow_s: 0.0 };
        let quarter_cycle_time = reference_time.offset_seconds(10_800.0);

        let displacement = constituent.displacement_enu_m(reference_time, quarter_cycle_time);

        assert!(displacement[0].abs() < 1.0e-12);
        assert!((displacement[1] + 0.08).abs() < 1.0e-12);
        assert!(displacement[2].abs() < 1.0e-12);
    }

    #[test]
    fn model_rotates_up_displacement_into_ecef_at_equator() {
        let receiver_ecef_m = {
            let (x, y, z) = geodetic_to_ecef(0.0, 0.0, 0.0);
            [x, y, z]
        };
        let model = OceanTideLoadingModel {
            reference_time: GpsTime { week: 2200, tow_s: 0.0 },
            constituents: vec![OceanTideLoadingConstituent::new(
                OceanTideConstituent::M2,
                0.0,
                0.0,
                0.0,
                0.0,
                0.15,
                0.0,
            )],
        };

        let displacement = model
            .displacement_ecef_m(receiver_ecef_m, GpsTime { week: 2200, tow_s: 0.0 })
            .expect("ECEF displacement");

        assert!((displacement[0] - 0.15).abs() < 1.0e-9);
        assert!(displacement[1].abs() < 1.0e-9);
        assert!(displacement[2].abs() < 1.0e-9);
    }

    #[test]
    fn model_rejects_non_finite_parameters() {
        let model = OceanTideLoadingModel {
            reference_time: GpsTime { week: 2200, tow_s: 0.0 },
            constituents: vec![OceanTideLoadingConstituent::new(
                OceanTideConstituent::K1,
                f64::NAN,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            )],
        };
        let receiver_ecef_m = [6_378_137.0, 0.0, 0.0];

        assert!(model
            .displacement_ecef_m(receiver_ecef_m, GpsTime { week: 2200, tow_s: 0.0 })
            .is_none());
    }
}
