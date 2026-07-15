#![allow(missing_docs)]

use serde::{Deserialize, Serialize};

use crate::formats::clk::ClkProvider;
use crate::formats::sp3::Sp3RecordAccuracy;
use crate::orbits::beidou::BeidouBroadcastNavigationData;
use crate::orbits::galileo::GalileoBroadcastNavigationData;
use crate::orbits::glonass::GlonassBroadcastNavigationFrame;
use crate::orbits::gps::GpsEphemeris;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SatelliteHealthStatus {
    Healthy,
    Unhealthy,
    Unknown,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SatelliteOrbitUncertaintySource {
    GpsUra,
    GalileoSisa,
    BeidouUrai,
    GlonassAccuracyCode,
    Sp3Accuracy,
    Unavailable,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SatelliteClockUncertaintySource {
    GpsUra,
    GalileoSisa,
    BeidouUrai,
    GlonassAccuracyCode,
    Sp3ClockAccuracy,
    ClkSigma,
    Unavailable,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SatelliteHealthSource {
    GpsSvHealth,
    GalileoSignalHealth,
    BeidouAutonomousHealth,
    GlonassImmediateHealth,
    Sp3Flags,
    Unavailable,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct SatelliteStateUncertainty {
    pub orbit_sigma_m: Option<f64>,
    pub clock_sigma_s: Option<f64>,
    pub orbit_source: SatelliteOrbitUncertaintySource,
    pub clock_source: SatelliteClockUncertaintySource,
    pub health_status: SatelliteHealthStatus,
    pub health_source: SatelliteHealthSource,
}

impl SatelliteStateUncertainty {
    pub const fn unavailable() -> Self {
        Self {
            orbit_sigma_m: None,
            clock_sigma_s: None,
            orbit_source: SatelliteOrbitUncertaintySource::Unavailable,
            clock_source: SatelliteClockUncertaintySource::Unavailable,
            health_status: SatelliteHealthStatus::Unknown,
            health_source: SatelliteHealthSource::Unavailable,
        }
    }

    pub fn with_clock_sigma_s(
        mut self,
        clock_sigma_s: Option<f64>,
        clock_source: SatelliteClockUncertaintySource,
    ) -> Self {
        if let Some(sigma_s) = finite_positive(clock_sigma_s) {
            self.clock_sigma_s = Some(sigma_s);
            self.clock_source = clock_source;
        }
        self
    }
}

pub fn gps_broadcast_uncertainty(eph: &GpsEphemeris) -> SatelliteStateUncertainty {
    SatelliteStateUncertainty {
        orbit_sigma_m: None,
        clock_sigma_s: None,
        orbit_source: SatelliteOrbitUncertaintySource::Unavailable,
        clock_source: SatelliteClockUncertaintySource::Unavailable,
        health_status: if eph.sv_health == 0 {
            SatelliteHealthStatus::Healthy
        } else {
            SatelliteHealthStatus::Unhealthy
        },
        health_source: SatelliteHealthSource::GpsSvHealth,
    }
}

pub fn galileo_broadcast_uncertainty(
    navigation: &GalileoBroadcastNavigationData,
) -> SatelliteStateUncertainty {
    let orbit_sigma_m = galileo_sisa_sigma_m(navigation.sisa_e1_e5b);
    SatelliteStateUncertainty {
        orbit_sigma_m,
        clock_sigma_s: None,
        orbit_source: source_for_optional_orbit(
            orbit_sigma_m,
            SatelliteOrbitUncertaintySource::GalileoSisa,
        ),
        clock_source: SatelliteClockUncertaintySource::Unavailable,
        health_status: if navigation.signal_health.e1b_data_valid
            && navigation.signal_health.e1b_signal_health == 0
        {
            SatelliteHealthStatus::Healthy
        } else {
            SatelliteHealthStatus::Unhealthy
        },
        health_source: SatelliteHealthSource::GalileoSignalHealth,
    }
}

pub fn beidou_broadcast_uncertainty(
    navigation: &BeidouBroadcastNavigationData,
) -> SatelliteStateUncertainty {
    let orbit_sigma_m = beidou_urai_sigma_m(navigation.urai);
    SatelliteStateUncertainty {
        orbit_sigma_m,
        clock_sigma_s: None,
        orbit_source: source_for_optional_orbit(
            orbit_sigma_m,
            SatelliteOrbitUncertaintySource::BeidouUrai,
        ),
        clock_source: SatelliteClockUncertaintySource::Unavailable,
        health_status: if navigation.signal_health.autonomous_satellite_good {
            SatelliteHealthStatus::Healthy
        } else {
            SatelliteHealthStatus::Unhealthy
        },
        health_source: SatelliteHealthSource::BeidouAutonomousHealth,
    }
}

pub fn glonass_broadcast_uncertainty(
    navigation: &GlonassBroadcastNavigationFrame,
) -> SatelliteStateUncertainty {
    let orbit_sigma_m = navigation.immediate.accuracy_code.and_then(glonass_accuracy_code_sigma_m);
    SatelliteStateUncertainty {
        orbit_sigma_m,
        clock_sigma_s: None,
        orbit_source: source_for_optional_orbit(
            orbit_sigma_m,
            SatelliteOrbitUncertaintySource::GlonassAccuracyCode,
        ),
        clock_source: SatelliteClockUncertaintySource::Unavailable,
        health_status: if navigation.immediate.health.line_unhealthy {
            SatelliteHealthStatus::Unhealthy
        } else {
            SatelliteHealthStatus::Healthy
        },
        health_source: SatelliteHealthSource::GlonassImmediateHealth,
    }
}

pub fn sp3_accuracy_uncertainty(accuracy: Sp3RecordAccuracy) -> SatelliteStateUncertainty {
    let orbit_sigma_m = max_finite_positive([accuracy.x_m, accuracy.y_m, accuracy.z_m]);
    SatelliteStateUncertainty {
        orbit_sigma_m,
        clock_sigma_s: finite_positive(accuracy.clock_s),
        orbit_source: source_for_optional_orbit(
            orbit_sigma_m,
            SatelliteOrbitUncertaintySource::Sp3Accuracy,
        ),
        clock_source: if finite_positive(accuracy.clock_s).is_some() {
            SatelliteClockUncertaintySource::Sp3ClockAccuracy
        } else {
            SatelliteClockUncertaintySource::Unavailable
        },
        health_status: SatelliteHealthStatus::Healthy,
        health_source: SatelliteHealthSource::Sp3Flags,
    }
}

pub fn clk_sigma_uncertainty(
    provider: &ClkProvider,
    sat: bijux_gnss_core::api::SatId,
    t_s: f64,
) -> Option<f64> {
    finite_positive(provider.sigma_s(sat, t_s))
}

pub fn galileo_sisa_sigma_m(index: u8) -> Option<f64> {
    match index {
        0..=49 => Some(f64::from(index) * 0.01),
        50..=74 => Some(0.5 + f64::from(index - 50) * 0.02),
        75..=99 => Some(1.0 + f64::from(index - 75) * 0.04),
        100..=125 => Some(2.0 + f64::from(index - 100) * 0.16),
        _ => None,
    }
}

pub fn beidou_urai_sigma_m(index: u8) -> Option<f64> {
    const URAI_SIGMA_M: [f64; 15] = [
        2.4, 3.4, 4.85, 6.85, 9.65, 13.65, 24.0, 48.0, 96.0, 192.0, 384.0, 768.0, 1536.0, 3072.0,
        6144.0,
    ];
    URAI_SIGMA_M.get(usize::from(index)).copied()
}

pub fn glonass_accuracy_code_sigma_m(code: u8) -> Option<f64> {
    const ACCURACY_SIGMA_M: [f64; 15] =
        [1.0, 2.0, 2.5, 4.0, 5.0, 7.0, 10.0, 12.0, 14.0, 16.0, 32.0, 64.0, 128.0, 256.0, 512.0];
    ACCURACY_SIGMA_M.get(usize::from(code)).copied()
}

fn source_for_optional_orbit(
    sigma_m: Option<f64>,
    source: SatelliteOrbitUncertaintySource,
) -> SatelliteOrbitUncertaintySource {
    if sigma_m.is_some() {
        source
    } else {
        SatelliteOrbitUncertaintySource::Unavailable
    }
}

fn finite_positive(value: Option<f64>) -> Option<f64> {
    value.filter(|value| value.is_finite() && *value >= 0.0)
}

fn max_finite_positive(values: [Option<f64>; 3]) -> Option<f64> {
    values.into_iter().filter_map(finite_positive).reduce(f64::max)
}

#[cfg(test)]
mod tests {
    use super::{
        beidou_urai_sigma_m, galileo_sisa_sigma_m, glonass_accuracy_code_sigma_m,
        sp3_accuracy_uncertainty, SatelliteClockUncertaintySource, SatelliteOrbitUncertaintySource,
    };
    use crate::formats::sp3::Sp3RecordAccuracy;

    #[test]
    fn galileo_sisa_mapping_reports_meters_and_refuses_unmonitored_codes() {
        assert_eq!(galileo_sisa_sigma_m(49), Some(0.49));
        assert_eq!(galileo_sisa_sigma_m(50), Some(0.5));
        assert_eq!(galileo_sisa_sigma_m(75), Some(1.0));
        assert_eq!(galileo_sisa_sigma_m(100), Some(2.0));
        assert_eq!(galileo_sisa_sigma_m(126), None);
    }

    #[test]
    fn broadcast_accuracy_mappings_refuse_unusable_codes() {
        assert_eq!(beidou_urai_sigma_m(0), Some(2.4));
        assert_eq!(beidou_urai_sigma_m(14), Some(6144.0));
        assert_eq!(beidou_urai_sigma_m(15), None);
        assert_eq!(glonass_accuracy_code_sigma_m(6), Some(10.0));
        assert_eq!(glonass_accuracy_code_sigma_m(15), None);
    }

    #[test]
    fn sp3_accuracy_uses_largest_axis_and_clock_sigma() {
        let uncertainty = sp3_accuracy_uncertainty(Sp3RecordAccuracy {
            x_m: Some(0.016),
            y_m: Some(0.032),
            z_m: Some(0.064),
            clock_s: Some(128.0e-12),
        });

        assert_eq!(uncertainty.orbit_sigma_m, Some(0.064));
        assert_eq!(uncertainty.clock_sigma_s, Some(128.0e-12));
        assert_eq!(uncertainty.orbit_source, SatelliteOrbitUncertaintySource::Sp3Accuracy);
        assert_eq!(uncertainty.clock_source, SatelliteClockUncertaintySource::Sp3ClockAccuracy);
    }
}
