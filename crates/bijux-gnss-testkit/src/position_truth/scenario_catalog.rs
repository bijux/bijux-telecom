use std::collections::BTreeMap;

use bijux_gnss_core::api::{Constellation, SatId};
use bijux_gnss_nav::api::{GpsEphemeris, KlobucharCoefficients, PositionObservation};

use crate::reference_math::coordinates::GeodeticPoint;

use super::{geodetic_point_to_tuple, timed_position_observation_from_truth};

#[derive(Debug, Clone, Copy, Default)]
pub struct BroadcastClockParameters {
    pub af0_s: f64,
    pub af1_s_per_s: f64,
    pub af2_s_per_s2: f64,
    pub tgd_s: f64,
}

#[derive(Debug, Clone)]
pub struct SyntheticPositionScenario {
    pub ephemerides: Vec<GpsEphemeris>,
    pub observations: Vec<PositionObservation>,
    pub truth_ecef_m: (f64, f64, f64),
    pub receiver_clock_bias_s: f64,
    pub t_rx_s: f64,
}

pub fn four_satellite_position_scenario(receiver_clock_bias_s: f64) -> SyntheticPositionScenario {
    four_satellite_position_scenario_from_truth(
        receiver_clock_bias_s,
        geodetic_point_to_tuple(GeodeticPoint { lat_deg: 37.0, lon_deg: -122.0, alt_m: 10.0 }),
        sample_ephemerides(),
    )
}

pub fn impossible_geometry_position_scenario(
    receiver_clock_bias_s: f64,
) -> SyntheticPositionScenario {
    four_satellite_position_scenario_from_truth(
        receiver_clock_bias_s,
        geodetic_point_to_tuple(GeodeticPoint {
            lat_deg: 37.0,
            lon_deg: -122.0,
            alt_m: -2_000_000.0,
        }),
        sample_ephemerides(),
    )
}

pub fn four_satellite_position_scenario_with_ephemerides(
    receiver_clock_bias_s: f64,
    ephemerides: Vec<GpsEphemeris>,
) -> SyntheticPositionScenario {
    four_satellite_position_scenario_from_truth(
        receiver_clock_bias_s,
        geodetic_point_to_tuple(GeodeticPoint { lat_deg: 37.0, lon_deg: -122.0, alt_m: 10.0 }),
        ephemerides,
    )
}

pub fn four_satellite_position_scenario_from_truth(
    receiver_clock_bias_s: f64,
    truth_ecef_m: (f64, f64, f64),
    ephemerides: Vec<GpsEphemeris>,
) -> SyntheticPositionScenario {
    let t_rx_s = 504_018.07 + receiver_clock_bias_s;
    let observations = ephemerides
        .iter()
        .map(|ephemeris| {
            timed_position_observation_from_truth(
                ephemeris,
                truth_ecef_m,
                t_rx_s,
                receiver_clock_bias_s,
            )
        })
        .collect();

    SyntheticPositionScenario {
        ephemerides,
        observations,
        truth_ecef_m,
        receiver_clock_bias_s,
        t_rx_s,
    }
}

pub fn sample_ephemerides() -> Vec<GpsEphemeris> {
    sample_ephemerides_with_clock_parameters(&[])
}

pub fn sample_ephemerides_with_clock_parameters(
    clock_parameters_by_prn: &[(u8, BroadcastClockParameters)],
) -> Vec<GpsEphemeris> {
    let clock_parameters_by_prn =
        clock_parameters_by_prn.iter().copied().collect::<BTreeMap<_, _>>();
    vec![
        sample_ephemeris_with_clock_parameters(
            1,
            0.0,
            0.0,
            clock_parameters_by_prn.get(&1).copied().unwrap_or_default(),
        ),
        sample_ephemeris_with_clock_parameters(
            2,
            0.8,
            0.9,
            clock_parameters_by_prn.get(&2).copied().unwrap_or_default(),
        ),
        sample_ephemeris_with_clock_parameters(
            3,
            1.6,
            1.8,
            clock_parameters_by_prn.get(&3).copied().unwrap_or_default(),
        ),
        sample_ephemeris_with_clock_parameters(
            4,
            2.4,
            2.7,
            clock_parameters_by_prn.get(&4).copied().unwrap_or_default(),
        ),
    ]
}

pub fn sample_ephemeris(prn: u8, omega0: f64, m0: f64) -> GpsEphemeris {
    sample_ephemeris_with_clock_parameters(prn, omega0, m0, BroadcastClockParameters::default())
}

pub fn sample_ephemeris_with_clock_parameters(
    prn: u8,
    omega0: f64,
    m0: f64,
    clock_parameters: BroadcastClockParameters,
) -> GpsEphemeris {
    GpsEphemeris {
        sat: SatId { constellation: Constellation::Gps, prn },
        iodc: 1,
        iode: 1,
        week: 2209,
        sv_health: 0,
        sv_accuracy: Some(2),
        toe_s: 504_000.0,
        toc_s: 504_018.0,
        sqrt_a: 5153.7954775,
        e: 0.01,
        i0: 0.94,
        idot: 0.0,
        omega0,
        omegadot: 0.0,
        w: 0.0,
        m0,
        delta_n: 0.0,
        cuc: 0.0,
        cus: 0.0,
        crc: 0.0,
        crs: 0.0,
        cic: 0.0,
        cis: 0.0,
        af0: clock_parameters.af0_s,
        af1: clock_parameters.af1_s_per_s,
        af2: clock_parameters.af2_s_per_s2,
        tgd: clock_parameters.tgd_s,
    }
}

pub fn clear_broadcast_clock_parameters(ephemerides: &[GpsEphemeris]) -> Vec<GpsEphemeris> {
    ephemerides
        .iter()
        .cloned()
        .map(|mut ephemeris| {
            ephemeris.af0 = 0.0;
            ephemeris.af1 = 0.0;
            ephemeris.af2 = 0.0;
            ephemeris.tgd = 0.0;
            ephemeris
        })
        .collect()
}

pub fn sample_klobuchar_coefficients() -> KlobucharCoefficients {
    KlobucharCoefficients::new(
        [0.1212e-7, 0.1490e-7, -0.5960e-7, 0.1192e-6],
        [0.1167e6, -0.2294e6, -0.1311e6, 0.1049e7],
    )
}
