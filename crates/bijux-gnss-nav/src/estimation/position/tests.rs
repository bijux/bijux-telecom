#![allow(missing_docs)]

use crate::estimation::position::solver::{
    geodesy::{ecef_to_enu, ecef_to_geodetic, geodetic_to_ecef},
    invert_4x4,
    weighting::{
        position_measurement_weight, weight_from_cn0, weight_from_elevation,
        weight_from_pseudorange_sigma, PositionWeightingModel, WeightingConfig,
    },
    PositionObservation, PositionSolver,
};
use crate::orbits::gps::GpsEphemeris;
use bijux_gnss_core::api::{Constellation, SatId};

#[test]
fn invert_identity() {
    let inv = invert_4x4([
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])
    .expect("invert identity");
    assert!((inv[0][0] - 1.0).abs() < 1e-12);
}

#[test]
fn pvt_sanity() {
    let solver = PositionSolver::new();
    let eph = GpsEphemeris {
        sat: SatId { constellation: Constellation::Gps, prn: 1 },
        iodc: 0,
        iode: 0,
        week: 0,
        sv_health: 0,
        sv_accuracy: Some(2),
        toe_s: 0.0,
        toc_s: 0.0,
        sqrt_a: 5153.7954775,
        e: 0.0,
        i0: 0.94,
        idot: 0.0,
        omega0: 0.0,
        omegadot: 0.0,
        w: 0.0,
        m0: 0.0,
        delta_n: 0.0,
        cuc: 0.0,
        cus: 0.0,
        crc: 0.0,
        crs: 0.0,
        cic: 0.0,
        cis: 0.0,
        af0: 0.0,
        af1: 0.0,
        af2: 0.0,
        tgd: 0.0,
    };
    let obs = vec![
        PositionObservation {
            sat: SatId { constellation: Constellation::Gps, prn: 1 },
            pseudorange_m: 20_000_000.0,
            doppler_hz: None,
            doppler_var_hz2: None,
            cn0_dbhz: 40.0,
            elevation_deg: None,
            weight: 1.0,
            gps_receive_time: None,
            signal_timing: None,
            signal_id: None,
        },
        PositionObservation {
            sat: SatId { constellation: Constellation::Gps, prn: 1 },
            pseudorange_m: 20_000_000.0,
            doppler_hz: None,
            doppler_var_hz2: None,
            cn0_dbhz: 40.0,
            elevation_deg: None,
            weight: 1.0,
            gps_receive_time: None,
            signal_timing: None,
            signal_id: None,
        },
        PositionObservation {
            sat: SatId { constellation: Constellation::Gps, prn: 1 },
            pseudorange_m: 20_000_000.0,
            doppler_hz: None,
            doppler_var_hz2: None,
            cn0_dbhz: 40.0,
            elevation_deg: None,
            weight: 1.0,
            gps_receive_time: None,
            signal_timing: None,
            signal_id: None,
        },
        PositionObservation {
            sat: SatId { constellation: Constellation::Gps, prn: 1 },
            pseudorange_m: 20_000_000.0,
            doppler_hz: None,
            doppler_var_hz2: None,
            cn0_dbhz: 40.0,
            elevation_deg: None,
            weight: 1.0,
            gps_receive_time: None,
            signal_timing: None,
            signal_id: None,
        },
    ];
    let res = solver.solve_wls(&obs, &[eph], 0.0);
    assert!(res.is_none());
}

#[test]
fn ecef_lla_roundtrip_equator() {
    let (x, y, z) = geodetic_to_ecef(0.0, 0.0, 0.0);
    assert!((x - 6378137.0).abs() < 1.0);
    assert!(y.abs() < 1.0);
    assert!(z.abs() < 1.0);
    let (lat, lon, alt) = ecef_to_geodetic(x, y, z);
    assert!(lat.abs() < 1e-6);
    assert!(lon.abs() < 1e-6);
    assert!(alt.abs() < 1.0);
}

#[test]
fn enu_zero_at_reference() {
    let (x, y, z) = geodetic_to_ecef(37.0, -122.0, 10.0);
    let (e, n, u) = ecef_to_enu(x, y, z, 37.0, -122.0, 10.0);
    assert!(e.abs() < 1e-6);
    assert!(n.abs() < 1e-6);
    assert!(u.abs() < 1e-6);
}

#[test]
fn pseudorange_sigma_weight_prefers_smaller_sigma() {
    let precise = weight_from_pseudorange_sigma(Some(2.0));
    let noisy = weight_from_pseudorange_sigma(Some(20.0));

    assert!((precise - 0.25).abs() < 1.0e-12);
    assert!((noisy - 0.0025).abs() < 1.0e-12);
    assert!(precise > noisy);
}

#[test]
fn elevation_weight_prefers_high_elevation_satellites() {
    let config = WeightingConfig::default();
    let low_elevation_weight = weight_from_elevation(10.0, config);
    let high_elevation_weight = weight_from_elevation(75.0, config);

    assert!(low_elevation_weight.is_finite());
    assert!(high_elevation_weight.is_finite());
    assert!(low_elevation_weight < high_elevation_weight);
}

#[test]
fn elevation_weight_respects_minimum_weight_floor() {
    let config = WeightingConfig {
        model: PositionWeightingModel::Elevation,
        enabled: true,
        min_elev_deg: 5.0,
        elev_exponent: 4.0,
        cn0_ref_dbhz: 50.0,
        min_weight: 0.2,
    };
    let horizon_weight = weight_from_elevation(0.0, config);

    assert!((horizon_weight - 0.2).abs() < 1.0e-12);
}

#[test]
fn cn0_weight_prefers_stronger_signals() {
    let config =
        WeightingConfig { model: PositionWeightingModel::Cn0, ..WeightingConfig::default() };
    let weak_signal_weight = weight_from_cn0(28.0, config);
    let strong_signal_weight = weight_from_cn0(48.0, config);

    assert!(weak_signal_weight.is_finite());
    assert!(strong_signal_weight.is_finite());
    assert!(weak_signal_weight < strong_signal_weight);
}

#[test]
fn cn0_weight_respects_minimum_weight_floor() {
    let config = WeightingConfig {
        model: PositionWeightingModel::Cn0,
        cn0_ref_dbhz: 45.0,
        min_weight: 0.25,
        ..WeightingConfig::default()
    };
    let floor_weight = weight_from_cn0(0.0, config);

    assert!((floor_weight - 0.25).abs() < 1.0e-12);
}

#[test]
fn elevation_cn0_weight_multiplies_geometry_terms() {
    let config = WeightingConfig {
        model: PositionWeightingModel::ElevationCn0,
        ..WeightingConfig::default()
    };
    let elevation_weight = weight_from_elevation(30.0, config);
    let cn0_weight = weight_from_cn0(35.0, config);
    let combined = position_measurement_weight(Some(35.0), Some(30.0), None, config);

    assert!((combined - (elevation_weight * cn0_weight)).abs() < 1.0e-12);
}

#[test]
fn composite_position_weight_multiplies_cn0_and_sigma_terms() {
    let config =
        WeightingConfig { model: PositionWeightingModel::Cn0, ..WeightingConfig::default() };
    let geometry_weight = weight_from_cn0(35.0, config);
    let sigma_weight = weight_from_pseudorange_sigma(Some(4.0));
    let composite = position_measurement_weight(Some(35.0), None, Some(4.0), config);

    assert!((composite - (geometry_weight * sigma_weight)).abs() < 1.0e-12);
}

#[test]
fn composite_position_weight_multiplies_elevation_cn0_and_sigma_terms() {
    let config = WeightingConfig {
        model: PositionWeightingModel::ElevationCn0,
        ..WeightingConfig::default()
    };
    let elevation_weight = weight_from_elevation(30.0, config);
    let cn0_weight = weight_from_cn0(35.0, config);
    let sigma_weight = weight_from_pseudorange_sigma(Some(4.0));
    let composite = position_measurement_weight(Some(35.0), Some(30.0), Some(4.0), config);

    assert!((composite - (elevation_weight * cn0_weight * sigma_weight)).abs() < 1.0e-12);
}

#[test]
fn composite_position_weight_multiplies_geometry_and_sigma_terms() {
    let config = WeightingConfig::default();
    let geometry_weight = weight_from_elevation(30.0, config);
    let sigma_weight = weight_from_pseudorange_sigma(Some(4.0));
    let composite = position_measurement_weight(None, Some(30.0), Some(4.0), config);

    assert!((composite - (geometry_weight * sigma_weight)).abs() < 1.0e-12);
}

#[test]
fn composite_position_weight_falls_back_to_unit_sigma_weight() {
    let config = WeightingConfig::default();
    let geometry_only = position_measurement_weight(None, Some(30.0), None, config);
    let invalid_sigma = position_measurement_weight(None, Some(30.0), Some(f64::NAN), config);

    assert!((geometry_only - invalid_sigma).abs() < 1.0e-12);
}
