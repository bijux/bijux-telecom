use std::fs;
use std::path::Path;

use bijux_gnss_core::api::{Constellation, GpsTime, ObsSignalTiming, SatId, Seconds};
use bijux_gnss_nav::api::{
    geodetic_to_ecef, sat_state_gps_l1ca, GpsEphemeris, PositionObservation, PositionSolver,
};
use serde::Deserialize;

#[derive(Debug, Deserialize)]
struct NavFixture {
    id: String,
    t_rx_s: f64,
    truth: TruthFixture,
    ephemerides: Vec<EphemerisFixture>,
    expect: NavExpectFixture,
}

#[derive(Debug, Deserialize)]
struct TruthFixture {
    lat_deg: f64,
    lon_deg: f64,
    alt_m: f64,
}

#[derive(Debug, Deserialize)]
struct EphemerisFixture {
    prn: u8,
    omega0: f64,
    m0: f64,
}

#[derive(Debug, Deserialize)]
struct NavExpectFixture {
    max_position_error_m: f64,
    max_clock_bias_s: f64,
    min_used_satellites: usize,
}

#[test]
fn synthetic_pvt_regression_fixtures_are_deterministic() {
    let solver = PositionSolver::new();
    for path in load_fixture_paths() {
        let fixture = load_fixture(&path);
        let truth_ecef =
            geodetic_to_ecef(fixture.truth.lat_deg, fixture.truth.lon_deg, fixture.truth.alt_m);
        let ephs = fixture
            .ephemerides
            .iter()
            .map(|row| make_eph(row.prn, row.omega0, row.m0, fixture.t_rx_s))
            .collect::<Vec<_>>();
        let obs = build_observations(fixture.t_rx_s, truth_ecef, &ephs);

        let first = solver.solve_wls(&obs, &ephs, fixture.t_rx_s).expect("fixture should solve");
        let second = solver
            .solve_wls(&obs, &ephs, fixture.t_rx_s)
            .expect("fixture should solve deterministically");

        let dx = first.ecef_x_m - truth_ecef.0;
        let dy = first.ecef_y_m - truth_ecef.1;
        let dz = first.ecef_z_m - truth_ecef.2;
        let pos_err = (dx * dx + dy * dy + dz * dz).sqrt();

        assert!(
            pos_err <= fixture.expect.max_position_error_m,
            "fixture {} position error {:.3} exceeded {:.3}",
            fixture.id,
            pos_err,
            fixture.expect.max_position_error_m
        );
        assert!(
            first.clock_bias_s.abs() <= fixture.expect.max_clock_bias_s,
            "fixture {} clock bias {} exceeded {}",
            fixture.id,
            first.clock_bias_s,
            fixture.expect.max_clock_bias_s
        );
        assert!(
            first.used_sat_count >= fixture.expect.min_used_satellites,
            "fixture {} used_sat_count {} below {}",
            fixture.id,
            first.used_sat_count,
            fixture.expect.min_used_satellites
        );
        assert!(first.hdop.is_some(), "fixture {} missing hdop", fixture.id);
        assert!(first.vdop.is_some(), "fixture {} missing vdop", fixture.id);
        assert!(first.gdop.is_some(), "fixture {} missing gdop", fixture.id);
        assert!(first.tdop.is_some(), "fixture {} missing tdop", fixture.id);

        assert!(
            (first.ecef_x_m - second.ecef_x_m).abs() <= 1e-9,
            "fixture {} x mismatch across runs",
            fixture.id
        );
        assert!(
            (first.ecef_y_m - second.ecef_y_m).abs() <= 1e-9,
            "fixture {} y mismatch across runs",
            fixture.id
        );
        assert!(
            (first.ecef_z_m - second.ecef_z_m).abs() <= 1e-9,
            "fixture {} z mismatch across runs",
            fixture.id
        );
        assert!(
            (first.clock_bias_s - second.clock_bias_s).abs() <= 1e-12,
            "fixture {} clock mismatch across runs",
            fixture.id
        );
    }
}

fn make_eph(prn: u8, omega0: f64, m0: f64, t_ref_s: f64) -> GpsEphemeris {
    GpsEphemeris {
        sat: SatId { constellation: Constellation::Gps, prn },
        iodc: 0,
        iode: 0,
        week: 0,
        sv_health: 0,
        sv_accuracy: Some(2),
        toe_s: t_ref_s,
        toc_s: t_ref_s,
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
        af0: 0.0,
        af1: 0.0,
        af2: 0.0,
        tgd: 0.0,
    }
}

fn timed_position_observation(sat: SatId, pseudorange_m: f64, t_rx_s: f64) -> PositionObservation {
    let signal_travel_time_s = pseudorange_m / 299_792_458.0;
    PositionObservation {
        sat,
        pseudorange_m,
        doppler_hz: None,
        doppler_var_hz2: None,
        cn0_dbhz: 45.0,
        elevation_deg: Some(45.0),
        weight: 1.0,
        gps_receive_time: Some(GpsTime { week: 0, tow_s: t_rx_s }),
        signal_timing: Some(ObsSignalTiming {
            signal_travel_time_s: Seconds(signal_travel_time_s),
            transmit_gps_time: GpsTime { week: 0, tow_s: t_rx_s - signal_travel_time_s },
        }),
        signal_id: None,
    }
}

fn build_observations(
    t_rx_s: f64,
    position_ecef: (f64, f64, f64),
    ephs: &[GpsEphemeris],
) -> Vec<PositionObservation> {
    ephs.iter()
        .map(|eph| {
            let pseudorange_m = synthetic_pseudorange_m(eph, t_rx_s, position_ecef);
            timed_position_observation(eph.sat, pseudorange_m, t_rx_s)
        })
        .collect()
}

#[test]
fn navigation_solver_accepts_receiver_fixtures_without_signal_timing_when_pseudoranges_are_finite()
{
    let fixture_path = load_fixture_paths().into_iter().next().expect("fixture path");
    let fixture = load_fixture(&fixture_path);
    let truth_ecef =
        geodetic_to_ecef(fixture.truth.lat_deg, fixture.truth.lon_deg, fixture.truth.alt_m);
    let ephs = fixture
        .ephemerides
        .iter()
        .map(|row| make_eph(row.prn, row.omega0, row.m0, fixture.t_rx_s))
        .collect::<Vec<_>>();
    let mut observations = build_observations(fixture.t_rx_s, truth_ecef, &ephs);
    for observation in &mut observations {
        observation.signal_timing = None;
    }

    let solution = PositionSolver::new()
        .solve_wls(&observations, &ephs, fixture.t_rx_s)
        .expect("finite pseudoranges without signal timing should still solve");

    assert!(solution.ecef_x_m.is_finite());
    assert!(solution.ecef_y_m.is_finite());
    assert!(solution.ecef_z_m.is_finite());
    assert!(solution.used_sat_count >= fixture.expect.min_used_satellites);
}

fn synthetic_pseudorange_m(eph: &GpsEphemeris, t_rx_s: f64, position_ecef: (f64, f64, f64)) -> f64 {
    let c = 299_792_458.0;
    let mut tau = 0.07;
    let mut pseudorange_m = 0.0;
    for _ in 0..10 {
        let sat = sat_state_gps_l1ca(eph, t_rx_s - tau, tau);
        let dx = position_ecef.0 - sat.x_m;
        let dy = position_ecef.1 - sat.y_m;
        let dz = position_ecef.2 - sat.z_m;
        let range = (dx * dx + dy * dy + dz * dz).sqrt();
        pseudorange_m = range - sat.clock_correction.bias_s * c;
        let next_tau = pseudorange_m / c;
        if (next_tau - tau).abs() < 1e-12 {
            break;
        }
        tau = next_tau;
    }
    pseudorange_m
}

fn fixture_root() -> std::path::PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/data/navigation")
}

fn load_fixture_paths() -> Vec<std::path::PathBuf> {
    let mut paths = fs::read_dir(fixture_root())
        .expect("read fixture directory")
        .filter_map(|entry| entry.ok().map(|item| item.path()))
        .filter(|path| path.extension().is_some_and(|ext| ext == "json"))
        .filter(|path| {
            path.file_stem()
                .and_then(|stem| stem.to_str())
                .is_some_and(|stem| stem.starts_with("pvt_nominal_"))
        })
        .collect::<Vec<_>>();
    paths.sort();
    paths
}

fn load_fixture(path: &Path) -> NavFixture {
    let raw = fs::read_to_string(path).expect("read fixture file");
    serde_json::from_str(&raw).expect("parse fixture")
}
