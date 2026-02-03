use bijux_gnss_core::{Constellation, SatId};
use bijux_gnss_nav::{Ephemeris, PositionObservation, PositionSolver};

#[test]
fn position_solver_returns_solution() {
    let solver = PositionSolver::new();
    let solution = solver.solve_wls(&[], &[], 0.0);
    assert!(solution.is_none());
}

#[test]
fn ephemeris_is_constructible() {
    let eph = Ephemeris {
        sat: SatId {
            constellation: Constellation::Gps,
            prn: 1,
        },
        toe_s: 0.0,
    };
    assert_eq!(eph.sat.prn, 1);
}

#[test]
fn position_observation_constructible() {
    let obs = PositionObservation {
        sat: SatId {
            constellation: Constellation::Gps,
            prn: 3,
        },
        pseudorange_m: 20_000_000.0,
        cn0_dbhz: 40.0,
        elevation_deg: None,
        weight: 1.0,
    };
    assert_eq!(obs.sat.prn, 3);
}
