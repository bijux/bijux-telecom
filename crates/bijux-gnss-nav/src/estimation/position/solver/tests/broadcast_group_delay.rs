use super::{PositionBroadcastNavigation, PositionObservation, PositionSolver};
use crate::corrections::broadcast_group_delay::gps_broadcast_group_delay_code_bias_m;
use crate::estimation::position::navigation::corrected_pseudorange_m;
use crate::orbits::gps::GpsEphemeris;
use bijux_gnss_core::api::{Constellation, SatId, SigId, SignalBand, SignalCode};

fn sample_gps_ephemeris_with_tgd(prn: u8, tgd_s: f64) -> GpsEphemeris {
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
        tgd: tgd_s,
    }
}

#[test]
fn position_solver_applies_broadcast_ionosphere_by_default() {
    assert!(PositionSolver::new().apply_broadcast_ionosphere);
}

#[test]
fn position_solver_can_disable_broadcast_ionosphere() {
    assert!(!PositionSolver::new().with_broadcast_ionosphere(false).apply_broadcast_ionosphere);
}

#[test]
fn position_solver_applies_broadcast_group_delay_by_default() {
    assert!(PositionSolver::new().apply_broadcast_group_delay);
}

#[test]
fn position_solver_can_disable_broadcast_group_delay() {
    assert!(!PositionSolver::new().with_broadcast_group_delay(false).apply_broadcast_group_delay);
}

#[test]
fn corrected_pseudorange_respects_broadcast_group_delay_toggle() {
    let ephemeris = sample_gps_ephemeris_with_tgd(7, -8.0e-9);
    let navigation = PositionBroadcastNavigation::Gps(ephemeris.clone());
    let sat = SatId { constellation: Constellation::Gps, prn: 7 };
    let signal_id = SigId { sat, band: SignalBand::L5, code: SignalCode::L5Q };
    let observation = PositionObservation {
        sat,
        pseudorange_m: 24_000_000.0,
        doppler_hz: None,
        doppler_var_hz2: None,
        cn0_dbhz: 45.0,
        elevation_deg: None,
        weight: 1.0,
        gps_receive_time: None,
        signal_timing: None,
        signal_id: Some(signal_id),
    };
    let expected_bias_m = gps_broadcast_group_delay_code_bias_m(signal_id, &ephemeris)
        .expect("GPS L5 broadcast group delay bias");

    let corrected = corrected_pseudorange_m(&observation, &navigation, true);
    let uncorrected = corrected_pseudorange_m(&observation, &navigation, false);

    assert_eq!(uncorrected, observation.pseudorange_m);
    assert!((corrected - (observation.pseudorange_m - expected_bias_m)).abs() < 1.0e-12);
}
