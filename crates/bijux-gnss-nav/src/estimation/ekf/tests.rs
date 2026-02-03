use crate::{Ekf, EkfConfig, Matrix, PseudorangeMeasurement};
use bijux_gnss_core::{Constellation, SatId, SigId, SignalBand, SignalCode};

#[test]
fn matrix_inversion_identity() {
    let m = Matrix::identity(3);
    let inv = m.invert().expect("invert");
    for i in 0..3 {
        for j in 0..3 {
            let expected = if i == j { 1.0 } else { 0.0 };
            assert!((inv[(i, j)] - expected).abs() < 1e-9);
        }
    }
}

#[test]
fn ekf_update_runs() {
    let x = vec![0.0; 8];
    let p = Matrix::identity(8);
    let mut ekf = Ekf::new(
        x,
        p,
        EkfConfig {
            gating_chi2_code: Some(100.0),
            gating_chi2_phase: Some(100.0),
            gating_chi2_doppler: Some(100.0),
            huber_k: Some(10.0),
            square_root: true,
            covariance_epsilon: 1e-6,
            divergence_max_variance: 1e12,
        },
    );
    let meas = PseudorangeMeasurement {
        sig: SigId {
            sat: SatId {
                constellation: Constellation::Gps,
                prn: 1,
            },
            band: SignalBand::L1,
            code: SignalCode::Ca,
        },
        z_m: 20_200_000.0,
        sat_pos_m: [15_000_000.0, 0.0, 21_000_000.0],
        sat_clock_s: 0.0,
        tropo_m: 0.0,
        iono_m: 0.0,
        sigma_m: 10.0,
        elevation_deg: None,
        ztd_index: None,
        isb_index: None,
    };
    assert!(ekf.update(&meas));
}
