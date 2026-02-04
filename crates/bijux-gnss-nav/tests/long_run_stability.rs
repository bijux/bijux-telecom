#![allow(missing_docs)]

use bijux_gnss_core::{Constellation, SatId, SigId, SignalBand, SignalCode};
use bijux_gnss_nav::{Ekf, EkfConfig, Matrix, PseudorangeMeasurement};

#[test]
fn ekf_long_run_stability() {
    let config = EkfConfig {
        gating_chi2_code: Some(1e9),
        gating_chi2_phase: Some(1e9),
        gating_chi2_doppler: Some(1e9),
        huber_k: None,
        square_root: true,
        covariance_epsilon: 1e-9,
        divergence_max_variance: 1e12,
    };
    let mut ekf = Ekf::new(vec![0.0; 8], Matrix::identity(8), config);
    let meas = PseudorangeMeasurement {
        sig: SigId {
            sat: SatId {
                constellation: Constellation::Gps,
                prn: 10,
            },
            band: SignalBand::L1,
            code: SignalCode::Ca,
        },
        z_m: 20_200_000.0,
        sat_pos_m: [15_000_000.0, 5_000_000.0, 21_000_000.0],
        sat_clock_s: 0.0,
        tropo_m: 0.0,
        iono_m: 0.0,
        sigma_m: 10.0,
        elevation_deg: None,
        ztd_index: None,
        isb_index: None,
    };

    for _ in 0..7200 {
        assert!(ekf.update(&meas));
        for value in &ekf.x {
            assert!(value.is_finite());
        }
        assert_eq!(ekf.x.len(), 8);
    }
}
