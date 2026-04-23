#![allow(missing_docs)]

use bijux_gnss_core::api::{Constellation, SatId, SigId, SignalBand, SignalCode};
use bijux_gnss_nav::api::{Ekf, EkfConfig, Matrix, PseudorangeMeasurement};

#[test]
fn ekf_checkpoint_roundtrip() {
    let config = EkfConfig {
        gating_chi2_code: None,
        gating_chi2_phase: None,
        gating_chi2_doppler: None,
        huber_k: None,
        square_root: false,
        covariance_epsilon: 1e-9,
        divergence_max_variance: 1e9,
    };
    let ekf = Ekf::new(vec![1.0, 2.0], Matrix::identity(2), config.clone());
    let checkpoint = ekf.checkpoint();
    let restored = Ekf::restore(checkpoint, config);
    assert_eq!(restored.x.len(), 2);
    assert_eq!(restored.p.rows(), 2);
    assert_eq!(restored.p.cols(), 2);
}

#[test]
fn ekf_checkpoint_resume_matches() {
    let config = EkfConfig {
        gating_chi2_code: Some(1e9),
        gating_chi2_phase: Some(1e9),
        gating_chi2_doppler: Some(1e9),
        huber_k: None,
        square_root: true,
        covariance_epsilon: 1e-9,
        divergence_max_variance: 1e12,
    };
    let mut ekf_full = Ekf::new(vec![0.0; 8], Matrix::identity(8), config.clone());

    let meas = PseudorangeMeasurement {
        sig: SigId {
            sat: SatId { constellation: Constellation::Gps, prn: 5 },
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

    for _ in 0..10 {
        assert!(ekf_full.update(&meas));
    }
    let checkpoint = ekf_full.checkpoint();
    let mut ekf_resume = Ekf::restore(checkpoint, config.clone());
    for _ in 0..10 {
        assert!(ekf_full.update(&meas));
        assert!(ekf_resume.update(&meas));
    }
    assert_eq!(ekf_full.x.len(), ekf_resume.x.len());
    for (a, b) in ekf_full.x.iter().zip(ekf_resume.x.iter()) {
        assert!((a - b).abs() < 1e-6);
    }
}
