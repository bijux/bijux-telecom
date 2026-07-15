#![allow(missing_docs)]

use crate::estimation::ekf::models::PseudorangeMeasurement;
use crate::estimation::ekf::state::{Ekf, EkfConfig};
use crate::estimation::ekf::statistics::InnovationConsistencyConfig;
use crate::linalg::Matrix;
use bijux_gnss_core::api::{Constellation, SatId, SigId, SignalBand, SignalCode};

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
            innovation_consistency: Some(InnovationConsistencyConfig::default()),
            huber_k: Some(10.0),
            square_root: true,
            covariance_epsilon: 1e-6,
            divergence_max_variance: 1e12,
        },
    );
    let meas = PseudorangeMeasurement {
        sig: SigId {
            sat: SatId { constellation: Constellation::Gps, prn: 1 },
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

#[test]
fn ekf_reset_epoch_health_clears_ephemeral_metrics() {
    let mut ekf = Ekf::new(
        vec![0.0; 8],
        Matrix::identity(8),
        EkfConfig {
            gating_chi2_code: Some(100.0),
            gating_chi2_phase: Some(100.0),
            gating_chi2_doppler: Some(100.0),
            innovation_consistency: Some(InnovationConsistencyConfig::default()),
            huber_k: Some(10.0),
            square_root: true,
            covariance_epsilon: 1e-6,
            divergence_max_variance: 1e12,
        },
    );
    ekf.health.innovation_rms = 25.0;
    ekf.health.peak_innovation_rms = 40.0;
    ekf.health.normalized_innovation_squared = Some(6.0);
    ekf.health.peak_normalized_innovation_squared = Some(10.0);
    ekf.health.condition_number = Some(3.0);
    ekf.health.peak_condition_number = Some(12.0);
    ekf.health.whiteness_ratio = Some(4.0);
    ekf.health.peak_whiteness_ratio = Some(9.0);
    ekf.health.predicted_variance = Some(16.0);
    ekf.health.observed_variance = Some(64.0);
    ekf.health.innovation_consistency_lower_bound = Some(0.1);
    ekf.health.innovation_consistency_upper_bound = Some(6.6);
    ekf.health.events.push(bijux_gnss_core::api::NavHealthEvent::CovarianceSymmetrized);

    ekf.reset_epoch_health();

    assert_eq!(ekf.health.innovation_rms, 0.0);
    assert_eq!(ekf.health.peak_innovation_rms, 0.0);
    assert_eq!(ekf.health.normalized_innovation_squared, None);
    assert_eq!(ekf.health.peak_normalized_innovation_squared, None);
    assert_eq!(ekf.health.condition_number, None);
    assert_eq!(ekf.health.peak_condition_number, None);
    assert_eq!(ekf.health.whiteness_ratio, None);
    assert_eq!(ekf.health.peak_whiteness_ratio, None);
    assert_eq!(ekf.health.predicted_variance, None);
    assert_eq!(ekf.health.observed_variance, None);
    assert_eq!(ekf.health.innovation_consistency_lower_bound, None);
    assert_eq!(ekf.health.innovation_consistency_upper_bound, None);
    assert!(ekf.health.events.is_empty());
}

#[test]
fn ekf_retain_states_preserves_selected_covariance_rows() {
    let mut ekf = Ekf::new(
        vec![10.0, 20.0, 30.0, 40.0],
        Matrix::new(4, 4, 0.0),
        EkfConfig {
            gating_chi2_code: None,
            gating_chi2_phase: None,
            gating_chi2_doppler: None,
            innovation_consistency: Some(InnovationConsistencyConfig::default()),
            huber_k: None,
            square_root: false,
            covariance_epsilon: 1e-9,
            divergence_max_variance: 1e12,
        },
    );
    ekf.labels = vec!["pos_x".into(), "clock".into(), "iono_g07".into(), "amb_g11".into()];
    for row in 0..4 {
        for col in 0..4 {
            ekf.p[(row, col)] = (row * 10 + col) as f64 + 1.0;
        }
    }

    assert!(ekf.retain_states(&[0, 2, 3]));

    assert_eq!(ekf.x, vec![10.0, 30.0, 40.0]);
    assert_eq!(ekf.labels, vec!["pos_x", "iono_g07", "amb_g11"]);
    assert_eq!(ekf.p.rows(), 3);
    assert_eq!(ekf.p.cols(), 3);
    assert_eq!(ekf.p[(0, 0)], 1.0);
    assert_eq!(ekf.p[(0, 1)], 0.5 * (3.0 + 21.0));
    assert_eq!(ekf.p[(1, 2)], 0.5 * (24.0 + 33.0));
    assert_eq!(ekf.p[(2, 2)], 34.0);
}

#[test]
fn ekf_retain_states_rejects_duplicate_or_out_of_range_indices() {
    let mut ekf = Ekf::new(
        vec![1.0, 2.0],
        Matrix::identity(2),
        EkfConfig {
            gating_chi2_code: None,
            gating_chi2_phase: None,
            gating_chi2_doppler: None,
            innovation_consistency: Some(InnovationConsistencyConfig::default()),
            huber_k: None,
            square_root: false,
            covariance_epsilon: 1e-9,
            divergence_max_variance: 1e12,
        },
    );

    assert!(!ekf.retain_states(&[0, 0]));
    assert!(!ekf.retain_states(&[0, 2]));
    assert_eq!(ekf.x, vec![1.0, 2.0]);
    assert_eq!(ekf.p.rows(), 2);
}

#[test]
fn ekf_records_innovation_consistency_anomaly_when_nis_exceeds_bounds() {
    let mut covariance = Matrix::identity(8);
    for index in 0..8 {
        covariance[(index, index)] = 1.0e-6;
    }
    let mut ekf = Ekf::new(
        vec![0.0; 8],
        covariance,
        EkfConfig {
            gating_chi2_code: None,
            gating_chi2_phase: None,
            gating_chi2_doppler: None,
            innovation_consistency: Some(InnovationConsistencyConfig::default()),
            huber_k: None,
            square_root: true,
            covariance_epsilon: 1e-6,
            divergence_max_variance: 1e12,
        },
    );
    let meas = PseudorangeMeasurement {
        sig: SigId {
            sat: SatId { constellation: Constellation::Gps, prn: 1 },
            band: SignalBand::L1,
            code: SignalCode::Ca,
        },
        z_m: 30_200_000.0,
        sat_pos_m: [15_000_000.0, 0.0, 21_000_000.0],
        sat_clock_s: 0.0,
        tropo_m: 0.0,
        iono_m: 0.0,
        sigma_m: 1.0,
        elevation_deg: None,
        ztd_index: None,
        isb_index: None,
    };

    assert!(ekf.update(&meas));
    assert!(ekf.health.normalized_innovation_squared.is_some());
    assert!(ekf.health.innovation_consistency_upper_bound.is_some());
    assert!(
        ekf.health.normalized_innovation_squared.unwrap()
            > ekf.health.innovation_consistency_upper_bound.unwrap()
    );
    assert!(ekf.health.events.iter().any(|event| matches!(
        event,
        bijux_gnss_core::api::NavHealthEvent::InnovationConsistencyAnomaly { .. }
    )));
}
