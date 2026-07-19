use super::*;

#[test]
fn validation_report_surfaces_innovation_consistency_anomalies() {
    let mut solution = fixture_solution(9, 1.0, 0.5, 4);
    solution.health.push(bijux_gnss_core::api::NavHealthEvent::InnovationConsistencyAnomaly {
        normalized_innovation_squared: 8.0,
        lower_bound: 0.1,
        upper_bound: 6.6,
        measurement_dimension: 1,
    });
    let reference = ValidationReferenceEpoch {
        epoch_idx: 9,
        t_rx_s: Some(9.0),
        latitude_deg: 0.0,
        longitude_deg: 0.0,
        altitude_m: 0.0,
        ecef_x_m: Some(0.0),
        ecef_y_m: Some(0.0),
        ecef_z_m: Some(0.0),
        vel_x_mps: None,
        vel_y_mps: None,
        vel_z_mps: None,
    };

    let report = build_validation_report(
        &[],
        &[],
        &[solution],
        &[reference],
        1.0,
        false,
        Vec::new(),
        ValidationSciencePolicy::default(),
    )
    .expect("validation report");

    assert!(report.consistency_warnings.iter().any(|warning| {
        warning.contains("innovation consistency anomalies") && warning.contains("peak NIS 8.000")
    }));
}
