use super::*;

#[test]
fn validation_report_surfaces_carrier_smoothed_code_summary_without_raw_residuals() {
    let report = build_validation_report(
        &[],
        &[
            dual_frequency_epoch(43, vec![carrier_smoothed_code_satellite(1, false)]),
            dual_frequency_epoch(44, vec![carrier_smoothed_code_satellite(6, false)]),
            dual_frequency_epoch(45, vec![carrier_smoothed_code_satellite(1, true)]),
        ],
        &[fixture_solution(43, 1.0, 0.5, 4)],
        &[],
        1.0,
        false,
        Vec::new(),
        ValidationSciencePolicy::default(),
    )
    .expect("validation report");

    assert_eq!(report.carrier_smoothed_code.observations, 3);
    assert_eq!(report.carrier_smoothed_code.accepted_observations, 3);
    assert_eq!(report.carrier_smoothed_code.smoothed_observations, 3);
    assert_eq!(report.carrier_smoothed_code.cycle_slip_observations, 1);
    assert_eq!(report.carrier_smoothed_code.slip_reset_observations, 1);
    assert_eq!(report.carrier_smoothed_code.hidden_slip_observations, 0);
    assert_eq!(report.carrier_smoothed_code.improvement_verified, None);
    assert_eq!(report.carrier_smoothed_code.slip_visibility_verified, Some(true));
}

#[test]
fn validation_report_summarizes_geometry_free_dynamics() {
    let report = build_validation_report(
        &[],
        &[
            dual_frequency_epoch(
                50,
                vec![
                    dual_frequency_satellite_with_phase(
                        SignalBand::L1,
                        SignalCode::Ca,
                        22_000_000.0,
                    ),
                    dual_frequency_satellite_with_phase(
                        SignalBand::L2,
                        SignalCode::Py,
                        21_999_999.50,
                    ),
                ],
            ),
            dual_frequency_epoch(
                51,
                vec![
                    dual_frequency_satellite_with_phase(
                        SignalBand::L1,
                        SignalCode::Ca,
                        22_000_000.0,
                    ),
                    dual_frequency_satellite_with_phase(
                        SignalBand::L2,
                        SignalCode::Py,
                        21_999_999.46,
                    ),
                ],
            ),
            dual_frequency_epoch(
                52,
                vec![
                    dual_frequency_satellite_with_phase(
                        SignalBand::L1,
                        SignalCode::Ca,
                        22_000_000.0,
                    ),
                    dual_frequency_satellite_with_phase(
                        SignalBand::L2,
                        SignalCode::Py,
                        21_999_999.20,
                    ),
                ],
            ),
        ],
        &[
            fixture_solution(50, 1.0, 0.5, 4),
            fixture_solution(51, 1.0, 0.5, 4),
            fixture_solution(52, 1.0, 0.5, 4),
        ],
        &[],
        1.0,
        true,
        Vec::new(),
        ValidationSciencePolicy::default(),
    )
    .expect("validation report");

    assert_eq!(report.geometry_free.observations, 3);
    assert_eq!(report.geometry_free.complete_pairs, 3);
    assert_eq!(report.geometry_free.unavailable, 0);
    assert_eq!(report.geometry_free.insufficient_history, 1);
    assert_eq!(report.geometry_free.ionosphere_drift, 1);
    assert_eq!(report.geometry_free.cycle_slip_suspects, 1);
    assert!(report.geometry_free.max_abs_delta_m.expect("max delta") > 0.2);
}

#[test]
fn validation_report_summarizes_melbourne_wubbena_dynamics() {
    let report = build_validation_report(
        &[],
        &[
            dual_frequency_epoch(
                60,
                vec![
                    dual_frequency_satellite_with_phase(
                        SignalBand::L1,
                        SignalCode::Ca,
                        21_999_999.0,
                    ),
                    dual_frequency_satellite_with_phase(
                        SignalBand::L2,
                        SignalCode::Py,
                        22_000_000.5,
                    ),
                ],
            ),
            dual_frequency_epoch(
                61,
                vec![
                    dual_frequency_satellite_with_phase(
                        SignalBand::L1,
                        SignalCode::Ca,
                        21_999_999.01,
                    ),
                    dual_frequency_satellite_with_phase(
                        SignalBand::L2,
                        SignalCode::Py,
                        22_000_000.49,
                    ),
                ],
            ),
            dual_frequency_epoch(
                62,
                vec![
                    dual_frequency_satellite_with_phase(
                        SignalBand::L1,
                        SignalCode::Ca,
                        22_000_005.5,
                    ),
                    dual_frequency_satellite_with_phase(
                        SignalBand::L2,
                        SignalCode::Py,
                        22_000_000.5,
                    ),
                ],
            ),
        ],
        &[
            fixture_solution(60, 1.0, 0.5, 4),
            fixture_solution(61, 1.0, 0.5, 4),
            fixture_solution(62, 1.0, 0.5, 4),
        ],
        &[],
        1.0,
        true,
        Vec::new(),
        ValidationSciencePolicy::default(),
    )
    .expect("validation report");

    assert_eq!(report.melbourne_wubbena.observations, 3);
    assert_eq!(report.melbourne_wubbena.complete_pairs, 3);
    assert_eq!(report.melbourne_wubbena.unavailable, 0);
    assert_eq!(report.melbourne_wubbena.insufficient_history, 1);
    assert_eq!(report.melbourne_wubbena.nominal, 1);
    assert_eq!(report.melbourne_wubbena.wide_lane_slip_suspects, 1);
    assert!(
        report.melbourne_wubbena.max_abs_delta_wide_lane_cycles.expect("max wide-lane delta")
            >= 0.5
    );
}
