#[test]
fn synthetic_signal_source_matches_materialized_generator() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 1_023_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let scenario = SyntheticScenario {
        sample_rate_hz: config.sampling_freq_hz,
        intermediate_freq_hz: config.intermediate_freq_hz,
        receiver_clock_frequency_bias_hz: 0.0,
        duration_s: 0.004,
        seed: 29,
        satellites: vec![SyntheticSignalParams {
            sat: SatId { constellation: Constellation::Gps, prn: 7 },
            glonass_frequency_channel: None,
            signal_band: bijux_gnss_core::api::SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Unknown,
            doppler_hz: 750.0,
            code_phase_chips: 15.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 47.0,
            navigation_data: false.into(),
        }],
        ephemerides: Vec::new(),
        id: "synthetic-stream".to_string(),
    };

    let expected = generate_l1_ca_multi(&config, &scenario);
    let mut source = SyntheticSignalSource::new(&config, &scenario);
    let streamed = collect_frames(&mut source, 2 * 1_023);

    assert_eq!(expected.len(), streamed.len());
    assert_eq!(expected.t0, streamed.t0);
    assert_eq!(expected.dt_s, streamed.dt_s);
    assert_eq!(expected.iq, streamed.iq);
    assert!(source.is_done());
}

#[test]
fn synthetic_signal_source_matches_materialized_generator_with_source_front_end_filter() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 1_023_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let scenario = SyntheticScenario {
        sample_rate_hz: config.sampling_freq_hz,
        intermediate_freq_hz: config.intermediate_freq_hz,
        receiver_clock_frequency_bias_hz: 0.0,
        duration_s: 0.004,
        seed: 31,
        satellites: vec![SyntheticSignalParams {
            sat: SatId { constellation: Constellation::Gps, prn: 11 },
            glonass_frequency_channel: None,
            signal_band: bijux_gnss_core::api::SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Ca,
            doppler_hz: 250.0,
            code_phase_chips: 63.0,
            carrier_phase_rad: 0.2,
            cn0_db_hz: 49.0,
            navigation_data: false.into(),
        }],
        ephemerides: Vec::new(),
        id: "synthetic-source-front-end-stream".to_string(),
    };
    let source_front_end_filter =
        bijux_gnss_signal::api::FrontEndFilterSpec::LowPass { cutoff_hz: 400_000.0, taps: 11 };

    let expected = super::generate_l1_ca_multi_with_source_front_end(
        &config,
        &scenario,
        Some(&source_front_end_filter),
    );
    let mut source = SyntheticSignalSource::new_with_signal_delay_alignments_and_source_front_end(
        &config,
        &scenario,
        Vec::new(),
        Some(&source_front_end_filter),
    );
    let streamed = collect_frames(&mut source, 2 * 1_023);

    assert_eq!(expected.len(), streamed.len());
    assert_eq!(expected.t0, streamed.t0);
    assert_eq!(expected.dt_s, streamed.dt_s);
    assert_eq!(expected.iq, streamed.iq);
    assert!(source.is_done());
}

#[test]
fn synthetic_signal_source_matches_materialized_generator_with_receiver_oscillator() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 1_023_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let scenario = SyntheticScenario {
        sample_rate_hz: config.sampling_freq_hz,
        intermediate_freq_hz: config.intermediate_freq_hz,
        receiver_clock_frequency_bias_hz: 0.0,
        duration_s: 0.004,
        seed: 37,
        satellites: vec![SyntheticSignalParams {
            sat: SatId { constellation: Constellation::Gps, prn: 13 },
            glonass_frequency_channel: None,
            signal_band: bijux_gnss_core::api::SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Ca,
            doppler_hz: 450.0,
            code_phase_chips: 27.0,
            carrier_phase_rad: 0.25,
            cn0_db_hz: 49.0,
            navigation_data: false.into(),
        }],
        ephemerides: Vec::new(),
        id: "synthetic-oscillator-stream".to_string(),
    };
    let receiver_oscillator = SyntheticReceiverOscillatorModel {
        carrier_frequency_bias_hz: 120.0,
        carrier_frequency_drift_hz_per_s: 30.0,
        sampling_clock_fractional_error: 80.0e-6,
        sampling_clock_fractional_drift_per_s: 0.0,
        phase_noise: SyntheticReceiverPhaseNoiseModel {
            seed: 53,
            knot_interval_samples: 1_023,
            step_std_rad: 0.03,
        },
        noise: SyntheticReceiverOscillatorNoiseModel::default(),
    };

    let expected =
        generate_l1_ca_multi_with_receiver_oscillator(&config, &scenario, &receiver_oscillator);
    let mut source = SyntheticSignalSource::new_with_receiver_oscillator(
        &config,
        &scenario,
        &receiver_oscillator,
    );
    let streamed = collect_frames(&mut source, 2 * 1_023);

    assert_eq!(expected.len(), streamed.len());
    assert_eq!(expected.t0, streamed.t0);
    assert_eq!(expected.dt_s, streamed.dt_s);
    assert_eq!(expected.iq, streamed.iq);
    assert!(source.is_done());
}

#[test]
fn iq_truth_bundle_records_source_front_end_filter_metadata() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let scenario = SyntheticScenario {
        sample_rate_hz: config.sampling_freq_hz,
        intermediate_freq_hz: config.intermediate_freq_hz,
        receiver_clock_frequency_bias_hz: 0.0,
        duration_s: 0.004,
        seed: 41,
        satellites: vec![SyntheticSignalParams {
            sat: SatId { constellation: Constellation::Gps, prn: 5 },
            glonass_frequency_channel: None,
            signal_band: bijux_gnss_core::api::SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Ca,
            doppler_hz: -500.0,
            code_phase_chips: 140.25,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 50.0,
            navigation_data: false.into(),
        }],
        ephemerides: Vec::new(),
        id: "synthetic-source-front-end-truth".to_string(),
    };
    let source_front_end_filter =
        bijux_gnss_signal::api::FrontEndFilterSpec::LowPass { cutoff_hz: 450_000.0, taps: 17 };
    let frame = super::generate_l1_ca_multi_with_source_front_end(
        &config,
        &scenario,
        Some(&source_front_end_filter),
    );
    let bundle = super::build_iq16_capture_bundle_with_source_front_end(
        &scenario.id,
        &scenario,
        &frame,
        "2026-07-13T00:00:00Z",
        Some("source front-end truth".to_string()),
        Some(&source_front_end_filter),
    );

    assert_eq!(bundle.truth.source_front_end_filter, Some(source_front_end_filter.clone()));
    assert_eq!(
        bundle.truth.source_front_end_sample_delay_samples,
        source_front_end_filter.group_delay_samples() as u64
    );
}

#[test]
fn truth_guided_acquisition_accounts_for_source_front_end_delay() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        acquisition_doppler_search_hz: 2_000,
        acquisition_doppler_step_hz: 250,
        acquisition_integration_ms: 1,
        acquisition_noncoherent: 1,
        ..ReceiverPipelineConfig::default()
    };
    let scenario = SyntheticScenario {
        sample_rate_hz: config.sampling_freq_hz,
        intermediate_freq_hz: config.intermediate_freq_hz,
        receiver_clock_frequency_bias_hz: 0.0,
        duration_s: 0.004,
        seed: 53,
        satellites: vec![SyntheticSignalParams {
            sat: SatId { constellation: Constellation::Gps, prn: 9 },
            glonass_frequency_channel: None,
            signal_band: bijux_gnss_core::api::SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Ca,
            doppler_hz: 750.0,
            code_phase_chips: 211.5,
            carrier_phase_rad: 0.1,
            cn0_db_hz: 55.0,
            navigation_data: false.into(),
        }],
        ephemerides: Vec::new(),
        id: "synthetic-source-front-end-acquisition".to_string(),
    };
    let source_front_end_filter =
        bijux_gnss_signal::api::FrontEndFilterSpec::LowPass { cutoff_hz: 450_000.0, taps: 13 };
    let frame = super::generate_l1_ca_multi_with_source_front_end(
        &config,
        &scenario,
        Some(&source_front_end_filter),
    );
    let bundle = super::build_iq16_capture_bundle_with_source_front_end(
        &scenario.id,
        &scenario,
        &frame,
        "2026-07-13T00:00:00Z",
        Some("source front-end acquisition".to_string()),
        Some(&source_front_end_filter),
    );
    let scaled_frame = SamplesFrame::new(
        frame.t0,
        frame.dt_s,
        frame.iq.iter().map(|sample| *sample * bundle.truth.output_scale_applied).collect(),
    );
    let report =
        super::validate_truth_guided_acquisition_table(&config, &scaled_frame, &bundle.truth, 1, 3);
    let row = report.satellites.first().expect("synthetic acquisition truth row");
    let unfiltered_expected = super::expected_acquisition_code_phase_samples_f64(
        &config,
        &scaled_frame,
        scenario.satellites[0].code_phase_chips,
    );
    let delayed_expected =
        unfiltered_expected + source_front_end_filter.group_delay_samples() as f64;

    assert!(report.pass, "{report:#?}");
    assert!((row.expected_code_phase_samples as f64 - delayed_expected).abs() <= 0.5, "{row:#?}");
    assert_eq!(
        bundle.truth.source_front_end_sample_delay_samples,
        source_front_end_filter.group_delay_samples() as u64
    );
}

#[test]
fn synthetic_signal_source_preserves_broadcast_ephemerides() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 1_023_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let ephemerides = vec![GpsEphemeris {
        sat: SatId { constellation: Constellation::Gps, prn: 7 },
        iodc: 0,
        iode: 0,
        week: 0,
        sv_health: 0,
        toe_s: 100_000.0,
        toc_s: 100_000.0,
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
    }];
    let scenario = SyntheticScenario {
        sample_rate_hz: config.sampling_freq_hz,
        intermediate_freq_hz: config.intermediate_freq_hz,
        receiver_clock_frequency_bias_hz: 0.0,
        duration_s: 0.004,
        seed: 17,
        satellites: vec![SyntheticSignalParams {
            sat: SatId { constellation: Constellation::Gps, prn: 7 },
            glonass_frequency_channel: None,
            signal_band: bijux_gnss_core::api::SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Unknown,
            doppler_hz: 500.0,
            code_phase_chips: 0.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 47.0,
            navigation_data: false.into(),
        }],
        ephemerides: ephemerides.clone(),
        id: "synthetic-navigation".to_string(),
    };

    let source = SyntheticSignalSource::new(&config, &scenario);

    assert_eq!(source.gps_ephemerides().len(), ephemerides.len());
    assert_eq!(source.gps_ephemerides()[0].sat, ephemerides[0].sat);
    assert_eq!(source.gps_ephemerides()[0].week, ephemerides[0].week);
    assert_eq!(source.gps_ephemerides()[0].toe_s, ephemerides[0].toe_s);
    assert!(source.as_any().downcast_ref::<SyntheticSignalSource>().is_some());
}

#[test]
fn pvt_truth_table_records_truth_measured_values_and_errors() {
    let truth_ecef = lla_to_ecef(37.0, -122.0, 10.0);
    let measured_ecef = (truth_ecef.0 + 1.5, truth_ecef.1 - 2.0, truth_ecef.2 + 0.75);
    let measured_geodetic = ecef_to_geodetic(measured_ecef.0, measured_ecef.1, measured_ecef.2);
    let truth_clock_bias_s = 2.0e-4;
    let measured_clock_bias_s = 2.5e-4;
    let solution = NavSolutionEpoch {
        epoch: Epoch { index: 7 },
        t_rx_s: Seconds(100_000.0),
        source_time: ReceiverSampleTrace::from_sample_index(0, 1.0),
        ecef_x_m: Meters(measured_ecef.0),
        ecef_y_m: Meters(measured_ecef.1),
        ecef_z_m: Meters(measured_ecef.2),
        position_covariance_ecef_m2: None,
        latitude_deg: measured_geodetic.0,
        longitude_deg: measured_geodetic.1,
        altitude_m: Meters(measured_geodetic.2),
        clock_bias_s: Seconds(measured_clock_bias_s),
        clock_bias_m: Meters(measured_clock_bias_s * SPEED_OF_LIGHT_MPS),
        clock_drift_s_per_s: 0.0,
        pdop: 1.2,
        pre_fit_residual_rms_m: Some(Meters(3.5)),
        post_fit_residual_rms_m: Some(Meters(1.25)),
        rms_m: Meters(1.25),
        status: SolutionStatus::CodeOnly,
        quality: NavQualityFlag::Float,
        validity: SolutionValidity::Stable,
        valid: true,
        processing_ms: None,
        residuals: Vec::new(),
        constellation_residual_rms: Vec::new(),
        health: Vec::new(),
        isb: Vec::new(),
        sigma_e_m: None,
        sigma_n_m: None,
        sigma_u_m: None,
        horizontal_error_ellipse_major_axis_m: None,
        horizontal_error_ellipse_minor_axis_m: None,
        horizontal_error_ellipse_azimuth_deg: None,
        sigma_h_m: None,
        sigma_v_m: None,
        innovation_rms_m: None,
        normalized_innovation_rms: None,
        normalized_innovation_max: None,
        ekf_innovation_rms: None,
        ekf_condition_number: None,
        ekf_whiteness_ratio: None,
        ekf_predicted_variance: None,
        ekf_observed_variance: None,
        integrity_hpl_m: None,
        integrity_vpl_m: None,
        model_version: NAV_SOLUTION_MODEL_VERSION,
        lifecycle_state: NavLifecycleState::CodeOnly,
        uncertainty_class: NavUncertaintyClass::Low,
        assumptions: None,
        refusal_class: None,
        artifact_id: "nav-epoch-0000000007-pvt-truth".to_string(),
        source_observation_epoch_id: "obs-epoch-0000000007-pvt-truth".to_string(),
        explain_decision: "accepted".to_string(),
        explain_reasons: vec!["navigation_solution_usable".to_string()],
        provenance: None,
        sat_count: 5,
        used_sat_count: 4,
        rejected_sat_count: 1,
        hdop: Some(0.9),
        vdop: Some(0.8),
        gdop: Some(1.3),
        tdop: Some(0.4),
        stability_signature: "navsig:v2:pvt-truth".to_string(),
        stability_signature_version: NAV_OUTPUT_STABILITY_SIGNATURE_VERSION,
    };
    let reference = SyntheticPvtTruthReferenceEpoch {
        position: ValidationReferenceEpoch {
            epoch_idx: 7,
            t_rx_s: Some(100_000.0),
            latitude_deg: 37.0,
            longitude_deg: -122.0,
            altitude_m: 10.0,
            ecef_x_m: Some(truth_ecef.0),
            ecef_y_m: Some(truth_ecef.1),
            ecef_z_m: Some(truth_ecef.2),
            vel_x_mps: None,
            vel_y_mps: None,
            vel_z_mps: None,
        },
        clock_bias_s: truth_clock_bias_s,
    };

    let report = validate_truth_guided_pvt_table("unit_test_pvt_truth", &[solution], &[reference]);
    let row = report.epochs.first().expect("pvt truth row");

    assert_eq!(report.solution_count, 1);
    assert_eq!(report.matched_epoch_count, 1);
    assert!(report.unmatched_solution_epochs.is_empty());
    assert!(report.unused_reference_epochs.is_empty());
    assert_eq!(row.truth_ecef_m.x_m, truth_ecef.0);
    assert_eq!(row.measured_ecef_m.y_m, measured_ecef.1);
    assert_eq!(row.ecef_error_m.x_m, 1.5);
    assert_eq!(row.ecef_error_m.y_m, -2.0);
    assert_eq!(row.ecef_error_m.z_m, 0.75);
    assert_eq!(row.truth_geodetic.latitude_deg, 37.0);
    assert_eq!(row.truth_geodetic.longitude_deg, -122.0);
    assert_eq!(row.truth_geodetic.altitude_m, 10.0);
    assert_eq!(row.clock_bias.truth_s, truth_clock_bias_s);
    assert_eq!(row.clock_bias.measured_s, measured_clock_bias_s);
    assert_eq!(row.clock_bias.error_s, measured_clock_bias_s - truth_clock_bias_s);
    assert!(
        (row.clock_bias.error_m
            - (measured_clock_bias_s - truth_clock_bias_s) * SPEED_OF_LIGHT_MPS)
            .abs()
            <= 1.0e-9
    );
    assert_eq!(row.residual_rms_m, 1.25);
    assert_eq!(row.pre_fit_residual_rms_m, Some(3.5));
    assert_eq!(row.post_fit_residual_rms_m, Some(1.25));
    assert_eq!(row.dop.pdop, 1.2);
    assert_eq!(row.solution_status, SolutionStatus::CodeOnly);
    assert_eq!(row.solution_quality, NavQualityFlag::Float);
    assert_eq!(row.solution_validity, SolutionValidity::Stable);
    assert!(row.valid);
}

#[test]
fn truth_guided_receiver_accuracy_budgets_are_hard_and_positive() {
    let budgets = truth_guided_receiver_accuracy_budgets();

    assert!(budgets.acquisition.max_doppler_error_hz > 0.0);
    assert!(budgets.acquisition.max_code_phase_error_samples > 0);
    assert!(budgets.acquisition.max_doppler_error_bins > 0.0);
    assert!(budgets.acquisition.max_code_phase_error_chips > 0.0);
    assert!(budgets.tracking.max_carrier_error_hz > 0.0);
    assert!(budgets.tracking.max_doppler_error_hz > 0.0);
    assert!(budgets.tracking.max_code_phase_error_samples > 0.0);
    assert!(budgets.tracking.max_cn0_error_db_hz > 0.0);
    assert!(budgets.observation.max_pseudorange_error_m > 0.0);
    assert!(budgets.observation.max_carrier_phase_error_cycles > 0.0);
    assert!(budgets.observation.max_doppler_error_hz > 0.0);
    assert!(budgets.observation.max_cn0_error_db_hz > 0.0);
    assert!(budgets.pvt.max_position_error_3d_m > 0.0);
    assert!(budgets.pvt.max_clock_bias_error_m > 0.0);
    assert!(budgets.pvt.max_residual_rms_m > 0.0);
    assert!(budgets.pvt.max_pdop > 0.0);
}

#[test]
fn acquisition_accuracy_budget_fails_when_truth_error_exceeds_threshold() {
    let report = SyntheticAcquisitionTruthTableReport {
        scenario_id: "acquisition_budget_failure".to_string(),
        doppler_tolerance_bins: 1,
        doppler_tolerance_hz: 500.0,
        code_phase_tolerance_samples: 2,
        sample_rate_hz: 1_023_000.0,
        period_samples: 1023,
        doppler_step_hz: 500,
        pass: true,
        satellites: vec![SyntheticAcquisitionTruthTableSatellite {
            sat: SatId { constellation: Constellation::Gps, prn: 3 },
            glonass_frequency_channel: None,
            signal_band: SignalBand::L1,
            signal_code: SignalCode::Ca,
            injected_doppler_hz: 750.0,
            expected_measured_doppler_hz: 750.0,
            measured_doppler_hz: 1_500.0,
            doppler_error_hz: 750.0,
            doppler_error_bins: 1.5,
            injected_code_phase_chips: 200.25,
            expected_code_phase_samples: 100,
            measured_code_phase_samples: 104,
            code_phase_error_samples: 4,
            peak_mean_ratio: 10.0,
            hypothesis: "accepted".to_string(),
            doppler_pass: false,
            code_phase_pass: false,
            pass: false,
        }],
    };

    let accuracy = validate_acquisition_accuracy_budget(
        &report,
        truth_guided_receiver_accuracy_budgets().acquisition,
    );
    let satellite = accuracy.satellites.first().expect("acquisition satellite");

    assert!(!accuracy.pass);
    assert_eq!(accuracy.passing_satellite_count, 0);
    assert!(!satellite.pass);
    assert_eq!(satellite.signal_band, SignalBand::L1);
    assert_eq!(satellite.signal_code, SignalCode::Ca);
}

#[test]
fn acquisition_accuracy_budget_requires_truth_satellites() {
    let report = SyntheticAcquisitionTruthTableReport {
        scenario_id: "acquisition_missing_truth".to_string(),
        doppler_tolerance_bins: 1,
        doppler_tolerance_hz: 500.0,
        code_phase_tolerance_samples: 2,
        sample_rate_hz: 1_023_000.0,
        period_samples: 1023,
        doppler_step_hz: 500,
        pass: false,
        satellites: Vec::new(),
    };

    let accuracy = validate_acquisition_accuracy_budget(
        &report,
        truth_guided_receiver_accuracy_budgets().acquisition,
    );

    assert!(!accuracy.truth_coverage_ready);
    assert_eq!(accuracy.truth_coverage_issues.len(), 1);
    assert_eq!(accuracy.truth_coverage_issues[0].code, "no_truth_satellites");
    assert!(!accuracy.pass);
}

#[test]
fn tracking_accuracy_budget_requires_stable_truth_epochs() {
    let report = SyntheticTrackingTruthTableReport {
        scenario_id: "tracking_missing_truth".to_string(),
        carrier_tolerance_hz: 10.0,
        doppler_tolerance_hz: 10.0,
        code_phase_tolerance_samples: 1.0,
        cn0_tolerance_db_hz: 8.0,
        sample_rate_hz: 1_023_000.0,
        period_samples: 1023,
        output_scale_applied: 1.0,
        pass: false,
        satellites: vec![SyntheticTrackingTruthTableSatellite {
            sat: SatId { constellation: Constellation::Gps, prn: 7 },
            signal_band: SignalBand::L1,
            signal_code: SignalCode::Ca,
            glonass_frequency_channel: None,
            injected_doppler_hz: -250.0,
            expected_measured_doppler_hz: -250.0,
            injected_code_phase_chips: 100.0,
            injected_cn0_db_hz: 45.0,
            epoch_count: 1,
            stable_epoch_count: 0,
            first_stable_epoch_index: None,
            pass: false,
            epochs: vec![SyntheticTrackingTruthTableEpoch {
                epoch_index: 0,
                sample_index: 0,
                expected_carrier_hz: -250.0,
                measured_carrier_hz: -250.0,
                carrier_error_hz: 0.0,
                expected_doppler_hz: -250.0,
                measured_doppler_hz: -250.0,
                doppler_error_hz: 0.0,
                pll_phase_error_rad: 0.0,
                pll_phase_error_cycles: 0.0,
                expected_code_phase_samples: 100.0,
                measured_code_phase_samples: 100.0,
                code_phase_error_samples: 0.0,
                expected_cn0_db_hz: 45.0,
                measured_cn0_dbhz: 45.0,
                cn0_error_db: 0.0,
                lock: false,
                pll_lock: false,
                dll_lock: false,
                fll_lock: false,
                cycle_slip: false,
                lock_state: "degraded".to_string(),
                lock_state_reason: Some("no_stable_truth_window".to_string()),
                stable_tracking_epoch: false,
                pass: false,
            }],
        }],
    };

    let accuracy = super::validate_tracking_accuracy_budget(
        &report,
        truth_guided_receiver_accuracy_budgets().tracking,
    );

    assert!(!accuracy.truth_coverage_ready);
    assert_eq!(accuracy.truth_coverage_issues.len(), 1);
    assert_eq!(accuracy.truth_coverage_issues[0].sat, Some(report.satellites[0].sat));
    assert_eq!(accuracy.truth_coverage_issues[0].code, "no_stable_tracking_truth_epochs");
    assert!(!accuracy.pass);
}

#[test]
fn supported_tracking_signal_identities_match_receiver_execution_surface() {
    let signals = super::supported_tracking_signal_identities();

    let l2c = SyntheticTrackingSignalIdentity {
        constellation: Constellation::Gps,
        signal_band: SignalBand::L2,
        signal_code: SignalCode::L2C,
        glonass_frequency_channel: None,
    };

    assert!(!signals.contains(&l2c));
    assert!(signals.contains(&SyntheticTrackingSignalIdentity {
        constellation: Constellation::Gps,
        signal_band: SignalBand::L5,
        signal_code: SignalCode::L5Q,
        glonass_frequency_channel: None,
    }));
    let glonass = SyntheticTrackingSignalIdentity {
        constellation: Constellation::Glonass,
        signal_band: SignalBand::L1,
        signal_code: SignalCode::Unknown,
        glonass_frequency_channel: Some(
            GlonassFrequencyChannel::new(-4).expect("valid GLONASS channel"),
        ),
    };
    let galileo_e1b = SyntheticTrackingSignalIdentity {
        constellation: Constellation::Galileo,
        signal_band: SignalBand::E1,
        signal_code: SignalCode::E1B,
        glonass_frequency_channel: None,
    };
    let beidou_b1i = SyntheticTrackingSignalIdentity {
        constellation: Constellation::Beidou,
        signal_band: SignalBand::B1,
        signal_code: SignalCode::B1I,
        glonass_frequency_channel: None,
    };
    let beidou_b2i = SyntheticTrackingSignalIdentity {
        constellation: Constellation::Beidou,
        signal_band: SignalBand::B2,
        signal_code: SignalCode::B2I,
        glonass_frequency_channel: None,
    };

    assert!(!signals.contains(&glonass));
    assert!(!signals.contains(&galileo_e1b));
    assert!(!signals.contains(&beidou_b1i));
    assert!(!signals.contains(&beidou_b2i));
    assert!(!signals.contains(&SyntheticTrackingSignalIdentity {
        constellation: Constellation::Gps,
        signal_band: SignalBand::L2,
        signal_code: SignalCode::Py,
        glonass_frequency_channel: None,
    }));
    assert!(!signals.contains(&SyntheticTrackingSignalIdentity {
        constellation: Constellation::Galileo,
        signal_band: SignalBand::E1,
        signal_code: SignalCode::E1C,
        glonass_frequency_channel: None,
    }));

    let empty_noise = super::characterize_supported_tracking_noise(&[], 1);
    assert!(empty_noise.tracking_only_signals.contains(&l2c), "{empty_noise:?}");
    assert!(
        empty_noise.unstable_tracking_truth_signals.contains(&glonass),
        "{empty_noise:?}"
    );
    assert!(
        empty_noise.unstable_tracking_truth_signals.contains(&galileo_e1b),
        "{empty_noise:?}"
    );
    assert!(
        empty_noise.unstable_tracking_truth_signals.contains(&beidou_b1i),
        "{empty_noise:?}"
    );
    assert!(
        empty_noise.unstable_tracking_truth_signals.contains(&beidou_b2i),
        "{empty_noise:?}"
    );
    assert_eq!(
        empty_noise.tracking_only_signal_count,
        empty_noise.tracking_only_signals.len()
    );
    assert_eq!(
        empty_noise.unstable_tracking_truth_signal_count,
        empty_noise.unstable_tracking_truth_signals.len()
    );
}

#[test]
fn tracking_noise_report_requires_supported_signal_coverage() {
    let signals = super::supported_tracking_signal_identities();
    let report = tracking_noise_truth_report(signals[0], 2, 40.0, 125.0);

    let noise = super::characterize_supported_tracking_noise(&[report], 2);

    assert!(!noise.pass);
    assert_eq!(noise.characterized_signal_count, 1);
    assert_eq!(noise.supported_signal_count, signals.len());
    assert!(noise.tracking_only_signal_count > 0, "{noise:?}");
    assert!(noise.unstable_tracking_truth_signal_count > 0, "{noise:?}");
    assert_eq!(noise.missing_signals.len(), signals.len() - 1);
    assert!(noise.under_sampled_signals.is_empty(), "{noise:?}");
}

#[test]
fn tracking_noise_report_summarizes_empirical_distributions() {
    let signals = super::supported_tracking_signal_identities();
    let reports = signals
        .iter()
        .enumerate()
        .map(|(index, signal)| {
            tracking_noise_truth_report(*signal, 3, 42.0 + index as f64, 100.0 + index as f64)
        })
        .collect::<Vec<_>>();

    let noise = super::characterize_supported_tracking_noise(&reports, 3);

    assert!(noise.pass, "{noise:?}");
    assert_eq!(noise.supported_signal_count, signals.len());
    assert_eq!(noise.characterized_signal_count, signals.len());
    assert!(noise.missing_signals.is_empty(), "{noise:?}");
    assert!(noise.under_sampled_signals.is_empty(), "{noise:?}");

    let profile = noise
        .profiles
        .iter()
        .find(|profile| profile.signal == signals[0])
        .expect("first supported signal profile");
    assert_eq!(profile.stable_epoch_count, 3);
    assert_eq!(profile.satellite_count, 1);
    assert_eq!(profile.dll_jitter_samples.sample_count, 3);
    assert_eq!(profile.dll_jitter_samples.p50_abs, 0.2);
    assert!((profile.dll_jitter_samples.p95_abs - 0.3).abs() < f64::EPSILON);
    assert_eq!(profile.pll_phase_error_cycles.sample_count, 3);
    assert!(profile.pll_phase_error_cycles.max_abs > 0.0, "{profile:?}");
    assert_eq!(profile.doppler_error_hz.max_abs, 1.5);
    assert!(profile.cn0_bias_db_hz.mean > 0.0, "{profile:?}");
    assert_eq!(profile.cycle_slip_count, 1);
    assert_eq!(profile.cycle_slip_probability, 1.0 / 3.0);
}

fn tracking_noise_truth_report(
    signal: SyntheticTrackingSignalIdentity,
    stable_epoch_count: usize,
    cn0_db_hz: f64,
    abs_doppler_hz: f64,
) -> SyntheticTrackingTruthTableReport {
    SyntheticTrackingTruthTableReport {
        scenario_id: format!(
            "tracking_noise_{:?}_{:?}_{:?}",
            signal.constellation, signal.signal_band, signal.signal_code
        ),
        carrier_tolerance_hz: 10.0,
        doppler_tolerance_hz: 10.0,
        code_phase_tolerance_samples: 1.0,
        cn0_tolerance_db_hz: 8.0,
        sample_rate_hz: 4_092_000.0,
        period_samples: 4092,
        output_scale_applied: 1.0,
        pass: true,
        satellites: vec![SyntheticTrackingTruthTableSatellite {
            sat: SatId { constellation: signal.constellation, prn: 7 },
            signal_band: signal.signal_band,
            signal_code: signal.signal_code,
            glonass_frequency_channel: signal.glonass_frequency_channel,
            injected_doppler_hz: abs_doppler_hz,
            expected_measured_doppler_hz: abs_doppler_hz,
            injected_code_phase_chips: 100.0,
            injected_cn0_db_hz: cn0_db_hz as f32,
            epoch_count: stable_epoch_count,
            stable_epoch_count,
            first_stable_epoch_index: Some(0),
            pass: true,
            epochs: (0..stable_epoch_count)
                .map(|index| tracking_noise_truth_epoch(index, cn0_db_hz, abs_doppler_hz))
                .collect(),
        }],
    }
}

fn tracking_noise_truth_epoch(
    index: usize,
    cn0_db_hz: f64,
    abs_doppler_hz: f64,
) -> SyntheticTrackingTruthTableEpoch {
    let code_error = 0.1 * (index as f64 + 1.0);
    SyntheticTrackingTruthTableEpoch {
        epoch_index: index,
        sample_index: index as u64 * 4092,
        expected_carrier_hz: abs_doppler_hz,
        measured_carrier_hz: abs_doppler_hz + 0.5,
        carrier_error_hz: 0.5,
        expected_doppler_hz: abs_doppler_hz,
        measured_doppler_hz: abs_doppler_hz + 0.5 * (index as f64 + 1.0),
        doppler_error_hz: 0.5 * (index as f64 + 1.0),
        pll_phase_error_rad: 0.01 * (index as f64 + 1.0),
        pll_phase_error_cycles: 0.01 * (index as f64 + 1.0) / std::f64::consts::TAU,
        expected_code_phase_samples: 100.0,
        measured_code_phase_samples: 100.0 + code_error,
        code_phase_error_samples: code_error,
        expected_cn0_db_hz: cn0_db_hz,
        measured_cn0_dbhz: cn0_db_hz + 0.25 * (index as f64 + 1.0),
        cn0_error_db: 0.25 * (index as f64 + 1.0),
        lock: true,
        pll_lock: true,
        dll_lock: true,
        fll_lock: true,
        cycle_slip: index == 2,
        lock_state: "tracking".to_string(),
        lock_state_reason: None,
        stable_tracking_epoch: true,
        pass: true,
    }
}

#[test]
fn pvt_accuracy_budget_fails_invalid_or_out_of_budget_epoch() {
    let report = SyntheticPvtTruthTableReport {
        scenario_id: "pvt_budget_failure".to_string(),
        solution_count: 1,
        matched_epoch_count: 1,
        unmatched_solution_epochs: Vec::new(),
        unused_reference_epochs: Vec::new(),
        epochs: vec![SyntheticPvtTruthTableEpoch {
            artifact_id: "nav-epoch-invalid".to_string(),
            source_observation_epoch_id: "obs-epoch-invalid".to_string(),
            epoch_index: 9,
            receive_time_s: 100_000.0,
            truth_ecef_m: SyntheticPvtTruthTableEcef { x_m: 0.0, y_m: 0.0, z_m: 0.0 },
            measured_ecef_m: SyntheticPvtTruthTableEcef { x_m: 0.0, y_m: 0.0, z_m: 6.0 },
            position_covariance_ecef_m2: None,
            ecef_error_m: SyntheticPvtTruthTableEcef { x_m: 0.0, y_m: 0.0, z_m: 6.0 },
            truth_geodetic: SyntheticPvtTruthTableGeodetic {
                latitude_deg: 0.0,
                longitude_deg: 0.0,
                altitude_m: 0.0,
            },
            measured_geodetic: SyntheticPvtTruthTableGeodetic {
                latitude_deg: 0.0,
                longitude_deg: 0.0,
                altitude_m: 6.0,
            },
            enu_error_m: SyntheticPvtTruthTableEnuError {
                east_m: 0.0,
                north_m: 0.0,
                up_m: 6.0,
                horiz_m: 0.0,
                vert_m: 6.0,
                error_3d_m: 6.0,
            },
            clock_bias: SyntheticPvtTruthTableClockBias {
                truth_s: 0.0,
                measured_s: 1.0e-6,
                error_s: 1.0e-6,
                truth_m: 0.0,
                measured_m: 10.0,
                error_m: 10.0,
            },
            residual_rms_m: 2.0,
            pre_fit_residual_rms_m: Some(2.0),
            post_fit_residual_rms_m: Some(2.0),
            dop: SyntheticPvtTruthTableDop {
                pdop: 4.0,
                hdop: Some(2.0),
                vdop: Some(2.0),
                gdop: Some(4.5),
                tdop: Some(1.0),
            },
            solution_status: SolutionStatus::Unavailable,
            solution_quality: NavQualityFlag::NoFix,
            solution_validity: SolutionValidity::Invalid,
            valid: false,
            sat_count: 4,
            used_sat_count: 4,
            rejected_sat_count: 0,
        }],
    };

    let accuracy =
        validate_pvt_accuracy_budget(&report, truth_guided_receiver_accuracy_budgets().pvt);
    let epoch = accuracy.epochs.first().expect("pvt epoch");

    assert!(!accuracy.pass);
    assert_eq!(accuracy.passing_epoch_count, 0);
    assert!(!epoch.pass);
}

#[test]
fn pvt_covariance_realism_reports_empirical_coverage_from_truth_table_rows() {
    let report = SyntheticPvtTruthTableReport {
        scenario_id: "pvt_covariance_realism".to_string(),
        solution_count: 20,
        matched_epoch_count: 20,
        unmatched_solution_epochs: Vec::new(),
        unused_reference_epochs: Vec::new(),
        epochs: (0..20)
            .map(|epoch_index| SyntheticPvtTruthTableEpoch {
                artifact_id: format!("nav-epoch-{epoch_index:04}"),
                source_observation_epoch_id: format!("obs-epoch-{epoch_index:04}"),
                epoch_index,
                receive_time_s: epoch_index as f64,
                truth_ecef_m: SyntheticPvtTruthTableEcef { x_m: 6_378_137.0, y_m: 0.0, z_m: 0.0 },
                measured_ecef_m: SyntheticPvtTruthTableEcef {
                    x_m: 6_378_137.0,
                    y_m: if epoch_index < 19 { 0.5 } else { 3.5 },
                    z_m: 0.0,
                },
                position_covariance_ecef_m2: Some([
                    [9.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0],
                    [0.0, 0.0, 1.0],
                ]),
                ecef_error_m: SyntheticPvtTruthTableEcef {
                    x_m: 0.0,
                    y_m: if epoch_index < 19 { 0.5 } else { 3.5 },
                    z_m: 0.0,
                },
                truth_geodetic: SyntheticPvtTruthTableGeodetic {
                    latitude_deg: 0.0,
                    longitude_deg: 0.0,
                    altitude_m: 0.0,
                },
                measured_geodetic: SyntheticPvtTruthTableGeodetic {
                    latitude_deg: 0.0,
                    longitude_deg: 0.0,
                    altitude_m: 0.0,
                },
                enu_error_m: SyntheticPvtTruthTableEnuError {
                    east_m: if epoch_index < 19 { 0.5 } else { 3.5 },
                    north_m: 0.0,
                    up_m: 0.0,
                    horiz_m: if epoch_index < 19 { 0.5 } else { 3.5 },
                    vert_m: 0.0,
                    error_3d_m: if epoch_index < 19 { 0.5 } else { 3.5 },
                },
                clock_bias: SyntheticPvtTruthTableClockBias {
                    truth_s: 0.0,
                    measured_s: 0.0,
                    error_s: 0.0,
                    truth_m: 0.0,
                    measured_m: 0.0,
                    error_m: 0.0,
                },
                residual_rms_m: 0.1,
                pre_fit_residual_rms_m: Some(0.1),
                post_fit_residual_rms_m: Some(0.1),
                dop: SyntheticPvtTruthTableDop {
                    pdop: 1.0,
                    hdop: Some(1.0),
                    vdop: Some(1.0),
                    gdop: Some(1.0),
                    tdop: Some(1.0),
                },
                solution_status: SolutionStatus::CodeOnly,
                solution_quality: NavQualityFlag::Float,
                solution_validity: SolutionValidity::Stable,
                valid: true,
                sat_count: 6,
                used_sat_count: 6,
                rejected_sat_count: 0,
            })
            .collect(),
    };

    let realism = super::validate_pvt_covariance_realism(&report);

    assert_eq!(realism.total_epoch_count, 20);
    assert_eq!(realism.covariance_epoch_count, 20);
    assert_eq!(realism.horizontal_95.inside_count, 19);
    assert_eq!(realism.vertical_95.inside_count, 20);
    assert_eq!(realism.position_3d_95.inside_count, 19);
}
