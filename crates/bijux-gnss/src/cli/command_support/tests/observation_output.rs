#[test]
fn nav_solution_output_preserves_source_trace() {
    let out_dir = std::env::temp_dir().join(format!(
        "bijux_nav_solution_output_{}_{}",
        std::process::id(),
        SystemTime::now().duration_since(UNIX_EPOCH).expect("unix epoch").as_nanos()
    ));
    fs::create_dir_all(&out_dir).expect("create output directory");
    let solution = NavSolutionEpoch {
        epoch: Epoch { index: 12 },
        t_rx_s: Seconds(1.5),
        source_time: ReceiverSampleTrace::from_sample_index(6_138, 4_092_000.0),
        ecef_x_m: Meters(1.0),
        ecef_y_m: Meters(2.0),
        ecef_z_m: Meters(3.0),
        position_covariance_ecef_m2: None,
        latitude_deg: 60.0,
        longitude_deg: 18.0,
        altitude_m: Meters(4.0),
        clock_bias_s: Seconds(0.001),
        clock_bias_m: Meters(299_792.458),
        clock_drift_s_per_s: 0.0,
        pdop: 1.0,
        pre_fit_residual_rms_m: None,
        post_fit_residual_rms_m: None,
        rms_m: Meters(2.0),
        status: SolutionStatus::CodeOnly,
        quality: SolutionStatus::CodeOnly.quality_flag(),
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
        sigma_h_m: Some(Meters(0.5)),
        sigma_v_m: Some(Meters(0.8)),
        innovation_rms_m: None,
        normalized_innovation_rms: None,
        normalized_innovation_max: None,
        ekf_innovation_rms: None,
        ekf_condition_number: None,
        wls_solver_rank: Some(4),
        wls_condition_number: Some(12.5),
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
        artifact_id: "nav-epoch-0000000012-source".to_string(),
        source_observation_epoch_id: "epoch-0000000012-sample-000000006138".to_string(),
        explain_decision: "accepted".to_string(),
        explain_reasons: vec!["navigation_solution_usable".to_string()],
        provenance: None,
        sat_count: 4,
        used_sat_count: 4,
        rejected_sat_count: 0,
        hdop: Some(0.8),
        vdop: Some(0.6),
        gdop: Some(1.05),
        tdop: Some(0.4),
        stability_signature: "navsig:v2:test".to_string(),
        stability_signature_version: NAV_OUTPUT_STABILITY_SIGNATURE_VERSION,
    };

    super::write_nav_solution_outputs(&out_dir, &[solution]).expect("write nav solution");

    let path = out_dir.join("nav_solution.jsonl");
    let line = fs::read_to_string(&path).expect("read nav solution");
    let payload: serde_json::Value =
        serde_json::from_str(line.lines().next().unwrap_or("")).expect("parse nav solution line");
    assert_eq!(payload["source_time"]["sample_index"], 6_138);
    assert_eq!(payload["source_time"]["sample_rate_hz"], 4_092_000.0);
    assert_eq!(payload["source_time"]["receiver_time_s"], 0.0015);
    assert_eq!(payload["source_observation_epoch_id"], "epoch-0000000012-sample-000000006138");
    assert_eq!(payload["clock_bias_s"], 0.001);
    assert_eq!(payload["clock_bias_m"], 299_792.458);
    assert_eq!(payload["dops"]["pdop"], 1.0);
    assert_eq!(payload["dops"]["hdop"], 0.8);
    assert_eq!(payload["dops"]["vdop"], 0.6);
    assert_eq!(payload["dops"]["gdop"], 1.05);
    assert_eq!(payload["dops"]["tdop"], 0.4);
    assert_eq!(payload["solver_diagnostics"]["wls_solver_rank"], 4);
    assert_eq!(payload["solver_diagnostics"]["wls_condition_number"], 12.5);
    assert!(payload["solver_diagnostics"]["ekf_condition_number"].is_null());

    fs::remove_file(&path).expect("remove nav solution output");
    fs::remove_dir(&out_dir).expect("remove output directory");
}

#[test]
fn write_obs_timeseries_emits_observation_residual_artifact() {
    let out_dir = unique_artifact_output_dir("obs_residual_output");
    let common = sample_common_args(out_dir.clone());
    let profile = ReceiverConfig::default();
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 1_023_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let sat = SatId { constellation: Constellation::Gps, prn: 19 };
    let carrier_hz = bijux_gnss_infra::api::receiver::carrier_hz_from_doppler_hz(0.0, 125.0);
    let track = bijux_gnss_infra::api::receiver::TrackingResult {
        sat,
        carrier_hz,
        code_phase_samples: 0.0,
        acquisition_hypothesis: "accepted".to_string(),
        acquisition_score: 1.0,
        acquisition_code_phase_samples: 0,
        acquisition_carrier_hz: carrier_hz,
        acq_to_track_state: "accepted".to_string(),
        epochs: vec![
            TrackEpoch {
                epoch: Epoch { index: 70 },
                sample_index: 70 * 1023,
                source_time: ReceiverSampleTrace::from_sample_index(70 * 1023, 1_023_000.0),
                sat,
                signal_band: SignalBand::L1,
                signal_code: bijux_gnss_infra::api::core::SignalCode::Unknown,
                glonass_frequency_channel: None,
                prompt_i: 1.0,
                prompt_q: 0.0,
                early_i: 0.0,
                early_q: 0.0,
                late_i: 0.0,
                late_q: 0.0,
                carrier_hz: Hertz(carrier_hz),
                carrier_phase_cycles: Cycles(10.0),
                code_rate_hz: Hertz(1_023_000.0),
                code_phase_samples: Chips(0.0),
                lock: true,
                cn0_dbhz: 45.0,
                pll_lock: true,
                dll_lock: true,
                fll_lock: true,
                cycle_slip: false,
                nav_bit_lock: true,
                navigation_bit_sign: Some(1),
                transmit_time: None,
                dll_err: 0.0,
                pll_err: 0.0,
                fll_err: 0.0,
                anti_false_lock: false,
                cycle_slip_reason: None,
                lock_state: "tracking".to_string(),
                lock_state_reason: Some("stable_tracking".to_string()),
                channel_id: None,
                channel_uid: String::new(),
                tracking_provenance: "test".to_string(),
                tracking_assumptions: None,
                signal_delay_alignment: Some(SignalDelayAlignment {
                    whole_code_periods: 68,
                    sample_delay_samples: 0,
                    source: "synthetic_truth".to_string(),
                }),
                tracking_uncertainty: Some(TrackingUncertainty {
                    code_phase_samples: 0.05,
                    carrier_phase_cycles: 0.02,
                    doppler_hz: 0.5,
                    cn0_dbhz: 0.75,
                }),
                processing_ms: None,
            },
            TrackEpoch {
                epoch: Epoch { index: 71 },
                sample_index: 71 * 1023,
                source_time: ReceiverSampleTrace::from_sample_index(71 * 1023, 1_023_000.0),
                sat,
                signal_band: SignalBand::L1,
                signal_code: bijux_gnss_infra::api::core::SignalCode::Unknown,
                glonass_frequency_channel: None,
                prompt_i: 1.0,
                prompt_q: 0.0,
                early_i: 0.0,
                early_q: 0.0,
                late_i: 0.0,
                late_q: 0.0,
                carrier_hz: Hertz(carrier_hz),
                carrier_phase_cycles: Cycles(10.125),
                code_rate_hz: Hertz(1_023_000.0),
                code_phase_samples: Chips(0.0),
                lock: true,
                cn0_dbhz: 45.0,
                pll_lock: true,
                dll_lock: true,
                fll_lock: true,
                cycle_slip: false,
                nav_bit_lock: true,
                navigation_bit_sign: Some(1),
                transmit_time: None,
                dll_err: 0.0,
                pll_err: 0.0,
                fll_err: 0.0,
                anti_false_lock: false,
                cycle_slip_reason: None,
                lock_state: "tracking".to_string(),
                lock_state_reason: Some("stable_tracking".to_string()),
                channel_id: None,
                channel_uid: String::new(),
                tracking_provenance: "test".to_string(),
                tracking_assumptions: None,
                signal_delay_alignment: Some(SignalDelayAlignment {
                    whole_code_periods: 68,
                    sample_delay_samples: 0,
                    source: "synthetic_truth".to_string(),
                }),
                tracking_uncertainty: Some(TrackingUncertainty {
                    code_phase_samples: 0.05,
                    carrier_phase_cycles: 0.02,
                    doppler_hz: 0.5,
                    cn0_dbhz: 0.75,
                }),
                processing_ms: None,
            },
        ],
        transitions: Vec::new(),
    };

    super::write_obs_timeseries(&common, &config, &[track], 10, &profile, None)
        .expect("write observation artifacts");

    let artifacts_dir = super::artifacts_dir(&common, "track", None).expect("artifacts dir");
    let residual_path = artifacts_dir.join("observation_residuals.jsonl");
    let residual_text = fs::read_to_string(&residual_path).expect("read residual artifact");
    let first_line = residual_text.lines().next().expect("residual line");
    let payload: serde_json::Value = serde_json::from_str(first_line).expect("parse residual");
    let raw_pseudorange_m = payload["payload"]["sats"][0]["pseudorange_m"]["raw"]
        .as_f64()
        .expect("raw pseudorange");
    let carrier_smoothed_code_validation_path =
        artifacts_dir.join("carrier_smoothed_code_validation.json");
    let carrier_smoothed_code_validation: serde_json::Value = serde_json::from_str(
        &fs::read_to_string(&carrier_smoothed_code_validation_path)
            .expect("read carrier-smoothed code validation artifact"),
    )
    .expect("parse carrier-smoothed code validation artifact");

    assert_eq!(payload["payload"]["artifact_id"], "obs-epoch-0000000070");
    assert_eq!(payload["payload"]["accepted"], true);
    assert!(raw_pseudorange_m > 0.0);
    assert_eq!(carrier_smoothed_code_validation["observations"], 2);
    assert_eq!(carrier_smoothed_code_validation["accepted_observations"], 2);
    assert_eq!(carrier_smoothed_code_validation["cycle_slip_observations"], 0);
    assert!(carrier_smoothed_code_validation["improvement_verified"].is_null());

    fs::remove_dir_all(&out_dir).expect("remove output directory");
}

#[test]
fn write_obs_timeseries_emits_observation_measurement_quality_artifact() {
    fn quality_signal(
        constellation: Constellation,
        band: SignalBand,
        code: SignalCode,
    ) -> bijux_gnss_infra::api::core::SignalSpec {
        signal_registry(constellation, band, code).expect("registered signal").spec
    }

    fn quality_alignment_periods(band: SignalBand) -> u64 {
        match band {
            SignalBand::L1 => 68,
            SignalBand::L2 => 4,
            SignalBand::L5 => 8,
            SignalBand::E1 => 17,
            SignalBand::E5 => 6,
            SignalBand::B1 => 8,
            SignalBand::B2 => 8,
            _ => 68,
        }
    }

    fn quality_track(
        config: &ReceiverPipelineConfig,
        sat: SatId,
        signal: bijux_gnss_infra::api::core::SignalSpec,
        cn0_dbhz: f64,
    ) -> bijux_gnss_infra::api::receiver::TrackingResult {
        let tracked_carrier_hz = signal.carrier_hz.value() - GPS_L1_CA_CARRIER_HZ.value() + 125.0;
        let epoch = TrackEpoch {
            epoch: Epoch { index: 0 },
            sample_index: 0,
            source_time: ReceiverSampleTrace::from_sample_index(0, config.sampling_freq_hz),
            sat,
            signal_band: signal.band,
            signal_code: signal.code,
            glonass_frequency_channel: None,
            prompt_i: 1.0,
            prompt_q: 0.0,
            early_i: 0.1,
            early_q: 0.0,
            late_i: -0.1,
            late_q: 0.0,
            carrier_hz: Hertz(tracked_carrier_hz + sat.prn as f64),
            carrier_phase_cycles: Cycles(2_400.0 + sat.prn as f64),
            code_rate_hz: Hertz(signal.code_rate_hz),
            code_phase_samples: Chips(0.0),
            lock: true,
            cn0_dbhz,
            pll_lock: true,
            dll_lock: true,
            fll_lock: true,
            cycle_slip: false,
            nav_bit_lock: false,
            navigation_bit_sign: None,
            transmit_time: None,
            dll_err: 0.0,
            pll_err: 0.0,
            fll_err: 0.0,
            anti_false_lock: false,
            cycle_slip_reason: None,
            lock_state: "tracking".to_string(),
            lock_state_reason: Some("stable_tracking".to_string()),
            channel_id: Some(sat.prn),
            channel_uid: format!("{:?}-{:02}-{:?}", sat.constellation, sat.prn, signal.band),
            tracking_provenance: "test".to_string(),
            tracking_assumptions: None,
            signal_delay_alignment: Some(SignalDelayAlignment {
                whole_code_periods: quality_alignment_periods(signal.band),
                sample_delay_samples: 0,
                source: "quality_fixture".to_string(),
            }),
            tracking_uncertainty: Some(TrackingUncertainty {
                code_phase_samples: 0.05,
                carrier_phase_cycles: 0.02,
                doppler_hz: 0.5,
                cn0_dbhz: 0.75,
            }),
            processing_ms: None,
        };
        bijux_gnss_infra::api::receiver::TrackingResult {
            sat,
            carrier_hz: epoch.carrier_hz.0,
            code_phase_samples: epoch.code_phase_samples.0,
            acquisition_hypothesis: "accepted".to_string(),
            acquisition_score: 1.0,
            acquisition_code_phase_samples: 0,
            acquisition_carrier_hz: epoch.carrier_hz.0,
            acq_to_track_state: "accepted".to_string(),
            epochs: vec![epoch],
            transitions: Vec::new(),
        }
    }

    let out_dir = unique_artifact_output_dir("obs_quality_output");
    let common = sample_common_args(out_dir.clone());
    let profile = ReceiverConfig::default();
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 10_230_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 8,
        tracking_integration_ms: 10,
        ..ReceiverPipelineConfig::default()
    };
    let mut l5 = quality_track(
        &config,
        SatId { constellation: Constellation::Gps, prn: 5 },
        quality_signal(Constellation::Gps, SignalBand::L5, SignalCode::L5I),
        43.0,
    );
    l5.epochs[0].cycle_slip = true;
    l5.epochs[0].cycle_slip_reason = Some("simulated_phase_slip".to_string());
    let mut b2 = quality_track(
        &config,
        SatId { constellation: Constellation::Beidou, prn: 8 },
        quality_signal(Constellation::Beidou, SignalBand::B2, SignalCode::B2I),
        39.0,
    );
    b2.epochs[0].pll_lock = false;
    let tracks = vec![
        quality_track(
            &config,
            SatId { constellation: Constellation::Gps, prn: 3 },
            signal_spec_gps_l1_ca(),
            46.0,
        ),
        quality_track(
            &config,
            SatId { constellation: Constellation::Gps, prn: 4 },
            quality_signal(Constellation::Gps, SignalBand::L2, SignalCode::L2C),
            44.0,
        ),
        l5,
        quality_track(
            &config,
            SatId { constellation: Constellation::Galileo, prn: 11 },
            quality_signal(Constellation::Galileo, SignalBand::E1, SignalCode::E1B),
            42.0,
        ),
        quality_track(
            &config,
            SatId { constellation: Constellation::Galileo, prn: 12 },
            quality_signal(Constellation::Galileo, SignalBand::E5, SignalCode::E5a),
            41.0,
        ),
        quality_track(
            &config,
            SatId { constellation: Constellation::Beidou, prn: 7 },
            quality_signal(Constellation::Beidou, SignalBand::B1, SignalCode::B1I),
            40.0,
        ),
        b2,
    ];

    super::write_obs_timeseries(&common, &config, &tracks, 10, &profile, None)
        .expect("write observation artifacts");

    let artifacts_dir = super::artifacts_dir(&common, "track", None).expect("artifacts dir");
    let quality_path = artifacts_dir.join("observation_measurement_quality.jsonl");
    let quality_text = fs::read_to_string(&quality_path).expect("read quality artifact");
    let first_line = quality_text.lines().next().expect("quality line");
    let payload: serde_json::Value = serde_json::from_str(first_line).expect("parse quality");
    let sats = payload["payload"]["sats"].as_array().expect("quality satellites");

    assert_eq!(sats.len(), 7);
    assert_eq!(
        sats.iter()
            .map(|sat| sat["signal_id"]["band"].as_str().expect("signal band"))
            .collect::<Vec<_>>(),
        vec!["L1", "L2", "L5", "E1", "E5", "B1", "B2"]
    );
    assert!(sats.iter().all(|sat| sat["cn0_dbhz"].as_f64().is_some()));
    assert!(sats.iter().all(|sat| sat["pseudorange_sigma_m"].as_f64().is_some()));
    assert!(sats.iter().all(|sat| sat["carrier_phase_sigma_cycles"].as_f64().is_some()));
    assert!(sats.iter().all(|sat| sat["doppler_sigma_hz"].as_f64().is_some()));
    assert!(sats.iter().all(|sat| sat["cn0_sigma_dbhz"].as_f64().is_some()));
    assert!(sats.iter().all(|sat| {
        let covariance = &sat["measurement_covariance"];
        covariance["status"].as_str() == Some("positive_semidefinite")
            && covariance["code_phase_m2"].as_f64().is_some_and(|value| value > 0.0)
            && covariance["carrier_phase_m2"].as_f64().is_some_and(|value| value > 0.0)
            && covariance["doppler_hz2"].as_f64().is_some_and(|value| value > 0.0)
            && covariance["code_carrier_m2"].as_f64() == covariance["carrier_code_m2"].as_f64()
            && covariance["carrier_doppler_m_hz"].as_f64()
                == covariance["doppler_carrier_hz_m"].as_f64()
    }));
    assert!(sats.iter().all(|sat| sat["observation_lock_state"]
        .as_str()
        .is_some_and(|state| !state.is_empty())));

    let l5_quality = sats.iter().find(|sat| sat["signal_id"]["band"] == "L5").expect("L5 quality");
    assert_eq!(l5_quality["cycle_slip"], true);
    assert_eq!(l5_quality["cycle_slip_reason"], "simulated_phase_slip");

    let b2_quality = sats.iter().find(|sat| sat["signal_id"]["band"] == "B2").expect("B2 quality");
    assert_eq!(b2_quality["lock_flags"]["code_lock"], true);
    assert_eq!(b2_quality["lock_flags"]["carrier_lock"], false);

    fs::remove_dir_all(&out_dir).expect("remove output directory");
}
