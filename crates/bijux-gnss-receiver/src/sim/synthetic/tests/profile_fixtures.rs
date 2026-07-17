    fn clock_profile_accuracy_report(
        scenario_id: &str,
        epoch_index: u64,
        clock_bias_error_m: f64,
    ) -> SyntheticPvtAccuracyReport {
        SyntheticPvtAccuracyReport {
            scenario_id: scenario_id.to_string(),
            max_position_error_3d_m: 5.0,
            max_clock_bias_error_m: 20.0,
            max_residual_rms_m: 4.0,
            max_pdop: 6.0,
            epoch_count: 1,
            passing_epoch_count: 1,
            truth_coverage_ready: true,
            truth_coverage_issues: Vec::new(),
            pass: true,
            epochs: vec![SyntheticPvtAccuracyEpoch {
                epoch_index,
                position_error_3d_m: 0.5,
                clock_bias_error_m,
                residual_rms_m: 0.25,
                pdop: 1.5,
                pass: true,
            }],
        }
    }

    fn clock_profile_solution(epoch_index: u64, clock_drift_s_per_s: f64) -> NavSolutionEpoch {
        NavSolutionEpoch {
            epoch: Epoch { index: epoch_index },
            t_rx_s: Seconds(100_000.0 + epoch_index as f64),
            source_time: ReceiverSampleTrace::from_sample_index(epoch_index, 1.0),
            ecef_x_m: Meters(0.0),
            ecef_y_m: Meters(0.0),
            ecef_z_m: Meters(0.0),
            position_covariance_ecef_m2: None,
            latitude_deg: 0.0,
            longitude_deg: 0.0,
            altitude_m: Meters(0.0),
            clock_bias_s: Seconds(0.0),
            clock_bias_m: Meters(0.0),
            clock_drift_s_per_s,
            pdop: 1.5,
            pre_fit_residual_rms_m: Some(Meters(0.25)),
            post_fit_residual_rms_m: Some(Meters(0.25)),
            rms_m: Meters(0.25),
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
            wls_solver_rank: None,
            wls_condition_number: None,
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
            artifact_id: format!("clock-profile-artifact-{epoch_index}"),
            source_observation_epoch_id: format!("clock-profile-observation-{epoch_index}"),
            explain_decision: "accepted".to_string(),
            explain_reasons: vec!["clock_profile_solution".to_string()],
            provenance: None,
            sat_count: 4,
            used_sat_count: 4,
            rejected_sat_count: 0,
            hdop: Some(1.0),
            vdop: Some(1.0),
            gdop: Some(1.8),
            tdop: Some(0.5),
            stability_signature: format!("navsig:v2:clock-profile-{epoch_index}"),
            stability_signature_version: NAV_OUTPUT_STABILITY_SIGNATURE_VERSION,
        }
    }

    fn clock_profile_obs_epoch(
        epoch_idx: u64,
        doppler_hz_values: &[f64],
    ) -> ObservationEpoch {
        crate::pipeline::observations::accepted_rover_observation_epoch(
            crate::pipeline::observations::AcceptedRoverObservationEpochRequest {
                t_rx_s: Seconds(100_000.0 + epoch_idx as f64),
                source_time: ReceiverSampleTrace::from_sample_index(epoch_idx, 1.0),
                gps_week: Some(0),
                tow_s: Some(Seconds(100_000.0 + epoch_idx as f64)),
                epoch_idx,
                discontinuity: false,
                sats: doppler_hz_values
                    .iter()
                    .enumerate()
                    .map(|(offset, doppler_hz)| ObsSatellite {
                        signal_id: SigId {
                            sat: SatId { constellation: Constellation::Gps, prn: 7 + offset as u8 },
                            band: SignalBand::L1,
                            code: SignalCode::Ca,
                        },
                        pseudorange_m: Meters(20_000_000.0 + offset as f64),
                        pseudorange_var_m2: 1.0,
                        carrier_phase_cycles: Cycles(1000.0 + offset as f64),
                        carrier_phase_var_cycles2: 1.0,
                        doppler_hz: Hertz(*doppler_hz),
                        doppler_var_hz2: 1.0,
                        cn0_dbhz: 45.0,
                        lock_flags: LockFlags {
                            code_lock: true,
                            carrier_lock: true,
                            bit_lock: true,
                            cycle_slip: false,
                        },
                        multipath_suspect: false,
                        observation_status: ObservationStatus::Accepted,
                        observation_reject_reasons: Vec::new(),
                        elevation_deg: Some(45.0),
                        azimuth_deg: Some(90.0),
                        weight: Some(1.0),
                        timing: None,
                        error_model: None,
                        metadata: ObsMetadata {
                            signal: SignalSpec {
                                constellation: Constellation::Gps,
                                band: SignalBand::L1,
                                code: SignalCode::Ca,
                                code_rate_hz: 1_023_000.0,
                                carrier_hz: FreqHz(1_575_420_000.0),
                            },
                            ..ObsMetadata::default()
                        },
                    })
                    .collect(),
                decision_reason: Some("clock_profile_test".to_string()),
                manifest: None,
            },
        )
    }

    fn sample_obs_epoch(epoch_idx: u64, cn0_values_dbhz: &[f64]) -> ObservationEpoch {
        crate::pipeline::observations::accepted_rover_observation_epoch(
            crate::pipeline::observations::AcceptedRoverObservationEpochRequest {
                t_rx_s: Seconds(epoch_idx as f64),
                source_time: ReceiverSampleTrace::from_sample_index(epoch_idx, 1.0),
                gps_week: Some(0),
                tow_s: Some(Seconds(epoch_idx as f64)),
                epoch_idx,
                discontinuity: false,
                sats: cn0_values_dbhz
                    .iter()
                    .enumerate()
                    .map(|(offset, cn0_dbhz)| ObsSatellite {
                        signal_id: SigId {
                            sat: SatId { constellation: Constellation::Gps, prn: 7 + offset as u8 },
                            band: SignalBand::L1,
                            code: SignalCode::Ca,
                        },
                        pseudorange_m: Meters(20_000_000.0 + offset as f64),
                        pseudorange_var_m2: 1.0,
                        carrier_phase_cycles: Cycles(1000.0 + offset as f64),
                        carrier_phase_var_cycles2: 1.0,
                        doppler_hz: Hertz(-500.0 + offset as f64),
                        doppler_var_hz2: 1.0,
                        cn0_dbhz: *cn0_dbhz,
                        lock_flags: LockFlags {
                            code_lock: true,
                            carrier_lock: true,
                            bit_lock: true,
                            cycle_slip: false,
                        },
                        multipath_suspect: false,
                        observation_status: ObservationStatus::Accepted,
                        observation_reject_reasons: Vec::new(),
                        elevation_deg: Some(45.0),
                        azimuth_deg: Some(90.0),
                        weight: Some(1.0),
                        timing: None,
                        error_model: None,
                        metadata: ObsMetadata {
                            signal: SignalSpec {
                                constellation: Constellation::Gps,
                                band: SignalBand::L1,
                                code: SignalCode::Ca,
                                code_rate_hz: 1_023_000.0,
                                carrier_hz: FreqHz(1_575_420_000.0),
                            },
                            ..ObsMetadata::default()
                        },
                    })
                    .collect(),
                decision_reason: Some("synthetic_cn0_profile".to_string()),
                manifest: None,
            },
        )
    }

    fn collect_frames(source: &mut SyntheticSignalSource, frame_len: usize) -> SamplesFrame {
        let mut frames = Vec::new();
        while let Some(frame) = source.next_frame(frame_len).expect("synthetic frame") {
            frames.push(frame);
        }
        let first = frames.first().expect("at least one frame");
        let t0 = first.t0;
        let dt_s = first.dt_s;
        let iq = frames.into_iter().flat_map(|frame| frame.iq).collect::<Vec<_>>();
        SamplesFrame::new(t0, dt_s, iq)
    }
