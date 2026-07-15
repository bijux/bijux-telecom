use super::*;

#[test]
fn observations_preserve_tracking_uncertainty_and_measurement_variances() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let sat = SatId { constellation: Constellation::Gps, prn: 9 };
    let uncertainty = TrackingUncertainty {
        code_phase_samples: 0.25,
        carrier_phase_cycles: 0.05,
        doppler_hz: 1.5,
        cn0_dbhz: 0.75,
    };
    let track = TrackingResult {
        sat,
        carrier_hz: 0.0,
        code_phase_samples: 0.0,
        acquisition_hypothesis: "accepted".to_string(),
        acquisition_score: 1.0,
        acquisition_code_phase_samples: 0,
        acquisition_carrier_hz: 0.0,
        acq_to_track_state: "accepted".to_string(),
        epochs: vec![TrackEpoch {
            epoch: Epoch { index: 70 },
            sample_index: 70 * 4092,
            source_time: ReceiverSampleTrace::from_sample_index(70 * 4092, config.sampling_freq_hz),
            sat,
            lock: true,
            cn0_dbhz: 48.0,
            pll_lock: true,
            dll_lock: true,
            fll_lock: true,
            lock_state: "tracking".to_string(),
            lock_state_reason: Some("stable_tracking".to_string()),
            transmit_time: None,
            tracking_uncertainty: Some(uncertainty.clone()),
            ..TrackEpoch::default()
        }],
        transitions: Vec::new(),
    };

    let report = observations_from_tracking_results(&config, &[track], 10);
    let epoch = report.output.first().expect("observation epoch");
    let sat = epoch.sats.first().expect("observation satellite");
    let quality = observation_measurement_quality_from_epochs(&report.output);
    let quality_sat = quality[0].sats.first().expect("measurement quality satellite");
    let meters_per_sample = SPEED_OF_LIGHT_MPS / config.sampling_freq_hz;
    let expected_pseudorange_sigma_m = uncertainty.code_phase_samples * meters_per_sample;

    assert_eq!(sat.metadata.tracking_uncertainty.as_ref(), Some(&uncertainty));
    assert!(
        (sat.pseudorange_var_m2 - expected_pseudorange_sigma_m.powi(2)).abs() <= 1.0e-9,
        "{sat:?}"
    );
    assert!(
        (sat.carrier_phase_var_cycles2 - uncertainty.carrier_phase_cycles.powi(2)).abs() <= 1.0e-12,
        "{sat:?}"
    );
    assert!((sat.doppler_var_hz2 - uncertainty.doppler_hz.powi(2)).abs() <= 1.0e-12, "{sat:?}");
    let error_model = sat.error_model.as_ref().expect("error model");
    assert!((error_model.tracking_jitter_m.0 - expected_pseudorange_sigma_m).abs() <= 1.0e-9);
    assert_eq!(error_model.thermal_noise_m, Meters(0.0));
    let covariance = sat.measurement_covariance().expect("measurement covariance");
    let covariance_matrix = covariance.matrix();
    assert!((covariance.code_phase_m2 - sat.pseudorange_var_m2).abs() <= 1.0e-9);
    assert!((covariance.doppler_hz2 - sat.doppler_var_hz2).abs() <= 1.0e-12);
    assert_eq!(covariance_matrix[0][1], covariance_matrix[1][0]);
    assert_eq!(covariance_matrix[1][2], covariance_matrix[2][1]);
    assert!(covariance.carrier_doppler_m_hz > 0.0);
    assert_eq!(quality_sat.measurement_covariance, Some(covariance));
    assert_eq!(quality_sat.code_carrier_divergence, sat.metadata.code_carrier_divergence);
    assert_eq!(quality_sat.cycle_slip_evidence, sat.metadata.cycle_slip_evidence);
    let slip_evidence = quality_sat.cycle_slip_evidence.as_ref().expect("cycle-slip evidence");
    assert_eq!(slip_evidence.detection_probability_budget, CYCLE_SLIP_DETECTION_PROBABILITY_BUDGET);
    assert_eq!(
        slip_evidence.false_alarm_probability_budget,
        CYCLE_SLIP_FALSE_ALARM_PROBABILITY_BUDGET
    );
    let divergence =
        quality_sat.code_carrier_divergence.expect("code-carrier divergence quality evidence");
    assert!(divergence.raw_m.is_finite());
    assert!(divergence.jump_m.is_finite());
}

#[test]
fn observations_mark_missing_variance_evidence_unusable() {
    let config = ReceiverPipelineConfig::default();
    let mut epoch = make_observation_ready_epoch(12, &config, 70);
    epoch.tracking_uncertainty = None;

    let report =
        observation_artifacts_from_tracking_results(&config, &[track_from_epoch(epoch)], 10);
    let observation_epoch = report.output.epochs.first().expect("observation epoch");
    let sat = observation_epoch.sats.first().expect("observation satellite");
    let quality =
        report.output.measurement_quality[0].sats.first().expect("measurement quality satellite");
    let residual = report.output.residuals[0].sats.first().expect("residual satellite");

    assert_eq!(observation_epoch.decision, ObservationEpochDecision::Rejected);
    assert_eq!(observation_epoch.decision_reason.as_deref(), Some("no_accepted_observables"));
    assert_eq!(sat.observation_status, ObservationStatus::Missing);
    assert!(sat
        .observation_reject_reasons
        .iter()
        .any(|reason| reason == "missing_tracking_uncertainty"));
    assert_eq!(sat.pseudorange_var_m2, 0.0);
    assert_eq!(sat.carrier_phase_var_cycles2, 0.0);
    assert_eq!(sat.doppler_var_hz2, 0.0);
    assert!(sat.error_model.is_none());
    assert!(sat.measurement_covariance().is_none());
    assert!(sat.covariance_pseudorange_sigma_m().is_none());
    assert_eq!(sat.metadata.observation_uncertainty_class, "unknown");
    assert!(quality.pseudorange_sigma_m.is_none());
    assert!(quality.carrier_phase_sigma_cycles.is_none());
    assert!(quality.doppler_sigma_hz.is_none());
    assert!(quality.measurement_covariance.is_none());
    assert_eq!(quality.code_carrier_divergence, sat.metadata.code_carrier_divergence);
    assert_eq!(quality.cycle_slip_evidence, sat.metadata.cycle_slip_evidence);
    assert!(residual.pseudorange_m.sigma.is_none());
    assert!(residual.carrier_phase_cycles.sigma.is_none());
    assert!(residual.doppler_hz.sigma.is_none());
}

#[test]
fn observations_preserve_degraded_doppler_uncertainty() {
    let config = ReceiverPipelineConfig::default();
    let sat = SatId { constellation: Constellation::Gps, prn: 14 };
    let expected_doppler_hz = 125.0;
    let doppler_uncertainty_hz: f64 = 180.0;
    let carrier_hz = crate::pipeline::doppler::carrier_hz_from_doppler_hz(
        config.intermediate_freq_hz,
        expected_doppler_hz,
    );
    let uncertainty = TrackingUncertainty {
        code_phase_samples: 0.4,
        carrier_phase_cycles: 0.2,
        doppler_hz: doppler_uncertainty_hz,
        cn0_dbhz: 1.0,
    };
    let epoch = TrackEpoch {
        epoch: Epoch { index: 70 },
        sample_index: epoch_sample_index(&config, 70),
        source_time: ReceiverSampleTrace::from_sample_index(
            epoch_sample_index(&config, 70),
            config.sampling_freq_hz,
        ),
        sat,
        carrier_hz: Hertz(carrier_hz),
        code_rate_hz: Hertz(config.code_freq_basis_hz),
        lock: true,
        pll_lock: false,
        dll_lock: true,
        fll_lock: false,
        cn0_dbhz: 45.0,
        lock_state: "degraded".to_string(),
        lock_state_reason: Some("doppler_estimator_divergence".to_string()),
        transmit_time: None,
        tracking_uncertainty: Some(uncertainty.clone()),
        ..TrackEpoch::default()
    };
    let report = observations_from_tracking_results(&config, &[track_from_epoch(epoch)], 10);
    let observation_epoch = report.output.first().expect("observation epoch");
    let observed_sat = observation_epoch.sats.first().expect("observation satellite");

    assert_eq!(observed_sat.metadata.tracking_state, "degraded");
    assert_eq!(
        observed_sat.metadata.observation_lock_reason.as_deref(),
        Some("doppler_estimator_divergence")
    );
    assert!(!observed_sat.lock_flags.carrier_lock, "{observed_sat:?}");
    assert!(
        (observed_sat.doppler_hz.0 - expected_doppler_hz).abs() <= f64::EPSILON,
        "{observed_sat:?}"
    );
    assert_eq!(observed_sat.metadata.tracking_uncertainty.as_ref(), Some(&uncertainty));
    assert!(
        (observed_sat.doppler_var_hz2 - doppler_uncertainty_hz.powi(2)).abs() <= 1.0e-12,
        "{observed_sat:?}"
    );
}

#[test]
fn observations_use_tracking_uncertainty_for_weaker_cn0() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let mut strong_epoch = make_observation_ready_epoch(30, &config, 70);
    strong_epoch.cn0_dbhz = 48.0;
    set_code_phase_uncertainty(&mut strong_epoch, 0.04);
    let mut weak_epoch = make_observation_ready_epoch(31, &config, 70);
    weak_epoch.cn0_dbhz = 28.0;
    set_code_phase_uncertainty(&mut weak_epoch, 0.16);

    let strong = observations_from_tracking_results(&config, &[track_from_epoch(strong_epoch)], 10);
    let weak = observations_from_tracking_results(&config, &[track_from_epoch(weak_epoch)], 10);
    let strong_sat = strong.output[0].sats.first().expect("strong satellite");
    let weak_sat = weak.output[0].sats.first().expect("weak satellite");

    assert!(
        weak_sat.pseudorange_var_m2 > strong_sat.pseudorange_var_m2,
        "weaker C/N0 should inflate pseudorange variance: strong={strong_sat:?} weak={weak_sat:?}"
    );
}

#[test]
fn observations_use_tracking_uncertainty_for_integration_duration() {
    let short_config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        tracking_integration_ms: 1,
        ..ReceiverPipelineConfig::default()
    };
    let long_config =
        ReceiverPipelineConfig { tracking_integration_ms: 10, ..short_config.clone() };
    let mut short_epoch = make_observation_ready_epoch(32, &short_config, 70);
    short_epoch.cn0_dbhz = 40.0;
    set_code_phase_uncertainty(&mut short_epoch, 0.14);
    let mut long_epoch = make_observation_ready_epoch(32, &long_config, 70);
    long_epoch.cn0_dbhz = 40.0;
    set_code_phase_uncertainty(&mut long_epoch, 0.05);

    let short =
        observations_from_tracking_results(&short_config, &[track_from_epoch(short_epoch)], 10);
    let long =
        observations_from_tracking_results(&long_config, &[track_from_epoch(long_epoch)], 10);
    let short_sat = short.output[0].sats.first().expect("short integration satellite");
    let long_sat = long.output[0].sats.first().expect("long integration satellite");

    assert!(
        long_sat.pseudorange_var_m2 < short_sat.pseudorange_var_m2,
        "longer coherent integration should tighten pseudorange variance: short={short_sat:?} long={long_sat:?}"
    );
}

#[test]
fn observations_use_tracking_uncertainty_when_dll_lock_is_lost() {
    let config = ReceiverPipelineConfig::default();
    let mut locked_epoch = make_observation_ready_epoch(33, &config, 70);
    set_code_phase_uncertainty(&mut locked_epoch, 0.05);
    let mut unlocked_epoch = make_observation_ready_epoch(34, &config, 70);
    unlocked_epoch.dll_lock = false;
    set_code_phase_uncertainty(&mut unlocked_epoch, 0.18);

    let locked = observations_from_tracking_results(&config, &[track_from_epoch(locked_epoch)], 10);
    let unlocked =
        observations_from_tracking_results(&config, &[track_from_epoch(unlocked_epoch)], 10);
    let locked_sat = locked.output[0].sats.first().expect("locked satellite");
    let unlocked_sat = unlocked.output[0].sats.first().expect("unlocked satellite");

    assert!(
        unlocked_sat.pseudorange_var_m2 > locked_sat.pseudorange_var_m2,
        "loss of DLL lock should inflate pseudorange variance: locked={locked_sat:?} unlocked={unlocked_sat:?}"
    );
}

#[test]
fn observations_use_tracking_uncertainty_for_low_lock_quality() {
    let config = ReceiverPipelineConfig::default();
    let mut locked_epoch = make_observation_ready_epoch(35, &config, 70);
    set_code_phase_uncertainty(&mut locked_epoch, 0.05);
    let mut guarded_epoch = make_observation_ready_epoch(36, &config, 70);
    guarded_epoch.anti_false_lock = true;
    set_code_phase_uncertainty(&mut guarded_epoch, 0.20);

    let locked = observations_from_tracking_results(&config, &[track_from_epoch(locked_epoch)], 10);
    let guarded =
        observations_from_tracking_results(&config, &[track_from_epoch(guarded_epoch)], 10);
    let locked_sat = locked.output[0].sats.first().expect("locked satellite");
    let guarded_sat = guarded.output[0].sats.first().expect("guarded satellite");

    assert!(
        guarded_sat.pseudorange_var_m2 > locked_sat.pseudorange_var_m2,
        "lower lock quality should inflate pseudorange variance: locked={locked_sat:?} guarded={guarded_sat:?}"
    );
}
