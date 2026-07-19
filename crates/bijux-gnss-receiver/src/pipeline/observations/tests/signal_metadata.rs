use super::*;

#[test]
fn observations_preserve_tracking_cn0_on_accepted_rows() {
    let config = ReceiverPipelineConfig::default();
    let sat = SatId { constellation: Constellation::Gps, prn: 10 };
    let expected_cn0_dbhz = 47.5;
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
            sample_index: epoch_sample_index(&config, 70),
            source_time: ReceiverSampleTrace::from_sample_index(
                epoch_sample_index(&config, 70),
                config.sampling_freq_hz,
            ),
            sat,
            lock: true,
            cn0_dbhz: expected_cn0_dbhz,
            pll_lock: true,
            dll_lock: true,
            fll_lock: true,
            lock_state: "tracking".to_string(),
            lock_state_reason: Some("stable_tracking".to_string()),
            tracking_uncertainty: Some(test_tracking_uncertainty()),
            ..TrackEpoch::default()
        }],
        transitions: Vec::new(),
    };

    let report = observations_from_tracking_results(&config, &[track], 10);
    let epoch = report.output.first().expect("observation epoch");
    let sat = epoch.sats.first().expect("observation satellite");

    assert_eq!(sat.observation_status, ObservationStatus::Accepted);
    assert!((sat.cn0_dbhz - expected_cn0_dbhz).abs() <= f64::EPSILON, "{sat:?}");
}

#[test]
fn observations_record_locked_observation_lock_state() {
    let config = ReceiverPipelineConfig::default();
    let epoch = make_observation_ready_epoch(14, &config, 70);
    let report = observations_from_tracking_results(&config, &[track_from_epoch(epoch)], 10);
    let sat = report.output[0].sats.first().expect("observation satellite");

    assert_eq!(sat.metadata.observation_lock_state, "locked");
    assert_eq!(sat.metadata.observation_lock_reason.as_deref(), Some("stable_tracking"));
    assert_eq!(sat.metadata.tracking_lock_state, sat.metadata.observation_lock_state);
}

#[test]
fn observations_record_degraded_observation_lock_state() {
    let config = ReceiverPipelineConfig::default();
    let mut epoch = make_observation_ready_epoch(15, &config, 70);
    epoch.lock_state = "degraded".to_string();
    epoch.lock_state_reason = Some("signal_fade".to_string());
    let report = observations_from_tracking_results(&config, &[track_from_epoch(epoch)], 10);
    let sat = report.output[0].sats.first().expect("observation satellite");

    assert_eq!(sat.metadata.observation_lock_state, "degraded");
    assert_eq!(sat.metadata.observation_lock_reason.as_deref(), Some("signal_fade"));
    assert_eq!(sat.metadata.tracking_lock_state, sat.metadata.observation_lock_state);
}

#[test]
fn observations_record_lost_observation_lock_state() {
    let config = ReceiverPipelineConfig::default();
    let mut epoch = make_observation_ready_epoch(16, &config, 70);
    epoch.lock = false;
    epoch.pll_lock = false;
    epoch.dll_lock = false;
    epoch.fll_lock = false;
    epoch.lock_state = "lost".to_string();
    epoch.lock_state_reason = Some("prompt_power_drop".to_string());
    let report = observations_from_tracking_results(&config, &[track_from_epoch(epoch)], 10);
    let sat = report.output[0].sats.first().expect("observation satellite");

    assert_eq!(sat.metadata.observation_lock_state, "lost");
    assert_eq!(sat.metadata.observation_lock_reason.as_deref(), Some("prompt_power_drop"));
    assert_eq!(sat.metadata.tracking_lock_state, sat.metadata.observation_lock_state);
}

#[test]
fn observations_record_reacquired_observation_lock_state() {
    let config = ReceiverPipelineConfig::default();
    let mut epoch = make_observation_ready_epoch(17, &config, 70);
    epoch.lock_state_reason = Some("reacquired".to_string());
    let report = observations_from_tracking_results(&config, &[track_from_epoch(epoch)], 10);
    let sat = report.output[0].sats.first().expect("observation satellite");

    assert_eq!(sat.metadata.observation_lock_state, "reacquired");
    assert_eq!(sat.metadata.observation_lock_reason.as_deref(), Some("reacquired"));
    assert_eq!(sat.metadata.tracking_lock_state, sat.metadata.observation_lock_state);
}

#[test]
fn observations_record_cycle_slip_observation_lock_state() {
    let config = ReceiverPipelineConfig::default();
    let mut epoch = make_observation_ready_epoch(18, &config, 70);
    epoch.lock = false;
    epoch.pll_lock = false;
    epoch.dll_lock = false;
    epoch.fll_lock = false;
    epoch.cycle_slip = true;
    epoch.cycle_slip_reason = Some("phase_jump".to_string());
    epoch.lock_state = "lost".to_string();
    epoch.lock_state_reason = Some("phase_jump".to_string());
    let report = observations_from_tracking_results(&config, &[track_from_epoch(epoch)], 10);
    let sat = report.output[0].sats.first().expect("observation satellite");

    assert_eq!(sat.metadata.observation_lock_state, "cycle_slip");
    assert_eq!(sat.metadata.observation_lock_reason.as_deref(), Some("phase_jump"));
    assert_eq!(sat.metadata.tracking_lock_state, sat.metadata.observation_lock_state);
}

#[test]
fn observations_declare_if_relative_doppler_contract() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 2_000.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let sat = SatId { constellation: Constellation::Gps, prn: 10 };
    let expected_doppler_hz = -250.0;
    let carrier_hz = crate::pipeline::doppler::carrier_hz_from_doppler_hz(
        config.intermediate_freq_hz,
        expected_doppler_hz,
    );
    let track = TrackingResult {
        sat,
        carrier_hz,
        code_phase_samples: 0.0,
        acquisition_hypothesis: "accepted".to_string(),
        acquisition_score: 1.0,
        acquisition_code_phase_samples: 0,
        acquisition_carrier_hz: carrier_hz,
        acq_to_track_state: "accepted".to_string(),
        epochs: vec![make_tracking_epoch(10, &config, 70, carrier_hz)],
        transitions: Vec::new(),
    };

    let report = observations_from_tracking_results(&config, &[track], 10);
    let epoch = report.output.first().expect("observation epoch");
    let sat = epoch.sats.first().expect("observation satellite");

    assert_eq!(
        sat.metadata.doppler_model,
        bijux_gnss_core::api::OBSERVATION_DOPPLER_MODEL_TRACKED_CARRIER_IF_OFFSET
    );
    assert!((sat.doppler_hz.0 - expected_doppler_hz).abs() <= f64::EPSILON, "{sat:?}");
}

#[test]
fn observations_emit_glonass_l1_signal_identity_and_fdma_relative_doppler() {
    let channel = GlonassFrequencyChannel::new(-4).expect("channel -4 must be valid");
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 2_044_000.0,
        intermediate_freq_hz: GPS_L1_CA_CARRIER_HZ.value() - glonass_l1_carrier_hz(channel).value(),
        code_freq_basis_hz: 511_000.0,
        code_length: 511,
        ..ReceiverPipelineConfig::default()
    };
    let sat = SatId { constellation: Constellation::Glonass, prn: 8 };
    let expected_doppler_hz = 125.0;
    let carrier_hz = crate::pipeline::doppler::carrier_hz_from_doppler_hz(0.0, expected_doppler_hz);
    let mut epoch = make_tracking_epoch(8, &config, 70, carrier_hz);
    epoch.sat = sat;
    epoch.signal_band = SignalBand::L1;
    epoch.glonass_frequency_channel = Some(channel);
    epoch.code_rate_hz = Hertz(511_000.0);
    let track = TrackingResult {
        sat,
        carrier_hz,
        code_phase_samples: 0.0,
        acquisition_hypothesis: "accepted".to_string(),
        acquisition_score: 1.0,
        acquisition_code_phase_samples: 0,
        acquisition_carrier_hz: carrier_hz,
        acq_to_track_state: "accepted".to_string(),
        epochs: vec![epoch],
        transitions: Vec::new(),
    };

    let report = observations_from_tracking_results(&config, &[track], 10);
    let epoch = report.output.first().expect("observation epoch");
    let obs_sat = epoch.sats.first().expect("observation satellite");

    assert_eq!(obs_sat.signal_id.sat, sat);
    assert_eq!(obs_sat.signal_id.band, SignalBand::L1);
    assert_eq!(obs_sat.signal_id.code, SignalCode::Unknown);
    assert_eq!(obs_sat.metadata.signal.code, SignalCode::Unknown);
    assert_eq!(obs_sat.metadata.signal.carrier_hz.value(), glonass_l1_carrier_hz(channel).value());
    assert!((obs_sat.doppler_hz.0 - expected_doppler_hz).abs() <= f64::EPSILON, "{obs_sat:?}");
}

#[test]
fn observations_emit_gps_l2c_signal_identity_and_civil_code_pseudorange() {
    let signal = signal_spec_gps_l2c();
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 5_115_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: signal.code_rate_hz,
        code_length: 10230,
        ..ReceiverPipelineConfig::default()
    };
    let whole_code_periods = 4;
    let aligned_code_phase_chips = 768.0;
    let expected_pseudorange_m = aligned_pseudorange_m_for_signal(
        signal,
        10230,
        whole_code_periods,
        aligned_code_phase_chips,
    );
    let epoch = TrackEpoch {
        sat: SatId { constellation: Constellation::Gps, prn: 11 },
        signal_band: SignalBand::L2,
        carrier_hz: Hertz(tracked_signal_center_hz(config.intermediate_freq_hz, signal)),
        code_rate_hz: Hertz(signal.code_rate_hz),
        code_phase_samples: Chips(test_tracking_code_phase_samples_for_signal(
            &config,
            signal,
            10230,
            aligned_code_phase_chips,
        )),
        signal_delay_alignment: Some(SignalDelayAlignment {
            whole_code_periods,
            sample_delay_samples: 0,
            source: "synthetic_truth".to_string(),
        }),
        ..make_tracking_epoch_with_phase(
            11,
            &config,
            70,
            tracked_signal_center_hz(config.intermediate_freq_hz, signal),
            0.0,
        )
    };
    let report = observations_from_tracking_results(&config, &[track_from_epoch(epoch)], 10);
    let obs_sat = report.output[0].sats.first().expect("observation satellite");

    assert_eq!(obs_sat.signal_id.band, SignalBand::L2);
    assert_eq!(obs_sat.signal_id.code, SignalCode::L2C);
    assert_eq!(obs_sat.metadata.signal, signal);
    assert!((obs_sat.metadata.signal.code_rate_hz - 511_500.0).abs() <= f64::EPSILON);
    assert_eq!(obs_sat.metadata.pseudorange_model, "tracked_code_phase_alignment");
    assert!((obs_sat.pseudorange_m.0 - expected_pseudorange_m).abs() <= 1.0e-6, "{obs_sat:?}");
}

#[test]
fn observations_subtract_known_sample_delay_from_aligned_pseudorange() {
    let signal = signal_spec_gps_l1_ca();
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: signal.code_rate_hz,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let whole_code_periods = 12;
    let aligned_code_phase_chips = 512.0;
    let sample_delay_samples = 20;
    let samples_per_chip = config.sampling_freq_hz / signal.code_rate_hz;
    let delayed_code_phase_chips =
        aligned_code_phase_chips + sample_delay_samples as f64 / samples_per_chip;
    let expected_pseudorange_m = aligned_pseudorange_m_for_signal(
        signal,
        1023,
        whole_code_periods,
        aligned_code_phase_chips,
    );
    let epoch = TrackEpoch {
        sat: SatId { constellation: Constellation::Gps, prn: 7 },
        signal_band: SignalBand::L1,
        carrier_hz: Hertz(tracked_signal_center_hz(config.intermediate_freq_hz, signal)),
        code_rate_hz: Hertz(signal.code_rate_hz),
        code_phase_samples: Chips(test_tracking_code_phase_samples_for_signal(
            &config,
            signal,
            1023,
            delayed_code_phase_chips,
        )),
        signal_delay_alignment: Some(SignalDelayAlignment {
            whole_code_periods,
            sample_delay_samples,
            source: "synthetic_truth".to_string(),
        }),
        ..make_tracking_epoch_with_phase(
            7,
            &config,
            70,
            tracked_signal_center_hz(config.intermediate_freq_hz, signal),
            0.0,
        )
    };

    let report = observations_from_tracking_results(&config, &[track_from_epoch(epoch)], 10);
    let obs_sat = report.output[0].sats.first().expect("observation satellite");

    assert_eq!(obs_sat.metadata.pseudorange_model, "tracked_code_phase_alignment");
    assert!((obs_sat.pseudorange_m.0 - expected_pseudorange_m).abs() <= 1.0e-6, "{obs_sat:?}");
}

#[test]
fn observations_emit_gps_l5_signal_identity_and_aligned_pseudorange() {
    let signal = signal_spec_gps_l5();
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 10_230_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: signal.code_rate_hz,
        code_length: 10230,
        ..ReceiverPipelineConfig::default()
    };
    let whole_code_periods = 8;
    let aligned_code_phase_chips = 2_048.0;
    let expected_pseudorange_m = aligned_pseudorange_m_for_signal(
        signal,
        10230,
        whole_code_periods,
        aligned_code_phase_chips,
    );
    let epoch = TrackEpoch {
        sat: SatId { constellation: Constellation::Gps, prn: 12 },
        signal_band: SignalBand::L5,
        carrier_hz: Hertz(tracked_signal_center_hz(config.intermediate_freq_hz, signal)),
        code_rate_hz: Hertz(signal.code_rate_hz),
        code_phase_samples: Chips(test_tracking_code_phase_samples_for_signal(
            &config,
            signal,
            10230,
            aligned_code_phase_chips,
        )),
        signal_delay_alignment: Some(SignalDelayAlignment {
            whole_code_periods,
            sample_delay_samples: 0,
            source: "synthetic_truth".to_string(),
        }),
        ..make_tracking_epoch_with_phase(
            12,
            &config,
            70,
            tracked_signal_center_hz(config.intermediate_freq_hz, signal),
            0.0,
        )
    };
    let report = observations_from_tracking_results(&config, &[track_from_epoch(epoch)], 10);
    let obs_sat = report.output[0].sats.first().expect("observation satellite");

    assert_eq!(obs_sat.signal_id.band, SignalBand::L5);
    assert_eq!(obs_sat.signal_id.code, SignalCode::L5I);
    assert_eq!(obs_sat.metadata.signal, signal);
    assert!((obs_sat.metadata.signal.code_rate_hz - 10_230_000.0).abs() <= f64::EPSILON);
    assert_eq!(obs_sat.metadata.pseudorange_model, "tracked_code_phase_alignment");
    assert!((obs_sat.pseudorange_m.0 - expected_pseudorange_m).abs() <= 1.0e-6, "{obs_sat:?}");
}

#[test]
fn observations_preserve_gps_l5_q_signal_identity_and_aligned_pseudorange() {
    let signal = signal_spec_gps_l5_q();
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 10_230_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: signal.code_rate_hz,
        code_length: 10230,
        ..ReceiverPipelineConfig::default()
    };
    let whole_code_periods = 8;
    let aligned_code_phase_chips = 2_048.0;
    let expected_pseudorange_m = aligned_pseudorange_m_for_signal(
        signal,
        10230,
        whole_code_periods,
        aligned_code_phase_chips,
    );
    let epoch = TrackEpoch {
        sat: SatId { constellation: Constellation::Gps, prn: 12 },
        signal_band: SignalBand::L5,
        signal_code: SignalCode::L5Q,
        carrier_hz: Hertz(tracked_signal_center_hz(config.intermediate_freq_hz, signal)),
        code_rate_hz: Hertz(signal.code_rate_hz),
        code_phase_samples: Chips(test_tracking_code_phase_samples_for_signal(
            &config,
            signal,
            10230,
            aligned_code_phase_chips,
        )),
        signal_delay_alignment: Some(SignalDelayAlignment {
            whole_code_periods,
            sample_delay_samples: 0,
            source: "synthetic_truth".to_string(),
        }),
        ..make_tracking_epoch_with_phase(
            12,
            &config,
            70,
            tracked_signal_center_hz(config.intermediate_freq_hz, signal),
            0.0,
        )
    };
    let report = observations_from_tracking_results(&config, &[track_from_epoch(epoch)], 10);
    let obs_sat = report.output[0].sats.first().expect("observation satellite");

    assert_eq!(obs_sat.signal_id.band, SignalBand::L5);
    assert_eq!(obs_sat.signal_id.code, SignalCode::L5Q);
    assert_eq!(obs_sat.metadata.signal, signal);
    assert!((obs_sat.metadata.signal.code_rate_hz - 10_230_000.0).abs() <= f64::EPSILON);
    assert_eq!(obs_sat.metadata.pseudorange_model, "tracked_code_phase_alignment");
    assert!((obs_sat.pseudorange_m.0 - expected_pseudorange_m).abs() <= 1.0e-6, "{obs_sat:?}");
}

#[test]
fn observations_emit_galileo_e5_signal_identity_and_aligned_pseudorange() {
    let signal = signal_spec_galileo_e5a();
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 10_230_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: signal.code_rate_hz,
        code_length: 10230,
        ..ReceiverPipelineConfig::default()
    };
    let whole_code_periods = 6;
    let aligned_code_phase_chips = 1_536.0;
    let expected_pseudorange_m = aligned_pseudorange_m_for_signal(
        signal,
        10230,
        whole_code_periods,
        aligned_code_phase_chips,
    );
    let expected_phase_cycles = signal_meters_to_cycles(Meters(expected_pseudorange_m), signal).0;
    let epoch = TrackEpoch {
        sat: SatId { constellation: Constellation::Galileo, prn: 11 },
        signal_band: SignalBand::E5,
        carrier_hz: Hertz(tracked_signal_center_hz(config.intermediate_freq_hz, signal)),
        code_rate_hz: Hertz(signal.code_rate_hz),
        code_phase_samples: Chips(test_tracking_code_phase_samples_for_signal(
            &config,
            signal,
            10230,
            aligned_code_phase_chips,
        )),
        signal_delay_alignment: Some(SignalDelayAlignment {
            whole_code_periods,
            sample_delay_samples: 0,
            source: "synthetic_truth".to_string(),
        }),
        ..make_tracking_epoch_with_phase(
            11,
            &config,
            70,
            tracked_signal_center_hz(config.intermediate_freq_hz, signal),
            expected_phase_cycles,
        )
    };
    let report = observations_from_tracking_results(&config, &[track_from_epoch(epoch)], 10);
    let obs_sat = report.output[0].sats.first().expect("observation satellite");

    assert_eq!(obs_sat.signal_id.band, SignalBand::E5);
    assert_eq!(obs_sat.signal_id.code, SignalCode::E5a);
    assert_eq!(obs_sat.metadata.signal, signal);
    assert!((obs_sat.metadata.signal.code_rate_hz - 10_230_000.0).abs() <= f64::EPSILON);
    assert_eq!(obs_sat.metadata.pseudorange_model, "tracked_code_phase_alignment");
    assert!((obs_sat.pseudorange_m.0 - expected_pseudorange_m).abs() <= 1.0e-6, "{obs_sat:?}");
    assert!(
        (signal_cycles_to_meters(obs_sat.carrier_phase_cycles, obs_sat.metadata.signal).0
            - expected_pseudorange_m)
            .abs()
            <= 1.0e-6,
        "{obs_sat:?}"
    );
}

#[test]
fn observations_emit_beidou_b2_signal_identity_and_aligned_pseudorange() {
    let signal = signal_spec_beidou_b2i();
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 2_046_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: signal.code_rate_hz,
        code_length: 2046,
        ..ReceiverPipelineConfig::default()
    };
    let whole_code_periods = 8;
    let aligned_code_phase_chips = 768.0;
    let expected_pseudorange_m = aligned_pseudorange_m_for_signal(
        signal,
        2046,
        whole_code_periods,
        aligned_code_phase_chips,
    );
    let expected_phase_cycles = signal_meters_to_cycles(Meters(expected_pseudorange_m), signal).0;
    let epoch = TrackEpoch {
        sat: SatId { constellation: Constellation::Beidou, prn: 11 },
        signal_band: SignalBand::B2,
        carrier_hz: Hertz(tracked_signal_center_hz(config.intermediate_freq_hz, signal)),
        code_rate_hz: Hertz(signal.code_rate_hz),
        code_phase_samples: Chips(test_tracking_code_phase_samples_for_signal(
            &config,
            signal,
            2046,
            aligned_code_phase_chips,
        )),
        signal_delay_alignment: Some(SignalDelayAlignment {
            whole_code_periods,
            sample_delay_samples: 0,
            source: "synthetic_truth".to_string(),
        }),
        ..make_tracking_epoch_with_phase(
            11,
            &config,
            70,
            tracked_signal_center_hz(config.intermediate_freq_hz, signal),
            expected_phase_cycles,
        )
    };
    let report = observations_from_tracking_results(&config, &[track_from_epoch(epoch)], 10);
    let obs_sat = report.output[0].sats.first().expect("observation satellite");

    assert_eq!(obs_sat.signal_id.band, SignalBand::B2);
    assert_eq!(obs_sat.signal_id.code, SignalCode::B2I);
    assert_eq!(obs_sat.metadata.signal, signal);
    assert!((obs_sat.metadata.signal.code_rate_hz - 2_046_000.0).abs() <= f64::EPSILON);
    assert_eq!(obs_sat.metadata.pseudorange_model, "tracked_code_phase_alignment");
    assert!((obs_sat.pseudorange_m.0 - expected_pseudorange_m).abs() <= 1.0e-6, "{obs_sat:?}");
    assert!(
        (signal_cycles_to_meters(obs_sat.carrier_phase_cycles, obs_sat.metadata.signal).0
            - expected_pseudorange_m)
            .abs()
            <= 1.0e-6,
        "{obs_sat:?}"
    );
}

#[test]
fn observation_metadata_sets_support_and_uncertainty_classes() {
    let config = ReceiverPipelineConfig::default();
    let sat = SatId { constellation: Constellation::Gps, prn: 7 };
    let mut weak = make_observation_ready_epoch(7, &config, 70);
    weak.cn0_dbhz = 20.0;
    let mut missing = make_observation_ready_epoch(8, &config, 70);
    missing.lock = false;
    missing.cn0_dbhz = 45.0;
    missing.lock_state = "lost".to_string();
    let tracks = vec![
        TrackingResult {
            sat,
            carrier_hz: 0.0,
            code_phase_samples: 0.0,
            acquisition_hypothesis: "deferred".to_string(),
            acquisition_score: 0.0,
            acquisition_code_phase_samples: 0,
            acquisition_carrier_hz: 0.0,
            acq_to_track_state: "deferred".to_string(),
            epochs: vec![weak],
            transitions: Vec::new(),
        },
        TrackingResult {
            sat: SatId { constellation: Constellation::Gps, prn: 8 },
            carrier_hz: 0.0,
            code_phase_samples: 0.0,
            acquisition_hypothesis: "deferred".to_string(),
            acquisition_score: 0.0,
            acquisition_code_phase_samples: 0,
            acquisition_carrier_hz: 0.0,
            acq_to_track_state: "deferred".to_string(),
            epochs: vec![missing],
            transitions: Vec::new(),
        },
    ];
    let report = observations_from_tracking_results(&config, &tracks, 10);
    let epoch = report.output.iter().find(|row| row.epoch_idx == 70).expect("epoch");
    let labels = epoch
        .sats
        .iter()
        .map(|sat| {
            (
                sat.metadata.observation_support_class.clone(),
                sat.metadata.observation_uncertainty_class.clone(),
            )
        })
        .collect::<Vec<_>>();
    assert!(labels
        .iter()
        .any(|(support, uncertainty)| support == "degraded" && uncertainty == "high"));
    assert!(labels
        .iter()
        .any(|(support, uncertainty)| support == "degraded" && uncertainty == "low"));
}
