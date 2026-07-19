use super::*;

#[test]
fn pseudorange_resolver_recovers_integer_code_period_ambiguity_from_transmit_time() {
    let receive_gps_time = GpsTime { week: 2200, tow_s: 345_600.250 };
    let code_period_s = 0.001;
    let code_delay_s = 0.000_375;
    let expected_periods = 72;
    let decoded_transmit_time =
        receive_gps_time.offset_seconds(-(expected_periods as f64 * code_period_s + code_delay_s));

    let resolved = resolve_pseudorange_from_transmit_time(
        receive_gps_time,
        decoded_transmit_time,
        code_delay_s,
        code_period_s,
    )
    .expect("resolved pseudorange timing");

    assert_eq!(resolved.integer_code_periods, expected_periods);
    assert!((resolved.code_delay_s.0 - code_delay_s).abs() <= 1.0e-12);
    assert!(
        (resolved.signal_travel_time_s.0
            - (expected_periods as f64 * code_period_s + code_delay_s))
            .abs()
            <= 1.0e-12
    );
    assert!((resolved.transmit_gps_time.tow_s - decoded_transmit_time.tow_s).abs() <= 1.0e-12);
}

#[test]
fn integer_code_period_solver_recovers_joint_vector_from_truth() {
    let receive_gps_time = GpsTime { week: 2200, tow_s: 345_600.250 };
    let code_period_s = 0.001;
    let first_delay_s = 0.000_375;
    let second_delay_s = 0.000_640;
    let first_periods = 72;
    let second_periods = 80;
    let first_signal_id = SigId {
        sat: SatId { constellation: Constellation::Gps, prn: 3 },
        band: SignalBand::L1,
        code: SignalCode::Ca,
    };
    let second_signal_id = SigId {
        sat: SatId { constellation: Constellation::Gps, prn: 9 },
        band: SignalBand::L1,
        code: SignalCode::Ca,
    };
    let inputs = vec![
        CodePeriodAmbiguityInput {
            signal_id: second_signal_id,
            receive_gps_time,
            decoded_transmit_gps_time: receive_gps_time
                .offset_seconds(-(second_periods as f64 * code_period_s + second_delay_s)),
            code_period_s,
            code_delay_s: second_delay_s,
        },
        CodePeriodAmbiguityInput {
            signal_id: first_signal_id,
            receive_gps_time,
            decoded_transmit_gps_time: receive_gps_time
                .offset_seconds(-(first_periods as f64 * code_period_s + first_delay_s)),
            code_period_s,
            code_delay_s: first_delay_s,
        },
    ];

    let solution =
        resolve_integer_code_period_ambiguities(&inputs).expect("unique ambiguity solution");

    assert!(solution.common_receiver_clock_bias_s.0.abs() <= CODE_PERIOD_AMBIGUITY_EPS_S);
    assert_eq!(solution.satellites[0].signal_id, first_signal_id);
    assert_eq!(solution.satellites[0].integer_code_periods, first_periods);
    assert_eq!(solution.satellites[1].signal_id, second_signal_id);
    assert_eq!(solution.satellites[1].integer_code_periods, second_periods);
}

#[test]
fn integer_code_period_solver_estimates_common_receiver_clock_bias() {
    let receive_gps_time = GpsTime { week: 2200, tow_s: 345_600.250 };
    let code_period_s = 0.001;
    let common_clock_bias_s = 0.000_120;
    let first_signal_id = SigId {
        sat: SatId { constellation: Constellation::Gps, prn: 5 },
        band: SignalBand::L1,
        code: SignalCode::Ca,
    };
    let second_signal_id = SigId {
        sat: SatId { constellation: Constellation::Gps, prn: 11 },
        band: SignalBand::L1,
        code: SignalCode::Ca,
    };
    let inputs = vec![
        CodePeriodAmbiguityInput {
            signal_id: first_signal_id,
            receive_gps_time,
            decoded_transmit_gps_time: receive_gps_time
                .offset_seconds(-(68.0 * code_period_s + 0.000_200 + common_clock_bias_s)),
            code_period_s,
            code_delay_s: 0.000_200,
        },
        CodePeriodAmbiguityInput {
            signal_id: second_signal_id,
            receive_gps_time,
            decoded_transmit_gps_time: receive_gps_time
                .offset_seconds(-(84.0 * code_period_s + 0.000_730 + common_clock_bias_s)),
            code_period_s,
            code_delay_s: 0.000_730,
        },
    ];

    let solution = resolve_integer_code_period_ambiguities(&inputs).expect("clock-biased solution");

    assert!(
        (solution.common_receiver_clock_bias_s.0 - common_clock_bias_s).abs()
            <= CODE_PERIOD_AMBIGUITY_EPS_S
    );
    assert_eq!(solution.satellites[0].integer_code_periods, 68);
    assert_eq!(solution.satellites[1].integer_code_periods, 84);
}

#[test]
fn integer_code_period_solver_refuses_non_unique_boundary() {
    let receive_gps_time = GpsTime { week: 2200, tow_s: 345_600.250 };
    let code_period_s = 0.001;
    let delay_s = 0.000_250;
    let signal_id = SigId {
        sat: SatId { constellation: Constellation::Gps, prn: 4 },
        band: SignalBand::L1,
        code: SignalCode::Ca,
    };
    let decoded_transmit_gps_time =
        receive_gps_time.offset_seconds(-(72.5 * code_period_s + delay_s));
    let inputs = vec![CodePeriodAmbiguityInput {
        signal_id,
        receive_gps_time,
        decoded_transmit_gps_time,
        code_period_s,
        code_delay_s: delay_s,
    }];

    let refusal = resolve_integer_code_period_ambiguities(&inputs)
        .expect_err("half-period ambiguity must not be resolved");

    assert_eq!(refusal, CODE_PERIOD_AMBIGUITY_NON_UNIQUE);
}

#[test]
fn observations_resolve_absolute_pseudorange_from_decoded_transmit_time() {
    let signal = signal_spec_gps_l1_ca();
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: signal.code_rate_hz,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let receive_gps_time = GpsTime { week: 2200, tow_s: 345_600.250 };
    let whole_code_periods = 72;
    let aligned_code_phase_chips = 384.0;
    let code_delay_s = aligned_code_phase_chips / signal.code_rate_hz;
    let expected_signal_travel_time_s =
        whole_code_periods as f64 * (1023.0 / signal.code_rate_hz) + code_delay_s;
    let decoded_transmit_time = receive_gps_time.offset_seconds(-expected_signal_travel_time_s);
    let epoch_sample_index =
        ((receive_gps_time.tow_s - 345_600.0) * config.sampling_freq_hz).round() as u64;
    let epoch = TrackEpoch {
        sat: SatId { constellation: Constellation::Gps, prn: 7 },
        signal_band: SignalBand::L1,
        carrier_hz: Hertz(tracked_signal_center_hz(config.intermediate_freq_hz, signal)),
        code_rate_hz: Hertz(signal.code_rate_hz),
        code_phase_samples: Chips(test_tracking_code_phase_samples_for_signal(
            &config,
            signal,
            1023,
            aligned_code_phase_chips,
        )),
        transmit_time: Some(TrackingTransmitTime {
            transmit_gps_time: decoded_transmit_time,
            source: "decoded_lnav_how".to_string(),
        }),
        sample_index: epoch_sample_index,
        source_time: ReceiverSampleTrace::from_sample_index(
            epoch_sample_index,
            config.sampling_freq_hz,
        ),
        ..make_tracking_epoch_with_phase(
            7,
            &config,
            250,
            tracked_signal_center_hz(config.intermediate_freq_hz, signal),
            0.0,
        )
    };

    let report = observations_from_tracking_results_with_gps_anchor(
        &config,
        Some(GpsTime { week: 2200, tow_s: 345_600.0 }),
        &[track_from_epoch(epoch)],
        10,
    );
    let obs_sat = report.output[0].sats.first().expect("observation satellite");
    let timing = obs_sat.timing.expect("resolved signal timing");

    assert_eq!(obs_sat.metadata.pseudorange_model, "decoded_transmit_time_code_phase");
    assert_eq!(obs_sat.metadata.pseudorange_time_source, "decoded_lnav_how");
    assert_eq!(obs_sat.metadata.pseudorange_integer_code_periods, Some(whole_code_periods));
    assert_eq!(obs_sat.metadata.signal_delay_alignment_source, "");
    assert_eq!(obs_sat.metadata.observation_support_class, "supported");
    assert!(
        (obs_sat.pseudorange_m.0 - timing.signal_travel_time_s.0 * SPEED_OF_LIGHT_MPS).abs()
            <= 1.0e-6
    );
    assert!(
        (timing.signal_travel_time_s.0 - expected_signal_travel_time_s).abs()
            <= CODE_PERIOD_AMBIGUITY_EPS_S
    );
    assert!(
        (timing.transmit_gps_time.tow_s - decoded_transmit_time.tow_s).abs()
            <= CODE_PERIOD_AMBIGUITY_EPS_S
    );
}

#[test]
fn decoded_transmit_time_observation_applies_receiver_clock_bias_once() {
    let signal = signal_spec_gps_l1_ca();
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: signal.code_rate_hz,
        code_length: 1023,
        receiver_clock_bias_s: 0.000_200,
        receiver_clock_source: "known_receiver_clock".to_string(),
        ..ReceiverPipelineConfig::default()
    };
    let capture_start_gps_time = GpsTime { week: 2200, tow_s: 345_600.0 };
    let true_receive_gps_time = GpsTime { week: 2200, tow_s: 345_600.250 };
    let whole_code_periods = 72;
    let aligned_code_phase_chips = 384.0;
    let physical_signal_time_s = whole_code_periods as f64 * (1023.0 / signal.code_rate_hz)
        + aligned_code_phase_chips / signal.code_rate_hz;
    let epoch = gps_l1ca_decoded_time_epoch(
        7,
        &config,
        250,
        capture_start_gps_time,
        true_receive_gps_time,
        whole_code_periods as f64,
        aligned_code_phase_chips,
    );

    let report = observations_from_tracking_results_with_gps_anchor(
        &config,
        Some(capture_start_gps_time),
        &[track_from_epoch(epoch)],
        10,
    );
    let obs_sat = report.output[0].sats.first().expect("observation satellite");
    let timing = obs_sat.timing.expect("resolved signal timing");
    let expected_observed_time_s = physical_signal_time_s + config.receiver_clock_bias_s;

    assert_eq!(obs_sat.metadata.pseudorange_integer_code_periods, Some(whole_code_periods));
    assert_eq!(obs_sat.metadata.receiver_clock_source, "known_receiver_clock");
    assert!((timing.signal_travel_time_s.0 - expected_observed_time_s).abs() <= 2.5e-7);
    assert!(
        (obs_sat.pseudorange_m.0 - expected_observed_time_s * SPEED_OF_LIGHT_MPS).abs()
            <= CODE_PERIOD_AMBIGUITY_EPS_S * SPEED_OF_LIGHT_MPS
    );
}

#[test]
fn observations_apply_joint_integer_code_period_vector() {
    let signal = signal_spec_gps_l1_ca();
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: signal.code_rate_hz,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let capture_start_gps_time = GpsTime { week: 2200, tow_s: 345_600.0 };
    let receive_gps_time = GpsTime { week: 2200, tow_s: 345_600.250 };
    let first_epoch = gps_l1ca_decoded_time_epoch(
        3,
        &config,
        250,
        capture_start_gps_time,
        receive_gps_time,
        72.0,
        384.0,
    );
    let second_epoch = gps_l1ca_decoded_time_epoch(
        9,
        &config,
        250,
        capture_start_gps_time,
        receive_gps_time,
        80.0,
        640.0,
    );

    let report = observations_from_tracking_results_with_gps_anchor(
        &config,
        Some(capture_start_gps_time),
        &[track_from_epoch(second_epoch), track_from_epoch(first_epoch)],
        10,
    );
    let epoch = report.output.first().expect("grouped observation epoch");
    let first_sat = epoch.sats.iter().find(|sat| sat.signal_id.sat.prn == 3).expect("sat");
    let second_sat = epoch.sats.iter().find(|sat| sat.signal_id.sat.prn == 9).expect("sat");
    let first_expected_s = 72.0 * (1023.0 / signal.code_rate_hz) + 384.0 / signal.code_rate_hz;
    let second_expected_s = 80.0 * (1023.0 / signal.code_rate_hz) + 640.0 / signal.code_rate_hz;

    assert_eq!(epoch.decision, ObservationEpochDecision::Accepted);
    assert_eq!(first_sat.metadata.pseudorange_integer_code_periods, Some(72));
    assert_eq!(second_sat.metadata.pseudorange_integer_code_periods, Some(80));
    assert_eq!(first_sat.metadata.observation_support_class, "supported");
    assert_eq!(second_sat.metadata.observation_support_class, "supported");
    assert!(
        (first_sat.pseudorange_m.0 - first_expected_s * SPEED_OF_LIGHT_MPS).abs()
            <= CODE_PERIOD_AMBIGUITY_EPS_S * SPEED_OF_LIGHT_MPS
    );
    assert!(
        (second_sat.pseudorange_m.0 - second_expected_s * SPEED_OF_LIGHT_MPS).abs()
            <= CODE_PERIOD_AMBIGUITY_EPS_S * SPEED_OF_LIGHT_MPS
    );
    assert!(!report
        .events
        .iter()
        .any(|event| event.code == "OBS_INTEGER_CODE_PERIOD_AMBIGUITY_REFUSED"));
}

#[test]
fn observations_refuse_non_unique_integer_code_period_boundary() {
    let signal = signal_spec_gps_l1_ca();
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: signal.code_rate_hz,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let capture_start_gps_time = GpsTime { week: 2200, tow_s: 345_600.0 };
    let receive_gps_time = GpsTime { week: 2200, tow_s: 345_600.250 };
    let epoch = gps_l1ca_decoded_time_epoch(
        4,
        &config,
        250,
        capture_start_gps_time,
        receive_gps_time,
        72.5,
        256.0,
    );

    let report = observations_from_tracking_results_with_gps_anchor(
        &config,
        Some(capture_start_gps_time),
        &[track_from_epoch(epoch)],
        10,
    );
    let epoch = report.output.first().expect("observation epoch");
    let sat = epoch.sats.first().expect("observation satellite");

    assert_eq!(epoch.decision, ObservationEpochDecision::Rejected);
    assert_eq!(epoch.decision_reason.as_deref(), Some("inconsistent_observable"));
    assert_eq!(sat.observation_status, ObservationStatus::Inconsistent);
    assert!(sat
        .observation_reject_reasons
        .iter()
        .any(|reason| reason == CODE_PERIOD_AMBIGUITY_NON_UNIQUE));
    assert_eq!(sat.metadata.observation_support_class, "unsupported");
    assert!(report.events.iter().any(|event| {
        event.code == "OBS_INTEGER_CODE_PERIOD_AMBIGUITY_REFUSED"
            && event
                .context
                .iter()
                .any(|(key, value)| key == "reason" && value == CODE_PERIOD_AMBIGUITY_NON_UNIQUE)
            && event.context.iter().any(|(key, value)| key == "signals" && value == "Gps:04:L1:Ca")
    }));
}
