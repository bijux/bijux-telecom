use super::*;

#[test]
fn observation_satellite_order_is_canonical() {
    let config = ReceiverPipelineConfig::default();
    let tracks_a = vec![make_track(2, &config), make_track(1, &config)];
    let tracks_b = vec![make_track(1, &config), make_track(2, &config)];

    let report_a = observations_from_tracking_results(&config, &tracks_a, 10);
    let report_b = observations_from_tracking_results(&config, &tracks_b, 10);

    let json_a = serde_json::to_string(&report_a.output).expect("serialize canonical report a");
    let json_b = serde_json::to_string(&report_b.output).expect("serialize canonical report b");
    assert_eq!(json_a, json_b);
}

#[test]
fn observations_from_tracking_derives_if_relative_doppler_from_carrier() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 2_000.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let doppler_hz = -250.0;
    let carrier_hz = crate::pipeline::doppler::carrier_hz_from_doppler_hz(
        config.intermediate_freq_hz,
        doppler_hz,
    );
    let epochs = vec![
        make_tracking_epoch(7, &config, 70, carrier_hz),
        make_tracking_epoch(7, &config, 71, carrier_hz),
    ];

    let (observations, diagnostics) = observations_from_tracking(&config, &epochs);

    assert!(diagnostics.is_empty(), "unexpected diagnostics: {diagnostics:?}");
    assert_eq!(observations.len(), 2);
    assert!(
        observations
            .iter()
            .all(|epoch| (epoch.sats[0].doppler_hz.0 - doppler_hz).abs() <= f64::EPSILON),
        "{observations:?}"
    );
}

#[test]
fn observations_from_tracking_results_preserve_distinct_satellite_dopplers() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 2_000.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let track_a = TrackingResult {
        sat: SatId { constellation: Constellation::Gps, prn: 3 },
        carrier_hz: 0.0,
        code_phase_samples: 0.0,
        acquisition_hypothesis: "deferred".to_string(),
        acquisition_score: 0.0,
        acquisition_code_phase_samples: 0,
        acquisition_carrier_hz: 0.0,
        acq_to_track_state: "deferred".to_string(),
        epochs: vec![make_tracking_epoch(
            3,
            &config,
            70,
            crate::pipeline::doppler::carrier_hz_from_doppler_hz(
                config.intermediate_freq_hz,
                125.0,
            ),
        )],
        transitions: Vec::new(),
    };
    let track_b = TrackingResult {
        sat: SatId { constellation: Constellation::Gps, prn: 8 },
        carrier_hz: 0.0,
        code_phase_samples: 0.0,
        acquisition_hypothesis: "deferred".to_string(),
        acquisition_score: 0.0,
        acquisition_code_phase_samples: 0,
        acquisition_carrier_hz: 0.0,
        acq_to_track_state: "deferred".to_string(),
        epochs: vec![make_tracking_epoch(
            8,
            &config,
            70,
            crate::pipeline::doppler::carrier_hz_from_doppler_hz(
                config.intermediate_freq_hz,
                -175.0,
            ),
        )],
        transitions: Vec::new(),
    };

    let report = observations_from_tracking_results(&config, &[track_a, track_b], 10);
    let epoch = report.output.first().expect("observation epoch");
    let dopplers =
        epoch.sats.iter().map(|sat| (sat.signal_id.sat.prn, sat.doppler_hz.0)).collect::<Vec<_>>();

    assert!(report.events.is_empty(), "unexpected diagnostics: {:?}", report.events);
    assert_eq!(dopplers, vec![(3, 125.0), (8, -175.0)]);
}
