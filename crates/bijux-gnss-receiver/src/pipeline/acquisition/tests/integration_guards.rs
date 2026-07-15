use super::*;

#[test]
fn acquisition_rejects_unsupported_coherent_integration_lengths() {
    let config = ReceiverPipelineConfig::default();
    let sat = SatId { constellation: Constellation::Gps, prn: 1 };
    let frame = SamplesFrame::new(
        SampleTime { sample_index: 0, sample_rate_hz: config.sampling_freq_hz },
        Seconds(1.0 / config.sampling_freq_hz),
        (0..16)
            .map(|idx| if idx % 2 == 0 { Complex::new(1.0, 0.0) } else { Complex::new(-1.0, 0.0) })
            .collect(),
    );
    let acquisition = Acquisition::new(config, ReceiverRuntime::default());

    let run = acquisition.run_fft_topn_with_explain(&frame, &[sat], 1, 3, 1);

    let result = run.results.first().and_then(|rows| rows.first()).expect("result");
    let explain = run.explains.first().expect("explain");
    assert_eq!(result.hypothesis.to_string(), AcqHypothesis::Deferred.to_string());
    assert_eq!(explain.selected_reason, "unsupported_coherent_integration_ms");
    assert_eq!(
        result.explain_selection_reason.as_deref(),
        Some(
            "unsupported_coherent_integration_ms: acquisition coherent integration must be one of [1, 2, 5, 10, 20] ms but received 3 ms"
        )
    );
}

#[test]
fn unsupported_coherent_integration_preserves_explicit_signal_code() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 10_230_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 10_230_000.0,
        code_length: 10_230,
        ..ReceiverPipelineConfig::default()
    };
    let sat = SatId { constellation: Constellation::Gps, prn: 7 };
    let frame = SamplesFrame::new(
        SampleTime { sample_index: 0, sample_rate_hz: config.sampling_freq_hz },
        Seconds(1.0 / config.sampling_freq_hz),
        (0..32)
            .map(|idx| if idx % 2 == 0 { Complex::new(1.0, 0.0) } else { Complex::new(-1.0, 0.0) })
            .collect(),
    );
    let acquisition = Acquisition::new(config, ReceiverRuntime::default());
    let request = AcqRequest {
        sat,
        glonass_frequency_channel: None,
        signal_band: SignalBand::L5,
        signal_code: SignalCode::L5Q,
        doppler_center_hz: 0.0,
        doppler_rate_center_hz_per_s: 0.0,
        doppler_rate_search_hz_per_s: 0,
        doppler_rate_step_hz_per_s: 250,
        expected_line_of_sight_doppler_hz: None,
        assistance_bounds: None,
        doppler_search_hz: 2_000,
        doppler_step_hz: 250,
        coherent_ms: 3,
        noncoherent: 1,
    };

    let run = acquisition.run_fft_topn_for_requests_with_explain(&frame, &[request], 1);

    let result = run.results[0].first().expect("deferred result");
    assert_eq!(result.signal_band, SignalBand::L5);
    assert_eq!(result.signal_code, SignalCode::L5Q);
    assert_eq!(result.hypothesis.to_string(), AcqHypothesis::Deferred.to_string());
    assert_eq!(run.explains[0].selected_reason, "unsupported_coherent_integration_ms");
}
