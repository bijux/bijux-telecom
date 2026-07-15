use super::*;

#[test]
fn run_fft_returns_deferred_result_for_each_satellite_when_frame_is_too_short() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        acquisition_integration_ms: 2,
        acquisition_noncoherent: 1,
        ..ReceiverPipelineConfig::default()
    };
    let samples_per_code =
        samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length);
    let frame = SamplesFrame::new(
        SampleTime { sample_index: 0, sample_rate_hz: config.sampling_freq_hz },
        Seconds(1.0 / config.sampling_freq_hz),
        (0..samples_per_code)
            .map(|idx| if idx % 2 == 0 { Complex::new(1.0, 0.0) } else { Complex::new(-1.0, 0.0) })
            .collect(),
    );
    let sats = vec![
        SatId { constellation: Constellation::Gps, prn: 3 },
        SatId { constellation: Constellation::Gps, prn: 7 },
    ];
    let acquisition = Acquisition::new(config, ReceiverRuntime::default());

    let run = acquisition.run_fft_topn_with_explain(&frame, &sats, 1, 2, 1);

    assert_eq!(run.results.len(), sats.len());
    assert_eq!(run.explains.len(), sats.len());
    for (idx, sat) in sats.iter().enumerate() {
        let result = run.results[idx].first().expect("placeholder result");
        assert_eq!(result.sat, *sat);
        assert_eq!(result.hypothesis.to_string(), AcqHypothesis::Deferred.to_string());
        assert_eq!(result.code_phase_samples, 0);
        assert_eq!(result.peak_mean_ratio, 0.0);
        assert_eq!(
            result.explain_selection_reason.as_deref(),
            Some("insufficient_frame: acquisition requires 8184 samples but received 4092"),
        );

        let explain = &run.explains[idx];
        assert_eq!(explain.sat, *sat);
        assert_eq!(explain.selected_reason, "insufficient_frame");
        assert_eq!(explain.candidate_count, 1);
    }
}

#[test]
fn insufficient_frame_preserves_explicit_signal_code() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 10_230_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 10_230_000.0,
        code_length: 10_230,
        ..ReceiverPipelineConfig::default()
    };
    let frame = SamplesFrame::new(
        SampleTime { sample_index: 0, sample_rate_hz: config.sampling_freq_hz },
        Seconds(1.0 / config.sampling_freq_hz),
        (0..1023)
            .map(|idx| if idx % 2 == 0 { Complex::new(1.0, 0.0) } else { Complex::new(-1.0, 0.0) })
            .collect(),
    );
    let acquisition = Acquisition::new(config, ReceiverRuntime::default());
    let request = AcqRequest {
        sat: SatId { constellation: Constellation::Galileo, prn: 11 },
        glonass_frequency_channel: None,
        signal_band: SignalBand::E5,
        signal_code: SignalCode::E5b,
        doppler_center_hz: 0.0,
        doppler_rate_center_hz_per_s: 0.0,
        doppler_rate_search_hz_per_s: 0,
        doppler_rate_step_hz_per_s: 250,
        expected_line_of_sight_doppler_hz: None,
        assistance_bounds: None,
        doppler_search_hz: 2_000,
        doppler_step_hz: 250,
        coherent_ms: 1,
        noncoherent: 1,
    };

    let run = acquisition.run_fft_topn_for_requests_with_explain(&frame, &[request], 1);

    let result = run.results[0].first().expect("deferred result");
    assert_eq!(result.signal_band, SignalBand::E5);
    assert_eq!(result.signal_code, SignalCode::E5b);
    assert_eq!(result.hypothesis.to_string(), AcqHypothesis::Deferred.to_string());
    assert_eq!(run.explains[0].selected_reason, "insufficient_frame");
}
