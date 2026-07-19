use super::*;

#[test]
fn incremental_tracking_matches_single_frame_tracking() {
    let config = crate::engine::receiver_config::ReceiverPipelineConfig {
        sampling_freq_hz: 1_023_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 4,
        ..crate::engine::receiver_config::ReceiverPipelineConfig::default()
    };
    let runtime = crate::engine::runtime::ReceiverRuntime::default();
    let tracking = Tracking::new(config.clone(), runtime.clone());
    let sat = SatId { constellation: bijux_gnss_core::api::Constellation::Gps, prn: 3 };
    let duration_s = 0.012;
    let frame = crate::sim::synthetic::generate_l1_ca(
        &config,
        crate::sim::synthetic::SyntheticSignalParams {
            sat,
            glonass_frequency_channel: None,
            signal_band: bijux_gnss_core::api::SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Unknown,
            doppler_hz: 1_000.0,
            code_phase_chips: 10.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 48.0,
            navigation_data: false.into(),
        },
        7,
        duration_s,
    );
    let acquisition = bijux_gnss_core::api::AcqResult {
        sat,
        signal_band: bijux_gnss_core::api::SignalBand::L1,
        signal_code: bijux_gnss_core::api::SignalCode::Unknown,
        glonass_frequency_channel: None,
        source_time: bijux_gnss_core::api::ReceiverSampleTrace::from_sample_time(frame.t0),
        candidate_rank: 1,
        is_primary_candidate: true,
        doppler_hz: bijux_gnss_core::api::Hertz(1_000.0),
        doppler_rate_hz_per_s: 0.0,
        carrier_hz: bijux_gnss_core::api::Hertz(1_000.0),
        code_phase_samples: 10,
        peak: 10.0,
        second_peak: 2.0,
        mean: 1.0,
        peak_mean_ratio: 10.0,
        peak_second_ratio: 5.0,
        cn0_proxy: 48.0,
        score: 0.98,
        hypothesis: AcqHypothesis::Accepted,
        assumptions: None,
        evidence: Vec::new(),
        threshold_provenance: None,
        explain_selection_reason: None,
        doppler_refinement: None,
        code_phase_refinement: None,
        signal_delay_alignment: None,
        uncertainty: None,
    };

    let single = tracking.track_from_acquisition(&frame, &[acquisition.clone()]);
    let mut incremental = tracking.begin_incremental_tracking(&[acquisition]);
    for chunk in split_frame(&frame, 4 * 1_023) {
        tracking.track_incremental_frame(&mut incremental, &chunk);
    }
    let streamed = tracking.finish_incremental_tracking(incremental);

    assert_eq!(single.len(), streamed.len());
    assert_eq!(single[0].epochs.len(), streamed[0].epochs.len());
    let single_keys = single[0]
        .epochs
        .iter()
        .map(|epoch| {
            (
                epoch.epoch.index,
                epoch.sample_index,
                epoch.lock,
                epoch.lock_state.clone(),
                epoch.lock_state_reason.clone(),
            )
        })
        .collect::<Vec<_>>();
    let streamed_keys = streamed[0]
        .epochs
        .iter()
        .map(|epoch| {
            (
                epoch.epoch.index,
                epoch.sample_index,
                epoch.lock,
                epoch.lock_state.clone(),
                epoch.lock_state_reason.clone(),
            )
        })
        .collect::<Vec<_>>();
    assert_eq!(single_keys, streamed_keys);
}

#[test]
fn begin_incremental_tracking_prioritizes_stronger_trackable_acquisitions() {
    let config = crate::engine::receiver_config::ReceiverPipelineConfig {
        channels: 2,
        ..crate::engine::receiver_config::ReceiverPipelineConfig::default()
    };
    let tracking = Tracking::new(config, ReceiverRuntime::default());
    let acquisitions = vec![
        bijux_gnss_core::api::AcqResult {
            sat: SatId { constellation: Constellation::Gps, prn: 3 },
            signal_band: SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Unknown,
            glonass_frequency_channel: None,
            source_time: ReceiverSampleTrace::default(),
            candidate_rank: 1,
            is_primary_candidate: true,
            doppler_hz: Hertz(0.0),
            doppler_rate_hz_per_s: 0.0,
            carrier_hz: Hertz(0.0),
            code_phase_samples: 0,
            peak: 1.0,
            second_peak: 0.1,
            mean: 0.01,
            peak_mean_ratio: 6.0,
            peak_second_ratio: 2.0,
            cn0_proxy: 38.0,
            score: 1.4,
            hypothesis: AcqHypothesis::Ambiguous,
            assumptions: None,
            evidence: Vec::new(),
            threshold_provenance: None,
            explain_selection_reason: None,
            doppler_refinement: None,
            code_phase_refinement: None,
            signal_delay_alignment: None,
            uncertainty: None,
        },
        bijux_gnss_core::api::AcqResult {
            sat: SatId { constellation: Constellation::Gps, prn: 7 },
            signal_band: SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Unknown,
            glonass_frequency_channel: None,
            source_time: ReceiverSampleTrace::default(),
            candidate_rank: 1,
            is_primary_candidate: true,
            doppler_hz: Hertz(0.0),
            doppler_rate_hz_per_s: 0.0,
            carrier_hz: Hertz(0.0),
            code_phase_samples: 0,
            peak: 1.0,
            second_peak: 0.1,
            mean: 0.01,
            peak_mean_ratio: 8.0,
            peak_second_ratio: 3.0,
            cn0_proxy: 52.0,
            score: 5.0,
            hypothesis: AcqHypothesis::Accepted,
            assumptions: None,
            evidence: Vec::new(),
            threshold_provenance: None,
            explain_selection_reason: None,
            doppler_refinement: None,
            code_phase_refinement: None,
            signal_delay_alignment: None,
            uncertainty: None,
        },
        bijux_gnss_core::api::AcqResult {
            sat: SatId { constellation: Constellation::Gps, prn: 23 },
            signal_band: SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Unknown,
            glonass_frequency_channel: None,
            source_time: ReceiverSampleTrace::default(),
            candidate_rank: 1,
            is_primary_candidate: true,
            doppler_hz: Hertz(0.0),
            doppler_rate_hz_per_s: 0.0,
            carrier_hz: Hertz(0.0),
            code_phase_samples: 0,
            peak: 1.0,
            second_peak: 0.1,
            mean: 0.01,
            peak_mean_ratio: 9.0,
            peak_second_ratio: 3.5,
            cn0_proxy: 48.0,
            score: 3.5,
            hypothesis: AcqHypothesis::Ambiguous,
            assumptions: None,
            evidence: Vec::new(),
            threshold_provenance: None,
            explain_selection_reason: None,
            doppler_refinement: None,
            code_phase_refinement: None,
            signal_delay_alignment: None,
            uncertainty: None,
        },
    ];

    let incremental = tracking.begin_incremental_tracking(&acquisitions);
    let selected_prns =
        incremental.channels.iter().map(|channel| channel.sat.prn).collect::<Vec<_>>();

    assert_eq!(selected_prns, vec![7, 23]);
}

#[test]
fn begin_incremental_tracking_preserves_glonass_channel_and_carrier_seed() {
    let channel =
        bijux_gnss_core::api::GlonassFrequencyChannel::new(-4).expect("channel -4 must be valid");
    let carrier_hz = 2_048_125.0;
    let config = crate::engine::receiver_config::ReceiverPipelineConfig {
        code_freq_basis_hz: 511_000.0,
        code_length: 511,
        ..crate::engine::receiver_config::ReceiverPipelineConfig::default()
    };
    let tracking = Tracking::new(config, ReceiverRuntime::default());
    let acquisition = bijux_gnss_core::api::AcqResult {
        sat: SatId { constellation: Constellation::Glonass, prn: 8 },
        signal_band: SignalBand::L1,
        signal_code: bijux_gnss_core::api::SignalCode::Unknown,
        glonass_frequency_channel: Some(channel),
        source_time: ReceiverSampleTrace::default(),
        candidate_rank: 1,
        is_primary_candidate: true,
        doppler_hz: Hertz(125.0),
        doppler_rate_hz_per_s: 0.0,
        carrier_hz: Hertz(carrier_hz),
        code_phase_samples: 37,
        peak: 1.0,
        second_peak: 0.1,
        mean: 0.01,
        peak_mean_ratio: 8.0,
        peak_second_ratio: 3.0,
        cn0_proxy: 48.0,
        score: 5.0,
        hypothesis: AcqHypothesis::Accepted,
        assumptions: None,
        evidence: Vec::new(),
        threshold_provenance: None,
        explain_selection_reason: None,
        doppler_refinement: None,
        code_phase_refinement: None,
        signal_delay_alignment: None,
        uncertainty: None,
    };

    let incremental = tracking.begin_incremental_tracking(&[acquisition]);
    let channel_state = incremental.channels.first().expect("GLONASS tracking channel");

    assert_eq!(channel_state.signal_model.signal_band, SignalBand::L1);
    assert_eq!(channel_state.signal_model.glonass_frequency_channel, Some(channel));
    assert_eq!(channel_state.state.carrier_hz, carrier_hz);
    assert!((channel_state.signal_model.code_rate_hz - 511_000.0).abs() <= f64::EPSILON);
    assert_eq!(channel_state.signal_model.code_length, 511);
}

#[test]
fn begin_incremental_tracking_seeds_code_rate_from_carrier_aid() {
    let config = crate::engine::receiver_config::ReceiverPipelineConfig {
        intermediate_freq_hz: GPS_L1_CA_CARRIER_HZ.value()
            - signal_spec_gps_l5_i().carrier_hz.value(),
        code_freq_basis_hz: 10_230_000.0,
        code_length: 10_230,
        ..crate::engine::receiver_config::ReceiverPipelineConfig::default()
    };
    let tracking = Tracking::new(config, ReceiverRuntime::default());
    let acquisition = bijux_gnss_core::api::AcqResult {
        sat: SatId { constellation: Constellation::Gps, prn: 18 },
        signal_band: SignalBand::L5,
        signal_code: SignalCode::L5I,
        glonass_frequency_channel: None,
        source_time: ReceiverSampleTrace::default(),
        candidate_rank: 1,
        is_primary_candidate: true,
        doppler_hz: Hertz(1_500.0),
        doppler_rate_hz_per_s: 0.0,
        carrier_hz: Hertz(1_500.0),
        code_phase_samples: 37,
        peak: 1.0,
        second_peak: 0.1,
        mean: 0.01,
        peak_mean_ratio: 8.0,
        peak_second_ratio: 3.0,
        cn0_proxy: 48.0,
        score: 5.0,
        hypothesis: AcqHypothesis::Accepted,
        assumptions: None,
        evidence: Vec::new(),
        threshold_provenance: None,
        explain_selection_reason: None,
        doppler_refinement: None,
        code_phase_refinement: None,
        signal_delay_alignment: None,
        uncertainty: None,
    };

    let incremental = tracking.begin_incremental_tracking(&[acquisition]);
    let channel_state = incremental.channels.first().expect("GPS L5 tracking channel");
    let expected_code_rate_hz =
        shared_path_code_rate_hz(1_500.0, signal_spec_gps_l5_i(), signal_spec_gps_l5_i())
            .expect("carrier-aided code rate");

    assert!((channel_state.state.code_rate_hz - expected_code_rate_hz).abs() <= 1.0e-9);
    assert!((channel_state.state.code_rate_reference_hz - expected_code_rate_hz).abs() <= 1.0e-9);
}

#[test]
fn begin_incremental_tracking_refuses_incompatible_carrier_aiding_seed() {
    let config = crate::engine::receiver_config::ReceiverPipelineConfig {
        intermediate_freq_hz: GPS_L1_CA_CARRIER_HZ.value()
            - signal_spec_gps_l5_i().carrier_hz.value(),
        code_freq_basis_hz: 10_230_000.0,
        code_length: 10_230,
        ..crate::engine::receiver_config::ReceiverPipelineConfig::default()
    };
    let tracking = Tracking::new(config, ReceiverRuntime::default());
    let acquisition = bijux_gnss_core::api::AcqResult {
        sat: SatId { constellation: Constellation::Gps, prn: 18 },
        signal_band: SignalBand::L5,
        signal_code: SignalCode::L5I,
        glonass_frequency_channel: None,
        source_time: ReceiverSampleTrace::default(),
        candidate_rank: 1,
        is_primary_candidate: true,
        doppler_hz: Hertz(1_500.0),
        doppler_rate_hz_per_s: 0.0,
        carrier_hz: Hertz(GPS_L1_CA_CARRIER_HZ.value() + 1_500.0),
        code_phase_samples: 37,
        peak: 1.0,
        second_peak: 0.1,
        mean: 0.01,
        peak_mean_ratio: 8.0,
        peak_second_ratio: 3.0,
        cn0_proxy: 48.0,
        score: 5.0,
        hypothesis: AcqHypothesis::Accepted,
        assumptions: None,
        evidence: Vec::new(),
        threshold_provenance: None,
        explain_selection_reason: None,
        doppler_refinement: None,
        code_phase_refinement: None,
        signal_delay_alignment: None,
        uncertainty: None,
    };

    let incremental = tracking.begin_incremental_tracking(&[acquisition]);

    assert!(
        incremental.channels.is_empty(),
        "wrong-band carrier seeds must not enter carrier-aided tracking: {incremental:?}",
    );
}

fn split_frame(frame: &SamplesFrame, chunk_len: usize) -> Vec<SamplesFrame> {
    let mut chunks = Vec::new();
    let mut start = 0usize;
    while start < frame.len() {
        let end = (start + chunk_len.max(1)).min(frame.len());
        chunks.push(SamplesFrame::new(
            SampleTime {
                sample_index: frame.t0.sample_index + start as u64,
                sample_rate_hz: frame.t0.sample_rate_hz,
            },
            Seconds(frame.dt_s.0),
            frame.iq[start..end].to_vec(),
        ));
        start = end;
    }
    chunks
}
