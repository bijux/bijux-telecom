#![allow(missing_docs)]

use bijux_gnss_core::api::{
    AcqHypothesis, AcqResult, Constellation, Hertz, ReceiverSampleTrace, SatId, SignalBand,
};
use bijux_gnss_receiver::api::{
    sim::{generate_l1_ca, generate_l1_ca_with_fades, SyntheticFadeWindow, SyntheticSignalParams},
    ReceiverPipelineConfig, ReceiverRuntime, TrackingChannelState, TrackingEngine,
};

fn accepted_acquisition(sat: SatId, doppler_hz: f64, code_phase_samples: usize) -> AcqResult {
    AcqResult {
        sat,
        signal_band: SignalBand::L1,
        source_time: ReceiverSampleTrace::default(),
        candidate_rank: 1,
        is_primary_candidate: true,
        doppler_hz: Hertz(doppler_hz),
        carrier_hz: Hertz(doppler_hz),
        code_phase_samples,
        peak: 1.0,
        second_peak: 0.1,
        mean: 0.01,
        peak_mean_ratio: 20.0,
        peak_second_ratio: 10.0,
        cn0_proxy: 60.0,
        score: 1.0,
        hypothesis: AcqHypothesis::Accepted,
        assumptions: None,
        evidence: Vec::new(),
        threshold_provenance: None,
        explain_selection_reason: Some("channel_state_report_seed".to_string()),
        doppler_refinement: None,
        code_phase_refinement: None,
        uncertainty: None,
    }
}

fn tracking_config() -> ReceiverPipelineConfig {
    ReceiverPipelineConfig {
        sampling_freq_hz: 1_023_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 4,
        early_late_spacing_chips: 0.5,
        dll_bw_hz: 2.0,
        pll_bw_hz: 15.0,
        fll_bw_hz: 10.0,
        ..ReceiverPipelineConfig::default()
    }
}

#[test]
fn tracking_session_reports_locked_channel_state() {
    let config = tracking_config();
    let sat = SatId { constellation: Constellation::Gps, prn: 21 };
    let frame = generate_l1_ca(
        &config,
        SyntheticSignalParams {
            sat,
            doppler_hz: 0.0,
            code_phase_chips: 0.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 60.0,
            data_bit_flip: false,
        },
        0xC11E_A001,
        0.012,
    );
    let tracking = TrackingEngine::new(config, ReceiverRuntime::default());
    let mut session = tracking.begin_tracking_session(&[accepted_acquisition(sat, 0.0, 0)]);
    tracking.track_session_frame(&mut session, &frame);
    let artifacts = tracking.finish_tracking_session(session);
    let report = artifacts.channel_state_reports.first().expect("channel state report");

    assert!(
        report.emitted_states.iter().any(|event| event.state == TrackingChannelState::Acquired),
        "{report:?}"
    );
    assert!(
        report.emitted_states.iter().any(|event| event.state == TrackingChannelState::Locked),
        "{report:?}"
    );
    assert_eq!(report.final_state, TrackingChannelState::Locked);
    assert_ne!(report.channel_uid, "");
}

#[test]
fn tracking_session_reports_degraded_channel_state_during_short_fade() {
    let config = tracking_config();
    let sat = SatId { constellation: Constellation::Gps, prn: 21 };
    let frame = generate_l1_ca_with_fades(
        &config,
        SyntheticSignalParams {
            sat,
            doppler_hz: 0.0,
            code_phase_chips: 0.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 60.0,
            data_bit_flip: false,
        },
        &[SyntheticFadeWindow { start_s: 0.030, end_s: 0.040, signal_scale: 0.0 }],
        0xC11E_A002,
        0.060,
    );
    let tracking = TrackingEngine::new(config, ReceiverRuntime::default());
    let mut session = tracking.begin_tracking_session(&[accepted_acquisition(sat, 0.0, 0)]);
    tracking.track_session_frame(&mut session, &frame);
    let artifacts = tracking.finish_tracking_session(session);
    let report = artifacts.channel_state_reports.first().expect("channel state report");

    assert!(
        report.emitted_states.iter().any(|event| event.state == TrackingChannelState::Degraded),
        "{report:?}"
    );
}
