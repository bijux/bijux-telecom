#![allow(missing_docs)]

use bijux_gnss_core::api::{
    AcqHypothesis, AcqResult, Constellation, Hertz, ReceiverSampleTrace, SatId, SignalBand,
    SignalCode, TrackEpoch,
};
use bijux_gnss_receiver::api::{
    carrier_hz_from_doppler_hz,
    sim::{
        expected_acquisition_code_phase_samples, generate_l1_ca_multi, SyntheticScenario,
        SyntheticSignalParams,
    },
    ReceiverPipelineConfig, ReceiverRuntime, TrackingArtifacts, TrackingEngine, TrackingResult,
};

const COMMON_FREQUENCY_CN0_DB_HZ: f32 = 72.0;
const COMMON_FREQUENCY_DURATION_S: f64 = 0.030;
const COMMON_FREQUENCY_WEAK_DURATION_S: f64 = 0.030;
const INJECTED_RECEIVER_FREQUENCY_ERROR_HZ: f64 = 2.0;
const COMMON_FREQUENCY_MAX_RESIDUAL_HZ: f64 = 2.0;
const COMMON_FREQUENCY_WEAK_SAT: SatId = SatId { constellation: Constellation::Gps, prn: 11 };

fn common_frequency_tracking_config(vector_tracking_enabled: bool) -> ReceiverPipelineConfig {
    ReceiverPipelineConfig {
        sampling_freq_hz: 1_023_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 4,
        early_late_spacing_chips: 0.5,
        dll_bw_hz: 2.0,
        pll_bw_hz: 18.0,
        fll_bw_hz: 10.0,
        adaptive_tracking_enabled: false,
        vector_tracking_enabled,
        tracking_budget_ms: 100.0,
        tracking_over_budget_action: "continue".to_string(),
        ..ReceiverPipelineConfig::default()
    }
}

fn common_frequency_signal(
    prn: u8,
    doppler_hz: f64,
    code_phase_chips: f64,
) -> SyntheticSignalParams {
    common_frequency_signal_with_cn0(prn, doppler_hz, code_phase_chips, COMMON_FREQUENCY_CN0_DB_HZ)
}

fn common_frequency_signal_with_cn0(
    prn: u8,
    doppler_hz: f64,
    code_phase_chips: f64,
    cn0_db_hz: f32,
) -> SyntheticSignalParams {
    SyntheticSignalParams {
        sat: SatId { constellation: Constellation::Gps, prn },
        glonass_frequency_channel: None,
        signal_band: SignalBand::L1,
        signal_code: SignalCode::Unknown,
        doppler_hz,
        code_phase_chips,
        carrier_phase_rad: 0.0,
        cn0_db_hz,
        navigation_data: false.into(),
    }
}

fn common_frequency_signals() -> Vec<SyntheticSignalParams> {
    vec![
        common_frequency_signal(3, 40.0, 17.25),
        common_frequency_signal(5, -65.0, 227.50),
        common_frequency_signal(7, 95.0, 713.75),
    ]
}

fn common_frequency_weak_channel_signals() -> Vec<SyntheticSignalParams> {
    vec![
        common_frequency_signal(3, 40.0, 17.25),
        common_frequency_signal(5, -65.0, 227.50),
        common_frequency_signal(7, 95.0, 713.75),
        common_frequency_signal_with_cn0(11, 43.0, 511.25, 31.0),
    ]
}

fn accepted_acquisition(
    config: &ReceiverPipelineConfig,
    frame: &bijux_gnss_core::api::SamplesFrame,
    signal: &SyntheticSignalParams,
) -> AcqResult {
    let code_phase_samples =
        expected_acquisition_code_phase_samples(config, frame, signal.code_phase_chips);
    AcqResult {
        sat: signal.sat,
        signal_band: signal.signal_band,
        signal_code: signal.signal_code,
        glonass_frequency_channel: signal.glonass_frequency_channel,
        source_time: ReceiverSampleTrace::default(),
        candidate_rank: 1,
        is_primary_candidate: true,
        doppler_hz: Hertz(signal.doppler_hz),
        doppler_rate_hz_per_s: 0.0,
        carrier_hz: Hertz(carrier_hz_from_doppler_hz(
            config.intermediate_freq_hz,
            signal.doppler_hz,
        )),
        code_phase_samples,
        peak: 1.0,
        second_peak: 0.1,
        mean: 0.01,
        peak_mean_ratio: 20.0,
        peak_second_ratio: 10.0,
        cn0_proxy: signal.cn0_db_hz,
        score: 1.0,
        hypothesis: AcqHypothesis::Accepted,
        assumptions: None,
        evidence: Vec::new(),
        threshold_provenance: None,
        explain_selection_reason: Some("common_frequency_tracking_start".to_string()),
        doppler_refinement: None,
        code_phase_refinement: None,
        signal_delay_alignment: None,
        uncertainty: None,
    }
}

fn track_common_frequency_case(
    vector_tracking_enabled: bool,
    duration_s: f64,
    seed: u64,
    signals: &[SyntheticSignalParams],
) -> TrackingArtifacts {
    let config = common_frequency_tracking_config(vector_tracking_enabled);
    let scenario = SyntheticScenario {
        sample_rate_hz: config.sampling_freq_hz,
        intermediate_freq_hz: config.intermediate_freq_hz,
        receiver_clock_frequency_bias_hz: INJECTED_RECEIVER_FREQUENCY_ERROR_HZ,
        duration_s,
        seed,
        satellites: signals.to_vec(),
        ephemerides: Vec::new(),
        id: "common-frequency-tracking".to_string(),
    };
    let frame = generate_l1_ca_multi(&config, &scenario);
    let acquisitions = signals
        .iter()
        .map(|signal| accepted_acquisition(&config, &frame, signal))
        .collect::<Vec<_>>();
    let tracking = TrackingEngine::new(config, ReceiverRuntime::default());
    let mut session = tracking.begin_tracking_session(&acquisitions);
    tracking.track_session_frame(&mut session, &frame);
    tracking.finish_tracking_session(session)
}

fn track_for_sat<'a>(artifacts: &'a TrackingArtifacts, sat: SatId) -> &'a TrackingResult {
    artifacts.tracking.iter().find(|track| track.sat == sat).expect("track for satellite")
}

fn receiver_state_aided_epoch_count(epochs: &[TrackEpoch]) -> usize {
    epochs
        .iter()
        .filter(|epoch| {
            epoch.tracking_assumptions.as_ref().is_some_and(|assumptions| {
                assumptions.aiding_mode.contains("vector_receiver_state")
            })
        })
        .count()
}

#[test]
fn tracking_artifacts_report_common_receiver_frequency_error_from_stable_channels() {
    let signals = common_frequency_signals();
    let artifacts =
        track_common_frequency_case(true, COMMON_FREQUENCY_DURATION_S, 0xC011_4101, &signals);
    let common_frequency = artifacts.common_frequency.unwrap_or_else(|| {
        panic!(
            "common frequency estimate missing: tracks={:?}",
            artifacts
                .tracking
                .iter()
                .map(|track| {
                    (
                        track.sat,
                        track
                            .epochs
                            .iter()
                            .filter(|epoch| {
                                epoch.lock_state == "tracking" && epoch.pll_lock && epoch.fll_lock
                            })
                            .count(),
                        track.epochs.last().map(|epoch| {
                            (
                                epoch.lock_state.clone(),
                                epoch.lock_state_reason.clone(),
                                epoch.cn0_dbhz,
                                epoch.fll_lock,
                                epoch.pll_lock,
                            )
                        }),
                    )
                })
                .collect::<Vec<_>>()
        )
    });

    assert!(
        common_frequency.support_count >= 2,
        "common frequency estimate should combine at least two stable channels: common_frequency={common_frequency:?}"
    );
    assert!(
        common_frequency.estimated_frequency_error_hz.is_finite(),
        "common frequency estimate must be finite: common_frequency={common_frequency:?}"
    );
    assert!(
        common_frequency.max_supporting_residual_hz <= COMMON_FREQUENCY_MAX_RESIDUAL_HZ,
        "common frequency estimate should not absorb satellite-specific motion: common_frequency={common_frequency:?}"
    );
}

#[test]
fn common_frequency_state_aids_weak_channel_from_stable_frequency_support() {
    let signals = common_frequency_weak_channel_signals();
    let vector_artifacts =
        track_common_frequency_case(true, COMMON_FREQUENCY_WEAK_DURATION_S, 0xC011_4102, &signals);
    let vector_weak = track_for_sat(&vector_artifacts, COMMON_FREQUENCY_WEAK_SAT);
    let common_frequency =
        vector_artifacts.common_frequency.as_ref().expect("common frequency state");

    assert!(
        receiver_state_aided_epoch_count(&vector_weak.epochs) > 0,
        "weak channel never received vector receiver-state aiding: vector_weak={vector_weak:?}"
    );
    assert!(
        vector_weak.epochs.iter().any(|epoch| {
            epoch.tracking_provenance.contains("vector_carrier_frequency_correction_hz=")
        }),
        "weak channel never reported carrier frequency correction provenance: vector_weak={vector_weak:?}"
    );
    assert!(
        common_frequency.support_count >= 2,
        "common frequency state should retain stable support: common_frequency={common_frequency:?}"
    );
    assert!(
        common_frequency
            .supporting_channels
            .iter()
            .all(|channel| channel.sat != COMMON_FREQUENCY_WEAK_SAT),
        "weak channel should consume the common state, not define it: common_frequency={common_frequency:?}"
    );
}
