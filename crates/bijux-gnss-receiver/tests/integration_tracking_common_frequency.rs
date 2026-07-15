#![allow(missing_docs)]

use bijux_gnss_core::api::{
    AcqHypothesis, AcqResult, Constellation, Hertz, ReceiverSampleTrace, SatId, SignalBand,
    SignalCode,
};
use bijux_gnss_receiver::api::{
    carrier_hz_from_doppler_hz,
    sim::{
        expected_acquisition_code_phase_samples, generate_l1_ca_multi, SyntheticScenario,
        SyntheticSignalParams,
    },
    ReceiverPipelineConfig, ReceiverRuntime, TrackingEngine,
};

const COMMON_FREQUENCY_CN0_DB_HZ: f32 = 72.0;
const COMMON_FREQUENCY_DURATION_S: f64 = 0.030;
const INJECTED_RECEIVER_FREQUENCY_ERROR_HZ: f64 = 2.0;
const COMMON_FREQUENCY_MAX_RESIDUAL_HZ: f64 = 1.5;

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
    SyntheticSignalParams {
        sat: SatId { constellation: Constellation::Gps, prn },
        glonass_frequency_channel: None,
        signal_band: SignalBand::L1,
        signal_code: SignalCode::Unknown,
        doppler_hz,
        code_phase_chips,
        carrier_phase_rad: 0.0,
        cn0_db_hz: COMMON_FREQUENCY_CN0_DB_HZ,
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
        cn0_proxy: COMMON_FREQUENCY_CN0_DB_HZ,
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

#[test]
fn tracking_artifacts_report_common_receiver_frequency_error_from_stable_channels() {
    let config = common_frequency_tracking_config(true);
    let signals = common_frequency_signals();
    let scenario = SyntheticScenario {
        sample_rate_hz: config.sampling_freq_hz,
        intermediate_freq_hz: config.intermediate_freq_hz,
        receiver_clock_frequency_bias_hz: INJECTED_RECEIVER_FREQUENCY_ERROR_HZ,
        duration_s: COMMON_FREQUENCY_DURATION_S,
        seed: 0xC011_4101,
        satellites: signals.clone(),
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
    let artifacts = tracking.finish_tracking_session(session);
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
