#![allow(missing_docs)]

mod support;

use bijux_gnss_core::api::{
    AcqHypothesis, AcqResult, Constellation, Hertz, ReceiverSampleTrace, SatId, SignalBand,
    TrackEpoch,
};
use bijux_gnss_receiver::api::{
    sim::{
        generate_l1_ca, measure_truth_guided_tracking_lock_probability,
        measure_truth_guided_tracking_lock_rate, SyntheticSignalParams,
        SyntheticTrackingLockRateCase,
    },
    ReceiverPipelineConfig, ReceiverRuntime, TrackingChannelState, TrackingEngine,
};

const TRACKING_LOCK_RATE_TRIAL_COUNT: usize = 12;
const LOW_CN0_TRACKING_DURATION_S: f64 = 0.060;
const LOW_CN0_SEEDED_DOPPLER_ERROR_HZ: f64 = 60.0;
const LOW_CN0_SEEDED_CODE_PHASE_ERROR_SAMPLES: isize = 1;
const LOW_CN0_MIN_LOCKED_EPOCHS: usize = 5;
const WEAK_SIGNAL_ADAPTATION_CN0_DB_HZ: f32 = 31.0;
const WEAK_SIGNAL_ADAPTATION_DURATION_S: f64 = 0.120;

#[test]
fn tracking_lock_rate_report_runs_multiple_measurement_points() {
    let report = measure_truth_guided_tracking_lock_rate(
        &low_cn0_tracking_profile(),
        &[tracking_lock_rate_case(24.0), tracking_lock_rate_case(30.0)],
        &trial_seeds(0x2407_19A0, TRACKING_LOCK_RATE_TRIAL_COUNT),
        "tracking_lock_rate_smoke",
    );

    assert_eq!(report.points.len(), 2);
    assert_eq!(report.points[0].trial_count, TRACKING_LOCK_RATE_TRIAL_COUNT);
    assert_eq!(report.points[1].trial_count, TRACKING_LOCK_RATE_TRIAL_COUNT);
    assert_eq!(report.points[0].seeded_doppler_error_hz, LOW_CN0_SEEDED_DOPPLER_ERROR_HZ);
    assert_eq!(
        report.points[0].seeded_code_phase_error_samples,
        LOW_CN0_SEEDED_CODE_PHASE_ERROR_SAMPLES
    );
}

#[test]
fn tracking_lock_rate_reports_cn0_refusal_sensitivity() {
    let report = measure_truth_guided_tracking_lock_rate(
        &low_cn0_tracking_profile(),
        &[
            tracking_lock_rate_case(20.0),
            tracking_lock_rate_case(24.0),
            tracking_lock_rate_case(28.0),
            tracking_lock_rate_case(45.0),
        ],
        &trial_seeds(0x2407_19A1, TRACKING_LOCK_RATE_TRIAL_COUNT),
        "tracking_lock_rate_cn0",
    );

    assert_eq!(report.points.len(), 4);
    assert!(
        report.points.last().expect("strongest cn0").refused_lock_count
            < report.points.first().expect("weakest cn0").refused_lock_count,
        "expected stronger C/N0 to reduce tracking lock refusal count: {report:?}"
    );
}

#[test]
fn tracking_refuses_stable_lock_below_cn0_floor() {
    let weak_case = tracking_lock_rate_case(24.0);
    let report = measure_truth_guided_tracking_lock_probability(
        &low_cn0_tracking_profile(),
        weak_case.signal,
        weak_case.duration_s,
        &trial_seeds(0x2407_19A2, TRACKING_LOCK_RATE_TRIAL_COUNT),
        "tracking_low_cn0_refusal",
        weak_case.seeded_doppler_error_hz,
        weak_case.seeded_code_phase_error_samples,
        weak_case.min_locked_epochs,
    );

    assert_eq!(report.trial_count, TRACKING_LOCK_RATE_TRIAL_COUNT);
    assert_eq!(report.stable_lock_count, 0, "{report:?}");
    assert_eq!(report.refused_lock_count, TRACKING_LOCK_RATE_TRIAL_COUNT, "{report:?}");
    assert!(
        report.trials.iter().all(|trial| {
            !trial.stable_lock
                && trial.refused_lock
                && trial.first_lock_epoch_index.is_none()
                && trial.final_lock_state != "tracking"
                && trial.final_lock_state_reason.as_deref() == Some("cn0_below_tracking_lock_floor")
        }),
        "{report:?}"
    );
}

fn low_cn0_tracking_profile() -> ReceiverPipelineConfig {
    low_cn0_tracking_profile_with_adaptation(false)
}

fn low_cn0_tracking_profile_with_adaptation(
    adaptive_tracking_enabled: bool,
) -> ReceiverPipelineConfig {
    ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 4,
        early_late_spacing_chips: 0.5,
        dll_bw_hz: 2.0,
        pll_bw_hz: 18.0,
        fll_bw_hz: 12.0,
        adaptive_tracking_enabled,
        ..ReceiverPipelineConfig::default()
    }
}

fn tracking_lock_rate_case(cn0_db_hz: f32) -> SyntheticTrackingLockRateCase {
    SyntheticTrackingLockRateCase {
        signal: SyntheticSignalParams {
            sat: SatId { constellation: Constellation::Gps, prn: 16 },
            glonass_frequency_channel: None,
            signal_band: bijux_gnss_core::api::SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Unknown,
            doppler_hz: 180.0,
            code_phase_chips: 211.25,
            carrier_phase_rad: 0.40,
            cn0_db_hz,
            navigation_data: false.into(),
        },
        duration_s: LOW_CN0_TRACKING_DURATION_S,
        seeded_doppler_error_hz: LOW_CN0_SEEDED_DOPPLER_ERROR_HZ,
        seeded_code_phase_error_samples: LOW_CN0_SEEDED_CODE_PHASE_ERROR_SAMPLES,
        min_locked_epochs: LOW_CN0_MIN_LOCKED_EPOCHS,
    }
}

fn trial_seeds(base_seed: u64, count: usize) -> Vec<u64> {
    (0..count)
        .map(|index| base_seed.wrapping_add((index as u64).wrapping_mul(0x9e37_79b9_7f4a_7c15)))
        .collect()
}

fn accepted_acquisition(
    config: &ReceiverPipelineConfig,
    signal: SyntheticSignalParams,
    seeded_doppler_error_hz: f64,
    seeded_code_phase_error_samples: isize,
) -> AcqResult {
    let code_phase_samples = (signal.code_phase_chips * config.sampling_freq_hz
        / config.code_freq_basis_hz)
        .round() as isize
        + seeded_code_phase_error_samples;
    AcqResult {
        sat: signal.sat,
        signal_band: SignalBand::L1,
        signal_code: bijux_gnss_core::api::SignalCode::Unknown,
        glonass_frequency_channel: None,
        source_time: ReceiverSampleTrace::default(),
        candidate_rank: 1,
        is_primary_candidate: true,
        doppler_hz: Hertz(signal.doppler_hz - seeded_doppler_error_hz),
        doppler_rate_hz_per_s: 0.0,
        carrier_hz: Hertz(signal.doppler_hz - seeded_doppler_error_hz),
        code_phase_samples: code_phase_samples.max(0) as usize,
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
        explain_selection_reason: Some("low_cn0_tracking_seed".to_string()),
        doppler_refinement: None,
        code_phase_refinement: None,
        signal_delay_alignment: None,
        uncertainty: None,
    }
}

#[test]
fn tracking_session_reports_refused_channel_state_below_cn0_floor() {
    let config = low_cn0_tracking_profile();
    let case = tracking_lock_rate_case(24.0);
    let frame = generate_l1_ca(&config, case.signal.clone(), 0x2407_19A3, case.duration_s);
    let tracking = TrackingEngine::new(config.clone(), ReceiverRuntime::default());
    let mut session = tracking.begin_tracking_session(&[accepted_acquisition(
        &config,
        case.signal,
        case.seeded_doppler_error_hz,
        case.seeded_code_phase_error_samples,
    )]);
    tracking.track_session_frame(&mut session, &frame);
    let artifacts = tracking.finish_tracking_session(session);
    let report = artifacts.channel_state_reports.first().expect("channel state report");

    assert!(
        report.emitted_states.iter().any(|event| event.state == TrackingChannelState::Refused),
        "{report:?}"
    );
    assert_eq!(report.final_state, TrackingChannelState::Refused);
    assert_eq!(report.final_reason.as_deref(), Some("cn0_below_tracking_lock_floor"));
}

#[test]
fn adaptive_tracking_lengthens_integration_after_weak_signal_lock() {
    let adaptive_epochs = track_weak_signal_case(true);
    let fixed_epochs = track_weak_signal_case(false);
    let adaptive_integration_ms = tracking_integration_ms_values(&adaptive_epochs);
    let fixed_integration_ms = tracking_integration_ms_values(&fixed_epochs);

    assert!(
        adaptive_epochs
            .iter()
            .any(|epoch| epoch.lock_state == "tracking" || epoch.lock_state == "degraded"),
        "adaptive_epochs={adaptive_epochs:?}"
    );
    assert!(
        adaptive_integration_ms.iter().any(|integration_ms| *integration_ms == 5),
        "adaptive_integration_ms={adaptive_integration_ms:?} adaptive_epochs={adaptive_epochs:?}"
    );
    assert!(
        fixed_integration_ms.iter().all(|integration_ms| *integration_ms == 1),
        "fixed_integration_ms={fixed_integration_ms:?} fixed_epochs={fixed_epochs:?}"
    );
}

fn track_weak_signal_case(adaptive_tracking_enabled: bool) -> Vec<TrackEpoch> {
    let config = low_cn0_tracking_profile_with_adaptation(adaptive_tracking_enabled);
    let signal = SyntheticSignalParams {
        sat: SatId { constellation: Constellation::Gps, prn: 16 },
        glonass_frequency_channel: None,
        signal_band: bijux_gnss_core::api::SignalBand::L1,
        signal_code: bijux_gnss_core::api::SignalCode::Unknown,
        doppler_hz: 40.0,
        code_phase_chips: 211.25,
        carrier_phase_rad: 0.10,
        cn0_db_hz: WEAK_SIGNAL_ADAPTATION_CN0_DB_HZ,
        navigation_data: false.into(),
    };
    let frame =
        generate_l1_ca(&config, signal.clone(), 0x2407_19A4, WEAK_SIGNAL_ADAPTATION_DURATION_S);
    let tracking = TrackingEngine::new(config.clone(), ReceiverRuntime::default());
    let tracks =
        tracking.track_from_acquisition(&frame, &[accepted_acquisition(&config, signal, 0.0, 0)]);

    tracks.first().expect("weak signal track").epochs.clone()
}

fn tracking_integration_ms_values(epochs: &[TrackEpoch]) -> Vec<u32> {
    epochs
        .iter()
        .filter_map(|epoch| {
            epoch.tracking_assumptions.as_ref().map(|assumptions| assumptions.integration_ms.max(1))
        })
        .collect()
}
