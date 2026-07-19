#![allow(missing_docs)]

mod support;

use bijux_gnss_core::api::{
    AcqCodePhaseRefinement, AcqHypothesis, AcqResult, Constellation, Hertz, ReceiverSampleTrace,
    SampleTime, SamplesFrame, SatId, Seconds, SignalBand, SignalCode, BEIDOU_B1_CARRIER_HZ,
    GPS_L1_CA_CARRIER_HZ,
};
use bijux_gnss_receiver::api::{
    sim::{
        expected_acquisition_code_phase_samples, expected_acquisition_code_phase_samples_f64,
        generate_l1_ca, SyntheticSignalParams,
    },
    ReceiverPipelineConfig, ReceiverRuntime, TrackingEngine,
};
use bijux_gnss_signal::api::{sample_ca_code, samples_per_code, Prn};
use num_complex::Complex;

use support::tracking_truth::{
    post_lock_code_phase_errors_samples, stable_tracking_window, wrapped_code_phase_error_samples,
};

const CLEAN_SIGNAL_LOCKED_CODE_ERROR_MAX_SAMPLES: f64 = 1.0;
const CLEAN_SIGNAL_MIN_LOCKED_CODE_EPOCHS: usize = 5;
const GALILEO_E1_LOCK_VALIDATION_DURATION_S: f64 = 0.120;

fn assert_clean_signal_code_lock(
    config: &ReceiverPipelineConfig,
    epochs: &[bijux_gnss_core::api::TrackEpoch],
    expected_code_phase_samples: f64,
) {
    let post_lock_errors_samples =
        post_lock_code_phase_errors_samples(config, epochs, expected_code_phase_samples);
    assert!(
        post_lock_errors_samples.len() >= CLEAN_SIGNAL_MIN_LOCKED_CODE_EPOCHS,
        "tracking did not maintain enough locked epochs for clean code-phase validation: post_lock_errors_samples={post_lock_errors_samples:?}, epochs={epochs:?}"
    );
    assert!(
        post_lock_errors_samples
            .iter()
            .all(|error_samples| *error_samples <= CLEAN_SIGNAL_LOCKED_CODE_ERROR_MAX_SAMPLES),
        "locked code phase error exceeded clean-signal threshold {CLEAN_SIGNAL_LOCKED_CODE_ERROR_MAX_SAMPLES} samples: post_lock_errors_samples={post_lock_errors_samples:?}, epochs={epochs:?}"
    );
}

fn code_locked_epochs(
    epochs: &[bijux_gnss_core::api::TrackEpoch],
) -> Vec<&bijux_gnss_core::api::TrackEpoch> {
    epochs
        .iter()
        .filter(|epoch| {
            matches!(epoch.lock_state.as_str(), "tracking" | "degraded")
                && epoch.lock
                && epoch.dll_lock
                && epoch.fll_lock
                && !epoch.cycle_slip
        })
        .collect()
}

fn accepted_acquisition(sat: SatId, doppler_hz: f64, code_phase_samples: usize) -> AcqResult {
    accepted_acquisition_with_signal_band(sat, SignalBand::L1, doppler_hz, code_phase_samples)
}

fn accepted_acquisition_with_signal_band(
    sat: SatId,
    signal_band: SignalBand,
    doppler_hz: f64,
    code_phase_samples: usize,
) -> AcqResult {
    AcqResult {
        sat,
        signal_band,
        signal_code: bijux_gnss_core::api::SignalCode::Unknown,
        glonass_frequency_channel: None,
        source_time: ReceiverSampleTrace::default(),
        candidate_rank: 1,
        is_primary_candidate: true,
        doppler_hz: Hertz(doppler_hz),
        doppler_rate_hz_per_s: 0.0,
        carrier_hz: Hertz(doppler_hz),
        code_phase_samples,
        peak: 1.0,
        second_peak: 0.1,
        mean: 0.01,
        peak_mean_ratio: 20.0,
        peak_second_ratio: 10.0,
        cn0_proxy: 52.0,
        score: 1.0,
        hypothesis: AcqHypothesis::Accepted,
        assumptions: None,
        evidence: Vec::new(),
        threshold_provenance: None,
        explain_selection_reason: Some("seeded_tracking_start".to_string()),
        doppler_refinement: None,
        code_phase_refinement: None,
        signal_delay_alignment: None,
        uncertainty: None,
    }
}

fn accepted_refined_acquisition_with_signal_band(
    sat: SatId,
    signal_band: SignalBand,
    doppler_hz: f64,
    code_phase_samples: usize,
    refined_code_phase_samples: f64,
) -> AcqResult {
    let mut acquisition =
        accepted_acquisition_with_signal_band(sat, signal_band, doppler_hz, code_phase_samples);
    acquisition.code_phase_refinement = Some(AcqCodePhaseRefinement {
        method: "synthetic_resolved_code_phase".to_string(),
        offset_samples: refined_code_phase_samples - code_phase_samples as f64,
        refined_code_phase_samples,
        left_correlation_norm: 0.0,
        center_correlation_norm: 1.0,
        right_correlation_norm: 0.0,
    });
    acquisition
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

fn galileo_tracking_config() -> ReceiverPipelineConfig {
    ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 4092,
        channels: 4,
        early_late_spacing_chips: 0.5,
        dll_bw_hz: 2.0,
        pll_bw_hz: 15.0,
        fll_bw_hz: 10.0,
        ..ReceiverPipelineConfig::default()
    }
}

fn beidou_tracking_config() -> ReceiverPipelineConfig {
    ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: GPS_L1_CA_CARRIER_HZ.value() - BEIDOU_B1_CARRIER_HZ.value(),
        code_freq_basis_hz: 2_046_000.0,
        code_length: 2046,
        channels: 4,
        early_late_spacing_chips: 0.5,
        dll_bw_hz: 2.0,
        pll_bw_hz: 15.0,
        fll_bw_hz: 10.0,
        ..ReceiverPipelineConfig::default()
    }
}

fn synthetic_frame_with_code_phase(
    config: &ReceiverPipelineConfig,
    sat: SatId,
    signal_band: SignalBand,
    signal_code: SignalCode,
    code_phase_chips: f64,
    duration_s: f64,
) -> bijux_gnss_core::api::SamplesFrame {
    generate_l1_ca(
        config,
        SyntheticSignalParams {
            sat,
            glonass_frequency_channel: None,
            signal_band,
            signal_code,
            doppler_hz: 0.0,
            code_phase_chips,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 52.0,
            navigation_data: false.into(),
        },
        0xD11C_600D,
        duration_s,
    )
}

fn track_clean_code_case(
    config: &ReceiverPipelineConfig,
    sat: SatId,
    signal_band: SignalBand,
    signal_code: SignalCode,
    code_phase_chips: f64,
    duration_s: f64,
    seeded_code_phase_samples: usize,
) -> Vec<bijux_gnss_core::api::TrackEpoch> {
    let frame = synthetic_frame_with_code_phase(
        config,
        sat,
        signal_band,
        signal_code,
        code_phase_chips,
        duration_s,
    );
    let tracking = TrackingEngine::new(config.clone(), ReceiverRuntime::default());
    let tracks = tracking.track_from_acquisition(
        &frame,
        &[accepted_acquisition_with_signal_band(sat, signal_band, 0.0, seeded_code_phase_samples)],
    );

    tracks.first().expect("track").epochs.clone()
}

#[test]
fn tracking_reduces_seeded_code_phase_error_from_acquisition_scale_offsets() {
    let config = tracking_config();
    let sat = SatId { constellation: Constellation::Gps, prn: 11 };

    for seeded_code_phase_samples in [1_usize, 1_022_usize] {
        let epochs = track_clean_code_case(
            &config,
            sat,
            SignalBand::L1,
            SignalCode::Ca,
            0.0,
            0.012,
            seeded_code_phase_samples,
        );
        let epochs = &epochs;
        assert!(epochs.len() >= 4, "epochs={epochs:?}");

        let errors = epochs
            .iter()
            .map(|epoch| wrapped_code_phase_error_samples(&config, epoch.code_phase_samples.0, 0.0))
            .collect::<Vec<_>>();
        let first_error = errors[0];
        let best_error = epochs
            .iter()
            .map(|epoch| wrapped_code_phase_error_samples(&config, epoch.code_phase_samples.0, 0.0))
            .min_by(|lhs, rhs| lhs.partial_cmp(rhs).expect("finite code phase error"))
            .unwrap_or(f64::INFINITY);

        assert!(
            best_error < first_error,
            "tracking did not reduce code phase error for seeded code phase sample {seeded_code_phase_samples}: first_error={first_error}, best_error={best_error}, epochs={epochs:?}"
        );
        assert!(
            errors.last().copied().unwrap_or(first_error) <= first_error,
            "tracking ended with worse code phase error for seeded code phase sample {seeded_code_phase_samples}: errors={errors:?}, epochs={epochs:?}"
        );
        assert!(
            !code_locked_epochs(epochs).is_empty(),
            "tracking never reached a post-pull-in tracking epoch for seeded code phase sample {seeded_code_phase_samples}: epochs={epochs:?}"
        );
        let assumptions = epochs
            .iter()
            .find_map(|epoch| epoch.tracking_assumptions.as_ref())
            .expect("tracking assumptions");
        assert_eq!(assumptions.early_late_spacing_chips, 0.25);
    }
}

#[test]
fn tracking_steers_code_rate_toward_faster_signal_code() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 4,
        early_late_spacing_chips: 0.5,
        dll_bw_hz: 900.0,
        pll_bw_hz: 0.0,
        fll_bw_hz: 0.0,
        ..ReceiverPipelineConfig::default()
    };
    let sat = SatId { constellation: Constellation::Gps, prn: 19 };
    let sample_count =
        samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length)
            * 12;
    let signal_code_rate_hz = config.code_freq_basis_hz + 300.0;
    let code_phase_chips = 144.25;
    let frame = SamplesFrame::new(
        SampleTime { sample_index: 0, sample_rate_hz: config.sampling_freq_hz },
        Seconds(1.0 / config.sampling_freq_hz),
        sample_ca_code(
            Prn(sat.prn),
            config.sampling_freq_hz,
            signal_code_rate_hz,
            code_phase_chips,
            sample_count,
        )
        .expect("valid sampled code with faster signal code rate")
        .into_iter()
        .map(|value| Complex::new(value, 0.0))
        .collect(),
    );
    let seeded_code_phase_samples =
        expected_acquisition_code_phase_samples(&config, &frame, code_phase_chips);
    let tracking = TrackingEngine::new(config.clone(), ReceiverRuntime::default());

    let tracks = tracking.track_from_acquisition(
        &frame,
        &[accepted_acquisition(sat, 0.0, seeded_code_phase_samples)],
    );
    let epochs = &tracks.first().expect("track").epochs;
    let best_code_rate_hz = epochs
        .iter()
        .map(|epoch| epoch.code_rate_hz.0)
        .max_by(|lhs, rhs| lhs.partial_cmp(rhs).expect("finite code rate"))
        .unwrap_or(f64::NEG_INFINITY);

    assert!(
        best_code_rate_hz > config.code_freq_basis_hz,
        "best_code_rate_hz={best_code_rate_hz} epochs={epochs:?}",
    );
}

#[test]
fn tracking_holds_clean_code_lock_for_fractional_phase_seed() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_000_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 4,
        early_late_spacing_chips: 0.5,
        dll_bw_hz: 2.0,
        pll_bw_hz: 15.0,
        fll_bw_hz: 10.0,
        ..ReceiverPipelineConfig::default()
    };
    let sat = SatId { constellation: Constellation::Gps, prn: 14 };
    let code_phase_chips = 144.375;
    let frame = synthetic_frame_with_code_phase(
        &config,
        sat,
        SignalBand::L1,
        SignalCode::Ca,
        code_phase_chips,
        0.012,
    );
    let expected_code_phase_samples =
        expected_acquisition_code_phase_samples_f64(&config, &frame, code_phase_chips);
    let seeded_code_phase_samples =
        expected_acquisition_code_phase_samples(&config, &frame, code_phase_chips);
    let epochs = track_clean_code_case(
        &config,
        sat,
        SignalBand::L1,
        SignalCode::Ca,
        code_phase_chips,
        0.012,
        seeded_code_phase_samples,
    );

    assert!(epochs.len() >= CLEAN_SIGNAL_MIN_LOCKED_CODE_EPOCHS, "epochs={epochs:?}");
    assert_clean_signal_code_lock(&config, &epochs, expected_code_phase_samples);
}

#[test]
fn tracking_holds_galileo_e1_lock_on_clean_synthetic_signal() {
    let config = galileo_tracking_config();
    let sat = SatId { constellation: Constellation::Galileo, prn: 11 };
    let code_phase_chips = 321.375;
    let frame = synthetic_frame_with_code_phase(
        &config,
        sat,
        SignalBand::E1,
        SignalCode::E1B,
        code_phase_chips,
        GALILEO_E1_LOCK_VALIDATION_DURATION_S,
    );
    let expected_code_phase_samples =
        expected_acquisition_code_phase_samples_f64(&config, &frame, code_phase_chips);
    let seeded_code_phase_samples =
        expected_acquisition_code_phase_samples(&config, &frame, code_phase_chips);
    let tracking = TrackingEngine::new(config.clone(), ReceiverRuntime::default());
    let tracks = tracking.track_from_acquisition(
        &frame,
        &[accepted_refined_acquisition_with_signal_band(
            sat,
            SignalBand::E1,
            0.0,
            seeded_code_phase_samples,
            expected_code_phase_samples,
        )],
    );
    let epochs = &tracks.first().expect("track").epochs;
    let locked_epochs = code_locked_epochs(epochs);
    let locked_errors_samples = locked_epochs
        .iter()
        .map(|epoch| {
            wrapped_code_phase_error_samples(
                &config,
                epoch.code_phase_samples.0,
                expected_code_phase_samples,
            )
        })
        .collect::<Vec<_>>();

    assert!(
        locked_epochs.len() >= CLEAN_SIGNAL_MIN_LOCKED_CODE_EPOCHS,
        "tracking did not sustain Galileo E1 lock: epochs={epochs:?}",
    );
    assert!(
        locked_errors_samples.iter().all(|error_samples| *error_samples <= 2.0),
        "Galileo E1 code tracking drifted after lock: locked_errors_samples={locked_errors_samples:?}, epochs={epochs:?}",
    );
    assert!(
        epochs.iter().all(|epoch| epoch.tracking_provenance.contains("acq_signal_band=E1")),
        "tracking provenance did not preserve Galileo E1 handoff metadata: epochs={epochs:?}",
    );
    assert!(
        epochs
            .iter()
            .all(|epoch| epoch.tracking_provenance.contains("subcarrier_code_phase_handoff=refined")),
        "tracking provenance did not preserve refined Galileo E1 code-phase handoff metadata: epochs={epochs:?}",
    );
}

#[test]
fn galileo_e1_side_peak_seed_does_not_report_biased_code_lock() {
    let config = galileo_tracking_config();
    let sat = SatId { constellation: Constellation::Galileo, prn: 11 };
    let code_phase_chips = 321.375;
    let frame = synthetic_frame_with_code_phase(
        &config,
        sat,
        SignalBand::E1,
        SignalCode::E1B,
        code_phase_chips,
        GALILEO_E1_LOCK_VALIDATION_DURATION_S,
    );
    let expected_code_phase_samples =
        expected_acquisition_code_phase_samples_f64(&config, &frame, code_phase_chips);
    let samples_per_chip = config.sampling_freq_hz / config.code_freq_basis_hz;
    let seeded_code_phase_samples =
        (expected_code_phase_samples + samples_per_chip * 0.5).round() as usize;
    let tracking = TrackingEngine::new(config.clone(), ReceiverRuntime::default());

    let tracks = tracking.track_from_acquisition(
        &frame,
        &[accepted_acquisition_with_signal_band(
            sat,
            SignalBand::E1,
            0.0,
            seeded_code_phase_samples,
        )],
    );
    let epochs = &tracks.first().expect("track").epochs;
    let biased_locked_epochs = epochs
        .iter()
        .filter(|epoch| {
            epoch.dll_lock
                && wrapped_code_phase_error_samples(
                    &config,
                    epoch.code_phase_samples.0,
                    expected_code_phase_samples,
                ) > samples_per_chip * 0.35
        })
        .collect::<Vec<_>>();

    assert!(
        biased_locked_epochs.is_empty(),
        "Galileo E1 side-peak seed reported valid code lock with a biased delay: biased_locked_epochs={biased_locked_epochs:?}, epochs={epochs:?}",
    );
    assert!(
        epochs.iter().any(|epoch| {
            epoch.tracking_provenance.contains("subcarrier_ambiguity_guard=side_peak_guard")
                && epoch.tracking_provenance.contains("prompt_relative_power=")
                && epoch.tracking_provenance.contains("subcarrier_code_phase_handoff=coarse")
        }),
        "Galileo E1 side-peak seed did not report subcarrier ambiguity guard evidence: epochs={epochs:?}",
    );
}

#[test]
fn tracking_holds_beidou_b1i_lock_on_clean_synthetic_signal() {
    let config = beidou_tracking_config();
    let sat = SatId { constellation: Constellation::Beidou, prn: 11 };
    let code_phase_chips = 321.375;
    let frame = synthetic_frame_with_code_phase(
        &config,
        sat,
        SignalBand::B1,
        SignalCode::Unknown,
        code_phase_chips,
        0.020,
    );
    let expected_code_phase_samples =
        expected_acquisition_code_phase_samples_f64(&config, &frame, code_phase_chips);
    let seeded_code_phase_samples =
        expected_acquisition_code_phase_samples(&config, &frame, code_phase_chips);
    let tracking = TrackingEngine::new(config.clone(), ReceiverRuntime::default());
    let tracks = tracking.track_from_acquisition(
        &frame,
        &[accepted_acquisition_with_signal_band(
            sat,
            SignalBand::B1,
            0.0,
            seeded_code_phase_samples,
        )],
    );
    let epochs = &tracks.first().expect("track").epochs;
    let stable_epochs = stable_tracking_window(epochs, CLEAN_SIGNAL_MIN_LOCKED_CODE_EPOCHS);
    let stable_errors_samples = stable_epochs
        .iter()
        .map(|epoch| {
            wrapped_code_phase_error_samples(
                &config,
                epoch.code_phase_samples.0,
                expected_code_phase_samples,
            )
        })
        .collect::<Vec<_>>();

    assert!(
        stable_epochs.len() >= CLEAN_SIGNAL_MIN_LOCKED_CODE_EPOCHS,
        "tracking did not sustain BeiDou B1I lock: epochs={epochs:?}",
    );
    assert!(
        stable_epochs
            .iter()
            .all(|epoch| epoch.lock && epoch.dll_lock && epoch.pll_lock && epoch.fll_lock),
        "BeiDou B1I post-lock epochs lost lock flags: epochs={epochs:?}",
    );
    assert!(
        stable_errors_samples.iter().all(|error_samples| *error_samples <= 2.0),
        "BeiDou B1I code tracking drifted after lock: stable_errors_samples={stable_errors_samples:?}, epochs={epochs:?}",
    );
    assert!(
        epochs.iter().all(|epoch| epoch.tracking_provenance.contains("acq_signal_band=B1")),
        "tracking provenance did not preserve BeiDou B1I handoff metadata: epochs={epochs:?}",
    );
}
