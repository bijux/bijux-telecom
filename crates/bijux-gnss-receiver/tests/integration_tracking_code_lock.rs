#![allow(missing_docs)]

mod support;

use bijux_gnss_core::api::{
    AcqHypothesis, AcqResult, Constellation, Hertz, ReceiverSampleTrace, SampleTime, SamplesFrame,
    SatId, Seconds, SignalBand,
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
    post_lock_code_phase_errors_samples, post_lock_epochs, wrapped_code_phase_error_samples,
};

const CLEAN_SIGNAL_LOCKED_CODE_ERROR_MAX_SAMPLES: f64 = 1.0;
const CLEAN_SIGNAL_MIN_LOCKED_CODE_EPOCHS: usize = 5;

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

fn synthetic_frame_with_code_phase(
    config: &ReceiverPipelineConfig,
    sat: SatId,
    code_phase_chips: f64,
    duration_s: f64,
) -> bijux_gnss_core::api::SamplesFrame {
    generate_l1_ca(
        config,
        SyntheticSignalParams {
            sat,
            glonass_frequency_channel: None,
            doppler_hz: 0.0,
            code_phase_chips,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 52.0,
            data_bit_flip: false,
        },
        0xD11C_600D,
        duration_s,
    )
}

fn track_clean_code_case(
    config: &ReceiverPipelineConfig,
    sat: SatId,
    code_phase_chips: f64,
    duration_s: f64,
    seeded_code_phase_samples: usize,
) -> Vec<bijux_gnss_core::api::TrackEpoch> {
    let frame = synthetic_frame_with_code_phase(config, sat, code_phase_chips, duration_s);
    let tracking = TrackingEngine::new(config.clone(), ReceiverRuntime::default());
    let tracks = tracking.track_from_acquisition(
        &frame,
        &[accepted_acquisition(sat, 0.0, seeded_code_phase_samples)],
    );

    tracks.first().expect("track").epochs.clone()
}

#[test]
fn tracking_reduces_seeded_code_phase_error_from_acquisition_scale_offsets() {
    let config = tracking_config();
    let sat = SatId { constellation: Constellation::Gps, prn: 11 };

    for seeded_code_phase_samples in [1_usize, 1_022_usize] {
        let epochs = track_clean_code_case(&config, sat, 0.0, 0.012, seeded_code_phase_samples);
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
            !post_lock_epochs(epochs).is_empty(),
            "tracking never reached a post-pull-in tracking epoch for seeded code phase sample {seeded_code_phase_samples}: epochs={epochs:?}"
        );
        let assumptions = epochs
            .iter()
            .find_map(|epoch| epoch.tracking_assumptions.as_ref())
            .expect("tracking assumptions");
        assert_eq!(assumptions.early_late_spacing_chips, 0.5);
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
        "best_code_rate_hz={} epochs={epochs:?}",
        best_code_rate_hz,
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
    let frame = synthetic_frame_with_code_phase(&config, sat, code_phase_chips, 0.012);
    let expected_code_phase_samples =
        expected_acquisition_code_phase_samples_f64(&config, &frame, code_phase_chips);
    let seeded_code_phase_samples =
        expected_acquisition_code_phase_samples(&config, &frame, code_phase_chips);
    let epochs =
        track_clean_code_case(&config, sat, code_phase_chips, 0.012, seeded_code_phase_samples);

    assert!(epochs.len() >= CLEAN_SIGNAL_MIN_LOCKED_CODE_EPOCHS, "epochs={epochs:?}");
    assert_clean_signal_code_lock(&config, &epochs, expected_code_phase_samples);
}

#[test]
fn tracking_holds_galileo_e1_lock_on_clean_synthetic_signal() {
    let config = galileo_tracking_config();
    let sat = SatId { constellation: Constellation::Galileo, prn: 11 };
    let code_phase_chips = 321.375;
    let frame = synthetic_frame_with_code_phase(&config, sat, code_phase_chips, 0.064);
    let expected_code_phase_samples =
        expected_acquisition_code_phase_samples_f64(&config, &frame, code_phase_chips);
    let seeded_code_phase_samples =
        expected_acquisition_code_phase_samples(&config, &frame, code_phase_chips);
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
    let locked_epochs = post_lock_epochs(epochs);
    let post_lock_errors_samples =
        post_lock_code_phase_errors_samples(&config, epochs, expected_code_phase_samples);

    assert!(
        locked_epochs.len() >= CLEAN_SIGNAL_MIN_LOCKED_CODE_EPOCHS,
        "tracking did not sustain Galileo E1 lock: epochs={epochs:?}",
    );
    assert!(
        locked_epochs
            .iter()
            .all(|epoch| epoch.lock && epoch.dll_lock && epoch.pll_lock && epoch.fll_lock),
        "Galileo E1 post-lock epochs lost lock flags: epochs={epochs:?}",
    );
    assert!(
        post_lock_errors_samples.iter().all(|error_samples| *error_samples <= 2.0),
        "Galileo E1 code tracking drifted after lock: post_lock_errors_samples={post_lock_errors_samples:?}, epochs={epochs:?}",
    );
    assert!(
        epochs.iter().all(|epoch| epoch.tracking_provenance.contains("acq_signal_band=E1")),
        "tracking provenance did not preserve Galileo E1 handoff metadata: epochs={epochs:?}",
    );
}
