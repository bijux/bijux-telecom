#![allow(missing_docs)]

mod support;

use bijux_gnss_core::api::{
    AcqHypothesis, AcqResult, Constellation, Hertz, ReceiverSampleTrace, SampleTime, SamplesFrame,
    SatId, Seconds, SignalBand,
};
use bijux_gnss_receiver::api::{
    sim::{generate_l1_ca, SyntheticSignalParams},
    ReceiverPipelineConfig, ReceiverRuntime, TrackingEngine,
};
use bijux_gnss_signal::api::{sample_ca_code, samples_per_code, Prn};
use num_complex::Complex;

use support::tracking_truth::wrapped_code_phase_error_samples;

fn accepted_acquisition(
    sat: SatId,
    doppler_hz: f64,
    code_phase_samples: usize,
) -> AcqResult {
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
        cn0_proxy: 52.0,
        score: 1.0,
        hypothesis: AcqHypothesis::Accepted,
        assumptions: None,
        evidence: Vec::new(),
        threshold_provenance: None,
        explain_selection_reason: Some("seeded_tracking_start".to_string()),
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

fn synthetic_frame(config: &ReceiverPipelineConfig, sat: SatId) -> bijux_gnss_core::api::SamplesFrame {
    generate_l1_ca(
        config,
        SyntheticSignalParams {
            sat,
            doppler_hz: 0.0,
            code_phase_chips: 0.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 52.0,
            data_bit_flip: false,
        },
        0xD11C_600D,
        0.012,
    )
}

#[test]
fn tracking_reduces_seeded_code_phase_error_from_acquisition_scale_offsets() {
    let config = tracking_config();
    let sat = SatId { constellation: Constellation::Gps, prn: 11 };
    let frame = synthetic_frame(&config, sat);
    let tracking = TrackingEngine::new(config.clone(), ReceiverRuntime::default());

    for seeded_code_phase_samples in [1_usize, 1_022_usize] {
        let tracks =
            tracking.track_from_acquisition(&frame, &[accepted_acquisition(sat, 0.0, seeded_code_phase_samples)]);
        let epochs = &tracks.first().expect("track").epochs;
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
    let sample_count = samples_per_code(
        config.sampling_freq_hz,
        config.code_freq_basis_hz,
        config.code_length,
    ) * 12;
    let signal_code_rate_hz = config.code_freq_basis_hz + 300.0;
    let code_phase_chips = 144.25;
    let code_phase_samples =
        code_phase_chips * config.sampling_freq_hz / config.code_freq_basis_hz;
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
    let tracking = TrackingEngine::new(config.clone(), ReceiverRuntime::default());

    let tracks = tracking.track_from_acquisition(
        &frame,
        &[accepted_acquisition(sat, 0.0, code_phase_samples.round() as usize)],
    );
    let epochs = &tracks.first().expect("track").epochs;
    let last_epoch = epochs.last().expect("last tracking epoch");

    assert!(
        last_epoch.code_rate_hz.0 > config.code_freq_basis_hz,
        "last_code_rate_hz={} epochs={epochs:?}",
        last_epoch.code_rate_hz.0,
    );
}
