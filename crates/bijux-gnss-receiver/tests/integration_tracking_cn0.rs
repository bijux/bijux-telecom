#![allow(missing_docs)]

mod support;

use bijux_gnss_core::api::{
    AcqHypothesis, AcqResult, Constellation, Hertz, ReceiverSampleTrace, SamplesFrame, SatId,
    SignalBand,
};
use bijux_gnss_receiver::api::{
    sim::{
        build_iq16_capture_bundle, generate_l1_ca_multi, validate_truth_guided_cn0,
        SyntheticScenario, SyntheticSignalParams,
    },
    ReceiverPipelineConfig, ReceiverRuntime, TrackingEngine,
};

use support::tracking_truth::{mean_tracking_cn0_dbhz, stable_tracking_cn0_estimates};

const TRACKING_CN0_TOLERANCE_DBHZ: f64 = 4.0;
const TRACKING_CN0_MIN_STABLE_EPOCHS: usize = 5;

#[test]
fn truth_guided_tracking_cn0_matches_injected_truth_across_signal_levels() {
    for sample_rate_hz in [4_092_000.0] {
        for cn0_db_hz in [48.0, 52.0, 58.0] {
            let config = tracking_config(sample_rate_hz);
            let scenario = cn0_scenario(&config, cn0_db_hz, "tracking_cn0_truth_sweep");
            let frame = generate_l1_ca_multi(&config, &scenario);
            let bundle = build_iq16_capture_bundle(
                &scenario.id,
                &scenario,
                &frame,
                "2026-07-10T00:00:00Z",
                Some("tracking cn0 truth sweep".to_string()),
            );
            let scaled_frame = scaled_capture_frame(&frame, bundle.truth.output_scale_applied);

            let report = validate_truth_guided_cn0(
                &config,
                &scaled_frame,
                &bundle.truth,
                TRACKING_CN0_TOLERANCE_DBHZ,
            );

            assert!(report.pass, "{report:?}");
            assert_eq!(report.satellites.len(), 1, "{report:?}");
            let row = &report.satellites[0];
            assert_eq!(row.sat.prn, 7, "{row:?}");
            assert!(row.epochs_measured >= TRACKING_CN0_MIN_STABLE_EPOCHS, "{row:?}");
            assert!(row.cn0_delta_db.abs() <= TRACKING_CN0_TOLERANCE_DBHZ, "{row:?}");
        }
    }
}

#[test]
fn tracking_epoch_cn0_estimates_increase_with_stronger_signal() {
    let config = tracking_config(4_092_000.0);
    let weak_epochs = track_cn0_case(&config, 48.0);
    let strong_epochs = track_cn0_case(&config, 58.0);

    let weak_cn0_values =
        stable_tracking_cn0_estimates(&weak_epochs, TRACKING_CN0_MIN_STABLE_EPOCHS);
    let strong_cn0_values =
        stable_tracking_cn0_estimates(&strong_epochs, TRACKING_CN0_MIN_STABLE_EPOCHS);
    let weak_mean_cn0_dbhz =
        mean_tracking_cn0_dbhz(&weak_epochs, TRACKING_CN0_MIN_STABLE_EPOCHS).expect("weak mean");
    let strong_mean_cn0_dbhz =
        mean_tracking_cn0_dbhz(&strong_epochs, TRACKING_CN0_MIN_STABLE_EPOCHS)
            .expect("strong mean");

    assert!(
        !weak_cn0_values.is_empty() && !strong_cn0_values.is_empty(),
        "missing stable tracking window: weak_epochs={weak_epochs:?} strong_epochs={strong_epochs:?}"
    );
    assert!(
        strong_mean_cn0_dbhz > weak_mean_cn0_dbhz + 4.0,
        "expected stronger signal to produce materially higher tracking C/N0: weak_mean_cn0_dbhz={weak_mean_cn0_dbhz} strong_mean_cn0_dbhz={strong_mean_cn0_dbhz}"
    );
}

fn tracking_config(sample_rate_hz: f64) -> ReceiverPipelineConfig {
    ReceiverPipelineConfig {
        sampling_freq_hz: sample_rate_hz,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 4,
        early_late_spacing_chips: 0.5,
        dll_bw_hz: 2.0,
        pll_bw_hz: 18.0,
        fll_bw_hz: 12.0,
        ..ReceiverPipelineConfig::default()
    }
}

fn cn0_scenario(
    config: &ReceiverPipelineConfig,
    cn0_db_hz: f32,
    scenario_id: &str,
) -> SyntheticScenario {
    SyntheticScenario {
        sample_rate_hz: config.sampling_freq_hz,
        intermediate_freq_hz: config.intermediate_freq_hz,
        receiver_clock_frequency_bias_hz: 0.0,
        duration_s: 0.012,
        seed: 0x710C_A000 + cn0_db_hz.round() as u64,
        satellites: vec![SyntheticSignalParams {
            sat: SatId { constellation: Constellation::Gps, prn: 7 },
            doppler_hz: -750.0,
            code_phase_chips: 211.25,
            carrier_phase_rad: 0.2,
            cn0_db_hz,
            data_bit_flip: false,
        }],
        ephemerides: Vec::new(),
        id: format!("{scenario_id}_{:.0}_{:.0}", config.sampling_freq_hz, cn0_db_hz),
    }
}

fn scaled_capture_frame(frame: &SamplesFrame, output_scale_applied: f32) -> SamplesFrame {
    SamplesFrame::new(
        frame.t0,
        frame.dt_s,
        frame.iq.iter().map(|sample| *sample * output_scale_applied).collect(),
    )
}

fn track_cn0_case(
    config: &ReceiverPipelineConfig,
    cn0_db_hz: f32,
) -> Vec<bijux_gnss_core::api::TrackEpoch> {
    let scenario = cn0_scenario(config, cn0_db_hz, "tracking_cn0_epoch_case");
    let frame = generate_l1_ca_multi(config, &scenario);
    let sat = scenario.satellites[0].sat;
    let period_samples = ((config.sampling_freq_hz * config.code_length as f64)
        / config.code_freq_basis_hz)
        .round() as usize;
    let injected_sample = (scenario.satellites[0].code_phase_chips * config.sampling_freq_hz
        / config.code_freq_basis_hz)
        .round() as usize
        % period_samples.max(1);
    let seeded_code_phase_samples =
        (period_samples.max(1) - injected_sample) % period_samples.max(1);
    let tracking = TrackingEngine::new(config.clone(), ReceiverRuntime::default());
    let tracks = tracking.track_from_acquisition(
        &frame,
        &[accepted_acquisition(
            sat,
            scenario.satellites[0].doppler_hz,
            seeded_code_phase_samples,
            cn0_db_hz,
        )],
    );

    tracks.first().expect("track").epochs.clone()
}

fn accepted_acquisition(
    sat: SatId,
    doppler_hz: f64,
    code_phase_samples: usize,
    cn0_db_hz: f32,
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
        cn0_proxy: cn0_db_hz,
        score: 1.0,
        hypothesis: AcqHypothesis::Accepted,
        assumptions: None,
        evidence: Vec::new(),
        threshold_provenance: None,
        explain_selection_reason: Some("tracking_cn0_seed".to_string()),
        doppler_refinement: None,
        code_phase_refinement: None,
        uncertainty: None,
    }
}
