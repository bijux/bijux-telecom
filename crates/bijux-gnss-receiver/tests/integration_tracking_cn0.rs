#![allow(missing_docs)]

mod support;

use bijux_gnss_core::api::{Constellation, SamplesFrame, SatId};
use bijux_gnss_receiver::api::{
    sim::{
        build_iq16_capture_bundle, generate_l1_ca_multi, validate_truth_guided_cn0,
        SyntheticScenario, SyntheticSignalParams,
    },
    ReceiverPipelineConfig,
};

const TRACKING_CN0_TOLERANCE_DBHZ: f64 = 4.5;
const TRACKING_CN0_MIN_STABLE_EPOCHS: usize = 5;
const TRACKING_CN0_MIN_SIGNAL_SEPARATION_DB: f64 = 3.5;

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
fn truth_guided_tracking_cn0_increases_with_stronger_signal() {
    let config = tracking_config(4_092_000.0);
    let weak = cn0_report(&config, 48.0);
    let strong = cn0_report(&config, 58.0);
    let weak_row = &weak.satellites[0];
    let strong_row = &strong.satellites[0];

    assert!(weak.pass, "{weak:?}");
    assert!(strong.pass, "{strong:?}");
    assert!(
        strong_row.measured_mean_cn0_dbhz
            > weak_row.measured_mean_cn0_dbhz + TRACKING_CN0_MIN_SIGNAL_SEPARATION_DB,
        "expected stronger signal to produce materially higher tracked C/N0: weak_row={weak_row:?} strong_row={strong_row:?}"
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
        duration_s: 0.05,
        seed: 17,
        satellites: vec![SyntheticSignalParams {
            sat: SatId { constellation: Constellation::Gps, prn: 7 },
            glonass_frequency_channel: None,
            signal_band: bijux_gnss_core::api::SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Unknown,
            doppler_hz: -1000.0,
            code_phase_chips: 321.0,
            carrier_phase_rad: 0.2,
            cn0_db_hz,
            navigation_data: false.into(),
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

fn cn0_report(
    config: &ReceiverPipelineConfig,
    cn0_db_hz: f32,
) -> bijux_gnss_receiver::api::sim::SyntheticCn0ValidationReport {
    let scenario = cn0_scenario(config, cn0_db_hz, "tracking_cn0_epoch_case");
    let frame = generate_l1_ca_multi(config, &scenario);
    let bundle = build_iq16_capture_bundle(
        &scenario.id,
        &scenario,
        &frame,
        "2026-07-10T00:00:00Z",
        Some("tracking cn0 epoch case".to_string()),
    );
    let scaled_frame = scaled_capture_frame(&frame, bundle.truth.output_scale_applied);
    validate_truth_guided_cn0(config, &scaled_frame, &bundle.truth, TRACKING_CN0_TOLERANCE_DBHZ)
}
