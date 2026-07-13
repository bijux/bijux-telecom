#![allow(missing_docs)]

use bijux_gnss_core::api::{Constellation, SamplesFrame, SatId};
use bijux_gnss_receiver::api::{
    sim::{
        build_iq16_capture_bundle, generate_l1_ca_multi, validate_truth_guided_acquisition_doppler,
        validate_truth_guided_acquisition_receiver_clock_offset, SyntheticScenario,
        SyntheticSignalParams,
    },
    ReceiverPipelineConfig,
};

#[test]
fn acquisition_reports_consistent_receiver_clock_offset_across_satellites() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        acquisition_doppler_search_hz: 10_000,
        acquisition_doppler_step_hz: 250,
        ..ReceiverPipelineConfig::default()
    };
    let scenario = SyntheticScenario {
        sample_rate_hz: config.sampling_freq_hz,
        intermediate_freq_hz: config.intermediate_freq_hz,
        receiver_clock_frequency_bias_hz: 500.0,
        duration_s: 0.04,
        seed: 2_407_1987,
        satellites: vec![
            SyntheticSignalParams {
                sat: SatId { constellation: Constellation::Gps, prn: 3 },
                glonass_frequency_channel: None,
                signal_band: bijux_gnss_core::api::SignalBand::L1,
                signal_code: bijux_gnss_core::api::SignalCode::Unknown,
                doppler_hz: 750.0,
                code_phase_chips: 200.25,
                carrier_phase_rad: 0.0,
                cn0_db_hz: 58.0,
                navigation_data: true.into(),
            },
            SyntheticSignalParams {
                sat: SatId { constellation: Constellation::Gps, prn: 7 },
                glonass_frequency_channel: None,
                signal_band: bijux_gnss_core::api::SignalBand::L1,
                signal_code: bijux_gnss_core::api::SignalCode::Unknown,
                doppler_hz: -1_000.0,
                code_phase_chips: 321.5,
                carrier_phase_rad: 0.2,
                cn0_db_hz: 52.0,
                navigation_data: false.into(),
            },
        ],
        ephemerides: Vec::new(),
        id: "integration_acquisition_receiver_clock_offset".to_string(),
    };
    let frame = generate_l1_ca_multi(&config, &scenario);
    let bundle = build_iq16_capture_bundle(
        &scenario.id,
        &scenario,
        &frame,
        "2026-07-09T00:00:00Z",
        Some("integration acquisition receiver clock offset".to_string()),
    );
    let scaled_frame = scaled_frame(&frame, bundle.truth.output_scale_applied);

    let report = validate_truth_guided_acquisition_receiver_clock_offset(
        &config,
        &scaled_frame,
        &bundle.truth,
        1,
    );
    let doppler_report =
        validate_truth_guided_acquisition_doppler(&config, &scaled_frame, &bundle.truth, 1);

    assert!(report.pass, "{report:?}");
    assert!(doppler_report.pass, "{doppler_report:?}");
    assert_eq!(report.sample_rate_hz, 4_092_000.0);
    assert_eq!(report.injected_receiver_clock_frequency_bias_hz, 500.0);
    assert_eq!(report.tolerance_hz, 250.0);
    assert_eq!(report.satellites.len(), 2);
    assert_eq!(doppler_report.satellites.len(), 2);
    assert!(
        (report.mean_measured_receiver_clock_frequency_bias_hz - 500.0).abs() <= 250.0,
        "{report:?}"
    );
    assert!(report.measured_receiver_clock_frequency_bias_spread_hz <= 250.0, "{report:?}");
    for row in &doppler_report.satellites {
        assert!(row.pass, "{row:?}");
        assert!(
            (row.expected_measured_doppler_hz - row.measured_doppler_hz).abs() <= 250.0,
            "{row:?}"
        );
    }
    for row in &report.satellites {
        assert!(row.pass, "{row:?}");
        assert_eq!(row.injected_receiver_clock_frequency_bias_hz, 500.0);
        assert!((row.measured_receiver_clock_frequency_bias_hz - 500.0).abs() <= 250.0, "{row:?}");
        assert!(
            (row.expected_measured_doppler_hz - row.measured_doppler_hz).abs() <= 250.0,
            "{row:?}"
        );
    }
}

fn scaled_frame(frame: &SamplesFrame, output_scale_applied: f32) -> SamplesFrame {
    SamplesFrame::new(
        frame.t0,
        frame.dt_s,
        frame.iq.iter().map(|sample| *sample * output_scale_applied).collect(),
    )
}
