#![allow(missing_docs)]

use bijux_gnss_core::api::{Constellation, SatId, SignalBand, SignalCode};
use bijux_gnss_receiver::api::{
    sim::{validate_truth_guided_acquisition_table, SyntheticScenario, SyntheticSignalParams},
    ReceiverPipelineConfig,
};

#[test]
fn gps_l5_acquisition_reports_signal_relative_doppler() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 10_230_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 10_230_000.0,
        code_length: 10_230,
        acquisition_doppler_search_hz: 2_000,
        acquisition_doppler_step_hz: 250,
        acquisition_integration_ms: 1,
        acquisition_noncoherent: 1,
        ..ReceiverPipelineConfig::default()
    };
    let scenario = SyntheticScenario {
        sample_rate_hz: config.sampling_freq_hz,
        intermediate_freq_hz: config.intermediate_freq_hz,
        receiver_clock_frequency_bias_hz: 0.0,
        duration_s: 0.040,
        seed: 0x2810_0103,
        satellites: vec![SyntheticSignalParams {
            sat: SatId { constellation: Constellation::Gps, prn: 21 },
            glonass_frequency_channel: None,
            signal_band: SignalBand::L5,
            signal_code: SignalCode::L5I,
            doppler_hz: 875.0,
            code_phase_chips: 2_048.25,
            carrier_phase_rad: 0.4,
            cn0_db_hz: 58.0,
            navigation_data: false.into(),
        }],
        ephemerides: Vec::new(),
        id: "integration-acquisition-wideband-doppler-frame".to_string(),
    };
    let frame = bijux_gnss_receiver::api::sim::generate_l1_ca_multi(&config, &scenario);
    let truth = bijux_gnss_receiver::api::sim::build_iq16_capture_bundle(
        &scenario.id,
        &scenario,
        &frame,
        "2026-07-14T00:00:00Z",
        Some("integration acquisition wideband doppler frame".to_string()),
    )
    .truth;
    let scaled_frame = bijux_gnss_core::api::SamplesFrame::new(
        frame.t0,
        frame.dt_s,
        frame.iq.iter().map(|sample| *sample * truth.output_scale_applied).collect(),
    );

    let report = validate_truth_guided_acquisition_table(&config, &scaled_frame, &truth, 1, 3);

    assert!(report.pass, "{report:#?}");
    assert_eq!(report.satellites.len(), 1);
    let satellite = &report.satellites[0];
    assert!(satellite.pass, "{satellite:#?}");
    assert!(satellite.doppler_pass, "{satellite:#?}");
    assert!(satellite.code_phase_pass, "{satellite:#?}");
    assert_eq!(satellite.injected_doppler_hz, 875.0);
}
