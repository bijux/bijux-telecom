#![allow(missing_docs)]

use bijux_gnss_core::api::{Constellation, SamplesFrame, SatId};
use bijux_gnss_receiver::api::{
    sim::{
        build_iq16_capture_bundle, generate_l1_ca_multi,
        validate_truth_guided_acquisition_coherent_integration, SyntheticScenario,
        SyntheticSignalParams,
    },
    ReceiverPipelineConfig,
};

#[test]
fn acquisition_supports_coherent_lengths_without_nav_bit_modulation() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        acquisition_doppler_search_hz: 10_000,
        acquisition_doppler_step_hz: 500,
        ..ReceiverPipelineConfig::default()
    };

    for coherent_ms in [1, 2, 5, 10, 20] {
        let scenario = SyntheticScenario {
            sample_rate_hz: config.sampling_freq_hz,
            intermediate_freq_hz: config.intermediate_freq_hz,
            receiver_clock_frequency_bias_hz: 0.0,
            duration_s: 0.04,
            seed: 2_407_1985,
            satellites: vec![
                SyntheticSignalParams {
                    sat: SatId { constellation: Constellation::Gps, prn: 3 },
                    glonass_frequency_channel: None,
                    signal_band: bijux_gnss_core::api::SignalBand::L1,
                    doppler_hz: 750.0,
                    code_phase_chips: 200.25,
                    carrier_phase_rad: 0.0,
                    cn0_db_hz: 58.0,
                    data_bit_flip: false,
                },
                SyntheticSignalParams {
                    sat: SatId { constellation: Constellation::Gps, prn: 7 },
                    glonass_frequency_channel: None,
                    signal_band: bijux_gnss_core::api::SignalBand::L1,
                    doppler_hz: -1_000.0,
                    code_phase_chips: 321.5,
                    carrier_phase_rad: 0.2,
                    cn0_db_hz: 52.0,
                    data_bit_flip: false,
                },
            ],
            ephemerides: Vec::new(),
            id: format!("acquisition_coherent_integration_constant_bits_{coherent_ms}ms"),
        };
        let frame = generate_l1_ca_multi(&config, &scenario);
        let bundle = build_iq16_capture_bundle(
            &scenario.id,
            &scenario,
            &frame,
            "2026-07-09T00:00:00Z",
            Some("integration acquisition coherent integration constant bits".to_string()),
        );
        let scaled_frame = scaled_frame(&frame, bundle.truth.output_scale_applied);
        let doppler_tolerance_bins = if coherent_ms >= 20 { 5 } else { 1 };

        let report = validate_truth_guided_acquisition_coherent_integration(
            &config,
            &scaled_frame,
            &bundle.truth,
            coherent_ms,
            1,
            2,
            doppler_tolerance_bins,
        );

        assert!(report.pass, "{report:?}");
        assert_eq!(report.coherent_ms, coherent_ms);
        assert_eq!(report.noncoherent, 1);
        assert_eq!(report.satellites.len(), 2);
        for row in &report.satellites {
            assert!(row.pass, "{row:?}");
            assert!(row.code_phase_error_samples <= 2, "{row:?}");
            assert!(row.doppler_error_bins <= 1.0 + f64::EPSILON, "{row:?}");
            assert!(matches!(row.hypothesis.as_str(), "accepted" | "ambiguous"));
        }
    }
}

#[test]
fn acquisition_supports_coherent_lengths_with_nav_bit_modulation() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        acquisition_doppler_search_hz: 10_000,
        acquisition_doppler_step_hz: 500,
        ..ReceiverPipelineConfig::default()
    };

    for coherent_ms in [1, 2, 5, 10, 20] {
        let scenario = SyntheticScenario {
            sample_rate_hz: config.sampling_freq_hz,
            intermediate_freq_hz: config.intermediate_freq_hz,
            receiver_clock_frequency_bias_hz: 0.0,
            duration_s: 0.04,
            seed: 2_407_1986,
            satellites: vec![
                SyntheticSignalParams {
                    sat: SatId { constellation: Constellation::Gps, prn: 5 },
                    glonass_frequency_channel: None,
                    signal_band: bijux_gnss_core::api::SignalBand::L1,
                    doppler_hz: 500.0,
                    code_phase_chips: 145.375,
                    carrier_phase_rad: 0.0,
                    cn0_db_hz: 58.0,
                    data_bit_flip: true,
                },
                SyntheticSignalParams {
                    sat: SatId { constellation: Constellation::Gps, prn: 11 },
                    glonass_frequency_channel: None,
                    signal_band: bijux_gnss_core::api::SignalBand::L1,
                    doppler_hz: -750.0,
                    code_phase_chips: 278.625,
                    carrier_phase_rad: 0.15,
                    cn0_db_hz: 54.0,
                    data_bit_flip: true,
                },
            ],
            ephemerides: Vec::new(),
            id: format!("acquisition_coherent_integration_nav_bits_{coherent_ms}ms"),
        };
        let frame = generate_l1_ca_multi(&config, &scenario);
        let bundle = build_iq16_capture_bundle(
            &scenario.id,
            &scenario,
            &frame,
            "2026-07-09T00:00:00Z",
            Some("integration acquisition coherent integration nav bits".to_string()),
        );
        let scaled_frame = scaled_frame(&frame, bundle.truth.output_scale_applied);
        let doppler_tolerance_bins = if coherent_ms >= 20 { 5 } else { 1 };

        let report = validate_truth_guided_acquisition_coherent_integration(
            &config,
            &scaled_frame,
            &bundle.truth,
            coherent_ms,
            1,
            2,
            doppler_tolerance_bins,
        );

        assert!(report.pass, "{report:?}");
        assert_eq!(report.coherent_ms, coherent_ms);
        assert_eq!(report.noncoherent, 1);
        assert_eq!(report.satellites.len(), 2);
        for row in &report.satellites {
            assert!(row.pass, "{row:?}");
            assert!(row.code_phase_error_samples <= 2, "{row:?}");
            assert!(
                row.doppler_error_bins <= doppler_tolerance_bins as f64 + f64::EPSILON,
                "{row:?}"
            );
            assert!(matches!(row.hypothesis.as_str(), "accepted" | "ambiguous"));
        }
    }
}

fn scaled_frame(frame: &SamplesFrame, output_scale_applied: f32) -> SamplesFrame {
    SamplesFrame::new(
        frame.t0,
        frame.dt_s,
        frame.iq.iter().map(|sample| *sample * output_scale_applied).collect(),
    )
}
