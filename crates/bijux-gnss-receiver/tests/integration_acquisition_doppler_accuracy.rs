#![allow(missing_docs)]

use bijux_gnss_core::api::{Constellation, SamplesFrame, SatId};
use bijux_gnss_receiver::api::{
    sim::{
        build_iq16_capture_bundle, generate_l1_ca_multi, validate_truth_guided_acquisition_doppler,
        SyntheticScenario, SyntheticSignalParams,
    },
    ReceiverPipelineConfig,
};

#[test]
fn acquisition_recovers_clean_synthetic_doppler_within_one_bin_at_multiple_sample_rates() {
    for (sampling_freq_hz, scenario_id) in [
        (4_092_000.0, "acquisition_doppler_accuracy_high_rate"),
        (4_000_000.0, "acquisition_doppler_accuracy_fractional_rate"),
    ] {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            acquisition_doppler_search_hz: 10_000,
            acquisition_doppler_step_hz: 500,
            ..ReceiverPipelineConfig::default()
        };
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
                    doppler_hz: 750.0,
                    code_phase_chips: 200.25,
                    carrier_phase_rad: 0.0,
                    cn0_db_hz: 58.0,
                    data_bit_flip: true,
                },
                SyntheticSignalParams {
                    sat: SatId { constellation: Constellation::Gps, prn: 7 },
                    glonass_frequency_channel: None,
                    doppler_hz: -1_000.0,
                    code_phase_chips: 321.5,
                    carrier_phase_rad: 0.2,
                    cn0_db_hz: 52.0,
                    data_bit_flip: false,
                },
            ],
            ephemerides: Vec::new(),
            id: scenario_id.to_string(),
        };
        let frame = generate_l1_ca_multi(&config, &scenario);
        let bundle = build_iq16_capture_bundle(
            &scenario.id,
            &scenario,
            &frame,
            "2026-07-09T00:00:00Z",
            Some("integration acquisition doppler accuracy".to_string()),
        );
        let scaled_frame = scaled_frame(&frame, bundle.truth.output_scale_applied);

        let report =
            validate_truth_guided_acquisition_doppler(&config, &scaled_frame, &bundle.truth, 1);

        assert!(report.pass, "{report:?}");
        assert_eq!(report.sample_rate_hz, sampling_freq_hz);
        assert_eq!(report.doppler_step_hz, 500);
        assert_eq!(report.satellites.len(), 2);
        for row in &report.satellites {
            assert!(row.pass, "{row:?}");
            assert!(row.doppler_error_hz <= 500.0 + f64::EPSILON, "{row:?}");
            assert!(row.doppler_error_bins <= 1.0 + f64::EPSILON, "{row:?}");
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
