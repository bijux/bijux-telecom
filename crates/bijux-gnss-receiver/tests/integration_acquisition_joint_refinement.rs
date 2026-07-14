#![allow(missing_docs)]

use bijux_gnss_core::api::{Constellation, SamplesFrame, SatId};
use bijux_gnss_receiver::api::{
    sim::{
        build_iq16_capture_bundle, generate_l1_ca_multi,
        validate_truth_guided_acquisition_joint_refinement, SyntheticScenario,
        SyntheticSignalParams,
    },
    ReceiverPipelineConfig,
};

#[test]
fn acquisition_joint_refinement_improves_code_phase_and_doppler_across_fractional_matrix() {
    for (sampling_freq_hz, doppler_hz, code_phase_chips, scenario_id) in [
        (4_000_000.0, 875.0, 200.125, "joint_refinement_low_rate_leading_phase"),
        (4_000_000.0, -1_125.0, 321.375, "joint_refinement_low_rate_trailing_phase"),
        (4_000_000.0, 1_375.0, 455.625, "joint_refinement_low_rate_high_doppler"),
        (4_092_000.0, 875.0, 200.0625, "joint_refinement_high_rate_leading_phase"),
        (4_092_000.0, -1_125.0, 321.1875, "joint_refinement_high_rate_trailing_phase"),
        (4_092_000.0, 1_375.0, 455.3125, "joint_refinement_high_rate_high_doppler"),
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
            seed: 24_071_985,
            satellites: vec![SyntheticSignalParams {
                sat: SatId { constellation: Constellation::Gps, prn: 3 },
                glonass_frequency_channel: None,
                signal_band: bijux_gnss_core::api::SignalBand::L1,
                signal_code: bijux_gnss_core::api::SignalCode::Ca,
                doppler_hz,
                code_phase_chips,
                carrier_phase_rad: 0.0,
                cn0_db_hz: 60.0,
                navigation_data: false.into(),
            }],
            ephemerides: Vec::new(),
            id: scenario_id.to_string(),
        };
        let frame = generate_l1_ca_multi(&config, &scenario);
        let bundle = build_iq16_capture_bundle(
            &scenario.id,
            &scenario,
            &frame,
            "2026-07-14T00:00:00Z",
            Some("integration acquisition joint refinement".to_string()),
        );
        let scaled_frame = scaled_frame(&frame, bundle.truth.output_scale_applied);

        let report = validate_truth_guided_acquisition_joint_refinement(
            &config,
            &scaled_frame,
            &bundle.truth,
        );

        assert!(report.pass, "{report:?}");
        assert_eq!(report.sample_rate_hz, sampling_freq_hz);
        assert_eq!(report.doppler_step_hz, 500);
        assert_eq!(report.satellites.len(), 1);
        let row = &report.satellites[0];
        assert!(row.pass, "{row:?}");
        assert!(matches!(row.hypothesis.as_str(), "accepted" | "ambiguous"));
        assert!(
            row.refined_doppler_error_hz + f64::EPSILON < row.coarse_doppler_error_hz,
            "{row:?}"
        );
        assert!(
            row.refined_code_phase_error_samples + f64::EPSILON
                < row.coarse_code_phase_error_samples,
            "{row:?}"
        );
        assert!(row.doppler_improvement_hz > 0.0, "{row:?}");
        assert!(row.code_phase_improvement_samples > 0.0, "{row:?}");
    }
}

fn scaled_frame(frame: &SamplesFrame, output_scale_applied: f32) -> SamplesFrame {
    SamplesFrame::new(
        frame.t0,
        frame.dt_s,
        frame.iq.iter().map(|sample| *sample * output_scale_applied).collect(),
    )
}
