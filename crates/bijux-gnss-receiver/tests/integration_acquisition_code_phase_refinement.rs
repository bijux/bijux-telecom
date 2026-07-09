#![allow(missing_docs)]

use bijux_gnss_core::api::{Constellation, SamplesFrame, SatId};
use bijux_gnss_receiver::api::{
    sim::{
        build_iq16_capture_bundle, generate_l1_ca_multi,
        validate_truth_guided_acquisition_code_phase_refinement, SyntheticScenario,
        SyntheticSignalParams,
    },
    ReceiverPipelineConfig,
};

#[test]
fn acquisition_refinement_improves_fractional_code_phase_initialization() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_000_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let scenario = SyntheticScenario {
        sample_rate_hz: config.sampling_freq_hz,
        intermediate_freq_hz: config.intermediate_freq_hz,
        duration_s: 0.04,
        seed: 2_407_1985,
        satellites: vec![
            SyntheticSignalParams {
                sat: SatId { constellation: Constellation::Gps, prn: 3 },
                doppler_hz: 0.0,
                code_phase_chips: 200.25,
                carrier_phase_rad: 0.0,
                cn0_db_hz: 65.0,
                data_bit_flip: false,
            },
            SyntheticSignalParams {
                sat: SatId { constellation: Constellation::Gps, prn: 7 },
                doppler_hz: 0.0,
                code_phase_chips: 321.5,
                carrier_phase_rad: 0.2,
                cn0_db_hz: 63.0,
                data_bit_flip: false,
            },
        ],
        ephemerides: Vec::new(),
        id: "acquisition_code_phase_refinement_fractional_rate".to_string(),
    };
    let frame = generate_l1_ca_multi(&config, &scenario);
    let bundle = build_iq16_capture_bundle(
        &scenario.id,
        &scenario,
        &frame,
        "2026-07-09T00:00:00Z",
        Some("integration acquisition code-phase refinement".to_string()),
    );
    let scaled_frame = scaled_frame(&frame, bundle.truth.output_scale_applied);

    let report = validate_truth_guided_acquisition_code_phase_refinement(
        &config,
        &scaled_frame,
        &bundle.truth,
    );

    assert!(report.pass, "{report:?}");
    assert_eq!(report.sample_rate_hz, config.sampling_freq_hz);
    assert_eq!(report.satellites.len(), 2);
    for row in &report.satellites {
        assert!(row.pass, "{row:?}");
        assert!(row.improvement_samples > 0.0, "{row:?}");
        assert!(row.improvement_m > 0.0, "{row:?}");
        assert!(row.refined_error_samples < row.coarse_error_samples, "{row:?}");
        assert!(matches!(row.hypothesis.as_str(), "accepted" | "ambiguous"));
    }
}

fn scaled_frame(frame: &SamplesFrame, output_scale_applied: f32) -> SamplesFrame {
    SamplesFrame::new(
        frame.t0,
        frame.dt_s,
        frame.iq.iter().map(|sample| *sample * output_scale_applied).collect(),
    )
}
