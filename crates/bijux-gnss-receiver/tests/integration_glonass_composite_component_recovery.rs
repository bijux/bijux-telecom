#![allow(missing_docs)]

use bijux_gnss_core::api::{
    glonass_slot_sat, Constellation, GlonassFrequencyChannel, GlonassSlot, SamplesFrame,
    SignalBand, SignalCode, GPS_L1_CA_CARRIER_HZ,
};
use bijux_gnss_receiver::api::{
    sim::{
        build_iq16_capture_bundle, generate_l1_ca_multi,
        validate_truth_guided_composite_component_recovery, SyntheticScenario,
        SyntheticSignalParams,
    },
    ReceiverPipelineConfig,
};
use bijux_gnss_signal::api::glonass_l1_carrier_hz;

const GLONASS_POWER_TOLERANCE_DB: f64 = 0.50;
const GLONASS_PHASE_TOLERANCE_RAD: f64 = 0.08;

#[test]
fn composite_component_recovery_keeps_glonass_fdma_channels_distinct() {
    let lower_slot = GlonassSlot::new(8).expect("slot 8 must be valid");
    let upper_slot = GlonassSlot::new(12).expect("slot 12 must be valid");
    let lower_sat = glonass_slot_sat(lower_slot);
    let upper_sat = glonass_slot_sat(upper_slot);
    let lower_channel = GlonassFrequencyChannel::new(-4).expect("channel -4 must be valid");
    let upper_channel = GlonassFrequencyChannel::new(2).expect("channel 2 must be valid");
    let reference_channel = GlonassFrequencyChannel::new(-1).expect("channel -1 must be valid");
    let config = glonass_composite_config(reference_channel);
    let scenario = glonass_composite_scenario(
        lower_sat,
        lower_channel,
        upper_sat,
        upper_channel,
        reference_channel,
    );
    let frame = generate_l1_ca_multi(&config, &scenario);
    let bundle = build_iq16_capture_bundle(
        &scenario.id,
        &scenario,
        &frame,
        "2026-07-13T00:00:00Z",
        Some("glonass fdma composite component recovery".to_string()),
    );
    let scaled_frame = scaled_capture_frame(&frame, bundle.truth.output_scale_applied);

    let report = validate_truth_guided_composite_component_recovery(
        &config,
        &scaled_frame,
        &bundle.truth,
        GLONASS_POWER_TOLERANCE_DB,
        GLONASS_PHASE_TOLERANCE_RAD,
    );

    assert!(report.pass, "{report:?}");
    assert_eq!(report.solver_status, "ok");
    assert_eq!(report.satellites.len(), 2, "{report:?}");
    assert!(
        report
            .satellites
            .iter()
            .all(|row| row.sat.constellation == Constellation::Glonass && row.pass),
        "{report:?}"
    );

    let lower_row =
        report.satellites.iter().find(|row| row.sat == lower_sat).expect("lower GLONASS row");
    let upper_row =
        report.satellites.iter().find(|row| row.sat == upper_sat).expect("upper GLONASS row");
    assert_eq!(lower_row.glonass_frequency_channel, Some(lower_channel), "{lower_row:?}");
    assert_eq!(upper_row.glonass_frequency_channel, Some(upper_channel), "{upper_row:?}");
    assert_ne!(lower_row.glonass_frequency_channel, upper_row.glonass_frequency_channel);
}

fn glonass_composite_config(reference_channel: GlonassFrequencyChannel) -> ReceiverPipelineConfig {
    ReceiverPipelineConfig {
        sampling_freq_hz: 8_176_000.0,
        intermediate_freq_hz: GPS_L1_CA_CARRIER_HZ.value()
            - glonass_l1_carrier_hz(reference_channel).value(),
        code_freq_basis_hz: 511_000.0,
        code_length: 511,
        channels: 4,
        ..ReceiverPipelineConfig::default()
    }
}

fn glonass_composite_scenario(
    lower_sat: bijux_gnss_core::api::SatId,
    lower_channel: GlonassFrequencyChannel,
    upper_sat: bijux_gnss_core::api::SatId,
    upper_channel: GlonassFrequencyChannel,
    reference_channel: GlonassFrequencyChannel,
) -> SyntheticScenario {
    SyntheticScenario {
        sample_rate_hz: 8_176_000.0,
        intermediate_freq_hz: GPS_L1_CA_CARRIER_HZ.value()
            - glonass_l1_carrier_hz(reference_channel).value(),
        receiver_clock_frequency_bias_hz: 45.0,
        duration_s: 0.08,
        seed: 0x6100_2590,
        satellites: vec![
            SyntheticSignalParams {
                sat: lower_sat,
                glonass_frequency_channel: Some(lower_channel),
                signal_band: SignalBand::L1,
                signal_code: SignalCode::Unknown,
                doppler_hz: 350.0,
                code_phase_chips: 147.25,
                carrier_phase_rad: 0.1,
                cn0_db_hz: 60.0,
                navigation_data: true.into(),
            },
            SyntheticSignalParams {
                sat: upper_sat,
                glonass_frequency_channel: Some(upper_channel),
                signal_band: SignalBand::L1,
                signal_code: SignalCode::Unknown,
                doppler_hz: -280.0,
                code_phase_chips: 311.5,
                carrier_phase_rad: -0.2,
                cn0_db_hz: 59.0,
                navigation_data: false.into(),
            },
        ],
        ephemerides: Vec::new(),
        id: "glonass_fdma_component_recovery".to_string(),
    }
}

fn scaled_capture_frame(frame: &SamplesFrame, output_scale_applied: f32) -> SamplesFrame {
    SamplesFrame::new(
        frame.t0,
        frame.dt_s,
        frame.iq.iter().map(|sample| *sample * output_scale_applied).collect(),
    )
}
