#![allow(missing_docs)]

use bijux_gnss_core::api::{
    glonass_l1_carrier_hz, glonass_slot_sat, AcqRequest, Constellation, GlonassFrequencyChannel,
    GlonassSlot, SignalBand, SignalCode, SupportStatus, GPS_L1_CA_CARRIER_HZ,
};
use bijux_gnss_receiver::api::{
    sim::{SyntheticScenario, SyntheticSignalParams, SyntheticSignalSource},
    Receiver, ReceiverPipelineConfig, ReceiverRuntime,
};

fn glonass_boundary_config(channel: GlonassFrequencyChannel) -> ReceiverPipelineConfig {
    ReceiverPipelineConfig {
        sampling_freq_hz: 2_044_000.0,
        intermediate_freq_hz: GPS_L1_CA_CARRIER_HZ.value() - glonass_l1_carrier_hz(channel).value(),
        code_freq_basis_hz: 511_000.0,
        code_length: 511,
        acquisition_doppler_search_hz: 0,
        acquisition_doppler_step_hz: 250,
        acquisition_integration_ms: 1,
        acquisition_noncoherent: 1,
        channels: 2,
        ..ReceiverPipelineConfig::default()
    }
}

fn glonass_boundary_scenario(
    sat: bijux_gnss_core::api::SatId,
    channel: GlonassFrequencyChannel,
) -> SyntheticScenario {
    SyntheticScenario {
        sample_rate_hz: 2_044_000.0,
        intermediate_freq_hz: GPS_L1_CA_CARRIER_HZ.value() - glonass_l1_carrier_hz(channel).value(),
        receiver_clock_frequency_bias_hz: 0.0,
        duration_s: 0.010,
        seed: 0x6100_0000,
        satellites: vec![SyntheticSignalParams {
            sat,
            glonass_frequency_channel: Some(channel),
            doppler_hz: 0.0,
            code_phase_chips: 147.25,
            carrier_phase_rad: 0.5,
            cn0_db_hz: 60.0,
            data_bit_flip: false,
        }],
        ephemerides: Vec::new(),
        id: "receiver-glonass-l1-boundary".to_string(),
    }
}

#[test]
fn receiver_run_with_explicit_glonass_requests_reports_acquisition() {
    let slot = GlonassSlot::new(8).expect("slot 8 must be valid");
    let sat = glonass_slot_sat(slot);
    let channel = GlonassFrequencyChannel::new(-4).expect("channel -4 must be valid");
    let config = glonass_boundary_config(channel);
    let scenario = glonass_boundary_scenario(sat, channel);
    let request = AcqRequest {
        sat,
        glonass_frequency_channel: Some(channel),
        doppler_search_hz: 0,
        doppler_step_hz: 250,
        coherent_ms: 1,
        noncoherent: 1,
    };
    let mut source = SyntheticSignalSource::new_signal_only(&config, &scenario);
    let receiver = Receiver::new(config, ReceiverRuntime::default());

    let artifacts = receiver
        .run_with_acquisition_requests(&mut source, &[request])
        .expect("receiver run with explicit GLONASS request");
    let acquisition = artifacts
        .acquisitions
        .iter()
        .find(|result| result.sat == sat)
        .expect("GLONASS acquisition result");

    assert_eq!(acquisition.signal_band, SignalBand::L1, "{acquisition:?}");
    assert_eq!(acquisition.glonass_frequency_channel, Some(channel), "{acquisition:?}");
    assert!(
        matches!(
            acquisition.hypothesis,
            bijux_gnss_core::api::AcqHypothesis::Accepted
                | bijux_gnss_core::api::AcqHypothesis::Ambiguous
        ),
        "{acquisition:?}"
    );
    assert!(artifacts.tracking.is_empty(), "{artifacts:?}");
}

#[test]
fn receiver_default_run_does_not_guess_glonass_frequency_channels() {
    let slot = GlonassSlot::new(8).expect("slot 8 must be valid");
    let sat = glonass_slot_sat(slot);
    let channel = GlonassFrequencyChannel::new(-4).expect("channel -4 must be valid");
    let config = glonass_boundary_config(channel);
    let scenario = glonass_boundary_scenario(sat, channel);
    let mut source = SyntheticSignalSource::new_signal_only(&config, &scenario);
    let receiver = Receiver::new(config, ReceiverRuntime::default());

    let artifacts = receiver.run(&mut source).expect("receiver default run");
    let support_matrix = artifacts.support_matrix.as_ref().expect("support matrix");
    let row = support_matrix
        .rows
        .iter()
        .find(|row| {
            row.constellation == Constellation::Glonass
                && row.band == SignalBand::L1
                && row.code == SignalCode::Unknown
        })
        .expect("GLONASS L1 support row");

    assert!(artifacts.acquisitions.is_empty(), "{artifacts:?}");
    assert!(artifacts.tracking.is_empty(), "{artifacts:?}");
    assert!(matches!(row.status, SupportStatus::Planned), "{row:?}");
    assert!(row.reason.contains("explicit FDMA channel requests"), "{row:?}");
}
