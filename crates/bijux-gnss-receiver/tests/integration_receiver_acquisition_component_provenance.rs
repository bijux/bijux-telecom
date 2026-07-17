#![allow(missing_docs)]

use bijux_gnss_core::api::{
    AcqComponentCombinationMode, AcqHypothesis, AcqRequest, Constellation, SatId, SignalBand,
    SignalCode, SignalComponentRole,
};
use bijux_gnss_receiver::api::{
    sim::{SyntheticScenario, SyntheticSignalParams, SyntheticSignalSource},
    MemorySamples, Receiver, ReceiverPipelineConfig, ReceiverRuntime, SignalSource,
};

const WIDEBAND_PRIMARY_PERIOD_SAMPLES: usize = 10_230;

fn wideband_config() -> ReceiverPipelineConfig {
    ReceiverPipelineConfig {
        sampling_freq_hz: 10_230_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 10_230_000.0,
        code_length: 10_230,
        acquisition_doppler_search_hz: 0,
        acquisition_doppler_step_hz: 250,
        acquisition_integration_ms: 1,
        acquisition_noncoherent: 1,
        channels: 4,
        tracking_budget_ms: 100.0,
        tracking_over_budget_action: "continue".to_string(),
        ..ReceiverPipelineConfig::default()
    }
}

fn wideband_signal_scenario(
    sat: SatId,
    signal_band: SignalBand,
    signal_code: SignalCode,
    scenario_id: &str,
) -> SyntheticScenario {
    SyntheticScenario {
        sample_rate_hz: 10_230_000.0,
        intermediate_freq_hz: 0.0,
        receiver_clock_frequency_bias_hz: 0.0,
        duration_s: 0.03,
        seed: 0xB102_5EED,
        satellites: vec![SyntheticSignalParams {
            sat,
            glonass_frequency_channel: None,
            signal_band,
            signal_code,
            doppler_hz: 0.0,
            code_phase_chips: 2_048.375,
            carrier_phase_rad: 0.25,
            cn0_db_hz: 60.0,
            navigation_data: false.into(),
        }],
        ephemerides: Vec::new(),
        id: scenario_id.to_string(),
    }
}

fn render_memory_samples(
    config: &ReceiverPipelineConfig,
    scenario: &SyntheticScenario,
) -> MemorySamples {
    let total_samples = (scenario.sample_rate_hz * scenario.duration_s).round() as usize;
    let mut synthetic = SyntheticSignalSource::new_signal_only(config, scenario);
    let mut iq = Vec::new();

    while let Some(frame) = synthetic.next_frame(total_samples).expect("synthetic frame") {
        iq.extend(frame.iq);
    }

    let mut interleaved_i16 = Vec::with_capacity(iq.len() * 2);
    for sample in iq {
        interleaved_i16.push((sample.re.clamp(-1.0, 1.0) * 32768.0).round() as i16);
        interleaved_i16.push((sample.im.clamp(-1.0, 1.0) * 32768.0).round() as i16);
    }

    MemorySamples::new(interleaved_i16, config.sampling_freq_hz).expect("memory source")
}

fn render_shifted_memory_samples(
    config: &ReceiverPipelineConfig,
    scenario: &SyntheticScenario,
    skip_primary_periods: usize,
    capture_primary_periods: usize,
) -> MemorySamples {
    let mut synthetic = SyntheticSignalSource::new_signal_only(config, scenario);
    let skip_samples = skip_primary_periods * WIDEBAND_PRIMARY_PERIOD_SAMPLES;
    if skip_samples > 0 {
        let skipped = synthetic
            .next_frame(skip_samples)
            .expect("skip synthetic frame")
            .expect("skipped frame");
        assert_eq!(skipped.iq.len(), skip_samples);
    }
    let capture = synthetic
        .next_frame(capture_primary_periods * WIDEBAND_PRIMARY_PERIOD_SAMPLES)
        .expect("capture frame")
        .expect("capture frame present");

    let mut interleaved_i16 = Vec::with_capacity(capture.iq.len() * 2);
    for sample in capture.iq {
        interleaved_i16.push((sample.re.clamp(-1.0, 1.0) * 32768.0).round() as i16);
        interleaved_i16.push((sample.im.clamp(-1.0, 1.0) * 32768.0).round() as i16);
    }

    MemorySamples::new(interleaved_i16, config.sampling_freq_hz).expect("shifted memory source")
}

fn wideband_acquisition_request(
    sat: SatId,
    signal_band: SignalBand,
    signal_code: SignalCode,
    coherent_ms: u32,
    noncoherent: u32,
) -> AcqRequest {
    AcqRequest {
        sat,
        glonass_frequency_channel: None,
        signal_band,
        signal_code,
        doppler_center_hz: 0.0,
        doppler_rate_center_hz_per_s: 0.0,
        expected_line_of_sight_doppler_hz: None,
        assistance_bounds: None,
        doppler_search_hz: 0,
        doppler_step_hz: 250,
        doppler_rate_search_hz_per_s: 0,
        doppler_rate_step_hz_per_s: 0,
        coherent_ms,
        noncoherent,
    }
}

#[test]
fn receiver_gps_l5q_acquisition_preserves_component_provenance_on_raw_source() {
    let config = wideband_config();
    let sat = SatId { constellation: Constellation::Gps, prn: 24 };
    let scenario = wideband_signal_scenario(
        sat,
        SignalBand::L5,
        SignalCode::L5Q,
        "receiver-gps-l5q-component-provenance-raw-source",
    );
    let mut source = render_memory_samples(&config, &scenario);
    let receiver = Receiver::new(config, ReceiverRuntime::default());
    let request = wideband_acquisition_request(sat, SignalBand::L5, SignalCode::L5Q, 1, 1);

    let artifacts =
        receiver.run_with_acquisition_requests(&mut source, &[request]).expect("receiver run");
    let acquisition = artifacts
        .acquisitions
        .iter()
        .find(|result| result.sat == sat && result.signal_code == SignalCode::L5Q)
        .expect("GPS L5 acquisition result");
    let provenance = acquisition.component_provenance().expect("GPS L5Q component provenance");

    assert_eq!(acquisition.signal_code, SignalCode::L5Q, "{acquisition:?}");
    assert!(
        matches!(acquisition.hypothesis, AcqHypothesis::Accepted | AcqHypothesis::Ambiguous),
        "{acquisition:?}"
    );
    let tracking = artifacts
        .tracking
        .iter()
        .find(|result| result.sat == sat)
        .expect("GPS L5Q tracking result");
    assert!(
        tracking.epochs.iter().all(|epoch| epoch.signal_code == SignalCode::L5Q),
        "{tracking:?}"
    );
    assert_eq!(provenance.combination_mode, AcqComponentCombinationMode::SingleComponent);
    assert_eq!(
        provenance.components.iter().map(|component| component.role).collect::<Vec<_>>(),
        vec![SignalComponentRole::Pilot]
    );
}

#[test]
fn receiver_gps_l5q_acquisition_reports_shifted_secondary_code_phase() {
    let mut config = wideband_config();
    config.acquisition_integration_ms = 20;
    config.acquisition_noncoherent = 1;

    let sat = SatId { constellation: Constellation::Gps, prn: 24 };
    let scenario = SyntheticScenario {
        sample_rate_hz: config.sampling_freq_hz,
        intermediate_freq_hz: config.intermediate_freq_hz,
        receiver_clock_frequency_bias_hz: 0.0,
        duration_s: 0.03,
        seed: 0xB102_5EEE,
        satellites: vec![SyntheticSignalParams {
            sat,
            glonass_frequency_channel: None,
            signal_band: SignalBand::L5,
            signal_code: SignalCode::L5Q,
            doppler_hz: 0.0,
            code_phase_chips: 0.0,
            carrier_phase_rad: 0.25,
            cn0_db_hz: 60.0,
            navigation_data: false.into(),
        }],
        ephemerides: Vec::new(),
        id: "receiver-gps-l5q-shifted-secondary-code-phase".to_string(),
    };
    let mut source = render_shifted_memory_samples(&config, &scenario, 7, 20);
    let receiver = Receiver::new(config, ReceiverRuntime::default());
    let request = wideband_acquisition_request(sat, SignalBand::L5, SignalCode::L5Q, 20, 1);

    let artifacts =
        receiver.run_with_acquisition_requests(&mut source, &[request]).expect("receiver run");
    let acquisition = artifacts
        .acquisitions
        .iter()
        .find(|result| result.sat == sat && result.signal_code == SignalCode::L5Q)
        .expect("shifted GPS L5 acquisition result");
    let pilot_component = acquisition
        .component_provenance()
        .expect("shifted GPS L5Q component provenance")
        .components
        .iter()
        .find(|component| component.role == SignalComponentRole::Pilot)
        .expect("GPS L5Q pilot component");

    assert_eq!(pilot_component.secondary_code_phase_periods, Some(7), "{acquisition:?}");
}

#[test]
fn receiver_galileo_e5b_acquisition_preserves_component_provenance_on_raw_source() {
    let config = wideband_config();
    let sat = SatId { constellation: Constellation::Galileo, prn: 11 };
    let scenario = wideband_signal_scenario(
        sat,
        SignalBand::E5,
        SignalCode::E5b,
        "receiver-galileo-e5b-component-provenance-raw-source",
    );
    let mut source = render_memory_samples(&config, &scenario);
    let receiver = Receiver::new(config, ReceiverRuntime::default());
    let request = wideband_acquisition_request(sat, SignalBand::E5, SignalCode::E5b, 1, 1);

    let artifacts =
        receiver.run_with_acquisition_requests(&mut source, &[request]).expect("receiver run");
    let acquisition = artifacts
        .acquisitions
        .iter()
        .find(|result| result.sat == sat && result.signal_code == SignalCode::E5b)
        .expect("Galileo E5b acquisition result");
    let provenance = acquisition.component_provenance().expect("Galileo E5b component provenance");
    let roles = provenance.components.iter().map(|component| component.role).collect::<Vec<_>>();

    assert_eq!(acquisition.signal_band, SignalBand::E5, "{acquisition:?}");
    assert!(
        matches!(acquisition.hypothesis, AcqHypothesis::Accepted | AcqHypothesis::Ambiguous),
        "{acquisition:?}"
    );
    let tracking = artifacts
        .tracking
        .iter()
        .find(|result| result.sat == sat)
        .expect("Galileo E5b tracking result");
    assert!(
        tracking.epochs.iter().all(|epoch| epoch.signal_code == SignalCode::E5b),
        "{tracking:?}"
    );
    assert!(matches!(
        (provenance.combination_mode, roles.as_slice()),
        (AcqComponentCombinationMode::SingleComponent, [SignalComponentRole::Data])
            | (AcqComponentCombinationMode::SingleComponent, [SignalComponentRole::Pilot])
            | (
                AcqComponentCombinationMode::NoncoherentComponentSum,
                [SignalComponentRole::Data, SignalComponentRole::Pilot],
            )
            | (
                AcqComponentCombinationMode::CoherentComponentSum,
                [SignalComponentRole::Data, SignalComponentRole::Pilot],
            )
    ));
}
