#![allow(missing_docs)]

use bijux_gnss_core::api::{
    AcqRequest, AcqResult, Constellation, SampleTime, SamplesFrame, SatId, Seconds, SignalBand,
    SignalCode, SignalComponentRole, GPS_L1_CA_CARRIER_HZ,
};
use bijux_gnss_receiver::api::{
    sim::{SyntheticScenario, SyntheticSignalParams, SyntheticSignalSource},
    AcquisitionEngine, ReceiverPipelineConfig, ReceiverRuntime, SignalSource,
};
use bijux_gnss_signal::api::{
    default_signal_carrier_hz_for_signal, sample_modulated_replica_at_sample_index,
    ReplicaCodeModel, ReplicaSampleIndexRequest,
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

fn aligned_secondary_code_scenario(
    sat: SatId,
    signal_band: SignalBand,
    signal_code: SignalCode,
    scenario_id: &str,
) -> SyntheticScenario {
    SyntheticScenario {
        sample_rate_hz: 10_230_000.0,
        intermediate_freq_hz: 0.0,
        receiver_clock_frequency_bias_hz: 0.0,
        duration_s: 0.100,
        seed: 0xA915_C0DE,
        satellites: vec![SyntheticSignalParams {
            sat,
            glonass_frequency_channel: None,
            signal_band,
            signal_code,
            doppler_hz: 0.0,
            code_phase_chips: 0.0,
            carrier_phase_rad: 0.25,
            cn0_db_hz: 60.0,
            navigation_data: false.into(),
        }],
        ephemerides: Vec::new(),
        id: scenario_id.to_string(),
    }
}

fn acquisition_request(
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
        doppler_rate_search_hz_per_s: 0,
        doppler_rate_step_hz_per_s: 250,
        expected_line_of_sight_doppler_hz: None,
        assistance_bounds: None,
        doppler_search_hz: 0,
        doppler_step_hz: 250,
        coherent_ms,
        noncoherent,
    }
}

fn offset_acquisition_frame(
    config: &ReceiverPipelineConfig,
    scenario: &SyntheticScenario,
    skip_primary_periods: usize,
    acquisition_periods: usize,
) -> SamplesFrame {
    let mut source = SyntheticSignalSource::new_signal_only(config, scenario);
    let skip_samples = skip_primary_periods * WIDEBAND_PRIMARY_PERIOD_SAMPLES;
    if skip_samples > 0 {
        let skipped =
            source.next_frame(skip_samples).expect("skip synthetic frame").expect("skipped frame");
        assert_eq!(skipped.iq.len(), skip_samples);
    }

    source
        .next_frame(acquisition_periods * WIDEBAND_PRIMARY_PERIOD_SAMPLES)
        .expect("acquisition frame")
        .expect("acquisition frame present")
}

fn component_secondary_code_phase(result: &AcqResult, role: SignalComponentRole) -> Option<u32> {
    result
        .component_provenance()?
        .components
        .iter()
        .find(|component| component.role == role)
        .and_then(|component| component.secondary_code_phase_periods)
}

fn candidate_pilot_secondary_code_phases(results: &[AcqResult]) -> Vec<u32> {
    results
        .iter()
        .filter_map(|result| component_secondary_code_phase(result, SignalComponentRole::Pilot))
        .collect()
}

fn galileo_qpsk_acquisition_frame(
    config: &ReceiverPipelineConfig,
    sat: SatId,
    signal_code: SignalCode,
    skip_primary_periods: usize,
    acquisition_periods: usize,
) -> SamplesFrame {
    let model = match signal_code {
        SignalCode::E5a => {
            ReplicaCodeModel::galileo_e5a_qpsk(sat.prn).expect("Galileo E5a QPSK replica")
        }
        SignalCode::E5b => {
            ReplicaCodeModel::galileo_e5b_qpsk(sat.prn).expect("Galileo E5b QPSK replica")
        }
        _ => panic!("unsupported Galileo pilot signal {:?}", signal_code),
    };
    let carrier_hz =
        default_signal_carrier_hz_for_signal(sat, Some(SignalBand::E5), signal_code, None)
            .expect("Galileo E5 carrier lookup")
            .expect("Galileo E5 carrier available")
            .value()
            - GPS_L1_CA_CARRIER_HZ.value();
    let start_sample_index = (skip_primary_periods * WIDEBAND_PRIMARY_PERIOD_SAMPLES) as u64;
    let sample_count = acquisition_periods * WIDEBAND_PRIMARY_PERIOD_SAMPLES;
    let iq = (0..sample_count)
        .map(|sample_offset| {
            sample_modulated_replica_at_sample_index(
                &model,
                ReplicaSampleIndexRequest {
                    sample_rate_hz: config.sampling_freq_hz,
                    initial_code_phase_chips: 0.0,
                    initial_carrier_phase_radians: 0.25,
                    initial_carrier_hz: carrier_hz,
                    carrier_rate_hz_per_s: 0.0,
                    sample_index: start_sample_index + sample_offset as u64,
                    data_bit: 1,
                    amplitude: 1.0,
                },
            )
            .expect("Galileo QPSK sample")
        })
        .collect();

    SamplesFrame::new(
        SampleTime { sample_index: start_sample_index, sample_rate_hz: config.sampling_freq_hz },
        Seconds(1.0 / config.sampling_freq_hz),
        iq,
    )
}

#[test]
fn gps_l5q_acquisition_recovers_secondary_code_phase_from_offset_capture() {
    let config = wideband_config();
    let sat = SatId { constellation: Constellation::Gps, prn: 24 };
    let scenario = aligned_secondary_code_scenario(
        sat,
        SignalBand::L5,
        SignalCode::L5Q,
        "acquisition-gps-l5q-secondary-code-phase",
    );
    let skip_primary_periods = 7usize;
    let frame = offset_acquisition_frame(&config, &scenario, skip_primary_periods, 20);
    let acquisition = AcquisitionEngine::new(config, ReceiverRuntime::default());

    let run = acquisition.run_fft_topn_for_requests_with_explain(
        &frame,
        &[acquisition_request(sat, SignalBand::L5, SignalCode::L5Q, 20, 1)],
        1,
    );
    let result = &run.results[0][0];

    assert_eq!(result.signal_code, SignalCode::L5Q, "{result:?}");
    assert_eq!(
        result
            .component_provenance()
            .expect("GPS L5Q component provenance")
            .components
            .iter()
            .map(|component| component.role)
            .collect::<Vec<_>>(),
        vec![SignalComponentRole::Pilot]
    );
    assert_eq!(
        component_secondary_code_phase(result, SignalComponentRole::Pilot),
        Some(skip_primary_periods as u32)
    );
}

#[test]
fn galileo_e5a_acquisition_recovers_pilot_secondary_code_phase_from_offset_capture() {
    let config = wideband_config();
    let sat = SatId { constellation: Constellation::Galileo, prn: 11 };
    let skip_primary_periods = 37usize;
    let frame =
        galileo_qpsk_acquisition_frame(&config, sat, SignalCode::E5a, skip_primary_periods, 20);
    let acquisition = AcquisitionEngine::new(config, ReceiverRuntime::default());

    let run = acquisition.run_fft_topn_for_requests_with_explain(
        &frame,
        &[acquisition_request(sat, SignalBand::E5, SignalCode::E5a, 20, 1)],
        3,
    );
    let pilot_phases = candidate_pilot_secondary_code_phases(&run.results[0]);

    assert!(!pilot_phases.is_empty(), "{:#?}", run.results[0]);
    assert!(
        pilot_phases.iter().all(|phase| *phase == skip_primary_periods as u32),
        "{:#?}",
        run.results[0]
    );
}

#[test]
fn galileo_e5b_acquisition_recovers_pilot_secondary_code_phase_from_offset_capture() {
    let config = wideband_config();
    let sat = SatId { constellation: Constellation::Galileo, prn: 11 };
    let skip_primary_periods = 58usize;
    let frame =
        galileo_qpsk_acquisition_frame(&config, sat, SignalCode::E5b, skip_primary_periods, 20);
    let acquisition = AcquisitionEngine::new(config, ReceiverRuntime::default());

    let run = acquisition.run_fft_topn_for_requests_with_explain(
        &frame,
        &[acquisition_request(sat, SignalBand::E5, SignalCode::E5b, 20, 1)],
        3,
    );
    let pilot_phases = candidate_pilot_secondary_code_phases(&run.results[0]);

    assert!(!pilot_phases.is_empty(), "{:#?}", run.results[0]);
    assert!(
        pilot_phases.iter().all(|phase| *phase == skip_primary_periods as u32),
        "{:#?}",
        run.results[0]
    );
}
