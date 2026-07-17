#![allow(missing_docs)]

use bijux_gnss_core::api::{
    AcqComponentCombinationMode, AcqRequest, Constellation, SatId, SignalBand, SignalCode,
    SignalComponentRole,
};
use bijux_gnss_receiver::api::{
    sim::{generate_l1_ca_multi, SyntheticScenario, SyntheticSignalParams},
    AcquisitionEngine, ReceiverPipelineConfig, ReceiverRuntime,
};

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

fn wideband_scenario(
    sat: SatId,
    signal_band: SignalBand,
    signal_code: SignalCode,
    duration_s: f64,
    scenario_id: &str,
) -> SyntheticScenario {
    SyntheticScenario {
        sample_rate_hz: 10_230_000.0,
        intermediate_freq_hz: 0.0,
        receiver_clock_frequency_bias_hz: 0.0,
        duration_s,
        seed: 0xA901_5EED,
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

fn acquisition_request(
    sat: SatId,
    signal_band: SignalBand,
    signal_code: SignalCode,
    coherent_ms: u32,
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
        noncoherent: 1,
    }
}

#[test]
fn galileo_e5a_acquisition_reports_all_component_strategies_at_one_millisecond() {
    let config = wideband_config();
    let sat = SatId { constellation: Constellation::Galileo, prn: 11 };
    let scenario = wideband_scenario(
        sat,
        SignalBand::E5,
        SignalCode::E5a,
        0.03,
        "acquisition-galileo-e5a-component-provenance",
    );
    let frame = generate_l1_ca_multi(&config, &scenario);
    let acquisition = AcquisitionEngine::new(config, ReceiverRuntime::default());

    let run = acquisition.run_fft_topn_for_requests_with_explain(
        &frame,
        &[acquisition_request(sat, SignalBand::E5, SignalCode::E5a, 1)],
        4,
    );
    let candidates = &run.results[0];

    assert_eq!(candidates.len(), 3);
    assert!(candidates.iter().all(|candidate| candidate.signal_code == SignalCode::E5a));
    assert!(candidates.iter().all(|candidate| candidate.component_provenance().is_some()));
    assert!(candidates.iter().any(|candidate| {
        candidate.component_provenance().is_some_and(|provenance| {
            provenance.combination_mode == AcqComponentCombinationMode::SingleComponent
                && provenance.components.iter().map(|component| component.role).collect::<Vec<_>>()
                    == vec![SignalComponentRole::Data]
        })
    }));
    assert!(candidates.iter().any(|candidate| {
        candidate.component_provenance().is_some_and(|provenance| {
            provenance.combination_mode == AcqComponentCombinationMode::SingleComponent
                && provenance.components.iter().map(|component| component.role).collect::<Vec<_>>()
                    == vec![SignalComponentRole::Pilot]
        })
    }));
    assert!(candidates.iter().any(|candidate| {
        candidate.component_provenance().is_some_and(|provenance| {
            provenance.combination_mode == AcqComponentCombinationMode::NoncoherentComponentSum
                && provenance.components.iter().map(|component| component.role).collect::<Vec<_>>()
                    == vec![SignalComponentRole::Data, SignalComponentRole::Pilot]
        })
    }));
    assert!(candidates.iter().all(|candidate| {
        candidate.component_provenance().is_some_and(|provenance| {
            provenance.combination_mode != AcqComponentCombinationMode::CoherentComponentSum
        })
    }));
}

#[test]
fn galileo_e5a_acquisition_skips_coherent_component_sum_at_two_milliseconds() {
    let config = wideband_config();
    let sat = SatId { constellation: Constellation::Galileo, prn: 11 };
    let scenario = wideband_scenario(
        sat,
        SignalBand::E5,
        SignalCode::E5a,
        0.03,
        "acquisition-galileo-e5a-noncoherent-component-provenance",
    );
    let frame = generate_l1_ca_multi(&config, &scenario);
    let acquisition = AcquisitionEngine::new(config, ReceiverRuntime::default());

    let run = acquisition.run_fft_topn_for_requests_with_explain(
        &frame,
        &[acquisition_request(sat, SignalBand::E5, SignalCode::E5a, 2)],
        4,
    );
    let candidates = &run.results[0];

    assert_eq!(candidates.len(), 3);
    assert!(candidates.iter().all(|candidate| {
        candidate.component_provenance().is_some_and(|provenance| {
            provenance.combination_mode != AcqComponentCombinationMode::CoherentComponentSum
        })
    }));
}

#[test]
fn gps_l5q_acquisition_reports_single_pilot_component_provenance() {
    let config = wideband_config();
    let sat = SatId { constellation: Constellation::Gps, prn: 24 };
    let scenario = wideband_scenario(
        sat,
        SignalBand::L5,
        SignalCode::L5Q,
        0.03,
        "acquisition-gps-l5q-component-provenance",
    );
    let frame = generate_l1_ca_multi(&config, &scenario);
    let acquisition = AcquisitionEngine::new(config, ReceiverRuntime::default());

    let run = acquisition.run_fft_topn_for_requests_with_explain(
        &frame,
        &[acquisition_request(sat, SignalBand::L5, SignalCode::L5Q, 1)],
        1,
    );
    let provenance =
        run.results[0][0].component_provenance().expect("GPS L5Q component provenance");

    assert_eq!(provenance.combination_mode, AcqComponentCombinationMode::SingleComponent);
    assert_eq!(
        provenance.components.iter().map(|component| component.role).collect::<Vec<_>>(),
        vec![SignalComponentRole::Pilot]
    );
}
