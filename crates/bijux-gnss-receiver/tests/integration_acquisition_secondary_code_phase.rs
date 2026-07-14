#![allow(missing_docs)]

use bijux_gnss_core::api::{
    AcqRequest, AcqResult, Constellation, SamplesFrame, SatId, SignalBand, SignalCode,
    SignalComponentRole,
};
use bijux_gnss_receiver::api::{
    sim::{SyntheticScenario, SyntheticSignalParams, SyntheticSignalSource},
    AcquisitionEngine, ReceiverPipelineConfig, ReceiverRuntime, SignalSource,
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
        duration_s: 0.040,
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
