#![allow(missing_docs)]

use bijux_gnss_core::api::{Constellation, SatId, SignalBand};
use bijux_gnss_receiver::api::{
    sim::{
        expected_acquisition_code_phase_samples, wrapped_code_phase_error_samples_f64,
        SyntheticScenario, SyntheticSignalParams, SyntheticSignalSource,
    },
    Receiver, ReceiverPipelineConfig, ReceiverRuntime,
};
use bijux_gnss_signal::api::samples_per_code;

fn galileo_e1_report_config() -> ReceiverPipelineConfig {
    ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 4092,
        acquisition_doppler_search_hz: 500,
        acquisition_doppler_step_hz: 500,
        acquisition_integration_ms: 20,
        acquisition_noncoherent: 1,
        channels: 4,
        tracking_budget_ms: 100.0,
        tracking_over_budget_action: "continue".to_string(),
        ..ReceiverPipelineConfig::default()
    }
}

fn galileo_e1_report_scenario(sat: SatId) -> SyntheticScenario {
    SyntheticScenario {
        sample_rate_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        receiver_clock_frequency_bias_hz: 0.0,
        duration_s: 0.020,
        seed: 0x6A11_E1A5,
        satellites: vec![SyntheticSignalParams {
            sat,
            glonass_frequency_channel: None,
            signal_band: bijux_gnss_core::api::SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Unknown,
            doppler_hz: 0.0,
            code_phase_chips: 321.0,
            carrier_phase_rad: 0.25,
            cn0_db_hz: 60.0,
            data_bit_flip: false,
        }],
        ephemerides: Vec::new(),
        id: "receiver-galileo-e1-acquisition-report".to_string(),
    }
}

#[test]
fn receiver_run_reports_galileo_e1_acquisition_fields() {
    let config = galileo_e1_report_config();
    let sat = SatId { constellation: Constellation::Galileo, prn: 11 };
    let scenario = galileo_e1_report_scenario(sat);
    let mut source = SyntheticSignalSource::new_signal_only(&config, &scenario);
    let receiver = Receiver::new(config.clone(), ReceiverRuntime::default());

    let artifacts = receiver.run(&mut source).expect("receiver run");
    let sat_result = artifacts
        .acquisitions
        .iter()
        .find(|result| result.sat == sat)
        .expect("Galileo acquisition result");
    let expected_code_phase_samples = expected_acquisition_code_phase_samples(
        &config,
        &galileo_e1_reference_frame(&config),
        321.0,
    ) as f64;
    let period_samples =
        samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length);
    let code_phase_error_samples = wrapped_code_phase_error_samples_f64(
        sat_result.resolved_code_phase_samples(),
        expected_code_phase_samples,
        period_samples,
    );
    let uncertainty = sat_result.uncertainty.as_ref().expect("Galileo acquisition uncertainty");

    assert_eq!(sat_result.sat, sat);
    assert_eq!(sat_result.signal_band, SignalBand::E1, "{sat_result:?}");
    assert_eq!(sat_result.hypothesis.to_string(), "accepted", "{sat_result:?}");
    assert_eq!(sat_result.doppler_hz.0, 0.0, "{sat_result:?}");
    assert!(sat_result.carrier_hz.0.abs() <= 50.0, "carrier estimate drifted: {:?}", sat_result);
    assert!(sat_result.peak_mean_ratio > 10.0, "{sat_result:?}");
    assert!(sat_result.peak_second_ratio > 1.2, "{sat_result:?}");
    assert!(code_phase_error_samples <= 0.5, "{sat_result:?}");
    assert!(
        uncertainty.doppler_hz.is_finite()
            && uncertainty.doppler_hz > 0.0
            && uncertainty.doppler_hz < 250.0,
        "{sat_result:?}"
    );
    assert!(
        uncertainty.code_phase_samples.is_finite()
            && uncertainty.code_phase_samples > 0.0
            && uncertainty.code_phase_samples < 0.5,
        "{sat_result:?}"
    );
}

fn galileo_e1_reference_frame(
    config: &ReceiverPipelineConfig,
) -> bijux_gnss_core::api::SamplesFrame {
    let sat = SatId { constellation: Constellation::Galileo, prn: 11 };
    bijux_gnss_receiver::api::sim::generate_l1_ca(
        config,
        SyntheticSignalParams {
            sat,
            glonass_frequency_channel: None,
            signal_band: bijux_gnss_core::api::SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Unknown,
            doppler_hz: 0.0,
            code_phase_chips: 321.0,
            carrier_phase_rad: 0.25,
            cn0_db_hz: 60.0,
            data_bit_flip: false,
        },
        0x6A11_E1A5,
        0.020,
    )
}
