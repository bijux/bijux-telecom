#![allow(missing_docs)]

use bijux_gnss_core::api::{
    Constellation, SatId, SignalBand, SignalCode, BEIDOU_B1_CARRIER_HZ, GPS_L1_CA_CARRIER_HZ,
};
use bijux_gnss_receiver::api::{
    sim::{SyntheticScenario, SyntheticSignalParams, SyntheticSignalSource},
    Receiver, ReceiverPipelineConfig, ReceiverRuntime,
};

fn l1_tracking_config() -> ReceiverPipelineConfig {
    ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        acquisition_doppler_search_hz: 1_000,
        acquisition_doppler_step_hz: 250,
        acquisition_integration_ms: 1,
        acquisition_noncoherent: 1,
        channels: 2,
        tracking_budget_ms: 100.0,
        tracking_over_budget_action: "continue".to_string(),
        ..ReceiverPipelineConfig::default()
    }
}

fn galileo_e1_tracking_config() -> ReceiverPipelineConfig {
    ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 4092,
        acquisition_doppler_search_hz: 0,
        acquisition_doppler_step_hz: 500,
        acquisition_integration_ms: 20,
        acquisition_noncoherent: 1,
        channels: 2,
        tracking_budget_ms: 100.0,
        tracking_over_budget_action: "continue".to_string(),
        ..ReceiverPipelineConfig::default()
    }
}

fn beidou_b1_tracking_config() -> ReceiverPipelineConfig {
    ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: GPS_L1_CA_CARRIER_HZ.value() - BEIDOU_B1_CARRIER_HZ.value(),
        code_freq_basis_hz: 2_046_000.0,
        code_length: 2046,
        acquisition_doppler_search_hz: 0,
        acquisition_doppler_step_hz: 250,
        acquisition_integration_ms: 1,
        acquisition_noncoherent: 1,
        channels: 2,
        tracking_budget_ms: 100.0,
        tracking_over_budget_action: "continue".to_string(),
        ..ReceiverPipelineConfig::default()
    }
}

fn wideband_tracking_config() -> ReceiverPipelineConfig {
    ReceiverPipelineConfig {
        sampling_freq_hz: 10_230_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 10_230_000.0,
        code_length: 10_230,
        acquisition_doppler_search_hz: 1_000,
        acquisition_doppler_step_hz: 250,
        acquisition_integration_ms: 1,
        acquisition_noncoherent: 1,
        channels: 2,
        tracking_budget_ms: 100.0,
        tracking_over_budget_action: "continue".to_string(),
        ..ReceiverPipelineConfig::default()
    }
}

fn run_tracking_case(
    config: &ReceiverPipelineConfig,
    scenario: &SyntheticScenario,
) -> bijux_gnss_receiver::api::RunArtifacts {
    let mut source = SyntheticSignalSource::new_signal_only(config, scenario);
    let receiver = Receiver::new(config.clone(), ReceiverRuntime::default());
    receiver.run(&mut source).expect("receiver run")
}

#[test]
fn gps_l5q_tracking_reports_pilot_metadata_without_navigation_bit_lock() {
    let config = wideband_tracking_config();
    let sat = SatId { constellation: Constellation::Gps, prn: 24 };
    let scenario = SyntheticScenario {
        sample_rate_hz: config.sampling_freq_hz,
        intermediate_freq_hz: config.intermediate_freq_hz,
        receiver_clock_frequency_bias_hz: 0.0,
        duration_s: 0.060,
        seed: 0x15AB_C5E1,
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
        id: "tracking-signal-metadata-gps-l5q".to_string(),
    };

    let artifacts = run_tracking_case(&config, &scenario);
    let tracking =
        artifacts.tracking.iter().find(|result| result.sat == sat).expect("GPS L5-Q tracking");

    assert!(
        tracking
            .epochs
            .iter()
            .any(|epoch| epoch.signal_band == SignalBand::L5 && epoch.signal_code == SignalCode::L5Q),
        "{tracking:#?}"
    );
    assert!(
        tracking.epochs.iter().all(|epoch| !epoch.nav_bit_lock),
        "{tracking:#?}"
    );
    assert!(
        tracking.epochs.iter().all(|epoch| epoch.navigation_bit_sign.is_none()),
        "{tracking:#?}"
    );
    assert!(
        tracking.epochs.iter().all(|epoch| {
            epoch.tracking_assumptions
                .as_ref()
                .is_some_and(|assumptions| assumptions.discriminator_family == "early_prompt_late")
        }),
        "{tracking:#?}"
    );
    assert!(
        tracking.epochs.iter().all(|epoch| {
            epoch.tracking_provenance.contains("track_component_role=Pilot")
                && epoch.tracking_provenance.contains("phase_transition_source=secondary_code")
                && epoch.tracking_provenance.contains("nominal_carrier_hz=1176450000.000")
        }),
        "{tracking:#?}"
    );
}

#[test]
fn gps_l1ca_tracking_still_reports_navigation_bit_lock_for_data_symbols() {
    let config = l1_tracking_config();
    let sat = SatId { constellation: Constellation::Gps, prn: 12 };
    let scenario = SyntheticScenario {
        sample_rate_hz: config.sampling_freq_hz,
        intermediate_freq_hz: config.intermediate_freq_hz,
        receiver_clock_frequency_bias_hz: 0.0,
        duration_s: 0.065,
        seed: 0x4A11_CA12,
        satellites: vec![SyntheticSignalParams {
            sat,
            glonass_frequency_channel: None,
            signal_band: SignalBand::L1,
            signal_code: SignalCode::Ca,
            doppler_hz: 120.0,
            code_phase_chips: 144.375,
            carrier_phase_rad: 0.3,
            cn0_db_hz: 60.0,
            navigation_data: true.into(),
        }],
        ephemerides: Vec::new(),
        id: "tracking-signal-metadata-gps-l1ca".to_string(),
    };

    let artifacts = run_tracking_case(&config, &scenario);
    let tracking =
        artifacts.tracking.iter().find(|result| result.sat == sat).expect("GPS L1 C/A tracking");

    assert!(tracking.epochs.iter().any(|epoch| epoch.nav_bit_lock), "{tracking:#?}");
    assert!(
        tracking.epochs.iter().any(|epoch| epoch.navigation_bit_sign.is_some()),
        "{tracking:#?}"
    );
}

#[test]
fn beidou_b1i_tracking_reports_secondary_code_metadata_without_navigation_bit_lock() {
    let config = beidou_b1_tracking_config();
    let sat = SatId { constellation: Constellation::Beidou, prn: 11 };
    let scenario = SyntheticScenario {
        sample_rate_hz: config.sampling_freq_hz,
        intermediate_freq_hz: config.intermediate_freq_hz,
        receiver_clock_frequency_bias_hz: 0.0,
        duration_s: 0.065,
        seed: 0xB1D0_1111,
        satellites: vec![SyntheticSignalParams {
            sat,
            glonass_frequency_channel: None,
            signal_band: SignalBand::B1,
            signal_code: SignalCode::B1I,
            doppler_hz: 0.0,
            code_phase_chips: 213.5,
            carrier_phase_rad: 0.2,
            cn0_db_hz: 60.0,
            navigation_data: false.into(),
        }],
        ephemerides: Vec::new(),
        id: "tracking-signal-metadata-beidou-b1i".to_string(),
    };

    let artifacts = run_tracking_case(&config, &scenario);
    let tracking =
        artifacts.tracking.iter().find(|result| result.sat == sat).expect("BeiDou B1I tracking");

    assert!(
        tracking.epochs.iter().all(|epoch| !epoch.nav_bit_lock),
        "{tracking:#?}"
    );
    assert!(
        tracking.epochs.iter().all(|epoch| {
            epoch.tracking_assumptions
                .as_ref()
                .is_some_and(|assumptions| assumptions.discriminator_family == "early_prompt_late")
        }),
        "{tracking:#?}"
    );
    assert!(
        tracking.epochs.iter().all(|epoch| {
            epoch.tracking_provenance.contains("track_component_role=Data")
                && epoch.tracking_provenance.contains("secondary_code=true")
                && epoch.tracking_provenance.contains("discriminator_family=early_prompt_late")
                && epoch.tracking_provenance.contains("phase_transition_source=secondary_code")
                && epoch.tracking_provenance.contains("nominal_carrier_hz=1561098000.000")
        }),
        "{tracking:#?}"
    );
}

#[test]
fn galileo_e1b_tracking_reports_cboc_discriminator_metadata() {
    let config = galileo_e1_tracking_config();
    let sat = SatId { constellation: Constellation::Galileo, prn: 11 };
    let scenario = SyntheticScenario {
        sample_rate_hz: config.sampling_freq_hz,
        intermediate_freq_hz: config.intermediate_freq_hz,
        receiver_clock_frequency_bias_hz: 0.0,
        duration_s: 0.080,
        seed: 0x6A11_E100,
        satellites: vec![SyntheticSignalParams {
            sat,
            glonass_frequency_channel: None,
            signal_band: SignalBand::E1,
            signal_code: SignalCode::E1B,
            doppler_hz: 0.0,
            code_phase_chips: 321.0,
            carrier_phase_rad: 0.25,
            cn0_db_hz: 60.0,
            navigation_data: false.into(),
        }],
        ephemerides: Vec::new(),
        id: "tracking-signal-metadata-galileo-e1b".to_string(),
    };
    let mut source = SyntheticSignalSource::new(&config, &scenario);
    let receiver = Receiver::new(config.clone(), ReceiverRuntime::default());
    let artifacts = receiver.run(&mut source).expect("receiver run");
    let tracking =
        artifacts.tracking.iter().find(|result| result.sat == sat).expect("Galileo E1-B tracking");

    assert!(
        tracking.epochs.iter().all(|epoch| {
            epoch.tracking_assumptions
                .as_ref()
                .is_some_and(|assumptions| assumptions.discriminator_family == "cboc_early_prompt_late")
        }),
        "{tracking:#?}"
    );
    assert!(
        tracking.epochs.iter().all(|epoch| {
            epoch.tracking_provenance.contains("track_component_role=Data")
                && epoch.tracking_provenance.contains("discriminator_family=cboc_early_prompt_late")
                && epoch.tracking_provenance.contains("nominal_carrier_hz=1575420000.000")
        }),
        "{tracking:#?}"
    );
}
