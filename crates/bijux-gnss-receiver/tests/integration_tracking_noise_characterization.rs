#![allow(missing_docs)]

use bijux_gnss_core::api::{
    Constellation, SamplesFrame, SatId, SignalBand, SignalCode, BEIDOU_B1_CARRIER_HZ,
    GPS_L1_CA_CARRIER_HZ, GPS_L5_CARRIER_HZ,
};
use bijux_gnss_receiver::api::{
    sim::{
        build_iq16_capture_bundle, characterize_supported_tracking_noise, generate_l1_ca_multi,
        supported_tracking_signal_identities, truth_guided_receiver_accuracy_budgets,
        validate_truth_guided_tracking_table, SyntheticIqTruthBundle, SyntheticScenario,
        SyntheticSignalParams, SyntheticTrackingSignalIdentity, SyntheticTrackingTruthTableReport,
    },
    ReceiverPipelineConfig,
};
use bijux_gnss_signal::api::{glonass_l1_carrier_hz, signal_registry};

const MIN_STABLE_TRACKING_NOISE_EPOCHS: usize = 1;

#[test]
fn tracking_noise_characterization_covers_supported_signals() {
    let reports = supported_tracking_signal_identities()
        .into_iter()
        .map(tracking_noise_truth_report_for_signal)
        .collect::<Vec<_>>();

    let noise = characterize_supported_tracking_noise(&reports, MIN_STABLE_TRACKING_NOISE_EPOCHS);

    assert!(
        noise.pass,
        "tracking noise characterization failed: missing={:?} under_sampled={:?} characterized={} supported={}",
        noise.missing_signals,
        noise.under_sampled_signals,
        noise.characterized_signal_count,
        noise.supported_signal_count
    );
    assert!(noise.missing_signals.is_empty(), "missing={:?}", noise.missing_signals);
    assert!(
        noise.under_sampled_signals.is_empty(),
        "under_sampled={:?}",
        noise.under_sampled_signals
    );
    assert_eq!(noise.supported_signal_count, noise.characterized_signal_count);

    for profile in &noise.profiles {
        assert!(
            profile.pass,
            "profile for {:?} is under-sampled: stable_epoch_count={}",
            profile.signal, profile.stable_epoch_count
        );
        assert!(
            profile.stable_epoch_count >= MIN_STABLE_TRACKING_NOISE_EPOCHS,
            "profile={:?}",
            profile.signal
        );
        assert_eq!(
            profile.dll_jitter_samples.sample_count, profile.stable_epoch_count,
            "profile={:?}",
            profile.signal
        );
        assert_eq!(
            profile.pll_phase_error_cycles.sample_count, profile.stable_epoch_count,
            "profile={:?}",
            profile.signal
        );
        assert_eq!(
            profile.doppler_error_hz.sample_count, profile.stable_epoch_count,
            "profile={:?}",
            profile.signal
        );
        assert_eq!(
            profile.cn0_bias_db_hz.sample_count, profile.stable_epoch_count,
            "profile={:?}",
            profile.signal
        );
        assert!(profile.dll_jitter_samples.p95_abs.is_finite(), "profile={:?}", profile.signal);
        assert!(profile.pll_phase_error_cycles.p95_abs.is_finite(), "profile={:?}", profile.signal);
        assert!(profile.doppler_error_hz.p95_abs.is_finite(), "profile={:?}", profile.signal);
        assert!(profile.cn0_bias_db_hz.p95_abs.is_finite(), "profile={:?}", profile.signal);
        assert!(profile.cycle_slip_probability.is_finite(), "profile={:?}", profile.signal);
        assert!(
            (0.0..=1.0).contains(&profile.cycle_slip_probability),
            "profile={:?} cycle_slip_probability={}",
            profile.signal,
            profile.cycle_slip_probability
        );
    }
}

fn tracking_noise_truth_report_for_signal(
    signal: SyntheticTrackingSignalIdentity,
) -> SyntheticTrackingTruthTableReport {
    let fixture = tracking_noise_fixture(signal);
    let budget = truth_guided_receiver_accuracy_budgets().tracking;
    let report = validate_truth_guided_tracking_table(
        &fixture.config,
        &fixture.frame,
        &fixture.truth,
        budget.max_carrier_error_hz,
        budget.max_doppler_error_hz,
        budget.max_code_phase_error_samples,
        budget.max_cn0_error_db_hz,
    );

    assert_eq!(report.satellites.len(), 1, "{report:#?}");
    assert!(
        report.satellites[0].stable_epoch_count >= MIN_STABLE_TRACKING_NOISE_EPOCHS,
        "stable truth coverage is required for {:?}: stable_epoch_count={} epoch_count={} pass={} states={}",
        signal,
        report.satellites[0].stable_epoch_count,
        report.satellites[0].epoch_count,
        report.pass,
        tracking_truth_state_summary(&report.satellites[0].epochs)
    );
    report
}

fn tracking_truth_state_summary(
    epochs: &[bijux_gnss_receiver::api::sim::SyntheticTrackingTruthTableEpoch],
) -> String {
    let mut states = std::collections::BTreeMap::<String, usize>::new();
    for epoch in epochs {
        let key = format!(
            "{}:{}",
            epoch.lock_state,
            epoch.lock_state_reason.as_deref().unwrap_or("none")
        );
        *states.entry(key).or_default() += 1;
    }
    states
        .into_iter()
        .map(|(state, count)| format!("{state}={count}"))
        .collect::<Vec<_>>()
        .join(",")
}

struct TrackingNoiseFixture {
    config: ReceiverPipelineConfig,
    frame: SamplesFrame,
    truth: SyntheticIqTruthBundle,
}

struct TrackingNoiseFixturePlan {
    sample_rate_hz: f64,
    intermediate_freq_hz: f64,
    code_freq_basis_hz: f64,
    code_length: usize,
    acquisition_doppler_search_hz: i32,
    acquisition_doppler_step_hz: i32,
    acquisition_integration_ms: u32,
    duration_s: f64,
    seed: u64,
    sat: SatId,
    doppler_hz: f64,
    code_phase_chips: f64,
    carrier_phase_rad: f64,
    cn0_db_hz: f32,
    navigation_data: bool,
}

fn tracking_noise_fixture(signal: SyntheticTrackingSignalIdentity) -> TrackingNoiseFixture {
    let plan = tracking_noise_fixture_plan(signal);
    let scenario = SyntheticScenario {
        sample_rate_hz: plan.sample_rate_hz,
        intermediate_freq_hz: plan.intermediate_freq_hz,
        receiver_clock_frequency_bias_hz: 0.0,
        duration_s: plan.duration_s,
        seed: plan.seed,
        satellites: vec![SyntheticSignalParams {
            sat: plan.sat,
            glonass_frequency_channel: signal.glonass_frequency_channel,
            signal_band: signal.signal_band,
            signal_code: signal.signal_code,
            doppler_hz: plan.doppler_hz,
            code_phase_chips: plan.code_phase_chips,
            carrier_phase_rad: plan.carrier_phase_rad,
            cn0_db_hz: plan.cn0_db_hz,
            navigation_data: plan.navigation_data.into(),
        }],
        ephemerides: Vec::new(),
        id: tracking_noise_scenario_id(signal),
    };
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: scenario.sample_rate_hz,
        intermediate_freq_hz: scenario.intermediate_freq_hz,
        code_freq_basis_hz: plan.code_freq_basis_hz,
        code_length: plan.code_length,
        acquisition_doppler_search_hz: plan.acquisition_doppler_search_hz,
        acquisition_doppler_step_hz: plan.acquisition_doppler_step_hz,
        acquisition_integration_ms: plan.acquisition_integration_ms,
        acquisition_noncoherent: 1,
        channels: 2,
        tracking_budget_ms: 100.0,
        tracking_over_budget_action: "continue".to_string(),
        ..ReceiverPipelineConfig::default()
    };
    let frame = generate_l1_ca_multi(&config, &scenario);
    let bundle = build_iq16_capture_bundle(
        &scenario.id,
        &scenario,
        &frame,
        "2026-07-15T00:00:00Z",
        Some(format!("tracking noise characterization {}", scenario.id)),
    );
    let scaled_frame = SamplesFrame::new(
        frame.t0,
        frame.dt_s,
        frame.iq.iter().map(|sample| *sample * bundle.truth.output_scale_applied).collect(),
    );

    TrackingNoiseFixture { config, frame: scaled_frame, truth: bundle.truth }
}

fn tracking_noise_fixture_plan(
    signal: SyntheticTrackingSignalIdentity,
) -> TrackingNoiseFixturePlan {
    match (signal.constellation, signal.signal_band, signal.signal_code) {
        (Constellation::Gps, SignalBand::L5, SignalCode::L5I) => TrackingNoiseFixturePlan {
            sample_rate_hz: 10_230_000.0,
            intermediate_freq_hz: GPS_L1_CA_CARRIER_HZ.value() - GPS_L5_CARRIER_HZ.value(),
            code_freq_basis_hz: 10_230_000.0,
            code_length: 10_230,
            acquisition_doppler_search_hz: 2_000,
            acquisition_doppler_step_hz: 250,
            acquisition_integration_ms: 1,
            duration_s: 0.060,
            seed: 0x15AB_C5E0,
            sat: SatId { constellation: Constellation::Gps, prn: 18 },
            doppler_hz: 750.0,
            code_phase_chips: 2_048.25,
            carrier_phase_rad: 0.4,
            cn0_db_hz: 60.0,
            navigation_data: false,
        },
        (Constellation::Gps, SignalBand::L5, SignalCode::L5Q) => TrackingNoiseFixturePlan {
            sample_rate_hz: 10_230_000.0,
            intermediate_freq_hz: GPS_L1_CA_CARRIER_HZ.value() - GPS_L5_CARRIER_HZ.value(),
            code_freq_basis_hz: 10_230_000.0,
            code_length: 10_230,
            acquisition_doppler_search_hz: 1_000,
            acquisition_doppler_step_hz: 250,
            acquisition_integration_ms: 1,
            duration_s: 0.060,
            seed: 0x15AB_C5E1,
            sat: SatId { constellation: Constellation::Gps, prn: 24 },
            doppler_hz: 0.0,
            code_phase_chips: 0.0,
            carrier_phase_rad: 0.25,
            cn0_db_hz: 60.0,
            navigation_data: false,
        },
        (Constellation::Galileo, SignalBand::E1, SignalCode::E1B | SignalCode::E1C) => {
            TrackingNoiseFixturePlan {
                sample_rate_hz: 4_092_000.0,
                intermediate_freq_hz: 0.0,
                code_freq_basis_hz: 1_023_000.0,
                code_length: 4092,
                acquisition_doppler_search_hz: 0,
                acquisition_doppler_step_hz: 500,
                acquisition_integration_ms: 20,
                duration_s: 0.200,
                seed: 0x6A11_E100,
                sat: SatId { constellation: Constellation::Galileo, prn: 11 },
                doppler_hz: 0.0,
                code_phase_chips: 321.0,
                carrier_phase_rad: 0.25,
                cn0_db_hz: 75.0,
                navigation_data: false,
            }
        }
        (Constellation::Galileo, SignalBand::E5, SignalCode::E5a | SignalCode::E5b) => {
            TrackingNoiseFixturePlan {
                sample_rate_hz: 10_230_000.0,
                intermediate_freq_hz: 0.0,
                code_freq_basis_hz: 10_230_000.0,
                code_length: 10_230,
                acquisition_doppler_search_hz: 2_000,
                acquisition_doppler_step_hz: 250,
                acquisition_integration_ms: 1,
                duration_s: 0.060,
                seed: 0x6AE5_A000,
                sat: SatId { constellation: Constellation::Galileo, prn: 18 },
                doppler_hz: 750.0,
                code_phase_chips: 2_048.25,
                carrier_phase_rad: 0.4,
                cn0_db_hz: 60.0,
                navigation_data: matches!(signal.signal_code, SignalCode::E5b),
            }
        }
        (Constellation::Glonass, SignalBand::L1, SignalCode::Unknown) => {
            let channel = signal
                .glonass_frequency_channel
                .expect("supported GLONASS signal must carry a frequency channel");
            TrackingNoiseFixturePlan {
                sample_rate_hz: 2_044_000.0,
                intermediate_freq_hz: GPS_L1_CA_CARRIER_HZ.value()
                    - glonass_l1_carrier_hz(channel).value(),
                code_freq_basis_hz: 511_000.0,
                code_length: 511,
                acquisition_doppler_search_hz: 0,
                acquisition_doppler_step_hz: 250,
                acquisition_integration_ms: 1,
                duration_s: 0.140,
                seed: 0x9100_0000,
                sat: representative_sat(signal.constellation),
                doppler_hz: 0.0,
                code_phase_chips: 147.25,
                carrier_phase_rad: 0.5,
                cn0_db_hz: 75.0,
                navigation_data: false,
            }
        }
        (Constellation::Beidou, SignalBand::B1, SignalCode::B1I) => TrackingNoiseFixturePlan {
            sample_rate_hz: 4_092_000.0,
            intermediate_freq_hz: GPS_L1_CA_CARRIER_HZ.value() - BEIDOU_B1_CARRIER_HZ.value(),
            code_freq_basis_hz: 2_046_000.0,
            code_length: 2046,
            acquisition_doppler_search_hz: 0,
            acquisition_doppler_step_hz: 250,
            acquisition_integration_ms: 1,
            duration_s: 0.065,
            seed: 0xB1D0_1111,
            sat: SatId { constellation: Constellation::Beidou, prn: 11 },
            doppler_hz: 0.0,
            code_phase_chips: 213.5,
            carrier_phase_rad: 0.2,
            cn0_db_hz: 60.0,
            navigation_data: false,
        },
        (Constellation::Beidou, SignalBand::B2, SignalCode::B2I) => TrackingNoiseFixturePlan {
            sample_rate_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 2_046_000.0,
            code_length: 2046,
            acquisition_doppler_search_hz: 2_000,
            acquisition_doppler_step_hz: 250,
            acquisition_integration_ms: 1,
            duration_s: 0.060,
            seed: 0xB2D0_2580,
            sat: SatId { constellation: Constellation::Beidou, prn: 11 },
            doppler_hz: 750.0,
            code_phase_chips: 321.375,
            carrier_phase_rad: 0.25,
            cn0_db_hz: 60.0,
            navigation_data: false,
        },
        _ => registry_driven_tracking_noise_fixture_plan(signal),
    }
}

fn registry_driven_tracking_noise_fixture_plan(
    signal: SyntheticTrackingSignalIdentity,
) -> TrackingNoiseFixturePlan {
    let registry = signal_registry(signal.constellation, signal.signal_band, signal.signal_code)
        .expect("supported tracking signal must exist in the signal registry");
    let component = registry
        .default_component()
        .expect("supported tracking signal must expose a default component");
    let long_code_signal = component.primary_code_period_s >= 0.010;
    let sample_rate_hz = if component.primary_code_rate_hz >= 10_000_000.0 {
        component.primary_code_rate_hz
    } else {
        4_092_000.0
    };

    TrackingNoiseFixturePlan {
        sample_rate_hz,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: component.primary_code_rate_hz,
        code_length: component.primary_code_chips as usize,
        acquisition_doppler_search_hz: 2_000,
        acquisition_doppler_step_hz: 250,
        acquisition_integration_ms: 1,
        duration_s: if long_code_signal { 0.800 } else { 0.140 },
        seed: tracking_noise_seed(signal),
        sat: representative_sat(signal.constellation),
        doppler_hz: if long_code_signal { 0.0 } else { 250.0 },
        code_phase_chips: 0.25 * component.primary_code_chips as f64,
        carrier_phase_rad: 0.25,
        cn0_db_hz: if long_code_signal { 75.0 } else { 60.0 },
        navigation_data: false,
    }
}

fn tracking_noise_scenario_id(signal: SyntheticTrackingSignalIdentity) -> String {
    format!(
        "tracking-noise-{}-{}-{}",
        constellation_label(signal.constellation),
        signal_band_label(signal.signal_band),
        signal_code_label(signal.signal_code)
    )
}

fn tracking_noise_seed(signal: SyntheticTrackingSignalIdentity) -> u64 {
    let constellation = match signal.constellation {
        Constellation::Gps => 0x1000,
        Constellation::Galileo => 0x2000,
        Constellation::Glonass => 0x3000,
        Constellation::Beidou => 0x4000,
        Constellation::Unknown => 0x5000,
    };
    let band = match signal.signal_band {
        SignalBand::L1 => 0x0100,
        SignalBand::L2 => 0x0200,
        SignalBand::L5 => 0x0300,
        SignalBand::E1 => 0x0400,
        SignalBand::E5 => 0x0500,
        SignalBand::B1 => 0x0600,
        SignalBand::B2 => 0x0700,
        SignalBand::Unknown => 0x0800,
    };
    let code = match signal.signal_code {
        SignalCode::Ca => 0x0010,
        SignalCode::L2C => 0x0020,
        SignalCode::L5I => 0x0030,
        SignalCode::L5Q => 0x0040,
        SignalCode::E1B => 0x0050,
        SignalCode::E1C => 0x0060,
        SignalCode::E5a => 0x0070,
        SignalCode::E5b => 0x0080,
        SignalCode::B1I => 0x0090,
        SignalCode::B2I => 0x00A0,
        SignalCode::Py => 0x00B0,
        SignalCode::Unknown => 0x00C0,
    };
    constellation | band | code
}

fn representative_sat(constellation: Constellation) -> SatId {
    SatId {
        constellation,
        prn: match constellation {
            Constellation::Glonass => 8,
            _ => 18,
        },
    }
}

fn constellation_label(constellation: Constellation) -> &'static str {
    match constellation {
        Constellation::Gps => "gps",
        Constellation::Galileo => "galileo",
        Constellation::Glonass => "glonass",
        Constellation::Beidou => "beidou",
        Constellation::Unknown => "unknown",
    }
}

fn signal_band_label(band: SignalBand) -> &'static str {
    match band {
        SignalBand::L1 => "l1",
        SignalBand::L2 => "l2",
        SignalBand::L5 => "l5",
        SignalBand::E1 => "e1",
        SignalBand::E5 => "e5",
        SignalBand::B1 => "b1",
        SignalBand::B2 => "b2",
        SignalBand::Unknown => "unknown",
    }
}

fn signal_code_label(code: SignalCode) -> &'static str {
    match code {
        SignalCode::Ca => "ca",
        SignalCode::L2C => "l2c",
        SignalCode::L5I => "l5i",
        SignalCode::L5Q => "l5q",
        SignalCode::E1B => "e1b",
        SignalCode::E1C => "e1c",
        SignalCode::E5a => "e5a",
        SignalCode::E5b => "e5b",
        SignalCode::B1I => "b1i",
        SignalCode::B2I => "b2i",
        SignalCode::Py => "py",
        SignalCode::Unknown => "unknown",
    }
}
