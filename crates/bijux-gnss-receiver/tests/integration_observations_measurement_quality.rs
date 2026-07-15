#![allow(missing_docs)]

use std::collections::BTreeMap;

use bijux_gnss_core::api::{
    Chips, Constellation, CycleSlipDetector, Cycles, Epoch, Hertz, ReceiverSampleTrace, SatId,
    SignalBand, SignalDelayAlignment, SignalSpec, TrackEpoch, TrackingUncertainty,
    GPS_L1_CA_CARRIER_HZ,
};
use bijux_gnss_receiver::api::{
    observation_measurement_quality_from_tracking_results, ReceiverPipelineConfig, TrackingResult,
};
use bijux_gnss_signal::api::{
    signal_spec_beidou_b1i, signal_spec_beidou_b2i, signal_spec_galileo_e1b,
    signal_spec_galileo_e5a, signal_spec_gps_l1_ca, signal_spec_gps_l2c, signal_spec_gps_l5,
};

fn observation_config() -> ReceiverPipelineConfig {
    ReceiverPipelineConfig {
        sampling_freq_hz: 10_230_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 8,
        early_late_spacing_chips: 0.5,
        dll_bw_hz: 2.0,
        pll_bw_hz: 15.0,
        fll_bw_hz: 10.0,
        tracking_integration_ms: 10,
        ..ReceiverPipelineConfig::default()
    }
}

fn tracking_epoch(
    config: &ReceiverPipelineConfig,
    sat: SatId,
    signal: SignalSpec,
    cn0_dbhz: f64,
) -> TrackEpoch {
    let tracked_carrier_hz = signal.carrier_hz.value() - GPS_L1_CA_CARRIER_HZ.value() + 125.0;
    TrackEpoch {
        epoch: Epoch { index: 0 },
        sample_index: 0,
        source_time: ReceiverSampleTrace::from_sample_index(0, config.sampling_freq_hz),
        sat,
        signal_band: signal.band,
        signal_code: signal.code,
        glonass_frequency_channel: None,
        prompt_i: 1.0,
        prompt_q: 0.0,
        early_i: 0.1,
        early_q: 0.0,
        late_i: -0.1,
        late_q: 0.0,
        carrier_hz: Hertz(tracked_carrier_hz + sat.prn as f64),
        carrier_phase_cycles: Cycles(2_400.0 + sat.prn as f64),
        code_rate_hz: Hertz(signal.code_rate_hz),
        code_phase_samples: Chips(0.0),
        lock: true,
        cn0_dbhz,
        pll_lock: true,
        dll_lock: true,
        fll_lock: true,
        cycle_slip: false,
        nav_bit_lock: false,
        navigation_bit_sign: None,
        dll_err: 0.0,
        pll_err: 0.0,
        fll_err: 0.0,
        anti_false_lock: false,
        cycle_slip_reason: None,
        lock_state: "tracking".to_string(),
        lock_state_reason: Some("stable_tracking".to_string()),
        channel_id: Some(sat.prn),
        channel_uid: format!("{:?}-{:02}-{:?}", sat.constellation, sat.prn, signal.band),
        tracking_provenance: "integration_observations_measurement_quality".to_string(),
        tracking_assumptions: None,
        signal_delay_alignment: Some(SignalDelayAlignment {
            whole_code_periods: quality_alignment_periods(signal.band),
            sample_delay_samples: 0,
            source: "quality_fixture".to_string(),
        }),
        transmit_time: None,
        tracking_uncertainty: Some(TrackingUncertainty {
            code_phase_samples: 0.05,
            carrier_phase_cycles: 0.02,
            doppler_hz: 0.5,
            cn0_dbhz: 0.75,
        }),
        processing_ms: None,
    }
}

fn quality_alignment_periods(band: SignalBand) -> u64 {
    match band {
        SignalBand::L1 => 68,
        SignalBand::L2 => 4,
        SignalBand::L5 => 8,
        SignalBand::E1 => 17,
        SignalBand::E5 => 6,
        SignalBand::B1 => 8,
        SignalBand::B2 => 8,
        _ => 68,
    }
}

fn track(epoch: TrackEpoch) -> TrackingResult {
    let sat = epoch.sat;
    TrackingResult {
        sat,
        carrier_hz: epoch.carrier_hz.0,
        code_phase_samples: epoch.code_phase_samples.0,
        acquisition_hypothesis: "accepted".to_string(),
        acquisition_score: 1.0,
        acquisition_code_phase_samples: 0,
        acquisition_carrier_hz: epoch.carrier_hz.0,
        acq_to_track_state: "accepted".to_string(),
        epochs: vec![epoch],
        transitions: Vec::new(),
    }
}

#[test]
fn measurement_quality_reports_supported_signal_bands() {
    let config = observation_config();
    let mut l5 = tracking_epoch(
        &config,
        SatId { constellation: Constellation::Gps, prn: 5 },
        signal_spec_gps_l5(),
        43.0,
    );
    l5.cycle_slip = true;
    l5.cycle_slip_reason = Some("simulated_phase_slip".to_string());

    let mut b2 = tracking_epoch(
        &config,
        SatId { constellation: Constellation::Beidou, prn: 8 },
        signal_spec_beidou_b2i(),
        39.0,
    );
    b2.pll_lock = false;

    let tracks = vec![
        track(tracking_epoch(
            &config,
            SatId { constellation: Constellation::Gps, prn: 3 },
            signal_spec_gps_l1_ca(),
            46.0,
        )),
        track(tracking_epoch(
            &config,
            SatId { constellation: Constellation::Gps, prn: 4 },
            signal_spec_gps_l2c(),
            44.0,
        )),
        track(l5),
        track(tracking_epoch(
            &config,
            SatId { constellation: Constellation::Galileo, prn: 11 },
            signal_spec_galileo_e1b(),
            42.0,
        )),
        track(tracking_epoch(
            &config,
            SatId { constellation: Constellation::Galileo, prn: 12 },
            signal_spec_galileo_e5a(),
            41.0,
        )),
        track(tracking_epoch(
            &config,
            SatId { constellation: Constellation::Beidou, prn: 7 },
            signal_spec_beidou_b1i(),
            40.0,
        )),
        track(b2),
    ];

    let report = observation_measurement_quality_from_tracking_results(&config, &tracks, 10);
    assert!(report.events.is_empty(), "{:#?}", report.events);
    assert_eq!(report.output.len(), 1);

    let epoch = &report.output[0];
    let by_band =
        epoch.sats.iter().map(|sat| (sat.signal_id.band, sat)).collect::<BTreeMap<_, _>>();

    assert_eq!(
        by_band.keys().copied().collect::<Vec<_>>(),
        vec![
            SignalBand::L1,
            SignalBand::L2,
            SignalBand::L5,
            SignalBand::E1,
            SignalBand::E5,
            SignalBand::B1,
            SignalBand::B2,
        ]
    );

    for band in [
        SignalBand::L1,
        SignalBand::L2,
        SignalBand::L5,
        SignalBand::E1,
        SignalBand::E5,
        SignalBand::B1,
        SignalBand::B2,
    ] {
        let sat = by_band.get(&band).expect("measurement quality row");
        assert!(sat.cn0_dbhz > 0.0, "{sat:?}");
        assert!(sat.pseudorange_sigma_m.is_some_and(|sigma| sigma > 0.0), "{sat:?}");
        assert!(sat.carrier_phase_sigma_cycles.is_some_and(|sigma| sigma > 0.0), "{sat:?}");
        assert!(sat.doppler_sigma_hz.is_some_and(|sigma| sigma > 0.0), "{sat:?}");
        assert!(sat.cn0_sigma_dbhz.is_some_and(|sigma| sigma > 0.0), "{sat:?}");
        let covariance = sat.measurement_covariance.expect("measurement covariance");
        assert_eq!(
            covariance.status,
            bijux_gnss_core::api::ObservationCovarianceStatus::PositiveSemidefinite
        );
        assert!(covariance.code_phase_m2 > 0.0, "{covariance:?}");
        assert!(covariance.carrier_phase_m2 > 0.0, "{covariance:?}");
        assert!(covariance.doppler_hz2 > 0.0, "{covariance:?}");
        assert_eq!(covariance.code_carrier_m2, covariance.carrier_code_m2);
        assert_eq!(covariance.carrier_doppler_m_hz, covariance.doppler_carrier_hz_m);
        let divergence = sat.code_carrier_divergence.expect("code-carrier divergence");
        assert!(divergence.raw_m.is_finite(), "{divergence:?}");
        assert!(divergence.jump_m.is_finite(), "{divergence:?}");
        assert!(divergence.expected_ionosphere_m.is_finite(), "{divergence:?}");
        assert!(divergence.multipath_m.is_finite(), "{divergence:?}");
        assert!(divergence.unexplained_m.is_finite(), "{divergence:?}");
        let slip_evidence = sat.cycle_slip_evidence.as_ref().expect("cycle-slip evidence");
        assert!(slip_evidence.detection_probability_budget > 0.0, "{slip_evidence:?}");
        assert!(slip_evidence.false_alarm_probability_budget > 0.0, "{slip_evidence:?}");
        assert!(!sat.observation_lock_state.is_empty(), "{sat:?}");
    }

    let l5_quality = by_band.get(&SignalBand::L5).expect("L5 measurement quality");
    assert!(l5_quality.cycle_slip, "{l5_quality:?}");
    assert_eq!(l5_quality.cycle_slip_reason.as_deref(), Some("simulated_phase_slip"));
    assert!(l5_quality
        .cycle_slip_evidence
        .as_ref()
        .expect("L5 cycle-slip evidence")
        .triggered_detectors()
        .contains(&CycleSlipDetector::TrackingLock));

    let b2_quality = by_band.get(&SignalBand::B2).expect("B2 measurement quality");
    assert!(b2_quality.lock_flags.code_lock, "{b2_quality:?}");
    assert!(!b2_quality.lock_flags.carrier_lock, "{b2_quality:?}");
}
