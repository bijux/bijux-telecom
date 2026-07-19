#![allow(missing_docs)]

use bijux_gnss_core::api::{
    Constellation, Cycles, GpsTime, Hertz, LockFlags, Meters, ObsEpoch, ObsMetadata, ObsSatellite,
    ObsSignalTiming, ObservationEpochDecision, ObservationStatus, ReceiverRole,
    ReceiverSampleTrace, SatId, Seconds, SigId, SignalBand, SignalCode, SignalSpec,
};
use bijux_gnss_nav::api::{
    combinations_from_obs_epochs, iono_free_code_from_obs_epochs, iono_free_phase_from_obs_epochs,
    measured_ionosphere_from_obs_epochs, narrow_lane_from_obs_epochs, ppp_ionosphere_delay_scale,
};
use bijux_gnss_signal::api::{
    signal_spec_galileo_e1b, signal_spec_galileo_e5a, signal_spec_gps_l1_ca, signal_spec_gps_l2_py,
};

fn satellite(sat: SatId, band: SignalBand, code: SignalCode, signal: SignalSpec) -> ObsSatellite {
    ObsSatellite {
        signal_id: SigId { sat, band, code },
        pseudorange_m: Meters(20_200_000.0),
        pseudorange_var_m2: 1.0,
        carrier_phase_cycles: Cycles(1000.0),
        carrier_phase_var_cycles2: 0.01,
        doppler_hz: Hertz(0.0),
        doppler_var_hz2: 1.0,
        cn0_dbhz: 45.0,
        lock_flags: LockFlags {
            code_lock: true,
            carrier_lock: true,
            bit_lock: false,
            cycle_slip: false,
        },
        multipath_suspect: false,
        observation_status: ObservationStatus::Accepted,
        observation_reject_reasons: Vec::new(),
        elevation_deg: None,
        azimuth_deg: None,
        weight: None,
        timing: None,
        error_model: None,
        metadata: ObsMetadata {
            tracking_mode: "test".to_string(),
            integration_ms: 1,
            lock_quality: 45.0,
            smoothing_window: 0,
            smoothing_age: 0,
            smoothing_resets: 0,
            signal,
            ..ObsMetadata::default()
        },
    }
}

fn epoch_with_sats(sats: Vec<ObsSatellite>) -> ObsEpoch {
    ObsEpoch {
        t_rx_s: Seconds(0.0),
        source_time: ReceiverSampleTrace::from_sample_index(0, 1_000.0),
        gps_week: None,
        tow_s: None,
        epoch_idx: 0,
        discontinuity: false,
        valid: true,
        processing_ms: None,
        role: ReceiverRole::Rover,
        sats,
        decision: ObservationEpochDecision::Accepted,
        decision_reason: Some("accepted_observables_present".to_string()),
        manifest: None,
    }
}

#[test]
fn public_dual_frequency_outputs_refuse_mixed_constellation_requests() {
    let gps_sat = SatId { constellation: Constellation::Gps, prn: 11 };
    let galileo_sat = SatId { constellation: Constellation::Galileo, prn: 19 };
    let epoch = epoch_with_sats(vec![
        satellite(gps_sat, SignalBand::L1, SignalCode::Ca, signal_spec_gps_l1_ca()),
        satellite(gps_sat, SignalBand::L2, SignalCode::Py, signal_spec_gps_l2_py()),
        satellite(galileo_sat, SignalBand::E1, SignalCode::E1B, signal_spec_galileo_e1b()),
        satellite(galileo_sat, SignalBand::E5, SignalCode::E5a, signal_spec_galileo_e5a()),
    ]);

    let combinations =
        combinations_from_obs_epochs(&[epoch.clone()], SignalBand::L1, SignalBand::L2);
    let iono_free_code =
        iono_free_code_from_obs_epochs(&[epoch.clone()], SignalBand::L1, SignalBand::L2);
    let iono_free_phase =
        iono_free_phase_from_obs_epochs(&[epoch.clone()], SignalBand::L1, SignalBand::L2);
    let measured_ionosphere =
        measured_ionosphere_from_obs_epochs(&[epoch.clone()], SignalBand::L1, SignalBand::L2);
    let narrow_lane = narrow_lane_from_obs_epochs(&[epoch], SignalBand::L1, SignalBand::L2);

    assert_eq!(combinations.len(), 2);
    assert_eq!(iono_free_code.len(), 2);
    assert_eq!(iono_free_phase.len(), 2);
    assert_eq!(measured_ionosphere.len(), 2);
    assert_eq!(narrow_lane.len(), 2);

    let invalid_combination = combinations
        .iter()
        .find(|observation| observation.sat.constellation == Constellation::Galileo)
        .expect("galileo refusal");
    let invalid_code = iono_free_code
        .iter()
        .find(|observation| observation.sat.constellation == Constellation::Galileo)
        .expect("galileo code refusal");
    let invalid_phase = iono_free_phase
        .iter()
        .find(|observation| observation.sat.constellation == Constellation::Galileo)
        .expect("galileo phase refusal");
    let invalid_measured_ionosphere = measured_ionosphere
        .iter()
        .find(|observation| observation.sat.constellation == Constellation::Galileo)
        .expect("galileo measured ionosphere refusal");
    let invalid_narrow_lane = narrow_lane
        .iter()
        .find(|observation| observation.sat.constellation == Constellation::Galileo)
        .expect("galileo narrow-lane refusal");

    assert_eq!(invalid_combination.reason, "unsupported_band_pair");
    assert_eq!(invalid_code.reason, "unsupported_band_pair");
    assert_eq!(invalid_phase.reason, "unsupported_band_pair");
    assert_eq!(invalid_measured_ionosphere.code_reason, "unsupported_band_pair");
    assert_eq!(invalid_measured_ionosphere.phase_reason, "unsupported_band_pair");
    assert_eq!(invalid_narrow_lane.reason, "unsupported_band_pair");
}

#[test]
fn public_ppp_ionosphere_scale_follows_signal_frequency() {
    let l1 = signal_spec_gps_l1_ca();
    let l2 = signal_spec_gps_l2_py();

    let l1_scale = ppp_ionosphere_delay_scale(l1);
    let l2_scale = ppp_ionosphere_delay_scale(l2);
    let expected_l2_scale = (l1.carrier_hz.value() / l2.carrier_hz.value()).powi(2);

    assert!((l1_scale - 1.0).abs() < 1.0e-12);
    assert!((l2_scale - expected_l2_scale).abs() < 1.0e-12);
    assert!(l2_scale > l1_scale);
}

#[test]
fn public_dual_frequency_outputs_refuse_mismatched_transmit_time_references() {
    let sat = SatId { constellation: Constellation::Gps, prn: 11 };
    let mut l1 = satellite(sat, SignalBand::L1, SignalCode::Ca, signal_spec_gps_l1_ca());
    let mut l2 = satellite(sat, SignalBand::L2, SignalCode::Py, signal_spec_gps_l2_py());
    l1.timing = Some(ObsSignalTiming {
        signal_travel_time_s: Seconds(0.075),
        transmit_gps_time: GpsTime { week: 2200, tow_s: 345_600.0 },
    });
    l2.timing = Some(ObsSignalTiming {
        signal_travel_time_s: Seconds(0.075),
        transmit_gps_time: GpsTime { week: 2201, tow_s: 0.0 },
    });
    let epoch = epoch_with_sats(vec![l1, l2]);

    let combination =
        combinations_from_obs_epochs(&[epoch.clone()], SignalBand::L1, SignalBand::L2);
    let iono_free_code =
        iono_free_code_from_obs_epochs(&[epoch.clone()], SignalBand::L1, SignalBand::L2);
    let iono_free_phase =
        iono_free_phase_from_obs_epochs(&[epoch.clone()], SignalBand::L1, SignalBand::L2);
    let measured_ionosphere =
        measured_ionosphere_from_obs_epochs(&[epoch.clone()], SignalBand::L1, SignalBand::L2);
    let narrow_lane = narrow_lane_from_obs_epochs(&[epoch], SignalBand::L1, SignalBand::L2);

    assert_eq!(combination[0].reason, "time_system_mismatch");
    assert_eq!(iono_free_code[0].reason, "time_system_mismatch");
    assert_eq!(iono_free_phase[0].reason, "time_system_mismatch");
    assert_eq!(measured_ionosphere[0].code_reason, "time_system_mismatch");
    assert_eq!(measured_ionosphere[0].phase_reason, "time_system_mismatch");
    assert_eq!(narrow_lane[0].reason, "time_system_mismatch");
}
