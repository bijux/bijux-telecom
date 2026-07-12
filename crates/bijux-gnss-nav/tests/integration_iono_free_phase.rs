#![allow(missing_docs)]

use bijux_gnss_core::api::{
    first_order_ionosphere_code_delay_m, signal_spec_gps_l1_ca, signal_spec_gps_l2_py,
    signal_spec_gps_l5, Constellation, Cycles, Hertz, LockFlags, Meters, ObsEpoch, ObsMetadata,
    ObsSatellite, ObservationEpochDecision, ObservationStatus, ReceiverRole, ReceiverSampleTrace,
    SatId, Seconds, SigId, SignalBand, SignalCode, SignalSpec,
};
use bijux_gnss_nav::api::{combinations_from_obs_epochs, iono_free_phase_from_obs_epochs};

const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;

fn dual_frequency_epoch(
    second_band: SignalBand,
    second_code: SignalCode,
    second_signal: SignalSpec,
    phase_1_cycles: f64,
    phase_2_cycles: f64,
    code_lock: bool,
    carrier_lock: bool,
) -> ObsEpoch {
    let sat = SatId { constellation: Constellation::Gps, prn: 11 };
    let l1 = signal_spec_gps_l1_ca();

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
        sats: vec![
            satellite(
                sat,
                SignalBand::L1,
                SignalCode::Ca,
                l1,
                phase_1_cycles,
                code_lock,
                carrier_lock,
            ),
            satellite(
                sat,
                second_band,
                second_code,
                second_signal,
                phase_2_cycles,
                code_lock,
                carrier_lock,
            ),
        ],
        decision: ObservationEpochDecision::Accepted,
        decision_reason: Some("accepted_observables_present".to_string()),
        manifest: None,
    }
}

fn satellite(
    sat: SatId,
    band: SignalBand,
    code: SignalCode,
    signal: SignalSpec,
    phase_cycles: f64,
    code_lock: bool,
    carrier_lock: bool,
) -> ObsSatellite {
    ObsSatellite {
        signal_id: SigId { sat, band, code },
        pseudorange_m: Meters(0.0),
        pseudorange_var_m2: f64::NAN,
        carrier_phase_cycles: Cycles(phase_cycles),
        carrier_phase_var_cycles2: 0.01,
        doppler_hz: Hertz(0.0),
        doppler_var_hz2: 1.0,
        cn0_dbhz: 45.0,
        lock_flags: LockFlags { code_lock, carrier_lock, bit_lock: false, cycle_slip: false },
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

fn carrier_cycles(range_m: f64, iono_m: f64, wavelength_m: f64, ambiguity_cycles: f64) -> f64 {
    (range_m - iono_m) / wavelength_m + ambiguity_cycles
}

fn dispersive_delay_at_band(
    iono_l1_m: f64,
    l1_signal: SignalSpec,
    target_signal: SignalSpec,
) -> f64 {
    first_order_ionosphere_code_delay_m(Meters(iono_l1_m), l1_signal, target_signal)
        .expect("finite first-order ionosphere delay")
        .0
}

#[test]
fn iono_free_phase_recovers_geometric_range_for_l1_l2() {
    let base_range_m = 20_200_000.0;
    let iono_l1_m = 5.0;
    let l1 = signal_spec_gps_l1_ca();
    let l2 = signal_spec_gps_l2_py();
    let epoch = dual_frequency_epoch(
        SignalBand::L2,
        SignalCode::Py,
        l2,
        carrier_cycles(base_range_m, iono_l1_m, SPEED_OF_LIGHT_MPS / l1.carrier_hz.value(), 0.0),
        carrier_cycles(
            base_range_m,
            dispersive_delay_at_band(iono_l1_m, l1, l2),
            SPEED_OF_LIGHT_MPS / l2.carrier_hz.value(),
            0.0,
        ),
        false,
        true,
    );

    let observations = iono_free_phase_from_obs_epochs(&[epoch], SignalBand::L1, SignalBand::L2);

    assert_eq!(observations.len(), 1);
    assert_eq!(observations[0].status, "ok");
    assert!((observations[0].phase_m.expect("iono-free phase") - base_range_m).abs() < 1.0e-6);
}

#[test]
fn iono_free_phase_recovers_geometric_range_for_l1_l5() {
    let base_range_m = 20_200_000.0;
    let iono_l1_m = 5.0;
    let l1 = signal_spec_gps_l1_ca();
    let l5 = signal_spec_gps_l5();
    let epoch = dual_frequency_epoch(
        SignalBand::L5,
        SignalCode::Unknown,
        l5,
        carrier_cycles(base_range_m, iono_l1_m, SPEED_OF_LIGHT_MPS / l1.carrier_hz.value(), 0.0),
        carrier_cycles(
            base_range_m,
            dispersive_delay_at_band(iono_l1_m, l1, l5),
            SPEED_OF_LIGHT_MPS / l5.carrier_hz.value(),
            0.0,
        ),
        false,
        true,
    );

    let observations = iono_free_phase_from_obs_epochs(&[epoch], SignalBand::L1, SignalBand::L5);

    assert_eq!(observations.len(), 1);
    assert_eq!(observations[0].status, "ok");
    assert!((observations[0].phase_m.expect("iono-free phase") - base_range_m).abs() < 1.0e-6);
}

#[test]
fn iono_free_phase_preserves_narrow_lane_ambiguity_projection() {
    let base_range_m = 20_200_000.0;
    let iono_l1_m = 5.0;
    let ambiguity_l1_cycles = 17.0;
    let ambiguity_l2_cycles = 11.0;
    let l1 = signal_spec_gps_l1_ca();
    let l2 = signal_spec_gps_l2_py();
    let epoch = dual_frequency_epoch(
        SignalBand::L2,
        SignalCode::Py,
        l2,
        carrier_cycles(
            base_range_m,
            iono_l1_m,
            SPEED_OF_LIGHT_MPS / l1.carrier_hz.value(),
            ambiguity_l1_cycles,
        ),
        carrier_cycles(
            base_range_m,
            dispersive_delay_at_band(iono_l1_m, l1, l2),
            SPEED_OF_LIGHT_MPS / l2.carrier_hz.value(),
            ambiguity_l2_cycles,
        ),
        false,
        true,
    );

    let observations = iono_free_phase_from_obs_epochs(&[epoch], SignalBand::L1, SignalBand::L2);
    let observation = &observations[0];
    let expected_narrow_lane_ambiguity_cycles = (l1.carrier_hz.value() * ambiguity_l1_cycles
        - l2.carrier_hz.value() * ambiguity_l2_cycles)
        / (l1.carrier_hz.value() - l2.carrier_hz.value());

    assert_eq!(observation.status, "ok");
    assert!(
        (observation.phase_cycles.expect("phase cycles")
            - (base_range_m / observation.narrow_lane_wavelength_m.expect("wavelength")
                + expected_narrow_lane_ambiguity_cycles))
            .abs()
            < 1.0e-6
    );
}

#[test]
fn iono_free_phase_stays_available_when_code_inputs_are_invalid() {
    let epoch = dual_frequency_epoch(
        SignalBand::L2,
        SignalCode::Py,
        signal_spec_gps_l2_py(),
        1000.0,
        1001.0,
        false,
        true,
    );

    let phase = iono_free_phase_from_obs_epochs(&[epoch.clone()], SignalBand::L1, SignalBand::L2);
    let combinations = combinations_from_obs_epochs(&[epoch], SignalBand::L1, SignalBand::L2);

    assert_eq!(phase.len(), 1);
    assert_eq!(phase[0].status, "ok");
    assert!(phase[0].phase_m.is_some());

    assert_eq!(combinations.len(), 1);
    assert_eq!(combinations[0].status, "invalid");
    assert_eq!(combinations[0].if_phase_status, "ok");
    assert!(combinations[0].if_phase_m.is_some());
    assert!(combinations[0].if_code_m.is_none());
}
