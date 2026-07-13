#![allow(missing_docs)]

use bijux_gnss_core::api::{
    first_order_ionosphere_code_delay_m, signal_meters_to_cycles, signal_spec_gps_l1_ca,
    signal_spec_gps_l5, Constellation, Cycles, Hertz, LockFlags, Meters, ObsEpoch, ObsMetadata,
    ObsSatellite, ObservationEpochDecision, ObservationStatus, ReceiverRole, ReceiverSampleTrace,
    SatId, Seconds, SigId, SignalBand, SignalCode, SignalSpec,
};
use bijux_gnss_nav::api::measured_ionosphere_from_obs_epochs;

fn dual_frequency_epoch(
    sat: SatId,
    epoch_idx: u64,
    discontinuity: bool,
    first_band: SignalBand,
    first_code: SignalCode,
    first_signal: SignalSpec,
    first_code_m: f64,
    second_band: SignalBand,
    second_code: SignalCode,
    second_signal: SignalSpec,
    second_code_m: f64,
    phase_bias_m: f64,
    cycle_slip: bool,
) -> ObsEpoch {
    let geometry_free_code_m = second_code_m - first_code_m;
    let geometry_free_phase_m = geometry_free_code_m + phase_bias_m;
    let reference_phase_m = 22_000_000.0;

    ObsEpoch {
        t_rx_s: Seconds(epoch_idx as f64),
        source_time: ReceiverSampleTrace::from_sample_index(epoch_idx, 1_000.0),
        gps_week: None,
        tow_s: None,
        epoch_idx,
        discontinuity,
        valid: true,
        processing_ms: None,
        role: ReceiverRole::Rover,
        sats: vec![
            satellite(
                sat,
                first_band,
                first_code,
                first_signal,
                first_code_m,
                signal_meters_to_cycles(
                    Meters(reference_phase_m + geometry_free_phase_m),
                    first_signal,
                )
                .0,
                cycle_slip,
            ),
            satellite(
                sat,
                second_band,
                second_code,
                second_signal,
                second_code_m,
                signal_meters_to_cycles(Meters(reference_phase_m), second_signal).0,
                cycle_slip,
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
    pseudorange_m: f64,
    carrier_phase_cycles: f64,
    cycle_slip: bool,
) -> ObsSatellite {
    ObsSatellite {
        signal_id: SigId { sat, band, code },
        pseudorange_m: Meters(pseudorange_m),
        pseudorange_var_m2: 1.0,
        carrier_phase_cycles: Cycles(carrier_phase_cycles),
        carrier_phase_var_cycles2: 0.01,
        doppler_hz: Hertz(0.0),
        doppler_var_hz2: 1.0,
        cn0_dbhz: 45.0,
        lock_flags: LockFlags { code_lock: true, carrier_lock: true, bit_lock: false, cycle_slip },
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

#[test]
fn measured_ionosphere_recovers_gps_l1_l5_slant_delay() {
    let sat = SatId { constellation: Constellation::Gps, prn: 11 };
    let l1 = signal_spec_gps_l1_ca();
    let l5 = signal_spec_gps_l5();
    let l1_delay_m = 6.5;
    let l5_delay_m = first_order_ionosphere_code_delay_m(Meters(l1_delay_m), l1, l5)
        .expect("finite L5 ionosphere delay")
        .0;

    let observations = measured_ionosphere_from_obs_epochs(
        &[dual_frequency_epoch(
            sat,
            0,
            false,
            SignalBand::L1,
            SignalCode::Ca,
            l1,
            24_000_000.0 + l1_delay_m,
            SignalBand::L5,
            SignalCode::Unknown,
            l5,
            24_000_000.0 + l5_delay_m,
            4.25,
            false,
        )],
        SignalBand::L1,
        SignalBand::L5,
    );

    assert_eq!(observations.len(), 1);
    let observation = &observations[0];
    assert_eq!(observation.code_status, "ok");
    assert_eq!(observation.phase_status, "ok");
    assert_eq!(observation.phase_arc_reset, true);
    assert!((observation.code_delay_band_1_m.expect("L1 delay") - l1_delay_m).abs() < 1.0e-6);
    assert!((observation.code_delay_band_2_m.expect("L5 delay") - l5_delay_m).abs() < 1.0e-6);
    assert!(
        (observation.phase_delay_band_1_m.expect("phase L1 delay") - l1_delay_m).abs() < 1.0e-6
    );
    assert!(
        (observation.phase_delay_band_2_m.expect("phase L5 delay") - l5_delay_m).abs() < 1.0e-6
    );
}

#[test]
fn measured_ionosphere_relevels_gps_l1_l5_phase_arc_after_cycle_slip() {
    let sat = SatId { constellation: Constellation::Gps, prn: 11 };
    let l1 = signal_spec_gps_l1_ca();
    let l5 = signal_spec_gps_l5();
    let epochs = vec![
        dual_frequency_epoch(
            sat,
            0,
            false,
            SignalBand::L1,
            SignalCode::Ca,
            l1,
            24_000_006.5,
            SignalBand::L5,
            SignalCode::Unknown,
            l5,
            24_000_003.0,
            4.25,
            false,
        ),
        dual_frequency_epoch(
            sat,
            1,
            false,
            SignalBand::L1,
            SignalCode::Ca,
            l1,
            24_000_006.5,
            SignalBand::L5,
            SignalCode::Unknown,
            l5,
            24_000_003.0,
            6.75,
            true,
        ),
    ];

    let observations = measured_ionosphere_from_obs_epochs(&epochs, SignalBand::L1, SignalBand::L5);

    assert_eq!(observations.len(), 2);
    assert_eq!(observations[0].phase_status, "ok");
    assert_eq!(observations[1].phase_status, "ok");
    assert!(observations[0].phase_arc_reset);
    assert!(observations[1].phase_arc_reset);
    assert!(
        (observations[1].phase_level_bias_m.expect("new bias")
            - observations[0].phase_level_bias_m.expect("old bias"))
        .abs()
            > 1.0
    );
    assert!(
        (observations[0].phase_delay_band_1_m.expect("first epoch L1 delay")
            - observations[1].phase_delay_band_1_m.expect("second epoch L1 delay"))
        .abs()
            < 1.0e-6
    );
}
