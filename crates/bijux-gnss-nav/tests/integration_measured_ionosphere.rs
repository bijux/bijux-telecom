#![allow(missing_docs)]

use bijux_gnss_core::api::{
    Constellation, Cycles, Hertz, LockFlags, Meters, ObsEpoch, ObsMetadata, ObsSatellite,
    ObservationEpochDecision, ObservationStatus, ReceiverRole, ReceiverSampleTrace, SatId, Seconds,
    SigId, SignalBand, SignalCode, SignalSpec,
};
use bijux_gnss_nav::api::measured_ionosphere_from_obs_epochs;
use bijux_gnss_signal::api::{
    first_order_ionosphere_code_delay_m, signal_meters_to_cycles, signal_spec_beidou_b1i,
    signal_spec_beidou_b2i, signal_spec_galileo_e1b, signal_spec_galileo_e5a,
    signal_spec_gps_l1_ca, signal_spec_gps_l2_py,
};

#[derive(Clone, Copy)]
struct DualFrequencySignalObservation {
    band: SignalBand,
    code: SignalCode,
    signal: SignalSpec,
    pseudorange_m: f64,
}

#[derive(Clone, Copy)]
struct DualFrequencyEpochRequest {
    sat: SatId,
    first_observation: DualFrequencySignalObservation,
    second_observation: DualFrequencySignalObservation,
    phase_bias_m: f64,
}

#[derive(Clone, Copy)]
struct MeasuredIonosphereRecoveryRequest {
    sat: SatId,
    first_observation: DualFrequencySignalObservation,
    second_observation: DualFrequencySignalObservation,
    first_delay_m: f64,
}

fn dual_frequency_epoch(request: DualFrequencyEpochRequest) -> ObsEpoch {
    let geometry_free_code_m =
        request.second_observation.pseudorange_m - request.first_observation.pseudorange_m;
    let geometry_free_phase_m = geometry_free_code_m + request.phase_bias_m;
    let reference_phase_m = 22_000_000.0;

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
                request.sat,
                request.first_observation.band,
                request.first_observation.code,
                request.first_observation.signal,
                request.first_observation.pseudorange_m,
                signal_meters_to_cycles(
                    Meters(reference_phase_m + geometry_free_phase_m),
                    request.first_observation.signal,
                )
                .0,
            ),
            satellite(
                request.sat,
                request.second_observation.band,
                request.second_observation.code,
                request.second_observation.signal,
                request.second_observation.pseudorange_m,
                signal_meters_to_cycles(
                    Meters(reference_phase_m),
                    request.second_observation.signal,
                )
                .0,
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

fn assert_measured_ionosphere_recovery(request: MeasuredIonosphereRecoveryRequest) {
    let base_range_m = 24_000_000.0;
    let second_delay_m = first_order_ionosphere_code_delay_m(
        Meters(request.first_delay_m),
        request.first_observation.signal,
        request.second_observation.signal,
    )
    .expect("finite second-band delay")
    .0;
    let observations = measured_ionosphere_from_obs_epochs(
        &[dual_frequency_epoch(DualFrequencyEpochRequest {
            sat: request.sat,
            first_observation: DualFrequencySignalObservation {
                pseudorange_m: base_range_m + request.first_delay_m,
                ..request.first_observation
            },
            second_observation: DualFrequencySignalObservation {
                pseudorange_m: base_range_m + second_delay_m,
                ..request.second_observation
            },
            phase_bias_m: 4.25,
        })],
        request.first_observation.band,
        request.second_observation.band,
    );

    assert_eq!(observations.len(), 1);
    let observation = &observations[0];
    assert_eq!(observation.code_status, "ok");
    assert_eq!(observation.phase_status, "ok");
    assert!(observation.phase_arc_reset);
    assert!(
        (observation.code_delay_band_1_m.expect("band 1 code delay") - request.first_delay_m).abs()
            < 1.0e-6
    );
    assert!(
        (observation.code_delay_band_2_m.expect("band 2 code delay") - second_delay_m).abs()
            < 1.0e-6
    );
    assert!(
        (observation.phase_delay_band_1_m.expect("band 1 phase delay") - request.first_delay_m)
            .abs()
            < 1.0e-6
    );
    assert!(
        (observation.phase_delay_band_2_m.expect("band 2 phase delay") - second_delay_m).abs()
            < 1.0e-6
    );
}

#[test]
fn measured_ionosphere_recovers_gps_l1_l2_delay() {
    assert_measured_ionosphere_recovery(MeasuredIonosphereRecoveryRequest {
        sat: SatId { constellation: Constellation::Gps, prn: 11 },
        first_observation: DualFrequencySignalObservation {
            band: SignalBand::L1,
            code: SignalCode::Ca,
            signal: signal_spec_gps_l1_ca(),
            pseudorange_m: 0.0,
        },
        second_observation: DualFrequencySignalObservation {
            band: SignalBand::L2,
            code: SignalCode::Py,
            signal: signal_spec_gps_l2_py(),
            pseudorange_m: 0.0,
        },
        first_delay_m: 6.5,
    });
}

#[test]
fn measured_ionosphere_recovers_galileo_e1_e5_delay() {
    assert_measured_ionosphere_recovery(MeasuredIonosphereRecoveryRequest {
        sat: SatId { constellation: Constellation::Galileo, prn: 19 },
        first_observation: DualFrequencySignalObservation {
            band: SignalBand::E1,
            code: SignalCode::E1B,
            signal: signal_spec_galileo_e1b(),
            pseudorange_m: 0.0,
        },
        second_observation: DualFrequencySignalObservation {
            band: SignalBand::E5,
            code: SignalCode::E5a,
            signal: signal_spec_galileo_e5a(),
            pseudorange_m: 0.0,
        },
        first_delay_m: 7.25,
    });
}

#[test]
fn measured_ionosphere_recovers_beidou_b1_b2_delay() {
    assert_measured_ionosphere_recovery(MeasuredIonosphereRecoveryRequest {
        sat: SatId { constellation: Constellation::Beidou, prn: 11 },
        first_observation: DualFrequencySignalObservation {
            band: SignalBand::B1,
            code: SignalCode::B1I,
            signal: signal_spec_beidou_b1i(),
            pseudorange_m: 0.0,
        },
        second_observation: DualFrequencySignalObservation {
            band: SignalBand::B2,
            code: SignalCode::B2I,
            signal: signal_spec_beidou_b2i(),
            pseudorange_m: 0.0,
        },
        first_delay_m: 8.0,
    });
}
