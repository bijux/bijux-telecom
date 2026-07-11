#![allow(missing_docs)]

use bijux_gnss_core::api::{
    ArtifactPayloadValidate, Constellation, GpsTime, ObsSignalTiming, SatId, Seconds, SigId,
    SignalBand,
};
use bijux_gnss_receiver::api::{DdObservation, SdObservation};

fn sample_sd_observation() -> SdObservation {
    SdObservation {
        sig: SigId {
            sat: SatId { constellation: Constellation::Gps, prn: 7 },
            band: SignalBand::L1,
            code: bijux_gnss_core::api::SignalCode::Ca,
        },
        rover_pseudorange_m: 20_000_250.0,
        rover_signal_timing: Some(ObsSignalTiming {
            signal_travel_time_s: Seconds(0.067_001),
            transmit_gps_time: GpsTime { week: 2200, tow_s: 345_600.123_000 },
        }),
        base_pseudorange_m: 20_000_100.0,
        base_signal_timing: Some(ObsSignalTiming {
            signal_travel_time_s: Seconds(0.066_998),
            transmit_gps_time: GpsTime { week: 2200, tow_s: 345_600.123_003 },
        }),
        code_m: 150.0,
        phase_cycles: 0.5,
        doppler_hz: -5.0,
        code_variance_m2: 4.0,
        phase_variance_cycles2: 0.01,
        ambiguity_rover: bijux_gnss_core::api::AmbiguityId {
            sig: SigId {
                sat: SatId { constellation: Constellation::Gps, prn: 7 },
                band: SignalBand::L1,
                code: bijux_gnss_core::api::SignalCode::Ca,
            },
            signal: "L1".to_string(),
        },
        ambiguity_base: bijux_gnss_core::api::AmbiguityId {
            sig: SigId {
                sat: SatId { constellation: Constellation::Gps, prn: 7 },
                band: SignalBand::L1,
                code: bijux_gnss_core::api::SignalCode::Ca,
            },
            signal: "L1".to_string(),
        },
    }
}

fn sample_dd_observation() -> DdObservation {
    DdObservation {
        sig: SigId {
            sat: SatId { constellation: Constellation::Gps, prn: 9 },
            band: SignalBand::L1,
            code: bijux_gnss_core::api::SignalCode::Ca,
        },
        ref_sig: SigId {
            sat: SatId { constellation: Constellation::Gps, prn: 3 },
            band: SignalBand::L1,
            code: bijux_gnss_core::api::SignalCode::Ca,
        },
        rover_signal_pseudorange_m: 20_000_250.0,
        rover_signal_timing: Some(ObsSignalTiming {
            signal_travel_time_s: Seconds(0.067_001),
            transmit_gps_time: GpsTime { week: 2200, tow_s: 345_600.123_000 },
        }),
        base_signal_pseudorange_m: 20_000_100.0,
        base_signal_timing: Some(ObsSignalTiming {
            signal_travel_time_s: Seconds(0.066_998),
            transmit_gps_time: GpsTime { week: 2200, tow_s: 345_600.123_003 },
        }),
        rover_ref_pseudorange_m: 20_100_180.0,
        rover_ref_signal_timing: Some(ObsSignalTiming {
            signal_travel_time_s: Seconds(0.067_101),
            transmit_gps_time: GpsTime { week: 2200, tow_s: 345_600.122_900 },
        }),
        base_ref_pseudorange_m: 20_100_050.0,
        base_ref_signal_timing: Some(ObsSignalTiming {
            signal_travel_time_s: Seconds(0.067_098),
            transmit_gps_time: GpsTime { week: 2200, tow_s: 345_600.122_903 },
        }),
        code_m: 20.0,
        phase_cycles: 0.25,
        doppler_hz: -2.0,
        code_variance_m2: 8.0,
        phase_variance_cycles2: 0.02,
        canceled: vec![],
    }
}

#[test]
fn sd_artifact_validation_accepts_finite_observables() {
    let diagnostics = sample_sd_observation().validate_payload();
    assert!(diagnostics.is_empty(), "{diagnostics:?}");
}

#[test]
fn sd_artifact_validation_rejects_invalid_receiver_observables() {
    let mut observation = sample_sd_observation();
    observation.rover_pseudorange_m = f64::NAN;
    observation.base_signal_timing = Some(ObsSignalTiming {
        signal_travel_time_s: Seconds(f64::INFINITY),
        transmit_gps_time: GpsTime { week: 2200, tow_s: 345_600.123_003 },
    });

    let diagnostics = observation.validate_payload();

    assert!(diagnostics.iter().any(|event| event.code == "RTK_SD_NUMERIC_INVALID"));
    assert!(diagnostics.iter().any(|event| event.code == "RTK_SD_TIMING_INVALID"));
}

#[test]
fn dd_artifact_validation_accepts_finite_observables() {
    let diagnostics = sample_dd_observation().validate_payload();
    assert!(diagnostics.is_empty(), "{diagnostics:?}");
}

#[test]
fn dd_artifact_validation_rejects_invalid_receiver_observables() {
    let mut observation = sample_dd_observation();
    observation.rover_signal_pseudorange_m = f64::NAN;
    observation.base_ref_signal_timing = Some(ObsSignalTiming {
        signal_travel_time_s: Seconds(f64::INFINITY),
        transmit_gps_time: GpsTime { week: 2200, tow_s: 345_600.122_903 },
    });
    observation.phase_variance_cycles2 = -1.0;

    let diagnostics = observation.validate_payload();

    assert!(diagnostics.iter().any(|event| event.code == "RTK_DD_NUMERIC_INVALID"));
    assert!(diagnostics.iter().any(|event| event.code == "RTK_DD_TIMING_INVALID"));
    assert!(diagnostics.iter().any(|event| event.code == "RTK_DD_VARIANCE_INVALID"));
}
