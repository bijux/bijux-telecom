#![allow(missing_docs)]

use bijux_gnss_core::api::{
    ArtifactPayloadValidate, Constellation, GpsTime, ObsSignalTiming, SatId, Seconds, SigId,
    SignalBand,
};
use bijux_gnss_receiver::api::SdObservation;

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
