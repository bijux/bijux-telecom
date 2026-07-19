use super::{
    detect_replay_timing_anomaly, PositionObservation, SatelliteState, SPEED_OF_LIGHT_MPS,
};
use crate::orbits::satellite_uncertainty::SatelliteStateUncertainty;
use bijux_gnss_core::api::{Constellation, GpsTime, ObsSignalTiming, SatId, Seconds};

#[test]
fn detects_centered_excess_delay_pattern() {
    let filtered = filtered_timing_residuals(&[
        ("G03", 105.0, true),
        ("G07", 48.0, true),
        ("G11", -44.0, true),
        ("G19", -109.0, true),
        ("G23", 92.0, true),
        ("G29", -95.0, true),
    ]);

    let anomaly =
        detect_replay_timing_anomaly(&filtered).expect("replay-like excess delays must surface");

    assert_eq!(anomaly.matched_satellite_count, 6);
    assert!(anomaly.centered_delay_rms_m >= anomaly.centered_delay_rms_threshold_m);
    assert!(anomaly.max_centered_delay_m >= anomaly.max_centered_delay_threshold_m);
    assert!(anomaly.median_excess_delay_m.abs() < 10.0);
}

#[test]
fn ignores_uniform_delay_shift() {
    let filtered = filtered_timing_residuals(&[
        ("G03", 85.0, true),
        ("G07", 84.0, true),
        ("G11", 86.0, true),
        ("G19", 83.0, true),
        ("G23", 87.0, true),
        ("G29", 84.5, true),
    ]);

    assert!(detect_replay_timing_anomaly(&filtered).is_none());
}

#[test]
fn requires_timing_tagged_satellites() {
    let filtered = filtered_timing_residuals(&[
        ("G03", 102.0, true),
        ("G07", 44.0, true),
        ("G11", -46.0, true),
        ("G19", -100.0, false),
        ("G23", 96.0, false),
        ("G29", -96.0, false),
    ]);

    assert!(detect_replay_timing_anomaly(&filtered).is_none());
}

fn filtered_timing_residuals(
    entries: &[(&str, f64, bool)],
) -> Vec<(PositionObservation, SatelliteState, f64, f64)> {
    entries
        .iter()
        .enumerate()
        .map(|(index, (satellite, residual_m, has_timing))| {
            let sat = parse_satellite_id(satellite);
            let observation = PositionObservation {
                sat,
                pseudorange_m: 24_000_000.0 + index as f64,
                doppler_hz: None,
                doppler_var_hz2: None,
                cn0_dbhz: 45.0,
                elevation_deg: Some(45.0),
                weight: 1.0,
                gps_receive_time: None,
                signal_timing: has_timing.then_some(ObsSignalTiming {
                    signal_travel_time_s: Seconds(24_000_000.0 / SPEED_OF_LIGHT_MPS),
                    transmit_gps_time: GpsTime { week: 0, tow_s: 100_000.0 },
                }),
                signal_id: None,
            };
            let state = SatelliteState {
                x_m: 20_000_000.0 + index as f64,
                y_m: 21_000_000.0 + index as f64,
                z_m: 22_000_000.0 + index as f64,
                vx_mps: 0.0,
                vy_mps: 0.0,
                vz_mps: 0.0,
                clock_bias_s: 0.0,
                clock_drift_s_per_s: 0.0,
                uncertainty: SatelliteStateUncertainty::unavailable(),
            };
            (observation, state, *residual_m, 1.0)
        })
        .collect()
}

fn parse_satellite_id(spec: &str) -> SatId {
    let prn = spec[1..].parse::<u8>().expect("two-digit PRN");
    let constellation = match &spec[..1] {
        "G" => Constellation::Gps,
        "E" => Constellation::Galileo,
        "C" => Constellation::Beidou,
        "R" => Constellation::Glonass,
        other => panic!("unexpected constellation designator: {other}"),
    };
    SatId { constellation, prn }
}
