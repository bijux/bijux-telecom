#![allow(missing_docs)]

use bijux_gnss_core::api::{
    AmbiguityId, ArtifactPayloadValidate, Constellation, GlonassFrequencyChannel, ReceiverRole,
    SatId, SigId, SignalBand, SignalCode,
};
use bijux_gnss_nav::api::{
    rtk_float_baseline_from_double_differences,
    rtk_float_baseline_from_double_differences_with_rover_prior,
    rtk_float_baseline_from_double_differences_with_satellite_states,
    rtk_float_baseline_reference_signals_by_constellation, rtk_switch_double_difference_reference,
    rtk_transform_float_baseline_reference, solve_baseline_dd_with_satellite_states,
    solve_float_baseline_dd_with_satellite_states, RtkConstellationTimeScale,
    RtkDoubleDifferenceCovarianceEvidence, RtkDoubleDifferenceObservation,
    RtkDoubleDifferenceSatelliteStates, RtkEpochAlignmentEvidence, RtkFloatAmbiguityEstimate,
    RtkFloatBaselineSolution, RtkGlonassInterFrequencyBiasEvidence,
    RtkGlonassInterFrequencyBiasStatus, RtkSatelliteStateEvidence,
    RtkSingleDifferenceCovarianceEvidence,
};
use bijux_gnss_signal::api::{glonass_l1_carrier_hz, signal_id_wavelength_m};
use bijux_gnss_testkit::reference_data::rtk_baseline::clean_gps_l1_short_baseline_case;

fn enu_to_ecef(base_ecef_m: [f64; 3], enu_m: [f64; 3]) -> [f64; 3] {
    let (lat_deg, lon_deg, _alt_m) =
        bijux_gnss_nav::api::ecef_to_geodetic(base_ecef_m[0], base_ecef_m[1], base_ecef_m[2]);
    let (sin_lat, cos_lat) = lat_deg.to_radians().sin_cos();
    let (sin_lon, cos_lon) = lon_deg.to_radians().sin_cos();
    let east_m = enu_m[0];
    let north_m = enu_m[1];
    let up_m = enu_m[2];
    let dx = -sin_lon * east_m - sin_lat * cos_lon * north_m + cos_lat * cos_lon * up_m;
    let dy = cos_lon * east_m - sin_lat * sin_lon * north_m + cos_lat * sin_lon * up_m;
    let dz = cos_lat * north_m + sin_lat * up_m;
    [base_ecef_m[0] + dx, base_ecef_m[1] + dy, base_ecef_m[2] + dz]
}

fn geometric_range_m(receiver_ecef_m: [f64; 3], satellite_ecef_m: [f64; 3]) -> f64 {
    let dx = receiver_ecef_m[0] - satellite_ecef_m[0];
    let dy = receiver_ecef_m[1] - satellite_ecef_m[1];
    let dz = receiver_ecef_m[2] - satellite_ecef_m[2];
    (dx * dx + dy * dy + dz * dz).sqrt()
}

fn retarget_double_differences_signal(
    observations: &[RtkDoubleDifferenceObservation],
    band: SignalBand,
    code: SignalCode,
) -> Vec<RtkDoubleDifferenceObservation> {
    observations
        .iter()
        .cloned()
        .map(|mut observation| {
            let source_wavelength_m =
                signal_id_wavelength_m(observation.sig).expect("fixture signal wavelength").0;
            observation.sig.band = band;
            observation.sig.code = code;
            observation.ref_sig.band = band;
            observation.ref_sig.code = code;
            let target_wavelength_m =
                signal_id_wavelength_m(observation.sig).expect("target signal wavelength").0;
            let phase_scale = source_wavelength_m / target_wavelength_m;
            observation.phase_cycles *= phase_scale;
            observation.phase_variance_cycles2 *= phase_scale * phase_scale;
            scale_phase_covariance(&mut observation.covariance_evidence.signal, phase_scale);
            scale_phase_covariance(&mut observation.covariance_evidence.reference, phase_scale);
            observation
        })
        .collect()
}

fn scale_phase_covariance(evidence: &mut RtkSingleDifferenceCovarianceEvidence, phase_scale: f64) {
    let variance_scale = phase_scale * phase_scale;
    evidence.rover.phase_cycles2 *= variance_scale;
    evidence.base.phase_cycles2 *= variance_scale;
    evidence.rover_base_phase_covariance_cycles2 *= variance_scale;
    evidence.shared_phase_covariance_cycles2 *= variance_scale;
}

fn sig(constellation: Constellation, prn: u8, band: SignalBand, code: SignalCode) -> SigId {
    SigId { sat: SatId { constellation, prn }, band, code }
}

fn satellite_state(
    sig: SigId,
    receiver_role: ReceiverRole,
    ecef_m: [f64; 3],
) -> RtkSatelliteStateEvidence {
    RtkSatelliteStateEvidence {
        sig,
        receiver_role,
        time_scale: RtkConstellationTimeScale::for_constellation(sig.sat.constellation),
        receive_time_s: 345_600.0,
        transmit_time_s: Some(345_599.93),
        ecef_m,
    }
}

fn wavelength_for_observation(observation: &RtkDoubleDifferenceObservation) -> f64 {
    if observation.sig.sat.constellation == Constellation::Glonass {
        let channel =
            observation.glonass_inter_frequency_bias.signal_channel.expect("GLONASS channel");
        299_792_458.0 / glonass_l1_carrier_hz(channel).value()
    } else {
        signal_id_wavelength_m(observation.sig).expect("signal wavelength").0
    }
}

struct SyntheticDoubleDifferenceRequest {
    sig: SigId,
    ref_sig: SigId,
    signal_position_m: [f64; 3],
    reference_position_m: [f64; 3],
    base_ecef_m: [f64; 3],
    rover_ecef_m: [f64; 3],
    ambiguity_cycles: f64,
    glonass_channel: Option<GlonassFrequencyChannel>,
}

fn synthetic_double_difference(
    request: SyntheticDoubleDifferenceRequest,
) -> (RtkDoubleDifferenceObservation, RtkDoubleDifferenceSatelliteStates) {
    let SyntheticDoubleDifferenceRequest {
        sig,
        ref_sig,
        signal_position_m,
        reference_position_m,
        base_ecef_m,
        rover_ecef_m,
        ambiguity_cycles,
        glonass_channel,
    } = request;
    let signal_rover_range_m = geometric_range_m(rover_ecef_m, signal_position_m);
    let signal_base_range_m = geometric_range_m(base_ecef_m, signal_position_m);
    let reference_rover_range_m = geometric_range_m(rover_ecef_m, reference_position_m);
    let reference_base_range_m = geometric_range_m(base_ecef_m, reference_position_m);
    let code_m = (signal_rover_range_m - signal_base_range_m)
        - (reference_rover_range_m - reference_base_range_m);
    let glonass_inter_frequency_bias = glonass_channel
        .map(|channel| RtkGlonassInterFrequencyBiasEvidence {
            status: RtkGlonassInterFrequencyBiasStatus::BiasHandled,
            signal_channel: Some(channel),
            reference_channel: Some(channel),
            code_bias_m: 0.0,
            phase_bias_cycles: 0.0,
        })
        .unwrap_or_default();
    let mut observation = RtkDoubleDifferenceObservation {
        sig,
        ref_sig,
        min_cn0_dbhz: 45.0,
        multipath_suspect: false,
        rover_signal_pseudorange_m: signal_rover_range_m,
        rover_signal_timing: None,
        base_signal_pseudorange_m: signal_base_range_m,
        base_signal_timing: None,
        rover_ref_pseudorange_m: reference_rover_range_m,
        rover_ref_signal_timing: None,
        base_ref_pseudorange_m: reference_base_range_m,
        base_ref_signal_timing: None,
        epoch_alignment: RtkEpochAlignmentEvidence {
            base_receive_time_s: 345_600.0,
            rover_receive_time_s: 345_600.0,
            delta_s: 0.0,
            tolerance_s: 0.0005,
        },
        covariance_evidence: RtkDoubleDifferenceCovarianceEvidence::default(),
        glonass_inter_frequency_bias,
        code_m,
        phase_cycles: 0.0,
        doppler_hz: 0.0,
        code_variance_m2: 4.0,
        phase_variance_cycles2: 0.01,
        canceled: vec![
            AmbiguityId { sig, signal: "carrier".to_string() },
            AmbiguityId { sig, signal: "carrier".to_string() },
            AmbiguityId { sig: ref_sig, signal: "carrier".to_string() },
            AmbiguityId { sig: ref_sig, signal: "carrier".to_string() },
        ],
    };
    let wavelength_m = wavelength_for_observation(&observation);
    observation.phase_cycles = code_m / wavelength_m + ambiguity_cycles;
    let states = RtkDoubleDifferenceSatelliteStates {
        sig,
        ref_sig,
        rover_signal: satellite_state(sig, ReceiverRole::Rover, signal_position_m),
        base_signal: satellite_state(sig, ReceiverRole::Base, signal_position_m),
        rover_reference: satellite_state(ref_sig, ReceiverRole::Rover, reference_position_m),
        base_reference: satellite_state(ref_sig, ReceiverRole::Base, reference_position_m),
    };
    (observation, states)
}

fn assert_solution_matches_truth(solution: &RtkFloatBaselineSolution, truth_enu_m: [f64; 3]) {
    assert!((solution.enu_m[0] - truth_enu_m[0]).abs() < 0.05, "east mismatch");
    assert!((solution.enu_m[1] - truth_enu_m[1]).abs() < 0.05, "north mismatch");
    assert!((solution.enu_m[2] - truth_enu_m[2]).abs() < 0.10, "up mismatch");
}

fn assert_float_solutions_close(
    actual: &RtkFloatBaselineSolution,
    expected: &RtkFloatBaselineSolution,
    tolerance: f64,
) {
    for axis in 0..3 {
        assert!((actual.enu_m[axis] - expected.enu_m[axis]).abs() < tolerance);
    }
    assert_eq!(actual.float_ambiguities.len(), expected.float_ambiguities.len());
    for (actual_ambiguity, expected_ambiguity) in
        actual.float_ambiguities.iter().zip(expected.float_ambiguities.iter())
    {
        assert_eq!(actual_ambiguity.sig, expected_ambiguity.sig);
        assert_eq!(actual_ambiguity.ref_sig, expected_ambiguity.ref_sig);
        assert!(
            (actual_ambiguity.float_cycles - expected_ambiguity.float_cycles).abs() < tolerance,
            "actual={} expected={}",
            actual_ambiguity.float_cycles,
            expected_ambiguity.float_cycles
        );
    }
}

#[test]
fn rtk_float_baseline_solver_recovers_truth_and_ambiguities() {
    let scenario = clean_gps_l1_short_baseline_case();

    let solution = rtk_float_baseline_from_double_differences(
        &scenario.double_differences,
        scenario.base_ecef_m,
        &scenario.ephemerides,
        scenario.receive_gps_time.tow_s,
    )
    .expect("float baseline");

    assert!((solution.enu_m[0] - scenario.truth_enu_m[0]).abs() < 0.05, "east mismatch");
    assert!((solution.enu_m[1] - scenario.truth_enu_m[1]).abs() < 0.05, "north mismatch");
    assert!((solution.enu_m[2] - scenario.truth_enu_m[2]).abs() < 0.10, "up mismatch");
    assert_eq!(solution.float_ambiguities.len(), scenario.double_differences.len());
    assert_eq!(solution.ambiguity_covariance_cycles2.len(), scenario.double_differences.len());
    assert_eq!(solution.enu_ambiguity_covariance_m_cycles.len(), 3);
    assert!(solution.covariance_enu_m2[0][0].is_finite());
    assert!(solution.covariance_enu_m2[1][1].is_finite());
    assert!(solution.covariance_enu_m2[2][2].is_finite());
    assert!(solution.covariance_enu_m2[0][0] > 0.0);
    assert!(solution.covariance_enu_m2[1][1] > 0.0);
    assert!(solution.covariance_enu_m2[2][2] > 0.0);
    for row in &solution.ambiguity_covariance_cycles2 {
        assert_eq!(row.len(), scenario.double_differences.len());
        assert!(row.iter().all(|value| value.is_finite()));
    }
    for row in &solution.enu_ambiguity_covariance_m_cycles {
        assert_eq!(row.len(), scenario.double_differences.len());
        assert!(row.iter().all(|value| value.is_finite()));
    }

    for ambiguity in &solution.float_ambiguities {
        let expected_cycles = scenario.rover_ambiguities_cycles[&ambiguity.sig.sat]
            - scenario.rover_ambiguities_cycles[&scenario.reference_sig.sat];
        assert!((ambiguity.float_cycles - expected_cycles).abs() < 0.05);
        assert!(ambiguity.variance_cycles2 >= 0.0);
    }
}

#[test]
fn rtk_float_baseline_solver_accepts_explicit_satellite_state_evidence_for_gps() {
    let scenario = clean_gps_l1_short_baseline_case();
    let satellite_states = scenario
        .double_differences
        .iter()
        .map(|observation| {
            let signal_ephemeris = scenario
                .ephemerides
                .iter()
                .find(|candidate| candidate.sat == observation.sig.sat)
                .expect("signal ephemeris");
            let reference_ephemeris = scenario
                .ephemerides
                .iter()
                .find(|candidate| candidate.sat == observation.ref_sig.sat)
                .expect("reference ephemeris");
            let signal_state = bijux_gnss_nav::api::sat_state_gps_l1ca_at_receive_time(
                signal_ephemeris,
                scenario.receive_gps_time.tow_s,
                0.07,
            );
            let reference_state = bijux_gnss_nav::api::sat_state_gps_l1ca_at_receive_time(
                reference_ephemeris,
                scenario.receive_gps_time.tow_s,
                0.07,
            );
            let signal_ecef_m = [signal_state.x_m, signal_state.y_m, signal_state.z_m];
            let reference_ecef_m = [reference_state.x_m, reference_state.y_m, reference_state.z_m];
            RtkDoubleDifferenceSatelliteStates {
                sig: observation.sig,
                ref_sig: observation.ref_sig,
                rover_signal: satellite_state(observation.sig, ReceiverRole::Rover, signal_ecef_m),
                base_signal: satellite_state(observation.sig, ReceiverRole::Base, signal_ecef_m),
                rover_reference: satellite_state(
                    observation.ref_sig,
                    ReceiverRole::Rover,
                    reference_ecef_m,
                ),
                base_reference: satellite_state(
                    observation.ref_sig,
                    ReceiverRole::Base,
                    reference_ecef_m,
                ),
            }
        })
        .collect::<Vec<_>>();

    let solution = rtk_float_baseline_from_double_differences_with_satellite_states(
        &scenario.double_differences,
        scenario.base_ecef_m,
        &satellite_states,
    )
    .expect("float baseline from explicit states");

    assert_solution_matches_truth(&solution, scenario.truth_enu_m);
}

#[test]
fn rtk_float_baseline_solver_combines_constellation_aware_double_differences() {
    let base_ecef_m = bijux_gnss_nav::api::geodetic_to_ecef(37.0, -122.0, 10.0);
    let base_ecef_m = [base_ecef_m.0, base_ecef_m.1, base_ecef_m.2];
    let truth_enu_m = [4.0, -2.0, 1.0];
    let rover_ecef_m = enu_to_ecef(base_ecef_m, truth_enu_m);
    let glonass_channel = GlonassFrequencyChannel::new(-4).expect("valid GLONASS channel");
    let cases = [
        (
            sig(Constellation::Gps, 7, SignalBand::L1, SignalCode::Ca),
            sig(Constellation::Gps, 3, SignalBand::L1, SignalCode::Ca),
            [15_600_000.0, -12_300_000.0, 21_700_000.0],
            [21_200_000.0, 4_100_000.0, 13_400_000.0],
            12.0,
            None,
        ),
        (
            sig(Constellation::Galileo, 19, SignalBand::E1, SignalCode::E1B),
            sig(Constellation::Galileo, 11, SignalBand::E1, SignalCode::E1B),
            [-14_500_000.0, 19_600_000.0, 18_100_000.0],
            [9_800_000.0, 21_400_000.0, 15_900_000.0],
            -7.0,
            None,
        ),
        (
            sig(Constellation::Beidou, 12, SignalBand::B1, SignalCode::B1I),
            sig(Constellation::Beidou, 11, SignalBand::B1, SignalCode::B1I),
            [23_100_000.0, -7_200_000.0, 14_900_000.0],
            [-8_700_000.0, -22_500_000.0, 13_700_000.0],
            5.0,
            None,
        ),
        (
            sig(Constellation::Glonass, 8, SignalBand::L1, SignalCode::Unknown),
            sig(Constellation::Glonass, 4, SignalBand::L1, SignalCode::Unknown),
            [-19_500_000.0, -10_400_000.0, 18_800_000.0],
            [4_400_000.0, -21_100_000.0, 16_600_000.0],
            -3.0,
            Some(glonass_channel),
        ),
    ];
    let mut observations = Vec::new();
    let mut satellite_states = Vec::new();
    for (sig, ref_sig, signal_position_m, reference_position_m, ambiguity_cycles, channel) in cases
    {
        let (observation, states) = synthetic_double_difference(SyntheticDoubleDifferenceRequest {
            sig,
            ref_sig,
            signal_position_m,
            reference_position_m,
            base_ecef_m,
            rover_ecef_m,
            ambiguity_cycles,
            glonass_channel: channel,
        });
        observations.push(observation);
        satellite_states.push(states);
    }

    let solution = rtk_float_baseline_from_double_differences_with_satellite_states(
        &observations,
        base_ecef_m,
        &satellite_states,
    )
    .expect("multi-constellation float baseline");

    assert_solution_matches_truth(&solution, truth_enu_m);
    assert_eq!(solution.float_ambiguities.len(), observations.len());
    let references = rtk_float_baseline_reference_signals_by_constellation(&solution)
        .expect("constellation references");
    assert_eq!(references.len(), 4);
    assert_eq!(
        references[&Constellation::Glonass],
        sig(Constellation::Glonass, 4, SignalBand::L1, SignalCode::Unknown)
    );
    assert!(solution
        .float_ambiguities
        .iter()
        .any(|ambiguity| { ambiguity.sig.sat.constellation == Constellation::Gps }));
    assert!(solution
        .float_ambiguities
        .iter()
        .any(|ambiguity| { ambiguity.sig.sat.constellation == Constellation::Galileo }));
    assert!(solution
        .float_ambiguities
        .iter()
        .any(|ambiguity| { ambiguity.sig.sat.constellation == Constellation::Beidou }));
    assert!(solution
        .float_ambiguities
        .iter()
        .any(|ambiguity| { ambiguity.sig.sat.constellation == Constellation::Glonass }));
}

#[test]
fn rtk_execution_wrappers_project_constellation_state_baseline() {
    let base_ecef_m = bijux_gnss_nav::api::geodetic_to_ecef(37.0, -122.0, 10.0);
    let base_ecef_m = [base_ecef_m.0, base_ecef_m.1, base_ecef_m.2];
    let truth_enu_m = [4.0, -2.0, 1.0];
    let rover_ecef_m = enu_to_ecef(base_ecef_m, truth_enu_m);
    let glonass_channel = GlonassFrequencyChannel::new(-4).expect("valid GLONASS channel");
    let cases = [
        (
            sig(Constellation::Gps, 7, SignalBand::L1, SignalCode::Ca),
            sig(Constellation::Gps, 3, SignalBand::L1, SignalCode::Ca),
            [15_600_000.0, -12_300_000.0, 21_700_000.0],
            [21_200_000.0, 4_100_000.0, 13_400_000.0],
            12.0,
            None,
        ),
        (
            sig(Constellation::Galileo, 19, SignalBand::E1, SignalCode::E1B),
            sig(Constellation::Galileo, 11, SignalBand::E1, SignalCode::E1B),
            [-14_500_000.0, 19_600_000.0, 18_100_000.0],
            [9_800_000.0, 21_400_000.0, 15_900_000.0],
            -7.0,
            None,
        ),
        (
            sig(Constellation::Beidou, 12, SignalBand::B1, SignalCode::B1I),
            sig(Constellation::Beidou, 11, SignalBand::B1, SignalCode::B1I),
            [23_100_000.0, -7_200_000.0, 14_900_000.0],
            [-8_700_000.0, -22_500_000.0, 13_700_000.0],
            5.0,
            None,
        ),
        (
            sig(Constellation::Glonass, 8, SignalBand::L1, SignalCode::Unknown),
            sig(Constellation::Glonass, 4, SignalBand::L1, SignalCode::Unknown),
            [-19_500_000.0, -10_400_000.0, 18_800_000.0],
            [4_400_000.0, -21_100_000.0, 16_600_000.0],
            -3.0,
            Some(glonass_channel),
        ),
    ];
    let mut observations = Vec::new();
    let mut satellite_states = Vec::new();
    for (sig, ref_sig, signal_position_m, reference_position_m, ambiguity_cycles, channel) in cases
    {
        let (observation, states) = synthetic_double_difference(SyntheticDoubleDifferenceRequest {
            sig,
            ref_sig,
            signal_position_m,
            reference_position_m,
            base_ecef_m,
            rover_ecef_m,
            ambiguity_cycles,
            glonass_channel: channel,
        });
        observations.push(observation);
        satellite_states.push(states);
    }

    let float_solution = solve_float_baseline_dd_with_satellite_states(
        &observations,
        base_ecef_m,
        &satellite_states,
    )
    .expect("execution float baseline");
    let projected_solution =
        solve_baseline_dd_with_satellite_states(&observations, base_ecef_m, &satellite_states)
            .expect("projected baseline");

    assert_solution_matches_truth(&float_solution, truth_enu_m);
    assert_eq!(projected_solution.enu_m, float_solution.enu_m);
    assert_eq!(
        projected_solution.covariance_m2.expect("projected covariance"),
        float_solution.covariance_enu_m2
    );
    assert!(!projected_solution.fixed);
}

#[test]
fn rtk_float_baseline_solver_refuses_mismatched_constellation_time_scale() {
    let base_ecef_m = [6_378_137.0, 0.0, 0.0];
    let rover_ecef_m = [6_378_141.0, -2.0, 1.0];
    let (observation, mut states) = synthetic_double_difference(SyntheticDoubleDifferenceRequest {
        sig: sig(Constellation::Galileo, 19, SignalBand::E1, SignalCode::E1B),
        ref_sig: sig(Constellation::Galileo, 11, SignalBand::E1, SignalCode::E1B),
        signal_position_m: [-14_500_000.0, 19_600_000.0, 18_100_000.0],
        reference_position_m: [9_800_000.0, 21_400_000.0, 15_900_000.0],
        base_ecef_m,
        rover_ecef_m,
        ambiguity_cycles: -7.0,
        glonass_channel: None,
    });
    states.rover_signal.time_scale = RtkConstellationTimeScale::Gps;
    let mut observations = vec![observation.clone(), observation.clone(), observation];
    observations[1].sig.sat.prn = 20;
    observations[2].sig.sat.prn = 21;
    let satellite_states = vec![states; 3];

    let solution = rtk_float_baseline_from_double_differences_with_satellite_states(
        &observations,
        base_ecef_m,
        &satellite_states,
    );

    assert!(solution.is_none());
}

#[test]
fn rtk_float_baseline_solver_refuses_uncalibrated_glonass_fdma_phase() {
    let base_ecef_m = bijux_gnss_nav::api::geodetic_to_ecef(37.0, -122.0, 10.0);
    let base_ecef_m = [base_ecef_m.0, base_ecef_m.1, base_ecef_m.2];
    let truth_enu_m = [4.0, -2.0, 1.0];
    let rover_ecef_m = enu_to_ecef(base_ecef_m, truth_enu_m);
    let shared_channel = GlonassFrequencyChannel::new(-4).expect("valid GLONASS channel");
    let other_channel = GlonassFrequencyChannel::new(3).expect("valid GLONASS channel");
    let (mut observation, states) = synthetic_double_difference(SyntheticDoubleDifferenceRequest {
        sig: sig(Constellation::Glonass, 8, SignalBand::L1, SignalCode::Unknown),
        ref_sig: sig(Constellation::Glonass, 4, SignalBand::L1, SignalCode::Unknown),
        signal_position_m: [-19_500_000.0, -10_400_000.0, 18_800_000.0],
        reference_position_m: [4_400_000.0, -21_100_000.0, 16_600_000.0],
        base_ecef_m,
        rover_ecef_m,
        ambiguity_cycles: -3.0,
        glonass_channel: Some(shared_channel),
    });
    observation.glonass_inter_frequency_bias = RtkGlonassInterFrequencyBiasEvidence {
        status: RtkGlonassInterFrequencyBiasStatus::CalibrationRequired,
        signal_channel: Some(shared_channel),
        reference_channel: Some(other_channel),
        code_bias_m: 0.0,
        phase_bias_cycles: 0.0,
    };
    let observations = vec![observation.clone(), observation.clone(), observation];
    let satellite_states = vec![states; 3];

    let solution = rtk_float_baseline_from_double_differences_with_satellite_states(
        &observations,
        base_ecef_m,
        &satellite_states,
    );

    assert!(solution.is_none());
}

#[test]
fn rtk_float_baseline_reference_switch_avoids_phase_jump() {
    let scenario = clean_gps_l1_short_baseline_case();
    let original_solution = rtk_float_baseline_from_double_differences(
        &scenario.double_differences,
        scenario.base_ecef_m,
        &scenario.ephemerides,
        scenario.receive_gps_time.tow_s,
    )
    .expect("original float baseline");
    let new_reference = scenario.double_differences[0].sig;
    let transformed_solution =
        rtk_transform_float_baseline_reference(&original_solution, new_reference)
            .expect("transformed solution");
    let switched_double_differences =
        rtk_switch_double_difference_reference(&scenario.double_differences, new_reference)
            .expect("switched double differences");
    let switched_solution = rtk_float_baseline_from_double_differences(
        &switched_double_differences,
        scenario.base_ecef_m,
        &scenario.ephemerides,
        scenario.receive_gps_time.tow_s,
    )
    .expect("switched float baseline");

    assert_float_solutions_close(&switched_solution, &transformed_solution, 1.0e-6);
    assert_solution_matches_truth(&switched_solution, scenario.truth_enu_m);
}

#[test]
fn rtk_float_baseline_solver_uses_differenced_covariance_coupling() {
    let scenario = clean_gps_l1_short_baseline_case();
    let independent_solution = rtk_float_baseline_from_double_differences(
        &scenario.double_differences,
        scenario.base_ecef_m,
        &scenario.ephemerides,
        scenario.receive_gps_time.tow_s,
    )
    .expect("independent covariance solution");

    let mut correlated_observations = scenario.double_differences.clone();
    for observation in &mut correlated_observations {
        observation.covariance_evidence.signal.shared_phase_covariance_cycles2 += 2.0e-5;
        observation.covariance_evidence.reference.shared_phase_covariance_cycles2 += 2.0e-5;
    }
    let correlated_solution = rtk_float_baseline_from_double_differences(
        &correlated_observations,
        scenario.base_ecef_m,
        &scenario.ephemerides,
        scenario.receive_gps_time.tow_s,
    )
    .expect("correlated covariance solution");

    let independent_ambiguity_trace: f64 = independent_solution
        .ambiguity_covariance_cycles2
        .iter()
        .enumerate()
        .map(|(index, row)| row[index])
        .sum();
    let correlated_ambiguity_trace: f64 = correlated_solution
        .ambiguity_covariance_cycles2
        .iter()
        .enumerate()
        .map(|(index, row)| row[index])
        .sum();
    assert!((independent_ambiguity_trace - correlated_ambiguity_trace).abs() > 1.0e-8);
    assert_solution_matches_truth(&correlated_solution, scenario.truth_enu_m);
}

#[test]
fn rtk_float_baseline_solver_accepts_explicit_rover_prior() {
    let scenario = clean_gps_l1_short_baseline_case();
    let rover_prior_ecef_m = enu_to_ecef(
        scenario.base_ecef_m,
        [
            scenario.truth_enu_m[0] + 1.5,
            scenario.truth_enu_m[1] - 1.0,
            scenario.truth_enu_m[2] + 0.5,
        ],
    );

    let solution = rtk_float_baseline_from_double_differences_with_rover_prior(
        &scenario.double_differences,
        scenario.base_ecef_m,
        rover_prior_ecef_m,
        &scenario.ephemerides,
        scenario.receive_gps_time.tow_s,
    )
    .expect("float baseline with prior");

    assert!((solution.enu_m[0] - scenario.truth_enu_m[0]).abs() < 0.05);
    assert!((solution.enu_m[1] - scenario.truth_enu_m[1]).abs() < 0.05);
    assert!((solution.enu_m[2] - scenario.truth_enu_m[2]).abs() < 0.10);
}

#[test]
fn rtk_float_baseline_solver_supports_gps_l2c_wavelengths() {
    let scenario = clean_gps_l1_short_baseline_case();
    let double_differences = retarget_double_differences_signal(
        &scenario.double_differences,
        SignalBand::L2,
        SignalCode::L2C,
    );

    let solution = rtk_float_baseline_from_double_differences(
        &double_differences,
        scenario.base_ecef_m,
        &scenario.ephemerides,
        scenario.receive_gps_time.tow_s,
    )
    .expect("float baseline");

    assert_solution_matches_truth(&solution, scenario.truth_enu_m);
}

#[test]
fn rtk_float_baseline_solver_supports_gps_l5_wavelengths() {
    let scenario = clean_gps_l1_short_baseline_case();
    let double_differences = retarget_double_differences_signal(
        &scenario.double_differences,
        SignalBand::L5,
        SignalCode::L5I,
    );

    let solution = rtk_float_baseline_from_double_differences(
        &double_differences,
        scenario.base_ecef_m,
        &scenario.ephemerides,
        scenario.receive_gps_time.tow_s,
    )
    .expect("float baseline");

    assert_solution_matches_truth(&solution, scenario.truth_enu_m);
}

#[test]
fn rtk_float_baseline_artifact_validation_rejects_invalid_values() {
    let ambiguity = RtkFloatAmbiguityEstimate {
        sig: SigId {
            sat: SatId { constellation: Constellation::Gps, prn: 7 },
            band: SignalBand::L1,
            code: bijux_gnss_core::api::SignalCode::Ca,
        },
        ref_sig: SigId {
            sat: SatId { constellation: Constellation::Galileo, prn: 3 },
            band: SignalBand::L1,
            code: bijux_gnss_core::api::SignalCode::Ca,
        },
        float_cycles: f64::NAN,
        variance_cycles2: -1.0,
    };
    let solution = RtkFloatBaselineSolution {
        enu_m: [0.0, f64::INFINITY, 0.0],
        covariance_enu_m2: [[0.0, 0.0, 0.0], [0.0, f64::NAN, 0.0], [0.0, 0.0, 0.0]],
        enu_ambiguity_covariance_m_cycles: vec![vec![f64::INFINITY], vec![0.0], Vec::new()],
        float_ambiguities: vec![ambiguity],
        ambiguity_covariance_cycles2: vec![vec![f64::NAN]],
    };

    let diagnostics = solution.validate_payload();

    assert!(diagnostics.iter().any(|event| event.code == "RTK_FLOAT_BASELINE_NUMERIC_INVALID"));
    assert!(diagnostics.iter().any(|event| event.code == "RTK_FLOAT_BASELINE_COVARIANCE_INVALID"));
    assert!(diagnostics.iter().any(|event| {
        event.code == "RTK_FLOAT_BASELINE_CROSS_COVARIANCE_SHAPE_INVALID"
            || event.code == "RTK_FLOAT_BASELINE_CROSS_COVARIANCE_NUMERIC_INVALID"
    }));
    assert!(diagnostics.iter().any(|event| event.code == "RTK_FLOAT_AMBIGUITY_NUMERIC_INVALID"));
    assert!(diagnostics.iter().any(|event| event.code == "RTK_FLOAT_AMBIGUITY_VARIANCE_INVALID"));
    assert!(diagnostics
        .iter()
        .any(|event| event.code == "RTK_FLOAT_AMBIGUITY_CONSTELLATION_MISMATCH"));
    assert!(diagnostics.iter().any(|event| {
        event.code == "RTK_FLOAT_AMBIGUITY_COVARIANCE_NUMERIC_INVALID"
            || event.code == "RTK_FLOAT_AMBIGUITY_COVARIANCE_SHAPE_INVALID"
    }));
    assert!(rtk_float_baseline_reference_signals_by_constellation(&solution).is_none());
}

#[test]
fn rtk_float_baseline_solver_refuses_underdetermined_inputs() {
    let scenario = clean_gps_l1_short_baseline_case();

    let solution = rtk_float_baseline_from_double_differences(
        &scenario.double_differences[..2],
        scenario.base_ecef_m,
        &scenario.ephemerides,
        scenario.receive_gps_time.tow_s,
    );

    assert!(solution.is_none());
}

#[test]
fn rtk_float_baseline_solver_refuses_unregistered_signal_identities() {
    let scenario = clean_gps_l1_short_baseline_case();
    let mut double_differences = scenario.double_differences.clone();
    for observation in &mut double_differences {
        observation.sig.band = SignalBand::L2;
        observation.sig.code = SignalCode::Unknown;
        observation.ref_sig.band = SignalBand::L2;
        observation.ref_sig.code = SignalCode::Unknown;
    }

    let solution = rtk_float_baseline_from_double_differences(
        &double_differences,
        scenario.base_ecef_m,
        &scenario.ephemerides,
        scenario.receive_gps_time.tow_s,
    );

    assert!(solution.is_none());
}
