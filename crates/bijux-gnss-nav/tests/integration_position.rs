#![allow(missing_docs)]
mod support;

use bijux_gnss_core::api::{
    Constellation, GpsTime, Llh, SatId, Seconds, SigId, SignalBand, SignalCode,
};
use bijux_gnss_nav::api::{
    beidou_broadcast_group_delay_code_bias_m, ecef_to_geodetic, elevation_azimuth_deg,
    ephemerides_from_decoded_gps_l1ca_lnav, galileo_broadcast_group_delay_code_bias_m,
    geodetic_to_ecef, gps_broadcast_group_delay_code_bias_m, parse_rinex_nav,
    position_broadcast_navigation_from_beidou_navigations,
    position_broadcast_navigation_from_glonass_frames,
    position_broadcast_navigation_from_gps_ephemerides, sat_state_beidou_b1i, sat_state_galileo_e1,
    sat_state_glonass_l1, sat_state_gps_l1ca, write_rinex_nav, BeidouBroadcastNavigationData,
    BeidouClockCorrection, BeidouEphemeris, BeidouIonosphericCorrection, BeidouSignalHealth,
    BeidouSystemTime, Ephemeris, GalileoBroadcastNavigationData, GalileoClockCorrection,
    GalileoEphemeris, GalileoIonosphericCorrection, GalileoIonosphericDisturbanceFlags,
    GalileoSignalHealth, GalileoSystemTime, GlonassAlmanacTimeData,
    GlonassBroadcastNavigationFrame, GlonassFrameTime, GlonassImmediateHealth,
    GlonassImmediateNavigationData, GlonassSatelliteType, GlonassStateVector, GlonassSystemTime,
    GpsBroadcastNavigationData, GpsEphemeris, GpsL1CaHowWord, GpsL1CaLnavDecodedSubframe,
    GpsL1CaLnavSubframe1Clock, GpsL1CaLnavSubframe2Orbit, GpsL1CaLnavSubframe3Orbit,
    GpsL1CaLnavSubframeAlignment, GpsL1CaTlmWord, GpsL1CaWordParitySummary,
    PositionBroadcastNavigation, PositionObservation, PositionRobustWeighting, PositionSolver,
};
use support::position_truth::{
    add_klobuchar_delay_to_observations, add_saastamoinen_delay_to_observations,
    clear_broadcast_clock_parameters, four_satellite_position_scenario,
    four_satellite_position_scenario_with_ephemerides, iterative_pseudorange_residual_m,
    iterative_pseudorange_residual_without_earth_rotation_m, sample_ephemerides,
    sample_ephemerides_with_clock_parameters, sample_ephemeris, sample_klobuchar_coefficients,
    timed_position_observation, timed_position_observation_from_truth, BroadcastClockParameters,
    SyntheticPositionScenario,
};

fn broadcast_clock_fixture_parameters() -> [(u8, BroadcastClockParameters); 4] {
    [
        (
            1,
            BroadcastClockParameters {
                af0_s: 240.0e-9,
                af1_s_per_s: 0.0,
                af2_s_per_s2: 0.0,
                tgd_s: -8.0e-9,
            },
        ),
        (
            2,
            BroadcastClockParameters {
                af0_s: -170.0e-9,
                af1_s_per_s: 0.0,
                af2_s_per_s2: 0.0,
                tgd_s: 6.0e-9,
            },
        ),
        (
            3,
            BroadcastClockParameters {
                af0_s: 330.0e-9,
                af1_s_per_s: 0.0,
                af2_s_per_s2: 0.0,
                tgd_s: -4.0e-9,
            },
        ),
        (
            4,
            BroadcastClockParameters {
                af0_s: -95.0e-9,
                af1_s_per_s: 0.0,
                af2_s_per_s2: 0.0,
                tgd_s: 10.0e-9,
            },
        ),
    ]
}

fn broadcast_clock_position_scenario(receiver_clock_bias_s: f64) -> SyntheticPositionScenario {
    four_satellite_position_scenario_with_ephemerides(
        receiver_clock_bias_s,
        sample_ephemerides_with_clock_parameters(&broadcast_clock_fixture_parameters()),
    )
}

fn position_error_3d_m(
    ecef_x_m: f64,
    ecef_y_m: f64,
    ecef_z_m: f64,
    truth_ecef_m: (f64, f64, f64),
) -> f64 {
    let dx = ecef_x_m - truth_ecef_m.0;
    let dy = ecef_y_m - truth_ecef_m.1;
    let dz = ecef_z_m - truth_ecef_m.2;
    (dx * dx + dy * dy + dz * dz).sqrt()
}

fn max_abs_residual_m(residuals: &[(SatId, f64, f64)]) -> f64 {
    residuals.iter().map(|(_, residual_m, _)| residual_m.abs()).fold(0.0, f64::max)
}

fn observation_with_signal_id(
    mut observation: PositionObservation,
    signal_id: SigId,
) -> PositionObservation {
    observation.signal_id = Some(signal_id);
    observation
}

fn observation_with_pseudorange_bias(
    mut observation: PositionObservation,
    pseudorange_bias_m: f64,
) -> PositionObservation {
    observation.pseudorange_m += pseudorange_bias_m;
    if let Some(signal_timing) = &mut observation.signal_timing {
        let delay_s = pseudorange_bias_m / 299_792_458.0;
        signal_timing.signal_travel_time_s =
            Seconds(signal_timing.signal_travel_time_s.0 + delay_s);
        signal_timing.transmit_gps_time = signal_timing.transmit_gps_time.offset_seconds(-delay_s);
    }
    observation
}

fn sample_galileo_navigation(prn: u8, omega0: f64, m0: f64) -> GalileoBroadcastNavigationData {
    GalileoBroadcastNavigationData {
        sat: SatId { constellation: Constellation::Galileo, prn },
        iodnav: prn as u16,
        gst: GalileoSystemTime { week: 2222, tow_s: 504_018 },
        sisa_e1_e5b: 77,
        signal_health: GalileoSignalHealth {
            e5b_signal_health: 0,
            e1b_signal_health: 0,
            e5b_data_valid: true,
            e1b_data_valid: true,
        },
        clock: GalileoClockCorrection {
            t0c_s: 504_018.0,
            af0: 0.0,
            af1: 0.0,
            af2: 0.0,
            bgd_e1_e5a_s: 0.0,
            bgd_e1_e5b_s: 0.0,
        },
        ephemeris: GalileoEphemeris {
            sat: SatId { constellation: Constellation::Galileo, prn },
            iodnav: prn as u16,
            toe_s: 504_000.0,
            sqrt_a: 5_440.612_319,
            e: 0.001_23,
            i0: 0.953,
            idot: -2.1e-10,
            omega0,
            omegadot: -5.8e-9,
            w: -0.37,
            m0,
            delta_n: 4.7e-9,
            cuc: -3.2e-6,
            cus: 4.1e-6,
            crc: 178.0,
            crs: -91.0,
            cic: 1.9e-7,
            cis: -2.4e-7,
        },
        ionosphere: GalileoIonosphericCorrection {
            ai0: 82.0,
            ai1: 0.18,
            ai2: -0.01,
            disturbance_flags: GalileoIonosphericDisturbanceFlags {
                region_1: false,
                region_2: false,
                region_3: false,
                region_4: false,
                region_5: false,
            },
        },
    }
}

fn sample_gps_ephemerides_at(reference_time_s: f64) -> Vec<GpsEphemeris> {
    let mut ephemerides = vec![
        sample_ephemeris(1, 0.0, 0.0),
        sample_ephemeris(2, 0.8, 0.9),
        sample_ephemeris(3, 1.6, 1.8),
        sample_ephemeris(4, 2.4, 2.7),
    ];
    for ephemeris in &mut ephemerides {
        ephemeris.toe_s = reference_time_s - 18.0;
        ephemeris.toc_s = reference_time_s;
    }
    ephemerides
}

fn sample_beidou_navigation_at(
    prn: u8,
    omega0: f64,
    m0: f64,
    reference_time_s: f64,
) -> BeidouBroadcastNavigationData {
    BeidouBroadcastNavigationData {
        sat: SatId { constellation: Constellation::Beidou, prn },
        bdt: BeidouSystemTime { week: 888, sow_s: reference_time_s as u32 },
        urai: 0,
        signal_health: BeidouSignalHealth { autonomous_satellite_good: true },
        clock: BeidouClockCorrection {
            toc_s: reference_time_s,
            aodc: prn,
            af0: 0.0,
            af1: 0.0,
            af2: 0.0,
            tgd1_s: 0.0,
            tgd2_s: 0.0,
        },
        ephemeris: BeidouEphemeris {
            sat: SatId { constellation: Constellation::Beidou, prn },
            aode: prn,
            toe_s: reference_time_s - 18.0,
            sqrt_a: 5_282.61,
            e: 0.0021,
            i0: 0.962,
            idot: -1.4e-10,
            omega0,
            omegadot: -7.9e-9,
            w: -0.41,
            m0,
            delta_n: 3.4e-9,
            cuc: -1.2e-6,
            cus: 3.1e-6,
            crc: 146.0,
            crs: -84.0,
            cic: 1.1e-7,
            cis: -1.7e-7,
        },
        ionosphere: BeidouIonosphericCorrection {
            alpha0: 0.0,
            alpha1: 0.0,
            alpha2: 0.0,
            alpha3: 0.0,
            beta0: 0.0,
            beta1: 0.0,
            beta2: 0.0,
            beta3: 0.0,
        },
    }
}

fn sample_beidou_navigation(prn: u8, omega0: f64, m0: f64) -> BeidouBroadcastNavigationData {
    sample_beidou_navigation_at(prn, omega0, m0, 345_618.0)
}

struct MixedSecondaryBandCase {
    observations: Vec<PositionObservation>,
    navigation: Vec<PositionBroadcastNavigation>,
    truth_ecef_m: (f64, f64, f64),
    t_rx_s: f64,
    receiver_clock_bias_s: f64,
    galileo_bias_s: f64,
    beidou_bias_s: f64,
}

struct GalileoPositionScenario {
    observations: Vec<PositionObservation>,
    navigation: Vec<PositionBroadcastNavigation>,
    truth_ecef_m: (f64, f64, f64),
    t_rx_s: f64,
}

fn mixed_secondary_band_case() -> MixedSecondaryBandCase {
    let gps_ephemerides =
        sample_ephemerides_with_clock_parameters(&broadcast_clock_fixture_parameters());
    let galileo_navigation =
        vec![sample_galileo_navigation(19, 1.17, 0.84), sample_galileo_navigation(24, -0.83, 1.52)];
    let beidou_navigation = vec![
        sample_beidou_navigation_at(11, 0.77, 0.53, 504_018.0),
        sample_beidou_navigation_at(12, -1.09, 1.31, 504_018.0),
    ];
    let truth_ecef_m = geodetic_to_ecef(37.0, -122.0, 10.0);
    let receiver_clock_bias_s = 2.75e-4;
    let galileo_bias_s = -1.15e-6;
    let beidou_bias_s = 9.25e-7;
    let t_rx_s = 504_018.07 + receiver_clock_bias_s;

    let mut observations = gps_ephemerides
        .iter()
        .map(|ephemeris| {
            let signal =
                SigId { sat: ephemeris.sat, band: SignalBand::L5, code: SignalCode::Unknown };
            let observation = timed_position_observation_from_truth(
                ephemeris,
                truth_ecef_m,
                t_rx_s,
                receiver_clock_bias_s,
            );
            observation_with_signal_id(
                observation_with_pseudorange_bias(
                    observation,
                    gps_broadcast_group_delay_code_bias_m(signal, ephemeris)
                        .expect("GPS L5 broadcast group delay bias"),
                ),
                signal,
            )
        })
        .collect::<Vec<_>>();
    observations.extend(galileo_navigation.iter().map(|navigation| {
        let signal = SigId { sat: navigation.sat, band: SignalBand::E5, code: SignalCode::E5a };
        let pseudorange_m = galileo_pseudorange_from_truth(
            navigation,
            truth_ecef_m,
            t_rx_s,
            receiver_clock_bias_s,
            galileo_bias_s,
        ) + galileo_broadcast_group_delay_code_bias_m(signal, navigation)
            .expect("Galileo E5a broadcast group delay bias");
        observation_with_signal_id(
            timed_position_observation(navigation.sat, pseudorange_m, t_rx_s),
            signal,
        )
    }));
    observations.extend(beidou_navigation.iter().map(|navigation| {
        let signal = SigId { sat: navigation.sat, band: SignalBand::B2, code: SignalCode::B2I };
        let pseudorange_m = beidou_pseudorange_from_truth(
            navigation,
            truth_ecef_m,
            t_rx_s,
            receiver_clock_bias_s,
            beidou_bias_s,
        ) + beidou_broadcast_group_delay_code_bias_m(signal, navigation)
            .expect("BeiDou B2I broadcast group delay bias");
        observation_with_signal_id(
            timed_position_observation(navigation.sat, pseudorange_m, t_rx_s),
            signal,
        )
    }));

    let mut navigation = position_broadcast_navigation_from_gps_ephemerides(&gps_ephemerides);
    navigation.extend(galileo_navigation.iter().cloned().map(PositionBroadcastNavigation::Galileo));
    navigation.extend(position_broadcast_navigation_from_beidou_navigations(&beidou_navigation));

    MixedSecondaryBandCase {
        observations,
        navigation,
        truth_ecef_m,
        t_rx_s,
        receiver_clock_bias_s,
        galileo_bias_s,
        beidou_bias_s,
    }
}

fn galileo_pseudorange_from_truth(
    navigation: &GalileoBroadcastNavigationData,
    truth_ecef_m: (f64, f64, f64),
    t_rx_s: f64,
    receiver_clock_bias_s: f64,
    galileo_bias_s: f64,
) -> f64 {
    let mut tau = 0.07;
    let mut pseudorange_m = 0.0;
    for _ in 0..10 {
        let state = sat_state_galileo_e1(navigation, t_rx_s - tau, tau);
        let dx = truth_ecef_m.0 - state.x_m;
        let dy = truth_ecef_m.1 - state.y_m;
        let dz = truth_ecef_m.2 - state.z_m;
        let range_m = (dx * dx + dy * dy + dz * dz).sqrt();
        pseudorange_m = range_m + (receiver_clock_bias_s + galileo_bias_s) * 299_792_458.0
            - state.clock_correction.bias_s * 299_792_458.0;
        let next_tau = pseudorange_m / 299_792_458.0;
        if (next_tau - tau).abs() < 1.0e-12 {
            break;
        }
        tau = next_tau;
    }
    pseudorange_m
}

fn galileo_four_satellite_position_scenario(receiver_clock_bias_s: f64) -> GalileoPositionScenario {
    let truth_ecef_m = geodetic_to_ecef(37.0, -122.0, 10.0);
    let t_rx_s = 504_018.07 + receiver_clock_bias_s;
    let galileo_navigation = [
        (11, 0.17, 0.24),
        (12, 1.17, 0.84),
        (19, -0.83, 1.52),
        (24, 2.11, -0.41),
        (27, -2.34, 0.61),
        (30, 0.48, -1.14),
        (31, 1.84, 2.27),
        (33, -1.47, -2.02),
        (36, 2.73, 1.06),
        (37, -0.26, 2.88),
        (39, 0.95, -2.41),
        (40, -2.88, -0.73),
    ]
    .into_iter()
    .map(|(prn, omega0, m0)| sample_galileo_navigation(prn, omega0, m0))
    .filter(|navigation| {
        let pseudorange_m = galileo_pseudorange_from_truth(
            navigation,
            truth_ecef_m,
            t_rx_s,
            receiver_clock_bias_s,
            0.0,
        );
        let signal_travel_time_s = pseudorange_m / 299_792_458.0;
        let state =
            sat_state_galileo_e1(navigation, t_rx_s - signal_travel_time_s, signal_travel_time_s);
        let (_azimuth_deg, elevation_deg) = elevation_azimuth_deg(
            truth_ecef_m.0,
            truth_ecef_m.1,
            truth_ecef_m.2,
            state.x_m,
            state.y_m,
            state.z_m,
        );
        elevation_deg > 10.0
    })
    .take(4)
    .collect::<Vec<_>>();
    assert_eq!(galileo_navigation.len(), 4, "expected four visible Galileo satellites");
    let observations = galileo_navigation
        .iter()
        .map(|navigation| {
            let signal = SigId { sat: navigation.sat, band: SignalBand::E1, code: SignalCode::E1B };
            let pseudorange_m = galileo_pseudorange_from_truth(
                navigation,
                truth_ecef_m,
                t_rx_s,
                receiver_clock_bias_s,
                0.0,
            );
            observation_with_signal_id(
                timed_position_observation(navigation.sat, pseudorange_m, t_rx_s),
                signal,
            )
        })
        .collect::<Vec<_>>();
    let navigation = galileo_navigation
        .into_iter()
        .map(PositionBroadcastNavigation::Galileo)
        .collect::<Vec<_>>();

    GalileoPositionScenario { observations, navigation, truth_ecef_m, t_rx_s }
}

fn add_galileo_nequick_delay_to_observations(
    observations: &[PositionObservation],
    navigation: &[PositionBroadcastNavigation],
    truth_ecef_m: (f64, f64, f64),
) -> Vec<PositionObservation> {
    let (receiver_latitude_deg, receiver_longitude_deg, receiver_altitude_m) =
        ecef_to_geodetic(truth_ecef_m.0, truth_ecef_m.1, truth_ecef_m.2);
    let receiver = Llh {
        lat_deg: receiver_latitude_deg,
        lon_deg: receiver_longitude_deg,
        alt_m: receiver_altitude_m,
    };

    observations
        .iter()
        .map(|observation| {
            let Some(signal) = observation.signal_id else {
                return observation.clone();
            };
            let Some(gps_time) = observation.gps_receive_time else {
                return observation.clone();
            };
            let Some(PositionBroadcastNavigation::Galileo(galileo_navigation)) =
                navigation.iter().find(|entry| entry.sat() == observation.sat)
            else {
                return observation.clone();
            };
            let signal_travel_time_s = observation
                .signal_timing
                .map(|timing| timing.signal_travel_time_s.0)
                .unwrap_or(observation.pseudorange_m / 299_792_458.0);
            let state = sat_state_galileo_e1(
                galileo_navigation,
                observation.gps_receive_time.map(|time| time.tow_s).unwrap_or(0.0)
                    - signal_travel_time_s,
                signal_travel_time_s,
            );
            let (satellite_latitude_deg, satellite_longitude_deg, satellite_altitude_m) =
                ecef_to_geodetic(state.x_m, state.y_m, state.z_m);
            let satellite = Llh {
                lat_deg: satellite_latitude_deg,
                lon_deg: satellite_longitude_deg,
                alt_m: satellite_altitude_m,
            };
            let delay_m = galileo_navigation
                .nequick_delay_m(signal, receiver, satellite, gps_time)
                .expect("Galileo NeQuick delay");
            observation_with_pseudorange_bias(observation.clone(), delay_m)
        })
        .collect()
}

fn beidou_pseudorange_from_truth(
    navigation: &BeidouBroadcastNavigationData,
    truth_ecef_m: (f64, f64, f64),
    t_rx_s: f64,
    receiver_clock_bias_s: f64,
    beidou_bias_s: f64,
) -> f64 {
    let mut tau = 0.07;
    let mut pseudorange_m = 0.0;
    for _ in 0..10 {
        let state = sat_state_beidou_b1i(navigation, t_rx_s - tau, tau);
        let dx = truth_ecef_m.0 - state.x_m;
        let dy = truth_ecef_m.1 - state.y_m;
        let dz = truth_ecef_m.2 - state.z_m;
        let range_m = (dx * dx + dy * dy + dz * dz).sqrt();
        pseudorange_m = range_m + (receiver_clock_bias_s + beidou_bias_s) * 299_792_458.0
            - state.clock_correction.bias_s * 299_792_458.0;
        let next_tau = pseudorange_m / 299_792_458.0;
        if (next_tau - tau).abs() < 1.0e-12 {
            break;
        }
        tau = next_tau;
    }
    pseudorange_m
}

fn sample_glonass_navigation(
    prn: u8,
    state_vector: GlonassStateVector,
    clock_bias_s: f64,
    l2_l1_delay_s: f64,
) -> GlonassBroadcastNavigationFrame {
    let sat = SatId { constellation: Constellation::Glonass, prn };
    GlonassBroadcastNavigationFrame {
        sat,
        immediate: GlonassImmediateNavigationData {
            sat,
            frame_time: GlonassFrameTime { hour: 23, minute: 18, half_minute: false },
            ephemeris_reference_time_s: 83_700,
            tb_update_interval_min: 30,
            tb_is_odd: Some(true),
            state_vector,
            relative_frequency_bias: 0.0,
            clock_bias_s,
            l2_l1_delay_s: Some(l2_l1_delay_s),
            health: GlonassImmediateHealth { line_unhealthy: false, status_code: 0 },
            immediate_data_age_days: 28,
            satellite_type: GlonassSatelliteType::GlonassM,
            reported_slot: None,
            system_time: Some(GlonassSystemTime { day_number: 864, four_year_interval: Some(8) }),
            accuracy_code: Some(2),
        },
        system_time: Some(GlonassAlmanacTimeData {
            system_time: GlonassSystemTime { day_number: 864, four_year_interval: Some(8) },
            utc_offset_s: 0.0,
            gps_minus_glonass_s: -10_782.0,
        }),
        almanac_entries: Vec::new(),
    }
}

fn sample_glonass_navigation_slot14() -> GlonassBroadcastNavigationFrame {
    sample_glonass_navigation(
        14,
        GlonassStateVector {
            x_m: -7_557_760.253_906_25,
            y_m: -23_962_225.585_937_5,
            z_m: -4_337_567.871_093_75,
            vx_mps: 101.318_359_375,
            vy_mps: 602.112_770_080_566_4,
            vz_mps: -3_495.733_261_108_398_4,
            ax_mps2: -3.725_290_298_461_914e-6,
            ay_mps2: 0.0,
            az_mps2: 1.862_645_149_230_957e-6,
        },
        -2.572_406_083_345_413_2e-5,
        5.587_935_448e-9,
    )
}

fn sample_glonass_navigation_slot8() -> GlonassBroadcastNavigationFrame {
    sample_glonass_navigation(
        8,
        GlonassStateVector {
            x_m: 9_639_804.687_5,
            y_m: 5_433_780.761_718_75,
            z_m: 23_045_452.148_437_5,
            vx_mps: -1_955.772_399_902_343_8,
            vy_mps: 2_454.154_968_261_718_8,
            vz_mps: 238.015_174_865_722_66,
            ax_mps2: 9.313_225_746_154_785e-7,
            ay_mps2: 0.0,
            az_mps2: -2.793_967_723_846_435_5e-6,
        },
        6.508_920_341_730_117_7e-5,
        -3.725_290_298e-9,
    )
}

fn glonass_pseudorange_from_truth(
    navigation: &GlonassBroadcastNavigationFrame,
    truth_ecef_m: (f64, f64, f64),
    t_rx_s: f64,
    receiver_clock_bias_s: f64,
    glonass_bias_s: f64,
) -> f64 {
    let mut tau = 0.07;
    let mut pseudorange_m = 0.0;
    for _ in 0..10 {
        let state = sat_state_glonass_l1(navigation, t_rx_s - tau, tau)
            .expect("GLONASS broadcast state for pseudorange generation");
        let dx = truth_ecef_m.0 - state.x_m;
        let dy = truth_ecef_m.1 - state.y_m;
        let dz = truth_ecef_m.2 - state.z_m;
        let range_m = (dx * dx + dy * dy + dz * dz).sqrt();
        pseudorange_m = range_m + (receiver_clock_bias_s + glonass_bias_s) * 299_792_458.0
            - state.clock_correction.bias_s * 299_792_458.0;
        let next_tau = pseudorange_m / 299_792_458.0;
        if (next_tau - tau).abs() < 1.0e-12 {
            break;
        }
        tau = next_tau;
    }
    pseudorange_m
}

#[derive(Debug, Clone)]
struct ControlledMixedGeometryCase {
    gps_ephemerides: Vec<GpsEphemeris>,
    galileo_navigation: Vec<GalileoBroadcastNavigationData>,
    beidou_navigation: Vec<BeidouBroadcastNavigationData>,
    truth_ecef_m: (f64, f64, f64),
    t_rx_s: f64,
    receiver_clock_bias_s: f64,
    galileo_bias_s: f64,
    beidou_bias_s: f64,
}

impl ControlledMixedGeometryCase {
    fn gps_ephemerides(&self, count: usize) -> Vec<GpsEphemeris> {
        self.gps_ephemerides.iter().take(count).cloned().collect()
    }

    fn observations(
        &self,
        gps_count: usize,
        galileo_count: usize,
        beidou_count: usize,
    ) -> Vec<PositionObservation> {
        let mut observations = self
            .gps_ephemerides
            .iter()
            .take(gps_count)
            .map(|ephemeris| {
                timed_position_observation_from_truth(
                    ephemeris,
                    self.truth_ecef_m,
                    self.t_rx_s,
                    self.receiver_clock_bias_s,
                )
            })
            .collect::<Vec<_>>();
        observations.extend(self.galileo_navigation.iter().take(galileo_count).map(|navigation| {
            let pseudorange_m = galileo_pseudorange_from_truth(
                navigation,
                self.truth_ecef_m,
                self.t_rx_s,
                self.receiver_clock_bias_s,
                self.galileo_bias_s,
            );
            timed_position_observation(navigation.sat, pseudorange_m, self.t_rx_s)
        }));
        observations.extend(self.beidou_navigation.iter().take(beidou_count).map(|navigation| {
            let pseudorange_m = beidou_pseudorange_from_truth(
                navigation,
                self.truth_ecef_m,
                self.t_rx_s,
                self.receiver_clock_bias_s,
                self.beidou_bias_s,
            );
            timed_position_observation(navigation.sat, pseudorange_m, self.t_rx_s)
        }));
        observations
    }

    fn navigation(
        &self,
        gps_count: usize,
        galileo_count: usize,
        beidou_count: usize,
    ) -> Vec<PositionBroadcastNavigation> {
        let gps_ephemerides = self.gps_ephemerides(gps_count);
        let mut navigation = position_broadcast_navigation_from_gps_ephemerides(&gps_ephemerides);
        navigation.extend(
            self.galileo_navigation
                .iter()
                .take(galileo_count)
                .cloned()
                .map(PositionBroadcastNavigation::Galileo),
        );
        let beidou_navigation =
            self.beidou_navigation.iter().take(beidou_count).cloned().collect::<Vec<_>>();
        navigation
            .extend(position_broadcast_navigation_from_beidou_navigations(&beidou_navigation));
        navigation
    }
}

fn controlled_mixed_geometry_case() -> ControlledMixedGeometryCase {
    ControlledMixedGeometryCase {
        gps_ephemerides: sample_ephemerides(),
        galileo_navigation: vec![
            sample_galileo_navigation(19, 1.17, 0.84),
            sample_galileo_navigation(24, -0.83, 1.52),
        ],
        beidou_navigation: vec![
            sample_beidou_navigation_at(11, 0.77, 0.53, 504_018.0),
            sample_beidou_navigation_at(12, -1.09, 1.31, 504_018.0),
        ],
        truth_ecef_m: geodetic_to_ecef(37.0, -122.0, 10.0),
        t_rx_s: 504_018.07 + 2.75e-4,
        receiver_clock_bias_s: 2.75e-4,
        galileo_bias_s: -1.15e-6,
        beidou_bias_s: 9.25e-7,
    }
}

#[test]
fn position_solver_returns_solution() {
    let solver = PositionSolver::new();
    let solution = solver.solve_wls(&[], &[], 0.0);
    assert!(solution.is_none());
}

#[test]
fn ephemeris_is_constructible() {
    let eph = Ephemeris { sat: SatId { constellation: Constellation::Gps, prn: 1 }, toe_s: 0.0 };
    assert_eq!(eph.sat.prn, 1);
}

#[test]
fn position_observation_constructible() {
    let obs = PositionObservation {
        sat: SatId { constellation: Constellation::Gps, prn: 3 },
        pseudorange_m: 20_000_000.0,
        doppler_hz: None,
        doppler_var_hz2: None,
        cn0_dbhz: 40.0,
        elevation_deg: None,
        weight: 1.0,
        gps_receive_time: None,
        signal_timing: None,
        signal_id: None,
    };
    assert_eq!(obs.sat.prn, 3);
}

#[test]
fn position_solver_solves_observations_without_signal_timing_when_pseudoranges_are_finite() {
    let t_rx_s = 504_018.07;
    let ephs = sample_ephemerides();
    let observations = ephs
        .iter()
        .map(|eph| PositionObservation {
            sat: eph.sat,
            pseudorange_m: 21_000_000.0,
            doppler_hz: None,
            doppler_var_hz2: None,
            cn0_dbhz: 45.0,
            elevation_deg: None,
            weight: 1.0,
            gps_receive_time: Some(GpsTime { week: 0, tow_s: t_rx_s }),
            signal_timing: None,
            signal_id: None,
        })
        .collect::<Vec<_>>();

    let solution = PositionSolver::new()
        .solve_wls(&observations, &ephs, t_rx_s)
        .expect("finite pseudoranges without signal timing should still solve");

    assert!(solution.ecef_x_m.is_finite());
    assert!(solution.ecef_y_m.is_finite());
    assert!(solution.ecef_z_m.is_finite());
}

#[test]
fn position_solver_refuses_inconsistent_signal_timing() {
    let t_rx_s = 504_018.07;
    let ephs = sample_ephemerides();
    let observations = ephs
        .iter()
        .map(|eph| {
            let mut obs = timed_position_observation(eph.sat, 21_000_000.0, t_rx_s);
            obs.signal_timing = obs.signal_timing.map(|mut timing| {
                timing.signal_travel_time_s = Seconds(timing.signal_travel_time_s.0 + 0.01);
                timing
            });
            obs
        })
        .collect::<Vec<_>>();

    assert!(PositionSolver::new().solve_wls(&observations, &ephs, t_rx_s).is_none());
}

#[test]
fn single_point_solver_recovers_four_satellite_fix() {
    let scenario = four_satellite_position_scenario(0.0);
    let solution = PositionSolver::new()
        .solve_wls(&scenario.observations, &scenario.ephemerides, scenario.t_rx_s)
        .expect("timed four-satellite observations should solve");

    assert_eq!(solution.sat_count, 4);
    assert_eq!(solution.used_sat_count, 4);
    assert_eq!(solution.rejected_sat_count, 0);
    assert!(solution.rejected.is_empty(), "unexpected rejections: {:?}", solution.rejected);
    assert!((solution.ecef_x_m - scenario.truth_ecef_m.0).abs() < 5.0);
    assert!((solution.ecef_y_m - scenario.truth_ecef_m.1).abs() < 5.0);
    assert!((solution.ecef_z_m - scenario.truth_ecef_m.2).abs() < 5.0);
    assert!(solution.clock_bias_s.abs() < 1.0e-9);
    assert!(solution.hdop.is_some());
    assert!(solution.vdop.is_some());
    assert!(solution.gdop.is_some());
    assert!(solution.tdop.is_some());
    assert!(
        (solution.pdop.powi(2)
            - (solution.hdop.expect("hdop").powi(2) + solution.vdop.expect("vdop").powi(2)))
        .abs()
            < 1.0e-9
    );
    assert!(
        (solution.gdop.expect("gdop").powi(2)
            - (solution.pdop.powi(2) + solution.tdop.expect("tdop").powi(2)))
        .abs()
            < 1.0e-9
    );
}

#[test]
fn single_point_solver_reports_ecef_position_covariance() {
    let scenario = four_satellite_position_scenario(0.0);
    let solution = PositionSolver::new()
        .solve_wls(&scenario.observations, &scenario.ephemerides, scenario.t_rx_s)
        .expect("timed four-satellite observations should solve");
    let covariance =
        solution.position_covariance_ecef_m2.expect("solver should emit position covariance");

    for row in covariance {
        for value in row {
            assert!(value.is_finite());
        }
    }
    assert!(covariance[0][0] > 0.0);
    assert!(covariance[1][1] > 0.0);
    assert!(covariance[2][2] > 0.0);
    assert!(solution.sigma_e_m.expect("east sigma") > 0.0);
    assert!(solution.sigma_n_m.expect("north sigma") > 0.0);
    assert!(solution.sigma_u_m.expect("up sigma") > 0.0);
    assert!(solution.horizontal_error_ellipse_major_axis_m.expect("ellipse major axis") > 0.0);
    assert!(solution.horizontal_error_ellipse_minor_axis_m.expect("ellipse minor axis") > 0.0);
    assert!(solution.horizontal_error_ellipse_azimuth_deg.expect("ellipse azimuth").is_finite());
}

#[test]
fn single_point_solver_recovers_receiver_clock_bias() {
    let scenario = four_satellite_position_scenario(2.75e-4);
    let solution = PositionSolver::new()
        .solve_wls(&scenario.observations, &scenario.ephemerides, scenario.t_rx_s)
        .expect("biased timed pseudoranges should solve");

    assert_eq!(solution.used_sat_count, 4);
    assert_eq!(solution.rejected_sat_count, 0);
    assert!((solution.ecef_x_m - scenario.truth_ecef_m.0).abs() < 5.0);
    assert!((solution.ecef_y_m - scenario.truth_ecef_m.1).abs() < 5.0);
    assert!((solution.ecef_z_m - scenario.truth_ecef_m.2).abs() < 5.0);
    assert!((solution.clock_bias_s - scenario.receiver_clock_bias_s).abs() < 1.0e-9);
}

#[test]
fn mixed_gps_galileo_solver_recovers_position_and_clock_split() {
    let gps_ephemerides = sample_ephemerides();
    let galileo_navigation =
        vec![sample_galileo_navigation(19, 1.17, 0.84), sample_galileo_navigation(24, -0.83, 1.52)];
    let truth_ecef_m = geodetic_to_ecef(37.0, -122.0, 10.0);
    let receiver_clock_bias_s = 2.75e-4;
    let galileo_bias_s = -1.15e-6;
    let t_rx_s = 504_018.07 + receiver_clock_bias_s;

    let mut observations = gps_ephemerides
        .iter()
        .map(|ephemeris| {
            timed_position_observation_from_truth(
                ephemeris,
                truth_ecef_m,
                t_rx_s,
                receiver_clock_bias_s,
            )
        })
        .collect::<Vec<_>>();
    observations.extend(galileo_navigation.iter().map(|navigation| {
        let pseudorange_m = galileo_pseudorange_from_truth(
            navigation,
            truth_ecef_m,
            t_rx_s,
            receiver_clock_bias_s,
            galileo_bias_s,
        );
        timed_position_observation(navigation.sat, pseudorange_m, t_rx_s)
    }));

    let mut navigation = position_broadcast_navigation_from_gps_ephemerides(&gps_ephemerides);
    navigation.extend(galileo_navigation.iter().cloned().map(PositionBroadcastNavigation::Galileo));

    let solution = PositionSolver::new()
        .solve_wls_with_navigation_data(&observations, &navigation, t_rx_s)
        .expect("mixed gps+galileo observations should solve");

    assert_eq!(solution.clock_reference_constellation, Constellation::Gps);
    assert_eq!(solution.used_sat_count, 6);
    assert_eq!(solution.rejected_sat_count, 0);
    assert!(solution.rejected.is_empty(), "unexpected rejections: {:?}", solution.rejected);
    assert!(
        position_error_3d_m(solution.ecef_x_m, solution.ecef_y_m, solution.ecef_z_m, truth_ecef_m,)
            < 5.0
    );
    assert!((solution.clock_bias_s - receiver_clock_bias_s).abs() < 1.0e-9);
    assert_eq!(solution.inter_system_biases.len(), 1);
    assert_eq!(solution.inter_system_biases[0].constellation, Constellation::Galileo);
    assert!((solution.inter_system_biases[0].bias_s.0 - galileo_bias_s).abs() < 1.0e-9);
}

#[test]
fn mixed_gps_beidou_solver_recovers_position_and_clock_split() {
    let gps_ephemerides = sample_gps_ephemerides_at(345_618.0);
    let beidou_navigation =
        vec![sample_beidou_navigation(11, 0.77, 0.53), sample_beidou_navigation(12, -1.09, 1.31)];
    let truth_ecef_m = geodetic_to_ecef(37.0, -122.0, 10.0);
    let receiver_clock_bias_s = 2.75e-4;
    let beidou_bias_s = 9.25e-7;
    let t_rx_s = 345_618.07 + receiver_clock_bias_s;

    let mut observations = gps_ephemerides
        .iter()
        .map(|ephemeris| {
            timed_position_observation_from_truth(
                ephemeris,
                truth_ecef_m,
                t_rx_s,
                receiver_clock_bias_s,
            )
        })
        .collect::<Vec<_>>();
    observations.extend(beidou_navigation.iter().map(|navigation| {
        let pseudorange_m = beidou_pseudorange_from_truth(
            navigation,
            truth_ecef_m,
            t_rx_s,
            receiver_clock_bias_s,
            beidou_bias_s,
        );
        timed_position_observation(navigation.sat, pseudorange_m, t_rx_s)
    }));

    let mut navigation = position_broadcast_navigation_from_gps_ephemerides(&gps_ephemerides);
    navigation.extend(position_broadcast_navigation_from_beidou_navigations(&beidou_navigation));

    let solution = PositionSolver::new()
        .solve_wls_with_navigation_data(&observations, &navigation, t_rx_s)
        .expect("mixed gps+beidou observations should solve");

    assert_eq!(solution.clock_reference_constellation, Constellation::Gps);
    assert_eq!(solution.used_sat_count, 6);
    assert_eq!(solution.rejected_sat_count, 0);
    assert!(solution.rejected.is_empty(), "unexpected rejections: {:?}", solution.rejected);
    assert!(
        position_error_3d_m(solution.ecef_x_m, solution.ecef_y_m, solution.ecef_z_m, truth_ecef_m,)
            < 5.0
    );
    assert!((solution.clock_bias_s - receiver_clock_bias_s).abs() < 1.0e-8);
    assert_eq!(solution.inter_system_biases.len(), 1);
    assert_eq!(solution.inter_system_biases[0].constellation, Constellation::Beidou);
    assert!((solution.inter_system_biases[0].bias_s.0 - beidou_bias_s).abs() < 1.0e-8);
}

#[test]
fn mixed_gps_galileo_beidou_solver_recovers_position_and_clock_splits() {
    let gps_ephemerides = sample_ephemerides();
    let galileo_navigation =
        vec![sample_galileo_navigation(19, 1.17, 0.84), sample_galileo_navigation(24, -0.83, 1.52)];
    let beidou_navigation = vec![
        sample_beidou_navigation_at(11, 0.77, 0.53, 504_018.0),
        sample_beidou_navigation_at(12, -1.09, 1.31, 504_018.0),
    ];
    let truth_ecef_m = geodetic_to_ecef(37.0, -122.0, 10.0);
    let receiver_clock_bias_s = 2.75e-4;
    let galileo_bias_s = -1.15e-6;
    let beidou_bias_s = 9.25e-7;
    let t_rx_s = 504_018.07 + receiver_clock_bias_s;

    let mut observations = gps_ephemerides
        .iter()
        .map(|ephemeris| {
            timed_position_observation_from_truth(
                ephemeris,
                truth_ecef_m,
                t_rx_s,
                receiver_clock_bias_s,
            )
        })
        .collect::<Vec<_>>();
    observations.extend(galileo_navigation.iter().map(|navigation| {
        let pseudorange_m = galileo_pseudorange_from_truth(
            navigation,
            truth_ecef_m,
            t_rx_s,
            receiver_clock_bias_s,
            galileo_bias_s,
        );
        timed_position_observation(navigation.sat, pseudorange_m, t_rx_s)
    }));
    observations.extend(beidou_navigation.iter().map(|navigation| {
        let pseudorange_m = beidou_pseudorange_from_truth(
            navigation,
            truth_ecef_m,
            t_rx_s,
            receiver_clock_bias_s,
            beidou_bias_s,
        );
        timed_position_observation(navigation.sat, pseudorange_m, t_rx_s)
    }));

    let mut navigation = position_broadcast_navigation_from_gps_ephemerides(&gps_ephemerides);
    navigation.extend(galileo_navigation.iter().cloned().map(PositionBroadcastNavigation::Galileo));
    navigation.extend(position_broadcast_navigation_from_beidou_navigations(&beidou_navigation));

    let solution = PositionSolver::new()
        .solve_wls_with_navigation_data(&observations, &navigation, t_rx_s)
        .expect("mixed gps+galileo+beidou observations should solve");

    assert_eq!(solution.clock_reference_constellation, Constellation::Gps);
    assert_eq!(solution.used_sat_count, 8);
    assert_eq!(solution.rejected_sat_count, 0);
    assert!(solution.rejected.is_empty(), "unexpected rejections: {:?}", solution.rejected);
    assert!(
        position_error_3d_m(solution.ecef_x_m, solution.ecef_y_m, solution.ecef_z_m, truth_ecef_m,)
            < 5.0
    );
    assert!((solution.clock_bias_s - receiver_clock_bias_s).abs() < 1.0e-8);
    assert_eq!(solution.inter_system_biases.len(), 2);
    assert_eq!(solution.constellation_residual_rms.len(), 3);
    for (constellation, pre_fit_sat_count, post_fit_sat_count) in
        [(Constellation::Gps, 4, 4), (Constellation::Galileo, 2, 2), (Constellation::Beidou, 2, 2)]
    {
        let summary = solution
            .constellation_residual_rms
            .iter()
            .find(|summary| summary.constellation == constellation)
            .unwrap_or_else(|| panic!("missing residual RMS summary for {constellation:?}"));
        assert_eq!(summary.pre_fit_sat_count, pre_fit_sat_count);
        assert_eq!(summary.post_fit_sat_count, post_fit_sat_count);
        assert!(summary.pre_fit_rms_m.is_some_and(|value| value.0.is_finite() && value.0 >= 0.0));
        assert!(summary.post_fit_rms_m.is_some_and(|value| value.0.is_finite() && value.0 >= 0.0));
    }
    assert_eq!(
        solution
            .constellation_residual_rms
            .iter()
            .map(|summary| summary.post_fit_sat_count)
            .sum::<usize>(),
        solution.used_sat_count
    );
    let galileo_isb = solution
        .inter_system_biases
        .iter()
        .find(|bias| bias.constellation == Constellation::Galileo)
        .expect("galileo inter-system bias");
    let beidou_isb = solution
        .inter_system_biases
        .iter()
        .find(|bias| bias.constellation == Constellation::Beidou)
        .expect("beidou inter-system bias");
    assert!((galileo_isb.bias_s.0 - galileo_bias_s).abs() < 1.0e-8);
    assert!((beidou_isb.bias_s.0 - beidou_bias_s).abs() < 1.0e-8);
}

#[test]
fn gps_l5_observations_show_tgd_residuals_when_corrections_are_disabled() {
    let truth_ecef_m = geodetic_to_ecef(37.0, -122.0, 10.0);
    let receiver_clock_bias_s = 2.75e-4;
    let t_rx_s = 504_018.07 + receiver_clock_bias_s;
    let mut gps_ephemerides =
        sample_ephemerides_with_clock_parameters(&broadcast_clock_fixture_parameters());
    let mut fifth_ephemeris = sample_ephemeris(5, 3.2, 3.4);
    fifth_ephemeris.toe_s = 504_000.0;
    fifth_ephemeris.toc_s = 504_018.0;
    fifth_ephemeris.tgd = -12.0e-9;
    gps_ephemerides.push(fifth_ephemeris);

    let observations = gps_ephemerides
        .iter()
        .map(|ephemeris| {
            let signal =
                SigId { sat: ephemeris.sat, band: SignalBand::L5, code: SignalCode::Unknown };
            let observation = timed_position_observation_from_truth(
                ephemeris,
                truth_ecef_m,
                t_rx_s,
                receiver_clock_bias_s,
            );
            observation_with_signal_id(
                observation_with_pseudorange_bias(
                    observation,
                    gps_broadcast_group_delay_code_bias_m(signal, ephemeris)
                        .expect("GPS L5 broadcast group delay bias"),
                ),
                signal,
            )
        })
        .collect::<Vec<_>>();
    let navigation = position_broadcast_navigation_from_gps_ephemerides(&gps_ephemerides);

    let corrected_solver = PositionSolver::new();
    let uncorrected_solver = PositionSolver::new().with_broadcast_group_delay(false);
    let corrected_solution = corrected_solver
        .solve_wls_with_navigation_data(&observations, &navigation, t_rx_s)
        .expect("corrected GPS L5 observations should solve");
    let uncorrected_solution = uncorrected_solver
        .solve_wls_with_navigation_data(&observations, &navigation, t_rx_s)
        .expect("uncorrected GPS L5 observations should still solve");

    let corrected_error_m = position_error_3d_m(
        corrected_solution.ecef_x_m,
        corrected_solution.ecef_y_m,
        corrected_solution.ecef_z_m,
        truth_ecef_m,
    );
    let uncorrected_error_m = position_error_3d_m(
        uncorrected_solution.ecef_x_m,
        uncorrected_solution.ecef_y_m,
        uncorrected_solution.ecef_z_m,
        truth_ecef_m,
    );

    assert_eq!(corrected_solution.used_sat_count, 5);
    assert!(corrected_solution.rejected.is_empty());
    assert!(uncorrected_solution.rejected.is_empty());
    assert!(corrected_error_m < 5.0);
    assert!(uncorrected_error_m > corrected_error_m + 1.0);
    assert!(corrected_solution.post_fit_residual_rms_m < 0.01);
    assert!(
        uncorrected_solution.post_fit_residual_rms_m
            > corrected_solution.post_fit_residual_rms_m + 0.5
    );
    assert!(
        max_abs_residual_m(&uncorrected_solution.residuals)
            > max_abs_residual_m(&corrected_solution.residuals) + 0.5
    );
}

#[test]
fn mixed_secondary_band_observations_recover_with_broadcast_group_delay_corrections() {
    let scenario = mixed_secondary_band_case();
    let solution = PositionSolver::new()
        .solve_wls_with_navigation_data(
            &scenario.observations,
            &scenario.navigation,
            scenario.t_rx_s,
        )
        .expect("mixed secondary-band observations should solve");

    assert_eq!(solution.clock_reference_constellation, Constellation::Gps);
    assert_eq!(solution.used_sat_count, 8);
    assert!(solution.rejected.is_empty(), "unexpected rejections: {:?}", solution.rejected);
    assert!(
        position_error_3d_m(
            solution.ecef_x_m,
            solution.ecef_y_m,
            solution.ecef_z_m,
            scenario.truth_ecef_m,
        ) < 5.0
    );
    assert!((solution.clock_bias_s - scenario.receiver_clock_bias_s).abs() < 1.0e-8);
    assert_eq!(solution.inter_system_biases.len(), 2);
    let galileo_isb = solution
        .inter_system_biases
        .iter()
        .find(|bias| bias.constellation == Constellation::Galileo)
        .expect("galileo inter-system bias");
    let beidou_isb = solution
        .inter_system_biases
        .iter()
        .find(|bias| bias.constellation == Constellation::Beidou)
        .expect("beidou inter-system bias");
    assert!((galileo_isb.bias_s.0 - scenario.galileo_bias_s).abs() < 1.0e-8);
    assert!((beidou_isb.bias_s.0 - scenario.beidou_bias_s).abs() < 1.0e-8);
}

#[test]
fn mixed_secondary_band_observations_show_group_delay_residuals_when_corrections_are_disabled() {
    let scenario = mixed_secondary_band_case();
    let corrected_solver = PositionSolver::new();
    let uncorrected_solver = PositionSolver::new().with_broadcast_group_delay(false);
    let corrected_solution = corrected_solver
        .solve_wls_with_navigation_data(
            &scenario.observations,
            &scenario.navigation,
            scenario.t_rx_s,
        )
        .expect("corrected mixed secondary-band observations should solve");
    let uncorrected_solution = uncorrected_solver
        .solve_wls_with_navigation_data(
            &scenario.observations,
            &scenario.navigation,
            scenario.t_rx_s,
        )
        .expect("uncorrected mixed secondary-band observations should still solve");

    let corrected_error_m = position_error_3d_m(
        corrected_solution.ecef_x_m,
        corrected_solution.ecef_y_m,
        corrected_solution.ecef_z_m,
        scenario.truth_ecef_m,
    );
    let uncorrected_error_m = position_error_3d_m(
        uncorrected_solution.ecef_x_m,
        uncorrected_solution.ecef_y_m,
        uncorrected_solution.ecef_z_m,
        scenario.truth_ecef_m,
    );

    assert!(corrected_solution.rejected.is_empty());
    assert!(uncorrected_solution.rejected.is_empty());
    assert!(corrected_error_m < 5.0);
    assert!(uncorrected_error_m > corrected_error_m + 1.0);
    assert!(corrected_solution.post_fit_residual_rms_m < 0.01);
    assert!(
        uncorrected_solution.post_fit_residual_rms_m
            > corrected_solution.post_fit_residual_rms_m + 0.5
    );
    assert!(
        max_abs_residual_m(&uncorrected_solution.residuals)
            > max_abs_residual_m(&corrected_solution.residuals) + 0.5
    );
}

#[test]
fn mixed_gps_galileo_beidou_solver_lowers_dop_against_same_gps_only_case() {
    let scenario = controlled_mixed_geometry_case();

    let gps_ephemerides = scenario.gps_ephemerides(4);
    let gps_only = PositionSolver::new()
        .solve_wls(&scenario.observations(4, 0, 0), &gps_ephemerides, scenario.t_rx_s)
        .expect("gps-only geometry should solve");
    let mixed = PositionSolver::new()
        .solve_wls_with_navigation_data(
            &scenario.observations(4, 2, 2),
            &scenario.navigation(4, 2, 2),
            scenario.t_rx_s,
        )
        .expect("mixed geometry should solve");

    assert!(
        position_error_3d_m(
            gps_only.ecef_x_m,
            gps_only.ecef_y_m,
            gps_only.ecef_z_m,
            scenario.truth_ecef_m,
        ) < 5.0
    );
    assert!(
        position_error_3d_m(mixed.ecef_x_m, mixed.ecef_y_m, mixed.ecef_z_m, scenario.truth_ecef_m,)
            < 5.0
    );
    assert!(gps_only.gdop.is_some());
    assert!(mixed.gdop.is_some());
    assert!(
        mixed.pdop < gps_only.pdop,
        "expected mixed PDOP {} to improve on GPS-only {}",
        mixed.pdop,
        gps_only.pdop
    );
    assert!(
        mixed.gdop.expect("mixed gdop") < gps_only.gdop.expect("gps-only gdop"),
        "expected mixed GDOP {:?} to improve on GPS-only {:?}",
        mixed.gdop,
        gps_only.gdop
    );
}

#[test]
fn mixed_gps_galileo_beidou_solver_improves_availability_against_gps_only_case() {
    let scenario = controlled_mixed_geometry_case();

    let gps_only = PositionSolver::new().solve_wls(
        &scenario.observations(3, 0, 0),
        &scenario.gps_ephemerides(3),
        scenario.t_rx_s,
    );
    assert!(
        gps_only.is_none(),
        "three GPS observations should stay unavailable in the single-constellation solve"
    );

    let mixed = PositionSolver::new()
        .solve_wls_with_navigation_data(
            &scenario.observations(3, 2, 2),
            &scenario.navigation(3, 2, 2),
            scenario.t_rx_s,
        )
        .expect("mixed observations should restore solution availability");

    assert_eq!(mixed.used_sat_count, 7);
    assert_eq!(mixed.inter_system_biases.len(), 2);
    assert!(
        position_error_3d_m(mixed.ecef_x_m, mixed.ecef_y_m, mixed.ecef_z_m, scenario.truth_ecef_m,)
            < 5.0
    );
    assert!(mixed.pdop.is_finite());
    assert!(mixed.gdop.is_some());
}

#[test]
fn mixed_gps_glonass_solver_recovers_position_and_clock_split() {
    let gps_ephemerides = sample_ephemerides();
    let glonass_navigation =
        vec![sample_glonass_navigation_slot14(), sample_glonass_navigation_slot8()];
    let truth_ecef_m = geodetic_to_ecef(37.0, -122.0, 10.0);
    let receiver_clock_bias_s = 2.75e-4;
    let glonass_bias_s = 8.5e-7;
    let t_rx_s = 504_918.07 + receiver_clock_bias_s;

    let mut observations = gps_ephemerides
        .iter()
        .map(|ephemeris| {
            timed_position_observation_from_truth(
                ephemeris,
                truth_ecef_m,
                t_rx_s,
                receiver_clock_bias_s,
            )
        })
        .collect::<Vec<_>>();
    observations.extend(glonass_navigation.iter().map(|navigation| {
        let pseudorange_m = glonass_pseudorange_from_truth(
            navigation,
            truth_ecef_m,
            t_rx_s,
            receiver_clock_bias_s,
            glonass_bias_s,
        );
        timed_position_observation(navigation.sat, pseudorange_m, t_rx_s)
    }));

    let mut navigation = position_broadcast_navigation_from_gps_ephemerides(&gps_ephemerides);
    navigation.extend(position_broadcast_navigation_from_glonass_frames(&glonass_navigation));

    let solution = PositionSolver::new()
        .solve_wls_with_navigation_data(&observations, &navigation, t_rx_s)
        .expect("mixed gps+glonass observations should solve");

    assert_eq!(solution.clock_reference_constellation, Constellation::Gps);
    assert_eq!(solution.used_sat_count, 6);
    assert_eq!(solution.rejected_sat_count, 0);
    assert!(solution.rejected.is_empty(), "unexpected rejections: {:?}", solution.rejected);
    assert!(
        position_error_3d_m(solution.ecef_x_m, solution.ecef_y_m, solution.ecef_z_m, truth_ecef_m,)
            < 5.0
    );
    assert!((solution.clock_bias_s - receiver_clock_bias_s).abs() < 1.0e-8);
    assert_eq!(solution.inter_system_biases.len(), 1);
    assert_eq!(solution.inter_system_biases[0].constellation, Constellation::Glonass);
    assert!((solution.inter_system_biases[0].bias_s.0 - glonass_bias_s).abs() < 1.0e-8);
    assert_eq!(solution.constellation_residual_rms.len(), 2);
    for (constellation, pre_fit_sat_count, post_fit_sat_count) in
        [(Constellation::Gps, 4, 4), (Constellation::Glonass, 2, 2)]
    {
        let summary = solution
            .constellation_residual_rms
            .iter()
            .find(|summary| summary.constellation == constellation)
            .unwrap_or_else(|| panic!("missing residual RMS summary for {constellation:?}"));
        assert_eq!(summary.pre_fit_sat_count, pre_fit_sat_count);
        assert_eq!(summary.post_fit_sat_count, post_fit_sat_count);
        assert!(summary.pre_fit_rms_m.is_some_and(|value| value.0.is_finite() && value.0 >= 0.0));
        assert!(summary.post_fit_rms_m.is_some_and(|value| value.0.is_finite() && value.0 >= 0.0));
    }
    assert_eq!(
        solution
            .constellation_residual_rms
            .iter()
            .map(|summary| summary.post_fit_sat_count)
            .sum::<usize>(),
        solution.used_sat_count
    );
}

#[test]
fn single_point_solver_applies_broadcast_ionosphere_correction() {
    let scenario = four_satellite_position_scenario(0.0);
    let klobuchar = sample_klobuchar_coefficients();
    let ionosphere_biased_observations = add_klobuchar_delay_to_observations(
        &scenario.observations,
        &scenario.ephemerides,
        scenario.truth_ecef_m,
        scenario.t_rx_s,
        klobuchar,
    );

    let corrected_solution = PositionSolver::new()
        .solve_wls_with_broadcast_ionosphere(
            &ionosphere_biased_observations,
            &scenario.ephemerides,
            scenario.t_rx_s,
            Some(&klobuchar),
        )
        .expect("ionosphere-corrected observations should solve");
    let uncorrected_solution = PositionSolver::new()
        .solve_wls(&ionosphere_biased_observations, &scenario.ephemerides, scenario.t_rx_s)
        .expect("uncorrected observations should still solve");

    let corrected_error_m = position_error_3d_m(
        corrected_solution.ecef_x_m,
        corrected_solution.ecef_y_m,
        corrected_solution.ecef_z_m,
        scenario.truth_ecef_m,
    );
    let uncorrected_error_m = position_error_3d_m(
        uncorrected_solution.ecef_x_m,
        uncorrected_solution.ecef_y_m,
        uncorrected_solution.ecef_z_m,
        scenario.truth_ecef_m,
    );

    assert!(corrected_solution.broadcast_ionosphere_applied);
    assert!(!uncorrected_solution.broadcast_ionosphere_applied);
    assert!(corrected_error_m < 5.0);
    assert!(uncorrected_error_m > corrected_error_m + 3.0);
}

#[test]
fn single_point_solver_uses_gps_broadcast_navigation_klobuchar_payload() {
    let scenario = four_satellite_position_scenario(0.0);
    let klobuchar = sample_klobuchar_coefficients();
    let navigation = GpsBroadcastNavigationData {
        ephemerides: scenario.ephemerides.clone(),
        klobuchar: Some(klobuchar),
    };
    let ionosphere_biased_observations = add_klobuchar_delay_to_observations(
        &scenario.observations,
        &scenario.ephemerides,
        scenario.truth_ecef_m,
        scenario.t_rx_s,
        klobuchar,
    );

    let corrected_solution = PositionSolver::new()
        .solve_wls_with_gps_broadcast_navigation(
            &ionosphere_biased_observations,
            &navigation,
            scenario.t_rx_s,
        )
        .expect("broadcast navigation payload should solve");
    let uncorrected_solution = PositionSolver::new()
        .solve_wls(&ionosphere_biased_observations, &navigation.ephemerides, scenario.t_rx_s)
        .expect("uncorrected observations should still solve");

    let corrected_error_m = position_error_3d_m(
        corrected_solution.ecef_x_m,
        corrected_solution.ecef_y_m,
        corrected_solution.ecef_z_m,
        scenario.truth_ecef_m,
    );
    let uncorrected_error_m = position_error_3d_m(
        uncorrected_solution.ecef_x_m,
        uncorrected_solution.ecef_y_m,
        uncorrected_solution.ecef_z_m,
        scenario.truth_ecef_m,
    );

    assert!(corrected_solution.broadcast_ionosphere_applied);
    assert!(!uncorrected_solution.broadcast_ionosphere_applied);
    assert!(corrected_error_m < 5.0);
    assert!(uncorrected_error_m > corrected_error_m + 3.0);
}

#[test]
fn single_point_solver_applies_galileo_broadcast_ionosphere_correction() {
    let scenario = galileo_four_satellite_position_scenario(0.0);
    let ionosphere_biased_observations = add_galileo_nequick_delay_to_observations(
        &scenario.observations,
        &scenario.navigation,
        scenario.truth_ecef_m,
    );

    let corrected_solution = PositionSolver::new()
        .solve_wls_with_navigation_data(
            &ionosphere_biased_observations,
            &scenario.navigation,
            scenario.t_rx_s,
        )
        .expect("Galileo broadcast-ionosphere observations should solve");
    let uncorrected_solution = PositionSolver::new()
        .with_broadcast_ionosphere(false)
        .solve_wls_with_navigation_data(
            &ionosphere_biased_observations,
            &scenario.navigation,
            scenario.t_rx_s,
        )
        .expect("uncorrected Galileo observations should still solve");

    let corrected_error_m = position_error_3d_m(
        corrected_solution.ecef_x_m,
        corrected_solution.ecef_y_m,
        corrected_solution.ecef_z_m,
        scenario.truth_ecef_m,
    );
    let uncorrected_error_m = position_error_3d_m(
        uncorrected_solution.ecef_x_m,
        uncorrected_solution.ecef_y_m,
        uncorrected_solution.ecef_z_m,
        scenario.truth_ecef_m,
    );

    assert!(corrected_solution.broadcast_ionosphere_applied);
    assert!(!uncorrected_solution.broadcast_ionosphere_applied);
    assert!(corrected_error_m < 5.0);
    assert!(uncorrected_error_m > corrected_error_m + 1.0);
}

#[test]
fn single_point_solver_applies_saastamoinen_troposphere_correction() {
    let scenario = four_satellite_position_scenario(0.0);
    let troposphere_biased_observations = add_saastamoinen_delay_to_observations(
        &scenario.observations,
        &scenario.ephemerides,
        scenario.truth_ecef_m,
        scenario.t_rx_s,
    );

    let corrected_solution = PositionSolver { apply_troposphere: true, ..PositionSolver::new() }
        .solve_wls(&troposphere_biased_observations, &scenario.ephemerides, scenario.t_rx_s)
        .expect("troposphere-corrected observations should solve");
    let uncorrected_solution = PositionSolver::new()
        .solve_wls(&troposphere_biased_observations, &scenario.ephemerides, scenario.t_rx_s)
        .expect("uncorrected observations should still solve");

    let corrected_error_m = position_error_3d_m(
        corrected_solution.ecef_x_m,
        corrected_solution.ecef_y_m,
        corrected_solution.ecef_z_m,
        scenario.truth_ecef_m,
    );
    let uncorrected_error_m = position_error_3d_m(
        uncorrected_solution.ecef_x_m,
        uncorrected_solution.ecef_y_m,
        uncorrected_solution.ecef_z_m,
        scenario.truth_ecef_m,
    );

    assert!(corrected_error_m < 5.0);
    assert!(uncorrected_error_m > corrected_error_m + 2.0);
}

#[test]
fn single_point_solver_refreshes_unbiased_geometry_after_state_update() {
    let scenario = four_satellite_position_scenario(0.0);
    let solver = PositionSolver {
        max_iterations: 1,
        residual_gate_m: 1.0e9,
        chi_square_gate: 1.0e18,
        robust_weighting: PositionRobustWeighting::Disabled,
        raim: false,
        ..PositionSolver::new()
    };
    let solution = solver
        .solve_wls(&scenario.observations, &scenario.ephemerides, scenario.t_rx_s)
        .expect("single-iteration geometry should still produce a solution");

    assert_eq!(solution.used_sat_count, 4);
    for (sat, residual_m, _weight) in &solution.residuals {
        let observation = scenario
            .observations
            .iter()
            .find(|observation| observation.sat == *sat)
            .expect("scenario observation");
        let ephemeris = scenario
            .ephemerides
            .iter()
            .find(|ephemeris| ephemeris.sat == *sat)
            .expect("scenario ephemeris");
        let expected_residual_m = iterative_pseudorange_residual_m(
            ephemeris,
            observation,
            (solution.ecef_x_m, solution.ecef_y_m, solution.ecef_z_m),
            solution.clock_bias_s,
            scenario.t_rx_s,
        );
        assert!((residual_m - expected_residual_m).abs() < 1.0e-6);
    }
}

#[test]
fn single_point_solver_refreshes_biased_geometry_after_state_update() {
    let scenario = four_satellite_position_scenario(2.75e-4);
    let solver = PositionSolver {
        max_iterations: 1,
        residual_gate_m: 1.0e9,
        chi_square_gate: 1.0e18,
        robust_weighting: PositionRobustWeighting::Disabled,
        raim: false,
        ..PositionSolver::new()
    };
    let solution = solver
        .solve_wls(&scenario.observations, &scenario.ephemerides, scenario.t_rx_s)
        .expect("single-iteration biased geometry should still produce a solution");

    assert_eq!(solution.used_sat_count, 4);
    for (sat, residual_m, _weight) in &solution.residuals {
        let observation = scenario
            .observations
            .iter()
            .find(|observation| observation.sat == *sat)
            .expect("scenario observation");
        let ephemeris = scenario
            .ephemerides
            .iter()
            .find(|ephemeris| ephemeris.sat == *sat)
            .expect("scenario ephemeris");
        let expected_residual_m = iterative_pseudorange_residual_m(
            ephemeris,
            observation,
            (solution.ecef_x_m, solution.ecef_y_m, solution.ecef_z_m),
            solution.clock_bias_s,
            scenario.t_rx_s,
        );
        assert!((residual_m - expected_residual_m).abs() < 1.0e-6);
    }
}

#[test]
fn single_point_solver_residuals_include_earth_rotation_correction() {
    let scenario = four_satellite_position_scenario(2.75e-4);
    let solution = PositionSolver::new()
        .solve_wls(&scenario.observations, &scenario.ephemerides, scenario.t_rx_s)
        .expect("timed four-satellite observations should solve");

    assert_eq!(solution.used_sat_count, 4);
    for (sat, residual_m, _weight) in &solution.residuals {
        let observation = scenario
            .observations
            .iter()
            .find(|observation| observation.sat == *sat)
            .expect("scenario observation");
        let ephemeris = scenario
            .ephemerides
            .iter()
            .find(|ephemeris| ephemeris.sat == *sat)
            .expect("scenario ephemeris");
        let corrected_residual_m = iterative_pseudorange_residual_m(
            ephemeris,
            observation,
            (solution.ecef_x_m, solution.ecef_y_m, solution.ecef_z_m),
            solution.clock_bias_s,
            scenario.t_rx_s,
        );
        let uncorrected_residual_m = iterative_pseudorange_residual_without_earth_rotation_m(
            ephemeris,
            observation,
            (solution.ecef_x_m, solution.ecef_y_m, solution.ecef_z_m),
            solution.clock_bias_s,
            scenario.t_rx_s,
        );

        assert!((residual_m - corrected_residual_m).abs() < 1.0e-6);
        assert!(uncorrected_residual_m.abs() > residual_m.abs() + 1.0);
    }
}

#[test]
fn single_point_solver_uses_broadcast_satellite_clock_correction() {
    let scenario = broadcast_clock_position_scenario(2.75e-4);
    let zero_clock_ephemerides = clear_broadcast_clock_parameters(&scenario.ephemerides);

    let corrected_solution = PositionSolver::new()
        .solve_wls(&scenario.observations, &scenario.ephemerides, scenario.t_rx_s)
        .expect("broadcast clock corrected position should solve");
    let zero_clock_solution = PositionSolver::new()
        .solve_wls(&scenario.observations, &zero_clock_ephemerides, scenario.t_rx_s)
        .expect("zero-clock comparison position should still solve");

    let corrected_error_m = position_error_3d_m(
        corrected_solution.ecef_x_m,
        corrected_solution.ecef_y_m,
        corrected_solution.ecef_z_m,
        scenario.truth_ecef_m,
    );
    let zero_clock_error_m = position_error_3d_m(
        zero_clock_solution.ecef_x_m,
        zero_clock_solution.ecef_y_m,
        zero_clock_solution.ecef_z_m,
        scenario.truth_ecef_m,
    );

    assert!(corrected_error_m < 5.0);
    assert!(zero_clock_error_m > corrected_error_m + 20.0);
    assert!((corrected_solution.clock_bias_s - scenario.receiver_clock_bias_s).abs() < 1.0e-9);
}

fn decoded_lnav_subframes_from_ephemeris(eph: &GpsEphemeris) -> Vec<GpsL1CaLnavDecodedSubframe> {
    let alignment = |subframe_index: usize| GpsL1CaLnavSubframeAlignment {
        start_bit_index: subframe_index * 300,
        end_bit_index_exclusive: (subframe_index + 1) * 300,
        start_prompt_index: subframe_index * 6_000,
        end_prompt_index_exclusive: (subframe_index + 1) * 6_000,
        inverted: false,
        word_count: 10,
        parity_ok_count: 10,
    };
    let tlm = GpsL1CaTlmWord { preamble: 0x8B, parity_ok: true };
    let parity = GpsL1CaWordParitySummary {
        word_count: 10,
        passed_word_count: 10,
        failed_word_indexes: Vec::new(),
    };
    let word_parity_ok = vec![true; 10];

    vec![
        GpsL1CaLnavDecodedSubframe {
            alignment: alignment(0),
            tlm: tlm.clone(),
            how: GpsL1CaHowWord {
                tow_count: (eph.toc_s / 6.0) as u32,
                tow_start_s: (eph.toc_s / 6.0) as u32 * 6,
                alert: false,
                anti_spoof: false,
                subframe_id: 1,
                parity_ok: true,
            },
            clock: Some(GpsL1CaLnavSubframe1Clock {
                week: (eph.week % 1024) as u16,
                iodc: eph.iodc,
                sv_health: eph.sv_health,
                toc_s: eph.toc_s,
                af0: eph.af0,
                af1: eph.af1,
                af2: eph.af2,
                tgd: eph.tgd,
            }),
            orbit_subframe_2: None,
            orbit_subframe_3: None,
            parity: parity.clone(),
            word_parity_ok: word_parity_ok.clone(),
        },
        GpsL1CaLnavDecodedSubframe {
            alignment: alignment(1),
            tlm: tlm.clone(),
            how: GpsL1CaHowWord {
                tow_count: (eph.toe_s / 6.0) as u32,
                tow_start_s: (eph.toe_s / 6.0) as u32 * 6,
                alert: false,
                anti_spoof: false,
                subframe_id: 2,
                parity_ok: true,
            },
            clock: None,
            orbit_subframe_2: Some(GpsL1CaLnavSubframe2Orbit {
                iode: eph.iode,
                crs: eph.crs,
                delta_n: eph.delta_n,
                m0: eph.m0,
                cuc: eph.cuc,
                e: eph.e,
                cus: eph.cus,
                sqrt_a: eph.sqrt_a,
                toe_s: eph.toe_s,
            }),
            orbit_subframe_3: None,
            parity: parity.clone(),
            word_parity_ok: word_parity_ok.clone(),
        },
        GpsL1CaLnavDecodedSubframe {
            alignment: alignment(2),
            tlm,
            how: GpsL1CaHowWord {
                tow_count: (eph.toe_s / 6.0) as u32 + 1,
                tow_start_s: ((eph.toe_s / 6.0) as u32 + 1) * 6,
                alert: false,
                anti_spoof: false,
                subframe_id: 3,
                parity_ok: true,
            },
            clock: None,
            orbit_subframe_2: None,
            orbit_subframe_3: Some(GpsL1CaLnavSubframe3Orbit {
                iode: eph.iode,
                cic: eph.cic,
                omega0: eph.omega0,
                cis: eph.cis,
                i0: eph.i0,
                crc: eph.crc,
                w: eph.w,
                omegadot: eph.omegadot,
                idot: eph.idot,
            }),
            parity,
            word_parity_ok,
        },
    ]
}

#[test]
fn rinex_nav_ephemeris_feeds_position_solver() {
    let source = vec![
        sample_ephemeris(1, 0.0, 0.0),
        sample_ephemeris(2, 0.8, 0.9),
        sample_ephemeris(3, 1.6, 1.8),
        sample_ephemeris(4, 2.4, 2.7),
    ];
    let path = std::env::temp_dir().join(format!(
        "bijux-rinex-nav-position-{}-{}.rnx",
        std::process::id(),
        source.len()
    ));
    write_rinex_nav(&path, &source, true).expect("write rinex nav");
    let data = std::fs::read_to_string(&path).expect("read rinex nav");
    let parsed = parse_rinex_nav(&data).expect("parse rinex nav");
    std::fs::remove_file(&path).expect("remove rinex nav");

    let (rx_x, rx_y, rx_z) = geodetic_to_ecef(37.0, -122.0, 10.0);
    let t_rx_s = 504_018.07;
    let observations = parsed
        .iter()
        .map(|eph| {
            let mut tau = 0.07;
            let mut pseudorange_m = 0.0;
            for _ in 0..10 {
                let state = sat_state_gps_l1ca(eph, t_rx_s - tau, tau);
                let dx = rx_x - state.x_m;
                let dy = rx_y - state.y_m;
                let dz = rx_z - state.z_m;
                let range = (dx * dx + dy * dy + dz * dz).sqrt();
                pseudorange_m = range - state.clock_correction.bias_s * 299_792_458.0;
                let next_tau = pseudorange_m / 299_792_458.0;
                if (next_tau - tau).abs() < 1.0e-12 {
                    break;
                }
                tau = next_tau;
            }
            timed_position_observation(eph.sat, pseudorange_m, t_rx_s)
        })
        .collect::<Vec<_>>();

    let solution = PositionSolver::new()
        .solve_wls(&observations, &parsed, t_rx_s)
        .expect("parsed rinex nav should support a position solve");

    assert!((solution.ecef_x_m - rx_x).abs() < 5.0);
    assert!((solution.ecef_y_m - rx_y).abs() < 5.0);
    assert!((solution.ecef_z_m - rx_z).abs() < 5.0);
}

#[test]
fn rinex_nav_position_solver_uses_broadcast_satellite_clock_correction() {
    let scenario = broadcast_clock_position_scenario(2.75e-4);
    let corrected_path = std::env::temp_dir().join(format!(
        "bijux-rinex-nav-clock-corrected-{}-{}.rnx",
        std::process::id(),
        scenario.ephemerides.len()
    ));
    let zero_clock_path = std::env::temp_dir().join(format!(
        "bijux-rinex-nav-zero-clock-{}-{}.rnx",
        std::process::id(),
        scenario.ephemerides.len()
    ));
    write_rinex_nav(&corrected_path, &scenario.ephemerides, true)
        .expect("write corrected rinex nav");
    write_rinex_nav(
        &zero_clock_path,
        &clear_broadcast_clock_parameters(&scenario.ephemerides),
        true,
    )
    .expect("write zero-clock rinex nav");
    let corrected = parse_rinex_nav(
        &std::fs::read_to_string(&corrected_path).expect("read corrected rinex nav"),
    )
    .expect("parse corrected rinex nav");
    let zero_clock = parse_rinex_nav(
        &std::fs::read_to_string(&zero_clock_path).expect("read zero-clock rinex nav"),
    )
    .expect("parse zero-clock rinex nav");
    std::fs::remove_file(&corrected_path).expect("remove corrected rinex nav");
    std::fs::remove_file(&zero_clock_path).expect("remove zero-clock rinex nav");

    let corrected_solution = PositionSolver::new()
        .solve_wls(&scenario.observations, &corrected, scenario.t_rx_s)
        .expect("corrected rinex position should solve");
    let zero_clock_solution = PositionSolver::new()
        .solve_wls(&scenario.observations, &zero_clock, scenario.t_rx_s)
        .expect("zero-clock rinex position should still solve");

    let corrected_error_m = position_error_3d_m(
        corrected_solution.ecef_x_m,
        corrected_solution.ecef_y_m,
        corrected_solution.ecef_z_m,
        scenario.truth_ecef_m,
    );
    let zero_clock_error_m = position_error_3d_m(
        zero_clock_solution.ecef_x_m,
        zero_clock_solution.ecef_y_m,
        zero_clock_solution.ecef_z_m,
        scenario.truth_ecef_m,
    );

    assert!(corrected_error_m < 5.0);
    assert!(zero_clock_error_m > corrected_error_m + 20.0);
}

#[test]
fn rinex_nav_position_solver_residuals_include_earth_rotation_correction() {
    let scenario = four_satellite_position_scenario(2.75e-4);
    let path = std::env::temp_dir().join(format!(
        "bijux-rinex-nav-earth-rotation-{}-{}.rnx",
        std::process::id(),
        scenario.ephemerides.len()
    ));
    write_rinex_nav(&path, &scenario.ephemerides, true).expect("write rinex nav");
    let parsed = parse_rinex_nav(&std::fs::read_to_string(&path).expect("read rinex nav"))
        .expect("parse rinex nav");
    std::fs::remove_file(&path).expect("remove rinex nav");

    let solution = PositionSolver::new()
        .solve_wls(&scenario.observations, &parsed, scenario.t_rx_s)
        .expect("parsed rinex nav should support a position solve");

    assert_eq!(solution.used_sat_count, 4);
    for (sat, residual_m, _weight) in &solution.residuals {
        let observation = scenario
            .observations
            .iter()
            .find(|observation| observation.sat == *sat)
            .expect("scenario observation");
        let ephemeris =
            parsed.iter().find(|ephemeris| ephemeris.sat == *sat).expect("parsed ephemeris");
        let corrected_residual_m = iterative_pseudorange_residual_m(
            ephemeris,
            observation,
            (solution.ecef_x_m, solution.ecef_y_m, solution.ecef_z_m),
            solution.clock_bias_s,
            scenario.t_rx_s,
        );
        let uncorrected_residual_m = iterative_pseudorange_residual_without_earth_rotation_m(
            ephemeris,
            observation,
            (solution.ecef_x_m, solution.ecef_y_m, solution.ecef_z_m),
            solution.clock_bias_s,
            scenario.t_rx_s,
        );

        assert!((residual_m - corrected_residual_m).abs() < 1.0e-6);
        assert!(uncorrected_residual_m.abs() > residual_m.abs() + 1.0);
    }
}

#[test]
fn decoded_lnav_ephemeris_feeds_position_solver() {
    let source = vec![
        sample_ephemeris(1, 0.0, 0.0),
        sample_ephemeris(2, 0.8, 0.9),
        sample_ephemeris(3, 1.6, 1.8),
        sample_ephemeris(4, 2.4, 2.7),
    ];
    let parsed = source
        .iter()
        .map(|eph| {
            let (ephemerides, rejections) = ephemerides_from_decoded_gps_l1ca_lnav(
                eph.sat.prn,
                &decoded_lnav_subframes_from_ephemeris(eph),
                Some(eph.week),
            );
            assert!(rejections.is_empty(), "rejections={rejections:?}");
            assert_eq!(ephemerides.len(), 1, "ephemerides={ephemerides:?}");
            ephemerides.into_iter().next().expect("decoded ephemeris")
        })
        .collect::<Vec<_>>();

    let (rx_x, rx_y, rx_z) = geodetic_to_ecef(37.0, -122.0, 10.0);
    let t_rx_s = 504_018.07;
    let observations = parsed
        .iter()
        .map(|eph| {
            let mut tau = 0.07;
            let mut pseudorange_m = 0.0;
            for _ in 0..10 {
                let state = sat_state_gps_l1ca(eph, t_rx_s - tau, tau);
                let dx = rx_x - state.x_m;
                let dy = rx_y - state.y_m;
                let dz = rx_z - state.z_m;
                let range = (dx * dx + dy * dy + dz * dz).sqrt();
                pseudorange_m = range - state.clock_correction.bias_s * 299_792_458.0;
                let next_tau = pseudorange_m / 299_792_458.0;
                if (next_tau - tau).abs() < 1.0e-12 {
                    break;
                }
                tau = next_tau;
            }
            timed_position_observation(eph.sat, pseudorange_m, t_rx_s)
        })
        .collect::<Vec<_>>();

    let solution = PositionSolver::new()
        .solve_wls(&observations, &parsed, t_rx_s)
        .expect("decoded LNAV ephemerides should support a position solve");

    assert!((solution.ecef_x_m - rx_x).abs() < 5.0);
    assert!((solution.ecef_y_m - rx_y).abs() < 5.0);
    assert!((solution.ecef_z_m - rx_z).abs() < 5.0);
}

#[test]
fn decoded_lnav_position_solver_uses_broadcast_satellite_clock_correction() {
    let scenario = broadcast_clock_position_scenario(2.75e-4);
    let zero_clock_ephemerides = clear_broadcast_clock_parameters(&scenario.ephemerides);
    let corrected = scenario
        .ephemerides
        .iter()
        .map(|eph| {
            let (ephemerides, rejections) = ephemerides_from_decoded_gps_l1ca_lnav(
                eph.sat.prn,
                &decoded_lnav_subframes_from_ephemeris(eph),
                Some(eph.week),
            );
            assert!(rejections.is_empty(), "rejections={rejections:?}");
            assert_eq!(ephemerides.len(), 1, "ephemerides={ephemerides:?}");
            ephemerides.into_iter().next().expect("decoded corrected ephemeris")
        })
        .collect::<Vec<_>>();
    let zero_clock = zero_clock_ephemerides
        .iter()
        .map(|eph| {
            let (ephemerides, rejections) = ephemerides_from_decoded_gps_l1ca_lnav(
                eph.sat.prn,
                &decoded_lnav_subframes_from_ephemeris(eph),
                Some(eph.week),
            );
            assert!(rejections.is_empty(), "rejections={rejections:?}");
            assert_eq!(ephemerides.len(), 1, "ephemerides={ephemerides:?}");
            ephemerides.into_iter().next().expect("decoded zero-clock ephemeris")
        })
        .collect::<Vec<_>>();

    let corrected_solution = PositionSolver::new()
        .solve_wls(&scenario.observations, &corrected, scenario.t_rx_s)
        .expect("corrected decoded LNAV position should solve");
    let zero_clock_solution = PositionSolver::new()
        .solve_wls(&scenario.observations, &zero_clock, scenario.t_rx_s)
        .expect("zero-clock decoded LNAV position should still solve");

    let corrected_error_m = position_error_3d_m(
        corrected_solution.ecef_x_m,
        corrected_solution.ecef_y_m,
        corrected_solution.ecef_z_m,
        scenario.truth_ecef_m,
    );
    let zero_clock_error_m = position_error_3d_m(
        zero_clock_solution.ecef_x_m,
        zero_clock_solution.ecef_y_m,
        zero_clock_solution.ecef_z_m,
        scenario.truth_ecef_m,
    );

    assert!(corrected_error_m < 5.0);
    assert!(zero_clock_error_m > corrected_error_m + 20.0);
}

#[test]
fn decoded_lnav_position_solver_residuals_include_earth_rotation_correction() {
    let scenario = four_satellite_position_scenario(2.75e-4);
    let parsed = scenario
        .ephemerides
        .iter()
        .map(|eph| {
            let (ephemerides, rejections) = ephemerides_from_decoded_gps_l1ca_lnav(
                eph.sat.prn,
                &decoded_lnav_subframes_from_ephemeris(eph),
                Some(eph.week),
            );
            assert!(rejections.is_empty(), "rejections={rejections:?}");
            assert_eq!(ephemerides.len(), 1, "ephemerides={ephemerides:?}");
            ephemerides.into_iter().next().expect("decoded ephemeris")
        })
        .collect::<Vec<_>>();

    let solution = PositionSolver::new()
        .solve_wls(&scenario.observations, &parsed, scenario.t_rx_s)
        .expect("decoded LNAV ephemerides should support a position solve");

    assert_eq!(solution.used_sat_count, 4);
    for (sat, residual_m, _weight) in &solution.residuals {
        let observation = scenario
            .observations
            .iter()
            .find(|observation| observation.sat == *sat)
            .expect("scenario observation");
        let ephemeris =
            parsed.iter().find(|ephemeris| ephemeris.sat == *sat).expect("decoded ephemeris");
        let corrected_residual_m = iterative_pseudorange_residual_m(
            ephemeris,
            observation,
            (solution.ecef_x_m, solution.ecef_y_m, solution.ecef_z_m),
            solution.clock_bias_s,
            scenario.t_rx_s,
        );
        let uncorrected_residual_m = iterative_pseudorange_residual_without_earth_rotation_m(
            ephemeris,
            observation,
            (solution.ecef_x_m, solution.ecef_y_m, solution.ecef_z_m),
            solution.clock_bias_s,
            scenario.t_rx_s,
        );

        assert!((residual_m - corrected_residual_m).abs() < 1.0e-6);
        assert!(uncorrected_residual_m.abs() > residual_m.abs() + 1.0);
    }
}
