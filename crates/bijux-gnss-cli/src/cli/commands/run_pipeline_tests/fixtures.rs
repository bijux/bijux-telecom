    use super::*;
    use bijux_gnss_infra::api::core::{
        Constellation, GpsTime, LockFlags, ObsEpoch, ObsMetadata, ObsSatellite, ObsSignalTiming,
        ObservationEpochDecision, ObservationStatus, ReceiverRole, ReceiverSampleTrace, SatId,
        Seconds, SigId, SignalBand, SignalCode,
    };
    use bijux_gnss_infra::api::nav::{
        ecef_to_geodetic, elevation_azimuth_deg, parse_rinex_nav, sat_state_gps_l1ca,
        write_rinex_broadcast_navigation, write_rinex_nav, GpsBroadcastNavigationData,
        GpsEphemeris, GpsL1CaHowWord, GpsL1CaLnavDecodedSubframe, GpsL1CaLnavSubframe1Clock,
        GpsL1CaLnavSubframe2Orbit, GpsL1CaLnavSubframe3Orbit, GpsL1CaLnavSubframeAlignment,
        GpsL1CaTlmWord, GpsL1CaWordParitySummary, IonosphereModel, KlobucharCoefficients,
        KlobucharModel, SaastamoinenModel, TroposphereModel,
    };
    use bijux_gnss_infra::api::receiver::sim::{SyntheticScenario, SyntheticSignalParams};
    use bijux_gnss_signal::api::signal_spec_gps_l1_ca;
    use std::collections::BTreeMap;
    use std::fs;
    use std::path::{Path, PathBuf};
    use std::time::{SystemTime, UNIX_EPOCH};

    const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;
    const SYNTHETIC_PVT_PSEUDORANGE_SIGMA_M: f64 = 2.0;
    const SYNTHETIC_PVT_CARRIER_PHASE_SIGMA_CYCLES: f64 = 0.1;
    const SYNTHETIC_PVT_DOPPLER_SIGMA_HZ: f64 = 2.0;

    struct SyntheticPvtCase {
        obs_epoch: ObsEpoch,
        truth_ecef_m: (f64, f64, f64),
        receiver_clock_bias_s: f64,
    }
    #[derive(Debug, Clone, Copy, Default)]
    struct BroadcastClockParameters {
        af0_s: f64,
        af1_s_per_s: f64,
        af2_s_per_s2: f64,
        tgd_s: f64,
    }

    #[derive(Debug, Clone, Copy)]
    struct SyntheticSatelliteAdjustment {
        pseudorange_bias_m: f64,
        pseudorange_sigma_m: f64,
        cn0_dbhz: Option<f64>,
        elevation_deg: Option<f64>,
    }

    impl Default for SyntheticSatelliteAdjustment {
        fn default() -> Self {
            Self {
                pseudorange_bias_m: 0.0,
                pseudorange_sigma_m: SYNTHETIC_PVT_PSEUDORANGE_SIGMA_M,
                cn0_dbhz: None,
                elevation_deg: None,
            }
        }
    }

    fn sample_common_args(out: PathBuf) -> CommonArgs {
        CommonArgs {
            config: None,
            dataset: None,
            unregistered_dataset: true,
            out: Some(out),
            report: ReportFormat::Json,
            seed: None,
            deterministic: true,
            dump: None,
            sidecar: None,
            resume: None,
        }
    }

    fn sample_common_args_with_troposphere(out: PathBuf, tropo_enabled: bool) -> CommonArgs {
        let mut common = sample_common_args(out.clone());
        if tropo_enabled == ReceiverConfig::default().navigation.tropo_enable {
            return common;
        }
        common.config = Some(write_receiver_config(&out, |profile| {
            profile.navigation.tropo_enable = tropo_enabled;
        }));
        common
    }

    fn sample_common_args_with_profile(
        out: PathBuf,
        configure: impl FnOnce(&mut ReceiverConfig),
    ) -> CommonArgs {
        let mut common = sample_common_args(out.clone());
        common.config = Some(write_receiver_config(&out, configure));
        common
    }

    fn write_receiver_config(root: &Path, configure: impl FnOnce(&mut ReceiverConfig)) -> PathBuf {
        let path = root.join("receiver.toml");
        let mut profile = ReceiverConfig::default();
        configure(&mut profile);
        fs::write(&path, toml::to_string_pretty(&profile).expect("serialize receiver config"))
            .expect("write receiver config");
        path
    }

    fn sample_ephemerides() -> Vec<GpsEphemeris> {
        sample_ephemerides_with_clock_parameters(&[])
    }

    fn sample_ephemerides_with_clock_parameters(
        clock_parameters_by_prn: &[(u8, BroadcastClockParameters)],
    ) -> Vec<GpsEphemeris> {
        let clock_parameters_by_prn =
            clock_parameters_by_prn.iter().copied().collect::<BTreeMap<_, _>>();
        [(1, 0.0, 0.0), (2, 0.8, 0.9), (3, 1.6, 1.8), (4, 2.4, 2.7), (5, 3.2, 3.6)]
            .into_iter()
            .map(|(prn, omega0, m0)| {
                let clock_parameters =
                    clock_parameters_by_prn.get(&prn).copied().unwrap_or_default();
                GpsEphemeris {
                    sat: SatId { constellation: Constellation::Gps, prn },
                    iodc: 1,
                    iode: 1,
                    week: 2209,
                    sv_health: 0,
                    sv_accuracy: Some(2),
                    toe_s: 504_000.0,
                    toc_s: 504_018.0,
                    sqrt_a: 5153.7954775,
                    e: 0.01,
                    i0: 0.94,
                    idot: 0.0,
                    omega0,
                    omegadot: 0.0,
                    w: 0.0,
                    m0,
                    delta_n: 0.0,
                    cuc: 0.0,
                    cus: 0.0,
                    crc: 0.0,
                    crs: 0.0,
                    cic: 0.0,
                    cis: 0.0,
                    af0: clock_parameters.af0_s,
                    af1: clock_parameters.af1_s_per_s,
                    af2: clock_parameters.af2_s_per_s2,
                    tgd: clock_parameters.tgd_s,
                }
            })
            .collect()
    }

    fn clear_broadcast_clock_parameters(ephemerides: &[GpsEphemeris]) -> Vec<GpsEphemeris> {
        ephemerides
            .iter()
            .cloned()
            .map(|mut ephemeris| {
                ephemeris.af0 = 0.0;
                ephemeris.af1 = 0.0;
                ephemeris.af2 = 0.0;
                ephemeris.tgd = 0.0;
                ephemeris
            })
            .collect()
    }

    fn sample_klobuchar_coefficients() -> KlobucharCoefficients {
        KlobucharCoefficients::new(
            [0.1212e-7, 0.1490e-7, -0.5960e-7, 0.1192e-6],
            [0.1167e6, -0.2294e6, -0.1311e6, 0.1049e7],
        )
    }

    fn broadcast_clock_fixture_parameters() -> [(u8, BroadcastClockParameters); 5] {
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
            (
                5,
                BroadcastClockParameters {
                    af0_s: 210.0e-9,
                    af1_s_per_s: 0.0,
                    af2_s_per_s2: 0.0,
                    tgd_s: -5.0e-9,
                },
            ),
        ]
    }

    fn broadcast_clock_pvt_case(
        receiver_clock_bias_s: f64,
    ) -> (Vec<GpsEphemeris>, SyntheticPvtCase) {
        let ephemerides =
            sample_ephemerides_with_clock_parameters(&broadcast_clock_fixture_parameters());
        let pvt_case = sample_pvt_case(&ephemerides, receiver_clock_bias_s);
        (ephemerides, pvt_case)
    }

    fn sample_pvt_case(ephs: &[GpsEphemeris], receiver_clock_bias_s: f64) -> SyntheticPvtCase {
        sample_pvt_case_with_adjustments(ephs, receiver_clock_bias_s, &[])
    }

    fn sample_pvt_case_with_adjustments(
        ephs: &[GpsEphemeris],
        receiver_clock_bias_s: f64,
        adjustments: &[(u8, SyntheticSatelliteAdjustment)],
    ) -> SyntheticPvtCase {
        add_saastamoinen_delay_to_pvt_case(
            &sample_troposphere_free_pvt_case_with_adjustments(
                ephs,
                receiver_clock_bias_s,
                adjustments,
            ),
            ephs,
        )
    }

    fn sample_troposphere_free_pvt_case(
        ephs: &[GpsEphemeris],
        receiver_clock_bias_s: f64,
    ) -> SyntheticPvtCase {
        sample_troposphere_free_pvt_case_with_adjustments(ephs, receiver_clock_bias_s, &[])
    }

    fn sample_troposphere_free_pvt_case_with_adjustments(
        ephs: &[GpsEphemeris],
        receiver_clock_bias_s: f64,
        adjustments: &[(u8, SyntheticSatelliteAdjustment)],
    ) -> SyntheticPvtCase {
        let t_rx_s = 504_018.07 + receiver_clock_bias_s;
        let (rx_x, rx_y, rx_z) = bijux_gnss_infra::api::nav::geodetic_to_ecef(37.0, -122.0, 10.0);
        let adjustments_by_prn =
            adjustments.iter().copied().collect::<BTreeMap<u8, SyntheticSatelliteAdjustment>>();
        let sats = ephs
            .iter()
            .map(|eph| {
                let adjustment = adjustments_by_prn.get(&eph.sat.prn).copied().unwrap_or_default();
                let mut tau = 0.07;
                let mut pseudorange_m = 0.0;
                for _ in 0..10 {
                    let state = sat_state_gps_l1ca(eph, t_rx_s - tau, tau);
                    let dx = rx_x - state.x_m;
                    let dy = rx_y - state.y_m;
                    let dz = rx_z - state.z_m;
                    let range = (dx * dx + dy * dy + dz * dz).sqrt();
                    pseudorange_m = range + receiver_clock_bias_s * SPEED_OF_LIGHT_MPS
                        - state.clock_correction.bias_s * SPEED_OF_LIGHT_MPS;
                    let next_tau = pseudorange_m / SPEED_OF_LIGHT_MPS;
                    if (next_tau - tau).abs() < 1.0e-12 {
                        break;
                    }
                    tau = next_tau;
                }
                pseudorange_m += adjustment.pseudorange_bias_m;
                let signal_travel_time_s = pseudorange_m / SPEED_OF_LIGHT_MPS;
                ObsSatellite {
                    signal_id: SigId { sat: eph.sat, band: SignalBand::L1, code: SignalCode::Ca },
                    pseudorange_m: bijux_gnss_infra::api::core::Meters(pseudorange_m),
                    pseudorange_var_m2: adjustment.pseudorange_sigma_m.powi(2),
                    carrier_phase_cycles: bijux_gnss_infra::api::core::Cycles(
                        1_000.0 + eph.sat.prn as f64,
                    ),
                    carrier_phase_var_cycles2: SYNTHETIC_PVT_CARRIER_PHASE_SIGMA_CYCLES.powi(2),
                    doppler_hz: bijux_gnss_infra::api::core::Hertz(-500.0),
                    doppler_var_hz2: SYNTHETIC_PVT_DOPPLER_SIGMA_HZ.powi(2),
                    cn0_dbhz: adjustment.cn0_dbhz.unwrap_or(45.0),
                    lock_flags: LockFlags {
                        code_lock: true,
                        carrier_lock: true,
                        bit_lock: false,
                        cycle_slip: false,
                    },
                    multipath_suspect: false,
                    observation_status: ObservationStatus::Accepted,
                    observation_reject_reasons: Vec::new(),
                    elevation_deg: adjustment.elevation_deg,
                    azimuth_deg: None,
                    weight: None,
                    timing: Some(ObsSignalTiming {
                        signal_travel_time_s: Seconds(signal_travel_time_s),
                        transmit_gps_time: GpsTime {
                            week: 2209,
                            tow_s: t_rx_s - signal_travel_time_s,
                        },
                    }),
                    error_model: None,
                    metadata: ObsMetadata {
                        tracking_mode: "test".to_string(),
                        integration_ms: 1,
                        lock_quality: 45.0,
                        smoothing_window: 0,
                        smoothing_age: 0,
                        smoothing_resets: 0,
                        signal: signal_spec_gps_l1_ca(),
                        ..ObsMetadata::default()
                    },
                }
            })
            .collect();

        SyntheticPvtCase {
            obs_epoch: ObsEpoch {
                t_rx_s: Seconds(t_rx_s),
                source_time: ReceiverSampleTrace::from_sample_index(1, 1_000.0),
                gps_week: Some(2209),
                tow_s: Some(Seconds(t_rx_s)),
                epoch_idx: 1,
                discontinuity: false,
                valid: true,
                processing_ms: None,
                role: ReceiverRole::Rover,
                sats,
                decision: ObservationEpochDecision::Accepted,
                decision_reason: Some("accepted_observables_present".to_string()),
                manifest: None,
            },
            truth_ecef_m: (rx_x, rx_y, rx_z),
            receiver_clock_bias_s,
        }
    }

    fn sample_experiment_scenario() -> SyntheticScenario {
        SyntheticScenario {
            sample_rate_hz: 1_023_000.0,
            intermediate_freq_hz: 0.0,
            receiver_clock_frequency_bias_hz: 0.0,
            duration_s: 0.04,
            seed: 1234,
            satellites: vec![
                SyntheticSignalParams {
                    sat: SatId { constellation: Constellation::Gps, prn: 3 },
                    glonass_frequency_channel: None,
                    signal_band: SignalBand::L1,
                    signal_code: SignalCode::Ca,
                    doppler_hz: 500.0,
                    code_phase_chips: 200.0,
                    carrier_phase_rad: 0.0,
                    cn0_db_hz: 50.0,
                    navigation_data: false.into(),
                },
                SyntheticSignalParams {
                    sat: SatId { constellation: Constellation::Gps, prn: 7 },
                    glonass_frequency_channel: None,
                    signal_band: SignalBand::L1,
                    signal_code: SignalCode::Ca,
                    doppler_hz: -1_000.0,
                    code_phase_chips: 321.0,
                    carrier_phase_rad: 0.2,
                    cn0_db_hz: 50.0,
                    navigation_data: false.into(),
                },
            ],
            ephemerides: Vec::new(),
            id: "canonical_receiver_experiment".to_string(),
        }
    }
