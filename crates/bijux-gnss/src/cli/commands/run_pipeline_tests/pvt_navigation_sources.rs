fn solve_pvt_case_with_decoded_lnav_reports(
        case_name: &str,
        ephs: &[GpsEphemeris],
        pvt_case: &SyntheticPvtCase,
    ) -> bijux_gnss_infra::api::core::NavSolutionEpoch {
        solve_pvt_case_with_decoded_lnav_reports_and_troposphere(case_name, ephs, pvt_case, true)
    }

    fn solve_pvt_case_with_decoded_lnav_reports_and_troposphere(
        case_name: &str,
        ephs: &[GpsEphemeris],
        pvt_case: &SyntheticPvtCase,
        tropo_enabled: bool,
    ) -> bijux_gnss_infra::api::core::NavSolutionEpoch {
        let root = std::env::temp_dir().join(format!(
            "bijux_pvt_{}_{}_{}",
            case_name,
            std::process::id(),
            SystemTime::now().duration_since(UNIX_EPOCH).expect("unix epoch").as_nanos()
        ));
        fs::create_dir_all(&root).expect("create test root");
        let obs_path = root.join("obs.jsonl");
        let eph_path = root.join("nav_decode_reports.json");
        fs::write(
            &obs_path,
            format!(
                "{}\n",
                serde_json::to_string(&pvt_case.obs_epoch).expect("serialize observation epoch")
            ),
        )
        .expect("write obs");
        let reports = ephs
            .iter()
            .map(|eph| {
                serde_json::json!({
                    "sat": eph.sat,
                    "reference_week": eph.week,
                    "decoded_subframes": decoded_lnav_subframes_from_ephemeris(eph),
                    "ephemerides": []
                })
            })
            .collect::<Vec<_>>();
        fs::write(
            &eph_path,
            serde_json::to_string_pretty(&reports).expect("serialize nav decode reports"),
        )
        .expect("write nav decode reports");

        let common = sample_common_args_with_troposphere(root.clone(), tropo_enabled);
        let out_dir = artifacts_dir(&common, "pvt", None).expect("artifacts dir");
        fs::create_dir_all(&out_dir).expect("create pvt artifacts dir");
        handle_pvt(GnssCommand::Pvt {
            common: common.clone(),
            obs: obs_path,
            eph: eph_path,
            ekf: false,
        })
        .expect("pvt command");

        let nav_path = out_dir.join("pvt.jsonl");
        let mut solutions = read_nav_solutions(&nav_path).expect("read nav solutions");
        fs::remove_dir_all(root).expect("remove test root");

        assert_eq!(solutions.len(), 1);
        solutions.remove(0)
    }

    fn solve_pvt_case_with_broadcast_navigation_data_and_troposphere(
        case_name: &str,
        navigation: &GpsBroadcastNavigationData,
        pvt_case: &SyntheticPvtCase,
        ekf: bool,
        tropo_enabled: bool,
    ) -> bijux_gnss_infra::api::core::NavSolutionEpoch {
        let root = std::env::temp_dir().join(format!(
            "bijux_pvt_broadcast_navigation_{}_{}_{}",
            case_name,
            std::process::id(),
            SystemTime::now().duration_since(UNIX_EPOCH).expect("unix epoch").as_nanos()
        ));
        fs::create_dir_all(&root).expect("create test root");
        let obs_path = root.join("obs.jsonl");
        let eph_path = root.join("broadcast_navigation.json");
        fs::write(
            &obs_path,
            format!(
                "{}\n",
                serde_json::to_string(&pvt_case.obs_epoch).expect("serialize observation epoch")
            ),
        )
        .expect("write obs");
        fs::write(
            &eph_path,
            serde_json::to_string_pretty(navigation).expect("serialize broadcast navigation"),
        )
        .expect("write broadcast navigation");

        let common = sample_common_args_with_troposphere(root.clone(), tropo_enabled);
        let out_dir = artifacts_dir(&common, "pvt", None).expect("artifacts dir");
        fs::create_dir_all(&out_dir).expect("create pvt artifacts dir");
        handle_pvt(GnssCommand::Pvt { common: common.clone(), obs: obs_path, eph: eph_path, ekf })
            .expect("pvt command");

        let nav_path = out_dir.join("pvt.jsonl");
        let mut solutions = read_nav_solutions(&nav_path).expect("read nav solutions");
        fs::remove_dir_all(root).expect("remove test root");

        assert_eq!(solutions.len(), 1);
        solutions.remove(0)
    }

    fn solve_pvt_case_with_rinex_broadcast_navigation_data_and_troposphere(
        case_name: &str,
        navigation: &GpsBroadcastNavigationData,
        pvt_case: &SyntheticPvtCase,
        ekf: bool,
        tropo_enabled: bool,
    ) -> bijux_gnss_infra::api::core::NavSolutionEpoch {
        let root = std::env::temp_dir().join(format!(
            "bijux_pvt_rinex_broadcast_navigation_{}_{}_{}",
            case_name,
            std::process::id(),
            SystemTime::now().duration_since(UNIX_EPOCH).expect("unix epoch").as_nanos()
        ));
        fs::create_dir_all(&root).expect("create test root");
        let obs_path = root.join("obs.jsonl");
        let eph_path = root.join("nav.rnx");
        fs::write(
            &obs_path,
            format!(
                "{}\n",
                serde_json::to_string(&pvt_case.obs_epoch).expect("serialize observation epoch")
            ),
        )
        .expect("write obs");
        write_rinex_broadcast_navigation(&eph_path, navigation, true)
            .expect("write rinex broadcast navigation");

        let common = sample_common_args_with_troposphere(root.clone(), tropo_enabled);
        let out_dir = artifacts_dir(&common, "pvt", None).expect("artifacts dir");
        fs::create_dir_all(&out_dir).expect("create pvt artifacts dir");
        handle_pvt(GnssCommand::Pvt { common: common.clone(), obs: obs_path, eph: eph_path, ekf })
            .expect("pvt command");

        let nav_path = out_dir.join("pvt.jsonl");
        let mut solutions = read_nav_solutions(&nav_path).expect("read nav solutions");
        fs::remove_dir_all(root).expect("remove test root");

        assert_eq!(solutions.len(), 1);
        solutions.remove(0)
    }
fn decoded_lnav_subframes_from_ephemeris(
        eph: &GpsEphemeris,
    ) -> Vec<GpsL1CaLnavDecodedSubframe> {
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
                    sv_accuracy: 0,
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
    fn pvt_command_accepts_rinex_navigation_input() {
        let root = std::env::temp_dir().join(format!(
            "bijux_pvt_rinex_nav_{}_{}",
            std::process::id(),
            SystemTime::now().duration_since(UNIX_EPOCH).expect("unix epoch").as_nanos()
        ));
        fs::create_dir_all(&root).expect("create test root");
        let obs_path = root.join("obs.jsonl");
        let eph_path = root.join("nav.rnx");

        let ephs = sample_ephemerides();
        let pvt_case = sample_pvt_case(&ephs, 2.75e-4);
        fs::write(
            &obs_path,
            format!("{}\n", serde_json::to_string(&pvt_case.obs_epoch).expect("serialize obs")),
        )
        .expect("write obs");
        write_rinex_nav(&eph_path, &ephs, true).expect("write rinex nav");
        let rinex = fs::read_to_string(&eph_path).expect("read rinex nav");
        assert_eq!(parse_rinex_nav(&rinex).expect("parse rinex nav").len(), ephs.len());

        let common = sample_common_args(root.clone());
        handle_pvt(GnssCommand::Pvt {
            common: common.clone(),
            obs: obs_path,
            eph: eph_path,
            ekf: false,
        })
        .expect("pvt command");

        let nav_path =
            artifacts_dir(&common, "pvt", None).expect("artifacts dir").join("pvt.jsonl");
        let solutions = read_nav_solutions(&nav_path).expect("read nav solutions");
        let raw_nav_line = fs::read_to_string(&nav_path).expect("read raw nav artifact");
        let raw_nav: serde_json::Value =
            serde_json::from_str(raw_nav_line.lines().next().unwrap_or("")).expect("parse raw nav");

        assert_eq!(solutions.len(), 1);
        assert!(solutions[0].valid);
        assert_eq!(solutions[0].sat_count, 5);
        assert_eq!(solutions[0].used_sat_count, 5);
        assert_eq!(solutions[0].rejected_sat_count, 0);
        assert!((solutions[0].ecef_x_m.0 - pvt_case.truth_ecef_m.0).abs() < 5.0);
        assert!((solutions[0].ecef_y_m.0 - pvt_case.truth_ecef_m.1).abs() < 5.0);
        assert!((solutions[0].ecef_z_m.0 - pvt_case.truth_ecef_m.2).abs() < 5.0);
        assert!(solutions[0].clock_bias_s.0.is_finite());
        assert!((solutions[0].clock_bias_s.0 - pvt_case.receiver_clock_bias_s).abs() < 1.0e-9);
        assert!(
            (solutions[0].clock_bias_m.0 - pvt_case.receiver_clock_bias_s * SPEED_OF_LIGHT_MPS)
                .abs()
                < 1.0e-6
        );
        assert!(
            (raw_nav["payload"]["clock_bias_s"].as_f64().expect("clock bias seconds")
                - pvt_case.receiver_clock_bias_s)
                .abs()
                < 1.0e-9
        );
        assert!(
            (raw_nav["payload"]["clock_bias_m"].as_f64().expect("clock bias meters")
                - pvt_case.receiver_clock_bias_s * SPEED_OF_LIGHT_MPS)
                .abs()
                < 1.0e-6
        );

        fs::remove_dir_all(root).expect("remove test root");
    }

    #[test]
    fn pvt_command_accepts_decoded_lnav_report_array() {
        let root = std::env::temp_dir().join(format!(
            "bijux_pvt_decoded_lnav_{}_{}",
            std::process::id(),
            SystemTime::now().duration_since(UNIX_EPOCH).expect("unix epoch").as_nanos()
        ));
        fs::create_dir_all(&root).expect("create test root");
        let obs_path = root.join("obs.jsonl");
        let eph_path = root.join("nav_decode_reports.json");

        let ephs = sample_ephemerides();
        let pvt_case = sample_pvt_case(&ephs, 2.75e-4);
        fs::write(
            &obs_path,
            format!("{}\n", serde_json::to_string(&pvt_case.obs_epoch).expect("serialize obs")),
        )
        .expect("write obs");
        let reports = ephs
            .iter()
            .map(|eph| {
                serde_json::json!({
                    "sat": eph.sat,
                    "reference_week": eph.week,
                    "decoded_subframes": decoded_lnav_subframes_from_ephemeris(eph),
                    "ephemerides": []
                })
            })
            .collect::<Vec<_>>();
        fs::write(
            &eph_path,
            serde_json::to_string_pretty(&reports).expect("serialize nav decode reports"),
        )
        .expect("write nav decode reports");

        let common = sample_common_args(root.clone());
        handle_pvt(GnssCommand::Pvt {
            common: common.clone(),
            obs: obs_path,
            eph: eph_path,
            ekf: false,
        })
        .expect("pvt command");

        let nav_path =
            artifacts_dir(&common, "pvt", None).expect("artifacts dir").join("pvt.jsonl");
        let solutions = read_nav_solutions(&nav_path).expect("read nav solutions");
        let raw_nav_line = fs::read_to_string(&nav_path).expect("read raw nav artifact");
        let raw_nav: serde_json::Value =
            serde_json::from_str(raw_nav_line.lines().next().unwrap_or("")).expect("parse raw nav");

        assert_eq!(solutions.len(), 1);
        assert!(solutions[0].valid);
        assert_eq!(solutions[0].sat_count, 5);
        assert_eq!(solutions[0].used_sat_count, 5);
        assert_eq!(solutions[0].rejected_sat_count, 0);
        assert!((solutions[0].ecef_x_m.0 - pvt_case.truth_ecef_m.0).abs() < 5.0);
        assert!((solutions[0].ecef_y_m.0 - pvt_case.truth_ecef_m.1).abs() < 5.0);
        assert!((solutions[0].ecef_z_m.0 - pvt_case.truth_ecef_m.2).abs() < 5.0);
        assert!(solutions[0].clock_bias_s.0.is_finite());
        assert!((solutions[0].clock_bias_s.0 - pvt_case.receiver_clock_bias_s).abs() < 1.0e-9);
        assert!(
            (solutions[0].clock_bias_m.0 - pvt_case.receiver_clock_bias_s * SPEED_OF_LIGHT_MPS)
                .abs()
                < 1.0e-6
        );
        assert!(
            (raw_nav["payload"]["clock_bias_s"].as_f64().expect("clock bias seconds")
                - pvt_case.receiver_clock_bias_s)
                .abs()
                < 1.0e-9
        );
        assert!(
            (raw_nav["payload"]["clock_bias_m"].as_f64().expect("clock bias meters")
                - pvt_case.receiver_clock_bias_s * SPEED_OF_LIGHT_MPS)
                .abs()
                < 1.0e-6
        );

        fs::remove_dir_all(root).expect("remove test root");
    }
