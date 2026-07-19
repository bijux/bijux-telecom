#[test]
fn locked_capture_value_rejects_drift() {
    let err = enforce_locked_capture_value("sample_rate_hz", Some(4_092_000.0), 5_000_000.0)
        .expect_err("mismatch must fail");
    assert!(err.to_string().contains("sample_rate_hz"));
}

#[test]
fn raw_iq_metadata_updates_receiver_profile() {
    let mut profile = ReceiverConfig::default();
    let metadata = RawIqMetadata {
        format: bijux_gnss_infra::api::signal::IqSampleFormat::Iq16Le,
        sample_rate_hz: 5_000_000.0,
        intermediate_freq_hz: 250_000.0,
        capture_start_utc: "2026-07-09T00:00:00Z".to_string(),
        offset_bytes: 64,
        quantization_bits: Some(16),
        notes: None,
    };

    apply_raw_iq_metadata(&mut profile, &metadata, Some(5_000_000.0), Some(250_000.0))
        .expect("apply raw iq metadata");

    assert_eq!(profile.sample_rate_hz, metadata.sample_rate_hz);
    assert_eq!(profile.intermediate_freq_hz, metadata.intermediate_freq_hz);
    assert_eq!(profile.quantization_bits, 16);
}

#[test]
fn read_ephemeris_accepts_rinex_navigation_files() {
    let path = std::env::temp_dir().join(format!(
        "bijux_read_ephemeris_rinex_{}_{}.rnx",
        std::process::id(),
        SystemTime::now().duration_since(UNIX_EPOCH).expect("unix epoch").as_nanos()
    ));
    let eph = sample_ephemeris();

    write_rinex_nav(&path, std::slice::from_ref(&eph), true).expect("write rinex nav");
    let parsed = read_ephemeris(&path).expect("read rinex nav");
    fs::remove_file(&path).expect("remove rinex nav");

    assert_eq!(parsed.len(), 1);
    assert_eq!(parsed[0].sat, eph.sat);
    assert_eq!(parsed[0].week, eph.week);
    assert_eq!(parsed[0].iode, eph.iode);
    assert_eq!(parsed[0].iodc, eph.iodc);
    assert!((parsed[0].toe_s - eph.toe_s).abs() < 1.0e-9);
}

#[test]
fn read_ephemeris_accepts_noaa_nav_data_header() {
    let repo_root = Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(Path::parent)
        .expect("workspace root");
    let path = repo_root.join("datasets/recorded/gps_l1_2022_03_27_broadcast_nav.22n");

    let parsed = read_ephemeris(&path).expect("read noaa rinex nav");

    assert!(!parsed.is_empty(), "expected public NOAA NAV file to parse");
    assert_eq!(parsed[0].sat.constellation, Constellation::Gps);
}

#[test]
fn read_ephemeris_accepts_nav_decode_reports() {
    let path = std::env::temp_dir().join(format!(
        "bijux_read_ephemeris_nav_decode_{}_{}.json",
        std::process::id(),
        SystemTime::now().duration_since(UNIX_EPOCH).expect("unix epoch").as_nanos()
    ));
    let eph = sample_ephemeris();
    let report = serde_json::json!({
        "sat": eph.sat,
        "reference_week": eph.week,
        "decoded_subframes": [],
        "ephemerides": [eph.clone()]
    });

    fs::write(
        &path,
        serde_json::to_string_pretty(&report).expect("serialize nav decode report"),
    )
    .expect("write nav decode report");
    let parsed = read_ephemeris(&path).expect("read nav decode report");
    fs::remove_file(&path).expect("remove nav decode report");

    assert_eq!(parsed.len(), 1);
    assert_eq!(parsed[0].sat, eph.sat);
    assert_eq!(parsed[0].week, eph.week);
    assert_eq!(parsed[0].iode, eph.iode);
    assert_eq!(parsed[0].iodc, eph.iodc);
}

#[test]
fn read_ephemeris_accepts_nav_decode_report_arrays() {
    let path = std::env::temp_dir().join(format!(
        "bijux_read_ephemeris_nav_decode_array_{}_{}.json",
        std::process::id(),
        SystemTime::now().duration_since(UNIX_EPOCH).expect("unix epoch").as_nanos()
    ));
    let first = sample_ephemeris();
    let second = GpsEphemeris {
        sat: SatId { constellation: Constellation::Gps, prn: 9 },
        ..sample_ephemeris()
    };
    let reports = serde_json::json!([
        {
            "sat": first.sat,
            "reference_week": first.week,
            "decoded_subframes": [],
            "ephemerides": [first.clone()]
        },
        {
            "sat": second.sat,
            "reference_week": second.week,
            "decoded_subframes": [],
            "ephemerides": [second.clone()]
        }
    ]);

    fs::write(
        &path,
        serde_json::to_string_pretty(&reports).expect("serialize nav decode reports"),
    )
    .expect("write nav decode reports");
    let parsed = read_ephemeris(&path).expect("read nav decode report array");
    fs::remove_file(&path).expect("remove nav decode reports");

    assert_eq!(parsed.len(), 2);
    assert_eq!(parsed[0].sat, first.sat);
    assert_eq!(parsed[1].sat, second.sat);
}

#[test]
fn read_broadcast_navigation_data_accepts_json_navigation_payload() {
    let path = std::env::temp_dir().join(format!(
        "bijux_read_broadcast_navigation_payload_{}_{}.json",
        std::process::id(),
        SystemTime::now().duration_since(UNIX_EPOCH).expect("unix epoch").as_nanos()
    ));
    let ephemeris = sample_ephemeris();
    let klobuchar = sample_klobuchar_coefficients();
    let navigation = GpsBroadcastNavigationData {
        ephemerides: vec![ephemeris.clone()],
        klobuchar: Some(klobuchar),
    };

    fs::write(
        &path,
        serde_json::to_string_pretty(&navigation).expect("serialize navigation payload"),
    )
    .expect("write navigation payload");
    let parsed = read_broadcast_navigation_data(&path).expect("read navigation payload");
    fs::remove_file(&path).expect("remove navigation payload");

    assert_eq!(parsed.ephemerides.len(), 1);
    assert_eq!(parsed.ephemerides[0].sat, ephemeris.sat);
    assert_eq!(parsed.klobuchar, Some(klobuchar));
}

#[test]
fn read_broadcast_navigation_data_accepts_wrapped_navigation_payload() {
    let path = std::env::temp_dir().join(format!(
        "bijux_read_broadcast_navigation_wrapped_{}_{}.json",
        std::process::id(),
        SystemTime::now().duration_since(UNIX_EPOCH).expect("unix epoch").as_nanos()
    ));
    let ephemeris = sample_ephemeris();
    let klobuchar = sample_klobuchar_coefficients();
    let wrapped = bijux_gnss_infra::api::core::ArtifactV1 {
        header: ArtifactHeaderV1 {
            schema_version: ArtifactReadPolicy::LATEST,
            producer: "bijux-gnss".to_string(),
            producer_version: "test".to_string(),
            created_at_unix_ms: 0,
            git_sha: "test".to_string(),
            config_hash: "test".to_string(),
            dataset_id: None,
            toolchain: "test".to_string(),
            features: Vec::new(),
            deterministic: true,
            git_dirty: false,
        },
        payload: GpsBroadcastNavigationData {
            ephemerides: vec![ephemeris.clone()],
            klobuchar: Some(klobuchar),
        },
    };

    fs::write(
        &path,
        serde_json::to_string_pretty(&wrapped).expect("serialize wrapped navigation payload"),
    )
    .expect("write wrapped navigation payload");
    let parsed = read_broadcast_navigation_data(&path).expect("read wrapped navigation payload");
    fs::remove_file(&path).expect("remove wrapped navigation payload");

    assert_eq!(parsed.ephemerides.len(), 1);
    assert_eq!(parsed.ephemerides[0].sat, ephemeris.sat);
    assert_eq!(parsed.klobuchar, Some(klobuchar));
}

#[test]
fn read_broadcast_navigation_data_accepts_rinex_navigation_klobuchar() {
    let path = std::env::temp_dir().join(format!(
        "bijux_read_broadcast_navigation_rinex_{}_{}.rnx",
        std::process::id(),
        SystemTime::now().duration_since(UNIX_EPOCH).expect("unix epoch").as_nanos()
    ));
    let ephemeris = sample_ephemeris();
    let klobuchar = sample_klobuchar_coefficients();
    let navigation = GpsBroadcastNavigationData {
        ephemerides: vec![ephemeris.clone()],
        klobuchar: Some(klobuchar),
    };

    write_rinex_broadcast_navigation(&path, &navigation, true)
        .expect("write rinex broadcast navigation");
    let parsed = read_broadcast_navigation_data(&path).expect("read rinex broadcast navigation");
    fs::remove_file(&path).expect("remove rinex broadcast navigation");

    assert_eq!(parsed.ephemerides.len(), 1);
    assert_eq!(parsed.ephemerides[0].sat, ephemeris.sat);
    assert_eq!(parsed.klobuchar, Some(klobuchar));
}
