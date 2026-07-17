#[test]
fn write_melbourne_wubbena_diagnostics_emits_wide_lane_events() {
    let out_dir = unique_artifact_output_dir("melbourne_wubbena_output");
    fs::create_dir_all(&out_dir).expect("create output directory");

    super::write_melbourne_wubbena_diagnostics(
        &out_dir,
        &[
            dual_frequency_epoch(0, 22_000_000.0, 22_000_002.0, 21_999_999.0, 22_000_000.5),
            dual_frequency_epoch(1, 22_000_000.0, 22_000_002.0, 21_999_999.01, 22_000_000.49),
            dual_frequency_epoch(2, 22_000_000.0, 22_000_002.0, 22_000_005.5, 22_000_000.5),
        ],
    )
    .expect("write melbourne-wubbena diagnostics");

    let path = out_dir.join("melbourne_wubbena.jsonl");
    let text = fs::read_to_string(&path).expect("read melbourne-wubbena artifact");
    let rows: Vec<serde_json::Value> = text
        .lines()
        .map(|line| serde_json::from_str(line).expect("parse melbourne-wubbena row"))
        .collect();

    assert_eq!(rows.len(), 3);
    assert_eq!(rows[0]["event"], "insufficient_history");
    assert_eq!(rows[1]["event"], "nominal");
    assert_eq!(rows[2]["event"], "wide_lane_slip_suspect");
    assert!(
        rows[2]["delta_from_previous_wide_lane_cycles"].as_f64().expect("wide-lane delta") >= 0.5
    );

    fs::remove_dir_all(&out_dir).expect("remove output directory");
}

#[test]
fn write_iono_free_code_artifact_emits_bias_corrected_rows() {
    let out_dir = unique_artifact_output_dir("iono_free_code_output");
    fs::create_dir_all(&out_dir).expect("create output directory");

    let provider = fs::read_to_string(bias_sinex_fixture("gps_l1_l2_absolute_biases.bia"))
        .expect("read bias sinex fixture")
        .parse::<BiasSinexProvider>()
        .expect("parse bias sinex fixture");
    let l1 = signal_spec_gps_l1_ca();
    let l2 = signal_spec_gps_l2_py();
    let base_range_m = 20_200_000.0;
    let iono_l1_m = 4.0;
    let l1_bias_m = 3.0e-9 * SPEED_OF_LIGHT_MPS;
    let l2_bias_m = 9.0e-9 * SPEED_OF_LIGHT_MPS;
    let l1_hz = l1.carrier_hz.value();
    let l2_hz = l2.carrier_hz.value();
    let l2_iono_m = iono_l1_m * (l1_hz * l1_hz) / (l2_hz * l2_hz);
    let mut epoch = dual_frequency_epoch(
        0,
        base_range_m + iono_l1_m + l1_bias_m,
        base_range_m + l2_iono_m + l2_bias_m,
        0.0,
        0.0,
    );
    for satellite in &mut epoch.sats {
        satellite.signal_id.sat.prn = 23;
    }

    super::write_iono_free_code_artifact(&out_dir, &[epoch], Some(&provider))
        .expect("write iono-free code artifact");

    let path = out_dir.join("iono_free_code.jsonl");
    let text = fs::read_to_string(&path).expect("read iono-free code artifact");
    let rows: Vec<serde_json::Value> = text
        .lines()
        .map(|line| serde_json::from_str(line).expect("parse iono-free code row"))
        .collect();

    assert_eq!(rows.len(), 1);
    assert_eq!(rows[0]["status"], "ok");
    assert!(rows[0]["code_bias_m"].as_f64().expect("code bias").abs() > 0.1);
    assert!(
        (rows[0]["corrected_code_m"].as_f64().expect("corrected code") - base_range_m).abs()
            < 1.0e-6
    );

    fs::remove_dir_all(&out_dir).expect("remove output directory");
}

#[test]
fn write_iono_free_code_artifact_emits_supported_constellation_rows() {
    let out_dir = unique_artifact_output_dir("iono_free_code_constellations_output");
    fs::create_dir_all(&out_dir).expect("create output directory");

    let epochs = [
        dual_frequency_epoch_for_pair(
            0,
            SatId { constellation: Constellation::Galileo, prn: 19 },
            (SignalBand::E1, SignalCode::E1B, 24_000_004.0, 0.0),
            (SignalBand::E5, SignalCode::E5a, 24_000_002.5, 0.0),
        ),
        dual_frequency_epoch_for_pair(
            1,
            SatId { constellation: Constellation::Beidou, prn: 7 },
            (SignalBand::B1, SignalCode::B1I, 24_100_003.0, 0.0),
            (SignalBand::B2, SignalCode::B2I, 24_100_002.0, 0.0),
        ),
    ];

    super::write_iono_free_code_artifact(&out_dir, &epochs, None)
        .expect("write iono-free code artifact");

    let path = out_dir.join("iono_free_code.jsonl");
    let text = fs::read_to_string(&path).expect("read iono-free code artifact");
    let rows: Vec<serde_json::Value> = text
        .lines()
        .map(|line| serde_json::from_str(line).expect("parse iono-free code row"))
        .collect();

    assert_eq!(rows.len(), 2);
    assert!(rows.iter().any(|row| row["band_1"] == "E1" && row["band_2"] == "E5"));
    assert!(rows.iter().any(|row| row["band_1"] == "B1" && row["band_2"] == "B2"));
    assert!(rows.iter().all(|row| row["status"] == "ok"));

    fs::remove_dir_all(&out_dir).expect("remove output directory");
}

#[test]
fn write_narrow_lane_artifact_emits_supported_constellation_rows() {
    let out_dir = unique_artifact_output_dir("narrow_lane_output");
    fs::create_dir_all(&out_dir).expect("create output directory");

    let epochs = vec![
        dual_frequency_epoch(0, 22_000_000.0, 22_000_002.0, 21_999_999.0, 22_000_000.5),
        dual_frequency_epoch_for_pair(
            1,
            SatId { constellation: Constellation::Galileo, prn: 19 },
            (SignalBand::E1, SignalCode::E1B, 24_345_678.125, 24_345_677.0),
            (SignalBand::E5, SignalCode::E5a, 24_345_679.875, 24_345_674.5),
        ),
        dual_frequency_epoch_for_pair(
            2,
            SatId { constellation: Constellation::Beidou, prn: 7 },
            (SignalBand::B1, SignalCode::B1I, 24_345_678.125, 24_345_677.5),
            (SignalBand::B2, SignalCode::B2I, 24_345_679.875, 24_345_674.25),
        ),
    ];

    super::write_narrow_lane_artifact(&out_dir, &epochs).expect("write narrow-lane artifact");

    let path = out_dir.join("narrow_lane.jsonl");
    let text = fs::read_to_string(&path).expect("read narrow-lane artifact");
    let rows: Vec<serde_json::Value> = text
        .lines()
        .map(|line| serde_json::from_str(line).expect("parse narrow-lane row"))
        .collect();

    assert_eq!(rows.len(), 3);
    assert_eq!(rows[0]["band_1"], "L1");
    assert_eq!(rows[0]["band_2"], "L2");
    assert_eq!(rows[1]["band_1"], "E1");
    assert_eq!(rows[1]["band_2"], "E5");
    assert_eq!(rows[2]["band_1"], "B1");
    assert_eq!(rows[2]["band_2"], "B2");
    assert!(rows.iter().all(|row| row["narrow_lane_wavelength_m"].as_f64().is_some()));
    assert!(rows.iter().all(|row| row["phase_cycles"].as_f64().is_some()));

    fs::remove_dir_all(&out_dir).expect("remove output directory");
}
