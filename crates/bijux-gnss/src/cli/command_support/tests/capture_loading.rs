#[test]
fn load_frame_removes_dc_offset_when_enabled() {
    let path = unique_iq_fixture_path("load_frame_dc_removal");
    let sample_count = 4_092usize;
    let mut raw = Vec::with_capacity(sample_count * 2);
    for _ in 0..sample_count {
        raw.push(64u8);
        raw.push(0u8);
    }
    fs::write(&path, raw).expect("write iq8 fixture");

    let metadata = RawIqMetadata {
        format: bijux_gnss_infra::api::signal::IqSampleFormat::Iq8,
        sample_rate_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        capture_start_utc: "2026-07-09T00:00:00Z".to_string(),
        offset_bytes: 0,
        quantization_bits: Some(8),
        notes: None,
    };
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: metadata.sample_rate_hz,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        remove_dc_offset: true,
        ..ReceiverPipelineConfig::default()
    };

    let frame = load_frame_window(&path, &config, &metadata, 1).expect("load frame");
    let metrics = bijux_gnss_infra::api::signal::measure_iq_front_end_metrics(&frame.iq);

    assert!(metrics.i_mean.abs() < 1e-6, "i_mean={}", metrics.i_mean);
    assert!(metrics.q_mean.abs() < 1e-6, "q_mean={}", metrics.q_mean);
    assert_eq!(metrics.i_power, 0.0);
    assert_eq!(metrics.q_power, 0.0);
    assert_eq!(metrics.iq_power_ratio, 1.0);
    assert!(!metrics.power_imbalance_warning);
    assert_eq!(metrics.quadrature_error_deg, None);
    assert!(!metrics.quadrature_error_warning);
    assert_eq!(metrics.centered_rms, 0.0);
    assert!(metrics.zero_signal_detected);
    assert!(metrics
        .zero_signal_reason
        .as_deref()
        .expect("zero_signal_reason")
        .contains("no varying signal energy"));
    assert!(!metrics.precision_claims_allowed);
    assert!(metrics
        .precision_claims_refused_reason
        .as_deref()
        .expect("precision_claims_refused_reason")
        .contains("no varying signal energy"));

    fs::remove_file(&path).expect("remove iq8 fixture");
}

#[test]
fn load_tracking_frame_reads_multiple_code_periods() {
    let path = unique_iq_fixture_path("load_tracking_frame_window");
    let sample_count = 4_092usize * super::TRACKING_HISTORY_CODE_PERIODS;
    let mut raw = Vec::with_capacity(sample_count * 2);
    for _ in 0..sample_count {
        raw.push(16u8);
        raw.push(0u8);
    }
    fs::write(&path, raw).expect("write iq8 fixture");

    let metadata = RawIqMetadata {
        format: bijux_gnss_infra::api::signal::IqSampleFormat::Iq8,
        sample_rate_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        capture_start_utc: "2026-07-09T00:00:00Z".to_string(),
        offset_bytes: 0,
        quantization_bits: Some(8),
        notes: None,
    };
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: metadata.sample_rate_hz,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };

    let frame =
        super::load_tracking_frame(&path, &config, &metadata).expect("load tracking frame");
    assert_eq!(frame.len(), sample_count);

    fs::remove_file(&path).expect("remove iq8 fixture");
}

#[test]
fn read_tracking_dump_accepts_wrapped_track_artifacts() {
    let path = unique_iq_fixture_path("read_tracking_dump_artifact");
    let wrapped = TrackEpochV1 {
        header: ArtifactHeaderV1 {
            schema_version: 1,
            producer: "test".to_string(),
            producer_version: "0.1.0".to_string(),
            created_at_unix_ms: 0,
            git_sha: "test".to_string(),
            config_hash: "test".to_string(),
            dataset_id: None,
            toolchain: "rustc test".to_string(),
            features: Vec::new(),
            deterministic: true,
            git_dirty: false,
        },
        payload: TrackEpoch {
            epoch: Epoch { index: 7 },
            sample_index: 28_644,
            source_time: ReceiverSampleTrace::from_sample_index(28_644, 4_092_000.0),
            sat: SatId { constellation: Constellation::Gps, prn: 12 },
            signal_band: SignalBand::L1,
            signal_code: bijux_gnss_infra::api::core::SignalCode::Unknown,
            glonass_frequency_channel: None,
            prompt_i: -12.0,
            prompt_q: 3.0,
            early_i: -8.0,
            early_q: 1.0,
            late_i: -7.0,
            late_q: 2.0,
            carrier_hz: Hertz(120.0),
            carrier_phase_cycles: Cycles(0.25),
            code_rate_hz: Hertz(1_023_000.0),
            code_phase_samples: Chips(144.0),
            lock: true,
            cn0_dbhz: 45.0,
            pll_lock: true,
            dll_lock: true,
            fll_lock: true,
            cycle_slip: false,
            nav_bit_lock: true,
            navigation_bit_sign: Some(-1),
            transmit_time: None,
            dll_err: 0.0,
            pll_err: 0.0,
            fll_err: 0.0,
            anti_false_lock: false,
            cycle_slip_reason: None,
            lock_state: "tracking".to_string(),
            lock_state_reason: None,
            channel_id: Some(0),
            channel_uid: "Gps-12-ch00".to_string(),
            tracking_provenance: "test".to_string(),
            tracking_assumptions: None,
            signal_delay_alignment: None,
            tracking_uncertainty: None,
            processing_ms: None,
        },
    };
    fs::write(&path, serde_json::to_string(&wrapped).expect("serialize track artifact"))
        .expect("write artifact");

    let rows = read_tracking_dump(&path).expect("read tracking dump");

    assert_eq!(rows.len(), 1);
    assert_eq!(rows[0].epoch_idx, 7);
    assert_eq!(rows[0].sat.prn, 12);
    assert_eq!(rows[0].navigation_bit_sign, Some(-1));

    fs::remove_file(&path).expect("remove track artifact");
}

#[test]
fn read_tracking_dump_accepts_track_report_json() {
    let path = unique_iq_fixture_path("read_tracking_dump_report");
    let report = TrackingReport {
        sats: vec![SatId { constellation: Constellation::Gps, prn: 12 }],
        doppler_search: DopplerSearchSettings {
            max_search_hz: 5_000,
            bin_width_hz: 500,
            bin_count: 21,
            intermediate_freq_hz: 0.0,
        },
        front_end_metrics: sample_front_end_metrics(),
        signal_quality: RawIqSignalQualityReport {
            format: "iq8".to_string(),
            sample_rate_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            capture_start_utc: "2026-07-10T00:00:00Z".to_string(),
            analyzed_samples: 4_092,
            usable_duration_s: 0.001,
            estimated_noise_floor_db: -30.0,
            front_end_metrics: sample_front_end_metrics(),
        },
        epochs: vec![TrackingRow {
            epoch_idx: 3,
            sample_index: 12_276,
            sat: SatId { constellation: Constellation::Gps, prn: 12 },
            carrier_hz: 120.0,
            carrier_phase_cycles: 0.5,
            code_rate_hz: 1_023_000.0,
            code_phase_samples: 144.0,
            prompt_i: 4.0,
            prompt_q: 2.0,
            early_i: 0.0,
            early_q: 0.0,
            late_i: 0.0,
            late_q: 0.0,
            lock: true,
            cn0_dbhz: 45.0,
            pll_lock: true,
            dll_lock: true,
            fll_lock: true,
            cycle_slip: false,
            nav_bit_lock: true,
            navigation_bit_sign: Some(1),
            dll_err: 0.0,
            pll_err: 0.0,
            fll_err: 0.0,
            anti_false_lock: false,
            cycle_slip_reason: None,
            lock_state: "tracking".to_string(),
            lock_state_reason: None,
        }],
    };
    fs::write(&path, serde_json::to_string_pretty(&report).expect("serialize track report"))
        .expect("write report");

    let rows = read_tracking_dump(&path).expect("read tracking report");

    assert_eq!(rows.len(), 1);
    assert_eq!(rows[0].epoch_idx, 3);
    assert_eq!(rows[0].navigation_bit_sign, Some(1));

    fs::remove_file(&path).expect("remove track report");
}

#[test]
fn load_acquisition_frame_reads_configured_integration_window() {
    let path = unique_iq_fixture_path("load_acquisition_frame_window");
    let coherent_ms = 5u32;
    let noncoherent = 4u32;
    let sample_count = 4_092usize * super::acquisition_code_periods(coherent_ms, noncoherent);
    let mut raw = Vec::with_capacity(sample_count * 2);
    for _ in 0..sample_count {
        raw.push(8u8);
        raw.push(0u8);
    }
    fs::write(&path, raw).expect("write iq8 fixture");

    let metadata = RawIqMetadata {
        format: bijux_gnss_infra::api::signal::IqSampleFormat::Iq8,
        sample_rate_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        capture_start_utc: "2026-07-09T00:00:00Z".to_string(),
        offset_bytes: 0,
        quantization_bits: Some(8),
        notes: None,
    };
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: metadata.sample_rate_hz,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        acquisition_integration_ms: coherent_ms,
        acquisition_noncoherent: noncoherent,
        ..ReceiverPipelineConfig::default()
    };

    let frame =
        super::load_acquisition_frame(&path, &config, &metadata).expect("load acquisition frame");
    assert_eq!(frame.len(), sample_count);

    fs::remove_file(&path).expect("remove iq8 fixture");
}
