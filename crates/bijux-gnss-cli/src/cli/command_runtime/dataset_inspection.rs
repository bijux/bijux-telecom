pub(crate) fn inspect_dataset(
    path: &Path,
    metadata: &RawIqMetadata,
    max_samples: usize,
) -> Result<InspectReport> {
    let signal_quality = measure_signal_quality_from_raw_iq(path, metadata, max_samples)?;
    let mut source = FileSamples::open_raw_iq(path, metadata.clone())
        .with_context(|| format!("failed to open {}", path.display()))?;
    let mut total_iq = 0usize;

    let mut power_histogram = vec![0u64; 8];

    while max_samples == 0 || total_iq < max_samples {
        let frame_len = if max_samples == 0 { 4096 } else { (max_samples - total_iq).min(4096) };
        let Some(frame) = source.next_frame(frame_len)? else {
            break;
        };
        for sample in &frame.iq {
            let i = sample.re as f64;
            let q = sample.im as f64;
            let power = i * i + q * q;
            let bin = ((power.sqrt() / 0.25).min(7.0)) as usize;
            power_histogram[bin] += 1;
            total_iq += 1;
            if max_samples != 0 && total_iq >= max_samples {
                break;
            }
        }
    }

    Ok(InspectReport {
        format: format!("{:?}", metadata.format),
        sample_rate_hz: metadata.sample_rate_hz,
        intermediate_freq_hz: metadata.intermediate_freq_hz,
        capture_start_utc: metadata.capture_start_utc.clone(),
        total_samples: total_iq,
        usable_duration_s: signal_quality.usable_duration_s,
        front_end_metrics: signal_quality.front_end_metrics.clone(),
        noise_floor_db: signal_quality.estimated_noise_floor_db,
        power_histogram,
        signal_quality,
    })
}

pub(crate) fn print_inspect_table(report: &InspectReport) {
    println!(
        "Format\tSampleRate(Hz)\tIF(Hz)\tCaptureStartUtc\tSamples\tUsableDuration(s)\tIMean\tQMean\tIPower\tQPower\tIqPowerRatio\tPowerWarning\tQuadratureErrorDeg\tQuadratureWarning\tClippingPct\tClippingWarning\tCenteredRms\tZeroSignalDetected\tZeroSignalReason\tPrecisionClaimsAllowed\tPrecisionRefusal\tRms\tDcImbalance\tNoiseFloor(dB)"
    );
    println!(
        "{}\t{:.1}\t{:.1}\t{}\t{}\t{:.6}\t{:.6}\t{:.6}\t{:.6}\t{:.6}\t{:.6}\t{}\t{}\t{}\t{}\t{}\t{:.6e}\t{}\t{}\t{}\t{}\t{:.6}\t{:.6}\t{:.2}",
        report.format,
        report.sample_rate_hz,
        report.intermediate_freq_hz,
        report.capture_start_utc,
        report.total_samples,
        report.usable_duration_s,
        report.front_end_metrics.i_mean,
        report.front_end_metrics.q_mean,
        report.front_end_metrics.i_power,
        report.front_end_metrics.q_power,
        report.front_end_metrics.iq_power_ratio,
        report.front_end_metrics.power_imbalance_warning,
        format_optional_degrees(report.front_end_metrics.quadrature_error_deg),
        report.front_end_metrics.quadrature_error_warning,
        format_optional_percent(report.front_end_metrics.clipping_pct),
        report.front_end_metrics.clipping_warning,
        report.front_end_metrics.centered_rms,
        report.front_end_metrics.zero_signal_detected,
        format_optional_reason(report.front_end_metrics.zero_signal_reason.as_deref()),
        report.front_end_metrics.precision_claims_allowed,
        format_optional_reason(
            report
                .front_end_metrics
                .precision_claims_refused_reason
                .as_deref(),
        ),
        report.front_end_metrics.rms,
        report.front_end_metrics.dc_imbalance,
        report.noise_floor_db
    );
    println!("Power histogram bins: {:?}", report.power_histogram);
}

#[cfg(test)]
mod tests {
    use super::inspect_dataset;
    use crate::RawIqMetadata;
    use bijux_gnss_infra::api::signal::IqSampleFormat;
    use std::fs;
    use std::path::PathBuf;
    use std::time::{SystemTime, UNIX_EPOCH};

    fn temp_file_path(name: &str) -> PathBuf {
        let nanos = SystemTime::now().duration_since(UNIX_EPOCH).expect("unix epoch").as_nanos();
        std::env::temp_dir().join(format!("bijux_{}_{}_{}.iq", name, std::process::id(), nanos))
    }

    #[test]
    fn inspect_dataset_reports_signed_8bit_metadata() {
        let path = temp_file_path("inspect_iq8");
        fs::write(&path, [0x80u8, 0x7fu8, 0x40u8, 0xc0u8]).expect("write iq8 fixture");

        let metadata = RawIqMetadata {
            format: IqSampleFormat::Iq8,
            sample_rate_hz: 2_000_000.0,
            intermediate_freq_hz: 125_000.0,
            capture_start_utc: "2026-07-09T00:00:00Z".to_string(),
            offset_bytes: 0,
            quantization_bits: Some(8),
            notes: None,
        };
        let report = inspect_dataset(&path, &metadata, 0).expect("inspect dataset");

        assert_eq!(report.format, "Iq8");
        assert_eq!(report.sample_rate_hz, metadata.sample_rate_hz);
        assert_eq!(report.intermediate_freq_hz, metadata.intermediate_freq_hz);
        assert_eq!(report.capture_start_utc, metadata.capture_start_utc);
        assert_eq!(report.total_samples, 2);
        assert_eq!(report.front_end_metrics.sample_count, 2);
        assert_eq!(report.front_end_metrics.clipping_pct, Some(50.0));
        assert!(report.front_end_metrics.clipping_warning);
        assert!(!report.front_end_metrics.precision_claims_allowed);
        assert!(report
            .front_end_metrics
            .precision_claims_refused_reason
            .as_deref()
            .expect("precision refusal reason")
            .contains("front-end clipping 50.000% exceeds"));

        fs::remove_file(&path).expect("remove iq8 fixture");
    }

    #[test]
    fn inspect_dataset_reports_signed_16bit_metadata() {
        let path = temp_file_path("inspect_iq16");
        fs::write(&path, [0x00u8, 0x80u8, 0xffu8, 0x7fu8, 0x00u8, 0x40u8, 0x00u8, 0xc0u8])
            .expect("write iq16 fixture");

        let metadata = RawIqMetadata {
            format: IqSampleFormat::Iq16Le,
            sample_rate_hz: 2_000_000.0,
            intermediate_freq_hz: 125_000.0,
            capture_start_utc: "2026-07-09T00:00:00Z".to_string(),
            offset_bytes: 0,
            quantization_bits: Some(16),
            notes: None,
        };
        let report = inspect_dataset(&path, &metadata, 0).expect("inspect dataset");

        assert_eq!(report.format, "Iq16Le");
        assert_eq!(report.sample_rate_hz, metadata.sample_rate_hz);
        assert_eq!(report.intermediate_freq_hz, metadata.intermediate_freq_hz);
        assert_eq!(report.capture_start_utc, metadata.capture_start_utc);
        assert_eq!(report.total_samples, 2);
        assert_eq!(report.front_end_metrics.sample_count, 2);
        assert_eq!(report.front_end_metrics.clipping_pct, Some(50.0));
        assert!(report.front_end_metrics.clipping_warning);
        assert!(!report.front_end_metrics.precision_claims_allowed);
        assert!(report
            .front_end_metrics
            .precision_claims_refused_reason
            .as_deref()
            .expect("precision refusal reason")
            .contains("front-end clipping 50.000% exceeds"));

        fs::remove_file(&path).expect("remove iq16 fixture");
    }

    #[test]
    fn inspect_dataset_reports_complex_float32_metadata() {
        let path = temp_file_path("inspect_cf32");
        fs::write(
            &path,
            [
                0x00u8, 0x00u8, 0x80u8, 0xbfu8, 0x00u8, 0x00u8, 0x40u8, 0x3fu8, 0x00u8, 0x00u8,
                0x00u8, 0x3fu8, 0x00u8, 0x00u8, 0x80u8, 0xbeu8,
            ],
        )
        .expect("write cf32 fixture");

        let metadata = RawIqMetadata {
            format: IqSampleFormat::Cf32Le,
            sample_rate_hz: 2_000_000.0,
            intermediate_freq_hz: 125_000.0,
            capture_start_utc: "2026-07-09T00:00:00Z".to_string(),
            offset_bytes: 0,
            quantization_bits: Some(32),
            notes: None,
        };
        let report = inspect_dataset(&path, &metadata, 0).expect("inspect dataset");

        assert_eq!(report.format, "Cf32Le");
        assert_eq!(report.sample_rate_hz, metadata.sample_rate_hz);
        assert_eq!(report.intermediate_freq_hz, metadata.intermediate_freq_hz);
        assert_eq!(report.capture_start_utc, metadata.capture_start_utc);
        assert_eq!(report.total_samples, 2);
        assert_eq!(report.front_end_metrics.sample_count, 2);
        assert_eq!(report.front_end_metrics.clipping_pct, None);
        assert!(!report.front_end_metrics.clipping_warning);
        assert!(report.front_end_metrics.precision_claims_allowed);
        assert_eq!(report.front_end_metrics.precision_claims_refused_reason, None);
        assert!(
            report.front_end_metrics.centered_rms > 0.0,
            "centered_rms={}",
            report.front_end_metrics.centered_rms
        );
        assert!(!report.front_end_metrics.zero_signal_detected);
        assert_eq!(report.front_end_metrics.zero_signal_reason, None);

        fs::remove_file(&path).expect("remove cf32 fixture");
    }

    #[test]
    fn inspect_dataset_reports_iq_power_imbalance_warning() {
        let path = temp_file_path("inspect_iq8_power_warning");
        fs::write(&path, [32u8, 0u8, 32u8, 0u8]).expect("write iq8 fixture");

        let metadata = RawIqMetadata {
            format: IqSampleFormat::Iq8,
            sample_rate_hz: 2_000_000.0,
            intermediate_freq_hz: 0.0,
            capture_start_utc: "2026-07-09T00:00:00Z".to_string(),
            offset_bytes: 0,
            quantization_bits: Some(8),
            notes: None,
        };
        let report = inspect_dataset(&path, &metadata, 0).expect("inspect dataset");

        assert_eq!(report.front_end_metrics.sample_count, 2);
        assert_eq!(report.front_end_metrics.i_power, 0.0625);
        assert_eq!(report.front_end_metrics.q_power, 0.0);
        assert!(report.front_end_metrics.iq_power_ratio > 1.0e10);
        assert!(report.front_end_metrics.power_imbalance_warning);
        assert_eq!(report.front_end_metrics.clipping_pct, Some(0.0));
        assert!(!report.front_end_metrics.clipping_warning);
        assert_eq!(report.front_end_metrics.centered_rms, 0.0);
        assert!(report.front_end_metrics.zero_signal_detected);
        assert!(report
            .front_end_metrics
            .zero_signal_reason
            .as_deref()
            .expect("zero_signal_reason")
            .contains("no varying signal energy"));
        assert!(!report.front_end_metrics.precision_claims_allowed);
        assert!(report
            .front_end_metrics
            .precision_claims_refused_reason
            .as_deref()
            .expect("precision_claims_refused_reason")
            .contains("no varying signal energy"));

        fs::remove_file(&path).expect("remove iq8 fixture");
    }
}
