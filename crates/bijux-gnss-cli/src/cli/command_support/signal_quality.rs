use super::*;

const SIGNAL_QUALITY_NOISE_FLOOR_BINS: usize = 256;
const SIGNAL_QUALITY_MAX_CENTERED_POWER: f64 = 4.0;
const SIGNAL_QUALITY_NOISE_FLOOR_PERCENTILE: f64 = 0.2;
const SIGNAL_QUALITY_NOISE_FLOOR_POWER_EPSILON: f64 = 1e-12;

pub(crate) fn build_signal_quality_report(
    metadata: &RawIqMetadata,
    front_end_metrics: bijux_gnss_infra::api::signal::IqFrontEndMetrics,
    estimated_noise_floor_db: f64,
) -> RawIqSignalQualityReport {
    RawIqSignalQualityReport {
        format: format!("{:?}", metadata.format),
        sample_rate_hz: metadata.sample_rate_hz,
        intermediate_freq_hz: metadata.intermediate_freq_hz,
        capture_start_utc: metadata.capture_start_utc.clone(),
        analyzed_samples: front_end_metrics.sample_count,
        usable_duration_s: front_end_metrics.sample_count as f64 / metadata.sample_rate_hz,
        estimated_noise_floor_db,
        front_end_metrics,
    }
}

pub(crate) fn measure_signal_quality_from_samples(
    metadata: &RawIqMetadata,
    samples: &[bijux_gnss_infra::api::core::Sample],
) -> RawIqSignalQualityReport {
    let front_end_metrics =
        bijux_gnss_infra::api::signal::measure_raw_iq_front_end_metrics(samples, metadata);
    let estimated_noise_floor_db =
        bijux_gnss_infra::api::signal::estimate_iq_noise_floor_db_from_metrics(
            samples,
            &front_end_metrics,
        );
    build_signal_quality_report(metadata, front_end_metrics, estimated_noise_floor_db)
}

pub(crate) fn measure_signal_quality_from_raw_iq(
    path: &Path,
    metadata: &RawIqMetadata,
    max_samples: usize,
) -> Result<RawIqSignalQualityReport> {
    let mut source = FileSamples::open_raw_iq(path, metadata.clone())
        .with_context(|| format!("failed to open {}", path.display()))?;
    let mut analyzer =
        bijux_gnss_infra::api::signal::IqFrontEndAnalyzer::for_raw_iq_metadata(metadata);
    let mut analyzed_samples = 0usize;
    while max_samples == 0 || analyzed_samples < max_samples {
        let frame_len =
            if max_samples == 0 { 4096 } else { (max_samples - analyzed_samples).min(4096) };
        let Some(frame) = source.next_frame(frame_len)? else {
            break;
        };
        analyzer.update(&frame.iq);
        analyzed_samples += frame.iq.len().min(frame_len);
    }
    let front_end_metrics = analyzer.finish();
    let estimated_noise_floor_db =
        estimate_noise_floor_from_raw_iq(path, metadata, max_samples, &front_end_metrics)?;
    Ok(build_signal_quality_report(metadata, front_end_metrics, estimated_noise_floor_db))
}

pub(crate) fn estimate_noise_floor_from_raw_iq(
    path: &Path,
    metadata: &RawIqMetadata,
    max_samples: usize,
    front_end_metrics: &bijux_gnss_infra::api::signal::IqFrontEndMetrics,
) -> Result<f64> {
    if front_end_metrics.sample_count == 0 {
        return Ok(10.0 * SIGNAL_QUALITY_NOISE_FLOOR_POWER_EPSILON.log10());
    }
    let mut source = FileSamples::open_raw_iq(path, metadata.clone())
        .with_context(|| format!("failed to open {}", path.display()))?;
    let mut histogram = [0u64; SIGNAL_QUALITY_NOISE_FLOOR_BINS];
    let mut analyzed_samples = 0usize;
    while max_samples == 0 || analyzed_samples < max_samples {
        let frame_len =
            if max_samples == 0 { 4096 } else { (max_samples - analyzed_samples).min(4096) };
        let Some(frame) = source.next_frame(frame_len)? else {
            break;
        };
        for sample in &frame.iq {
            let centered_i = sample.re as f64 - front_end_metrics.i_mean;
            let centered_q = sample.im as f64 - front_end_metrics.q_mean;
            let centered_power = centered_i * centered_i + centered_q * centered_q;
            let bin = noise_floor_histogram_bin(centered_power);
            histogram[bin] += 1;
            analyzed_samples += 1;
            if max_samples != 0 && analyzed_samples >= max_samples {
                break;
            }
        }
    }
    let percentile_power = histogram_percentile_power(
        &histogram,
        analyzed_samples,
        SIGNAL_QUALITY_NOISE_FLOOR_PERCENTILE,
    );
    let percentile_scale = -((1.0 - SIGNAL_QUALITY_NOISE_FLOOR_PERCENTILE).ln());
    let noise_floor_power =
        (percentile_power / percentile_scale).max(SIGNAL_QUALITY_NOISE_FLOOR_POWER_EPSILON);
    Ok(10.0 * noise_floor_power.log10())
}

pub(crate) fn noise_floor_histogram_bin(centered_power: f64) -> usize {
    if centered_power <= 0.0 {
        return 0;
    }
    if centered_power >= SIGNAL_QUALITY_MAX_CENTERED_POWER {
        return SIGNAL_QUALITY_NOISE_FLOOR_BINS - 1;
    }
    let normalized = centered_power / SIGNAL_QUALITY_MAX_CENTERED_POWER;
    (normalized * (SIGNAL_QUALITY_NOISE_FLOOR_BINS - 1) as f64).floor() as usize
}

pub(crate) fn histogram_percentile_power(
    histogram: &[u64],
    sample_count: usize,
    percentile: f64,
) -> f64 {
    if sample_count == 0 {
        return SIGNAL_QUALITY_NOISE_FLOOR_POWER_EPSILON;
    }
    let target_rank =
        ((sample_count.saturating_sub(1)) as f64 * percentile.clamp(0.0, 1.0)).floor() as u64;
    let mut seen = 0u64;
    for (idx, count) in histogram.iter().enumerate() {
        seen += *count;
        if seen > target_rank {
            let bin_fraction = idx as f64 / (SIGNAL_QUALITY_NOISE_FLOOR_BINS - 1) as f64;
            return (bin_fraction * SIGNAL_QUALITY_MAX_CENTERED_POWER)
                .max(SIGNAL_QUALITY_NOISE_FLOOR_POWER_EPSILON);
        }
    }
    SIGNAL_QUALITY_MAX_CENTERED_POWER
}

pub(crate) fn write_signal_quality_report(
    common: &CommonArgs,
    command: &str,
    signal_quality: &RawIqSignalQualityReport,
) -> Result<()> {
    let dataset = load_dataset(common).ok().flatten();
    let run_dir = run_dir(common, command, dataset.as_ref())?;
    let path = run_dir.join("signal_quality_report.json");
    fs::write(&path, serde_json::to_string_pretty(signal_quality)?)?;
    Ok(())
}

pub(crate) fn emit_report<T: Serialize>(
    common: &CommonArgs,
    command: &str,
    report: &T,
) -> Result<()> {
    let summary = serde_json::to_value(report)?;
    let dataset = load_dataset(common).ok().flatten();
    let run_dir = run_dir(common, command, dataset.as_ref())?;
    let summary_path = run_dir.join("summary.json");
    fs::write(&summary_path, serde_json::to_string_pretty(&summary)?)?;

    match common.report {
        ReportFormat::Json => {
            let json = serde_json::to_string_pretty(report)?;
            let report_path = run_dir.join(format!("{command}_report.json"));
            fs::write(&report_path, json)?;
            println!("wrote {}", report_path.display());
        }
        ReportFormat::Table => {
            let json = serde_json::to_string_pretty(report)?;
            println!("{json}");
        }
    }
    Ok(())
}
