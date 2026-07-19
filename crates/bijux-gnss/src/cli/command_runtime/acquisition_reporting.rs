use bijux_gnss_infra::api::core::format_sat;

pub(crate) fn print_acquisition_table(report: &AcquisitionReport) {
    println!(
        "I mean: {:.6}  Q mean: {:.6}  I power: {:.6}  Q power: {:.6}  I/Q ratio: {:.6}  Power warning: {}  Quadrature error(deg): {}  Quadrature warning: {}  Clipping(%): {}  Clipping warning: {}  Centered RMS: {:.6e}  Zero-signal: {}  Zero-signal reason: {}  Precision claims allowed: {}  Precision refusal: {}  RMS: {:.6}  DC imbalance: {:.6}",
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
        report.front_end_metrics.dc_imbalance
    );
    println!("Doppler search: {}", format_doppler_search(report.doppler_search));
    println!("Code-phase search: {}", format_code_phase_search(&report.code_phase_search));
    println!("Search summary: {}", format_search_summary(report));
    println!("Reported PRNs: {}", format_reported_prns(report));
    println!(
        "Sat\tBand\tStartSample\tDoppler(Hz)\tRank\tPrimary\tCarrier(Hz)\tCoarseCarrier(Hz)\tRefine(Hz)\tRefine(Bins)\tDopplerUnc(Hz)\tCodePhase\tRefinedCodePhase\tCodePhaseRefine\tCodePhaseUnc\tPeak\tPeak/Mean\tPeak/2nd\tHypothesis\tReason"
    );
    for row in &report.results {
        let coarse_carrier_hz =
            row.coarse_carrier_hz.map(|value| format!("{value:.1}")).unwrap_or_else(|| "n/a".to_string());
        let doppler_refinement_hz = row
            .doppler_refinement_hz
            .map(|value| format!("{value:.3}"))
            .unwrap_or_else(|| "n/a".to_string());
        let doppler_refinement_bins = row
            .doppler_refinement_bins
            .map(|value| format!("{value:.6}"))
            .unwrap_or_else(|| "n/a".to_string());
        let doppler_uncertainty_hz = row
            .doppler_uncertainty_hz
            .map(|value| format!("{value:.3}"))
            .unwrap_or_else(|| "n/a".to_string());
        let refined_code_phase_samples = row
            .refined_code_phase_samples
            .map(|value| format!("{value:.6}"))
            .unwrap_or_else(|| "n/a".to_string());
        let code_phase_refinement_samples = row
            .code_phase_refinement_samples
            .map(|value| format!("{value:.6}"))
            .unwrap_or_else(|| "n/a".to_string());
        let code_phase_uncertainty_samples = row
            .code_phase_uncertainty_samples
            .map(|value| format!("{value:.6}"))
            .unwrap_or_else(|| "n/a".to_string());
        println!(
            "{}\t{:?}\t{}\t{:.3}\t{}\t{}\t{:.1}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{:.3}\t{:.2}\t{:.2}\t{}\t{}",
            format_sat(row.sat),
            row.signal_band,
            row.source_sample_index,
            row.doppler_hz,
            row.candidate_rank,
            row.is_primary_candidate,
            row.carrier_hz,
            coarse_carrier_hz,
            doppler_refinement_hz,
            doppler_refinement_bins,
            doppler_uncertainty_hz,
            row.code_phase_samples,
            refined_code_phase_samples,
            code_phase_refinement_samples,
            code_phase_uncertainty_samples,
            row.peak,
            row.peak_mean_ratio,
            row.peak_second_ratio,
            row.hypothesis,
            format_optional_reason(row.selection_reason.as_deref()),
        );
    }
}

fn format_reported_prns(report: &AcquisitionReport) -> String {
    if report.reported_prns.is_empty() {
        return "none".to_string();
    }

    report
        .reported_prns
        .iter()
        .map(|entry| {
            format!(
                "{} ({}, {:.1} Hz, peak/mean {:.2}, peak/2nd {:.2})",
                format_sat(entry.sat),
                entry.classification,
                entry.carrier_hz,
                entry.peak_mean_ratio,
                entry.peak_second_ratio,
            )
        })
        .collect::<Vec<_>>()
        .join(", ")
}

fn format_search_summary(report: &AcquisitionReport) -> String {
    format!(
        "searched {} PRNs; accepted {}; ambiguous {}; rejected {}; deferred {}",
        report.search_summary.searched_satellites,
        report.search_summary.accepted,
        report.search_summary.ambiguous,
        report.search_summary.rejected,
        report.search_summary.deferred,
    )
}

fn format_doppler_search(settings: DopplerSearchSettings) -> String {
    format!(
        "+/-{} Hz in {} Hz bins ({} bins, IF {:.1} Hz)",
        settings.max_search_hz,
        settings.bin_width_hz,
        settings.bin_count,
        settings.intermediate_freq_hz
    )
}

fn format_code_phase_search(settings: &CodePhaseSearchSettings) -> String {
    format!(
        "{} samples from {} in {}-sample steps ({} bins, mode {})",
        settings.period_samples,
        settings.start_sample,
        settings.step_samples,
        settings.bin_count,
        settings.mode
    )
}
