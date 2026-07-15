use super::*;

#[test]
fn acquisition_stability_keys_are_sorted() {
    let sat = SatId { constellation: bijux_gnss_core::api::Constellation::Gps, prn: 1 };
    let mut rows = vec![
        AcqResult {
            sat,
            signal_band: SignalBand::L1,
            signal_code: SignalCode::Unknown,
            glonass_frequency_channel: None,
            source_time: ReceiverSampleTrace::default(),
            candidate_rank: 1,
            is_primary_candidate: true,
            doppler_hz: Hertz(100.0),
            doppler_rate_hz_per_s: 0.0,
            carrier_hz: Hertz(100.0),
            code_phase_samples: 10,
            peak: 10.0,
            second_peak: 2.0,
            mean: 1.0,
            peak_mean_ratio: 10.0,
            peak_second_ratio: 5.0,
            cn0_proxy: 10.0,
            score: 2.0,
            hypothesis: AcqHypothesis::Accepted,
            assumptions: None,
            evidence: Vec::new(),
            threshold_provenance: None,
            explain_selection_reason: None,
            doppler_refinement: None,
            code_phase_refinement: None,
            signal_delay_alignment: None,
            uncertainty: None,
        },
        AcqResult {
            sat,
            signal_band: SignalBand::L1,
            signal_code: SignalCode::Unknown,
            glonass_frequency_channel: None,
            source_time: ReceiverSampleTrace::default(),
            candidate_rank: 1,
            is_primary_candidate: true,
            doppler_hz: Hertz(50.0),
            doppler_rate_hz_per_s: 0.0,
            carrier_hz: Hertz(50.0),
            code_phase_samples: 20,
            peak: 10.0,
            second_peak: 2.0,
            mean: 1.0,
            peak_mean_ratio: 10.0,
            peak_second_ratio: 5.0,
            cn0_proxy: 10.0,
            score: 2.0,
            hypothesis: AcqHypothesis::Accepted,
            assumptions: None,
            evidence: Vec::new(),
            threshold_provenance: None,
            explain_selection_reason: None,
            doppler_refinement: None,
            code_phase_refinement: None,
            signal_delay_alignment: None,
            uncertainty: None,
        },
    ];
    rows.sort_by(|a, b| {
        let primary =
            b.peak_mean_ratio.partial_cmp(&a.peak_mean_ratio).unwrap_or(std::cmp::Ordering::Equal);
        if primary == std::cmp::Ordering::Equal {
            return acq_result_stability_key(a).cmp(&acq_result_stability_key(b));
        }
        primary
    });
    let keys = stable_acq_result_keys(&rows);
    assert!(keys.windows(2).all(|window| window[0] <= window[1]));
}
