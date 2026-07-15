use super::*;

#[test]
fn search_window_diagnostic_detects_rising_upper_edge_peak() {
    let sat = SatId { constellation: bijux_gnss_core::api::Constellation::Gps, prn: 1 };
    let diagnostic = signal_outside_search_range(
        &[
            candidate_for_search_window_test(sat, -1_500.0, 1.5),
            candidate_for_search_window_test(sat, -1_250.0, 1.7),
            candidate_for_search_window_test(sat, 1_250.0, 2.4),
            candidate_for_search_window_test(sat, 1_500.0, 2.9),
        ],
        0.0,
        1_500,
        250,
        2.5,
    )
    .expect("search-window diagnostic");

    assert_eq!(diagnostic.dimension, SearchWindowDimension::Doppler);
    assert_eq!(diagnostic.edge, SearchWindowEdge::Upper);
    assert_eq!(diagnostic.best_axis_value, 1_500.0);
    assert_eq!(diagnostic.interior_axis_value, 1_250.0);
}

#[test]
fn search_window_diagnostic_ignores_interior_peak() {
    let sat = SatId { constellation: bijux_gnss_core::api::Constellation::Gps, prn: 1 };
    let diagnostic = signal_outside_search_range(
        &[
            candidate_for_search_window_test(sat, -1_500.0, 1.4),
            candidate_for_search_window_test(sat, -1_250.0, 1.8),
            candidate_for_search_window_test(sat, 0.0, 3.2),
            candidate_for_search_window_test(sat, 1_500.0, 2.7),
        ],
        0.0,
        1_500,
        250,
        2.5,
    );

    assert!(diagnostic.is_none());
}

#[test]
fn search_window_diagnostic_prefers_edge_signal_over_interior_ambiguity() {
    let sat = SatId { constellation: bijux_gnss_core::api::Constellation::Gps, prn: 1 };
    let diagnostic = signal_outside_search_range(
        &[
            candidate_for_search_window_test(sat, -1_500.0, 3.5),
            candidate_for_search_window_test(sat, -1_250.0, 1.6),
            candidate_for_search_window_test(sat, 0.0, 3.6),
            candidate_for_search_window_test(sat, 1_500.0, 1.9),
        ],
        0.0,
        1_500,
        250,
        2.5,
    )
    .expect("search-window diagnostic");

    assert_eq!(diagnostic.dimension, SearchWindowDimension::Doppler);
    assert_eq!(diagnostic.edge, SearchWindowEdge::Lower);
    assert_eq!(diagnostic.best_axis_value, -1_500.0);
    assert_eq!(diagnostic.interior_axis_value, -1_250.0);
}

#[test]
fn search_window_diagnostic_ignores_weak_edge_peak() {
    let sat = SatId { constellation: bijux_gnss_core::api::Constellation::Gps, prn: 1 };
    let diagnostic = signal_outside_search_range(
        &[
            candidate_for_search_window_test(sat, -1_500.0, 1.1),
            candidate_for_search_window_test(sat, -1_250.0, 1.0),
            candidate_for_search_window_test(sat, 1_250.0, 1.8),
            candidate_for_search_window_test(sat, 1_500.0, 2.0),
        ],
        0.0,
        1_500,
        250,
        2.5,
    );

    assert!(diagnostic.is_none());
}

#[test]
fn search_window_diagnostic_respects_nonzero_intermediate_frequency() {
    let sat = SatId { constellation: bijux_gnss_core::api::Constellation::Gps, prn: 1 };
    let intermediate_freq_hz = 1_250_000.0;
    let diagnostic = signal_outside_search_range(
        &[
            candidate_for_search_window_test(
                sat,
                carrier_hz_from_doppler_hz(intermediate_freq_hz, -1_500.0),
                1.5,
            ),
            candidate_for_search_window_test(
                sat,
                carrier_hz_from_doppler_hz(intermediate_freq_hz, -1_250.0),
                1.7,
            ),
            candidate_for_search_window_test(
                sat,
                carrier_hz_from_doppler_hz(intermediate_freq_hz, 1_250.0),
                2.4,
            ),
            candidate_for_search_window_test(
                sat,
                carrier_hz_from_doppler_hz(intermediate_freq_hz, 1_500.0),
                2.9,
            ),
        ],
        intermediate_freq_hz,
        1_500,
        250,
        2.5,
    )
    .expect("search-window diagnostic");

    assert_eq!(diagnostic.dimension, SearchWindowDimension::Doppler);
    assert_eq!(diagnostic.edge, SearchWindowEdge::Upper);
    assert_eq!(
        diagnostic.best_axis_value,
        carrier_hz_from_doppler_hz(intermediate_freq_hz, 1_500.0)
    );
    assert_eq!(
        diagnostic.interior_axis_value,
        carrier_hz_from_doppler_hz(intermediate_freq_hz, 1_250.0)
    );
}

#[test]
fn search_window_diagnostic_detects_doppler_rate_edge_rise() {
    let sat = SatId { constellation: Constellation::Gps, prn: 1 };
    let candidates = vec![
        search_window_rate_candidate(sat, 250.0, -20_000.0, 6.0),
        search_window_rate_candidate(sat, 250.0, -15_000.0, 5.0),
        search_window_rate_candidate(sat, 250.0, 15_000.0, 7.0),
        search_window_rate_candidate(sat, 250.0, 20_000.0, 9.0),
    ];

    let diagnostic =
        signal_outside_doppler_rate_search_range(&candidates, 250.0, 0.0, 20_000, 5_000, 4.0)
            .expect("doppler-rate edge diagnostic");

    assert_eq!(diagnostic.dimension, SearchWindowDimension::DopplerRate);
    assert_eq!(diagnostic.edge, SearchWindowEdge::Upper);
    assert_eq!(diagnostic.best_axis_value, 20_000.0);
    assert_eq!(diagnostic.interior_axis_value, 15_000.0);
}

fn search_window_rate_candidate(
    sat: SatId,
    carrier_hz: f64,
    doppler_rate_hz_per_s: f64,
    peak_mean_ratio: f32,
) -> AcqResult {
    let mut candidate = candidate_for_search_window_test(sat, carrier_hz, peak_mean_ratio);
    candidate.doppler_rate_hz_per_s = doppler_rate_hz_per_s;
    candidate
}
