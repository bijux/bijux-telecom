#![allow(missing_docs)]

use bijux_gnss_core::api::{
    AcqHypothesis, AcqResult, Constellation, Hertz, ReceiverSampleTrace, SatId, SignalBand,
};
use bijux_gnss_receiver::api::{
    sim::{generate_l1_ca, SyntheticSignalParams},
    ReceiverPipelineConfig, ReceiverRuntime, TrackingEngine,
};

fn accepted_acquisition(sat: SatId, doppler_hz: f64, code_phase_samples: usize) -> AcqResult {
    AcqResult {
        sat,
        signal_band: SignalBand::L1,
        signal_code: bijux_gnss_core::api::SignalCode::Unknown,
        glonass_frequency_channel: None,
        source_time: ReceiverSampleTrace::default(),
        candidate_rank: 1,
        is_primary_candidate: true,
        doppler_hz: Hertz(doppler_hz),
        doppler_rate_hz_per_s: 0.0,
        carrier_hz: Hertz(doppler_hz),
        code_phase_samples,
        peak: 1.0,
        second_peak: 0.1,
        mean: 0.01,
        peak_mean_ratio: 20.0,
        peak_second_ratio: 10.0,
        cn0_proxy: 60.0,
        score: 1.0,
        hypothesis: AcqHypothesis::Accepted,
        assumptions: None,
        evidence: Vec::new(),
        threshold_provenance: None,
        explain_selection_reason: Some("tracking_loop_bandwidth_start".to_string()),
        doppler_refinement: None,
        code_phase_refinement: None,
        signal_delay_alignment: None,
        uncertainty: None,
    }
}

#[test]
fn wider_pll_bandwidth_produces_stronger_first_epoch_carrier_correction() {
    let sat = SatId { constellation: Constellation::Gps, prn: 6 };
    let true_doppler_hz = 120.0;
    let seeded_doppler_hz = 80.0;
    let narrow = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 4,
        early_late_spacing_chips: 0.5,
        dll_bw_hz: 2.0,
        pll_bw_hz: 5.0,
        fll_bw_hz: 0.0,
        ..ReceiverPipelineConfig::default()
    };
    let mut wide = narrow.clone();
    wide.pll_bw_hz = 25.0;

    let frame = generate_l1_ca(
        &narrow,
        SyntheticSignalParams {
            sat,
            glonass_frequency_channel: None,
            signal_band: bijux_gnss_core::api::SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Unknown,
            doppler_hz: true_doppler_hz,
            code_phase_chips: 0.0,
            carrier_phase_rad: 0.30,
            cn0_db_hz: 60.0,
            navigation_data: false.into(),
        },
        0x4F0A_15C6,
        0.020,
    );
    let acquisition = accepted_acquisition(sat, seeded_doppler_hz, 0);

    let narrow_tracks = TrackingEngine::new(narrow, ReceiverRuntime::default())
        .track_from_acquisition(&frame, &[acquisition.clone()]);
    let wide_tracks = TrackingEngine::new(wide, ReceiverRuntime::default())
        .track_from_acquisition(&frame, &[acquisition]);

    let narrow_first_epoch = &narrow_tracks.first().expect("narrow track").epochs[0];
    let wide_first_epoch = &wide_tracks.first().expect("wide track").epochs[0];

    assert!(
        (wide_first_epoch.carrier_hz.0 - seeded_doppler_hz).abs()
            > (narrow_first_epoch.carrier_hz.0 - seeded_doppler_hz).abs(),
        "wide_first_epoch={wide_first_epoch:?} narrow_first_epoch={narrow_first_epoch:?}"
    );
}

#[test]
fn wider_fll_bandwidth_produces_stronger_pull_in_frequency_step() {
    let sat = SatId { constellation: Constellation::Gps, prn: 17 };
    let true_doppler_hz = 220.0;
    let seeded_doppler_hz = 20.0;
    let narrow = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 4,
        early_late_spacing_chips: 0.5,
        dll_bw_hz: 2.0,
        pll_bw_hz: 15.0,
        fll_bw_hz: 4.0,
        ..ReceiverPipelineConfig::default()
    };
    let mut wide = narrow.clone();
    wide.fll_bw_hz = 12.0;

    let frame = generate_l1_ca(
        &narrow,
        SyntheticSignalParams {
            sat,
            glonass_frequency_channel: None,
            signal_band: bijux_gnss_core::api::SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Unknown,
            doppler_hz: true_doppler_hz,
            code_phase_chips: 0.0,
            carrier_phase_rad: 0.25,
            cn0_db_hz: 60.0,
            navigation_data: false.into(),
        },
        0xA531_1D10,
        0.030,
    );
    let acquisition = accepted_acquisition(sat, seeded_doppler_hz, 0);

    let narrow_tracks = TrackingEngine::new(narrow, ReceiverRuntime::default())
        .track_from_acquisition(&frame, &[acquisition.clone()]);
    let wide_tracks = TrackingEngine::new(wide, ReceiverRuntime::default())
        .track_from_acquisition(&frame, &[acquisition]);

    let narrow_epoch = &narrow_tracks.first().expect("narrow track").epochs[1];
    let wide_epoch = &wide_tracks.first().expect("wide track").epochs[1];

    assert!(
        wide_epoch.carrier_hz.0 > narrow_epoch.carrier_hz.0,
        "wide_epoch={wide_epoch:?} narrow_epoch={narrow_epoch:?}"
    );
}
