use bijux_gnss_core::api::{Chips, Constellation, Epoch, Hertz, SatId, TrackEpoch};
use bijux_gnss_receiver::api::TrackingResult;
use bijux_gnss_receiver::api::{observations_from_tracking_results, ReceiverPipelineConfig};

fn make_track(prn: u8, config: &ReceiverPipelineConfig) -> TrackingResult {
    let sat = SatId { constellation: Constellation::Gps, prn };
    let epoch = TrackEpoch {
        epoch: Epoch { index: 0 },
        sample_index: 0,
        sat,
        prompt_i: 1.0,
        prompt_q: 0.0,
        carrier_hz: Hertz(0.0),
        code_rate_hz: Hertz(config.code_freq_basis_hz),
        code_phase_samples: Chips(0.0),
        lock: true,
        cn0_dbhz: 45.0,
        pll_lock: true,
        dll_lock: true,
        fll_lock: true,
        cycle_slip: false,
        nav_bit_lock: false,
        dll_err: 0.0,
        pll_err: 0.0,
        fll_err: 0.0,
        processing_ms: None,
    };
    TrackingResult { sat, carrier_hz: 0.0, code_phase_samples: 0.0, epochs: vec![epoch] }
}

#[test]
fn observations_stage_is_deterministic() {
    let config = ReceiverPipelineConfig::default();
    let tracks = vec![make_track(2, &config), make_track(1, &config)];

    let report_a = observations_from_tracking_results(&config, &tracks, 10);
    let report_b = observations_from_tracking_results(&config, &tracks, 10);

    let out_a = serde_json::to_string(&report_a.output).unwrap();
    let out_b = serde_json::to_string(&report_b.output).unwrap();
    assert_eq!(out_a, out_b);

    let events_a = serde_json::to_string(&report_a.events).unwrap();
    let events_b = serde_json::to_string(&report_b.events).unwrap();
    assert_eq!(events_a, events_b);
}
