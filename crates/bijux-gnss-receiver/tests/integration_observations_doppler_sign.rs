#![allow(missing_docs)]

use std::f64::consts::TAU;

use bijux_gnss_core::api::{
    Chips, Constellation, Epoch, Hertz, ReceiverSampleTrace, SatId, TrackEpoch,
};
use bijux_gnss_receiver::api::{
    carrier_hz_from_doppler_hz, observations_from_tracking, ReceiverPipelineConfig,
};

const START_EPOCH_INDEX: u64 = 70;
const START_SAMPLE_INDEX: u64 = START_EPOCH_INDEX * 4092;

#[test]
fn observations_report_doppler_relative_to_intermediate_frequency() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 2_000.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let sat = SatId { constellation: Constellation::Gps, prn: 7 };

    for doppler_hz in [250.0, -250.0] {
        let carrier_hz = carrier_hz_from_doppler_hz(config.intermediate_freq_hz, doppler_hz);
        let epochs = vec![
            tracking_epoch(&config, sat, START_EPOCH_INDEX, START_SAMPLE_INDEX, carrier_hz, 0.10),
            tracking_epoch(
                &config,
                sat,
                START_EPOCH_INDEX + 1,
                START_SAMPLE_INDEX + 4092,
                carrier_hz,
                0.10 + doppler_hz * 0.001,
            ),
        ];

        let (observations, diagnostics) = observations_from_tracking(&config, &epochs);

        assert!(diagnostics.is_empty(), "unexpected observation diagnostics: {diagnostics:?}");
        assert_eq!(observations.len(), 2);
        for observation in &observations {
            assert_eq!(
                observation.sats[0].doppler_hz.0, doppler_hz,
                "observation doppler sign mismatch for carrier {carrier_hz}"
            );
        }
    }
}

fn tracking_epoch(
    config: &ReceiverPipelineConfig,
    sat: SatId,
    epoch_idx: u64,
    sample_index: u64,
    carrier_hz: f64,
    prompt_phase_cycles: f64,
) -> TrackEpoch {
    let phase_rad = prompt_phase_cycles * TAU;
    TrackEpoch {
        epoch: Epoch { index: epoch_idx },
        sample_index,
        source_time: ReceiverSampleTrace::from_sample_index(sample_index, config.sampling_freq_hz),
        sat,
        signal_band: bijux_gnss_core::api::SignalBand::L1,
        signal_code: bijux_gnss_core::api::SignalCode::Unknown,
        glonass_frequency_channel: None,
        prompt_i: phase_rad.cos() as f32,
        prompt_q: phase_rad.sin() as f32,
        early_i: 0.0,
        early_q: 0.0,
        late_i: 0.0,
        late_q: 0.0,
        carrier_hz: Hertz(carrier_hz),
        carrier_phase_cycles: bijux_gnss_core::api::Cycles(prompt_phase_cycles),
        code_rate_hz: Hertz(config.code_freq_basis_hz),
        code_phase_samples: Chips(0.0),
        lock: true,
        cn0_dbhz: 45.0,
        pll_lock: true,
        dll_lock: true,
        fll_lock: true,
        cycle_slip: false,
        nav_bit_lock: false,
        navigation_bit_sign: None,
        dll_err: 0.0,
        pll_err: 0.0,
        fll_err: 0.0,
        anti_false_lock: false,
        cycle_slip_reason: None,
        lock_state: "tracking".to_string(),
        lock_state_reason: None,
        tracking_assumptions: None,
        signal_delay_alignment: None,
        channel_id: Some(0),
        channel_uid: "Gps-07-ch00".to_string(),
        tracking_provenance: "integration_observations_doppler_sign".to_string(),
        transmit_time: None,
        tracking_uncertainty: Some(bijux_gnss_core::api::TrackingUncertainty {
            code_phase_samples: 0.05,
            carrier_phase_cycles: 0.02,
            doppler_hz: 1.0,
            cn0_dbhz: 0.5,
        }),
        processing_ms: None,
    }
}
