#![allow(missing_docs)]

use bijux_gnss_core::api::{Constellation, SampleTime, SamplesFrame, SatId, Seconds};
use bijux_gnss_receiver::api::{
    sim::{generate_l1_ca, SyntheticSignalParams},
    ReceiverPipelineConfig, TrackingEngine,
};
use bijux_gnss_signal::api::{advance_code_phase_seconds, samples_per_code};

const NAV_BIT_PERIOD_MS: usize = 20;

#[test]
fn synthetic_nav_bits_flip_prompt_polarity_every_twenty_milliseconds() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 12,
        ..ReceiverPipelineConfig::default()
    };
    let samples_per_epoch =
        samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length);
    let samples_per_nav_window = samples_per_epoch * NAV_BIT_PERIOD_MS;
    let sat = SatId { constellation: Constellation::Gps, prn: 9 };
    let code_phase_chips = 0.0;
    let frame = generate_l1_ca(
        &config,
        SyntheticSignalParams {
            sat,
            glonass_frequency_channel: None,
            signal_band: bijux_gnss_core::api::SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Ca,
            doppler_hz: 0.0,
            code_phase_chips,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 70.0,
            navigation_data: true.into(),
        },
        0x91B17,
        0.05,
    );
    let tracking =
        TrackingEngine::new(config.clone(), bijux_gnss_receiver::api::ReceiverRuntime::default());

    let prompt_i = [
        (0_u64, samples_per_nav_window),
        (samples_per_nav_window as u64, samples_per_nav_window),
        ((samples_per_nav_window * 2) as u64, samples_per_epoch * 10),
    ]
    .into_iter()
    .map(|(sample_index, sample_count)| {
        let start = sample_index as usize;
        let end = start + sample_count;
        let window_frame = SamplesFrame::new(
            SampleTime { sample_index, sample_rate_hz: config.sampling_freq_hz },
            Seconds(1.0 / config.sampling_freq_hz),
            frame.iq[start..end].to_vec(),
        );
        let window_code_phase_chips = advance_code_phase_seconds(
            code_phase_chips,
            config.code_freq_basis_hz,
            sample_index as f64 / config.sampling_freq_hz,
            config.code_length,
        )
        .expect("valid navigation-window code phase");
        let window_code_phase_samples =
            window_code_phase_chips * config.sampling_freq_hz / config.code_freq_basis_hz;
        let correlator =
            tracking.correlate_epoch(bijux_gnss_receiver::api::TrackingCorrelationRequest {
                frame: &window_frame,
                sat,
                carrier_hz: 0.0,
                carrier_phase_cycles: 0.0,
                code_rate_hz: config.code_freq_basis_hz,
                code_phase_samples: window_code_phase_samples,
                early_late_spacing_chips: 0.5,
            });
        correlator.prompt.re as f64
    })
    .collect::<Vec<_>>();

    assert_eq!(prompt_i.len(), 3);
    let first_window_mean = prompt_i[0];
    let second_window_mean = prompt_i[1];
    let third_window_mean = prompt_i[2];

    assert!(
        first_window_mean.abs() > 1.0,
        "expected a coherent prompt axis before the first nav-bit transition: prompt_i={prompt_i:?}"
    );
    assert!(
        first_window_mean * second_window_mean < 0.0,
        "expected prompt polarity to invert across the first nav-bit transition: prompt_i={prompt_i:?}"
    );
    assert!(
        first_window_mean * third_window_mean > 0.0,
        "expected prompt polarity to return after the second nav-bit transition: prompt_i={prompt_i:?}"
    );
}
