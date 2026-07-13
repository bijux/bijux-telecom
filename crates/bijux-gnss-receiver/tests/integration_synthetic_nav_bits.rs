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
    let sat = SatId { constellation: Constellation::Gps, prn: 9 };
    let code_phase_chips = 200.375;
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

    let prompt_i = frame
        .iq
        .chunks_exact(samples_per_epoch)
        .enumerate()
        .map(|(epoch_index, epoch_samples)| {
            let sample_index = epoch_index as u64 * samples_per_epoch as u64;
            let epoch_frame = SamplesFrame::new(
                SampleTime { sample_index, sample_rate_hz: config.sampling_freq_hz },
                Seconds(1.0 / config.sampling_freq_hz),
                epoch_samples.to_vec(),
            );
            let epoch_code_phase_chips = advance_code_phase_seconds(
                code_phase_chips,
                config.code_freq_basis_hz,
                sample_index as f64 / config.sampling_freq_hz,
                config.code_length,
            )
            .expect("valid epoch code phase");
            let epoch_code_phase_samples =
                epoch_code_phase_chips * config.sampling_freq_hz / config.code_freq_basis_hz;
            let correlator = tracking.correlate_epoch(
                &epoch_frame,
                sat,
                0.0,
                0.0,
                config.code_freq_basis_hz,
                epoch_code_phase_samples,
                0.5,
            );
            correlator.prompt.re as f64
        })
        .collect::<Vec<_>>();

    assert_eq!(prompt_i.len(), 50);
    assert!(
        mean(&prompt_i[0..NAV_BIT_PERIOD_MS]) > 0.0,
        "expected positive prompt polarity before first nav-bit transition"
    );
    assert!(
        mean(&prompt_i[NAV_BIT_PERIOD_MS..2 * NAV_BIT_PERIOD_MS]) < 0.0,
        "expected negative prompt polarity after first nav-bit transition"
    );
    assert!(
        mean(&prompt_i[2 * NAV_BIT_PERIOD_MS..]) > 0.0,
        "expected positive prompt polarity after second nav-bit transition"
    );
}

fn mean(values: &[f64]) -> f64 {
    values.iter().copied().sum::<f64>() / values.len().max(1) as f64
}
