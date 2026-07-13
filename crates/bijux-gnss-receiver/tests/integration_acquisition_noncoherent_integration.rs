#![allow(missing_docs)]

use bijux_gnss_core::api::{Constellation, SatId};
use bijux_gnss_receiver::api::{
    sim::{
        measure_noise_only_acquisition_false_alarm_rate,
        measure_truth_guided_acquisition_detection_probability, SyntheticSignalParams,
    },
    ReceiverPipelineConfig,
};

const LOW_CN0_TRIAL_COUNT: usize = 24;
const NOISE_ONLY_TRIAL_COUNT: usize = 48;

#[test]
fn noncoherent_integration_improves_low_cn0_detection_probability() {
    let config = acquisition_profile();
    let seeds = trial_seeds(0x2407_1987, LOW_CN0_TRIAL_COUNT);
    let signal = SyntheticSignalParams {
        sat: SatId { constellation: Constellation::Gps, prn: 7 },
        glonass_frequency_channel: None,
        signal_band: bijux_gnss_core::api::SignalBand::L1,
        signal_code: bijux_gnss_core::api::SignalCode::Ca,
        doppler_hz: 250.0,
        code_phase_chips: 300.0,
        carrier_phase_rad: 0.0,
        cn0_db_hz: 30.0,
        navigation_data: false.into(),
    };

    let coherent_only = measure_truth_guided_acquisition_detection_probability(
        &config,
        signal.clone(),
        1,
        1,
        &seeds,
        "acquisition_low_cn0_coherent_only",
        2,
        1,
    );
    let noncoherent = measure_truth_guided_acquisition_detection_probability(
        &config,
        signal,
        1,
        4,
        &seeds,
        "acquisition_low_cn0_noncoherent",
        2,
        1,
    );

    assert!(
        noncoherent.detection_probability > coherent_only.detection_probability,
        "expected noncoherent integration to improve detection probability: coherent_only={coherent_only:?}, noncoherent={noncoherent:?}"
    );
}

#[test]
fn noncoherent_integration_keeps_noise_only_false_alarms_below_threshold() {
    let config = acquisition_profile();
    let seeds = trial_seeds(0x2407_1988, NOISE_ONLY_TRIAL_COUNT);
    let sat = SatId { constellation: Constellation::Gps, prn: 7 };

    let coherent_only = measure_noise_only_acquisition_false_alarm_rate(
        &config,
        sat,
        1,
        1,
        &seeds,
        "acquisition_noise_only_coherent_only",
    );
    let noncoherent = measure_noise_only_acquisition_false_alarm_rate(
        &config,
        sat,
        1,
        4,
        &seeds,
        "acquisition_noise_only_noncoherent",
    );

    assert!(
        noncoherent.false_alarm_rate <= 0.05,
        "expected noncoherent integration to stay below the false-alarm threshold: {noncoherent:?}"
    );
    assert!(
        noncoherent.false_alarm_rate <= coherent_only.false_alarm_rate + 0.05,
        "expected noncoherent integration to avoid a material false-alarm regression: coherent_only={coherent_only:?}, noncoherent={noncoherent:?}"
    );
}

fn acquisition_profile() -> ReceiverPipelineConfig {
    ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        acquisition_doppler_search_hz: 1_500,
        acquisition_doppler_step_hz: 250,
        ..ReceiverPipelineConfig::default()
    }
}

fn trial_seeds(base_seed: u64, count: usize) -> Vec<u64> {
    (0..count)
        .map(|index| base_seed.wrapping_add((index as u64).wrapping_mul(0x9e37_79b9_7f4a_7c15)))
        .collect()
}
