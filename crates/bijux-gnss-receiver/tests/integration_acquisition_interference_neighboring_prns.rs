#![allow(missing_docs)]

use bijux_gnss_core::api::{Constellation, SatId, SignalBand, SignalCode};
use bijux_gnss_receiver::api::{
    sim::{
        measure_truth_guided_acquisition_interference, SyntheticAcquisitionInterferenceCase,
        SyntheticSignalParams,
    },
    ReceiverPipelineConfig,
};

const INTERFERENCE_TRIAL_COUNT: usize = 8;

#[test]
fn neighboring_prn_interference_reduces_target_detection_probability() {
    let config = acquisition_profile();
    let report = measure_truth_guided_acquisition_interference(
        &config,
        &[SyntheticAcquisitionInterferenceCase {
            target_signal: gps_l1_signal(7, 250.0, 300.0, 38.0, 0.0),
            interfering_signals: vec![gps_l1_signal(3, 250.0, 300.0, 60.0, 0.35)],
            coherent_ms: 20,
            noncoherent: 1,
        }],
        &trial_seeds(0x2820_0101, INTERFERENCE_TRIAL_COUNT),
        "acquisition_interference_neighboring_prns",
        2,
        1,
    );

    assert_eq!(report.points.len(), 1);
    let point = &report.points[0];
    assert_eq!(point.trial_count, INTERFERENCE_TRIAL_COUNT);
    assert!(
        point.interfered_detection_probability <= point.isolated_detection_probability,
        "{report:#?}"
    );
    assert!(
        point.mean_interfered_peak_mean_ratio < point.mean_isolated_peak_mean_ratio,
        "{report:#?}"
    );
    assert!(
        point.cross_signal_interference_failure_count > 0
            || point.cross_signal_false_alarm_count > point.thermal_noise_false_alarm_count,
        "{report:#?}"
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

fn gps_l1_signal(
    prn: u8,
    doppler_hz: f64,
    code_phase_chips: f64,
    cn0_db_hz: f32,
    carrier_phase_rad: f64,
) -> SyntheticSignalParams {
    SyntheticSignalParams {
        sat: SatId { constellation: Constellation::Gps, prn },
        glonass_frequency_channel: None,
        signal_band: SignalBand::L1,
        signal_code: SignalCode::Ca,
        doppler_hz,
        code_phase_chips,
        carrier_phase_rad,
        cn0_db_hz,
        navigation_data: false.into(),
    }
}

fn trial_seeds(base_seed: u64, count: usize) -> Vec<u64> {
    (0..count)
        .map(|index| base_seed.wrapping_add((index as u64).wrapping_mul(0x9e37_79b9_7f4a_7c15)))
        .collect()
}
