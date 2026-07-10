#![allow(missing_docs)]

use bijux_gnss_core::api::TrackEpoch;
use bijux_gnss_receiver::api::ReceiverPipelineConfig;
use bijux_gnss_signal::api::samples_per_code;

pub fn wrapped_code_phase_error_samples(
    config: &ReceiverPipelineConfig,
    measured_code_phase_samples: f64,
    expected_code_phase_samples: f64,
) -> f64 {
    let period_samples =
        samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length)
            .max(1) as f64;
    let wrapped_error =
        (measured_code_phase_samples - expected_code_phase_samples).rem_euclid(period_samples);
    if wrapped_error > period_samples / 2.0 {
        period_samples - wrapped_error
    } else {
        wrapped_error
    }
}

pub fn code_phase_error_samples(
    config: &ReceiverPipelineConfig,
    epoch: &TrackEpoch,
    expected_code_phase_samples: f64,
) -> f64 {
    wrapped_code_phase_error_samples(
        config,
        epoch.code_phase_samples.0,
        expected_code_phase_samples,
    )
}

pub fn carrier_frequency_error_hz(epoch: &TrackEpoch, expected_carrier_hz: f64) -> f64 {
    (epoch.carrier_hz.0 - expected_carrier_hz).abs()
}

pub fn first_tracking_lock_epoch_index(epochs: &[TrackEpoch]) -> Option<usize> {
    epochs
        .iter()
        .position(|epoch| epoch.pll_lock && epoch.fll_lock && epoch.lock_state == "tracking")
}

pub fn post_lock_carrier_frequency_errors_hz(
    epochs: &[TrackEpoch],
    expected_carrier_hz: f64,
) -> Vec<f64> {
    let Some(first_lock_epoch_index) = first_tracking_lock_epoch_index(epochs) else {
        return Vec::new();
    };
    epochs[first_lock_epoch_index..]
        .iter()
        .map(|epoch| carrier_frequency_error_hz(epoch, expected_carrier_hz))
        .collect()
}

pub fn post_lock_code_phase_errors_samples(
    config: &ReceiverPipelineConfig,
    epochs: &[TrackEpoch],
    expected_code_phase_samples: f64,
) -> Vec<f64> {
    let Some(first_lock_epoch_index) = first_tracking_lock_epoch_index(epochs) else {
        return Vec::new();
    };
    epochs[first_lock_epoch_index..]
        .iter()
        .map(|epoch| code_phase_error_samples(config, epoch, expected_code_phase_samples))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::{
        carrier_frequency_error_hz, code_phase_error_samples, first_tracking_lock_epoch_index,
        post_lock_carrier_frequency_errors_hz, post_lock_code_phase_errors_samples,
        wrapped_code_phase_error_samples,
    };
    use bijux_gnss_core::api::{Chips, Cycles, Epoch, Hertz, TrackEpoch};
    use bijux_gnss_receiver::api::ReceiverPipelineConfig;

    #[test]
    fn wrapped_code_phase_error_prefers_shortest_distance() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 1_023_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            ..ReceiverPipelineConfig::default()
        };

        let error = wrapped_code_phase_error_samples(&config, 1_020.0, 2.0);

        assert_eq!(error, 5.0);
    }

    #[test]
    fn carrier_frequency_error_hz_measures_absolute_offset() {
        let epoch = TrackEpoch { carrier_hz: Hertz(101.5), ..TrackEpoch::default() };

        assert_eq!(carrier_frequency_error_hz(&epoch, 100.0), 1.5);
    }

    #[test]
    fn code_phase_error_samples_uses_wrapped_distance_for_epoch() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 1_023_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            ..ReceiverPipelineConfig::default()
        };
        let epoch = TrackEpoch { code_phase_samples: Chips(1_020.0), ..TrackEpoch::default() };

        assert_eq!(code_phase_error_samples(&config, &epoch, 2.0), 5.0);
    }

    #[test]
    fn first_tracking_lock_epoch_index_finds_first_tracking_lock_epoch() {
        let epochs = vec![
            TrackEpoch {
                pll_lock: false,
                fll_lock: true,
                lock_state: "pull_in".to_string(),
                ..TrackEpoch::default()
            },
            TrackEpoch {
                pll_lock: true,
                fll_lock: true,
                lock_state: "tracking".to_string(),
                ..TrackEpoch::default()
            },
            TrackEpoch {
                pll_lock: true,
                fll_lock: true,
                lock_state: "tracking".to_string(),
                ..TrackEpoch::default()
            },
        ];

        assert_eq!(first_tracking_lock_epoch_index(&epochs), Some(1));
    }

    #[test]
    fn post_lock_carrier_frequency_errors_hz_starts_at_first_tracking_lock_epoch() {
        let epochs = vec![
            TrackEpoch {
                carrier_hz: Hertz(90.0),
                carrier_phase_cycles: Cycles(0.0),
                pll_lock: false,
                fll_lock: true,
                lock_state: "pull_in".to_string(),
                ..TrackEpoch::default()
            },
            TrackEpoch {
                epoch: Epoch { index: 1 },
                carrier_hz: Hertz(101.5),
                carrier_phase_cycles: Cycles(0.25),
                pll_lock: true,
                fll_lock: true,
                lock_state: "tracking".to_string(),
                ..TrackEpoch::default()
            },
            TrackEpoch {
                epoch: Epoch { index: 2 },
                carrier_hz: Hertz(99.25),
                carrier_phase_cycles: Cycles(0.5),
                pll_lock: true,
                fll_lock: true,
                lock_state: "tracking".to_string(),
                ..TrackEpoch::default()
            },
        ];

        assert_eq!(post_lock_carrier_frequency_errors_hz(&epochs, 100.0), vec![1.5, 0.75]);
    }

    #[test]
    fn post_lock_code_phase_errors_samples_starts_at_first_tracking_lock_epoch() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 1_023_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            ..ReceiverPipelineConfig::default()
        };
        let epochs = vec![
            TrackEpoch {
                code_phase_samples: Chips(20.0),
                pll_lock: false,
                fll_lock: true,
                lock_state: "pull_in".to_string(),
                ..TrackEpoch::default()
            },
            TrackEpoch {
                epoch: Epoch { index: 1 },
                code_phase_samples: Chips(1_020.0),
                pll_lock: true,
                fll_lock: true,
                lock_state: "tracking".to_string(),
                ..TrackEpoch::default()
            },
            TrackEpoch {
                epoch: Epoch { index: 2 },
                code_phase_samples: Chips(4.0),
                pll_lock: true,
                fll_lock: true,
                lock_state: "tracking".to_string(),
                ..TrackEpoch::default()
            },
        ];

        assert_eq!(
            post_lock_code_phase_errors_samples(&config, &epochs, 2.0),
            vec![5.0, 2.0]
        );
    }
}
