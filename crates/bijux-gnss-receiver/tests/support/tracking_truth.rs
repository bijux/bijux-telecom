#![allow(missing_docs)]

use bijux_gnss_core::api::TrackEpoch;
use bijux_gnss_receiver::api::ReceiverPipelineConfig;
use bijux_gnss_signal::api::samples_per_code;

const GPS_LNAV_NAV_BIT_PERIOD_S: f64 = 0.02;

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

pub fn post_lock_epochs(epochs: &[TrackEpoch]) -> &[TrackEpoch] {
    let Some(first_lock_epoch_index) = first_tracking_lock_epoch_index(epochs) else {
        return &[];
    };
    &epochs[first_lock_epoch_index..]
}

pub fn nav_bit_transition_epoch_indices(epochs: &[TrackEpoch], sample_rate_hz: f64) -> Vec<usize> {
    let nav_bit_period_samples = gps_lnav_nav_bit_period_samples(sample_rate_hz);
    epochs
        .windows(2)
        .enumerate()
        .filter_map(|(index, pair)| {
            let previous_bit_index = pair[0].sample_index / nav_bit_period_samples;
            let current_bit_index = pair[1].sample_index / nav_bit_period_samples;
            (previous_bit_index != current_bit_index).then_some(index + 1)
        })
        .collect()
}

pub fn post_lock_carrier_frequency_errors_hz(
    epochs: &[TrackEpoch],
    expected_carrier_hz: f64,
) -> Vec<f64> {
    post_lock_epochs(epochs)
        .iter()
        .map(|epoch| carrier_frequency_error_hz(epoch, expected_carrier_hz))
        .collect()
}

pub fn post_lock_code_phase_errors_samples(
    config: &ReceiverPipelineConfig,
    epochs: &[TrackEpoch],
    expected_code_phase_samples: f64,
) -> Vec<f64> {
    post_lock_epochs(epochs)
        .iter()
        .map(|epoch| code_phase_error_samples(config, epoch, expected_code_phase_samples))
        .collect()
}

fn gps_lnav_nav_bit_period_samples(sample_rate_hz: f64) -> u64 {
    ((sample_rate_hz * GPS_LNAV_NAV_BIT_PERIOD_S).round() as u64).max(1)
}

#[cfg(test)]
mod tests {
    use super::{
        carrier_frequency_error_hz, code_phase_error_samples, first_tracking_lock_epoch_index,
        nav_bit_transition_epoch_indices, post_lock_carrier_frequency_errors_hz,
        post_lock_code_phase_errors_samples, post_lock_epochs,
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
    fn post_lock_epochs_returns_empty_when_tracking_never_locks() {
        let epochs = vec![TrackEpoch {
            pll_lock: false,
            fll_lock: true,
            lock_state: "pull_in".to_string(),
            ..TrackEpoch::default()
        }];

        assert!(post_lock_epochs(&epochs).is_empty());
    }

    #[test]
    fn post_lock_epochs_starts_at_first_tracking_lock_epoch() {
        let epochs = vec![
            TrackEpoch {
                epoch: Epoch { index: 0 },
                pll_lock: false,
                fll_lock: true,
                lock_state: "pull_in".to_string(),
                ..TrackEpoch::default()
            },
            TrackEpoch {
                epoch: Epoch { index: 1 },
                pll_lock: true,
                fll_lock: true,
                lock_state: "tracking".to_string(),
                ..TrackEpoch::default()
            },
            TrackEpoch {
                epoch: Epoch { index: 2 },
                pll_lock: true,
                fll_lock: true,
                lock_state: "tracking".to_string(),
                ..TrackEpoch::default()
            },
        ];

        assert_eq!(
            post_lock_epochs(&epochs)
                .iter()
                .map(|epoch| epoch.epoch.index)
                .collect::<Vec<_>>(),
            vec![1, 2]
        );
    }

    #[test]
    fn nav_bit_transition_epoch_indices_report_twenty_millisecond_boundaries() {
        let epochs = (0..45)
            .map(|epoch_index| TrackEpoch {
                sample_index: epoch_index * 4_092,
                ..TrackEpoch::default()
            })
            .collect::<Vec<_>>();

        assert_eq!(
            nav_bit_transition_epoch_indices(&epochs, 4_092_000.0),
            vec![20, 40]
        );
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
