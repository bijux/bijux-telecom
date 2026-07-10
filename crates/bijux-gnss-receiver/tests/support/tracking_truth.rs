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

pub fn expected_linear_doppler_hz(
    sample_index: u64,
    sample_rate_hz: f64,
    initial_doppler_hz: f64,
    doppler_rate_hz_per_s: f64,
) -> f64 {
    initial_doppler_hz + ((sample_index as f64) / sample_rate_hz) * doppler_rate_hz_per_s
}

pub fn carrier_frequency_error_under_linear_doppler_hz(
    epoch: &TrackEpoch,
    sample_rate_hz: f64,
    initial_doppler_hz: f64,
    doppler_rate_hz_per_s: f64,
) -> f64 {
    carrier_frequency_error_hz(
        epoch,
        expected_linear_doppler_hz(
            epoch.sample_index,
            sample_rate_hz,
            initial_doppler_hz,
            doppler_rate_hz_per_s,
        ),
    )
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

pub fn stable_tracking_window(epochs: &[TrackEpoch], min_locked_epochs: usize) -> &[TrackEpoch] {
    if min_locked_epochs == 0 {
        return epochs;
    }

    let mut stable_start = None;
    for (index, epoch) in epochs.iter().enumerate() {
        let stable = epoch.lock
            && epoch.lock_state == "tracking"
            && epoch.pll_lock
            && epoch.fll_lock
            && !epoch.cycle_slip
            && epoch.lock_state_reason.as_deref() != Some("lock_lost");
        match (stable_start, stable) {
            (None, true) => stable_start = Some(index),
            (Some(start), false) => {
                if index - start >= min_locked_epochs {
                    return &epochs[start..index];
                }
                stable_start = None;
            }
            _ => {}
        }
    }

    if let Some(start) = stable_start {
        if epochs.len() - start >= min_locked_epochs {
            return &epochs[start..];
        }
    }

    &[]
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

pub fn epoch_indices_with_lock_state(epochs: &[TrackEpoch], lock_state: &str) -> Vec<usize> {
    epochs
        .iter()
        .enumerate()
        .filter_map(|(index, epoch)| (epoch.lock_state == lock_state).then_some(index))
        .collect()
}

pub fn epoch_indices_with_lock_state_reason(
    epochs: &[TrackEpoch],
    lock_state_reason: &str,
) -> Vec<usize> {
    epochs
        .iter()
        .enumerate()
        .filter_map(|(index, epoch)| {
            (epoch.lock_state_reason.as_deref() == Some(lock_state_reason)).then_some(index)
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

pub fn post_lock_carrier_frequency_errors_under_linear_doppler_hz(
    epochs: &[TrackEpoch],
    sample_rate_hz: f64,
    initial_doppler_hz: f64,
    doppler_rate_hz_per_s: f64,
) -> Vec<f64> {
    post_lock_epochs(epochs)
        .iter()
        .map(|epoch| {
            carrier_frequency_error_under_linear_doppler_hz(
                epoch,
                sample_rate_hz,
                initial_doppler_hz,
                doppler_rate_hz_per_s,
            )
        })
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

pub fn stable_tracking_cn0_estimates(epochs: &[TrackEpoch], min_locked_epochs: usize) -> Vec<f64> {
    stable_tracking_window(epochs, min_locked_epochs)
        .iter()
        .filter_map(|epoch| epoch.cn0_dbhz.is_finite().then_some(epoch.cn0_dbhz))
        .collect()
}

pub fn mean_tracking_cn0_dbhz(epochs: &[TrackEpoch], min_locked_epochs: usize) -> Option<f64> {
    let cn0_values = stable_tracking_cn0_estimates(epochs, min_locked_epochs);
    (!cn0_values.is_empty()).then_some(cn0_values.iter().sum::<f64>() / cn0_values.len() as f64)
}

fn gps_lnav_nav_bit_period_samples(sample_rate_hz: f64) -> u64 {
    ((sample_rate_hz * GPS_LNAV_NAV_BIT_PERIOD_S).round() as u64).max(1)
}

#[cfg(test)]
mod tests {
    use super::{
        carrier_frequency_error_hz, carrier_frequency_error_under_linear_doppler_hz,
        code_phase_error_samples, epoch_indices_with_lock_state,
        epoch_indices_with_lock_state_reason, expected_linear_doppler_hz,
        first_tracking_lock_epoch_index, mean_tracking_cn0_dbhz, nav_bit_transition_epoch_indices,
        post_lock_carrier_frequency_errors_hz,
        post_lock_carrier_frequency_errors_under_linear_doppler_hz,
        post_lock_code_phase_errors_samples, post_lock_epochs, stable_tracking_cn0_estimates,
        stable_tracking_window, wrapped_code_phase_error_samples,
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
    fn expected_linear_doppler_hz_applies_rate_from_sample_time() {
        assert_eq!(expected_linear_doppler_hz(6_000, 4_000.0, 100.0, 8.0), 112.0);
    }

    #[test]
    fn carrier_frequency_error_under_linear_doppler_hz_uses_epoch_sample_index() {
        let epoch =
            TrackEpoch { sample_index: 8_000, carrier_hz: Hertz(116.5), ..TrackEpoch::default() };

        assert_eq!(
            carrier_frequency_error_under_linear_doppler_hz(&epoch, 4_000.0, 100.0, 8.0),
            0.5
        );
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
    fn epoch_indices_with_lock_state_collects_matching_epochs() {
        let epochs = vec![
            TrackEpoch { lock_state: "pull_in".to_string(), ..TrackEpoch::default() },
            TrackEpoch { lock_state: "degraded".to_string(), ..TrackEpoch::default() },
            TrackEpoch { lock_state: "degraded".to_string(), ..TrackEpoch::default() },
            TrackEpoch { lock_state: "tracking".to_string(), ..TrackEpoch::default() },
        ];

        assert_eq!(epoch_indices_with_lock_state(&epochs, "degraded"), vec![1, 2]);
    }

    #[test]
    fn epoch_indices_with_lock_state_reason_collects_matching_epochs() {
        let epochs = vec![
            TrackEpoch {
                lock_state_reason: Some("carrier_pull_in".to_string()),
                ..TrackEpoch::default()
            },
            TrackEpoch {
                lock_state_reason: Some("signal_fade".to_string()),
                ..TrackEpoch::default()
            },
            TrackEpoch {
                lock_state_reason: Some("signal_fade".to_string()),
                ..TrackEpoch::default()
            },
            TrackEpoch {
                lock_state_reason: Some("fade_recovered".to_string()),
                ..TrackEpoch::default()
            },
        ];

        assert_eq!(epoch_indices_with_lock_state_reason(&epochs, "signal_fade"), vec![1, 2]);
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
            post_lock_epochs(&epochs).iter().map(|epoch| epoch.epoch.index).collect::<Vec<_>>(),
            vec![1, 2]
        );
    }

    #[test]
    fn stable_tracking_window_returns_first_sustained_tracking_slice() {
        let epochs = vec![
            TrackEpoch {
                lock: true,
                pll_lock: false,
                fll_lock: true,
                lock_state: "pull_in".to_string(),
                ..TrackEpoch::default()
            },
            TrackEpoch {
                epoch: Epoch { index: 1 },
                lock: true,
                pll_lock: true,
                fll_lock: true,
                lock_state: "tracking".to_string(),
                ..TrackEpoch::default()
            },
            TrackEpoch {
                epoch: Epoch { index: 2 },
                lock: true,
                pll_lock: true,
                fll_lock: true,
                lock_state: "tracking".to_string(),
                ..TrackEpoch::default()
            },
        ];

        assert_eq!(
            stable_tracking_window(&epochs, 2)
                .iter()
                .map(|epoch| epoch.epoch.index)
                .collect::<Vec<_>>(),
            vec![1, 2]
        );
    }

    #[test]
    fn stable_tracking_window_rejects_short_or_lost_lock_sequences() {
        let epochs = vec![
            TrackEpoch {
                epoch: Epoch { index: 0 },
                lock: true,
                pll_lock: true,
                fll_lock: true,
                lock_state: "tracking".to_string(),
                lock_state_reason: Some("carrier_converged".to_string()),
                ..TrackEpoch::default()
            },
            TrackEpoch {
                epoch: Epoch { index: 1 },
                lock: true,
                pll_lock: true,
                fll_lock: true,
                lock_state: "tracking".to_string(),
                lock_state_reason: Some("lock_lost".to_string()),
                ..TrackEpoch::default()
            },
        ];

        assert!(stable_tracking_window(&epochs, 2).is_empty());
        assert!(stable_tracking_window(&epochs, 3).is_empty());
    }

    #[test]
    fn stable_tracking_window_keeps_first_qualifying_contiguous_segment() {
        let epochs = vec![
            TrackEpoch {
                epoch: Epoch { index: 0 },
                lock: true,
                pll_lock: true,
                fll_lock: true,
                lock_state: "tracking".to_string(),
                ..TrackEpoch::default()
            },
            TrackEpoch {
                epoch: Epoch { index: 1 },
                lock: true,
                pll_lock: true,
                fll_lock: true,
                lock_state: "tracking".to_string(),
                ..TrackEpoch::default()
            },
            TrackEpoch {
                epoch: Epoch { index: 2 },
                lock: false,
                pll_lock: false,
                fll_lock: false,
                lock_state: "lost".to_string(),
                lock_state_reason: Some("lock_lost".to_string()),
                ..TrackEpoch::default()
            },
            TrackEpoch {
                epoch: Epoch { index: 3 },
                lock: true,
                pll_lock: true,
                fll_lock: true,
                lock_state: "tracking".to_string(),
                ..TrackEpoch::default()
            },
        ];

        assert_eq!(
            stable_tracking_window(&epochs, 2)
                .iter()
                .map(|epoch| epoch.epoch.index)
                .collect::<Vec<_>>(),
            vec![0, 1]
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

        assert_eq!(nav_bit_transition_epoch_indices(&epochs, 4_092_000.0), vec![20, 40]);
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
    fn post_lock_linear_carrier_frequency_errors_starts_at_first_tracking_lock_epoch() {
        let epochs = vec![
            TrackEpoch {
                sample_index: 0,
                carrier_hz: Hertz(90.0),
                pll_lock: false,
                fll_lock: true,
                lock_state: "pull_in".to_string(),
                ..TrackEpoch::default()
            },
            TrackEpoch {
                sample_index: 4_000,
                carrier_hz: Hertz(110.5),
                pll_lock: true,
                fll_lock: true,
                lock_state: "tracking".to_string(),
                ..TrackEpoch::default()
            },
            TrackEpoch {
                sample_index: 8_000,
                carrier_hz: Hertz(120.75),
                pll_lock: true,
                fll_lock: true,
                lock_state: "tracking".to_string(),
                ..TrackEpoch::default()
            },
        ];

        assert_eq!(
            post_lock_carrier_frequency_errors_under_linear_doppler_hz(
                &epochs, 4_000.0, 100.0, 10.0
            ),
            vec![0.5, 0.75]
        );
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

        assert_eq!(post_lock_code_phase_errors_samples(&config, &epochs, 2.0), vec![5.0, 2.0]);
    }

    #[test]
    fn stable_tracking_cn0_estimates_keep_only_finite_stable_epochs() {
        let epochs = vec![
            TrackEpoch {
                lock: true,
                pll_lock: true,
                fll_lock: true,
                lock_state: "tracking".to_string(),
                cn0_dbhz: 45.0,
                ..TrackEpoch::default()
            },
            TrackEpoch {
                lock: true,
                pll_lock: true,
                fll_lock: true,
                lock_state: "tracking".to_string(),
                cn0_dbhz: f64::NAN,
                ..TrackEpoch::default()
            },
            TrackEpoch {
                lock: true,
                pll_lock: true,
                fll_lock: true,
                lock_state: "tracking".to_string(),
                cn0_dbhz: 47.0,
                ..TrackEpoch::default()
            },
        ];

        let cn0_values = stable_tracking_cn0_estimates(&epochs, 3);

        assert_eq!(cn0_values, vec![45.0, 47.0]);
    }

    #[test]
    fn mean_tracking_cn0_dbhz_averages_stable_window() {
        let epochs = vec![
            TrackEpoch {
                lock: true,
                pll_lock: true,
                fll_lock: true,
                lock_state: "tracking".to_string(),
                cn0_dbhz: 45.0,
                ..TrackEpoch::default()
            },
            TrackEpoch {
                lock: true,
                pll_lock: true,
                fll_lock: true,
                lock_state: "tracking".to_string(),
                cn0_dbhz: 47.0,
                ..TrackEpoch::default()
            },
            TrackEpoch {
                lock: true,
                pll_lock: true,
                fll_lock: true,
                lock_state: "tracking".to_string(),
                cn0_dbhz: 49.0,
                ..TrackEpoch::default()
            },
        ];

        let mean_cn0_dbhz = mean_tracking_cn0_dbhz(&epochs, 3).expect("mean cn0");

        assert!((mean_cn0_dbhz - 47.0).abs() <= f64::EPSILON);
    }
}
