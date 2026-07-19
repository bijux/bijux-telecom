use super::*;
use std::collections::BTreeSet;

use bijux_gnss_core::api::{
    Chips, Epoch, Hertz, Meters, SatId, SignalDelayAlignment, TrackingTransmitTime,
    TrackingUncertainty,
};
use bijux_gnss_signal::api::{
    registered_signal_registry_entries, signal_cycles_to_meters, signal_meters_to_cycles,
};

#[path = "tests/carrier_phase_arcs.rs"]
mod carrier_phase_arcs;
#[path = "tests/divergence_detection.rs"]
mod divergence_detection;
#[path = "tests/hatch_smoothing.rs"]
mod hatch_smoothing;
#[path = "tests/measurement_variance.rs"]
mod measurement_variance;
#[path = "tests/pseudorange_resolution.rs"]
mod pseudorange_resolution;
#[path = "tests/rejection_status.rs"]
mod rejection_status;
#[path = "tests/residual_reporting.rs"]
mod residual_reporting;
#[path = "tests/signal_metadata.rs"]
mod signal_metadata;
#[path = "tests/signal_support.rs"]
mod signal_support;
#[path = "tests/timing_metadata.rs"]
mod timing_metadata;
#[path = "tests/tracking_observables.rs"]
mod tracking_observables;

fn make_tracking_epoch(
    prn: u8,
    config: &ReceiverPipelineConfig,
    epoch_idx: u64,
    carrier_hz: f64,
) -> TrackEpoch {
    make_tracking_epoch_with_phase(prn, config, epoch_idx, carrier_hz, 0.0)
}

fn make_tracking_epoch_with_phase(
    prn: u8,
    config: &ReceiverPipelineConfig,
    epoch_idx: u64,
    carrier_hz: f64,
    carrier_phase_cycles: f64,
) -> TrackEpoch {
    let sample_index = epoch_idx
        * samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length)
            as u64;
    TrackEpoch {
        epoch: Epoch { index: epoch_idx },
        sample_index,
        source_time: ReceiverSampleTrace::from_sample_index(sample_index, config.sampling_freq_hz),
        sat: SatId { constellation: Constellation::Gps, prn },
        prompt_i: 1.0,
        prompt_q: 0.0,
        carrier_hz: Hertz(carrier_hz),
        carrier_phase_cycles: Cycles(carrier_phase_cycles),
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
        anti_false_lock: false,
        cycle_slip_reason: None,
        lock_state: "tracking".to_string(),
        lock_state_reason: Some("stable_tracking".to_string()),
        tracking_uncertainty: Some(test_tracking_uncertainty()),
        processing_ms: None,
        ..TrackEpoch::default()
    }
}

fn test_tracking_uncertainty() -> TrackingUncertainty {
    TrackingUncertainty {
        code_phase_samples: 0.05,
        carrier_phase_cycles: 0.02,
        doppler_hz: 1.0,
        cn0_dbhz: 0.5,
    }
}

fn set_code_phase_uncertainty(epoch: &mut TrackEpoch, code_phase_samples: f64) {
    let mut uncertainty =
        epoch.tracking_uncertainty.clone().unwrap_or_else(test_tracking_uncertainty);
    uncertainty.code_phase_samples = code_phase_samples;
    epoch.tracking_uncertainty = Some(uncertainty);
}

fn make_tracking_epoch_with_alignment(
    prn: u8,
    config: &ReceiverPipelineConfig,
    epoch_idx: u64,
    carrier_hz: f64,
    carrier_phase_cycles: f64,
    whole_code_periods: u64,
    code_phase_samples: f64,
) -> TrackEpoch {
    TrackEpoch {
        code_phase_samples: Chips(test_tracking_code_phase_samples(config, code_phase_samples)),
        signal_delay_alignment: Some(SignalDelayAlignment {
            whole_code_periods,
            sample_delay_samples: 0,
            source: "synthetic_truth".to_string(),
        }),
        ..make_tracking_epoch_with_phase(prn, config, epoch_idx, carrier_hz, carrier_phase_cycles)
    }
}

fn test_tracking_code_phase_samples(
    config: &ReceiverPipelineConfig,
    aligned_code_phase_samples: f64,
) -> f64 {
    if !aligned_code_phase_samples.is_finite() || aligned_code_phase_samples < 0.0 {
        return aligned_code_phase_samples;
    }
    let period_samples =
        samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length)
            as f64;
    (period_samples - aligned_code_phase_samples).rem_euclid(period_samples)
}

fn make_observation_ready_epoch(
    prn: u8,
    config: &ReceiverPipelineConfig,
    epoch_idx: u64,
) -> TrackEpoch {
    make_tracking_epoch_with_alignment(prn, config, epoch_idx, 0.0, 0.0, 68, 128.0)
}

fn aligned_pseudorange_m(
    config: &ReceiverPipelineConfig,
    whole_code_periods: u64,
    code_phase_samples: f64,
) -> f64 {
    let code_phase_chips = code_phase_samples
        / (samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length)
            as f64
            / config.code_length as f64);
    ((whole_code_periods as f64 * config.code_length as f64) + code_phase_chips)
        / config.code_freq_basis_hz
        * SPEED_OF_LIGHT_MPS
}

fn test_tracking_code_phase_samples_for_signal(
    config: &ReceiverPipelineConfig,
    signal: SignalSpec,
    code_length: usize,
    aligned_code_phase_chips: f64,
) -> f64 {
    if !aligned_code_phase_chips.is_finite() || aligned_code_phase_chips < 0.0 {
        return aligned_code_phase_chips;
    }
    let samples_per_chip = config.sampling_freq_hz / signal.code_rate_hz;
    let period_samples = samples_per_chip * code_length as f64;
    let aligned_code_phase_samples = aligned_code_phase_chips * samples_per_chip;
    (period_samples - aligned_code_phase_samples).rem_euclid(period_samples)
}

fn aligned_pseudorange_m_for_signal(
    signal: SignalSpec,
    code_length: usize,
    whole_code_periods: u64,
    code_phase_chips: f64,
) -> f64 {
    ((whole_code_periods as f64 * code_length as f64) + code_phase_chips) / signal.code_rate_hz
        * SPEED_OF_LIGHT_MPS
}

fn gps_l1ca_decoded_time_epoch(
    prn: u8,
    config: &ReceiverPipelineConfig,
    epoch_idx: u64,
    capture_start_gps_time: GpsTime,
    receive_gps_time: GpsTime,
    decoded_code_periods: f64,
    aligned_code_phase_chips: f64,
) -> TrackEpoch {
    let signal = signal_spec_gps_l1_ca();
    let code_period_s = 1023.0 / signal.code_rate_hz;
    let code_delay_s = aligned_code_phase_chips / signal.code_rate_hz;
    let expected_signal_travel_time_s = decoded_code_periods * code_period_s + code_delay_s;
    let decoded_transmit_time = receive_gps_time.offset_seconds(-expected_signal_travel_time_s);
    let epoch_sample_index = ((receive_gps_time.tow_s - capture_start_gps_time.tow_s)
        * config.sampling_freq_hz)
        .round() as u64;

    TrackEpoch {
        sat: SatId { constellation: Constellation::Gps, prn },
        signal_band: SignalBand::L1,
        carrier_hz: Hertz(tracked_signal_center_hz(config.intermediate_freq_hz, signal)),
        code_rate_hz: Hertz(signal.code_rate_hz),
        code_phase_samples: Chips(test_tracking_code_phase_samples_for_signal(
            config,
            signal,
            1023,
            aligned_code_phase_chips,
        )),
        transmit_time: Some(TrackingTransmitTime {
            transmit_gps_time: decoded_transmit_time,
            source: "decoded_lnav_how".to_string(),
        }),
        sample_index: epoch_sample_index,
        source_time: ReceiverSampleTrace::from_sample_index(
            epoch_sample_index,
            config.sampling_freq_hz,
        ),
        signal_delay_alignment: None,
        ..make_tracking_epoch_with_phase(
            prn,
            config,
            epoch_idx,
            tracked_signal_center_hz(config.intermediate_freq_hz, signal),
            0.0,
        )
    }
}

fn epoch_sample_index(config: &ReceiverPipelineConfig, epoch_idx: u64) -> u64 {
    epoch_idx
        * samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length)
            as u64
}

fn observation_sample_index(
    config: &ReceiverPipelineConfig,
    initial_sample_index: u64,
    observation_offset: u64,
) -> u64 {
    initial_sample_index + observation_offset * observation_interval_samples(config)
}

fn make_track(prn: u8, config: &ReceiverPipelineConfig) -> TrackingResult {
    let sat = SatId { constellation: Constellation::Gps, prn };
    let epoch = make_tracking_epoch(prn, config, 70, 0.0);
    TrackingResult {
        sat,
        carrier_hz: 0.0,
        code_phase_samples: 0.0,
        acquisition_hypothesis: "deferred".to_string(),
        acquisition_score: 0.0,
        acquisition_code_phase_samples: 0,
        acquisition_carrier_hz: 0.0,
        acq_to_track_state: "deferred".to_string(),
        epochs: vec![epoch],
        transitions: Vec::new(),
    }
}

fn track_from_epoch(epoch: TrackEpoch) -> TrackingResult {
    let sat = epoch.sat;
    TrackingResult {
        sat,
        carrier_hz: epoch.carrier_hz.0,
        code_phase_samples: epoch.code_phase_samples.0,
        acquisition_hypothesis: "accepted".to_string(),
        acquisition_score: 1.0,
        acquisition_code_phase_samples: 0,
        acquisition_carrier_hz: epoch.carrier_hz.0,
        acq_to_track_state: "accepted".to_string(),
        epochs: vec![epoch],
        transitions: Vec::new(),
    }
}
