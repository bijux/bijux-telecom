use bijux_gnss_core::api::{GpsTime, Meters, ObsSignalTiming, Seconds, TrackEpoch};

const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;

#[derive(Debug, Clone)]
pub(super) struct PseudorangeComputation {
    pub(super) pseudorange_m: Meters,
    pub(super) timing: Option<ObsSignalTiming>,
    pub(super) model: &'static str,
    pub(super) time_source: Option<String>,
    pub(super) integer_code_periods: Option<u64>,
    pub(super) code_delay_s: Option<Seconds>,
    pub(super) alignment_source: Option<String>,
    pub(super) alignment_resolved: bool,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct ResolvedPseudorange {
    pub(super) signal_travel_time_s: Seconds,
    pub(super) transmit_gps_time: GpsTime,
    pub(super) integer_code_periods: u64,
    pub(super) code_delay_s: Seconds,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct CodePhaseTiming {
    pub(super) code_period_s: f64,
    pub(super) code_delay_s: f64,
}

pub(super) fn pseudorange_from_tracking_epoch(
    epoch: &TrackEpoch,
    samples_per_chip: f64,
    code_rate_hz: f64,
    code_length_chips: f64,
    receive_gps_time: Option<GpsTime>,
    clock_bias_s: f64,
) -> PseudorangeComputation {
    let Some(code_phase_timing) = code_phase_timing_from_tracking_epoch(
        epoch,
        samples_per_chip,
        code_rate_hz,
        code_length_chips,
    ) else {
        return PseudorangeComputation {
            pseudorange_m: Meters(f64::NAN),
            timing: None,
            model: "receiver_epoch_fallback",
            time_source: None,
            integer_code_periods: None,
            code_delay_s: None,
            alignment_source: None,
            alignment_resolved: false,
        };
    };
    let code_period_s = code_phase_timing.code_period_s;
    let code_delay_s = code_phase_timing.code_delay_s;

    if let (Some(receive_gps_time), Some(transmit_time)) = (receive_gps_time, &epoch.transmit_time)
    {
        if let Some(resolved) = resolve_pseudorange_from_transmit_time(
            receive_gps_time,
            transmit_time.transmit_gps_time,
            code_delay_s,
            code_period_s,
        ) {
            return PseudorangeComputation {
                pseudorange_m: Meters(resolved.signal_travel_time_s.0 * SPEED_OF_LIGHT_MPS),
                timing: Some(ObsSignalTiming {
                    signal_travel_time_s: resolved.signal_travel_time_s,
                    transmit_gps_time: resolved.transmit_gps_time,
                }),
                model: "decoded_transmit_time_code_phase",
                time_source: Some(transmit_time.source.clone()),
                integer_code_periods: Some(resolved.integer_code_periods),
                code_delay_s: Some(resolved.code_delay_s),
                alignment_source: epoch
                    .signal_delay_alignment
                    .as_ref()
                    .map(|alignment| alignment.source.clone()),
                alignment_resolved: true,
            };
        }
    }

    if let Some(alignment) = &epoch.signal_delay_alignment {
        let code_time_s = (alignment.whole_code_periods as f64 * code_length_chips
            + code_delay_s * code_rate_hz)
            / code_rate_hz;
        let pseudorange_m =
            Meters(code_time_s * SPEED_OF_LIGHT_MPS + clock_bias_s * SPEED_OF_LIGHT_MPS);
        let signal_travel_time_s = Seconds(pseudorange_m.0 / SPEED_OF_LIGHT_MPS);
        let timing = receive_gps_time.map(|gps_time| ObsSignalTiming {
            signal_travel_time_s,
            transmit_gps_time: gps_time.offset_seconds(-signal_travel_time_s.0),
        });
        return PseudorangeComputation {
            pseudorange_m,
            timing,
            model: "tracked_code_phase_alignment",
            time_source: receive_gps_time.map(|_| "capture_start_gps_time".to_string()),
            integer_code_periods: Some(alignment.whole_code_periods),
            code_delay_s: Some(Seconds(code_delay_s)),
            alignment_source: Some(alignment.source.clone()),
            alignment_resolved: true,
        };
    }

    let code_epoch = epoch.epoch.index as f64;
    let code_time_s = code_epoch * code_period_s + code_delay_s;
    PseudorangeComputation {
        pseudorange_m: Meters(code_time_s * SPEED_OF_LIGHT_MPS + clock_bias_s * SPEED_OF_LIGHT_MPS),
        timing: None,
        model: "receiver_epoch_fallback",
        time_source: None,
        integer_code_periods: None,
        code_delay_s: Some(Seconds(code_delay_s)),
        alignment_source: None,
        alignment_resolved: false,
    }
}

pub(super) fn code_phase_timing_from_tracking_epoch(
    epoch: &TrackEpoch,
    samples_per_chip: f64,
    code_rate_hz: f64,
    code_length_chips: f64,
) -> Option<CodePhaseTiming> {
    if !samples_per_chip.is_finite()
        || !code_rate_hz.is_finite()
        || !code_length_chips.is_finite()
        || samples_per_chip <= 0.0
        || code_rate_hz <= 0.0
        || code_length_chips <= 0.0
    {
        return None;
    }

    let sample_delay_samples = epoch
        .signal_delay_alignment
        .as_ref()
        .map(|alignment| alignment.sample_delay_samples)
        .unwrap_or_default();
    let corrected_code_phase_samples = epoch.code_phase_samples.0 + sample_delay_samples as f64;
    let code_phase_chips = aligned_code_phase_chips(
        code_length_chips,
        corrected_code_phase_samples / samples_per_chip,
    );
    let code_period_s = code_length_chips / code_rate_hz;
    let code_delay_s = code_phase_chips / code_rate_hz;

    (code_period_s.is_finite() && code_period_s > 0.0 && code_delay_s.is_finite())
        .then_some(CodePhaseTiming { code_period_s, code_delay_s })
}

pub(super) fn resolve_pseudorange_from_transmit_time(
    receive_gps_time: GpsTime,
    decoded_transmit_gps_time: GpsTime,
    code_delay_s: f64,
    code_period_s: f64,
) -> Option<ResolvedPseudorange> {
    if !code_delay_s.is_finite()
        || !code_period_s.is_finite()
        || code_delay_s < 0.0
        || code_delay_s >= code_period_s
        || code_period_s <= 0.0
        || !decoded_transmit_gps_time.tow_s.is_finite()
        || !receive_gps_time.tow_s.is_finite()
    {
        return None;
    }
    let decoded_travel_time_s =
        receive_gps_time.to_seconds() - decoded_transmit_gps_time.to_seconds();
    if !decoded_travel_time_s.is_finite() || decoded_travel_time_s <= 0.0 {
        return None;
    }

    let integer_code_periods_f64 = ((decoded_travel_time_s - code_delay_s) / code_period_s).round();
    if !integer_code_periods_f64.is_finite() || integer_code_periods_f64 < 0.0 {
        return None;
    }
    let integer_code_periods = integer_code_periods_f64 as u64;
    let signal_travel_time_s = integer_code_periods as f64 * code_period_s + code_delay_s;
    if !signal_travel_time_s.is_finite() || signal_travel_time_s <= 0.0 {
        return None;
    }

    Some(ResolvedPseudorange {
        signal_travel_time_s: Seconds(signal_travel_time_s),
        transmit_gps_time: receive_gps_time.offset_seconds(-signal_travel_time_s),
        integer_code_periods,
        code_delay_s: Seconds(code_delay_s),
    })
}

pub(super) fn aligned_code_phase_chips(
    code_length_chips: f64,
    tracking_code_phase_chips: f64,
) -> f64 {
    if !tracking_code_phase_chips.is_finite() || tracking_code_phase_chips < 0.0 {
        return tracking_code_phase_chips;
    }
    (code_length_chips - tracking_code_phase_chips.rem_euclid(code_length_chips))
        .rem_euclid(code_length_chips)
}
