const NAV_SYMBOL_SYNC_MIN_CONFIDENCE: f64 = 0.05;
#[cfg(feature = "nav")]
const NAV_SYMBOL_SYNC_MIN_COMPLETE_WINDOWS: usize = 2;
#[cfg(feature = "nav")]
const NAV_SYMBOL_BIT_MIN_CONFIDENCE: f64 = 0.75;

#[cfg(feature = "nav")]
fn annotate_navigation_bit_signs(signal_model: &TrackingSignalModel, epochs: &mut [TrackEpoch]) {
    if !signal_model.supports_navigation_bit_sign_recovery() || epochs.is_empty() {
        return;
    }

    let prompt_history = epochs.iter().map(|epoch| epoch.prompt_i).collect::<Vec<_>>();
    let demodulation = bijux_gnss_nav::api::demodulate_gps_l1ca_navigation_bits(&prompt_history);
    if !navigation_symbol_sync_is_confident(&demodulation) {
        return;
    }

    for bit in demodulation.bits {
        if bit.confidence < NAV_SYMBOL_BIT_MIN_CONFIDENCE {
            continue;
        }
        for epoch in epochs[bit.start_prompt_index..bit.end_prompt_index_exclusive].iter_mut() {
            epoch.navigation_bit_sign = Some(bit.sign);
            epoch.nav_bit_lock = true;
            annotate_navigation_symbol_sync_provenance(
                epoch,
                demodulation.bit_start_ms,
                demodulation.sync_confidence,
                bit.confidence,
            );
        }
    }
}

#[cfg(feature = "nav")]
fn annotate_navigation_symbol_sync_provenance(
    epoch: &mut TrackEpoch,
    bit_start_ms: usize,
    sync_confidence: f64,
    bit_confidence: f64,
) {
    epoch.tracking_provenance.push_str(&format!(
        " nav_symbol_sync=confident nav_symbol_start_ms={bit_start_ms} nav_symbol_sync_confidence={sync_confidence:.6} nav_symbol_bit_confidence={bit_confidence:.6}"
    ));
}

#[cfg(feature = "nav")]
fn navigation_symbol_sync_is_confident(
    demodulation: &bijux_gnss_nav::api::GpsL1CaNavigationBits,
) -> bool {
    demodulation.complete_window_count >= NAV_SYMBOL_SYNC_MIN_COMPLETE_WINDOWS
        && demodulation.sync_confidence >= NAV_SYMBOL_SYNC_MIN_CONFIDENCE
}

#[cfg(feature = "nav")]
fn annotate_lnav_transmit_times(
    signal_model: &TrackingSignalModel,
    capture_start_gps_time: Option<GpsTime>,
    epochs: &mut [TrackEpoch],
) {
    if !signal_model.supports_navigation_bit_sign_recovery() || epochs.is_empty() {
        return;
    }
    let Some(capture_start_gps_time) = capture_start_gps_time else {
        return;
    };

    let prompt_history = epochs.iter().map(|epoch| epoch.prompt_i).collect::<Vec<_>>();
    let decoded = bijux_gnss_nav::api::decode_gps_l1ca_lnav_from_prompt(&prompt_history);
    for subframe in decoded.subframes {
        if !subframe.tlm.parity_ok || !subframe.how.parity_ok {
            continue;
        }
        let start = subframe.alignment.start_prompt_index.min(epochs.len());
        let end = subframe.alignment.end_prompt_index_exclusive.min(epochs.len());
        if start >= end {
            continue;
        }

        for (offset_ms, epoch) in epochs[start..end].iter_mut().enumerate() {
            let transmit_tow_s = subframe.how.tow_start_s as f64 + offset_ms as f64 * 0.001;
            let transmit_gps_time =
                gps_transmit_time_near_receive_time(capture_start_gps_time, epoch, transmit_tow_s);
            epoch.transmit_time = Some(TrackingTransmitTime {
                transmit_gps_time,
                source: "gps_l1ca_lnav_how".to_string(),
            });
            epoch.tracking_provenance.push_str(&format!(
                " lnav_transmit_time=decoded how_tow_s={} lnav_parity_ok_count={}",
                subframe.how.tow_start_s, subframe.alignment.parity_ok_count
            ));
        }
    }
}

#[cfg(feature = "nav")]
fn gps_transmit_time_near_receive_time(
    capture_start_gps_time: GpsTime,
    epoch: &TrackEpoch,
    transmit_tow_s: f64,
) -> GpsTime {
    let receive_time = capture_start_gps_time.offset_seconds(epoch.source_time.receiver_time_s.0);
    let receive_seconds = receive_time.to_seconds();
    let base_week = receive_time.week;
    let mut best = GpsTime { week: base_week, tow_s: transmit_tow_s };
    let mut best_delta = (best.to_seconds() - receive_seconds).abs();
    for week in [base_week.saturating_sub(1), base_week.saturating_add(1)] {
        let candidate = GpsTime { week, tow_s: transmit_tow_s };
        let delta = (candidate.to_seconds() - receive_seconds).abs();
        if delta < best_delta {
            best = candidate;
            best_delta = delta;
        }
    }
    best
}

#[cfg(not(feature = "nav"))]
fn annotate_navigation_bit_signs(_signal_model: &TrackingSignalModel, _epochs: &mut [TrackEpoch]) {}

#[cfg(not(feature = "nav"))]
fn annotate_lnav_transmit_times(
    _signal_model: &TrackingSignalModel,
    _reference_week: Option<u32>,
    _epochs: &mut [TrackEpoch],
) {
}

fn stabilize_joint_navigation_bit_signs(
    signal_model: &TrackingSignalModel,
    epochs: &mut [TrackEpoch],
) {
    if signal_model.aiding_mode != TrackingAidingMode::PilotCarrier
        || !signal_model.supports_epoch_data_symbol_sign_recovery()
        || epochs.len() < 3
    {
        return;
    }

    let stable_tracking_epoch = |epoch: &TrackEpoch| {
        epoch.lock_state == "tracking" && epoch.pll_lock && epoch.dll_lock && !epoch.cycle_slip
    };
    let mut smoothed_signs = Vec::new();
    for index in 1..epochs.len() - 1 {
        let previous = &epochs[index - 1];
        let current = &epochs[index];
        let next = &epochs[index + 1];
        if !stable_tracking_epoch(previous)
            || !stable_tracking_epoch(current)
            || !stable_tracking_epoch(next)
        {
            continue;
        }
        let Some(previous_sign) = previous.navigation_bit_sign else {
            continue;
        };
        let Some(next_sign) = next.navigation_bit_sign else {
            continue;
        };
        if previous_sign != next_sign {
            continue;
        }
        match current.navigation_bit_sign {
            None => smoothed_signs.push((index, previous_sign)),
            Some(current_sign) if current_sign != previous_sign => {
                smoothed_signs.push((index, previous_sign));
            }
            _ => {}
        }
    }

    for (index, sign) in smoothed_signs {
        epochs[index].navigation_bit_sign = Some(sign);
        epochs[index].nav_bit_lock = true;
    }
}
