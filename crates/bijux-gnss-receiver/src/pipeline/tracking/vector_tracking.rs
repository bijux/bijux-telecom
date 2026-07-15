#[derive(Debug, Clone, Copy, PartialEq)]
struct VectorTrackingMeasurement {
    sat: SatId,
    channel_id: u8,
    epoch_idx: u64,
    sample_index: u64,
    cn0_dbhz: f64,
    dll_error_samples: f64,
    pll_error_rad: f64,
    fll_error_hz: f64,
    code_rate_error_hz: f64,
    carrier_rate_hz_per_s: f64,
    prompt_locked: bool,
    dll_locked: bool,
    pll_locked: bool,
    fll_locked: bool,
    channel_state: ChannelState,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct CommonTrackingFrequencySignalEstimate {
    pub sat: SatId,
    pub channel_id: u8,
    pub epoch_idx: u64,
    pub sample_index: u64,
    pub cn0_dbhz: f64,
    pub frequency_error_hz: f64,
    pub residual_hz: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct CommonTrackingFrequencyEstimate {
    pub sample_index: u64,
    pub estimated_frequency_error_hz: f64,
    pub support_count: usize,
    pub mean_cn0_dbhz: f64,
    pub residual_spread_hz: f64,
    pub max_supporting_residual_hz: f64,
    pub supporting_channels: Vec<CommonTrackingFrequencySignalEstimate>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct VectorTrackingPrediction {
    sample_index: u64,
    contributor_count: usize,
    mean_cn0_dbhz: f64,
    receiver_position_code_phase_error_samples: f64,
    receiver_clock_frequency_error_hz: f64,
    receiver_clock_frequency_residual_spread_hz: f64,
    receiver_clock_frequency_max_residual_hz: f64,
    receiver_code_rate_error_hz: f64,
    receiver_motion_frequency_rate_hz_per_s: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct VectorTrackingApplication {
    prediction: VectorTrackingPrediction,
    carrier_frequency_correction_hz: f64,
    code_rate_correction_hz: f64,
    code_phase_correction_samples: f64,
    carrier_rate_correction_hz_per_s: f64,
}

#[derive(Debug, Clone, Default)]
struct VectorTrackingState {
    measurements: VecDeque<VectorTrackingMeasurement>,
}

impl VectorTrackingState {
    fn record(&mut self, measurement: VectorTrackingMeasurement, sample_rate_hz: f64) {
        if !vector_tracking_measurement_is_usable(measurement) {
            return;
        }
        self.measurements.push_back(measurement);
        let history_samples = vector_tracking_history_samples(sample_rate_hz);
        let latest_sample_index = measurement.sample_index;
        while self.measurements.front().is_some_and(|oldest| {
            latest_sample_index.saturating_sub(oldest.sample_index) > history_samples
        }) {
            self.measurements.pop_front();
        }
    }

    fn prediction_for(
        &self,
        sample_index: u64,
        sample_rate_hz: f64,
    ) -> Option<VectorTrackingPrediction> {
        vector_tracking_prediction(&self.latest_measurements_for(sample_index, sample_rate_hz))
    }

    fn common_frequency_estimate(
        &self,
        sample_rate_hz: f64,
    ) -> Option<CommonTrackingFrequencyEstimate> {
        let latest_sample_index =
            self.measurements.iter().map(|measurement| measurement.sample_index).max()?;
        common_tracking_frequency_estimate(
            &self.latest_measurements_for(latest_sample_index, sample_rate_hz),
        )
    }

    fn latest_measurements_for(
        &self,
        sample_index: u64,
        sample_rate_hz: f64,
    ) -> Vec<VectorTrackingMeasurement> {
        let history_samples = vector_tracking_history_samples(sample_rate_hz);
        let mut latest_by_channel = Vec::new();
        for measurement in self.measurements.iter().rev().copied() {
            if measurement.sample_index > sample_index {
                continue;
            }
            if sample_index.saturating_sub(measurement.sample_index) > history_samples {
                continue;
            }
            if latest_by_channel.iter().any(|candidate: &VectorTrackingMeasurement| {
                candidate.channel_id == measurement.channel_id
            }) {
                continue;
            }
            latest_by_channel.push(measurement);
        }
        latest_by_channel
    }
}

fn vector_tracking_history_samples(sample_rate_hz: f64) -> u64 {
    ((sample_rate_hz.max(1.0) * VECTOR_TRACKING_HISTORY_SECONDS).round() as u64).max(1)
}

fn vector_tracking_measurement_is_usable(measurement: VectorTrackingMeasurement) -> bool {
    let carrier_reliable = measurement.prompt_locked
        && measurement.fll_locked
        && measurement.cn0_dbhz >= VECTOR_TRACKING_MIN_CN0_DBHZ;
    let state_reliable = match measurement.channel_state {
        ChannelState::PullIn => carrier_reliable,
        ChannelState::Tracking | ChannelState::Degraded => {
            carrier_reliable && measurement.dll_locked && measurement.pll_locked
        }
        ChannelState::Idle | ChannelState::Acquired | ChannelState::Lost => false,
    };
    state_reliable
        && measurement.cn0_dbhz.is_finite()
        && measurement.dll_error_samples.is_finite()
        && measurement.pll_error_rad.is_finite()
        && measurement.fll_error_hz.is_finite()
        && measurement.code_rate_error_hz.is_finite()
        && measurement.carrier_rate_hz_per_s.is_finite()
}

fn vector_tracking_prediction(
    measurements: &[VectorTrackingMeasurement],
) -> Option<VectorTrackingPrediction> {
    if measurements.len() < VECTOR_TRACKING_MIN_CONTRIBUTORS {
        return None;
    }
    let common_frequency = common_tracking_frequency_estimate(measurements)?;
    let mut weighted_position_error_samples = 0.0;
    let mut weighted_code_error_hz = 0.0;
    let mut weighted_motion_rate_hz_per_s = 0.0;
    let mut weight_sum = 0.0;
    let mut code_weight_sum = 0.0;
    let mut sample_index = 0;
    for measurement in measurements {
        let weight = vector_tracking_cn0_weight(measurement.cn0_dbhz);
        if measurement.dll_locked {
            weighted_position_error_samples += measurement.dll_error_samples * weight;
            weighted_code_error_hz += measurement.code_rate_error_hz * weight;
            code_weight_sum += weight;
        }
        weighted_motion_rate_hz_per_s += measurement.carrier_rate_hz_per_s * weight;
        weight_sum += weight;
        sample_index = sample_index.max(measurement.sample_index);
    }
    if weight_sum <= f64::EPSILON {
        return None;
    }
    Some(VectorTrackingPrediction {
        sample_index,
        contributor_count: measurements.len(),
        mean_cn0_dbhz: common_frequency.mean_cn0_dbhz,
        receiver_position_code_phase_error_samples: if code_weight_sum > f64::EPSILON {
            weighted_position_error_samples / code_weight_sum
        } else {
            0.0
        },
        receiver_clock_frequency_error_hz: common_frequency.estimated_frequency_error_hz,
        receiver_clock_frequency_residual_spread_hz: common_frequency.residual_spread_hz,
        receiver_clock_frequency_max_residual_hz: common_frequency.max_supporting_residual_hz,
        receiver_code_rate_error_hz: if code_weight_sum > f64::EPSILON {
            weighted_code_error_hz / code_weight_sum
        } else {
            0.0
        },
        receiver_motion_frequency_rate_hz_per_s: weighted_motion_rate_hz_per_s / weight_sum,
    })
}

fn common_tracking_frequency_estimate(
    measurements: &[VectorTrackingMeasurement],
) -> Option<CommonTrackingFrequencyEstimate> {
    let mut support = measurements
        .iter()
        .copied()
        .filter(|measurement| vector_tracking_measurement_is_usable(*measurement))
        .map(|measurement| CommonTrackingFrequencySignalEstimate {
            sat: measurement.sat,
            channel_id: measurement.channel_id,
            epoch_idx: measurement.epoch_idx,
            sample_index: measurement.sample_index,
            cn0_dbhz: measurement.cn0_dbhz,
            frequency_error_hz: measurement.fll_error_hz,
            residual_hz: 0.0,
        })
        .collect::<Vec<_>>();
    if support.len() < COMMON_TRACKING_FREQUENCY_MIN_CONTRIBUTORS {
        return None;
    }
    let median_frequency_error_hz =
        median_common_tracking_frequency_error_hz(&support).filter(|median| median.is_finite())?;
    support.retain(|signal| {
        (signal.frequency_error_hz - median_frequency_error_hz).abs()
            <= COMMON_TRACKING_FREQUENCY_MAX_MEDIAN_RESIDUAL_HZ
    });
    if support.len() < COMMON_TRACKING_FREQUENCY_MIN_CONTRIBUTORS {
        return None;
    }

    let mut weighted_frequency_error_hz = 0.0;
    let mut weighted_cn0_dbhz = 0.0;
    let mut weight_sum = 0.0;
    let mut sample_index = 0;
    for signal in &support {
        let weight = vector_tracking_cn0_weight(signal.cn0_dbhz);
        weighted_frequency_error_hz += signal.frequency_error_hz * weight;
        weighted_cn0_dbhz += signal.cn0_dbhz * weight;
        weight_sum += weight;
        sample_index = sample_index.max(signal.sample_index);
    }
    if weight_sum <= f64::EPSILON {
        return None;
    }

    let estimated_frequency_error_hz = weighted_frequency_error_hz / weight_sum;
    for signal in &mut support {
        signal.residual_hz = signal.frequency_error_hz - estimated_frequency_error_hz;
    }
    let (min_residual_hz, max_residual_hz, max_supporting_residual_hz) = support.iter().fold(
        (f64::INFINITY, f64::NEG_INFINITY, 0.0_f64),
        |(min_residual_hz, max_residual_hz, max_supporting_residual_hz), signal| {
            (
                min_residual_hz.min(signal.residual_hz),
                max_residual_hz.max(signal.residual_hz),
                max_supporting_residual_hz.max(signal.residual_hz.abs()),
            )
        },
    );

    Some(CommonTrackingFrequencyEstimate {
        sample_index,
        estimated_frequency_error_hz,
        support_count: support.len(),
        mean_cn0_dbhz: weighted_cn0_dbhz / weight_sum,
        residual_spread_hz: max_residual_hz - min_residual_hz,
        max_supporting_residual_hz,
        supporting_channels: support,
    })
}

fn median_common_tracking_frequency_error_hz(
    support: &[CommonTrackingFrequencySignalEstimate],
) -> Option<f64> {
    let mut values = support
        .iter()
        .map(|signal| signal.frequency_error_hz)
        .filter(|value| value.is_finite())
        .collect::<Vec<_>>();
    if values.is_empty() {
        return None;
    }
    values.sort_by(f64::total_cmp);
    let middle = values.len() / 2;
    if values.len() % 2 == 0 {
        Some((values[middle - 1] + values[middle]) / 2.0)
    } else {
        Some(values[middle])
    }
}

fn vector_tracking_cn0_weight(cn0_dbhz: f64) -> f64 {
    let relative_db = (cn0_dbhz - VECTOR_TRACKING_MIN_CN0_DBHZ).clamp(0.0, 15.0);
    10.0_f64.powf(relative_db / 10.0)
}

fn vector_tracking_application(
    prediction: VectorTrackingPrediction,
    state: &LoopState,
) -> Option<VectorTrackingApplication> {
    let gain = vector_tracking_channel_gain(state.state);
    if gain <= f64::EPSILON {
        return None;
    }
    let carrier_frequency_gain = vector_tracking_carrier_frequency_gain(state);
    Some(VectorTrackingApplication {
        prediction,
        carrier_frequency_correction_hz: prediction
            .receiver_clock_frequency_error_hz
            .clamp(-VECTOR_TRACKING_MAX_CARRIER_AID_HZ, VECTOR_TRACKING_MAX_CARRIER_AID_HZ)
            * -carrier_frequency_gain,
        code_rate_correction_hz: prediction
            .receiver_code_rate_error_hz
            .clamp(-VECTOR_TRACKING_MAX_CODE_RATE_AID_HZ, VECTOR_TRACKING_MAX_CODE_RATE_AID_HZ)
            * gain,
        code_phase_correction_samples: prediction.receiver_position_code_phase_error_samples.clamp(
            -VECTOR_TRACKING_MAX_CODE_PHASE_AID_SAMPLES,
            VECTOR_TRACKING_MAX_CODE_PHASE_AID_SAMPLES,
        ) * gain,
        carrier_rate_correction_hz_per_s: prediction.receiver_motion_frequency_rate_hz_per_s.clamp(
            -VECTOR_TRACKING_MAX_CARRIER_RATE_AID_HZ_PER_S,
            VECTOR_TRACKING_MAX_CARRIER_RATE_AID_HZ_PER_S,
        ) * gain,
    })
}

fn vector_tracking_carrier_frequency_gain(state: &LoopState) -> f64 {
    let gain = vector_tracking_channel_gain(state.state);
    if state.lock_reference_cn0_dbhz >= VECTOR_TRACKING_MIN_CN0_DBHZ {
        0.0
    } else {
        gain
    }
}

fn vector_tracking_channel_gain(state: ChannelState) -> f64 {
    match state {
        ChannelState::Acquired | ChannelState::PullIn => 0.35,
        ChannelState::Degraded => 0.20,
        ChannelState::Tracking => 0.05,
        ChannelState::Idle | ChannelState::Lost => 0.0,
    }
}

fn vector_tracking_aiding_mode_label(signal_model: &TrackingSignalModel) -> String {
    match signal_model.aiding_mode {
        TrackingAidingMode::None => "vector_receiver_state".to_string(),
        TrackingAidingMode::PilotCarrier => "pilot_carrier+vector_receiver_state".to_string(),
    }
}

fn vector_tracking_provenance(application: VectorTrackingApplication) -> String {
    format!(
        "vector_tracking=applied vector_prediction_sample_index={} vector_contributors={} vector_mean_cn0_dbhz={:.3} vector_position_code_phase_error_samples={:.6} vector_clock_frequency_error_hz={:.6} vector_clock_frequency_residual_spread_hz={:.6} vector_clock_frequency_max_residual_hz={:.6} vector_code_rate_error_hz={:.6} vector_motion_frequency_rate_hz_per_s={:.6} vector_carrier_frequency_correction_hz={:.6} vector_code_rate_correction_hz={:.6} vector_code_phase_correction_samples={:.6} vector_carrier_rate_correction_hz_per_s={:.6}",
        application.prediction.sample_index,
        application.prediction.contributor_count,
        application.prediction.mean_cn0_dbhz,
        application.prediction.receiver_position_code_phase_error_samples,
        application.prediction.receiver_clock_frequency_error_hz,
        application.prediction.receiver_clock_frequency_residual_spread_hz,
        application.prediction.receiver_clock_frequency_max_residual_hz,
        application.prediction.receiver_code_rate_error_hz,
        application.prediction.receiver_motion_frequency_rate_hz_per_s,
        application.carrier_frequency_correction_hz,
        application.code_rate_correction_hz,
        application.code_phase_correction_samples,
        application.carrier_rate_correction_hz_per_s,
    )
}
