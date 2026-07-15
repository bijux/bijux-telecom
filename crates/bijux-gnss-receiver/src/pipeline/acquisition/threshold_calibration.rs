use bijux_gnss_core::api::{AcqHypothesis, AcqRequest};
use bijux_gnss_signal::api::AcquisitionSignalModel;

use crate::engine::receiver_config::AcquisitionThresholdMode;
use crate::engine::runtime::{ReceiverRuntime, TraceRecord};

use super::false_alarm_calibration::{
    calibration_seed, false_alarm_rate, mix_seed, noise_only_frame, wilson_confidence_interval,
    FalseAlarmRateMeasurement,
};
use super::request_planning::required_samples_for_request;
use super::signal_model::resolved_request_signal_code;
use super::threshold_resolution::{
    threshold_provenance_for_request, AcquisitionThresholdCacheKey, ResolvedAcquisitionThresholds,
};
use super::Acquisition;

const FALSE_ALARM_CALIBRATION_SEARCH_ITERATIONS: usize = 7;

impl Acquisition {
    pub(super) fn resolve_thresholds_for_request(
        &self,
        request: AcqRequest,
        signal_model: &AcquisitionSignalModel,
    ) -> ResolvedAcquisitionThresholds {
        match self.config.acquisition_threshold_policy.mode {
            AcquisitionThresholdMode::FixedRatio => self.fixed_thresholds_for_request(request),
            AcquisitionThresholdMode::CalibratedFalseAlarm => {
                self.calibrated_thresholds_for_request(request, signal_model)
            }
        }
    }

    fn fixed_thresholds_for_request(&self, request: AcqRequest) -> ResolvedAcquisitionThresholds {
        let mut provenance = threshold_provenance_for_request(&self.config, request);
        provenance.mode = "fixed_ratio".to_string();
        provenance.peak_mean_threshold = self.config.acquisition_peak_mean_threshold;
        provenance.peak_second_threshold = self.config.acquisition_peak_second_threshold;
        provenance.false_alarm_probability = None;
        provenance.calibration_trial_count = None;
        provenance.calibration_confidence_level = None;
        provenance.calibration_false_alarm_rate = None;
        provenance.calibration_false_alarm_interval_low = None;
        provenance.calibration_false_alarm_interval_high = None;
        ResolvedAcquisitionThresholds {
            peak_mean_threshold: self.config.acquisition_peak_mean_threshold,
            peak_second_threshold: self.config.acquisition_peak_second_threshold,
            provenance,
        }
    }

    fn calibrated_thresholds_for_request(
        &self,
        request: AcqRequest,
        signal_model: &AcquisitionSignalModel,
    ) -> ResolvedAcquisitionThresholds {
        let signal_code = resolved_request_signal_code(request);
        let cache_key = AcquisitionThresholdCacheKey::from_request(
            request,
            signal_model.signal_band,
            signal_code,
            signal_model.samples_per_code(self.config.sampling_freq_hz),
        );
        if let Some(cached) =
            self.threshold_cache.lock().ok().and_then(|cache| cache.get(&cache_key).cloned())
        {
            return cached;
        }
        let resolved = self.calibrate_thresholds_for_request(request, signal_model);
        if let Ok(mut cache) = self.threshold_cache.lock() {
            cache.insert(cache_key, resolved.clone());
        }
        resolved
    }

    fn calibrate_thresholds_for_request(
        &self,
        request: AcqRequest,
        signal_model: &AcquisitionSignalModel,
    ) -> ResolvedAcquisitionThresholds {
        let policy = &self.config.acquisition_threshold_policy;
        let target_false_alarm_probability = policy.false_alarm_probability;
        let trial_count = policy.calibration_trial_count;
        let confidence_level = policy.confidence_level;
        let mut lower = 1.0_f32;
        let mut upper = self.config.acquisition_peak_mean_threshold.max(lower + 1.0);
        let mut upper_measurement = self.measure_noise_only_false_alarm_rate(
            request,
            signal_model,
            upper,
            trial_count,
            confidence_level,
        );
        while upper_measurement.false_alarm_rate > target_false_alarm_probability && upper < 64.0 {
            lower = upper;
            upper = (upper * 2.0).min(64.0);
            upper_measurement = self.measure_noise_only_false_alarm_rate(
                request,
                signal_model,
                upper,
                trial_count,
                confidence_level,
            );
        }

        for _ in 0..FALSE_ALARM_CALIBRATION_SEARCH_ITERATIONS {
            let midpoint = (lower + upper) * 0.5;
            let midpoint_measurement = self.measure_noise_only_false_alarm_rate(
                request,
                signal_model,
                midpoint,
                trial_count,
                confidence_level,
            );
            if midpoint_measurement.false_alarm_rate <= target_false_alarm_probability {
                upper = midpoint;
                upper_measurement = midpoint_measurement;
            } else {
                lower = midpoint;
            }
        }

        self.runtime.trace.record(TraceRecord {
            name: "acquisition_threshold_calibration",
            fields: vec![
                ("constellation", format!("{:?}", request.sat.constellation)),
                ("prn", request.sat.prn.to_string()),
                ("signal_band", format!("{:?}", signal_model.signal_band)),
                ("signal_code", format!("{:?}", resolved_request_signal_code(request))),
                ("trial_count", trial_count.to_string()),
                ("target_false_alarm_probability", format!("{target_false_alarm_probability:.9}")),
                ("peak_mean_threshold", format!("{upper:.6}")),
                ("measured_false_alarm_rate", format!("{:.9}", upper_measurement.false_alarm_rate)),
                (
                    "confidence_interval_low",
                    format!("{:.9}", upper_measurement.confidence_interval_low),
                ),
                (
                    "confidence_interval_high",
                    format!("{:.9}", upper_measurement.confidence_interval_high),
                ),
            ],
        });

        let mut provenance = threshold_provenance_for_request(&self.config, request);
        provenance.mode = "calibrated_false_alarm".to_string();
        provenance.peak_mean_threshold = upper;
        provenance.peak_second_threshold = self.config.acquisition_peak_second_threshold;
        provenance.false_alarm_probability = Some(target_false_alarm_probability);
        provenance.calibration_trial_count = Some(trial_count);
        provenance.calibration_confidence_level = Some(confidence_level);
        provenance.calibration_false_alarm_rate = Some(upper_measurement.false_alarm_rate);
        provenance.calibration_false_alarm_interval_low =
            Some(upper_measurement.confidence_interval_low);
        provenance.calibration_false_alarm_interval_high =
            Some(upper_measurement.confidence_interval_high);

        ResolvedAcquisitionThresholds {
            peak_mean_threshold: upper,
            peak_second_threshold: self.config.acquisition_peak_second_threshold,
            provenance,
        }
    }

    fn measure_noise_only_false_alarm_rate(
        &self,
        request: AcqRequest,
        signal_model: &AcquisitionSignalModel,
        peak_mean_threshold: f32,
        trial_count: usize,
        confidence_level: f64,
    ) -> FalseAlarmRateMeasurement {
        let mut calibration_config = self.config.clone();
        calibration_config.acquisition_peak_mean_threshold = peak_mean_threshold;
        calibration_config.acquisition_threshold_policy.mode = AcquisitionThresholdMode::FixedRatio;
        let calibration_engine = Self::new(calibration_config, ReceiverRuntime::default());
        let required_samples = required_samples_for_request(
            &self.config,
            signal_model,
            request.coherent_ms,
            request.noncoherent,
        );
        let calibration_seed = calibration_seed(request, signal_model, peak_mean_threshold);
        let false_alarm_count = (0..trial_count)
            .filter(|trial_index| {
                let frame = noise_only_frame(
                    self.config.sampling_freq_hz,
                    required_samples,
                    mix_seed(calibration_seed, *trial_index as u64),
                );
                calibration_engine
                    .run_fft_for_requests(&frame, &[request])
                    .into_iter()
                    .any(|result| matches!(result.hypothesis, AcqHypothesis::Accepted))
            })
            .count();
        let (confidence_interval_low, confidence_interval_high) =
            wilson_confidence_interval(false_alarm_count, trial_count, confidence_level);
        FalseAlarmRateMeasurement {
            false_alarm_rate: false_alarm_rate(false_alarm_count, trial_count),
            confidence_interval_low,
            confidence_interval_high,
        }
    }
}
