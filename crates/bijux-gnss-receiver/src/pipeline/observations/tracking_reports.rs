use bijux_gnss_core::api::{GpsTime, ObsEpoch};

use crate::engine::receiver_config::ReceiverPipelineConfig;
use crate::pipeline::observations::measurement_quality::ObservationMeasurementQualityEpochReport;
use crate::pipeline::observations::residual_reports::ObservationResidualEpochReport;
use crate::pipeline::observations::{
    observation_artifacts_from_tracking_results_with_gps_anchor, ObservationPipelineArtifacts,
};
use crate::pipeline::tracking::TrackingResult;
use crate::pipeline::StepReport;

pub fn observations_from_tracking_results(
    config: &ReceiverPipelineConfig,
    tracks: &[TrackingResult],
    hatch_window: u32,
) -> StepReport<Vec<ObsEpoch>> {
    let report = observation_artifacts_from_tracking_results(config, tracks, hatch_window);
    let StepReport { output, events, stats } = report;
    StepReport { output: output.epochs, events, stats }
}

pub fn observations_from_tracking_results_with_gps_anchor(
    config: &ReceiverPipelineConfig,
    capture_start_gps_time: Option<GpsTime>,
    tracks: &[TrackingResult],
    hatch_window: u32,
) -> StepReport<Vec<ObsEpoch>> {
    let report = observation_artifacts_from_tracking_results_with_gps_anchor(
        config,
        capture_start_gps_time,
        tracks,
        hatch_window,
    );
    let StepReport { output, events, stats } = report;
    StepReport { output: output.epochs, events, stats }
}

pub fn observation_residuals_from_tracking_results(
    config: &ReceiverPipelineConfig,
    tracks: &[TrackingResult],
    hatch_window: u32,
) -> StepReport<Vec<ObservationResidualEpochReport>> {
    let report = observation_artifacts_from_tracking_results(config, tracks, hatch_window);
    let StepReport { output, events, stats } = report;
    StepReport { output: output.residuals, events, stats }
}

pub fn observation_residuals_from_tracking_results_with_gps_anchor(
    config: &ReceiverPipelineConfig,
    capture_start_gps_time: Option<GpsTime>,
    tracks: &[TrackingResult],
    hatch_window: u32,
) -> StepReport<Vec<ObservationResidualEpochReport>> {
    let report = observation_artifacts_from_tracking_results_with_gps_anchor(
        config,
        capture_start_gps_time,
        tracks,
        hatch_window,
    );
    let StepReport { output, events, stats } = report;
    StepReport { output: output.residuals, events, stats }
}

pub fn observation_measurement_quality_from_tracking_results(
    config: &ReceiverPipelineConfig,
    tracks: &[TrackingResult],
    hatch_window: u32,
) -> StepReport<Vec<ObservationMeasurementQualityEpochReport>> {
    observation_measurement_quality_from_tracking_results_with_gps_anchor(
        config,
        None,
        tracks,
        hatch_window,
    )
}

pub fn observation_measurement_quality_from_tracking_results_with_gps_anchor(
    config: &ReceiverPipelineConfig,
    capture_start_gps_time: Option<GpsTime>,
    tracks: &[TrackingResult],
    hatch_window: u32,
) -> StepReport<Vec<ObservationMeasurementQualityEpochReport>> {
    let report = observation_artifacts_from_tracking_results_with_gps_anchor(
        config,
        capture_start_gps_time,
        tracks,
        hatch_window,
    );
    let StepReport { output, events, stats } = report;
    StepReport { output: output.measurement_quality, events, stats }
}

pub fn observation_artifacts_from_tracking_results(
    config: &ReceiverPipelineConfig,
    tracks: &[TrackingResult],
    hatch_window: u32,
) -> StepReport<ObservationPipelineArtifacts> {
    observation_artifacts_from_tracking_results_with_gps_anchor(config, None, tracks, hatch_window)
}
