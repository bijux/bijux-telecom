#![allow(missing_docs)]

use crate::estimation::position::solver::{ecef_to_enu, ecef_to_geodetic};

#[derive(Debug, Clone, PartialEq)]
pub struct TrajectoryReconstructionInput {
    pub t_rx_s: f64,
    pub estimated_ecef_m: [f64; 3],
    pub truth_ecef_m: [f64; 3],
}

#[derive(Debug, Clone, PartialEq)]
pub struct TrajectoryReconstructionSample {
    pub t_rx_s: f64,
    pub estimated_ecef_m: [f64; 3],
    pub truth_ecef_m: [f64; 3],
    pub error_ecef_m: [f64; 3],
    pub position_error_3d_m: f64,
    pub horizontal_error_m: f64,
    pub vertical_error_m: f64,
    pub step_error_3d_m: Option<f64>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct TrajectoryReconstructionReport {
    pub samples: Vec<TrajectoryReconstructionSample>,
    pub rms_position_error_3d_m: f64,
    pub max_position_error_3d_m: f64,
    pub final_position_error_3d_m: f64,
    pub rms_step_error_3d_m: Option<f64>,
    pub max_step_error_3d_m: Option<f64>,
    pub cumulative_truth_distance_m: f64,
    pub cumulative_estimated_distance_m: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub enum TrajectoryReconstructionError {
    EmptyInput,
    NonMonotonicTime { previous_t_rx_s: f64, t_rx_s: f64 },
}

pub fn trajectory_reconstruction_report(
    inputs: &[TrajectoryReconstructionInput],
) -> Result<TrajectoryReconstructionReport, TrajectoryReconstructionError> {
    if inputs.is_empty() {
        return Err(TrajectoryReconstructionError::EmptyInput);
    }

    let mut samples = Vec::with_capacity(inputs.len());
    let mut squared_position_error_sum_m2 = 0.0;
    let mut max_position_error_3d_m: f64 = 0.0;
    let mut step_squared_error_sum_m2 = 0.0;
    let mut step_count = 0usize;
    let mut max_step_error_3d_m: f64 = 0.0;
    let mut cumulative_truth_distance_m = 0.0;
    let mut cumulative_estimated_distance_m = 0.0;

    for (index, input) in inputs.iter().enumerate() {
        if index > 0 && input.t_rx_s <= inputs[index - 1].t_rx_s {
            return Err(TrajectoryReconstructionError::NonMonotonicTime {
                previous_t_rx_s: inputs[index - 1].t_rx_s,
                t_rx_s: input.t_rx_s,
            });
        }

        let error_ecef_m = [
            input.estimated_ecef_m[0] - input.truth_ecef_m[0],
            input.estimated_ecef_m[1] - input.truth_ecef_m[1],
            input.estimated_ecef_m[2] - input.truth_ecef_m[2],
        ];
        let position_error_3d_m = vector_norm(error_ecef_m);
        let (truth_lat_deg, truth_lon_deg, truth_alt_m) =
            ecef_to_geodetic(input.truth_ecef_m[0], input.truth_ecef_m[1], input.truth_ecef_m[2]);
        let (east_error_m, north_error_m, up_error_m) = ecef_to_enu(
            input.estimated_ecef_m[0],
            input.estimated_ecef_m[1],
            input.estimated_ecef_m[2],
            truth_lat_deg,
            truth_lon_deg,
            truth_alt_m,
        );
        let horizontal_error_m =
            (east_error_m * east_error_m + north_error_m * north_error_m).sqrt();
        let vertical_error_m = up_error_m.abs();

        squared_position_error_sum_m2 += position_error_3d_m * position_error_3d_m;
        max_position_error_3d_m = max_position_error_3d_m.max(position_error_3d_m);

        let step_error_3d_m = if index == 0 {
            None
        } else {
            let previous_input = &inputs[index - 1];
            let truth_delta_m = [
                input.truth_ecef_m[0] - previous_input.truth_ecef_m[0],
                input.truth_ecef_m[1] - previous_input.truth_ecef_m[1],
                input.truth_ecef_m[2] - previous_input.truth_ecef_m[2],
            ];
            let estimated_delta_m = [
                input.estimated_ecef_m[0] - previous_input.estimated_ecef_m[0],
                input.estimated_ecef_m[1] - previous_input.estimated_ecef_m[1],
                input.estimated_ecef_m[2] - previous_input.estimated_ecef_m[2],
            ];
            let step_error_3d_m = vector_norm([
                estimated_delta_m[0] - truth_delta_m[0],
                estimated_delta_m[1] - truth_delta_m[1],
                estimated_delta_m[2] - truth_delta_m[2],
            ]);

            cumulative_truth_distance_m += vector_norm(truth_delta_m);
            cumulative_estimated_distance_m += vector_norm(estimated_delta_m);
            step_squared_error_sum_m2 += step_error_3d_m * step_error_3d_m;
            step_count += 1;
            max_step_error_3d_m = max_step_error_3d_m.max(step_error_3d_m);
            Some(step_error_3d_m)
        };

        samples.push(TrajectoryReconstructionSample {
            t_rx_s: input.t_rx_s,
            estimated_ecef_m: input.estimated_ecef_m,
            truth_ecef_m: input.truth_ecef_m,
            error_ecef_m,
            position_error_3d_m,
            horizontal_error_m,
            vertical_error_m,
            step_error_3d_m,
        });
    }

    let rms_position_error_3d_m = (squared_position_error_sum_m2 / samples.len() as f64).sqrt();
    let rms_step_error_3d_m =
        (step_count > 0).then(|| (step_squared_error_sum_m2 / step_count as f64).sqrt());
    let max_step_error_3d_m = (step_count > 0).then_some(max_step_error_3d_m);

    Ok(TrajectoryReconstructionReport {
        final_position_error_3d_m: samples
            .last()
            .map(|sample| sample.position_error_3d_m)
            .unwrap_or(0.0),
        samples,
        rms_position_error_3d_m,
        max_position_error_3d_m,
        rms_step_error_3d_m,
        max_step_error_3d_m,
        cumulative_truth_distance_m,
        cumulative_estimated_distance_m,
    })
}

fn vector_norm(vector: [f64; 3]) -> f64 {
    (vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]).sqrt()
}

#[cfg(test)]
mod tests {
    use super::{
        trajectory_reconstruction_report, TrajectoryReconstructionError,
        TrajectoryReconstructionInput,
    };

    #[test]
    fn trajectory_reconstruction_report_aggregates_path_errors_over_time() {
        let report = trajectory_reconstruction_report(&[
            TrajectoryReconstructionInput {
                t_rx_s: 0.0,
                estimated_ecef_m: [1.0, 0.0, 0.0],
                truth_ecef_m: [0.0, 0.0, 0.0],
            },
            TrajectoryReconstructionInput {
                t_rx_s: 1.0,
                estimated_ecef_m: [11.0, 2.0, 0.0],
                truth_ecef_m: [10.0, 0.0, 0.0],
            },
            TrajectoryReconstructionInput {
                t_rx_s: 2.0,
                estimated_ecef_m: [19.0, 1.0, 0.0],
                truth_ecef_m: [20.0, 0.0, 0.0],
            },
        ])
        .expect("trajectory report should build");

        assert_eq!(report.samples.len(), 3);
        assert_eq!(report.samples[0].step_error_3d_m, None);
        assert!((report.samples[1].step_error_3d_m.expect("step error") - 2.0).abs() < 1.0e-12);
        assert!(
            (report.samples[2].step_error_3d_m.expect("step error") - 5.0f64.sqrt()).abs()
                < 1.0e-12
        );
        assert!((report.final_position_error_3d_m - 2.0f64.sqrt()).abs() < 1.0e-12);
        assert!((report.max_position_error_3d_m - 5.0f64.sqrt()).abs() < 1.0e-12);
        assert!((report.rms_position_error_3d_m - (8.0f64 / 3.0).sqrt()).abs() < 1.0e-12);
        assert!(
            (report.rms_step_error_3d_m.expect("step rms") - (4.5f64).sqrt()).abs() < 1.0e-12
        );
        assert!((report.max_step_error_3d_m.expect("step max") - 5.0f64.sqrt()).abs() < 1.0e-12);
        assert!((report.cumulative_truth_distance_m - 20.0).abs() < 1.0e-12);
        assert!(
            (report.cumulative_estimated_distance_m - (104.0f64.sqrt() + 65.0f64.sqrt())).abs()
                < 1.0e-12
        );
    }

    #[test]
    fn trajectory_reconstruction_report_rejects_empty_input() {
        let error = trajectory_reconstruction_report(&[]).expect_err("empty input should fail");

        assert_eq!(error, TrajectoryReconstructionError::EmptyInput);
    }

    #[test]
    fn trajectory_reconstruction_report_rejects_non_monotonic_time() {
        let error = trajectory_reconstruction_report(&[
            TrajectoryReconstructionInput {
                t_rx_s: 5.0,
                estimated_ecef_m: [0.0, 0.0, 0.0],
                truth_ecef_m: [0.0, 0.0, 0.0],
            },
            TrajectoryReconstructionInput {
                t_rx_s: 5.0,
                estimated_ecef_m: [1.0, 0.0, 0.0],
                truth_ecef_m: [1.0, 0.0, 0.0],
            },
        ])
        .expect_err("non-monotonic time should fail");

        assert_eq!(
            error,
            TrajectoryReconstructionError::NonMonotonicTime {
                previous_t_rx_s: 5.0,
                t_rx_s: 5.0,
            }
        );
    }
}
