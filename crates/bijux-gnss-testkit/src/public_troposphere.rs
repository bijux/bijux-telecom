//! Public troposphere-validation reporting helpers.

use std::collections::BTreeMap;

use bijux_gnss_core::api::{Constellation, GpsTime, ObsEpoch, SatId};
use bijux_gnss_nav::api::{
    position_observations_from_epoch, GpsBroadcastNavigationData, PositionSolver,
    RinexGpsObservationDataset,
};
use serde::{Deserialize, Serialize};

use crate::public_station::PublicStationTruth;
use crate::reference_math::coordinates::elevation_azimuth_deg;
use crate::reference_math::gps_broadcast::satellite_state_from_observation;

pub const LOW_ELEVATION_CEILING_DEG: f64 = 20.0;
pub const MID_ELEVATION_CEILING_DEG: f64 = 45.0;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TroposphereElevationBucketReport {
    pub bucket_name: String,
    pub min_elevation_deg: f64,
    pub max_elevation_deg: Option<f64>,
    pub sample_count: usize,
    pub corrected_mean_abs_residual_m: f64,
    pub uncorrected_mean_abs_residual_m: f64,
    pub corrected_rms_residual_m: f64,
    pub uncorrected_rms_residual_m: f64,
    pub mean_abs_improvement_m: f64,
    pub rms_improvement_m: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PublicTroposphereElevationEpoch {
    pub gps_time: GpsTime,
    pub compared_satellite_count: usize,
    pub low_elevation_satellite_count: usize,
    pub low_elevation_mean_abs_improvement_m: Option<f64>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PublicTroposphereElevationReport {
    pub marker_name: String,
    pub fixture_name: String,
    pub compared_epoch_count: usize,
    pub compared_sample_count: usize,
    pub low_elevation_sample_count: usize,
    pub overall_mean_abs_improvement_m: f64,
    pub overall_rms_improvement_m: f64,
    pub buckets: Vec<TroposphereElevationBucketReport>,
    pub epochs: Vec<PublicTroposphereElevationEpoch>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct ResidualComparisonSample {
    elevation_deg: f64,
    corrected_abs_residual_m: f64,
    uncorrected_abs_residual_m: f64,
}

#[derive(Debug, Clone, Copy, Default, PartialEq)]
struct ResidualAggregate {
    sample_count: usize,
    corrected_abs_sum_m: f64,
    uncorrected_abs_sum_m: f64,
    corrected_squared_sum_m2: f64,
    uncorrected_squared_sum_m2: f64,
}

impl ResidualAggregate {
    fn record(&mut self, sample: ResidualComparisonSample) {
        self.sample_count += 1;
        self.corrected_abs_sum_m += sample.corrected_abs_residual_m;
        self.uncorrected_abs_sum_m += sample.uncorrected_abs_residual_m;
        self.corrected_squared_sum_m2 +=
            sample.corrected_abs_residual_m * sample.corrected_abs_residual_m;
        self.uncorrected_squared_sum_m2 +=
            sample.uncorrected_abs_residual_m * sample.uncorrected_abs_residual_m;
    }

    fn mean_abs_corrected_m(self) -> f64 {
        self.corrected_abs_sum_m / self.sample_count as f64
    }

    fn mean_abs_uncorrected_m(self) -> f64 {
        self.uncorrected_abs_sum_m / self.sample_count as f64
    }

    fn corrected_rms_m(self) -> f64 {
        (self.corrected_squared_sum_m2 / self.sample_count as f64).sqrt()
    }

    fn uncorrected_rms_m(self) -> f64 {
        (self.uncorrected_squared_sum_m2 / self.sample_count as f64).sqrt()
    }
}

pub fn build_public_troposphere_elevation_report(
    observations: &RinexGpsObservationDataset,
    navigation: &GpsBroadcastNavigationData,
    station_truth: &PublicStationTruth,
) -> Result<PublicTroposphereElevationReport, String> {
    let corrected_solver =
        PositionSolver { raim: false, apply_troposphere: true, ..PositionSolver::new() };
    let uncorrected_solver =
        PositionSolver { raim: false, apply_troposphere: false, ..PositionSolver::new() };
    let receiver_ecef_m = station_truth.truth_ecef_m();

    let mut all_samples = Vec::new();
    let mut epochs = Vec::new();

    for epoch in &observations.epochs {
        let gps_time = epoch.gps_time().ok_or_else(|| {
            format!(
                "{} public troposphere validation epoch {} is missing GPS time",
                station_truth.marker_name, epoch.epoch_idx
            )
        })?;
        let receive_tow_s = gps_time.tow_s;
        let position_observations = position_observations_from_epoch(epoch);
        let corrected = corrected_solver.try_solve_wls_with_gps_broadcast_navigation(
            &position_observations,
            navigation,
            receive_tow_s,
        );
        let uncorrected = uncorrected_solver.try_solve_wls_with_gps_broadcast_navigation(
            &position_observations,
            navigation,
            receive_tow_s,
        );
        let (Ok(corrected), Ok(uncorrected)) = (corrected, uncorrected) else {
            continue;
        };

        let corrected_by_sat = corrected
            .residuals
            .iter()
            .map(|(sat, residual_m, _weight)| (*sat, residual_m.abs()))
            .collect::<BTreeMap<_, _>>();
        let uncorrected_by_sat = uncorrected
            .residuals
            .iter()
            .map(|(sat, residual_m, _weight)| (*sat, residual_m.abs()))
            .collect::<BTreeMap<_, _>>();
        let epoch_samples = residual_comparison_samples(
            epoch,
            navigation,
            receiver_ecef_m,
            receive_tow_s,
            &corrected_by_sat,
            &uncorrected_by_sat,
        )?;
        if epoch_samples.is_empty() {
            continue;
        }

        let low_samples = epoch_samples
            .iter()
            .copied()
            .filter(|sample| sample.elevation_deg < LOW_ELEVATION_CEILING_DEG)
            .collect::<Vec<_>>();
        epochs.push(PublicTroposphereElevationEpoch {
            gps_time,
            compared_satellite_count: epoch_samples.len(),
            low_elevation_satellite_count: low_samples.len(),
            low_elevation_mean_abs_improvement_m: summarize_samples(&low_samples).map(
                |aggregate| aggregate.mean_abs_uncorrected_m() - aggregate.mean_abs_corrected_m(),
            ),
        });
        all_samples.extend(epoch_samples);
    }

    if all_samples.is_empty() {
        return Err(format!(
            "{} public troposphere validation found no comparable residual samples",
            station_truth.marker_name
        ));
    }

    let overall = summarize_samples(&all_samples).expect("non-empty troposphere sample summary");
    let buckets = bucket_reports(&all_samples);
    let low_elevation_sample_count = all_samples
        .iter()
        .filter(|sample| sample.elevation_deg < LOW_ELEVATION_CEILING_DEG)
        .count();

    Ok(PublicTroposphereElevationReport {
        marker_name: station_truth.marker_name.clone(),
        fixture_name: station_truth.fixture_name.clone(),
        compared_epoch_count: epochs.len(),
        compared_sample_count: all_samples.len(),
        low_elevation_sample_count,
        overall_mean_abs_improvement_m: overall.mean_abs_uncorrected_m()
            - overall.mean_abs_corrected_m(),
        overall_rms_improvement_m: overall.uncorrected_rms_m() - overall.corrected_rms_m(),
        buckets,
        epochs,
    })
}

fn residual_comparison_samples(
    epoch: &ObsEpoch,
    navigation: &GpsBroadcastNavigationData,
    receiver_ecef_m: (f64, f64, f64),
    receive_tow_s: f64,
    corrected_by_sat: &BTreeMap<SatId, f64>,
    uncorrected_by_sat: &BTreeMap<SatId, f64>,
) -> Result<Vec<ResidualComparisonSample>, String> {
    let mut samples = Vec::new();
    for sat in corrected_by_sat.keys() {
        let Some(uncorrected_abs_residual_m) = uncorrected_by_sat.get(sat) else {
            continue;
        };
        let observation = epoch
            .sats
            .iter()
            .find(|candidate| {
                candidate.signal_id.sat == *sat
                    && candidate.signal_id.sat.constellation == Constellation::Gps
            })
            .ok_or_else(|| format!("missing observation for satellite {:?}", sat))?;
        let ephemeris = navigation
            .ephemerides
            .iter()
            .find(|candidate| candidate.sat == *sat)
            .ok_or_else(|| format!("missing ephemeris for satellite {:?}", sat))?;
        let state = satellite_state_from_observation(
            ephemeris,
            receive_tow_s,
            observation.pseudorange_m.0,
            observation.timing,
        );
        let (_azimuth_deg, elevation_deg) = elevation_azimuth_deg(
            [receiver_ecef_m.0, receiver_ecef_m.1, receiver_ecef_m.2],
            [state.x_m, state.y_m, state.z_m],
        );
        if !elevation_deg.is_finite() || elevation_deg <= 0.0 {
            continue;
        }
        samples.push(ResidualComparisonSample {
            elevation_deg,
            corrected_abs_residual_m: corrected_by_sat[sat],
            uncorrected_abs_residual_m: *uncorrected_abs_residual_m,
        });
    }
    Ok(samples)
}

fn bucket_reports(samples: &[ResidualComparisonSample]) -> Vec<TroposphereElevationBucketReport> {
    [
        ("low", 0.0, Some(LOW_ELEVATION_CEILING_DEG)),
        ("mid", LOW_ELEVATION_CEILING_DEG, Some(MID_ELEVATION_CEILING_DEG)),
        ("high", MID_ELEVATION_CEILING_DEG, None),
    ]
    .into_iter()
    .filter_map(|(bucket_name, min_elevation_deg, max_elevation_deg)| {
        let bucket_samples = samples
            .iter()
            .copied()
            .filter(|sample| sample_in_bucket(*sample, min_elevation_deg, max_elevation_deg))
            .collect::<Vec<_>>();
        let aggregate = summarize_samples(&bucket_samples)?;
        Some(TroposphereElevationBucketReport {
            bucket_name: bucket_name.to_string(),
            min_elevation_deg,
            max_elevation_deg,
            sample_count: aggregate.sample_count,
            corrected_mean_abs_residual_m: aggregate.mean_abs_corrected_m(),
            uncorrected_mean_abs_residual_m: aggregate.mean_abs_uncorrected_m(),
            corrected_rms_residual_m: aggregate.corrected_rms_m(),
            uncorrected_rms_residual_m: aggregate.uncorrected_rms_m(),
            mean_abs_improvement_m: aggregate.mean_abs_uncorrected_m()
                - aggregate.mean_abs_corrected_m(),
            rms_improvement_m: aggregate.uncorrected_rms_m() - aggregate.corrected_rms_m(),
        })
    })
    .collect()
}

fn sample_in_bucket(
    sample: ResidualComparisonSample,
    min_elevation_deg: f64,
    max_elevation_deg: Option<f64>,
) -> bool {
    sample.elevation_deg >= min_elevation_deg
        && max_elevation_deg.map(|max_deg| sample.elevation_deg < max_deg).unwrap_or(true)
}

fn summarize_samples(samples: &[ResidualComparisonSample]) -> Option<ResidualAggregate> {
    if samples.is_empty() {
        return None;
    }
    let mut aggregate = ResidualAggregate::default();
    for sample in samples {
        aggregate.record(*sample);
    }
    Some(aggregate)
}

#[cfg(test)]
mod tests {
    use super::{
        bucket_reports, sample_in_bucket, summarize_samples, ResidualComparisonSample,
        LOW_ELEVATION_CEILING_DEG, MID_ELEVATION_CEILING_DEG,
    };

    fn sample(
        elevation_deg: f64,
        corrected_abs_residual_m: f64,
        uncorrected_abs_residual_m: f64,
    ) -> ResidualComparisonSample {
        ResidualComparisonSample {
            elevation_deg,
            corrected_abs_residual_m,
            uncorrected_abs_residual_m,
        }
    }

    #[test]
    fn summarize_samples_computes_mean_and_rms_improvement() {
        let aggregate = summarize_samples(&[sample(10.0, 1.0, 2.0), sample(15.0, 3.0, 5.0)])
            .expect("aggregate");

        assert_eq!(aggregate.sample_count, 2);
        assert!((aggregate.mean_abs_corrected_m() - 2.0).abs() < 1.0e-12);
        assert!((aggregate.mean_abs_uncorrected_m() - 3.5).abs() < 1.0e-12);
        assert!((aggregate.corrected_rms_m() - 5.0_f64.sqrt()).abs() < 1.0e-12);
        assert!((aggregate.uncorrected_rms_m() - 29.0_f64.sqrt() / 2.0_f64.sqrt()).abs() < 1.0e-12);
    }

    #[test]
    fn bucket_reports_split_samples_by_elevation_band() {
        let buckets = bucket_reports(&[
            sample(10.0, 1.0, 2.5),
            sample(30.0, 0.8, 1.2),
            sample(55.0, 0.6, 0.7),
        ]);

        assert_eq!(buckets.len(), 3);
        assert_eq!(buckets[0].bucket_name, "low");
        assert_eq!(buckets[0].sample_count, 1);
        assert_eq!(buckets[1].bucket_name, "mid");
        assert_eq!(buckets[1].sample_count, 1);
        assert_eq!(buckets[2].bucket_name, "high");
        assert_eq!(buckets[2].sample_count, 1);
        assert!(buckets[0].mean_abs_improvement_m > buckets[2].mean_abs_improvement_m);
    }

    #[test]
    fn sample_in_bucket_uses_half_open_upper_bounds() {
        let low_edge = sample(LOW_ELEVATION_CEILING_DEG, 1.0, 2.0);
        let mid_edge = sample(MID_ELEVATION_CEILING_DEG, 1.0, 2.0);

        assert!(!sample_in_bucket(low_edge, 0.0, Some(LOW_ELEVATION_CEILING_DEG)));
        assert!(sample_in_bucket(
            low_edge,
            LOW_ELEVATION_CEILING_DEG,
            Some(MID_ELEVATION_CEILING_DEG)
        ));
        assert!(!sample_in_bucket(
            mid_edge,
            LOW_ELEVATION_CEILING_DEG,
            Some(MID_ELEVATION_CEILING_DEG)
        ));
        assert!(sample_in_bucket(mid_edge, MID_ELEVATION_CEILING_DEG, None));
    }
}
