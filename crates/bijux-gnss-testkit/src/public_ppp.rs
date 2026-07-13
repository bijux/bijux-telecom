//! Public PPP convergence regression helpers.

use bijux_gnss_core::api::{GpsTime, ObsEpoch, SatId, SignalBand};
use bijux_gnss_nav::api::{
    BroadcastProductsProvider, GpsBroadcastNavigationData, PppConfig, PppFilter,
    PppTroposphereSource, RinexGpsObservationDataset,
};
use serde::{Deserialize, Serialize};

use crate::public_station::{station_enu_error_m, PublicStationTruth};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PublicPppConvergenceEpoch {
    pub gps_time: GpsTime,
    pub elapsed_s: f64,
    pub gps_satellite_count: usize,
    pub dual_frequency_satellite_count: usize,
    pub solved: bool,
    pub three_dimensional_error_m: Option<f64>,
    pub last_rejection: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PublicPppConvergenceReport {
    pub marker_name: String,
    pub fixture_name: String,
    pub troposphere_source: PppTroposphereSource,
    pub observation_epoch_count: usize,
    pub solved_epoch_count: usize,
    pub all_epochs_solved: bool,
    pub time_to_1m_s: Option<f64>,
    pub time_to_decimeter_s: Option<f64>,
    pub final_3d_error_m: f64,
    pub best_3d_error_m: f64,
    pub worst_3d_error_m: f64,
    pub epochs: Vec<PublicPppConvergenceEpoch>,
}

pub fn build_public_ppp_convergence_report(
    observations: &RinexGpsObservationDataset,
    navigation: &GpsBroadcastNavigationData,
    station_truth: &PublicStationTruth,
    config: PppConfig,
) -> Result<PublicPppConvergenceReport, String> {
    let initial_position = observations.approx_position_ecef_m.ok_or_else(|| {
        format!(
            "{} public RINEX observation fixture is missing APPROX POSITION XYZ",
            station_truth.marker_name
        )
    })?;
    let first_time = observations
        .epochs
        .first()
        .ok_or_else(|| {
            format!("{} public PPP dataset contains no epochs", station_truth.marker_name)
        })?
        .gps_time()
        .ok_or_else(|| {
            format!("{} public PPP dataset starts without GPS time", station_truth.marker_name)
        })?;

    let mut filter = PppFilter::new(config);
    let troposphere_source = filter.config.troposphere_source();
    filter.seed_receiver_state([initial_position.0, initial_position.1, initial_position.2], 0.0);
    let products = BroadcastProductsProvider::new(navigation.ephemerides.clone());

    let mut solved_epoch_count = 0usize;
    let mut time_to_1m_s = None;
    let mut time_to_decimeter_s = None;
    let mut best_3d_error_m = f64::INFINITY;
    let mut worst_3d_error_m = 0.0_f64;
    let mut epochs = Vec::with_capacity(observations.epochs.len());

    for epoch in &observations.epochs {
        let gps_time = epoch.gps_time().ok_or_else(|| {
            format!(
                "{} public PPP epoch {} is missing GPS time",
                station_truth.marker_name, epoch.epoch_idx
            )
        })?;
        let elapsed_s = elapsed_seconds(first_time, gps_time);
        let gps_satellites = gps_satellites(epoch);
        let solution = filter.solve_epoch(epoch, &navigation.ephemerides, &products);
        let three_dimensional_error_m = solution.as_ref().map(|solution| {
            station_enu_error_m(
                (solution.ecef_x_m, solution.ecef_y_m, solution.ecef_z_m),
                station_truth,
            )
            .three_dimensional_m
        });

        if let Some(error_m) = three_dimensional_error_m {
            solved_epoch_count += 1;
            best_3d_error_m = best_3d_error_m.min(error_m);
            worst_3d_error_m = worst_3d_error_m.max(error_m);
            if time_to_1m_s.is_none() && error_m <= 1.0 {
                time_to_1m_s = Some(elapsed_s);
            }
            if time_to_decimeter_s.is_none() && error_m <= 0.1 {
                time_to_decimeter_s = Some(elapsed_s);
            }
        }

        epochs.push(PublicPppConvergenceEpoch {
            gps_time,
            elapsed_s,
            gps_satellite_count: gps_satellites.len(),
            dual_frequency_satellite_count: dual_frequency_satellite_count(epoch, &gps_satellites),
            solved: solution.is_some(),
            three_dimensional_error_m,
            last_rejection: filter.ekf.health.last_rejection.clone(),
        });
    }

    if solved_epoch_count == 0 {
        return Err(format!(
            "{} public PPP run produced no solution epochs",
            station_truth.marker_name
        ));
    }

    let final_3d_error_m =
        epochs.iter().rev().find_map(|epoch| epoch.three_dimensional_error_m).ok_or_else(|| {
            format!(
                "{} public PPP report is missing a final solved epoch",
                station_truth.marker_name
            )
        })?;

    Ok(PublicPppConvergenceReport {
        marker_name: station_truth.marker_name.clone(),
        fixture_name: station_truth.fixture_name.clone(),
        troposphere_source,
        observation_epoch_count: observations.epochs.len(),
        solved_epoch_count,
        all_epochs_solved: solved_epoch_count == observations.epochs.len(),
        time_to_1m_s,
        time_to_decimeter_s,
        final_3d_error_m,
        best_3d_error_m,
        worst_3d_error_m,
        epochs,
    })
}

fn elapsed_seconds(start: GpsTime, current: GpsTime) -> f64 {
    (current.week - start.week) as f64 * 604_800.0 + (current.tow_s - start.tow_s)
}

fn gps_satellites(epoch: &ObsEpoch) -> Vec<SatId> {
    let mut satellites = epoch
        .sats
        .iter()
        .filter(|observation| {
            observation.signal_id.sat.constellation == bijux_gnss_core::api::Constellation::Gps
        })
        .map(|observation| observation.signal_id.sat)
        .collect::<Vec<_>>();
    satellites.sort();
    satellites.dedup();
    satellites
}

fn dual_frequency_satellite_count(epoch: &ObsEpoch, satellites: &[SatId]) -> usize {
    satellites
        .iter()
        .filter(|sat| {
            has_band(epoch, **sat, SignalBand::L1) && has_band(epoch, **sat, SignalBand::L2)
        })
        .count()
}

fn has_band(epoch: &ObsEpoch, sat: SatId, band: SignalBand) -> bool {
    epoch
        .sats
        .iter()
        .any(|observation| observation.signal_id.sat == sat && observation.signal_id.band == band)
}
