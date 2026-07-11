#![allow(dead_code, missing_docs)]

use bijux_gnss_core::api::GpsTime;
use bijux_gnss_core::api::{Constellation, ObsEpoch, SatId, SignalBand};
use bijux_gnss_nav::api::{BroadcastProductsProvider, PppConfig, PppFilter, PppSolutionEpoch};

use super::public_spp_case::ab43_public_spp_case;
use super::public_station_truth::{station_enu_error_m, EnuError};

pub struct PublicPppEpochSolution {
    pub gps_time: GpsTime,
    pub solution: PppSolutionEpoch,
    pub enu_error_m: EnuError,
}

pub struct PublicPppConvergenceMetrics {
    pub solved_epochs: usize,
    pub time_to_1m_s: Option<f64>,
    pub time_to_decimeter_s: Option<f64>,
    pub final_3d_error_m: f64,
    pub best_3d_error_m: f64,
    pub worst_3d_error_m: f64,
}

pub struct PublicPppEpochDiagnostics {
    pub gps_time: GpsTime,
    pub gps_satellite_count: usize,
    pub dual_frequency_satellite_count: usize,
    pub solved: bool,
    pub three_dimensional_error_m: Option<f64>,
    pub last_rejection: Option<String>,
}

pub fn public_ab43_ppp_config() -> PppConfig {
    PppConfig { use_iono_free: true, reset_gap_s: 30.0, ..PppConfig::default() }
}

pub fn solve_public_ab43_ppp(
    config: PppConfig,
) -> Result<Vec<PublicPppEpochSolution>, String> {
    let case = ab43_public_spp_case();
    let initial_position = case
        .observations
        .approx_position_ecef_m
        .ok_or_else(|| "AB43 public RINEX observation fixture is missing APPROX POSITION XYZ".to_string())?;
    let mut filter = PppFilter::new(config);
    filter.seed_receiver_state(
        [initial_position.0, initial_position.1, initial_position.2],
        0.0,
    );
    let products = BroadcastProductsProvider::new(case.navigation.ephemerides.clone());
    let truth = &case.station_truth;

    let mut solutions = Vec::new();
    for epoch in &case.observations.epochs {
        let Some(solution) =
            filter.solve_epoch(epoch, &case.navigation.ephemerides, &products)
        else {
            continue;
        };
        let gps_time = epoch.gps_time().ok_or_else(|| {
            format!("AB43 public epoch {} is missing GPS time", epoch.epoch_idx)
        })?;
        let enu_error_m = station_enu_error_m(
            (solution.ecef_x_m, solution.ecef_y_m, solution.ecef_z_m),
            truth,
        );
        solutions.push(PublicPppEpochSolution { gps_time, solution, enu_error_m });
    }

    if solutions.is_empty() {
        return Err("AB43 public PPP run produced no solution epochs".to_string());
    }

    Ok(solutions)
}

pub fn public_ppp_convergence_metrics(
    solutions: &[PublicPppEpochSolution],
) -> Result<PublicPppConvergenceMetrics, String> {
    let first_time = solutions
        .first()
        .map(|solution| solution.gps_time)
        .ok_or_else(|| "public PPP convergence metrics require at least one solution epoch".to_string())?;
    let mut time_to_1m_s = None;
    let mut time_to_decimeter_s = None;
    let mut best_3d_error_m = f64::INFINITY;
    let mut worst_3d_error_m = 0.0_f64;

    for solution in solutions {
        let elapsed_s =
            (solution.gps_time.week - first_time.week) as f64 * 604_800.0
                + (solution.gps_time.tow_s - first_time.tow_s);
        let error_m = solution.enu_error_m.three_dimensional_m;
        best_3d_error_m = best_3d_error_m.min(error_m);
        worst_3d_error_m = worst_3d_error_m.max(error_m);
        if time_to_1m_s.is_none() && error_m <= 1.0 {
            time_to_1m_s = Some(elapsed_s);
        }
        if time_to_decimeter_s.is_none() && error_m <= 0.1 {
            time_to_decimeter_s = Some(elapsed_s);
        }
    }

    let final_3d_error_m = solutions
        .last()
        .map(|solution| solution.enu_error_m.three_dimensional_m)
        .ok_or_else(|| "public PPP convergence metrics require a final solution epoch".to_string())?;

    Ok(PublicPppConvergenceMetrics {
        solved_epochs: solutions.len(),
        time_to_1m_s,
        time_to_decimeter_s,
        final_3d_error_m,
        best_3d_error_m,
        worst_3d_error_m,
    })
}

pub fn diagnose_public_ab43_ppp_epochs(
    config: PppConfig,
) -> Result<Vec<PublicPppEpochDiagnostics>, String> {
    let case = ab43_public_spp_case();
    let initial_position = case
        .observations
        .approx_position_ecef_m
        .ok_or_else(|| "AB43 public RINEX observation fixture is missing APPROX POSITION XYZ".to_string())?;
    let mut filter = PppFilter::new(config);
    filter.seed_receiver_state(
        [initial_position.0, initial_position.1, initial_position.2],
        0.0,
    );
    let products = BroadcastProductsProvider::new(case.navigation.ephemerides.clone());

    case.observations
        .epochs
        .iter()
        .map(|epoch| {
            let gps_time = epoch.gps_time().ok_or_else(|| {
                format!("AB43 public epoch {} is missing GPS time", epoch.epoch_idx)
            })?;
            let gps_satellites = gps_satellites(epoch);
            let solution = filter.solve_epoch(epoch, &case.navigation.ephemerides, &products);
            let three_dimensional_error_m = solution.as_ref().map(|solution| {
                station_enu_error_m(
                    (solution.ecef_x_m, solution.ecef_y_m, solution.ecef_z_m),
                    &case.station_truth,
                )
                .three_dimensional_m
            });
            Ok(PublicPppEpochDiagnostics {
                gps_time,
                gps_satellite_count: gps_satellites.len(),
                dual_frequency_satellite_count: dual_frequency_satellite_count(epoch, &gps_satellites),
                solved: solution.is_some(),
                three_dimensional_error_m,
                last_rejection: filter.ekf.health.last_rejection.clone(),
            })
        })
        .collect()
}

fn gps_satellites(epoch: &ObsEpoch) -> Vec<SatId> {
    let mut satellites = epoch
        .sats
        .iter()
        .filter(|observation| observation.signal_id.sat.constellation == Constellation::Gps)
        .map(|observation| observation.signal_id.sat)
        .collect::<Vec<_>>();
    satellites.sort();
    satellites.dedup();
    satellites
}

fn dual_frequency_satellite_count(epoch: &ObsEpoch, satellites: &[SatId]) -> usize {
    satellites
        .iter()
        .filter(|sat| has_band(epoch, **sat, SignalBand::L1) && has_band(epoch, **sat, SignalBand::L2))
        .count()
}

fn has_band(epoch: &ObsEpoch, sat: SatId, band: SignalBand) -> bool {
    epoch.sats
        .iter()
        .any(|observation| observation.signal_id.sat == sat && observation.signal_id.band == band)
}
