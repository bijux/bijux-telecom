#![allow(dead_code, missing_docs)]

use std::path::PathBuf;
use std::sync::OnceLock;

use bijux_gnss_core::api::{GpsTime, ObsEpoch, SatId};
use bijux_gnss_nav::api::{GpsBroadcastNavigationData, PositionObservation, PositionSolver};
use bijux_gnss_nav::parse_rinex_gps_observation_dataset;

use super::public_station_truth::{public_station_truth_by_fixture, PublicStationTruth};

pub struct PublicSppCase {
    pub station_truth: PublicStationTruth,
    pub observations: bijux_gnss_nav::RinexGpsObservationDataset,
    pub navigation: GpsBroadcastNavigationData,
}

#[derive(Debug, Clone, PartialEq)]
pub struct SolvedPublicEpoch {
    pub gps_time: GpsTime,
    pub ecef_m: (f64, f64, f64),
    pub clock_bias_s: f64,
    pub pre_fit_residual_rms_m: f64,
    pub post_fit_residual_rms_m: f64,
    pub residuals: Vec<(SatId, f64, f64)>,
}

pub fn ab43_public_spp_case() -> &'static PublicSppCase {
    static CASE: OnceLock<PublicSppCase> = OnceLock::new();
    CASE.get_or_init(|| PublicSppCase {
        station_truth: public_station_truth_by_fixture("unavco_ab43_20180114.obs"),
        observations: parse_rinex_gps_observation_dataset(&fixture("unavco_ab43_20180114.obs"))
            .expect("parse AB43 public RINEX observations"),
        navigation: bijux_gnss_nav::api::parse_rinex_broadcast_navigation(&fixture(
            "noaa_brdc0140_20180114.nav",
        ))
        .expect("parse AB43 public RINEX navigation"),
    })
}

pub fn position_observations(epoch: &ObsEpoch) -> Vec<PositionObservation> {
    let gps_receive_time = epoch.gps_time();
    epoch
        .sats
        .iter()
        .map(|sat| PositionObservation {
            sat: sat.signal_id.sat,
            pseudorange_m: sat.pseudorange_m.0,
            cn0_dbhz: sat.cn0_dbhz,
            elevation_deg: sat.elevation_deg,
            weight: 1.0,
            gps_receive_time,
            signal_timing: sat.timing,
        })
        .collect()
}

pub fn solve_public_ab43_epoch(epoch: &ObsEpoch) -> Result<SolvedPublicEpoch, String> {
    let case = ab43_public_spp_case();
    let solver = PositionSolver { raim: false, apply_troposphere: true, ..PositionSolver::new() };
    let solution = solver
        .try_solve_wls_with_broadcast_ionosphere(
            &position_observations(epoch),
            &case.navigation.ephemerides,
            epoch.gps_time().expect("AB43 epoch GPS time").tow_s,
            case.navigation.klobuchar.as_ref(),
        )
        .map_err(|err| format!("{:?}", err.kind))?;

    Ok(SolvedPublicEpoch {
        gps_time: epoch.gps_time().expect("AB43 epoch GPS time"),
        ecef_m: (solution.ecef_x_m, solution.ecef_y_m, solution.ecef_z_m),
        clock_bias_s: solution.clock_bias_s,
        pre_fit_residual_rms_m: solution.pre_fit_residual_rms_m,
        post_fit_residual_rms_m: solution.post_fit_residual_rms_m,
        residuals: solution.residuals,
    })
}

fn fixture(name: &str) -> String {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/data").join(name);
    std::fs::read_to_string(&path).unwrap_or_else(|_| panic!("read fixture {}", path.display()))
}
