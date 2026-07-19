#![allow(dead_code, missing_docs)]

use std::path::PathBuf;
use std::sync::OnceLock;

use bijux_gnss_core::api::{GpsTime, ObsEpoch, SatId};
use bijux_gnss_nav::api::{
    parse_rinex_gps_observation_dataset, position_observations_from_epoch,
    sat_state_gps_l1ca_from_observation, GpsBroadcastNavigationData, PositionObservation,
    PositionSolver, RinexGpsObservationDataset,
};

use super::public_station_truth::{public_station_truth_by_fixture, PublicStationTruth};

pub struct PublicSppCase {
    pub station_truth: PublicStationTruth,
    pub observations: RinexGpsObservationDataset,
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

#[derive(Debug, Clone, PartialEq)]
pub struct PublicSatelliteGeometry {
    pub sat: SatId,
    pub ecef_m: [f64; 3],
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
    position_observations_from_epoch(epoch)
}

pub fn solve_public_ab43_epoch(epoch: &ObsEpoch) -> Result<SolvedPublicEpoch, String> {
    solve_public_ab43_epoch_with_satellites(epoch, None)
}

pub fn solve_public_ab43_epoch_with_satellites(
    epoch: &ObsEpoch,
    sats: Option<&[SatId]>,
) -> Result<SolvedPublicEpoch, String> {
    let case = ab43_public_spp_case();
    let solver = PositionSolver { raim: false, apply_troposphere: true, ..PositionSolver::new() };
    let observations = position_observations(epoch)
        .into_iter()
        .filter(|observation| sats.map(|sats| sats.contains(&observation.sat)).unwrap_or(true))
        .collect::<Vec<_>>();
    let solution = solver
        .try_solve_wls_with_gps_broadcast_navigation(
            &observations,
            &case.navigation,
            epoch.gps_time().expect("AB43 epoch GPS time").tow_s,
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

pub fn public_ab43_epoch(gps_time: GpsTime) -> Result<&'static ObsEpoch, String> {
    ab43_public_spp_case()
        .observations
        .epochs
        .iter()
        .find(|epoch| epoch.gps_time() == Some(gps_time))
        .ok_or_else(|| format!("no AB43 public epoch found at GPS time {gps_time:?}"))
}

pub fn public_ab43_satellite_geometry(
    epoch: &ObsEpoch,
    sats: &[SatId],
) -> Result<Vec<PublicSatelliteGeometry>, String> {
    let case = ab43_public_spp_case();
    let receive_tow_s = epoch.gps_time().expect("AB43 epoch GPS time").tow_s;

    sats.iter()
        .map(|sat| {
            let observation =
                epoch
                    .sats
                    .iter()
                    .find(|candidate| candidate.signal_id.sat == *sat)
                    .ok_or_else(|| format!("missing AB43 observation for satellite {sat:?}"))?;
            let ephemeris = case
                .navigation
                .ephemerides
                .iter()
                .find(|candidate| candidate.sat == *sat)
                .ok_or_else(|| format!("missing AB43 ephemeris for satellite {sat:?}"))?;
            let state = sat_state_gps_l1ca_from_observation(
                ephemeris,
                receive_tow_s,
                observation.pseudorange_m.0,
                observation.timing,
            );
            Ok(PublicSatelliteGeometry { sat: *sat, ecef_m: [state.x_m, state.y_m, state.z_m] })
        })
        .collect()
}

fn fixture(name: &str) -> String {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/data").join(name);
    std::fs::read_to_string(&path).unwrap_or_else(|_| panic!("read fixture {}", path.display()))
}
