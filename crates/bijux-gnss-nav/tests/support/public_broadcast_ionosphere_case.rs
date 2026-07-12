#![allow(dead_code, missing_docs)]

use std::path::PathBuf;
use std::sync::OnceLock;

use bijux_gnss_nav::api::{
    gps_broadcast_ionosphere_residuals_from_obs_epochs, parse_rinex_broadcast_navigation,
    parse_rinex_gps_observation_dataset, summarize_broadcast_ionosphere_residuals,
    BroadcastIonosphereResidualObservation, BroadcastIonosphereResidualSummary,
    GpsBroadcastNavigationData, RinexGpsObservationDataset,
};
use bijux_gnss_core::api::SignalBand;

pub struct PublicGpsBroadcastIonosphereCase {
    pub observations: RinexGpsObservationDataset,
    pub navigation: GpsBroadcastNavigationData,
    pub receiver_ecef_m: (f64, f64, f64),
    pub residuals: Vec<BroadcastIonosphereResidualObservation>,
    pub summary: BroadcastIonosphereResidualSummary,
}

pub fn ab43_public_gps_broadcast_ionosphere_case() -> &'static PublicGpsBroadcastIonosphereCase {
    static CASE: OnceLock<PublicGpsBroadcastIonosphereCase> = OnceLock::new();
    CASE.get_or_init(|| {
        let observations = parse_rinex_gps_observation_dataset(&fixture("unavco_ab43_20180114.obs"))
            .expect("parse AB43 public RINEX observations");
        let navigation = parse_rinex_broadcast_navigation(&fixture("noaa_brdc0140_20180114.nav"))
            .expect("parse AB43 public RINEX navigation");
        let receiver_ecef_m = observations
            .approx_position_ecef_m
            .expect("AB43 public RINEX header should provide approximate receiver coordinates");
        let residuals = gps_broadcast_ionosphere_residuals_from_obs_epochs(
            &observations.epochs,
            receiver_ecef_m,
            &navigation,
            SignalBand::L1,
            SignalBand::L2,
        );
        let summary = summarize_broadcast_ionosphere_residuals(&residuals);

        PublicGpsBroadcastIonosphereCase {
            observations,
            navigation,
            receiver_ecef_m,
            residuals,
            summary,
        }
    })
}

pub fn valid_broadcast_residuals() -> Vec<&'static BroadcastIonosphereResidualObservation> {
    ab43_public_gps_broadcast_ionosphere_case()
        .residuals
        .iter()
        .filter(|observation| observation.broadcast_status == "ok")
        .collect()
}

fn fixture(name: &str) -> String {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/data").join(name);
    std::fs::read_to_string(&path).unwrap_or_else(|_| panic!("read fixture {}", path.display()))
}
