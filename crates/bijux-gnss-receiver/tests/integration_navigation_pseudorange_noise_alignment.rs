#![allow(missing_docs)]

mod support;

use std::collections::BTreeMap;

use bijux_gnss_core::api::{Constellation, SatId};

use support::navigation_pipeline::{
    clean_synthetic_navigation_run, noisy_synthetic_navigation_run, SatellitePseudorangeNoise,
    SyntheticPseudorangeNoiseProfile,
};

const PSEUDORANGE_NOISE_TOLERANCE_M: f64 = 1.0e-6;

#[test]
fn noisy_synthetic_navigation_preserves_injected_pseudorange_offsets() {
    let clean_run = clean_synthetic_navigation_run();
    let noise_profile = SyntheticPseudorangeNoiseProfile {
        profile_name: "observation_offset_alignment",
        satellites: vec![
            satellite_noise(3, -2.0),
            satellite_noise(7, 0.5),
            satellite_noise(11, 1.5),
            satellite_noise(19, -0.75),
            satellite_noise(23, 2.25),
        ],
    };
    let noisy_run = noisy_synthetic_navigation_run(noise_profile.clone());
    let clean_pseudoranges = pseudorange_by_satellite(&clean_run);
    let noisy_pseudoranges = pseudorange_by_satellite(&noisy_run.run);

    for injected in &noise_profile.satellites {
        let clean_pseudorange_m = clean_pseudoranges
            .get(&injected.sat)
            .unwrap_or_else(|| panic!("missing clean pseudorange for {:?}", injected.sat));
        let noisy_pseudorange_m = noisy_pseudoranges
            .get(&injected.sat)
            .unwrap_or_else(|| panic!("missing noisy pseudorange for {:?}", injected.sat));
        let observed_delta_m = noisy_pseudorange_m - clean_pseudorange_m;

        assert!(
            (observed_delta_m - injected.pseudorange_noise_m).abs() <= PSEUDORANGE_NOISE_TOLERANCE_M,
            "expected {:?} to carry {} m, observed {} m",
            injected.sat,
            injected.pseudorange_noise_m,
            observed_delta_m,
        );
    }
}

fn pseudorange_by_satellite(
    run: &support::navigation_pipeline::CleanSyntheticNavigationRun,
) -> BTreeMap<SatId, f64> {
    run.observations
        .iter()
        .flat_map(|epoch| epoch.sats.iter())
        .map(|satellite| (satellite.signal_id.sat, satellite.pseudorange_m.0))
        .collect()
}

fn satellite_noise(prn: u8, pseudorange_noise_m: f64) -> SatellitePseudorangeNoise {
    SatellitePseudorangeNoise {
        sat: SatId { constellation: Constellation::Gps, prn },
        pseudorange_noise_m,
    }
}
