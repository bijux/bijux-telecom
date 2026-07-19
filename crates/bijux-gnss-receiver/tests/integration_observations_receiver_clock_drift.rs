#![allow(missing_docs)]

#[path = "support/navigation_clock_profile.rs"]
mod navigation_clock_profile;

use bijux_gnss_core::api::ObsEpoch;

use navigation_clock_profile::{
    build_navigation_clock_case, clock_profile_base_doppler_hz,
    receiver_clock_drift_doppler_offset_hz, synthetic_navigation_clock_profile,
    truth_clock_bias_by_epoch, truth_clock_drift_s_per_s,
};

const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;

#[test]
fn receiver_clock_drift_is_visible_in_observation_doppler_and_pseudorange() {
    let stable_profile = synthetic_navigation_clock_profile("stable_receiver_clock");
    let drifting_profile = synthetic_navigation_clock_profile("oscillator_drift_receiver_clock");
    let truth_clock_bias_s = truth_clock_bias_by_epoch(&drifting_profile);
    let truth_clock_drift_s_per_s = truth_clock_drift_s_per_s(&drifting_profile);
    let expected_doppler_offset_hz =
        receiver_clock_drift_doppler_offset_hz(truth_clock_drift_s_per_s);

    let stable_case = build_navigation_clock_case(stable_profile);
    let drifting_case = build_navigation_clock_case(drifting_profile);

    for drifting_epoch in &drifting_case.observations {
        let stable_epoch = stable_case
            .observations
            .iter()
            .find(|epoch| epoch.epoch_idx == drifting_epoch.epoch_idx)
            .expect("stable observation epoch");
        let truth_bias_m = truth_clock_bias_s
            .get(&drifting_epoch.epoch_idx)
            .copied()
            .expect("truth clock bias epoch")
            * SPEED_OF_LIGHT_MPS;

        for drifting_sat in &drifting_epoch.sats {
            let stable_sat = observation_satellite(stable_epoch, drifting_sat.signal_id.sat.prn);
            let expected_doppler_hz = clock_profile_base_doppler_hz(drifting_sat.signal_id.sat)
                + expected_doppler_offset_hz;

            assert!(
                (drifting_sat.doppler_hz.0 - expected_doppler_hz).abs() <= f64::EPSILON,
                "drifting observation lost the injected receiver clock Doppler offset: sat={:?} expected_doppler_hz={expected_doppler_hz}",
                drifting_sat.signal_id.sat
            );
            assert!(
                (drifting_sat.doppler_hz.0 - stable_sat.doppler_hz.0 - expected_doppler_offset_hz).abs()
                    <= f64::EPSILON,
                "stable and drifting observations disagree with the expected oscillator Doppler offset: prn={}",
                drifting_sat.signal_id.sat.prn
            );
            assert!(
                (drifting_sat.pseudorange_m.0 - stable_sat.pseudorange_m.0 - truth_bias_m).abs()
                    <= 1.0e-6,
                "stable and drifting observations disagree with the injected receiver clock bias: prn={} epoch_idx={}",
                drifting_sat.signal_id.sat.prn,
                drifting_epoch.epoch_idx
            );
        }
    }
}

fn observation_satellite(epoch: &ObsEpoch, prn: u8) -> &bijux_gnss_core::api::ObsSatellite {
    epoch.sats.iter().find(|sat| sat.signal_id.sat.prn == prn).expect("observation satellite")
}
