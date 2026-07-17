#![allow(missing_docs)]

#[path = "support/navigation_cn0_profile.rs"]
mod navigation_cn0_profile;
#[path = "support/navigation_truth.rs"]
mod navigation_truth;

use bijux_gnss_core::api::{Constellation, SatId};
use bijux_gnss_receiver::api::{
    sim::{
        measure_truth_guided_acquisition_detection_rate, measure_truth_guided_tracking_lock_rate,
        summarize_truth_guided_accuracy_cn0_profile, summarize_truth_guided_pvt_cn0_profile,
        SyntheticAcquisitionDetectionRateCase, SyntheticPvtCn0ProfileCase, SyntheticSignalParams,
        SyntheticTrackingLockRateCase,
    },
    ReceiverPipelineConfig,
};

use navigation_cn0_profile::build_truth_seeded_navigation_cn0_case;

#[test]
fn merged_accuracy_profile_includes_acquisition_tracking_and_pvt_by_signal_strength() {
    let weak_case = build_truth_seeded_navigation_cn0_case(30.0, "accuracy_cn0_profile");
    let medium_case = build_truth_seeded_navigation_cn0_case(40.0, "accuracy_cn0_profile");
    let strong_case = build_truth_seeded_navigation_cn0_case(52.0, "accuracy_cn0_profile");
    let pvt = summarize_truth_guided_pvt_cn0_profile(
        &[
            SyntheticPvtCn0ProfileCase {
                scenario_id: &strong_case.scenario_id,
                observations: &strong_case.observations,
                accuracy: &strong_case.pvt_accuracy,
            },
            SyntheticPvtCn0ProfileCase {
                scenario_id: &weak_case.scenario_id,
                observations: &weak_case.observations,
                accuracy: &weak_case.pvt_accuracy,
            },
            SyntheticPvtCn0ProfileCase {
                scenario_id: &medium_case.scenario_id,
                observations: &medium_case.observations,
                accuracy: &medium_case.pvt_accuracy,
            },
        ],
        "accuracy_cn0_profile",
    );

    let sat = SatId { constellation: Constellation::Gps, prn: 7 };
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 4,
        acquisition_doppler_search_hz: 1_500,
        acquisition_doppler_step_hz: 250,
        early_late_spacing_chips: 0.5,
        dll_bw_hz: 2.0,
        pll_bw_hz: 18.0,
        fll_bw_hz: 12.0,
        ..ReceiverPipelineConfig::default()
    };
    let acquisition_cases = pvt
        .points
        .iter()
        .map(|point| acquisition_case(sat, round_cn0_to_tenth(point.mean_observation_cn0_dbhz)))
        .collect::<Vec<_>>();
    let tracking_cases = pvt
        .points
        .iter()
        .map(|point| tracking_case(sat, round_cn0_to_tenth(point.mean_observation_cn0_dbhz)))
        .collect::<Vec<_>>();
    let acquisition = measure_truth_guided_acquisition_detection_rate(
        &config,
        &acquisition_cases,
        &[17, 29, 43],
        "accuracy_cn0_profile_acquisition",
        2,
        1,
    );
    let tracking = measure_truth_guided_tracking_lock_rate(
        &config,
        &tracking_cases,
        &[17, 29, 43],
        "accuracy_cn0_profile_tracking",
    );
    let report = summarize_truth_guided_accuracy_cn0_profile(
        "accuracy_cn0_profile",
        &acquisition,
        &tracking,
        &pvt,
    );

    assert_eq!(report.points.len(), 3, "{report:?}");
    assert!(report.points.iter().all(|point| point.acquisition_case_count == 1), "{report:?}");
    assert!(report.points.iter().all(|point| point.tracking_case_count == 1), "{report:?}");
    assert!(report.points.iter().all(|point| point.pvt_case_count == 1), "{report:?}");

    assert!(report.points.windows(2).all(|window| window[0].cn0_db_hz < window[1].cn0_db_hz));

    for point in &report.points {
        let acquisition_point = acquisition
            .points
            .iter()
            .find(|candidate| cn0_tenths(candidate.cn0_db_hz as f64) == cn0_tenths(point.cn0_db_hz))
            .expect("acquisition point for merged cn0 bin");
        let tracking_point = tracking
            .points
            .iter()
            .find(|candidate| cn0_tenths(candidate.cn0_db_hz as f64) == cn0_tenths(point.cn0_db_hz))
            .expect("tracking point for merged cn0 bin");
        let pvt_point = pvt
            .points
            .iter()
            .find(|candidate| {
                cn0_tenths(candidate.mean_observation_cn0_dbhz) == cn0_tenths(point.cn0_db_hz)
            })
            .expect("pvt point for merged cn0 bin");

        assert_eq!(point.acquisition_trial_count, acquisition_point.trial_count, "{report:?}");
        assert_eq!(point.tracking_trial_count, tracking_point.trial_count, "{report:?}");
        assert_eq!(point.pvt_epoch_count, pvt_point.epoch_count, "{report:?}");
        assert_eq!(
            point.acquisition_detection_probability_mean,
            Some(acquisition_point.detection_probability),
            "{report:?}"
        );
        assert_eq!(
            point.tracking_lock_probability_mean,
            Some(tracking_point.lock_probability),
            "{report:?}"
        );
        assert_eq!(point.pvt_pass_rate_mean, Some(pvt_point.pass_rate), "{report:?}");
    }
}

fn acquisition_case(sat: SatId, cn0_db_hz: f32) -> SyntheticAcquisitionDetectionRateCase {
    SyntheticAcquisitionDetectionRateCase {
        signal: SyntheticSignalParams {
            sat,
            glonass_frequency_channel: None,
            signal_band: bijux_gnss_core::api::SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Ca,
            doppler_hz: 250.0,
            code_phase_chips: 300.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz,
            navigation_data: false.into(),
        },
        coherent_ms: 1,
        noncoherent: 1,
    }
}

fn tracking_case(sat: SatId, cn0_db_hz: f32) -> SyntheticTrackingLockRateCase {
    SyntheticTrackingLockRateCase {
        signal: SyntheticSignalParams {
            sat,
            glonass_frequency_channel: None,
            signal_band: bijux_gnss_core::api::SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Ca,
            doppler_hz: 180.0,
            code_phase_chips: 211.25,
            carrier_phase_rad: 0.0,
            cn0_db_hz,
            navigation_data: false.into(),
        },
        duration_s: 0.08,
        seeded_doppler_error_hz: 0.0,
        seeded_code_phase_error_samples: 0,
        min_locked_epochs: 1,
    }
}

fn round_cn0_to_tenth(value: f64) -> f32 {
    ((value * 10.0).round() / 10.0) as f32
}

fn cn0_tenths(value: f64) -> i32 {
    (value * 10.0).round() as i32
}
