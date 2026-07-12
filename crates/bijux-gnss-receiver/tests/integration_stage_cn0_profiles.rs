#![allow(missing_docs)]

use bijux_gnss_core::api::{Constellation, SatId};
use bijux_gnss_receiver::api::{
    sim::{
        measure_truth_guided_acquisition_detection_rate, measure_truth_guided_tracking_lock_rate,
        SyntheticAcquisitionDetectionRateCase, SyntheticSignalParams,
        SyntheticTrackingLockRateCase,
    },
    ReceiverPipelineConfig,
};

#[test]
fn acquisition_and_tracking_profiles_improve_with_stronger_signal() {
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
    let acquisition = measure_truth_guided_acquisition_detection_rate(
        &config,
        &[acquisition_case(sat, 24.0), acquisition_case(sat, 32.0), acquisition_case(sat, 52.0)],
        &[17, 29, 43, 59],
        "stage_cn0_profile_acquisition",
        2,
        1,
    );
    let tracking = measure_truth_guided_tracking_lock_rate(
        &config,
        &[tracking_case(sat, 24.0), tracking_case(sat, 32.0), tracking_case(sat, 52.0)],
        &[17, 29, 43, 59],
        "stage_cn0_profile_tracking",
    );

    assert_eq!(acquisition.points.len(), 3);
    assert_eq!(tracking.points.len(), 3);
    assert!(acquisition.points[0].cn0_db_hz < acquisition.points[1].cn0_db_hz);
    assert!(acquisition.points[1].cn0_db_hz < acquisition.points[2].cn0_db_hz);
    assert!(tracking.points[0].cn0_db_hz < tracking.points[1].cn0_db_hz);
    assert!(tracking.points[1].cn0_db_hz < tracking.points[2].cn0_db_hz);

    let weak_acquisition = &acquisition.points[0];
    let strong_acquisition = &acquisition.points[2];
    assert!(
        strong_acquisition.detection_probability > weak_acquisition.detection_probability,
        "{acquisition:?}"
    );
    assert!(
        strong_acquisition.acceptance_probability >= weak_acquisition.acceptance_probability,
        "{acquisition:?}"
    );

    let weak_tracking = &tracking.points[0];
    let strong_tracking = &tracking.points[2];
    assert!(strong_tracking.lock_probability > weak_tracking.lock_probability, "{tracking:?}");
    assert!(strong_tracking.mean_locked_epochs >= weak_tracking.mean_locked_epochs, "{tracking:?}");
}

fn acquisition_case(sat: SatId, cn0_db_hz: f32) -> SyntheticAcquisitionDetectionRateCase {
    SyntheticAcquisitionDetectionRateCase {
        signal: SyntheticSignalParams {
            sat,
            glonass_frequency_channel: None,
            doppler_hz: 250.0,
            code_phase_chips: 300.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz,
            data_bit_flip: false,
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
            doppler_hz: 180.0,
            code_phase_chips: 211.25,
            carrier_phase_rad: 0.0,
            cn0_db_hz,
            data_bit_flip: false,
        },
        duration_s: 0.08,
        seeded_doppler_error_hz: 0.0,
        seeded_code_phase_error_samples: 0,
        min_locked_epochs: 1,
    }
}
