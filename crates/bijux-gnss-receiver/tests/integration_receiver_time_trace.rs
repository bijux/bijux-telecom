#![allow(missing_docs)]

use bijux_gnss_core::api::{Constellation, ReceiverSampleTrace, SampleTime, SatId};
use bijux_gnss_receiver::api::{
    observations_from_tracking_results,
    sim::{generate_l1_ca, SyntheticSignalParams},
    AcquisitionEngine, Navigation, ReceiverPipelineConfig, ReceiverRuntime, TrackingEngine,
};

#[test]
fn receiver_pipeline_preserves_sample_trace_across_stage_outputs() {
    let config = ReceiverPipelineConfig::default();
    let runtime = ReceiverRuntime::default();
    let sat = SatId { constellation: Constellation::Gps, prn: 7 };
    let mut frame = generate_l1_ca(
        &config,
        SyntheticSignalParams {
            sat,
            glonass_frequency_channel: None,
            signal_band: bijux_gnss_core::api::SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Ca,
            doppler_hz: 750.0,
            code_phase_chips: 0.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 50.0,
            navigation_data: false.into(),
        },
        0xCAFE_BABE,
        0.012,
    );
    frame.t0 = SampleTime { sample_index: 4_092, sample_rate_hz: config.sampling_freq_hz };
    let expected_frame_trace = ReceiverSampleTrace::from_sample_time(frame.t0);

    let acquisition =
        AcquisitionEngine::new(config.clone(), runtime.clone()).run_fft(&frame, &[sat]);
    assert_eq!(acquisition.len(), 1);
    assert_eq!(acquisition[0].source_time, expected_frame_trace);

    let tracking = TrackingEngine::new(config.clone(), runtime.clone());
    let tracks = tracking.track_from_acquisition(&frame, &acquisition);
    let first_track = tracks.first().expect("tracking result");
    assert!(!first_track.epochs.is_empty(), "tracking must emit epochs");
    for epoch in &first_track.epochs {
        assert_eq!(epoch.source_time.sample_index, epoch.sample_index);
        assert_eq!(
            epoch.source_time,
            ReceiverSampleTrace::from_sample_index(epoch.sample_index, config.sampling_freq_hz)
        );
    }

    let observation_report = observations_from_tracking_results(&config, &tracks, 10);
    assert_eq!(observation_report.output.len(), first_track.epochs.len());
    for (track_epoch, obs_epoch) in first_track.epochs.iter().zip(observation_report.output.iter())
    {
        assert_eq!(obs_epoch.source_time.sample_index, track_epoch.sample_index);
        assert_eq!(obs_epoch.source_time.receiver_time_s, obs_epoch.t_rx_s);
        let manifest = obs_epoch.manifest.as_ref().expect("observation manifest");
        assert_eq!(manifest.source_time, obs_epoch.source_time);
        assert_eq!(manifest.source_sample_index, obs_epoch.source_time.sample_index);
    }

    let mut navigation = Navigation::new(config.clone(), runtime);
    let nav_solution = navigation
        .solve_epoch(observation_report.output.first().expect("observation epoch"), &[])
        .expect("navigation solution");
    let obs_epoch = &observation_report.output[0];
    assert_eq!(nav_solution.source_time, obs_epoch.source_time);
    assert_eq!(nav_solution.t_rx_s, obs_epoch.t_rx_s);
}
