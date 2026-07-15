use super::*;

#[test]
fn code_fft_cache_reuses_local_code_across_integration_profiles() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let sat = SatId { constellation: Constellation::Gps, prn: 11 };
    let samples_per_code =
        samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length);
    let acquisition = Acquisition::new(config, ReceiverRuntime::default());
    let mut planner = FftPlanner::<f32>::new();
    let fft = planner.plan_fft_forward(samples_per_code);
    let signal_model = acquisition_signal_model_for_sat(
        &acquisition.config,
        sat,
        SignalBand::L1,
        SignalCode::Unknown,
        None,
    );
    let component = acquisition_component_plan_for_signal(sat, SignalBand::L1, SignalCode::Ca, 1);

    acquisition.code_fft(
        &signal_model,
        &component,
        sat,
        SignalCode::Ca,
        samples_per_code,
        1,
        1,
        fft.as_ref(),
    );
    let after_first_profile = acquisition.stats_snapshot();
    assert_eq!(after_first_profile.cache_misses, 1);
    assert_eq!(after_first_profile.cache_hits, 0);

    acquisition.code_fft(
        &signal_model,
        &component,
        sat,
        SignalCode::Ca,
        samples_per_code,
        1,
        4,
        fft.as_ref(),
    );
    let after_second_profile = acquisition.stats_snapshot();
    assert_eq!(after_second_profile.cache_misses, 1);
    assert_eq!(after_second_profile.cache_hits, 1);
    assert_eq!(after_second_profile.cache_miss_incompatible, 0);
}

#[test]
fn code_fft_cache_separates_gps_l5_signal_codes() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 10_230_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 10_230_000.0,
        code_length: 10_230,
        ..ReceiverPipelineConfig::default()
    };
    let sat = SatId { constellation: Constellation::Gps, prn: 7 };
    let acquisition = Acquisition::new(config, ReceiverRuntime::default());
    let samples_per_code = 10_230;
    let mut planner = FftPlanner::<f32>::new();
    let fft = planner.plan_fft_forward(samples_per_code);
    let gps_l5_i = acquisition_signal_model_for_sat(
        &acquisition.config,
        sat,
        SignalBand::L5,
        SignalCode::L5I,
        None,
    );
    let gps_l5_q = acquisition_signal_model_for_sat(
        &acquisition.config,
        sat,
        SignalBand::L5,
        SignalCode::L5Q,
        None,
    );
    let gps_l5_i_component =
        acquisition_component_plan_for_signal(sat, SignalBand::L5, SignalCode::L5I, 1);
    let gps_l5_q_component =
        acquisition_component_plan_for_signal(sat, SignalBand::L5, SignalCode::L5Q, 1);

    acquisition.code_fft(
        &gps_l5_i,
        &gps_l5_i_component,
        sat,
        SignalCode::L5I,
        samples_per_code,
        1,
        1,
        fft.as_ref(),
    );
    let after_l5_i = acquisition.stats_snapshot();
    assert_eq!(after_l5_i.cache_misses, 1);
    assert_eq!(after_l5_i.cache_hits, 0);

    acquisition.code_fft(
        &gps_l5_q,
        &gps_l5_q_component,
        sat,
        SignalCode::L5Q,
        samples_per_code,
        1,
        1,
        fft.as_ref(),
    );
    let after_l5_q = acquisition.stats_snapshot();
    assert_eq!(after_l5_q.cache_misses, 2);
    assert_eq!(after_l5_q.cache_hits, 0);
    assert_eq!(after_l5_q.cache_miss_incompatible, 1);

    acquisition.code_fft(
        &gps_l5_q,
        &gps_l5_q_component,
        sat,
        SignalCode::L5Q,
        samples_per_code,
        1,
        1,
        fft.as_ref(),
    );
    let after_l5_q_reuse = acquisition.stats_snapshot();
    assert_eq!(after_l5_q_reuse.cache_misses, 2);
    assert_eq!(after_l5_q_reuse.cache_hits, 1);
}
