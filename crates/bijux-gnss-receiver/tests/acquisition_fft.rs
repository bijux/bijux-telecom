use bijux_gnss_receiver::{
    acquisition::Acquisition,
    signal::samples_per_code,
    synthetic::{generate_l1_ca, SyntheticSignalParams},
    types::ReceiverConfig,
};

#[test]
fn acquisition_fft_detects_synthetic_signal() {
    let config = ReceiverConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 12,
        ..ReceiverConfig::default()
    };

    let samples_per_code = samples_per_code(
        config.sampling_freq_hz,
        config.code_freq_basis_hz,
        config.code_length,
    );

    let sat = bijux_gnss_core::SatId {
        constellation: bijux_gnss_core::Constellation::Gps,
        prn: 1,
    };
    let frame = generate_l1_ca(
        &config,
        SyntheticSignalParams {
            sat,
            doppler_hz: 0.0,
            code_phase_chips: 100.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 50.0,
            data_bit_flip: false,
        },
        0x1234_5678,
        samples_per_code as f64 / config.sampling_freq_hz,
    );

    let acquisition = Acquisition::new(config).with_doppler(0, 500);
    let results = acquisition.run_fft(&frame, &[sat]);

    let r = &results[0];
    assert_eq!(r.sat, sat);
    assert!(r.peak_mean_ratio > 2.0);
}
