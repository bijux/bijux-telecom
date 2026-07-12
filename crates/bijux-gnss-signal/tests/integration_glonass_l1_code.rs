use bijux_gnss_signal::api::{
    generate_glonass_l1_st_code, periodic_correlation, sample_glonass_l1_st_code,
    GLONASS_L1_ST_CODE_CHIPS, GLONASS_L1_ST_CODE_RATE_HZ,
};

#[test]
fn glonass_l1_period_has_m_sequence_autocorrelation() {
    let code = generate_glonass_l1_st_code();
    let autocorrelation = periodic_correlation(&code, &code).expect("valid autocorrelation");
    let (peak, sidelobes) =
        autocorrelation.split_first().expect("GLONASS L1 autocorrelation must not be empty");

    assert_eq!(*peak, GLONASS_L1_ST_CODE_CHIPS as i16);
    assert!(sidelobes.iter().all(|value| *value == -1));
}

#[test]
fn glonass_l1_fractional_sampling_matches_integer_chip_values() {
    let sample_count = 16;
    let start_chip_phase = 3.5;
    let samples =
        sample_glonass_l1_st_code(GLONASS_L1_ST_CODE_RATE_HZ * 2.0, start_chip_phase, sample_count)
            .expect("GLONASS L1 fractional sampling must succeed");
    let code = generate_glonass_l1_st_code();
    let expected = (0..sample_count)
        .map(|sample_index| {
            let chip_phase = start_chip_phase + sample_index as f64 * 0.5;
            code[chip_phase.floor() as usize % GLONASS_L1_ST_CODE_CHIPS] as f32
        })
        .collect::<Vec<_>>();

    assert_eq!(samples.len(), sample_count);
    assert!(samples.iter().all(|sample| sample.abs() == 1.0));
    assert_eq!(samples, expected);
}
