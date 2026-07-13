use bijux_gnss_signal::api::{
    generate_beidou_b1i_code, periodic_correlation, sample_beidou_b1i_code, BEIDOU_B1I_CODE_CHIPS,
    BEIDOU_B1I_CODE_RATE_HZ,
};

#[test]
fn beidou_b1i_period_has_expected_autocorrelation_peak() {
    let code = generate_beidou_b1i_code(11).expect("valid B1I PRN");
    let autocorrelation = periodic_correlation(&code, &code).expect("valid autocorrelation");
    let (peak, sidelobes) =
        autocorrelation.split_first().expect("BeiDou B1I autocorrelation must not be empty");

    assert_eq!(*peak, BEIDOU_B1I_CODE_CHIPS as i16);
    assert!(sidelobes.iter().all(|value| value.abs() < *peak));
}

#[test]
fn beidou_b1i_fractional_sampling_matches_integer_chip_values() {
    let sample_count = 16;
    let start_chip_phase = 5.5;
    let samples =
        sample_beidou_b1i_code(19, BEIDOU_B1I_CODE_RATE_HZ * 2.0, start_chip_phase, sample_count)
            .expect("BeiDou B1I fractional sampling must succeed");
    let code = generate_beidou_b1i_code(19).expect("valid B1I PRN");
    let expected = (0..sample_count)
        .map(|sample_index| {
            let chip_phase = start_chip_phase + sample_index as f64 * 0.5;
            code[chip_phase.floor() as usize % BEIDOU_B1I_CODE_CHIPS] as f32
        })
        .collect::<Vec<_>>();

    assert_eq!(samples.len(), sample_count);
    assert!(samples.iter().all(|sample| sample.abs() == 1.0));
    assert_eq!(samples, expected);
}
