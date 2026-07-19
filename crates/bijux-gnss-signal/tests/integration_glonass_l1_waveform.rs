use bijux_gnss_signal::api::{
    glonass_l1_string_symbol, glonass_l1_string_symbol_at_time_s, GLONASS_L1_STRING_DATA_BITS,
    GLONASS_L1_STRING_DATA_SYMBOLS, GLONASS_L1_STRING_SYMBOLS, GLONASS_L1_SYMBOL_PERIOD_S,
    GLONASS_L1_TIME_MARK,
};

#[test]
fn public_api_aligns_glonass_symbol_times_to_10ms_boundaries() {
    let raw_data_bits = [1; GLONASS_L1_STRING_DATA_BITS];

    for symbol_index in [0usize, 1, 2, 169, 170, 199, 200, 201] {
        let time_s = symbol_index as f64 * GLONASS_L1_SYMBOL_PERIOD_S;
        let at_index = glonass_l1_string_symbol(&raw_data_bits, symbol_index).expect("symbol");
        let at_time =
            glonass_l1_string_symbol_at_time_s(&raw_data_bits, time_s).expect("symbol at time");

        assert_eq!(at_time, at_index, "symbol mismatch at {time_s:.3} s");
    }
}

#[test]
fn public_api_expands_relative_data_symbols_with_meander_over_first_1p7_seconds() {
    let mut raw_data_bits = [1; GLONASS_L1_STRING_DATA_BITS];
    for (index, bit) in raw_data_bits.iter_mut().enumerate().skip(1) {
        *bit = if index % 2 == 0 { 1 } else { -1 };
    }

    assert_eq!(glonass_l1_string_symbol(&raw_data_bits, 0).expect("symbol"), 1);
    assert_eq!(glonass_l1_string_symbol(&raw_data_bits, 1).expect("symbol"), -1);
    assert_eq!(glonass_l1_string_symbol(&raw_data_bits, 2).expect("symbol"), -1);
    assert_eq!(glonass_l1_string_symbol(&raw_data_bits, 3).expect("symbol"), 1);
    assert_eq!(
        glonass_l1_string_symbol(&raw_data_bits, GLONASS_L1_STRING_DATA_SYMBOLS - 1)
            .expect("last data symbol"),
        1
    );
}

#[test]
fn public_api_emits_time_mark_during_last_300ms_of_each_string() {
    let raw_data_bits = [1; GLONASS_L1_STRING_DATA_BITS];

    for (offset, expected) in GLONASS_L1_TIME_MARK.iter().enumerate() {
        let symbol =
            glonass_l1_string_symbol(&raw_data_bits, GLONASS_L1_STRING_DATA_SYMBOLS + offset)
                .expect("time mark symbol");
        assert_eq!(symbol, *expected, "time mark mismatch at offset {offset}");
    }

    assert_eq!(
        glonass_l1_string_symbol(&raw_data_bits, GLONASS_L1_STRING_SYMBOLS)
            .expect("wrapped symbol"),
        1
    );
}
