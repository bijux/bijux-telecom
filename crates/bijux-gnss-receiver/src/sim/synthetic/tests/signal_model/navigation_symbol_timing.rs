#[test]
fn alternating_nav_bit_sign_flips_on_twenty_millisecond_boundaries() {
    assert_eq!(nav_bit_index_at_time_s(-1.0), 0);
    assert_eq!(nav_bit_index_at_time_s(0.0), 0);
    assert_eq!(nav_bit_index_at_time_s(0.019_999_999), 0);
    assert_eq!(nav_bit_index_at_time_s(0.020_000_000), 1);
    assert_eq!(nav_bit_index_at_time_s(0.039_999_999), 1);
    assert_eq!(nav_bit_index_at_time_s(0.040_000_000), 2);

    assert_eq!(nav_bit_sign_at_time_s(false, 0.0), 1);
    assert_eq!(nav_bit_sign_at_time_s(true, 0.0), 1);
    assert_eq!(nav_bit_sign_at_time_s(true, 0.019_999_999), 1);
    assert_eq!(nav_bit_sign_at_time_s(true, 0.020_000_000), -1);
    assert_eq!(nav_bit_sign_at_time_s(true, 0.039_999_999), -1);
    assert_eq!(nav_bit_sign_at_time_s(true, 0.040_000_000), 1);
}
