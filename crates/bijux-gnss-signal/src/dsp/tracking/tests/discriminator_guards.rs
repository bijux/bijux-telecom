use super::*;

#[test]
fn anti_false_lock_detected_rejects_early_late_energy_near_prompt() {
    assert!(anti_false_lock_detected(
        Complex::new(8.0, 0.0),
        Complex::new(8.0, 0.0),
        Complex::new(8.0, 0.0),
    ));
    assert!(!anti_false_lock_detected(
        Complex::new(1.0, 0.0),
        Complex::new(8.0, 0.0),
        Complex::new(1.0, 0.0),
    ));
}

#[test]
fn anti_false_lock_detected_accepts_clean_tracking_geometry_below_guard_band() {
    assert!(!anti_false_lock_detected(
        Complex::new(7.44, 0.0),
        Complex::new(8.0, 0.0),
        Complex::new(7.44, 0.0),
    ));
}

#[test]
fn normalize_dll_discriminator_preserves_reference_spacing_gain() {
    assert_eq!(normalize_dll_discriminator(0.25, 0.5), 0.25);
}

#[test]
fn normalize_dll_discriminator_scales_narrow_correlator_gain() {
    let raw = 0.10;
    let standard = normalize_dll_discriminator(raw, 0.5);
    let narrow = normalize_dll_discriminator(raw, 0.25);

    assert!(
        narrow > standard,
        "narrow spacing must compensate discriminator gain: standard={standard} narrow={narrow}",
    );
    assert!((narrow - 0.116_666_67).abs() <= f32::EPSILON);
}

#[test]
fn dll_discriminator_from_early_late_matches_tracking_discriminators() {
    let early = Complex::new(3.0, 4.0);
    let prompt = Complex::new(10.0, 0.0);
    let late = Complex::new(1.0, 0.0);

    let (dll, _, _, _) = super::discriminators(early, prompt, late, None);

    assert_eq!(dll, dll_discriminator_from_early_late(early, late));
}

#[test]
fn double_delta_dll_discriminator_suppresses_outer_lobe_bias() {
    let inner_early = Complex::new(8.0, 0.0);
    let inner_late = Complex::new(4.0, 0.0);
    let outer_early = Complex::new(6.0, 0.0);
    let outer_late = Complex::new(2.0, 0.0);

    let early_late = dll_discriminator_from_early_late(inner_early, inner_late);
    let double_delta =
        double_delta_dll_discriminator(inner_early, inner_late, outer_early, outer_late);

    assert!(double_delta.abs() < early_late.abs());
    assert!((double_delta - 0.166_666_67).abs() <= f32::EPSILON);
}
