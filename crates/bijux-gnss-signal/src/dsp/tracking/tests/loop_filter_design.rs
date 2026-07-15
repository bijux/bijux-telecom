use super::*;

#[test]
fn first_order_loop_coefficients_scale_with_bandwidth_and_integration_time() {
    let narrow = first_order_loop_coefficients(2.0, 0.001);
    let wide = first_order_loop_coefficients(10.0, 0.001);
    let long = first_order_loop_coefficients(2.0, 0.020);

    assert!(wide.error_blend > narrow.error_blend, "{wide:?} {narrow:?}");
    assert!(wide.rate_gain_hz > narrow.rate_gain_hz, "{wide:?} {narrow:?}");
    assert!(long.error_blend > narrow.error_blend, "{long:?} {narrow:?}");
    assert!(long.rate_gain_hz < narrow.rate_gain_hz, "{long:?} {narrow:?}");
}

#[test]
fn first_order_loop_design_matches_requested_noise_bandwidth() {
    let coherent_integration_s = 0.001;
    let target_bandwidth_hz = 12.5;
    let design = super::super::loop_observer_design(
        super::super::LoopFilterOrder::First,
        target_bandwidth_hz,
        coherent_integration_s,
    );
    let measured_bandwidth_hz = super::super::loop_noise_bandwidth_hz(
        super::super::LoopFilterOrder::First,
        coherent_integration_s,
        design.correction_gains,
    );

    assert!(
        (measured_bandwidth_hz - target_bandwidth_hz).abs() <= 1.0e-6,
        "design={design:?} measured_bandwidth_hz={measured_bandwidth_hz}",
    );
}

#[test]
fn first_order_loop_design_places_requested_closed_loop_pole() {
    let coherent_integration_s = 0.001;
    let design = super::super::loop_observer_design(
        super::super::LoopFilterOrder::First,
        8.0,
        coherent_integration_s,
    );
    let transition = super::super::corrected_loop_transition(
        super::super::LoopFilterOrder::First,
        coherent_integration_s,
        design.correction_gains,
    );

    assert!(
        (transition[0][0] - design.closed_loop_pole).abs() <= 1.0e-12,
        "design={design:?} transition={transition:?}",
    );
}

fn loop_observer_response<F>(
    order: super::super::LoopFilterOrder,
    correction_gains: [f64; 3],
    coherent_integration_s: f64,
    epochs: usize,
    measurement_at_epoch: F,
) -> Vec<[f64; 3]>
where
    F: Fn(usize) -> f64,
{
    let dimension = order.as_usize();
    let transition =
        super::super::corrected_loop_transition(order, coherent_integration_s, correction_gains);
    let mut state = [0.0; 3];
    let mut response = Vec::with_capacity(epochs);
    for epoch in 0..epochs {
        let measurement = measurement_at_epoch(epoch);
        let mut next_state = [0.0; 3];
        for row in 0..dimension {
            next_state[row] = correction_gains[row] * measurement;
            for column in 0..dimension {
                next_state[row] += transition[row][column] * state[column];
            }
        }
        state = next_state;
        response.push(state);
    }
    response
}

#[test]
fn loop_filter_step_responses_converge_for_each_order() {
    let coherent_integration_s = 0.001;
    let cases = [
        (super::super::LoopFilterOrder::First, 8.0),
        (super::super::LoopFilterOrder::Second, 6.0),
        (super::super::LoopFilterOrder::Third, 18.0),
    ];

    for (order, bandwidth_hz) in cases {
        let design =
            super::super::loop_observer_design(order, bandwidth_hz, coherent_integration_s);
        let response = loop_observer_response(
            order,
            design.correction_gains,
            coherent_integration_s,
            6_000,
            |_| 1.0,
        );

        let final_output = response.last().expect("step response sample")[0];
        let peak_output = response.iter().map(|state| state[0]).fold(f64::NEG_INFINITY, f64::max);
        assert!(
            (final_output - 1.0).abs() <= 1.0e-8,
            "order={order:?} design={design:?} final_output={final_output}",
        );
        assert!(peak_output <= 1.25, "order={order:?} design={design:?} peak_output={peak_output}",);
    }
}

#[test]
fn loop_filter_ramp_responses_match_order_capability() {
    let coherent_integration_s = 0.001;
    let ramp_rate_units_per_s = 0.75;
    let cases = [
        (super::super::LoopFilterOrder::First, 8.0),
        (super::super::LoopFilterOrder::Second, 6.0),
        (super::super::LoopFilterOrder::Third, 18.0),
    ];

    for (order, bandwidth_hz) in cases {
        let design =
            super::super::loop_observer_design(order, bandwidth_hz, coherent_integration_s);
        let response = loop_observer_response(
            order,
            design.correction_gains,
            coherent_integration_s,
            12_000,
            |epoch| epoch as f64 * coherent_integration_s * ramp_rate_units_per_s,
        );
        let final_state = response.last().expect("ramp response sample");
        let final_measurement =
            (response.len() - 1) as f64 * coherent_integration_s * ramp_rate_units_per_s;
        let final_error = final_measurement - final_state[0];

        match order {
            super::super::LoopFilterOrder::First => {
                let ramp_step_units = ramp_rate_units_per_s * coherent_integration_s;
                let expected_lag =
                    design.closed_loop_pole * ramp_step_units / design.correction_gains[0];
                assert!(
                    (final_error - expected_lag).abs() <= 1.0e-8,
                    "design={design:?} final_error={final_error} expected_lag={expected_lag}",
                );
            }
            super::super::LoopFilterOrder::Second | super::super::LoopFilterOrder::Third => {
                let next_epoch_measurement =
                    final_measurement + ramp_rate_units_per_s * coherent_integration_s;
                assert!(
                        (next_epoch_measurement - final_state[0]).abs() <= 1.0e-6,
                        "order={order:?} design={design:?} next_epoch_measurement={next_epoch_measurement} position_state={}",
                        final_state[0],
                    );
                assert!(
                    (final_state[1] - ramp_rate_units_per_s).abs() <= 1.0e-6,
                    "order={order:?} design={design:?} rate_state={}",
                    final_state[1],
                );
            }
        }
    }
}

#[test]
fn second_order_loop_design_matches_requested_noise_bandwidth() {
    let coherent_integration_s = 0.001;
    let target_bandwidth_hz = 6.0;
    let design = super::super::loop_observer_design(
        super::super::LoopFilterOrder::Second,
        target_bandwidth_hz,
        coherent_integration_s,
    );
    let measured_bandwidth_hz = super::super::loop_noise_bandwidth_hz(
        super::super::LoopFilterOrder::Second,
        coherent_integration_s,
        design.correction_gains,
    );

    assert!(
        (measured_bandwidth_hz - target_bandwidth_hz).abs() <= 1.0e-6,
        "design={design:?} measured_bandwidth_hz={measured_bandwidth_hz}",
    );
}

#[test]
fn second_order_loop_design_repeats_the_requested_pole() {
    let coherent_integration_s = 0.001;
    let design = super::super::loop_observer_design(
        super::super::LoopFilterOrder::Second,
        6.0,
        coherent_integration_s,
    );
    let transition = super::super::corrected_loop_transition(
        super::super::LoopFilterOrder::Second,
        coherent_integration_s,
        design.correction_gains,
    );
    let trace = transition[0][0] + transition[1][1];
    let determinant = transition[0][0] * transition[1][1] - transition[0][1] * transition[1][0];

    assert!((trace - 2.0 * design.closed_loop_pole).abs() <= 1.0e-9, "{transition:?}");
    assert!(
        (determinant - design.closed_loop_pole * design.closed_loop_pole).abs() <= 1.0e-9,
        "{transition:?}",
    );
}

#[test]
fn third_order_loop_design_matches_requested_noise_bandwidth() {
    let coherent_integration_s = 0.001;
    let target_bandwidth_hz = 18.0;
    let design = super::super::loop_observer_design(
        super::super::LoopFilterOrder::Third,
        target_bandwidth_hz,
        coherent_integration_s,
    );
    let measured_bandwidth_hz = super::super::loop_noise_bandwidth_hz(
        super::super::LoopFilterOrder::Third,
        coherent_integration_s,
        design.correction_gains,
    );

    assert!(
        (measured_bandwidth_hz - target_bandwidth_hz).abs() <= 1.0e-6,
        "design={design:?} measured_bandwidth_hz={measured_bandwidth_hz}",
    );
}

#[test]
fn third_order_loop_design_repeats_the_requested_pole() {
    let coherent_integration_s = 0.001;
    let design = super::super::loop_observer_design(
        super::super::LoopFilterOrder::Third,
        18.0,
        coherent_integration_s,
    );
    let transition = super::super::corrected_loop_transition(
        super::super::LoopFilterOrder::Third,
        coherent_integration_s,
        design.correction_gains,
    );
    let trace = transition[0][0] + transition[1][1] + transition[2][2];
    let principal_minors = transition[0][0] * transition[1][1]
        + transition[0][0] * transition[2][2]
        + transition[1][1] * transition[2][2]
        - transition[0][1] * transition[1][0]
        - transition[0][2] * transition[2][0]
        - transition[1][2] * transition[2][1];
    let determinant = transition[0][0]
        * (transition[1][1] * transition[2][2] - transition[1][2] * transition[2][1])
        - transition[0][1]
            * (transition[1][0] * transition[2][2] - transition[1][2] * transition[2][0])
        + transition[0][2]
            * (transition[1][0] * transition[2][1] - transition[1][1] * transition[2][0]);

    assert!((trace - 3.0 * design.closed_loop_pole).abs() <= 1.0e-9, "{transition:?}");
    assert!(
        (principal_minors - 3.0 * design.closed_loop_pole * design.closed_loop_pole).abs()
            <= 1.0e-9,
        "{transition:?}",
    );
    assert!((determinant - design.closed_loop_pole.powi(3)).abs() <= 1.0e-9, "{transition:?}",);
}

#[test]
fn phase_lock_loop_coefficients_are_finite_and_stronger_for_wider_bandwidths() {
    let narrow = phase_lock_loop_coefficients(5.0, 0.001);
    let wide = phase_lock_loop_coefficients(15.0, 0.001);

    assert!(narrow.phase_blend.is_finite());
    assert!(narrow.frequency_gain_hz_per_rad.is_finite());
    assert!(narrow.frequency_rate_gain_hz_per_s_per_rad.is_finite());
    assert!(wide.phase_blend > narrow.phase_blend, "{wide:?} {narrow:?}");
    assert!(
        wide.frequency_gain_hz_per_rad > narrow.frequency_gain_hz_per_rad,
        "{wide:?} {narrow:?}"
    );
    assert!(
        wide.frequency_rate_gain_hz_per_s_per_rad > narrow.frequency_rate_gain_hz_per_s_per_rad,
        "{wide:?} {narrow:?}"
    );
}

#[test]
fn angular_first_order_loop_coefficients_strengthen_frequency_response() {
    let linear = first_order_loop_coefficients(10.0, 0.001);
    let angular = first_order_angular_loop_coefficients(10.0, 0.001);

    assert_eq!(angular, linear);
}

#[test]
fn delay_lock_loop_coefficients_are_finite_and_stronger_for_wider_bandwidths() {
    let narrow = delay_lock_loop_coefficients(2.0, 0.001);
    let wide = delay_lock_loop_coefficients(8.0, 0.001);

    assert!(narrow.phase_blend.is_finite());
    assert!(narrow.rate_gain_hz_per_chip.is_finite());
    assert!(wide.phase_blend > narrow.phase_blend, "{wide:?} {narrow:?}");
    assert!(wide.rate_gain_hz_per_chip > narrow.rate_gain_hz_per_chip, "{wide:?} {narrow:?}");
}

#[test]
fn dll_lock_threshold_relaxes_for_subsample_early_late_spacing() {
    assert_eq!(dll_lock_threshold(1.0, 0.5), 0.6);
    assert_eq!(dll_lock_threshold(4.0, 0.5), 0.2);
}
