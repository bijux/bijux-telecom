//! Shared GPS L2C register and short-cycle code-generation helpers.

pub(crate) type RegisterState = [u8; 27];

pub(crate) fn register_state_from_octal(state_octal: u32) -> RegisterState {
    let mut state = [0_u8; 27];
    for (index, bit) in format!("{state_octal:027b}").bytes().enumerate() {
        state[index] = match bit {
            b'0' => 0,
            b'1' => 1,
            _ => unreachable!("binary formatting must only emit 0 or 1"),
        };
    }
    state
}

#[cfg(test)]
pub(crate) fn register_state_to_octal(state: &RegisterState) -> u32 {
    let mut value = 0_u32;
    for bit in state {
        value = (value << 1) | u32::from(*bit);
    }
    value
}

pub(crate) fn advance_register_state(state: &mut RegisterState) {
    let output_bit = state[26];
    for destination_index in (1..state.len()).rev() {
        let source_position = destination_index;
        let mut next_value = state[destination_index - 1];
        if gps_l2c_feedback_applies(source_position) {
            next_value ^= output_bit;
        }
        state[destination_index] = next_value;
    }
    state[0] = output_bit;
}

pub(crate) fn generate_bipolar_code_period(initial_state_octal: u32, chip_count: usize) -> Vec<i8> {
    generate_bipolar_code_range(initial_state_octal, chip_count, 0, chip_count)
}

pub(crate) fn generate_bipolar_code_range(
    initial_state_octal: u32,
    period_chips: usize,
    start_chip: usize,
    chip_count: usize,
) -> Vec<i8> {
    let mut state = register_state_from_octal(initial_state_octal);
    let start_offset = start_chip % period_chips;

    for _ in 0..start_offset {
        advance_register_state(&mut state);
    }

    let mut code = Vec::with_capacity(chip_count);
    let mut period_offset = start_offset;

    for _ in 0..chip_count {
        let output_bit = state[26];
        code.push(if output_bit == 0 { 1 } else { -1 });

        period_offset += 1;
        if period_offset == period_chips {
            state = register_state_from_octal(initial_state_octal);
            period_offset = 0;
        } else {
            advance_register_state(&mut state);
        }
    }

    code
}

fn gps_l2c_feedback_applies(source_position: usize) -> bool {
    matches!(source_position, 3 | 6 | 8 | 11 | 14 | 16 | 18 | 21 | 22 | 23 | 24)
}
