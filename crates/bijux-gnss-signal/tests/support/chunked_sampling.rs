#[allow(dead_code)]
pub fn deterministic_chunk_lengths(total_samples: usize) -> Vec<usize> {
    let mut remaining = total_samples;
    let mut state = 0x9E37_79B9_7F4A_7C15_u64;
    let mut chunk_lengths = Vec::new();

    while remaining > 0 {
        state ^= state << 7;
        state ^= state >> 9;
        state ^= state << 8;
        let candidate = ((state as usize) % 2_047) + 1;
        let chunk_len = candidate.min(remaining);
        chunk_lengths.push(chunk_len);
        remaining -= chunk_len;
    }

    chunk_lengths
}
