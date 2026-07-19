#[derive(Debug, Clone)]
struct XorShift64 {
    state: u64,
}

impl XorShift64 {
    fn new(seed: u64) -> Self {
        let seed = if seed == 0 { 0xDEADBEEFCAFEBABE } else { seed };
        Self { state: seed }
    }

    fn next_u64(&mut self) -> u64 {
        let mut x = self.state;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.state = x;
        x
    }

    fn next_f32(&mut self) -> f32 {
        let val = (self.next_u64() >> 40) as u32;
        val as f32 / (u32::MAX as f32)
    }

    fn next_gaussian(&mut self) -> f32 {
        let u1 = self.next_f32().max(1e-12);
        let u2 = self.next_f32();
        let r = (-2.0 * u1.ln()).sqrt();
        let theta = TAU * u2;
        r * theta.cos()
    }
}
