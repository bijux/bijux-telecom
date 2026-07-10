#![allow(missing_docs)]

#[derive(Debug, Clone, Copy)]
pub(crate) struct HatchFilterOutput {
    pub smoothed_pseudorange_m: f64,
    pub smoothing_age_epochs: u32,
    pub reset_count: u32,
}

#[derive(Debug, Clone, Default)]
pub(crate) struct HatchFilterState {
    smoothed_pseudorange_m: f64,
    observation_count: u32,
    last_carrier_cycles: f64,
    last_divergence_m: f64,
    smoothing_age_epochs: u32,
    reset_count: u32,
    initialized: bool,
}

impl HatchFilterState {
    pub(crate) fn reset_count(&self) -> u32 {
        self.reset_count
    }

    pub(crate) fn clear_arc(&mut self) {
        if self.initialized {
            self.reset_count = self.reset_count.saturating_add(1);
        }
        self.initialized = false;
        self.observation_count = 0;
        self.smoothing_age_epochs = 0;
    }

    pub(crate) fn divergence_jump_m(&self, raw_divergence_m: f64) -> Option<f64> {
        self.initialized.then(|| (raw_divergence_m - self.last_divergence_m).abs())
    }

    pub(crate) fn observe(
        &mut self,
        raw_pseudorange_m: f64,
        carrier_phase_cycles: f64,
        raw_divergence_m: f64,
        lambda_m: f64,
        window_epochs: u32,
    ) -> HatchFilterOutput {
        if !self.initialized {
            self.smoothed_pseudorange_m = raw_pseudorange_m;
            self.observation_count = 1;
            self.last_carrier_cycles = carrier_phase_cycles;
            self.last_divergence_m = raw_divergence_m;
            self.initialized = true;
            self.smoothing_age_epochs = 1;
        } else {
            let capped_window_epochs = window_epochs.max(1);
            let delta_carrier_m = (carrier_phase_cycles - self.last_carrier_cycles) * lambda_m;
            let predicted_pseudorange_m = self.smoothed_pseudorange_m + delta_carrier_m;
            let effective_count = self
                .observation_count
                .saturating_add(1)
                .min(capped_window_epochs) as f64;
            self.smoothed_pseudorange_m = predicted_pseudorange_m
                + (raw_pseudorange_m - predicted_pseudorange_m) / effective_count;
            self.observation_count =
                self.observation_count.saturating_add(1).min(capped_window_epochs);
            self.last_carrier_cycles = carrier_phase_cycles;
            self.last_divergence_m = raw_divergence_m;
            self.smoothing_age_epochs = self.smoothing_age_epochs.saturating_add(1);
        }

        HatchFilterOutput {
            smoothed_pseudorange_m: self.smoothed_pseudorange_m,
            smoothing_age_epochs: self.smoothing_age_epochs,
            reset_count: self.reset_count,
        }
    }
}
