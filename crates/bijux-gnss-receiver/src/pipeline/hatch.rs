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
    carrier_phase_arc_id: Option<String>,
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
        self.carrier_phase_arc_id = None;
    }

    pub(crate) fn align_carrier_phase_arc(&mut self, arc_id: &str) {
        if self.carrier_phase_arc_id.as_deref() != Some(arc_id) {
            self.clear_arc();
            self.carrier_phase_arc_id = Some(arc_id.to_string());
        }
    }

    pub(crate) fn divergence_delta_m(&self, raw_divergence_m: f64) -> Option<f64> {
        self.initialized.then_some(raw_divergence_m - self.last_divergence_m)
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
            let effective_count =
                self.observation_count.saturating_add(1).min(capped_window_epochs) as f64;
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

#[cfg(test)]
mod tests {
    use super::HatchFilterState;

    #[test]
    fn hatch_filter_resets_when_carrier_phase_arc_changes() {
        let mut state = HatchFilterState::default();

        state.align_carrier_phase_arc("gps-06-l1-ca-e0000000070-s000000286440");
        let first = state.observe(20_000_000.0, 10.0, 0.0, 0.19, 10);
        let second = state.observe(20_000_001.0, 10.5, 0.0, 0.19, 10);

        assert_eq!(first.smoothing_age_epochs, 1);
        assert_eq!(second.smoothing_age_epochs, 2);
        assert_eq!(second.reset_count, 0);

        state.align_carrier_phase_arc("gps-06-l1-ca-e0000000072-s000000294624");
        let reset = state.observe(20_000_010.0, 1.0, 0.0, 0.19, 10);

        assert_eq!(reset.smoothing_age_epochs, 1);
        assert_eq!(reset.reset_count, 1);
    }

    #[test]
    fn hatch_filter_keeps_state_for_same_carrier_phase_arc() {
        let mut state = HatchFilterState::default();

        state.align_carrier_phase_arc("gps-06-l1-ca-e0000000070-s000000286440");
        let first = state.observe(20_000_000.0, 10.0, 0.0, 0.19, 10);
        state.align_carrier_phase_arc("gps-06-l1-ca-e0000000070-s000000286440");
        let second = state.observe(20_000_001.0, 10.5, 0.0, 0.19, 10);

        assert_eq!(first.smoothing_age_epochs, 1);
        assert_eq!(second.smoothing_age_epochs, 2);
        assert_eq!(second.reset_count, 0);
    }
}
