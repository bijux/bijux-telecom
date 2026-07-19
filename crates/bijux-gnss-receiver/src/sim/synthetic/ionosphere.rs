#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct SyntheticIonosphereDelayModel {
    /// Code delay applied at the reference signal, in meters.
    pub reference_delay_m: f64,
    /// Signal that defines the reference delay magnitude.
    pub reference_signal: SignalSpec,
}

impl SyntheticIonosphereDelayModel {
    pub fn code_delay_m(self, signal: SignalSpec) -> Option<f64> {
        first_order_ionosphere_code_delay_m(
            Meters(self.reference_delay_m),
            self.reference_signal,
            signal,
        )
        .map(|delay_m| delay_m.0)
    }

    pub fn pseudorange_m(self, geometric_pseudorange_m: f64, signal: SignalSpec) -> Option<f64> {
        let delay_m = self.code_delay_m(signal)?;
        let pseudorange_m = geometric_pseudorange_m + delay_m;
        pseudorange_m.is_finite().then_some(pseudorange_m)
    }
}
