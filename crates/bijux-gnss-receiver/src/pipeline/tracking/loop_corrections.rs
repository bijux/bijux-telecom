type CodeLoopUpdate = bijux_gnss_signal::api::CodeLoopUpdate;
type CarrierLoopUpdate = bijux_gnss_signal::api::CarrierTrackingLoopUpdate;
type CodeLoopInput = bijux_gnss_signal::api::CodeLoopInput;
type CarrierLoopInput = bijux_gnss_signal::api::CarrierTrackingLoopInput;

fn apply_carrier_loop(input: CarrierLoopInput) -> CarrierLoopUpdate {
    let update =
        signal_apply_carrier_tracking_loop(bijux_gnss_signal::api::CarrierTrackingLoopInput {
            current_carrier_hz: input.current_carrier_hz,
            current_carrier_phase_cycles: input.current_carrier_phase_cycles,
            current_carrier_rate_hz_per_s: input.current_carrier_rate_hz_per_s,
            epoch_len_samples: input.epoch_len_samples,
            sample_rate_hz: input.sample_rate_hz,
            coherent_integration_s: input.coherent_integration_s,
            pll_bw_hz: input.pll_bw_hz,
            pll_err_rad: input.pll_err_rad,
            fll_bw_hz: input.fll_bw_hz,
            fll_err_hz: input.fll_err_hz,
            apply_fll: input.apply_fll,
            apply_pll_frequency: input.apply_pll_frequency,
            apply_pll_phase: input.apply_pll_phase,
        });
    CarrierLoopUpdate {
        carrier_hz: update.carrier_hz,
        carrier_phase_cycles: update.carrier_phase_cycles,
        carrier_rate_hz_per_s: update.carrier_rate_hz_per_s,
    }
}
