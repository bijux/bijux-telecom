//! Public API for bijux-gnss-signal.

use std::any::Any;

/// Signal catalog, wavelength, and multi-band scaling helpers.
pub use crate::catalog::{
    carrier_wavelength_m, default_acquisition_sats, default_acquisition_signal,
    first_order_ionosphere_code_delay_m, first_order_ionosphere_phase_advance_m,
    glonass_l1_carrier_hz, registered_signal_registry_entries, resolved_signal_registry_entry,
    shared_path_doppler_hz, signal_cycles_to_meters, signal_id_cycles_to_meters,
    signal_id_meters_to_cycles, signal_id_wavelength_m, signal_meters_to_cycles, signal_registry,
    signal_spec_beidou_b1i, signal_spec_beidou_b2i, signal_spec_galileo_e1b,
    signal_spec_galileo_e1c, signal_spec_galileo_e5a, signal_spec_galileo_e5b,
    signal_spec_glonass_l1, signal_spec_gps_l1_ca, signal_spec_gps_l2_py, signal_spec_gps_l2c,
    signal_spec_gps_l5, signal_spec_gps_l5_i, signal_spec_gps_l5_q, signal_wavelength_m,
};
/// Spreading code generators.
pub use crate::codes::beidou_b1i::{
    beidou_b1i_code_assignment, beidou_b1i_code_assignments, generate_beidou_b1i_code,
    generate_beidou_b1i_code_chips, sample_beidou_b1i_code, BeidouB1iCodeAssignment,
    BEIDOU_B1I_CODE_CHIPS, BEIDOU_B1I_CODE_RATE_HZ,
};
pub use crate::codes::beidou_b2i::{
    beidou_b2i_code_assignment, beidou_b2i_code_assignments, generate_beidou_b2i_code,
    generate_beidou_b2i_code_chips, sample_beidou_b2i_code, BeidouB2iCodeAssignment,
    BEIDOU_B2I_CODE_CHIPS, BEIDOU_B2I_CODE_RATE_HZ,
};
pub use crate::codes::beidou_d1::{
    beidou_d1_data_symbol_index, beidou_d1_epoch_symbol, beidou_d1_neumann_hoffman_code,
    beidou_d1_nh_chip, beidou_d1_symbol_epoch, BEIDOU_D1_NAV_SYMBOL_PERIOD_S,
    BEIDOU_D1_PRIMARY_EPOCHS_PER_SYMBOL, BEIDOU_D1_SECONDARY_CHIP_PERIOD_S,
    BEIDOU_D1_SECONDARY_CODE_CHIPS,
};
pub use crate::codes::ca_code::{
    ca_code_assignment, ca_code_assignments, ca_code_autocorrelation_summary,
    ca_code_cross_correlation_summary, ca_code_periodic_autocorrelation,
    ca_code_periodic_cross_correlation, generate_ca_code, generate_ca_code_chips,
    periodic_correlation, CaCodeAssignment, CaCodeAutocorrelationSummary,
    CaCodeCrossCorrelationSummary, Prn, CA_CODE_PERIOD_CHIPS,
};
pub use crate::codes::galileo_e1::{
    boc_subcarrier_value, galileo_e1_cboc_value, galileo_e1_pilot_cboc_value,
    galileo_e1c_secondary_chip, galileo_e1c_secondary_code, generate_galileo_e1_primary_code,
    generate_galileo_e1b_code, generate_galileo_e1c_code, sample_boc_code,
    sample_galileo_e1_boc11_code, sample_galileo_e1_cboc, sample_galileo_e1_pilot_cboc_code,
    GalileoE1Channel, GALILEO_E1_CBOC_ALPHA, GALILEO_E1_CBOC_BETA, GALILEO_E1_CODE_RATE_HZ,
    GALILEO_E1_PRIMARY_CODE_CHIPS, GALILEO_E1_PRIMARY_PERIOD_MS, GALILEO_E1_SECONDARY_CODE_CHIPS,
};
pub use crate::codes::galileo_e5::{
    galileo_e5a_i_code_assignment, galileo_e5a_i_code_assignments, galileo_e5a_i_data_symbol_index,
    galileo_e5a_i_epoch_symbol, galileo_e5a_i_secondary_chip, galileo_e5a_i_secondary_code,
    galileo_e5a_i_value, galileo_e5a_primary_autocorrelation, galileo_e5a_q_code_assignment,
    galileo_e5a_q_code_assignments, galileo_e5a_q_epoch_symbol, galileo_e5a_q_secondary_chip,
    galileo_e5a_q_secondary_code, galileo_e5a_q_value, galileo_e5a_qpsk_value,
    galileo_e5b_i_code_assignment, galileo_e5b_i_code_assignments, galileo_e5b_i_data_symbol_index,
    galileo_e5b_i_epoch_symbol, galileo_e5b_i_secondary_chip, galileo_e5b_i_secondary_code,
    galileo_e5b_i_value, galileo_e5b_primary_autocorrelation, galileo_e5b_q_code_assignment,
    galileo_e5b_q_code_assignments, galileo_e5b_q_epoch_symbol, galileo_e5b_q_secondary_chip,
    galileo_e5b_q_secondary_code, galileo_e5b_q_value, galileo_e5b_qpsk_value,
    generate_galileo_e5a_i_code, generate_galileo_e5a_q_code, generate_galileo_e5b_i_code,
    generate_galileo_e5b_q_code, sample_galileo_e5a_i_primary_code,
    sample_galileo_e5a_q_primary_code, sample_galileo_e5b_i_primary_code,
    sample_galileo_e5b_q_primary_code, GalileoE5aICodeAssignment, GalileoE5aQCodeAssignment,
    GalileoE5bICodeAssignment, GalileoE5bQCodeAssignment, GALILEO_E5A_CODE_RATE_HZ,
    GALILEO_E5A_I_PRIMARY_EPOCHS_PER_SYMBOL, GALILEO_E5A_I_SECONDARY_CODE_CHIPS,
    GALILEO_E5A_PRIMARY_CODE_CHIPS, GALILEO_E5A_Q_SECONDARY_CODE_CHIPS, GALILEO_E5B_CODE_RATE_HZ,
    GALILEO_E5B_I_PRIMARY_EPOCHS_PER_SYMBOL, GALILEO_E5B_I_SECONDARY_CODE_CHIPS,
    GALILEO_E5B_PRIMARY_CODE_CHIPS, GALILEO_E5B_Q_SECONDARY_CODE_CHIPS,
};
pub use crate::codes::glonass_l1::{
    generate_glonass_l1_st_code, generate_glonass_l1_st_code_chips, glonass_l1_meander_symbol,
    glonass_l1_relative_data_symbol, glonass_l1_string_symbol, glonass_l1_string_symbol_at_time_s,
    sample_glonass_l1_st_code, GLONASS_L1_DATA_BIT_PERIOD_S, GLONASS_L1_STRING_DATA_BITS,
    GLONASS_L1_STRING_DATA_SYMBOLS, GLONASS_L1_STRING_DURATION_S, GLONASS_L1_STRING_SYMBOLS,
    GLONASS_L1_ST_CODE_CHIPS, GLONASS_L1_ST_CODE_RATE_HZ, GLONASS_L1_SYMBOL_PERIOD_S,
    GLONASS_L1_TIME_MARK, GLONASS_L1_TIME_MARK_SYMBOLS,
};
pub use crate::codes::gps_l2c::{
    generate_gps_l2c_time_multiplexed_chips,
    generate_gps_l2c_time_multiplexed_chips_from_components, gps_l2c_time_multiplexed_component,
    gps_l2c_time_multiplexed_value, sample_gps_l2c_time_multiplexed,
    sample_gps_l2c_time_multiplexed_from_components, GpsL2cTimeMultiplexedComponent,
    GPS_L2C_TIME_MULTIPLEXED_CODE_CHIPS, GPS_L2C_TIME_MULTIPLEXED_CODE_RATE_HZ,
    GPS_L2C_TIME_MULTIPLEXED_SYMBOL_CHIPS,
};
pub use crate::codes::gps_l2c_cl::{
    generate_gps_l2c_cl_code, generate_gps_l2c_cl_code_range, gps_l2c_cl_code_assignment,
    gps_l2c_cl_code_assignments, sample_gps_l2c_cl_code, GpsL2cClCodeAssignment,
    GPS_L2C_CL_CODE_CHIPS, GPS_L2C_CL_CODE_RATE_HZ,
};
pub use crate::codes::gps_l2c_cm::{
    generate_gps_l2c_cm_code, generate_gps_l2c_cm_code_chips, gps_l2c_cm_code_assignment,
    gps_l2c_cm_code_assignments, sample_gps_l2c_cm_code, GpsL2cCmCodeAssignment,
    GPS_L2C_CM_CODE_CHIPS, GPS_L2C_CM_CODE_RATE_HZ,
};
pub use crate::codes::gps_l5::{
    generate_gps_l5_i_chips, generate_gps_l5_i_chips_from_code, generate_gps_l5_i_code,
    generate_gps_l5_q_chips, generate_gps_l5_q_chips_from_code, generate_gps_l5_q_code,
    gps_l5_i_code_assignment, gps_l5_i_code_assignments, gps_l5_i_data_symbol_index,
    gps_l5_i_epoch_symbol, gps_l5_i_neumann_hoffman_code, gps_l5_i_nh_chip, gps_l5_i_symbol_epoch,
    gps_l5_i_value, gps_l5_q_code_assignment, gps_l5_q_code_assignments, gps_l5_q_epoch_symbol,
    gps_l5_q_neumann_hoffman_code, gps_l5_q_nh_chip, gps_l5_q_symbol_epoch, gps_l5_q_value,
    sample_gps_l5_i, sample_gps_l5_i_code, sample_gps_l5_i_from_code, sample_gps_l5_i_primary_code,
    sample_gps_l5_q, sample_gps_l5_q_code, sample_gps_l5_q_from_code, sample_gps_l5_q_primary_code,
    GpsL5ICodeAssignment, GpsL5QCodeAssignment, GPS_L5I_SYMBOL_CHIPS, GPS_L5I_SYMBOL_CODE_EPOCHS,
    GPS_L5Q_SYMBOL_CHIPS, GPS_L5Q_SYMBOL_CODE_EPOCHS, GPS_L5_CODE_CHIPS, GPS_L5_CODE_RATE_HZ,
    GPS_L5_I_PRIMARY_EPOCHS_PER_SYMBOL, GPS_L5_I_SYMBOL_CHIPS, GPS_L5_PRIMARY_CODE_CHIPS,
    GPS_L5_PRIMARY_CODE_RATE_HZ, GPS_L5_Q_PRIMARY_EPOCHS_PER_SYMBOL, GPS_L5_Q_SYMBOL_CHIPS,
};
/// Complex front-end FIR design and response helpers.
pub use crate::dsp::front_end::{
    measure_transfer_response_db, FrontEndFilterSpec, FrontEndFirFilter, MeasuredFrontEndResponse,
};
/// Tracking-oriented local-code models.
pub use crate::dsp::local_code::{
    default_local_code_model, default_local_code_model_for_signal, LocalCodeModel,
};
/// Numerically controlled oscillator helper.
pub use crate::dsp::nco::Nco;
/// Front-end quality metrics derived from complex I/Q samples.
pub use crate::dsp::quality::{
    estimate_iq_noise_floor_db, estimate_iq_noise_floor_db_from_metrics,
    measure_iq_front_end_metrics, measure_raw_iq_front_end_metrics, remove_dc_offset_in_place,
    IqFrontEndAnalyzer, IqFrontEndMetrics,
};
/// Replica-generation and synthetic modulation helpers.
pub use crate::dsp::replica::{
    carrier_hz_at_time, carrier_phase_radians_at_time, default_signal_carrier_hz,
    default_signal_carrier_hz_for_band, default_signal_carrier_hz_for_signal,
    sample_modulated_replica_at_sample_index, sample_modulated_replica_at_time,
    signal_amplitude_from_cn0_db_hz, wipeoff_carrier_with_linear_rate, AcquisitionSignalModel,
    ReplicaCodeModel, UNIT_VARIANCE_COMPLEX_NOISE_POWER,
};
/// Absolute sample-index timing helpers for chunk-stable generation.
pub use crate::dsp::sample_timing::{code_sample_position_at_index, CodeSamplePosition};
/// Signal processing utilities.
pub use crate::dsp::signal::{
    advance_code_phase_chips, advance_code_phase_seconds, code_phase_samples_at_sample_index,
    code_value_at_phase, epoch_start_code_phase_samples_from_receiver_phase,
    receiver_code_phase_samples_from_epoch_start_phase, receiver_search_code_phase_samples,
    sample_ca_code, sample_code, samples_per_code, wipeoff_carrier, wrap_code_phase_samples,
    wrapped_code_phase_delta_samples, wrapped_code_phase_error_samples,
    wrapped_code_phase_error_samples_f64,
};
/// Signal-spectrum analysis helpers.
pub use crate::dsp::spectrum::{
    apply_front_end_transfer_power_spectral_density, estimate_complex_power_spectral_density,
    estimate_power_spectral_density, expected_component_power_spectral_density,
    expected_filtered_component_power_spectral_density,
    expected_filtered_signal_power_spectral_density, expected_signal_power_spectral_density,
    find_deep_spectrum_nulls, summarize_power_spectral_density, PowerSpectralDensityPoint,
    PowerSpectralDensitySummary, SpectrumEstimatorConfig, SpectrumNull,
};
/// Tracking helpers.
pub use crate::dsp::tracking::{
    adaptive_bandwidth, anti_false_lock_detected, apply_carrier_tracking_loop, apply_code_loop,
    carrier_frequency_error_hz_from_phase_delta, carrier_phase_offset_radians, code_at,
    coherent_integration_seconds, correlate_early_prompt_late, delay_lock_loop_coefficients,
    discriminators, dll_hold_threshold, dll_lock_threshold, estimate_cn0_dbhz,
    estimate_tracking_uncertainty, first_order_angular_loop_coefficients,
    first_order_loop_coefficients, fll_lock_threshold_hz, phase_lock_loop_coefficients,
    prompt_power_ratio, push_tracking_uncertainty_sample, refresh_lock_reference_cn0_dbhz,
    refresh_prompt_power_reference, update_prompt_power_reference,
    update_windowed_tracking_cn0_estimate, wrap_phase_cycles_signed, wrap_phase_radians_positive,
    wrapped_phase_delta_cycles, CarrierTrackingLoopInput, CarrierTrackingLoopUpdate, CodeLoopInput,
    CodeLoopUpdate, DelayLockLoopCoefficients, EarlyPromptLateCorrelation,
    FirstOrderLoopCoefficients, PhaseLockLoopCoefficients, TrackingQualityClass,
    TrackingUncertaintyInputs,
};
/// Error types.
pub use crate::error::SignalError;
/// Observation validation and dual-frequency compatibility helpers.
pub use crate::obs_validation::{
    check_dual_frequency_observations, check_inter_frequency_alignment,
    supported_dual_frequency_band_pairs, supported_dual_frequency_band_pairs_for_constellation,
    validate_obs_epochs, BandLagEvent, DualFrequencyObservationPair,
    DualFrequencyObservationReport, DualFrequencyPairIssue, DualFrequencyPairStatus,
    InterFrequencyAlignmentReport,
};
/// Raw IQ metadata contracts.
pub use crate::raw_iq::{IqQuantization, IqSampleFormat, RawIqMetadata};
/// Sample conversion helpers.
pub use crate::samples::{
    encode_quantized_samples, iq_f32_to_samples, iq_i16_to_samples, iq_i8_to_samples,
    quantize_samples_for_storage,
};

/// Streaming signal source interface.
pub trait SignalSource {
    /// Error type returned by the source.
    type Error;

    /// Sample rate in Hz.
    fn sample_rate_hz(&self) -> f64;

    /// Fetch the next sample frame.
    fn next_frame(
        &mut self,
        frame_len: usize,
    ) -> Result<Option<bijux_gnss_core::api::SamplesFrame>, Self::Error>;

    /// Whether the source has reached end-of-stream.
    fn is_done(&self) -> bool;

    /// Borrow the concrete source for optional source-specific integrations.
    fn as_any(&self) -> &dyn Any;
}

/// Minimal sample source interface.
pub trait SampleSource {
    /// Error type returned by the source.
    type Error;

    /// Fetch the next sample frame.
    fn next_samples(
        &mut self,
        frame_len: usize,
    ) -> Result<Option<bijux_gnss_core::api::SamplesFrame>, Self::Error>;
}

/// Correlator interface for tracking tests.
pub trait Correlator {
    /// Correlate a sample frame with a local code.
    fn correlate(
        &self,
        frame: &bijux_gnss_core::api::SamplesFrame,
        prn: u8,
        carrier_hz: f64,
        code_phase_samples: f64,
        early_late_spacing_chips: f64,
    ) -> (num_complex::Complex<f32>, num_complex::Complex<f32>, num_complex::Complex<f32>);
}

/// Sink interface for writing sample frames.
pub trait SampleSink {
    /// Error type returned by the sink.
    type Error;

    /// Push a sample frame into the sink.
    fn push_frame(&mut self, frame: &bijux_gnss_core::api::SamplesFrame)
        -> Result<(), Self::Error>;
}
