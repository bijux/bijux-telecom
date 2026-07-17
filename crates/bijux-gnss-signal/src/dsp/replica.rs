//! Replica-generation and modulation helpers for synthetic and tracking workflows.

use crate::dsp::sample_timing::code_sample_position_at_index;
use crate::error::SignalError;
use num_complex::Complex;

mod acquisition_model;
mod carrier_trajectory;
mod code_model;
mod signal_identity;

pub use acquisition_model::AcquisitionSignalModel;
pub use carrier_trajectory::{
    carrier_hz_at_time, carrier_hz_at_time_with_jerk, carrier_phase_radians_at_time,
    carrier_phase_radians_at_time_with_jerk, wipeoff_carrier_with_linear_rate,
};
pub use code_model::ReplicaCodeModel;
pub use signal_identity::{
    default_signal_carrier_hz, default_signal_carrier_hz_for_band, default_signal_carrier_hz_for_signal,
};
pub(crate) use signal_identity::default_signal_code_for_band;

/// Complex noise power implied by unit-variance I and Q components.
pub const UNIT_VARIANCE_COMPLEX_NOISE_POWER: f64 = 2.0;

/// Convert C/N0 into the synthesized signal amplitude for a known complex noise power.
pub fn signal_amplitude_from_cn0_db_hz(
    cn0_db_hz: f32,
    sample_rate_hz: f64,
    complex_noise_power: f64,
) -> f32 {
    let cn0_linear = 10.0_f64.powf(cn0_db_hz as f64 / 10.0).max(1e-12);
    ((cn0_linear * complex_noise_power.max(1e-12)) / sample_rate_hz.max(1e-12)).sqrt() as f32
}

/// Sample a modulated replica at one absolute sample index.
pub fn sample_modulated_replica_at_sample_index(
    model: &ReplicaCodeModel,
    sample_rate_hz: f64,
    initial_code_phase_chips: f64,
    initial_carrier_phase_radians: f64,
    initial_carrier_hz: f64,
    carrier_rate_hz_per_s: f64,
    sample_index: u64,
    data_bit: i8,
    amplitude: f32,
) -> Result<Complex<f32>, SignalError> {
    let position = code_sample_position_at_index(
        initial_code_phase_chips,
        sample_rate_hz,
        model.code_rate_hz(),
        model.code_length(),
        sample_index,
    )?;
    let signal_value =
        model.sample_value(position.chip_phase, position.primary_code_period_index, data_bit)?;
    let phase = carrier_phase_radians_at_time(
        initial_carrier_phase_radians,
        initial_carrier_hz,
        carrier_rate_hz_per_s,
        position.elapsed_s,
    ) as f32;
    let carrier = Complex::new(phase.cos(), phase.sin());
    Ok(carrier * signal_value * amplitude)
}

/// Sample a modulated replica at one elapsed time.
pub fn sample_modulated_replica_at_time(
    model: &ReplicaCodeModel,
    initial_code_phase_chips: f64,
    initial_carrier_phase_radians: f64,
    initial_carrier_hz: f64,
    carrier_rate_hz_per_s: f64,
    elapsed_s: f64,
    data_bit: i8,
    amplitude: f32,
) -> Result<Complex<f32>, SignalError> {
    let total_chip_phase = initial_code_phase_chips + model.code_rate_hz() * elapsed_s;
    let code_length = model.code_length().max(1) as f64;
    let primary_code_period_index =
        if total_chip_phase <= 0.0 { 0 } else { (total_chip_phase / code_length).floor() as usize };
    let code_phase = total_chip_phase.rem_euclid(code_length);
    let signal_value = model.sample_value(code_phase, primary_code_period_index, data_bit)?;
    let phase = carrier_phase_radians_at_time(
        initial_carrier_phase_radians,
        initial_carrier_hz,
        carrier_rate_hz_per_s,
        elapsed_s,
    ) as f32;
    let carrier = Complex::new(phase.cos(), phase.sin());
    Ok(carrier * signal_value * amplitude)
}

#[cfg(test)]
mod tests {
    use super::{
        carrier_hz_at_time, carrier_hz_at_time_with_jerk, carrier_phase_radians_at_time,
        carrier_phase_radians_at_time_with_jerk, default_signal_carrier_hz,
        default_signal_carrier_hz_for_band, sample_modulated_replica_at_sample_index,
        sample_modulated_replica_at_time, signal_amplitude_from_cn0_db_hz,
        wipeoff_carrier_with_linear_rate, AcquisitionSignalModel, ReplicaCodeModel,
        UNIT_VARIANCE_COMPLEX_NOISE_POWER,
    };
    use crate::catalog::resolved_signal_registry_entry;
    use crate::codes::beidou_d1::{beidou_d1_epoch_symbol, BEIDOU_D1_PRIMARY_EPOCHS_PER_SYMBOL};
    use crate::codes::galileo_e1::{
        sample_galileo_e1_boc11_code, GalileoE1Channel, GALILEO_E1_CODE_RATE_HZ,
    };
    use crate::codes::galileo_e5::{
        sample_galileo_e5a_i_primary_code, sample_galileo_e5b_i_primary_code,
        GALILEO_E5A_CODE_RATE_HZ, GALILEO_E5B_CODE_RATE_HZ,
    };
    use crate::codes::glonass_l1::sample_glonass_l1_st_code;
    use crate::codes::gps_l2c::{
        GPS_L2C_TIME_MULTIPLEXED_CODE_CHIPS, GPS_L2C_TIME_MULTIPLEXED_CODE_RATE_HZ,
    };
    use crate::codes::gps_l2c_cl::{sample_gps_l2c_cl_code, GPS_L2C_CL_CODE_RATE_HZ};
    use crate::codes::gps_l2c_cm::{sample_gps_l2c_cm_code, GPS_L2C_CM_CODE_RATE_HZ};
    use crate::codes::gps_l5::{
        sample_gps_l5_i_primary_code, sample_gps_l5_q_primary_code, GPS_L5_PRIMARY_CODE_RATE_HZ,
    };
    use crate::dsp::local_code::{
        default_local_code_model, default_local_code_model_for_signal, LocalCodeModel,
    };
    use crate::error::SignalError;
    use bijux_gnss_core::api::{
        Constellation, GlonassFrequencyChannel, SatId, SignalBand, SignalCode,
        GALILEO_E5A_CARRIER_HZ, GALILEO_E5B_CARRIER_HZ, GPS_L1_CA_CARRIER_HZ, GPS_L5_CARRIER_HZ,
    };
    use num_complex::Complex;

    #[test]
    fn carrier_hz_at_time_applies_linear_rate() {
        assert!((carrier_hz_at_time(1_350.0, 40.0, 0.5) - 1_370.0).abs() < 1.0e-12);
    }

    #[test]
    fn carrier_phase_radians_at_time_integrates_linear_rate() {
        let phase = carrier_phase_radians_at_time(0.25, 1_000.0, 20.0, 0.5);
        let expected = 0.25 + std::f64::consts::TAU * (1_000.0 * 0.5 + 0.5 * 20.0 * 0.25);
        assert!((phase - expected).abs() < 1.0e-12, "phase={phase}");
    }

    #[test]
    fn carrier_hz_at_time_with_jerk_applies_quadratic_frequency() {
        let carrier_hz = carrier_hz_at_time_with_jerk(1_350.0, 40.0, 120.0, 0.5);

        assert!((carrier_hz - 1_385.0).abs() < 1.0e-12);
    }

    #[test]
    fn carrier_phase_radians_at_time_with_jerk_integrates_quadratic_frequency() {
        let phase = carrier_phase_radians_at_time_with_jerk(0.25, 1_000.0, 20.0, 120.0, 0.5);
        let expected = 0.25
            + std::f64::consts::TAU * (1_000.0 * 0.5 + 0.5 * 20.0 * 0.25 + 120.0 * 0.125 / 6.0);

        assert!((phase - expected).abs() < 1.0e-12, "phase={phase}");
    }

    #[test]
    fn zero_jerk_carrier_model_matches_linear_model() {
        let carrier_hz = carrier_hz_at_time_with_jerk(750.0, -35.0, 0.0, 0.25);
        let carrier_phase = carrier_phase_radians_at_time_with_jerk(0.1, 750.0, -35.0, 0.0, 0.25);

        assert!((carrier_hz - carrier_hz_at_time(750.0, -35.0, 0.25)).abs() < 1.0e-12);
        assert!(
            (carrier_phase - carrier_phase_radians_at_time(0.1, 750.0, -35.0, 0.25)).abs()
                < 1.0e-12
        );
    }

    #[test]
    fn wipeoff_carrier_with_linear_rate_removes_ramped_phase() {
        let samples = (0..8)
            .map(|sample_index| {
                let phase = carrier_phase_radians_at_time(
                    0.3,
                    1_200.0,
                    40.0,
                    sample_index as f64 / 4_000.0,
                );
                Complex::from_polar(1.0_f32, phase as f32)
            })
            .collect::<Vec<_>>();

        let wiped = wipeoff_carrier_with_linear_rate(&samples, 1_200.0, 40.0, 4_000.0, 0, 0.3)
            .expect("valid linear carrier wipeoff");

        for sample in wiped {
            assert!((sample.re - 1.0).abs() <= 1.0e-5, "sample={sample:?}");
            assert!(sample.im.abs() <= 1.0e-5, "sample={sample:?}");
        }
    }

    #[test]
    fn wipeoff_carrier_with_linear_rate_honors_frame_relative_start_index() {
        let start_sample_index = 16_u64;
        let samples = (0..8)
            .map(|offset| {
                let elapsed_s = (start_sample_index + offset) as f64 / 4_000.0;
                let phase = carrier_phase_radians_at_time(0.1, 750.0, 25.0, elapsed_s);
                Complex::from_polar(1.0_f32, phase as f32)
            })
            .collect::<Vec<_>>();

        let wiped = wipeoff_carrier_with_linear_rate(
            &samples,
            750.0,
            25.0,
            4_000.0,
            start_sample_index,
            0.1,
        )
        .expect("valid linear carrier wipeoff");

        for sample in wiped {
            assert!((sample.re - 1.0).abs() <= 1.0e-5, "sample={sample:?}");
            assert!(sample.im.abs() <= 1.0e-5, "sample={sample:?}");
        }
    }

    #[test]
    fn signal_amplitude_from_cn0_db_hz_scales_with_cn0() {
        let weak =
            signal_amplitude_from_cn0_db_hz(35.0, 4_000_000.0, UNIT_VARIANCE_COMPLEX_NOISE_POWER);
        let strong =
            signal_amplitude_from_cn0_db_hz(45.0, 4_000_000.0, UNIT_VARIANCE_COMPLEX_NOISE_POWER);
        assert!(strong > weak, "weak={weak} strong={strong}");
    }

    #[test]
    fn sample_modulated_replica_at_time_matches_unity_carrier_and_code_origin() {
        let model = ReplicaCodeModel::gps_l1_ca(1).expect("valid GPS PRN");
        let sample = sample_modulated_replica_at_time(&model, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 2.0)
            .expect("valid replica");
        assert!((sample.norm() - 2.0).abs() < 1.0e-6, "sample={sample:?}");
    }

    #[test]
    fn sample_modulated_replica_at_sample_index_matches_elapsed_time_model() {
        let model = ReplicaCodeModel::galileo_e1_cboc(11).expect("valid Galileo PRN");
        let sample_rate_hz = 4_000_000.0;
        let sample_index = 1_733;
        let elapsed_s = sample_index as f64 / sample_rate_hz;

        let by_time = sample_modulated_replica_at_time(
            &model, 137.625, 0.125, 4_100.0, 2.5, elapsed_s, -1, 0.75,
        )
        .expect("elapsed-time replica");
        let by_index = sample_modulated_replica_at_sample_index(
            &model,
            sample_rate_hz,
            137.625,
            0.125,
            4_100.0,
            2.5,
            sample_index,
            -1,
            0.75,
        )
        .expect("sample-index replica");

        assert_eq!(by_index, by_time);
    }

    #[test]
    fn sample_block_matches_single_sample_iteration_at_origin() {
        let model = ReplicaCodeModel::gps_l2c_time_multiplexed(38).expect("valid GPS L2C PRN");
        let sample_rate_hz = 4_000_000.0;
        let block = model
            .sample_block(sample_rate_hz, 137.625, 0.125, 4_100.0, 2.5, 0, 1, 0.75, 64)
            .expect("replica block");
        let iterated = (0..64_u64)
            .map(|sample_index| {
                sample_modulated_replica_at_sample_index(
                    &model,
                    sample_rate_hz,
                    137.625,
                    0.125,
                    4_100.0,
                    2.5,
                    sample_index,
                    1,
                    0.75,
                )
                .expect("single-sample replica")
            })
            .collect::<Vec<_>>();

        assert_eq!(block, iterated);
    }

    #[test]
    fn replica_code_model_fallback_constructors_preserve_code_lengths() {
        let gps = ReplicaCodeModel::gps_l1_ca_or_ones(0);
        let gps_l2c = ReplicaCodeModel::gps_l2c_time_multiplexed_or_ones(0);
        let gps_l5 = ReplicaCodeModel::gps_l5_i_or_ones(0);
        let gps_l5q = ReplicaCodeModel::gps_l5_q_or_ones(0);
        let galileo = ReplicaCodeModel::galileo_e1_cboc_or_ones(0);
        let galileo_e5a = ReplicaCodeModel::galileo_e5a_or_ones(0);
        let galileo_e5a_qpsk = ReplicaCodeModel::galileo_e5a_qpsk_or_ones(0);
        let galileo_e5b = ReplicaCodeModel::galileo_e5b_or_ones(0);
        let galileo_e5b_qpsk = ReplicaCodeModel::galileo_e5b_qpsk_or_ones(0);
        let beidou = ReplicaCodeModel::beidou_b1i_or_ones(0);

        assert_eq!(gps.code_length(), 1023);
        assert_eq!(gps_l2c.code_length(), GPS_L2C_TIME_MULTIPLEXED_CODE_CHIPS);
        assert_eq!(gps_l5.code_length(), 10_230);
        assert_eq!(gps_l5q.code_length(), 10_230);
        assert_eq!(galileo.code_length(), 4092);
        assert_eq!(galileo_e5a.code_length(), 10_230);
        assert_eq!(galileo_e5a_qpsk.code_length(), 10_230);
        assert_eq!(galileo_e5b.code_length(), 10_230);
        assert_eq!(galileo_e5b_qpsk.code_length(), 10_230);
        assert_eq!(beidou.code_length(), 2046);
    }

    #[test]
    fn local_code_model_samples_glonass_period_consistently() {
        let model = LocalCodeModel::glonass_l1_st();
        let samples = model.sample_period(511_000.0, 0.0, 511).expect("GLONASS local code period");
        let expected = sample_glonass_l1_st_code(511_000.0, 0.0, 511).expect("GLONASS reference");
        assert_eq!(samples, expected);
        assert!(!model.supports_secondary_peak_multipath_screening());
    }

    #[test]
    fn local_code_model_samples_gps_l2c_cm_period_consistently() {
        let model = LocalCodeModel::gps_l2c_cm(38).expect("valid GPS L2C CM PRN");
        let samples = model
            .sample_period(GPS_L2C_CM_CODE_RATE_HZ, 0.0, 32)
            .expect("GPS L2C CM local code period");
        let expected = sample_gps_l2c_cm_code(38, GPS_L2C_CM_CODE_RATE_HZ, 0.0, 32)
            .expect("GPS L2C CM reference");

        assert_eq!(samples, expected);
        assert!(model.supports_secondary_peak_multipath_screening());
    }

    #[test]
    fn local_code_model_samples_gps_l2c_cl_period_consistently() {
        let model = LocalCodeModel::gps_l2c_cl(38).expect("valid GPS L2C CL PRN");
        let samples = model
            .sample_period(GPS_L2C_CL_CODE_RATE_HZ, 0.0, 32)
            .expect("GPS L2C CL local code period");
        let expected = sample_gps_l2c_cl_code(38, GPS_L2C_CL_CODE_RATE_HZ, 0.0, 32)
            .expect("GPS L2C CL reference");

        assert_eq!(samples, expected);
        assert!(model.supports_secondary_peak_multipath_screening());
    }

    #[test]
    fn local_code_model_samples_gps_l5_i_period_consistently() {
        let model = LocalCodeModel::gps_l5_i(7).expect("valid GPS L5-I PRN");
        let samples = model
            .sample_period(GPS_L5_PRIMARY_CODE_RATE_HZ, 0.0, 32)
            .expect("GPS L5-I local code period");
        let expected = sample_gps_l5_i_primary_code(7, GPS_L5_PRIMARY_CODE_RATE_HZ, 0.0, 32)
            .expect("GPS L5-I reference");

        assert_eq!(samples, expected);
        assert!(model.supports_secondary_peak_multipath_screening());
    }

    #[test]
    fn local_code_model_samples_gps_l5_q_period_consistently() {
        let model = LocalCodeModel::gps_l5_q(7).expect("valid GPS L5-Q PRN");
        let samples = model
            .sample_period(GPS_L5_PRIMARY_CODE_RATE_HZ, 0.0, 32)
            .expect("GPS L5-Q local code period");
        let expected = sample_gps_l5_q_primary_code(7, GPS_L5_PRIMARY_CODE_RATE_HZ, 0.0, 32)
            .expect("GPS L5-Q reference");

        assert_eq!(samples, expected);
        assert!(model.supports_secondary_peak_multipath_screening());
    }

    #[test]
    fn local_code_model_ones_uses_requested_rate_and_length() {
        let model = LocalCodeModel::ones(7, 2_500.0);
        assert_eq!(model.code_length(), 7);
        assert_eq!(model.code_rate_hz(), 2_500.0);
        assert_eq!(model.sample_value(3.25).expect("ones code"), 1.0);
    }

    #[test]
    fn local_code_model_samples_galileo_boc11_value() {
        let model = LocalCodeModel::galileo_e1_boc11(11).expect("valid Galileo PRN");
        let chip_phase = 12.25;
        let sample = model.sample_value(chip_phase).expect("sample value");
        let expected =
            model.sample_period(GALILEO_E1_CODE_RATE_HZ, chip_phase, 1).expect("single sample")[0];
        assert!(
            (sample.abs() - expected.abs()).abs() < 1.0e-6,
            "sample={sample} expected={expected}"
        );
    }

    #[test]
    fn local_code_model_samples_galileo_period_consistently() {
        let model = LocalCodeModel::galileo_e1_boc11(11).expect("valid Galileo PRN");
        let samples = model.sample_period(4_092_000.0, 0.0, 16).expect("Galileo local code period");
        let expected =
            sample_galileo_e1_boc11_code(11, GalileoE1Channel::E1B, 4_092_000.0, 0.0, 16)
                .expect("Galileo reference");
        assert_eq!(samples, expected);
        assert!(model.supports_secondary_peak_multipath_screening());
    }

    #[test]
    fn local_code_model_samples_galileo_e5a_period_consistently() {
        let model = LocalCodeModel::galileo_e5a_i(11).expect("valid Galileo E5a PRN");
        let samples = model
            .sample_period(GALILEO_E5A_CODE_RATE_HZ, 0.0, 32)
            .expect("Galileo E5a local code period");
        let expected = sample_galileo_e5a_i_primary_code(11, GALILEO_E5A_CODE_RATE_HZ, 0.0, 32)
            .expect("Galileo E5a reference");

        assert_eq!(samples, expected);
        assert!(model.supports_secondary_peak_multipath_screening());
    }

    #[test]
    fn local_code_model_samples_galileo_e5b_period_consistently() {
        let model = LocalCodeModel::galileo_e5b_i(11).expect("valid Galileo E5b PRN");
        let samples = model
            .sample_period(GALILEO_E5B_CODE_RATE_HZ, 0.0, 32)
            .expect("Galileo E5b local code period");
        let expected = sample_galileo_e5b_i_primary_code(11, GALILEO_E5B_CODE_RATE_HZ, 0.0, 32)
            .expect("Galileo E5b reference");

        assert_eq!(samples, expected);
        assert!(model.supports_secondary_peak_multipath_screening());
    }

    #[test]
    fn acquisition_signal_model_samples_glonass_period_consistently() {
        let sat = SatId { constellation: Constellation::Glonass, prn: 8 };
        let channel = GlonassFrequencyChannel::new(-4).expect("valid GLONASS channel");
        let model = AcquisitionSignalModel::from_default_signal(sat, Some(channel))
            .expect("signal model result")
            .expect("GLONASS acquisition model");
        let samples = model.local_code_period(511_000.0).expect("GLONASS local code period");
        let expected = sample_glonass_l1_st_code(511_000.0, 0.0, 511).expect("GLONASS reference");

        assert_eq!(model.signal_band, SignalBand::L1);
        assert_eq!(model.code_rate_hz, 511_000.0);
        assert_eq!(model.code_length, 511);
        assert_eq!(model.code_period_ms, 1);
        assert_eq!(samples, expected);
        assert!(!model.supports_secondary_peak_multipath_screening());
    }

    #[test]
    fn acquisition_signal_model_requires_glonass_channel() {
        let sat = SatId { constellation: Constellation::Glonass, prn: 8 };
        let error = AcquisitionSignalModel::from_default_signal(sat, None)
            .expect_err("GLONASS acquisition model must reject missing channel");

        assert_eq!(error, SignalError::MissingGlonassFrequencyChannel(sat));
    }

    #[test]
    fn acquisition_signal_model_reports_search_center_relative_to_l1_ca() {
        let model = AcquisitionSignalModel::gps_l1_ca(11, 1_023_000.0, 1023, GPS_L1_CA_CARRIER_HZ)
            .expect("GPS acquisition model");

        assert!((model.search_center_hz(125_000.0) - 125_000.0).abs() < f64::EPSILON);
    }

    #[test]
    fn acquisition_signal_model_builds_gps_l5_i_search_metadata() {
        let sat = SatId { constellation: Constellation::Gps, prn: 7 };
        let model = AcquisitionSignalModel::for_sat_signal_band(sat, Some(SignalBand::L5), None)
            .expect("signal model result")
            .expect("GPS L5-I acquisition model");

        assert_eq!(model.signal_band, SignalBand::L5);
        assert_eq!(model.code_rate_hz, 10_230_000.0);
        assert_eq!(model.code_length, 10_230);
        assert_eq!(model.code_period_ms, 1);
        assert_eq!(model.carrier_hz, GPS_L5_CARRIER_HZ);
    }

    #[test]
    fn acquisition_signal_model_builds_gps_l5_q_search_metadata() {
        let sat = SatId { constellation: Constellation::Gps, prn: 7 };
        let model = AcquisitionSignalModel::for_sat_signal(
            sat,
            Some(SignalBand::L5),
            SignalCode::L5Q,
            None,
        )
        .expect("signal model result")
        .expect("GPS L5-Q acquisition model");

        assert_eq!(model.signal_band, SignalBand::L5);
        assert_eq!(model.code_rate_hz, 10_230_000.0);
        assert_eq!(model.code_length, 10_230);
        assert_eq!(model.code_period_ms, 1);
        assert_eq!(model.carrier_hz, GPS_L5_CARRIER_HZ);
    }

    #[test]
    fn acquisition_signal_model_builds_galileo_e5a_search_metadata() {
        let sat = SatId { constellation: Constellation::Galileo, prn: 11 };
        let model = AcquisitionSignalModel::for_sat_signal(
            sat,
            Some(SignalBand::E5),
            SignalCode::E5a,
            None,
        )
        .expect("signal model result")
        .expect("Galileo E5a acquisition model");

        assert_eq!(model.signal_band, SignalBand::E5);
        assert_eq!(model.code_rate_hz, GALILEO_E5A_CODE_RATE_HZ);
        assert_eq!(model.code_length, 10_230);
        assert_eq!(model.code_period_ms, 1);
        assert_eq!(model.carrier_hz, GALILEO_E5A_CARRIER_HZ);
    }

    #[test]
    fn acquisition_signal_model_builds_galileo_e5b_search_metadata() {
        let sat = SatId { constellation: Constellation::Galileo, prn: 11 };
        let model = AcquisitionSignalModel::for_sat_signal(
            sat,
            Some(SignalBand::E5),
            SignalCode::E5b,
            None,
        )
        .expect("signal model result")
        .expect("Galileo E5b acquisition model");

        assert_eq!(model.signal_band, SignalBand::E5);
        assert_eq!(model.code_rate_hz, GALILEO_E5B_CODE_RATE_HZ);
        assert_eq!(model.code_length, 10_230);
        assert_eq!(model.code_period_ms, 1);
        assert_eq!(model.carrier_hz, GALILEO_E5B_CARRIER_HZ);
    }

    #[test]
    fn replica_code_model_builds_default_galileo_signal() {
        let sat = SatId { constellation: Constellation::Galileo, prn: 11 };
        let model = ReplicaCodeModel::from_default_signal(sat)
            .expect("signal model result")
            .expect("Galileo default signal");

        assert_eq!(model.code_length(), 4092);
        assert!((model.code_rate_hz() - GALILEO_E1_CODE_RATE_HZ).abs() < f64::EPSILON);
    }

    #[test]
    fn replica_code_model_builds_galileo_e5a_signal() {
        let sat = SatId { constellation: Constellation::Galileo, prn: 11 };
        let model = ReplicaCodeModel::for_sat_signal(sat, Some(SignalBand::E5), SignalCode::E5a)
            .expect("signal model result")
            .expect("Galileo E5a signal");

        assert_eq!(model.code_length(), 10_230);
        assert_eq!(model.code_rate_hz(), GALILEO_E5A_CODE_RATE_HZ);
    }

    #[test]
    fn replica_code_model_builds_galileo_e5b_signal() {
        let sat = SatId { constellation: Constellation::Galileo, prn: 11 };
        let model = ReplicaCodeModel::for_sat_signal(sat, Some(SignalBand::E5), SignalCode::E5b)
            .expect("signal model result")
            .expect("Galileo E5b signal");

        assert_eq!(model.code_length(), 10_230);
        assert_eq!(model.code_rate_hz(), GALILEO_E5B_CODE_RATE_HZ);
    }

    #[test]
    fn replica_code_model_builds_gps_l2c_time_multiplexed_signal() {
        let model = ReplicaCodeModel::gps_l2c_time_multiplexed(38).expect("valid GPS L2C PRN");

        assert_eq!(model.code_length(), GPS_L2C_TIME_MULTIPLEXED_CODE_CHIPS);
        assert!(
            (model.code_rate_hz() - GPS_L2C_TIME_MULTIPLEXED_CODE_RATE_HZ).abs() < f64::EPSILON
        );
    }

    #[test]
    fn replica_code_model_builds_gps_l5_i_signal() {
        let sat = SatId { constellation: Constellation::Gps, prn: 7 };
        let model = ReplicaCodeModel::for_sat_signal_band(sat, Some(SignalBand::L5))
            .expect("signal model result")
            .expect("GPS L5-I signal");

        assert_eq!(model.code_length(), 10_230);
        assert_eq!(model.code_rate_hz(), 10_230_000.0);
    }

    #[test]
    fn replica_code_model_builds_gps_l5_q_signal() {
        let sat = SatId { constellation: Constellation::Gps, prn: 7 };
        let model = ReplicaCodeModel::for_sat_signal(sat, Some(SignalBand::L5), SignalCode::L5Q)
            .expect("signal model result")
            .expect("GPS L5-Q signal");

        assert_eq!(model.code_length(), 10_230);
        assert_eq!(model.code_rate_hz(), 10_230_000.0);
    }

    #[test]
    fn gps_l5_i_replica_uses_half_power_component_scaling() {
        let model = ReplicaCodeModel::gps_l5_i(7).expect("valid GPS L5-I PRN");
        let sample = sample_modulated_replica_at_time(&model, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 1.0)
            .expect("valid replica");

        assert!((sample.norm() - std::f32::consts::FRAC_1_SQRT_2).abs() < 1.0e-6, "{sample:?}");
    }

    #[test]
    fn gps_l5_q_replica_uses_half_power_component_scaling() {
        let model = ReplicaCodeModel::gps_l5_q(7).expect("valid GPS L5-Q PRN");
        let sample = sample_modulated_replica_at_time(&model, 0.0, 0.0, 0.0, 0.0, 0.0, -1, 1.0)
            .expect("valid replica");

        assert!((sample.norm() - std::f32::consts::FRAC_1_SQRT_2).abs() < 1.0e-6, "{sample:?}");
    }

    #[test]
    fn beidou_b1i_replica_applies_d1_epoch_modulation() {
        let model = ReplicaCodeModel::beidou_b1i(11).expect("valid BeiDou B1I PRN");
        let first_epoch = sample_modulated_replica_at_sample_index(
            &model,
            2_046_000.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0,
            1,
            1.0,
        )
        .expect("first D1 epoch");
        let nh_flipped_epoch = sample_modulated_replica_at_sample_index(
            &model,
            2_046_000.0,
            0.0,
            0.0,
            0.0,
            0.0,
            5 * 2046,
            1,
            1.0,
        )
        .expect("NH-flipped epoch");
        let next_data_symbol_epoch = sample_modulated_replica_at_sample_index(
            &model,
            2_046_000.0,
            0.0,
            0.0,
            0.0,
            0.0,
            (BEIDOU_D1_PRIMARY_EPOCHS_PER_SYMBOL * 2046) as u64,
            -1,
            1.0,
        )
        .expect("next data-symbol epoch");

        assert!(
            (first_epoch.re - beidou_d1_epoch_symbol(&[1], 0).expect("first symbol") as f32).abs()
                <= 1.0e-6
        );
        assert!(
            (nh_flipped_epoch.re - beidou_d1_epoch_symbol(&[1], 5).expect("nh symbol") as f32)
                .abs()
                <= 1.0e-6
        );
        assert!(
            (next_data_symbol_epoch.re
                - beidou_d1_epoch_symbol(&[-1], BEIDOU_D1_PRIMARY_EPOCHS_PER_SYMBOL)
                    .expect("next data symbol") as f32)
                .abs()
                <= 1.0e-6
        );
    }

    #[test]
    fn galileo_e5a_replica_uses_half_power_component_scaling() {
        let model = ReplicaCodeModel::galileo_e5a(11).expect("valid Galileo E5a PRN");
        let sample = sample_modulated_replica_at_time(&model, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 1.0)
            .expect("valid replica");

        assert!((sample.norm() - std::f32::consts::FRAC_1_SQRT_2).abs() < 1.0e-6, "{sample:?}");
    }

    #[test]
    fn galileo_e5a_qpsk_replica_uses_full_signal_power_scaling() {
        let model = ReplicaCodeModel::galileo_e5a_qpsk(11).expect("valid Galileo E5a PRN");
        let sample = sample_modulated_replica_at_time(&model, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 1.0)
            .expect("valid replica");

        assert!((sample.norm() - 1.0).abs() < 1.0e-6, "{sample:?}");
    }

    #[test]
    fn galileo_e5b_replica_uses_half_power_component_scaling() {
        let model = ReplicaCodeModel::galileo_e5b(11).expect("valid Galileo E5b PRN");
        let sample = sample_modulated_replica_at_time(&model, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 1.0)
            .expect("valid replica");

        assert!((sample.norm() - std::f32::consts::FRAC_1_SQRT_2).abs() < 1.0e-6, "{sample:?}");
    }

    #[test]
    fn galileo_e5b_qpsk_replica_uses_full_signal_power_scaling() {
        let model = ReplicaCodeModel::galileo_e5b_qpsk(11).expect("valid Galileo E5b PRN");
        let sample = sample_modulated_replica_at_time(&model, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 1.0)
            .expect("valid replica");

        assert!((sample.norm() - 1.0).abs() < 1.0e-6, "{sample:?}");
    }

    #[test]
    fn gps_l5_q_tracking_replica_interpolates_subchip_boundaries() {
        let model = LocalCodeModel::GpsL5Q { code: vec![1, -1] };

        let sample = model.sample_tracking_value(0.5, 0).expect("tracking sample");

        assert!(sample.abs() < 1.0e-6, "{sample}");
    }

    #[test]
    fn default_signal_carrier_hz_requires_glonass_channel() {
        let sat = SatId { constellation: Constellation::Glonass, prn: 8 };
        let error = default_signal_carrier_hz(sat, None)
            .expect_err("GLONASS carrier lookup must reject missing channel");

        assert_eq!(error, SignalError::MissingGlonassFrequencyChannel(sat));
    }

    #[test]
    fn default_local_code_model_builds_galileo_tracking_code() {
        let sat = SatId { constellation: Constellation::Galileo, prn: 11 };
        let model = default_local_code_model(sat, SignalBand::E1)
            .expect("local code result")
            .expect("Galileo local code");

        assert_eq!(model.code_length(), 4092);
        assert!(model.supports_secondary_peak_multipath_screening());
    }

    #[test]
    fn default_local_code_model_builds_galileo_e5_tracking_code() {
        let sat = SatId { constellation: Constellation::Galileo, prn: 11 };
        let model = default_local_code_model(sat, SignalBand::E5)
            .expect("local code result")
            .expect("Galileo E5 local code");

        assert_eq!(model.code_length(), 10_230);
        assert!((model.code_rate_hz() - GALILEO_E5A_CODE_RATE_HZ).abs() < f64::EPSILON);
        assert!(model.supports_secondary_peak_multipath_screening());
    }

    #[test]
    fn default_local_code_model_builds_gps_l2c_tracking_code() {
        let sat = SatId { constellation: Constellation::Gps, prn: 38 };
        let model = default_local_code_model(sat, SignalBand::L2)
            .expect("local code result")
            .expect("GPS L2C local code");

        assert_eq!(model.code_length(), 10_230);
        assert!((model.code_rate_hz() - GPS_L2C_CM_CODE_RATE_HZ).abs() < f64::EPSILON);
        assert!(model.supports_secondary_peak_multipath_screening());
    }

    #[test]
    fn default_local_code_model_builds_gps_l5_i_tracking_code() {
        let sat = SatId { constellation: Constellation::Gps, prn: 7 };
        let model = default_local_code_model(sat, SignalBand::L5)
            .expect("local code result")
            .expect("GPS L5-I local code");

        assert_eq!(model.code_length(), 10_230);
        assert!((model.code_rate_hz() - 10_230_000.0).abs() < f64::EPSILON);
        assert!(model.supports_secondary_peak_multipath_screening());
    }

    #[test]
    fn default_signal_carrier_hz_for_band_returns_gps_l5_carrier() {
        let sat = SatId { constellation: Constellation::Gps, prn: 7 };
        let carrier = default_signal_carrier_hz_for_band(sat, Some(SignalBand::L5), None)
            .expect("carrier lookup")
            .expect("GPS L5 carrier");

        assert_eq!(carrier, GPS_L5_CARRIER_HZ);
    }

    #[test]
    fn default_signal_carrier_hz_for_band_returns_galileo_e5a_carrier() {
        let sat = SatId { constellation: Constellation::Galileo, prn: 11 };
        let carrier = default_signal_carrier_hz_for_band(sat, Some(SignalBand::E5), None)
            .expect("carrier lookup")
            .expect("Galileo E5a carrier");

        assert_eq!(carrier, GALILEO_E5A_CARRIER_HZ);
    }

    #[test]
    fn default_signal_carrier_hz_for_signal_returns_galileo_e5b_carrier() {
        let sat = SatId { constellation: Constellation::Galileo, prn: 11 };
        let carrier = super::default_signal_carrier_hz_for_signal(
            sat,
            Some(SignalBand::E5),
            SignalCode::E5b,
            None,
        )
        .expect("carrier lookup")
        .expect("Galileo E5b carrier");

        assert_eq!(carrier, GALILEO_E5B_CARRIER_HZ);
    }

    #[test]
    fn resolved_signal_registry_entry_applies_glonass_frequency_channel() {
        let sat = SatId { constellation: Constellation::Glonass, prn: 8 };
        let channel = GlonassFrequencyChannel::new(-4).expect("valid GLONASS channel");
        let entry =
            resolved_signal_registry_entry(sat, SignalBand::L1, SignalCode::Unknown, Some(channel))
                .expect("resolved registry lookup")
                .expect("glonass l1 registry entry");

        assert_eq!(entry.spec.carrier_hz.value(), 1_599_750_000.0);
        assert_eq!(
            entry.default_component().expect("glonass default component").primary_code_chips,
            511
        );
    }

    #[test]
    fn acquisition_signal_model_uses_default_component_metadata() {
        let sat = SatId { constellation: Constellation::Galileo, prn: 11 };
        let model = AcquisitionSignalModel::for_sat_signal(
            sat,
            Some(SignalBand::E5),
            SignalCode::E5b,
            None,
        )
        .expect("acquisition model lookup")
        .expect("galileo e5b acquisition model");

        assert_eq!(model.signal_band, SignalBand::E5);
        assert_eq!(model.code_length, 10_230);
        assert_eq!(model.code_period_ms, 1);
        assert!((model.code_rate_hz - GALILEO_E5B_CODE_RATE_HZ).abs() < f64::EPSILON);
        assert_eq!(model.carrier_hz, GALILEO_E5B_CARRIER_HZ);
    }

    #[test]
    fn default_local_code_model_for_signal_builds_gps_l5_q_tracking_code() {
        let sat = SatId { constellation: Constellation::Gps, prn: 7 };
        let model = default_local_code_model_for_signal(sat, SignalBand::L5, SignalCode::L5Q)
            .expect("local code result")
            .expect("GPS L5-Q local code");

        assert_eq!(model.code_length(), 10_230);
        assert!((model.code_rate_hz() - 10_230_000.0).abs() < f64::EPSILON);
        assert!(model.supports_secondary_peak_multipath_screening());
    }
}
