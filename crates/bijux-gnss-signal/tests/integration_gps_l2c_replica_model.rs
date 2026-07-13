#![allow(missing_docs)]

use bijux_gnss_signal::api::{
    sample_gps_l2c_time_multiplexed, sample_modulated_replica_at_time, ReplicaCodeModel,
    GPS_L2C_TIME_MULTIPLEXED_CODE_RATE_HZ,
};

#[test]
fn gps_l2c_replica_model_matches_public_multiplex_samples() {
    let model = ReplicaCodeModel::gps_l2c_time_multiplexed(38).expect("valid GPS L2C PRN");
    let expected =
        sample_gps_l2c_time_multiplexed(38, GPS_L2C_TIME_MULTIPLEXED_CODE_RATE_HZ, 0.0, 8, &[1])
            .expect("public multiplex samples");
    let actual = (0..8)
        .map(|chip_index| {
            sample_modulated_replica_at_time(
                &model,
                0.0,
                0.0,
                0.0,
                0.0,
                chip_index as f64 / GPS_L2C_TIME_MULTIPLEXED_CODE_RATE_HZ,
                1,
                1.0,
            )
            .expect("replica sample")
            .re
        })
        .collect::<Vec<_>>();

    assert_eq!(actual, expected);
}

#[test]
fn gps_l2c_replica_model_flips_only_cm_slots_when_data_bit_changes() {
    let model = ReplicaCodeModel::gps_l2c_time_multiplexed(38).expect("valid GPS L2C PRN");
    let positive = sample_replica_chips(&model, 1);
    let negative = sample_replica_chips(&model, -1);

    for chip_index in 0..positive.len() {
        if chip_index % 2 == 0 {
            assert_eq!(negative[chip_index], -positive[chip_index]);
        } else {
            assert_eq!(negative[chip_index], positive[chip_index]);
        }
    }
}

fn sample_replica_chips(model: &ReplicaCodeModel, data_bit: i8) -> Vec<f32> {
    (0..8)
        .map(|chip_index| {
            sample_modulated_replica_at_time(
                model,
                0.0,
                0.0,
                0.0,
                0.0,
                chip_index as f64 / GPS_L2C_TIME_MULTIPLEXED_CODE_RATE_HZ,
                data_bit,
                1.0,
            )
            .expect("replica sample")
            .re
        })
        .collect()
}
