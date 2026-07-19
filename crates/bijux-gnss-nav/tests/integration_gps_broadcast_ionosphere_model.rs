#![allow(missing_docs)]

use bijux_gnss_core::api::{Llh, Seconds};
use bijux_gnss_nav::api::{IonosphereModel, KlobucharCoefficients, KlobucharModel};

#[derive(Debug, Clone, Copy)]
struct KlobucharDelayCase {
    receiver: Llh,
    azimuth_deg: f64,
    elevation_deg: f64,
    receive_tow_s: f64,
    expected_delay_m: f64,
}

const SAMPLE_KLOBUCHAR: KlobucharCoefficients = KlobucharCoefficients {
    alpha: [0.1212e-7, 0.1490e-7, -0.5960e-7, 0.1192e-6],
    beta: [0.1167e6, -0.2294e6, -0.1311e6, 0.1049e7],
};

const VALIDATION_CASES: [KlobucharDelayCase; 4] = [
    KlobucharDelayCase {
        receiver: Llh { lat_deg: 37.0, lon_deg: -122.0, alt_m: 10.0 },
        azimuth_deg: 120.0,
        elevation_deg: 30.0,
        receive_tow_s: 50_400.0,
        expected_delay_m: 2.649_302_814_714_910_2,
    },
    KlobucharDelayCase {
        receiver: Llh { lat_deg: 0.0, lon_deg: 0.0, alt_m: 10.0 },
        azimuth_deg: 120.0,
        elevation_deg: 15.0,
        receive_tow_s: 50_400.0,
        expected_delay_m: 12.301_824_367_395_728,
    },
    KlobucharDelayCase {
        receiver: Llh { lat_deg: 64.2, lon_deg: 21.1, alt_m: 35.0 },
        azimuth_deg: 245.0,
        elevation_deg: 22.5,
        receive_tow_s: 86_370.0,
        expected_delay_m: 3.092_182_326_719_78,
    },
    KlobucharDelayCase {
        receiver: Llh { lat_deg: -33.9, lon_deg: 151.2, alt_m: 25.0 },
        azimuth_deg: 42.0,
        elevation_deg: 55.0,
        receive_tow_s: 14_400.0,
        expected_delay_m: 3.261_402_696_002_035,
    },
];

#[test]
fn gps_klobuchar_model_matches_validation_vectors() {
    let model = KlobucharModel::new(SAMPLE_KLOBUCHAR);

    for case in VALIDATION_CASES {
        let delay_m = model.delay_m(
            case.receiver,
            case.azimuth_deg,
            case.elevation_deg,
            Seconds(case.receive_tow_s),
        );

        assert!(
            (delay_m - case.expected_delay_m).abs() < 1.0e-12,
            "receiver={:?} azimuth_deg={} elevation_deg={} tow_s={} expected={} actual={}",
            case.receiver,
            case.azimuth_deg,
            case.elevation_deg,
            case.receive_tow_s,
            case.expected_delay_m,
            delay_m,
        );
    }
}

#[test]
fn gps_klobuchar_delay_is_periodic_across_gps_day_rollover() {
    let model = KlobucharModel::new(SAMPLE_KLOBUCHAR);
    let receiver = Llh { lat_deg: 64.2, lon_deg: 21.1, alt_m: 35.0 };
    let azimuth_deg = 245.0;
    let elevation_deg = 22.5;

    let late_day_delay_m = model.delay_m(receiver, azimuth_deg, elevation_deg, Seconds(86_370.0));
    let next_day_delay_m = model.delay_m(receiver, azimuth_deg, elevation_deg, Seconds(172_770.0));

    assert!((late_day_delay_m - next_day_delay_m).abs() < 1.0e-12);
}
