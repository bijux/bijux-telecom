#![allow(missing_docs)]

use bijux_gnss_core::api::{
    Constellation, GpsTime, SatId, SignalBand, SignalCode, GPS_L1_CA_CARRIER_HZ,
    GPS_L2_PY_CARRIER_HZ,
};
use bijux_gnss_nav::api::{
    parse_antex_receiver_calibrations, parse_antex_satellite_calibrations, BiasSinexBiasSource,
    BiasSinexProvider,
};

#[test]
fn precise_correction_queries_are_unambiguous_by_signal_and_epoch() {
    let satellite_antex = [
        antex_line("", "START OF ANTENNA"),
        antex_line(&format!("{:<20}{:<20}", "GPS TEST", "G01"), "TYPE / SERIAL NO"),
        antex_line("  2020  01  01  00  00  00", "VALID FROM"),
        antex_line("G01", "START OF FREQUENCY"),
        antex_line("   100.0     0.0  1000.0", "NORTH / EAST / UP"),
        antex_line("     0.0    20.0    10.0", "ZEN1 / ZEN2 / DZEN"),
        antex_line("NOAZI       0.0    10.0    20.0", "NOAZI"),
        antex_line("", "END OF FREQUENCY"),
        antex_line("G02", "START OF FREQUENCY"),
        antex_line("   200.0     0.0  1200.0", "NORTH / EAST / UP"),
        antex_line("     0.0    20.0    10.0", "ZEN1 / ZEN2 / DZEN"),
        antex_line("NOAZI       5.0    15.0    25.0", "NOAZI"),
        antex_line("", "END OF FREQUENCY"),
        antex_line("", "END OF ANTENNA"),
    ]
    .join("");
    let receiver_antex = [
        antex_line("", "START OF ANTENNA"),
        antex_line(&format!("{:<20}{:<20}{:<20}", "AOAD/M_T", "NONE", "12345"), "TYPE / SERIAL NO"),
        antex_line("G01", "START OF FREQUENCY"),
        antex_line("   100.0   200.0  1200.0", "NORTH / EAST / UP"),
        antex_line("    0.0    20.0    10.0", "ZEN1 / ZEN2 / DZEN"),
        antex_line("NOAZI       0.0    10.0    20.0", "NOAZI"),
        antex_line("    0.0     0.0    20.0    40.0", ""),
        antex_line("   90.0    10.0    30.0    50.0", ""),
        antex_line("", "END OF FREQUENCY"),
        antex_line("", "END OF ANTENNA"),
    ]
    .join("");
    let satellite_calibrations =
        parse_antex_satellite_calibrations(&satellite_antex).expect("satellite ANTEX parse");
    let receiver_calibrations =
        parse_antex_receiver_calibrations(&receiver_antex).expect("receiver ANTEX parse");
    let sat = SatId { constellation: Constellation::Gps, prn: 1 };
    let epoch = Some(GpsTime { week: 2138, tow_s: 259_218.0 });

    let satellite_l1_pcv_m = satellite_calibrations
        .phase_center_variation_m(sat, SignalBand::L1, epoch, 75.0, None)
        .expect("satellite L1 PCV");
    let satellite_l2_pcv_m = satellite_calibrations
        .phase_center_variation_m(sat, SignalBand::L2, epoch, 75.0, None)
        .expect("satellite L2 PCV");
    let receiver_l1_pcv_m = receiver_calibrations
        .phase_center_variation_m("aoad/m_t none", SignalBand::L1, epoch, 75.0, Some(45.0))
        .expect("receiver L1 azimuth PCV");

    assert!((satellite_l1_pcv_m - 0.015).abs() < 1.0e-12);
    assert!((satellite_l2_pcv_m - 0.020).abs() < 1.0e-12);
    assert!((receiver_l1_pcv_m - 0.035).abs() < 1.0e-12);

    let bias_sinex = "\
+BIAS/DESCRIPTION
TIME_SYSTEM G
-BIAS/DESCRIPTION
+BIAS/SOLUTION
OSB G063 G01 C1C 2016:271:00000 2016:272:00000 m 1.2500 0.0500
OSB G063 G01 C2W 2016:271:00000 2016:272:00000 m 2.0000 0.1200
-BIAS/SOLUTION
%=ENDBIA
";
    let biases = bias_sinex.parse::<BiasSinexProvider>().expect("Bias-SINEX parse");
    let l1 = bijux_gnss_core::api::SigId { sat, band: SignalBand::L1, code: SignalCode::Ca };
    let l2 = bijux_gnss_core::api::SigId { sat, band: SignalBand::L2, code: SignalCode::Py };
    let bias_epoch = GpsTime::from_seconds(1_158_969_600.0);
    let bias_query_time = Some(bias_epoch);

    let osb = biases.code_bias_at(l1, bias_query_time).expect("typed OSB query");
    let iono_free = biases
        .iono_free_code_bias_at(
            l1,
            l2,
            GPS_L1_CA_CARRIER_HZ.value(),
            GPS_L2_PY_CARRIER_HZ.value(),
            bias_query_time,
        )
        .expect("typed ionosphere-free query");

    assert_eq!(osb.source, BiasSinexBiasSource::ObservableSpecific);
    assert_eq!(osb.reference_signal, None);
    assert!((osb.bias_m - 1.25).abs() < 1.0e-12);
    assert!((osb.uncertainty_m.expect("OSB uncertainty") - 0.05).abs() < 1.0e-12);
    assert_eq!(iono_free.reference_signal, Some(l2));
    assert_eq!(iono_free.source, BiasSinexBiasSource::IonosphereFreeObservableCombination);
    assert!(iono_free.window_start.to_seconds() <= bias_epoch.to_seconds());
    assert!(iono_free.window_end.to_seconds() >= bias_epoch.to_seconds());
}

fn antex_line(value: &str, label: &str) -> String {
    format!("{value:<60}{label}\n")
}
