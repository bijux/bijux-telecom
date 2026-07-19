use bijux_gnss_core::api::{
    Constellation, SignalBand, SignalCode, SignalComponentRole, SignalSubcarrierSpec,
};
use bijux_gnss_signal::api::signal_registry;

fn assert_close(actual: f64, expected: f64) {
    assert!((actual - expected).abs() <= 1.0e-12, "actual={actual} expected={expected}");
}

#[test]
fn gps_l1_ca_registry_exposes_data_component_timing() {
    let entry = signal_registry(Constellation::Gps, SignalBand::L1, SignalCode::Ca)
        .expect("gps l1 ca must be registered");
    let component = entry.default_component().expect("gps l1 ca must expose a default component");

    assert_eq!(entry.default_component_role, SignalComponentRole::Data);
    assert_eq!(entry.components.len(), 1);
    assert_eq!(component.role, SignalComponentRole::Data);
    assert_eq!(component.primary_code_chips, 1_023);
    assert_close(component.primary_code_rate_hz, 1_023_000.0);
    assert_close(component.primary_code_period_s, 0.001);
    assert_eq!(component.secondary_code, None);
    assert_eq!(component.subcarrier, SignalSubcarrierSpec::None);
    assert_eq!(component.symbol_period_s, Some(0.020));
    assert_close(f64::from(component.power_fraction), 1.0);
}

#[test]
fn gps_l2c_registry_exposes_data_and_pilot_components() {
    let entry = signal_registry(Constellation::Gps, SignalBand::L2, SignalCode::L2C)
        .expect("gps l2c must be registered");
    let data = entry.component(SignalComponentRole::Data).expect("gps l2c data component");
    let pilot = entry.component(SignalComponentRole::Pilot).expect("gps l2c pilot component");

    assert_eq!(entry.default_component_role, SignalComponentRole::Data);
    assert_eq!(entry.components.len(), 2);
    assert_eq!(entry.code_length, Some(10_230));
    assert_eq!(data.primary_code_chips, 10_230);
    assert_close(data.primary_code_rate_hz, 511_500.0);
    assert_close(data.primary_code_period_s, 0.020);
    assert_eq!(data.symbol_period_s, Some(0.020));
    assert_eq!(pilot.primary_code_chips, 767_250);
    assert_close(pilot.primary_code_rate_hz, 511_500.0);
    assert_close(pilot.primary_code_period_s, 1.5);
    assert_eq!(pilot.symbol_period_s, None);
    assert_close(f64::from(data.power_fraction + pilot.power_fraction), 1.0);
}

#[test]
fn gps_l5q_registry_exposes_pilot_secondary_code_timing() {
    let entry = signal_registry(Constellation::Gps, SignalBand::L5, SignalCode::L5Q)
        .expect("gps l5q must be registered");
    let component = entry.default_component().expect("gps l5q default component");
    let secondary = component.secondary_code.expect("gps l5q secondary code");

    assert_eq!(entry.default_component_role, SignalComponentRole::Pilot);
    assert_eq!(component.role, SignalComponentRole::Pilot);
    assert_eq!(component.symbol_period_s, None);
    assert_eq!(secondary.chip_count, 20);
    assert_close(secondary.chip_period_s, 0.001);
}

#[test]
fn galileo_e5b_registry_exposes_balanced_data_and_pilot_components() {
    let entry = signal_registry(Constellation::Galileo, SignalBand::E5, SignalCode::E5b)
        .expect("galileo e5b must be registered");
    let data = entry.component(SignalComponentRole::Data).expect("galileo e5b data component");
    let pilot = entry.component(SignalComponentRole::Pilot).expect("galileo e5b pilot component");

    assert_eq!(entry.default_component_role, SignalComponentRole::Data);
    assert_eq!(data.secondary_code.expect("e5b-i secondary").chip_count, 4);
    assert_eq!(data.symbol_period_s, Some(0.004));
    assert_eq!(pilot.secondary_code.expect("e5b-q secondary").chip_count, 100);
    assert_eq!(pilot.symbol_period_s, None);
    assert_close(f64::from(data.power_fraction), 0.5);
    assert_close(f64::from(pilot.power_fraction), 0.5);
}

#[test]
fn galileo_e1c_registry_exposes_cboc_pilot_metadata() {
    let entry = signal_registry(Constellation::Galileo, SignalBand::E1, SignalCode::E1C)
        .expect("galileo e1c must be registered");
    let component = entry.default_component().expect("galileo e1c default component");

    assert_eq!(entry.default_component_role, SignalComponentRole::Pilot);
    assert_eq!(component.role, SignalComponentRole::Pilot);
    assert_eq!(component.secondary_code.expect("e1c secondary code").chip_count, 25);
    assert_eq!(
        component.subcarrier,
        SignalSubcarrierSpec::Cboc { boc11_weight: 0.953_462_6, boc61_weight: 0.301_511_35 }
    );
}

#[test]
fn glonass_l1_registry_exposes_ten_millisecond_symbol_timing() {
    let entry = signal_registry(Constellation::Glonass, SignalBand::L1, SignalCode::Unknown)
        .expect("glonass l1 must be registered");
    let component = entry.default_component().expect("glonass l1 default component");

    assert_eq!(component.role, SignalComponentRole::Data);
    assert_eq!(component.primary_code_chips, 511);
    assert_close(component.primary_code_rate_hz, 511_000.0);
    assert_close(component.primary_code_period_s, 0.001);
    assert_eq!(component.symbol_period_s, Some(0.010));
}
