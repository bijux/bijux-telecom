use bijux_gnss_receiver::{signal::samples_per_code, Receiver, ReceiverConfig};

#[test]
fn samples_per_code_is_reasonable() {
    let samples = samples_per_code(5_000_000.0, 1_023_000.0, 1023);
    assert!(samples > 0);
}

#[test]
fn receiver_constructs() {
    let receiver = Receiver::new(ReceiverConfig::default());
    assert!(receiver.config().channels > 0);
}
