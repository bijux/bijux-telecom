#![allow(missing_docs)]
use bijux_gnss_receiver::api::{Receiver, ReceiverPipelineConfig};
use bijux_gnss_signal::api::samples_per_code;

#[test]
fn samples_per_code_is_reasonable() {
    let samples = samples_per_code(5_000_000.0, 1_023_000.0, 1023);
    assert!(samples > 0);
}

#[test]
fn receiver_constructs() {
    let runtime = bijux_gnss_receiver::api::ReceiverRuntime::default();
    let receiver = Receiver::new(ReceiverPipelineConfig::default(), runtime);
    assert!(receiver.config().channels > 0);
}
