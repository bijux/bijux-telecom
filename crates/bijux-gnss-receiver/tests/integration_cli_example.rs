#![allow(missing_docs)]
use bijux_gnss_receiver::api::{data::MemorySamples, Receiver, ReceiverConfig};

#[test]
fn receiver_cli_smoke() {
    let config = ReceiverConfig::default();
    let receiver = Receiver::new(config.clone());

    let samples = vec![0i16; 10_000];
    let mut source = MemorySamples::new(samples, config.sampling_freq_hz).expect("valid samples");

    let result = receiver.run(&mut source);
    assert!(result.is_ok(), "receiver error: {result:?}");
}
