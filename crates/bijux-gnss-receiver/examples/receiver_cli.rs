use bijux_gnss_receiver::{data::MemorySamples, Receiver, ReceiverConfig};

fn main() {
    let config = ReceiverConfig::default();
    let receiver = Receiver::new(config.clone());

    let samples = vec![0i16; 10_000];
    let mut source = MemorySamples::new(samples, config.sampling_freq_hz).expect("valid samples");

    if let Err(err) = receiver.run(&mut source) {
        eprintln!("receiver error: {err}");
    }
}
