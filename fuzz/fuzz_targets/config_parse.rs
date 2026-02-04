#![no_main]

use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    if let Ok(text) = std::str::from_utf8(data) {
        let _ = toml::from_str::<bijux_gnss_receiver::api::ReceiverProfile>(text);
    }
});
