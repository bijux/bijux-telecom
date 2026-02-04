#![no_main]

use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    if let Ok(text) = std::str::from_utf8(data) {
        let _ = bijux_gnss_nav::api::parse_rinex_obs_header(text);
    }
});
