# Glossary

Each entry includes meaning, context, and primary code locations.

- GNSS: Global Navigation Satellite System. Context: overall system. Code: `crates/bijux-gnss-core`, `crates/bijux-gnss-receiver`, `crates/bijux-gnss-nav`.
- PRN: Pseudo-Random Noise identifier for a satellite. Context: acquisition/tracking. Code: `crates/bijux-gnss-core/src/ids.rs`, `crates/bijux-gnss-receiver`.
- C/N0: Carrier-to-noise density ratio. Context: lock quality and weighting. Code: `crates/bijux-gnss-receiver`, `crates/bijux-gnss-nav`.
- DLL: Delay Lock Loop. Context: code tracking. Code: `crates/bijux-gnss-receiver`.
- PLL: Phase Lock Loop. Context: carrier tracking. Code: `crates/bijux-gnss-receiver`.
- FLL: Frequency Lock Loop. Context: carrier tracking pull-in. Code: `crates/bijux-gnss-receiver`.
- EKF: Extended Kalman Filter. Context: nav estimation. Code: `crates/bijux-gnss-nav/src/estimation/ekf/`.
- PPP: Precise Point Positioning. Context: nav estimation using precise products. Code: `crates/bijux-gnss-nav/src/estimation/ppp/`.
- RTK: Real-Time Kinematic. Context: base/rover positioning. Code: `crates/bijux-gnss-nav/src/estimation/rtk/` and `crates/bijux-gnss-receiver/src/rtk/`.
- SD: Single Difference. Context: RTK differencing. Code: `crates/bijux-gnss-receiver/src/rtk/`.
- DD: Double Difference. Context: RTK differencing. Code: `crates/bijux-gnss-receiver/src/rtk/`.
- ISB: Inter-System Bias. Context: multi-constellation. Code: `crates/bijux-gnss-nav/src/estimation/`.
- ZTD: Zenith Tropospheric Delay. Context: atmosphere modeling. Code: `crates/bijux-gnss-nav/src/corrections/`.
- DCB: Differential Code Bias. Context: PPP corrections. Code: `crates/bijux-gnss-nav/src/corrections/`.
- SP3: Precise orbit file format. Context: precise products. Code: `crates/bijux-gnss-nav/src/formats/`.
- CLK: Precise clock file format. Context: precise products. Code: `crates/bijux-gnss-nav/src/formats/`.
