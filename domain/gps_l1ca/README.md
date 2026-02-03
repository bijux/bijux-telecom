# GPS L1 C/A Domain

This folder captures GNSS-domain specific constants, references, and expected behaviors for GPS L1 C/A.

## Signals
- Carrier: L1 (1575.42 MHz)
- Code: C/A (1.023 MHz, 1023 chips, 1 ms period)

## Notes
- Code generation is implemented in `bijux-gnss-receiver::ca_code`.
- Acquisition and tracking defaults live in `configs/` profiles.
