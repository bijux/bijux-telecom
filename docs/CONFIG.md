# Configuration

Receiver configuration is defined in TOML files and validated against JSON schema.

Key sections:
- `receiver`: sample rate, IF, quantization
- `acquisition`: Doppler bins, thresholds
- `tracking`: loop bandwidths, correlator spacing
- `navigation`: solver settings and weighting

Use:
```bash
bijux gnss validate-config --file configs/receiver.toml
bijux gnss config-schema --out schemas/receiver_config.schema.json
```
