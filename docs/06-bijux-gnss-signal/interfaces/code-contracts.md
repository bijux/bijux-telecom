---
title: Code Contracts
audience: mixed
type: interfaces
status: canonical
owner: bijux-gnss-signal-docs
last_reviewed: 2026-07-17
---

# Code Contracts

The code-family surface is one of the strongest public contracts in the
repository because acquisition, validation, and synthetic generation all rely
on it.

## Published Guarantees

- supported families have one canonical code generator in this crate
- sampling helpers such as `sample_ca_code`, `sample_gps_l2c_time_multiplexed`,
  and constellation-specific samplers are part of the public signal contract
- family-specific assignments, constants, and secondary-code helpers stay
  grouped by actual signal family

## Contract Boundaries

- public code helpers own signal-code truth
- receiver search policy does not belong here
- navigation-message interpretation does not belong here
- hidden table modules remain implementation detail unless a durable external
  need appears
