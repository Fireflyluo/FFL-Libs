# SHT40 New Driver

Portable SHT40 driver rebuilt with a unified transaction bus contract.

- Core only depends on bus `xfer` and user-provided `delay_ms` callback.
- Sync/async APIs share the same contract.
- Platform code exists only in adapters.
