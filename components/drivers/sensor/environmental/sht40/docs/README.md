# SHT40 New Driver

Portable SHT40 driver rebuilt with a unified transaction bus contract.

- Use `ffl/sht40.h` and bind an I2C transport plus required `delay_ms` callback.
- Sync/async APIs share the same contract.
- Platform code exists only in adapters.
- Device objects must be zero-initialized and bound before the first init;
  rebinding after initialization, other operations, and address changes return
  `-EBUSY` while an asynchronous transfer is pending.
- `ffl_sht40_cancel_async()` also returns `-EBUSY` while an asynchronous
  transfer is submitting or a callback is processing its command/read stage; retry it
  after that short transition completes.
- When a transport completes inline from `xfer()`, the driver keeps the device held
  until the outermost `xfer()` returns. Do not start another operation on that device
  from the user completion callback; it returns `-EBUSY`.
