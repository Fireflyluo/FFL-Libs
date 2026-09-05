# API REFERENCE

Include the public interface with `#include "ffl/sht40.h"`.

## Lifecycle

1. Fill an `ffl_transport_t` with an I2C transport and endpoint.
2. Call `ffl_sht40_bind()` once for each device handle.
3. Initialize `ffl_sht40_config_t` with `ffl_sht40_config_init()` and call
   `ffl_sht40_init()`.
4. Read measurements with `ffl_sht40_read_sample()` or its async counterpart.

`ffl_sht40_bind()` does not take ownership of the transport or time objects;
they must remain valid while the device handle is used.

`ffl_sht40_device_t` currently aliases the existing internal device storage so
the object target remains source-compatible during migration. Applications
should include only `ffl/sht40.h` and should not access the aliased core fields.

## Functions

- `ffl_sht40_config_init`: initializes the configuration with I2C address
  `0x46`.
- `ffl_sht40_bind`: binds a generic I2C transport and optional millisecond
  delay provider.
- `ffl_sht40_set_i2c_addr`: changes the 7-bit address before a transaction.
- `ffl_sht40_init`: performs the sensor initialization sequence.
- `ffl_sht40_soft_reset`: requests a software reset.
- `ffl_sht40_read_serial`: reads the 32-bit serial number.
- `ffl_sht40_read_sample`: synchronously measures temperature and humidity.
- `ffl_sht40_heater`: runs one of the supported heater commands.
- `ffl_sht40_soft_reset_async`, `ffl_sht40_read_sample_async`, and
  `ffl_sht40_cancel_async`: asynchronous operations.

All functions return `0` on success and a negative errno-compatible value on
failure.
