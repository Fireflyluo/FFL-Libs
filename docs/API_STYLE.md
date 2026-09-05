# Public API Style

## Scope

This document defines public C APIs under `components/`. It applies to new
components immediately and to existing components as they are migrated.

## Names

- Functions use `ffl_<component>_<verb>_<object>`.
- Public types use `ffl_<component>_<noun>_t`.
- Callback types use `ffl_<component>_<event>_fn`.
- Enumerators and constants use `FFL_<COMPONENT>_<VALUE>`.
- Public headers are included as `ffl/<component>.h`.
- Xmake targets use `ffl.<component>`.

## Lifecycle

Hardware drivers use `bind`, `config_init`, `init`, `configure`, `read_*`,
`*_async`, and `deinit` consistently. `get_*` reads cached state; `read_*`
starts a hardware transaction.

## Parameters and Results

- The device handle is the first parameter.
- Input parameters precede output pointers.
- Async callbacks and their `user` pointer are the final parameters.
- APIs return `0` on success and a negative errno-compatible status on failure.

## Driver Ports

Use `ffl/driver_port.h` for transport and time capabilities. A driver core
must not include a board, MCU, vendor HAL, GPIO, I2C, SPI, DMA, or IRQ header.
Official ports are optional: consuming projects may implement
`ffl_transport_ops_t` directly.
