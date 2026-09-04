/**
 * @file demo_accel.h
 * @brief Dual-entry demonstration driver's public API.
 *
 * This mock accelerometer has no MCU or HAL dependency. The same API is
 * consumed either from a complete static library or from a source target
 * whose FIFO capability is selected by the final firmware build.
 *
 * The application owns all board integration in a real driver. This example
 * only exposes deterministic host-side behavior for build verification.
 */
#ifndef FFL_DEMO_ACCEL_H
#define FFL_DEMO_ACCEL_H

#ifndef DEMO_ACCEL_FIFO_ENABLED
#define DEMO_ACCEL_FIFO_ENABLED 1
#endif

int demo_accel_init(void);
int demo_accel_read_mg(void);
const char *demo_accel_profile(void);

#if DEMO_ACCEL_FIFO_ENABLED
int demo_accel_fifo_capacity(void);
#endif

#endif
