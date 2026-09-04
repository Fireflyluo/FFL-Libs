/**
 * @file main.c
 * @brief Firmware-side consumer of the cropped source-driver target.
 *
 * The target depends on the same driver core as the complete package but
 * compiles it with FIFO disabled by default. The conditional API proves that
 * the public header receives the same feature configuration as the driver.
 */
#include <stdio.h>

#include "demo_accel.h"

int main(void)
{
    if (demo_accel_init() != 0) {
        return 1;
    }

    printf("profile=%s, sample=%d mg\n", demo_accel_profile(), demo_accel_read_mg());
#if DEMO_ACCEL_FIFO_ENABLED
    printf("fifo=%d\n", demo_accel_fifo_capacity());
#else
    printf("fifo=excluded at compile time\n");
#endif
    return 0;
}
