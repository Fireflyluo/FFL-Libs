/**
 * @file main.c
 * @brief Firmware-side consumer of the complete local Xmake package.
 *
 * The package installs the same driver core as the cropped example, but its
 * fixed full profile always exposes FIFO. Board-specific port code remains
 * the responsibility of the consuming firmware in a real driver.
 */
#include <stdio.h>

#include "demo_accel.h"

int main(void)
{
    if (demo_accel_init() != 0) {
        return 1;
    }

    printf("profile=%s, sample=%d mg, fifo=%d\n",
           demo_accel_profile(),
           demo_accel_read_mg(),
           demo_accel_fifo_capacity());
    return 0;
}
