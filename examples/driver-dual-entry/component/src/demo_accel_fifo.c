/**
 * @file demo_accel_fifo.c
 * @brief Optional FIFO capability of the dual-entry demonstration driver.
 *
 * This translation unit is deliberately absent from the cropped target when
 * FIFO is disabled. It demonstrates source-file-level Flash and RAM feature
 * trimming rather than merely hiding an API behind a preprocessor guard.
 */
#include "demo_accel.h"

int demo_accel_fifo_capacity(void)
{
    return 32;
}
