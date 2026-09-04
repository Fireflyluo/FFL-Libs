/**
 * @file demo_accel_core.c
 * @brief Common implementation shared by both demo driver delivery paths.
 *
 * The full-library target and the cropped-source target compile this file
 * with their own feature definitions. No hardware state, callback, or board
 * resource is owned here; a real driver would receive those through its port
 * interface.
 */
#include "demo_accel.h"

int demo_accel_init(void)
{
    return 0;
}

int demo_accel_read_mg(void)
{
    return 981;
}

const char *demo_accel_profile(void)
{
#if DEMO_ACCEL_FIFO_ENABLED
    return "full";
#else
    return "trimmed";
#endif
}
