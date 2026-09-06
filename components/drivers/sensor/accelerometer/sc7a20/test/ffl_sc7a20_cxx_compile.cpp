#include "ffl/sc7a20.h"

int main()
{
    ffl_sc7a20_config_t config{};
    ffl_sc7a20_device_t device{};

    ffl_sc7a20_config_init(&config);
    return (config.range == FFL_SC7A20_RANGE_2G && !device.core.initialized) ? 0 : 1;
}
