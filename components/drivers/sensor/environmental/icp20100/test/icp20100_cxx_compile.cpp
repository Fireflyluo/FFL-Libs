#include "ffl/icp20100.h"

int main()
{
    ffl_icp20100_config_t config{};

    ffl_icp20100_config_init(&config);
    return config.addr == FFL_ICP20100_DEFAULT_ADDR7 ? 0 : 1;
}
