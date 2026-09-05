#include "ffl/qmc5883p.h"

int main()
{
    ffl_qmc5883p_config_t config{};

    ffl_qmc5883p_config_init(&config);
    return config.range == FFL_QMC5883P_RANGE_8G ? 0 : 1;
}
