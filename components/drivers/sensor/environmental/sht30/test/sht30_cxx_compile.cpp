#include "ffl/sht30.h"

int main()
{
    ffl_sht30_device_t device = {};
    ffl_sht30_config_t config = {};
    ffl_transport_t transport = {};

    transport.endpoint = ffl_endpoint_i2c7(0x44u);
    ffl_sht30_config_init(&config);
    return ffl_sht30_bind(&device, &transport, nullptr, nullptr) == 0 ? 1 : 0;
}
