#include "ffl/driver_port.h"

int main()
{
    ffl_endpoint_t i2c = ffl_endpoint_i2c7(0x44u);
    ffl_endpoint_t spi = ffl_endpoint_spi();
    ffl_gpio_t gpio = {};
    ffl_irq_t irq = {};

    return ffl_endpoint_is_valid(&i2c) && ffl_endpoint_is_valid(&spi) &&
                   !ffl_gpio_is_valid(&gpio) && !ffl_irq_is_valid(&irq)
               ? 0
               : 1;
}
