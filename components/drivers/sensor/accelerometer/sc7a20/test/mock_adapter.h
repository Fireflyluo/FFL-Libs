#ifndef SC7A20_MOCK_ADAPTER_H
#define SC7A20_MOCK_ADAPTER_H

#include "../inc/sc7a20_core.h"

typedef struct {
    uint8_t regs[256];
    int force_error;
} sc7a20_mock_bus_t;

extern const sc7a20_bus_ops_t g_sc7a20_mock_ops;
void sc7a20_mock_bus_init(sc7a20_mock_bus_t *bus);

#endif
