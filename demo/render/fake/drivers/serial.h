#pragma once
#include "host_stubs.h"
typedef struct serialPinConfig_s { ioTag_t ioTagTx[1], ioTagRx[1]; } serialPinConfig_t;
static inline const serialPinConfig_t *serialPinConfig(void) { static serialPinConfig_t c; return &c; }
