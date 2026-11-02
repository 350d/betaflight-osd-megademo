// Host render: the chip state the renderer reads and the clock the harness drives
#pragma once
#include <stdint.h>
#include <stdbool.h>

#define CHIP_CELLS 480

typedef struct {
    uint8_t nvm[512][54];   // the font (AT7456: 512 glyphs)
    uint8_t code[CHIP_CELLS]; // display memory
    uint8_t inv[CHIP_CELLS];
    uint8_t rb[16];         // row brightness: black << 2 | (3 − white)
    uint8_t vm1;            // bit 7 grey background, 6..4 its level (7 % steps)
    uint8_t hos, vos;       // 32 / 16 = centred
    uint8_t syncMode;       // 1 internal: no camera picture
} hostChip_t;

extern hostChip_t chip;
extern uint64_t hostCycles;
extern bool hostAudioOn;

void hostChipInit(void);
void hostSetTimeUs(uint64_t us);
