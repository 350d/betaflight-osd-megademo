/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * AY-3-8910 soft synth — block renderer, platform independent.
 *
 * The platform layer owns a PWM timer and feeds it CCR values. It calls
 * aySynthRender() for a whole block (DMA half-buffer or a small software
 * ring) instead of one sample per timer interrupt, so all generator state
 * stays in registers for the duration of the block.
 *
 * Output samples are PWM compare values in [0, maxDuty].
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#define AY_CLOCK_HZ     1773400u    /* ZX Spectrum 128 */
#define AY_FRAME_HZ     50u

typedef enum {
    AY_DAC_AY = 0,      /* measured AY-3-8910 level curve (default) */
    AY_DAC_LEGACY,      /* old 8-bit Craft-style table */
} ayDacCurve_e;

/* Register-derived generator parameters. Expensive part (divisions) is done
 * here so it can run outside a critical section, then aySynthCommit(). */
typedef struct aySynthFrame_s {
    uint8_t regs[16];
    uint32_t toneStep[3];   /* 0.32 phase step per sample; 0 = tone gated off */
    uint32_t noiseStep;     /* 16.16 */
    uint32_t envStep;       /* 16.16 envelope steps per sample */
    uint8_t toneConst;      /* bit ch: tone gate constant high (mixer off or ultrasonic) */
    uint8_t toneHalf;       /* bit ch: tone above Nyquist — average level (half amplitude) */
    uint8_t envRestart;
} aySynthFrame_t;

typedef void (*aySynthFrameCallback_t)(void);

/* sampleRateHz is the real (post-prescaler) rate the timer produces. */
void aySynthConfigure(uint32_t sampleRateHz, uint16_t maxDuty, uint8_t gain16,
                      bool noiseEnable, uint8_t chMask, ayDacCurve_e dac);
void aySynthReset(void);

void aySynthPrepare(const uint8_t regs[16], aySynthFrame_t *frame);
void aySynthCommit(const aySynthFrame_t *frame);
/* Prepare + commit in one go (render context only, e.g. frame callback). */
void aySynthApplyRegs(const uint8_t regs[16]);

/* Called on the 50 Hz frame boundary from inside aySynthRender(). */
void aySynthSetFrameCallback(aySynthFrameCallback_t cb);
void aySynthResetFrameClock(void);
uint32_t aySynthFrameTicks(void);

void aySynthRender(uint16_t *out, uint32_t count);
