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
 * Hot-path layout (per output sample, all in registers):
 *   tone:  phase += step; gate = phase >> 31 (ASR → 0 / ~0)
 *   mix:   sum(ampN[ch] & gate[ch])         — ampN already noise-gated
 * Noise / envelope only touch amplitudes when their counter rolls over,
 * which is rare relative to the sample rate. Channels whose tone is gated
 * off (mixer bit or ultrasonic) run with a constant gate; their real phase
 * is advanced once per segment.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_CHIPTUNE

#include "drivers/ay_synth.h"

#ifndef FAST_CODE_NOINLINE
#define FAST_CODE_NOINLINE
#endif

#define AY_PHASE_ONE    0x10000u
#define AY_TONE_CONST   0x80000000u     /* MSB set → ASR gate is always ~0 */

/* AY-3-8910 DAC, normalised to 65535 (MAME / ayumi measurements). */
static const uint16_t ayDacCurve[16] = {
    0, 655, 947, 1380, 2012, 2985, 4227, 7036,
    8296, 13434, 19150, 24434, 32278, 41636, 52794, 65535,
};

/* Previous 8-bit table, kept selectable for A/B listening. */
static const uint16_t ayDacLegacy[16] = {
    0, 0, 1, 1, 1, 2, 3, 4, 7, 8, 13, 19, 24, 32, 41, 52,
};

/* Two envelope segments per shape; shapes that hold stop at index 31. */
static const uint8_t ayEnvShapes[16][32] = {
    {15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0,15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0},
    {15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0,0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15},
    {15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15},
    {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15},
    {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15},
    {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0},
    {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
};

typedef struct aySynthState_s {
    /* configuration */
    uint32_t rateHz;
    uint32_t toneK;             /* AY_CLOCK · 2^32 / (16 · rate) >> toneShift (fits 32 bits) */
    uint8_t toneShift;
    uint32_t stepK;             /* AY_CLOCK · 2^16 / (16 · rate) — noise & envelope */
    uint16_t maxDuty;
    uint8_t gain16;
    uint8_t noiseEnable;
    uint8_t chMask;
    uint8_t dac;
    uint16_t dacDuty[16];       /* per-channel PWM counts per AY volume */
    uint16_t envDuty[32];       /* dacDuty[] through the current envelope shape */

    /* current register frame */
    aySynthFrame_t cur;
    uint32_t fixAmp[3];         /* fixed-volume amplitude (0 for env channels) */
    uint32_t envSel[3];         /* ~0 if channel follows the envelope */
    uint32_t noiseOr[3];        /* ~0 if noise does not gate this channel */
    uint8_t ampShift[3];        /* 1 for ultrasonic tone (average level) */
    bool noiseUsed;
    bool envUsed;
    bool needClamp;             /* gain > 16: three channels can exceed full scale */
    /* Final per-channel amplitude for every (noise bit, envelope level): the render
     * loop reloads three entries on a noise/envelope event instead of recomputing. */
    uint16_t ampTab[2][32][4];
    bool envDutyChanged;        /* envDuty[] rebuilt — env channels need their ampTab column */

    /* generators */
    uint32_t tonePh[3];
    uint32_t noisePh;
    uint32_t lfsr;
    uint32_t envPh;
    uint8_t envLvl;
    uint8_t envShape;
    uint8_t envHold;
    uint8_t envHoldShape;

    /* 50 Hz frame clock */
    uint32_t frameLeft;
    uint32_t frameAcc;
    uint32_t frameTicks;
    aySynthFrameCallback_t frameCb;
} aySynthState_t;

static aySynthState_t ay;

static void ayBuildEnvDuty(void)
{
    const uint8_t *shape = ayEnvShapes[ay.envShape & 15];
    for (unsigned i = 0; i < 32; i++) {
        ay.envDuty[i] = ay.dacDuty[shape[i]];
    }
    ay.envDutyChanged = true;
}

static void ayBuildDacDuty(void)
{
    const uint16_t *curve = (ay.dac == AY_DAC_LEGACY) ? ayDacLegacy : ayDacCurve;
    const uint32_t curveMax = curve[15];
    /* Full scale = three channels at volume 15, same normalisation as before. */
    const uint64_t num = (uint64_t)ay.maxDuty * ay.gain16;
    const uint64_t den = (uint64_t)curveMax * 3u * 16u;
    for (unsigned v = 0; v < 16; v++) {
        ay.dacDuty[v] = (uint16_t)((curve[v] * num + den / 2u) / den);
    }
    ayBuildEnvDuty();
}

static void ayBuildAmpColumn(unsigned ch)
{
    const uint32_t fix = ay.fixAmp[ch];
    const uint32_t sel = ay.envSel[ch];
    const uint32_t nOr = ay.noiseOr[ch];
    const uint8_t sh = ay.ampShift[ch];
    for (unsigned lvl = 0; lvl < 32; lvl++) {
        const uint32_t base = (fix | (ay.envDuty[lvl] & sel)) >> sh;
        ay.ampTab[0][lvl][ch] = (uint16_t)(base & nOr);    /* noise bit low */
        ay.ampTab[1][lvl][ch] = (uint16_t)base;            /* noise bit high */
    }
}

/* Derive per-channel amplitude masks from the latched registers. */
static void ayUpdateChannels(void)
{
    const uint8_t *r = ay.cur.regs;
    const uint8_t mixer = r[7];
    ay.noiseUsed = false;
    ay.envUsed = false;
    for (unsigned ch = 0; ch < 3; ch++) {
        const bool on = (ay.chMask >> ch) & 1u;
        const bool env = (r[8 + ch] >> 4) & 1u;
        const bool noiseOn = ay.noiseEnable && !((mixer >> (3 + ch)) & 1u);
        const uint32_t sel = (on && env) ? ~0u : 0u;
        const uint32_t fix = (on && !env) ? ay.dacDuty[r[8 + ch] & 0x0f] : 0u;
        const uint32_t nOr = noiseOn ? 0u : ~0u;
        const uint8_t sh = (ay.cur.toneHalf >> ch) & 1u;
        /* Volumes change most frames; rebuild only the columns that moved. */
        if (sel != ay.envSel[ch] || fix != ay.fixAmp[ch] || nOr != ay.noiseOr[ch]
            || sh != ay.ampShift[ch] || (sel && ay.envDutyChanged)) {
            ay.envSel[ch] = sel;
            ay.fixAmp[ch] = fix;
            ay.noiseOr[ch] = nOr;
            ay.ampShift[ch] = sh;
            ayBuildAmpColumn(ch);
        }
        if (on && noiseOn) {
            ay.noiseUsed = true;
        }
        if (on && env) {
            ay.envUsed = true;
        }
    }
    ay.envDutyChanged = false;
}

void aySynthPrepare(const uint8_t regs[16], aySynthFrame_t *f)
{
    memcpy(f->regs, regs, 16);
    f->toneConst = 0;
    f->toneHalf = 0;
    const uint8_t mixer = regs[7];
    for (unsigned ch = 0; ch < 3; ch++) {
        uint32_t tp = ((uint32_t)(regs[ch * 2 + 1] & 0x0f) << 8) | regs[ch * 2];
        if (tp == 0) {
            tp = 1;                 /* AY treats period 0 as 1 */
        }
        /* One 32-bit UDIV; toneShift keeps ≥ 29 significant bits (< 0.001 cent). */
        const uint32_t q = ay.toneK / tp;
        const uint64_t step = (uint64_t)q << ay.toneShift;
        if (step >= 0x80000000ull) {
            /* Tone at or above Nyquist: on the chip it is far above audio and the
             * output filter averages it, so play the mean level instead of an
             * aliased square. */
            f->toneStep[ch] = (uint32_t)step;
            f->toneConst |= (uint8_t)(1u << ch);
            f->toneHalf |= (uint8_t)(1u << ch);
        } else {
            f->toneStep[ch] = (uint32_t)step;
        }
        if ((mixer >> ch) & 1u) {
            f->toneConst |= (uint8_t)(1u << ch);
            f->toneHalf &= (uint8_t)~(1u << ch);
        }
    }
    uint32_t np = regs[6] & 0x1f;
    if (np == 0) {
        np = 1;
    }
    f->noiseStep = ay.stepK / np;
    uint32_t ep = ((uint32_t)regs[12] << 8) | regs[11];
    if (ep == 0) {
        ep = 1;
    }
    /* 16 envelope steps per 256·EP clocks → one step every 16·EP clocks. */
    f->envStep = ay.stepK / ep;
    f->envRestart = (regs[13] != 0xff) ? 1u : 0u;
}

void aySynthCommit(const aySynthFrame_t *f)
{
    ay.cur = *f;
    if (f->envRestart) {
        ay.envShape = f->regs[13] & 0x0f;
        ay.envHoldShape = (!(ay.envShape & 8) || (ay.envShape & 1)) ? 1u : 0u;
        ay.envPh = 0;
        ay.envLvl = 0;
        ay.envHold = 0;
        ayBuildEnvDuty();
    }
    ayUpdateChannels();
}

void aySynthApplyRegs(const uint8_t regs[16])
{
    aySynthFrame_t f;
    aySynthPrepare(regs, &f);
    aySynthCommit(&f);
}

static void ayNextFrame(void)
{
    ay.frameAcc += ay.rateHz;
    const uint32_t n = ay.frameAcc / AY_FRAME_HZ;
    ay.frameAcc -= n * AY_FRAME_HZ;
    ay.frameLeft = n ? n : 1u;
}

void aySynthConfigure(uint32_t sampleRateHz, uint16_t maxDuty, uint8_t gain16,
                      bool noiseEnable, uint8_t chMask, ayDacCurve_e dac)
{
    if (sampleRateHz < 1000u) {
        sampleRateHz = 1000u;
    }
    const bool rateChanged = ay.rateHz != sampleRateHz;
    ay.rateHz = sampleRateHz;
    const uint64_t toneK = ((uint64_t)AY_CLOCK_HZ << 32) / (16ull * sampleRateHz);
    ay.toneShift = 0;
    while ((toneK >> ay.toneShift) > 0xffffffffull) {
        ay.toneShift++;
    }
    ay.toneK = (uint32_t)(toneK >> ay.toneShift);
    ay.stepK = (uint32_t)(((uint64_t)AY_CLOCK_HZ << 16) / (16ull * sampleRateHz));
    ay.maxDuty = maxDuty;
    ay.gain16 = gain16;
    ay.noiseEnable = noiseEnable ? 1u : 0u;
    ay.chMask = chMask & 7u;
    ay.dac = (uint8_t)dac;
    ayBuildDacDuty();
    ay.needClamp = 3u * ay.dacDuty[15] > ay.maxDuty;

    /* Steps depend on the rate — re-derive from the latched registers. */
    aySynthFrame_t f;
    aySynthPrepare(ay.cur.regs, &f);
    f.envRestart = 0;
    aySynthCommit(&f);
    if (rateChanged) {
        aySynthResetFrameClock();
    }
}

void aySynthReset(void)
{
    uint8_t regs[16];
    memset(regs, 0, sizeof(regs));
    regs[7] = 0x3f;
    regs[13] = 0xff;
    for (unsigned ch = 0; ch < 3; ch++) {
        ay.tonePh[ch] = 0;
    }
    ay.noisePh = 0;
    ay.lfsr = 1;
    ay.envPh = 0;
    ay.envLvl = 0;
    ay.envShape = 0;
    ay.envHold = 0;
    ay.envHoldShape = 1;
    ayBuildEnvDuty();
    aySynthApplyRegs(regs);
    aySynthResetFrameClock();
}

void aySynthSetFrameCallback(aySynthFrameCallback_t cb)
{
    ay.frameCb = cb;
}

void aySynthResetFrameClock(void)
{
    ay.frameAcc = 0;
    ay.frameTicks = 0;
    ayNextFrame();
}

uint32_t aySynthFrameTicks(void)
{
    return ay.frameTicks;
}

/* Advance an unheard envelope analytically (no channel uses it). */
static void ayEnvSkip(uint32_t n)
{
    if (ay.envHold) {
        return;
    }
    const uint64_t total = (uint64_t)ay.envPh + (uint64_t)ay.cur.envStep * n;
    ay.envPh = (uint32_t)(total & 0xffffu);
    const uint64_t lvl = (uint64_t)ay.envLvl + (total >> 16);
    if (lvl >= 32u) {
        if (ay.envHoldShape) {
            ay.envLvl = 31;
            ay.envHold = 1;
            ay.envPh = 0;
        } else {
            ay.envLvl = (uint8_t)(lvl & 31u);
        }
    } else {
        ay.envLvl = (uint8_t)lvl;
    }
}

/*
 * Inner loop. Per sample: envelope and noise counters (add + compare), three
 * tone phases with ASR gates, sum, store. On a noise/envelope rollover the three
 * amplitudes are reloaded from ampTab — no per-channel recomputation. Unused
 * generators run with step 0, so their compare never fires.
 */
static inline __attribute__((always_inline)) void ayRenderLoop(uint16_t *out, uint32_t n,
                                                               const bool doClamp)
{
    const uint8_t tc = ay.cur.toneConst;
    uint32_t t0 = (tc & 1u) ? AY_TONE_CONST : ay.tonePh[0];
    uint32_t t1 = (tc & 2u) ? AY_TONE_CONST : ay.tonePh[1];
    uint32_t t2 = (tc & 4u) ? AY_TONE_CONST : ay.tonePh[2];
    const uint32_t s0 = (tc & 1u) ? 0u : ay.cur.toneStep[0];
    const uint32_t s1 = (tc & 2u) ? 0u : ay.cur.toneStep[1];
    const uint32_t s2 = (tc & 4u) ? 0u : ay.cur.toneStep[2];

    const bool doEnv = ay.envUsed && !ay.envHold;
    uint32_t nPh = ay.noisePh;
    const uint32_t nSt = ay.noiseUsed ? ay.cur.noiseStep : 0u;
    uint32_t lfsr = ay.lfsr;
    uint32_t ePh = ay.envPh;
    uint32_t eSt = doEnv ? ay.cur.envStep : 0u;
    uint32_t eLvl = ay.envLvl;

    const uint16_t *tab = &ay.ampTab[0][0][0];
    const uint16_t *amp = tab + ((((lfsr & 1u) << 5) | eLvl) << 2);
    uint32_t a0 = amp[0];
    uint32_t a1 = amp[1];
    uint32_t a2 = amp[2];
    const uint32_t maxDuty = ay.maxDuty;

    for (uint32_t i = 0; i < n; i++) {
        bool reload = false;
        ePh += eSt;
        if (ePh >= AY_PHASE_ONE) {
            eLvl += ePh >> 16;
            ePh &= 0xffffu;
            if (eLvl >= 32u) {
                if (ay.envHoldShape) {
                    eLvl = 31u;
                    eSt = 0u;
                    ePh = 0u;
                    ay.envHold = 1;
                } else {
                    eLvl &= 31u;
                }
            }
            reload = true;
        }
        nPh += nSt;
        if (nPh >= AY_PHASE_ONE) {
            nPh &= 0xffffu;
            lfsr = (lfsr >> 1) | (((lfsr ^ (lfsr >> 3)) & 1u) << 16);
            reload = true;
        }
        if (reload) {
            amp = tab + ((((lfsr & 1u) << 5) | eLvl) << 2);
            a0 = amp[0];
            a1 = amp[1];
            a2 = amp[2];
        }
        t0 += s0;
        t1 += s1;
        t2 += s2;
        uint32_t mix = (a0 & (uint32_t)((int32_t)t0 >> 31))
                     + (a1 & (uint32_t)((int32_t)t1 >> 31))
                     + (a2 & (uint32_t)((int32_t)t2 >> 31));
        if (doClamp && mix > maxDuty) {
            mix = maxDuty;
        }
        out[i] = (uint16_t)mix;
    }

    /* Write back; gated-off tones still advance like the real counters. */
    ay.tonePh[0] = (tc & 1u) ? ay.tonePh[0] + ay.cur.toneStep[0] * n : t0;
    ay.tonePh[1] = (tc & 2u) ? ay.tonePh[1] + ay.cur.toneStep[1] * n : t1;
    ay.tonePh[2] = (tc & 4u) ? ay.tonePh[2] + ay.cur.toneStep[2] * n : t2;
    ay.noisePh = nPh;
    ay.lfsr = lfsr;
    if (doEnv) {
        ay.envPh = ePh;
        ay.envLvl = (uint8_t)eLvl;
    } else {
        /* An unheard envelope still advances (exactly, in one step). */
        ayEnvSkip(n);
    }
    /* An unheard noise generator is left parked — its sequence is random anyway. */
}

static FAST_CODE_NOINLINE void ayRenderSegment(uint16_t *out, uint32_t n)
{
    if (ay.needClamp) {
        ayRenderLoop(out, n, true);
    } else {
        ayRenderLoop(out, n, false);
    }
}

void aySynthRender(uint16_t *out, uint32_t count)
{
    while (count) {
        if (ay.frameLeft == 0) {
            ay.frameTicks++;
            if (ay.frameCb) {
                ay.frameCb();
            }
            ayNextFrame();
        }
        const uint32_t n = (count < ay.frameLeft) ? count : ay.frameLeft;
        ayRenderSegment(out, n);
        out += n;
        count -= n;
        ay.frameLeft -= n;
    }
}

#endif /* USE_CHIPTUNE */
