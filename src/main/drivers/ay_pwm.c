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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#if defined(USE_CHIPTUNE) && (defined(STM32F4) || defined(STM32F7))

#include "build/atomic.h"

#include "common/maths.h"
#include "common/utils.h"

#include "drivers/ay_pwm.h"
#include "drivers/ay_synth.h"
#include "drivers/dma.h"
#include "drivers/dma_reqmap.h"
#include "drivers/io.h"
#include "drivers/motor.h"
#include "drivers/nvic.h"
#include "drivers/pwm_output.h"
#include "drivers/serial.h"
#include "drivers/serial_resource.h"
#include "drivers/timer.h"

#include "pg/bus_i2c.h"
#include "pg/motor.h"

#include "platform/dma.h"
#include "platform/timer.h"

// The pin: the LED strip pad unless the build names another (EXTRA_FLAGS=-DAY_PWM_PIN=PB6).
#if !defined(AY_PWM_PIN) && defined(LED_STRIP_PIN)
#define AY_PWM_PIN LED_STRIP_PIN
#endif

// Samples at 32 kHz. On an advanced timer (TIM1 / TIM8) the PWM runs 4 periods a sample (RCR):
// a 128 kHz carrier that one RC stage on an audio input knocks down ~24 dB.
#define AY_SAMPLE_HZ    32000u
#define AY_CARRIER_MUL  4u
#define AY_GAIN         16u
// The CPU renders one half of the ring per DMA half / full interrupt (8 ms of samples), at a
// low preempt group (the deadline is a whole half), but above group 3: the OSD demo builds its
// frames in a group-3 interrupt for ~30 ms at a time.
#define AY_DMA_HALF     256u
#define AY_RING_SAMPLES (2u * AY_DMA_HALF)
#define AY_DMA_IRQ_PRIO NVIC_BUILD_PRIORITY(2, 1)
#define AY_LOCK_PRIO    NVIC_PRIO_TIMER

static DMA_DATA_ZERO_INIT uint16_t ayRing[AY_RING_SAMPLES] __attribute__((aligned(32)));

static const timerHardware_t *ayTimer;
static timerChannel_t ayChannel;
static dmaResource_t *ayDmaRef;
static uint32_t ayDmaChannel;
static dmaChannelDescriptor_t *ayDmaDesc;
static bool ayDmaUpdate;  // request on the timer update (UDE, RCR carrier) instead of CCx
static uint8_t ayCarrierMul;
static uint16_t ayArr;
static uint32_t ayRateActual;
static int32_t ayClockPpm;
static volatile bool ayRunning;

static aySynthFrame_t ayPending;  // task → render handoff
static volatile bool ayPendingDirty;

static void ayRenderBlock(uint16_t *dst)
{
    if (ayPendingDirty) {
        aySynthCommit(&ayPending);
        ayPendingDirty = false;
    }
    aySynthRender(dst, AY_DMA_HALF);
}

static FAST_IRQ_HANDLER void ayDmaIrqHandler(dmaChannelDescriptor_t *descriptor)
{
    if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_HTIF)) {
        DMA_CLEAR_FLAG(descriptor, DMA_IT_HTIF);
        ayRenderBlock(&ayRing[0]);
    }
    if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_TCIF)) {
        DMA_CLEAR_FLAG(descriptor, DMA_IT_TCIF);
        ayRenderBlock(&ayRing[AY_DMA_HALF]);
    }
    DMA_CLEAR_FLAG(descriptor, DMA_IT_TEIF | DMA_IT_DMEIF | DMA_IT_FEIF);
}

static bool ayTimerHasRcr(const TIM_TypeDef *tim)
{
#ifdef TIM8
    if (tim == TIM8) {
        return true;
    }
#endif
    return tim == TIM1;
}

// A DMA stream for the timer channel: the update request on an advanced timer (for the 128 kHz
// carrier) when asked for, else the channel's compare request, on any free option.
static bool ayClaimDma(const timerHardware_t *timer, bool update)
{
    if (update) {
#ifdef USE_TIMER_DMA
        if (ayTimerHasRcr((const TIM_TypeDef *)timer->tim) && timer->dmaTimUPRef
            && dmaAllocate(dmaGetIdentifier(timer->dmaTimUPRef), OWNER_BEEPER, 0)) {
            ayDmaRef = timer->dmaTimUPRef;
            ayDmaChannel = timer->dmaTimUPChannel;
            ayDmaUpdate = true;
            return true;
        }
#endif
        return false;
    }
    for (int opt = 0; opt < MAX_TIMER_DMA_OPTIONS; opt++) {
        const dmaChannelSpec_t *spec = dmaGetChannelSpecByTimerValue(timer->tim, timer->channel, opt);
        if (spec && dmaAllocate(dmaGetIdentifier(spec->ref), OWNER_BEEPER, 0)) {
            ayDmaRef = spec->ref;
            ayDmaChannel = spec->channel;
            ayDmaUpdate = false;
            return true;
        }
    }
    return false;
}

// The pad's timer channels, the OSD demo's raster timers (TIM2 / TIM3 / TIM5) left out. A pad
// owned by `owner` is taken over as if free.
static const timerHardware_t *ayClaimTimerFrom(ioTag_t tag, bool update, resourceOwner_e owner)
{
    const IO_t io = IOGetByTag(tag);
    if (!io || (IOGetOwner(io) != OWNER_FREE && IOGetOwner(io) != owner)) {
        return NULL;
    }
    for (unsigned i = 1; ; i++) {
        const timerHardware_t *timer = timerGetByTagAndIndex(tag, i);
        if (!timer) {
            return NULL;
        }
        const TIM_TypeDef *tim = (const TIM_TypeDef *)timer->tim;
        if (tim != TIM2 && tim != TIM3 && tim != TIM5 && ayClaimDma(timer, update)) {
            return timer;
        }
    }
}

static const timerHardware_t *ayClaimTimer(ioTag_t tag, bool update)
{
    return ayClaimTimerFrom(tag, update, OWNER_FREE);
}

// The sound's pad: the LED strip pad (or AY_PWM_PIN), else an I2C pad (the demo turns the baro
// off; some AIO boards bring out no UART pad with a usable timer), else the first free UART pad
// with a timer and DMA for it, a TIM1 / TIM8 one with the carrier first.
bool ayPwmClaim(void)
{
    if (ayTimer) {
        return true;
    }
    const timerHardware_t *timer = NULL;
#ifdef AY_PWM_PIN
    timer = ayClaimTimer(IO_TAG(AY_PWM_PIN), true);
    if (!timer) {
        timer = ayClaimTimer(IO_TAG(AY_PWM_PIN), false);
    }
#endif
#ifdef USE_I2C
    for (unsigned i = 0; i < I2CDEV_COUNT && !timer; i++) {
        timer = ayClaimTimerFrom(i2cConfig(i)->ioTagScl, false, OWNER_I2C_SCL);
        if (!timer) {
            timer = ayClaimTimerFrom(i2cConfig(i)->ioTagSda, false, OWNER_I2C_SDA);
        }
    }
#endif
    for (int pass = 0; pass < 2 && !timer; pass++) {
        for (unsigned i = 0; i < RESOURCE_UART_COUNT && !timer; i++) {
            timer = ayClaimTimer(serialPinConfig()->ioTagTx[RESOURCE_UART_OFFSET + i], pass == 0);
            if (!timer) {
                timer = ayClaimTimer(serialPinConfig()->ioTagRx[RESOURCE_UART_OFFSET + i], pass == 0);
            }
        }
    }
    if (!timer) {
        return false;
    }
    // A motor on the same timer: the sound takes its time base (the demo needs a reboot anyway).
    for (unsigned i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
        const timerHardware_t *motor = timerGetConfiguredByTag(motorConfig()->dev.ioTags[i]);
        if (motor && motor->tim == timer->tim && motorIsEnabled()) {
            motorDisable();
        }
    }
    const dmaIdentifier_e id = dmaGetIdentifier(ayDmaRef);
    dmaEnable(id);
    dmaSetHandler(id, ayDmaIrqHandler, AY_DMA_IRQ_PRIO, 0);
    const IO_t io = IOGetByTag(timer->tag);
    IOInit(io, OWNER_BEEPER, 0);
    IOConfigGPIOAF(io, IOCFG_AF_PP, timer->alternateFunction);
    ayTimer = timer;
    ayDmaDesc = dmaGetDescriptorByIdentifier(id);
    return true;
}

ioTag_t ayPwmPin(void)
{
    return ayTimer ? ayTimer->tag : IO_TAG_NONE;
}

static uint32_t ayDmaSource(void)
{
    return ayDmaUpdate ? TIM_DIER_UDE : timerDmaSource(ayTimer->channel);
}

// The MCU clock's error against real time (+ = fast, ppm), measured by the OSD demo against
// its video: the sample rate is that much off what the clock tree says, so pitch and tempo
// follow the true rate.
static void ayConfigureSynth(void)
{
    const uint32_t rate = (uint32_t)((uint64_t)ayRateActual * (uint64_t)(1000000 + ayClockPpm) / 1000000u);
    aySynthConfigure(rate, (uint16_t)(ayArr - 1u), AY_GAIN, true, 7, AY_DAC_AY);
}

bool ayPwmStart(void)
{
    ayPwmStop();
    if (!ayPwmClaim()) {
        return false;
    }
    TIM_TypeDef *tim = (TIM_TypeDef *)ayTimer->tim;
    const uint32_t tclk = timerClock(ayTimer);
    ayCarrierMul = ayDmaUpdate ? AY_CARRIER_MUL : 1u;
    const uint32_t carrierHz = AY_SAMPLE_HZ * ayCarrierMul;
    ayArr = (uint16_t)constrain((tclk + carrierHz / 2u) / carrierHz, 64u, 65535u);
    ayRateActual = tclk / ((uint32_t)ayArr * ayCarrierMul); // pitch and tempo come from it
    ayPendingDirty = false;
    aySynthSetFrameCallback(NULL);
    ayConfigureSynth();
    aySynthReset();

    pwmOutputConfig(&ayChannel, ayTimer, tclk, ayArr, 0, 0);
    if (ayTimerHasRcr(tim)) {
        tim->RCR = (uint32_t)(ayCarrierMul - 1u);
        tim->EGR = TIM_EGR_UG; // load RCR / ARR now
    }
    aySynthRender(ayRing, AY_RING_SAMPLES); // the whole ring valid before the first request

    DMA_Stream_TypeDef *stream = (DMA_Stream_TypeDef *)ayDmaRef;
    stream->PAR = (uint32_t)ayChannel.ccr;
    stream->M0AR = (uint32_t)ayRing;
    stream->NDTR = AY_RING_SAMPLES;
    stream->FCR = 0; // direct mode
    stream->CR = ayDmaChannel | DMA_SxCR_PL_1 | DMA_SxCR_MSIZE_0 | DMA_SxCR_PSIZE_0 | DMA_SxCR_MINC
               | DMA_SxCR_CIRC | DMA_SxCR_DIR_0 | DMA_SxCR_TCIE | DMA_SxCR_HTIE | DMA_SxCR_TEIE;
    stream->CR |= DMA_SxCR_EN;
    tim->DIER |= ayDmaSource();
    ayRunning = true;
    return true;
}

void ayPwmStop(void)
{
    if (!ayRunning) {
        return;
    }
    ayRunning = false;
    ATOMIC_BLOCK(AY_LOCK_PRIO) {
        aySynthSetFrameCallback(NULL);
    }
    TIM_TypeDef *tim = (TIM_TypeDef *)ayTimer->tim;
    tim->DIER &= ~ayDmaSource();
    DMA_Stream_TypeDef *stream = (DMA_Stream_TypeDef *)ayDmaRef;
    stream->CR &= ~DMA_SxCR_EN;
    while (stream->CR & DMA_SxCR_EN) {
    }
    DMA_CLEAR_FLAG(ayDmaDesc, DMA_IT_TCIF | DMA_IT_HTIF | DMA_IT_TEIF | DMA_IT_DMEIF | DMA_IT_FEIF);
    if (ayTimerHasRcr(tim)) {
        tim->RCR = 0;
        tim->EGR = TIM_EGR_UG;
    }
    // Silent = the pin held low, not a PWM switching the output with the music off
    *ayChannel.ccr = 0;
}

bool ayPwmIsRunning(void)
{
    return ayRunning;
}

void ayPwmSetRegs(const uint8_t regs[16])
{
    if (!ayRunning) {
        return;
    }
    aySynthFrame_t f;
    aySynthPrepare(regs, &f); // the divisions here, in the task
    ATOMIC_BLOCK(AY_LOCK_PRIO) {
        ayPending = f;
        ayPendingDirty = true;
    }
}

void ayPwmApplyRegsIsr(const uint8_t regs[16])
{
    aySynthApplyRegs(regs);
}

void ayPwmSetFrameCallback(void (*cb)(void))
{
    ATOMIC_BLOCK(AY_LOCK_PRIO) {
        aySynthSetFrameCallback(cb);
    }
}

void ayPwmResetFrameClock(void)
{
    ATOMIC_BLOCK(AY_LOCK_PRIO) {
        aySynthResetFrameClock();
    }
}

void ayPwmSetClockErrorPpm(int32_t ppm)
{
    ayClockPpm = constrain(ppm, -50000, 50000);
    if (ayRunning) {
        ATOMIC_BLOCK(AY_LOCK_PRIO) {
            ayConfigureSynth(); // the new rate from the next block
        }
    }
}

#endif
