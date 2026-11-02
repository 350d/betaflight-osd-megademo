// Host render: the AT7456E as the demo sees it (display memory, attributes, NVM, row brightness,
// VM1 background, HOS / VOS, sync) and the firmware services the demo and the player call, on a
// virtual clock the harness drives.
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "platform.h"
#include "host_stubs.h"
#include "common/maths.h"
#include "drivers/max7456.h"
#include "drivers/ay_pwm.h"
#include "drivers/ay_synth.h"
#include "hostenv.h"

HOST_REG_TypeDef hostRegs[16];
int16_t debug[8];
hostGyro_t gyro;
attitudeEulerAngles_t attitude;

// --- Virtual clock: DWT cycles at 100 MHz; busy waits move it on a little each read ----------
uint64_t hostCycles;

void hostSetTimeUs(uint64_t us)
{
    const uint64_t c = us * (HOST_CPU_HZ / 1000000u);
    if (c > hostCycles) {
        hostCycles = c;
    }
}

uint32_t getCycleCounter(void)
{
    hostCycles += 20;
    return (uint32_t)hostCycles;
}
uint32_t clockMicrosToCycles(uint32_t us) { return us * (HOST_CPU_HZ / 1000000u); }
uint32_t clockCyclesToMicros(uint32_t c) { return c / (HOST_CPU_HZ / 1000000u); }
timeUs_t micros(void) { return (timeUs_t)(hostCycles / (HOST_CPU_HZ / 1000000u)); }
timeUs_t microsISR(void) { return micros(); }
timeMs_t millis(void) { return (timeMs_t)(hostCycles / (HOST_CPU_HZ / 1000u)); }
void delay(timeMs_t ms) { hostCycles += (uint64_t)ms * (HOST_CPU_HZ / 1000u); }
void delayMicroseconds(timeUs_t us) { hostCycles += (uint64_t)us * (HOST_CPU_HZ / 1000000u); }

// --- Features, motors, IO, timers, DMA: inert -------------------------------------------------
static uint32_t features = FEATURE_OSD;
bool featureIsEnabled(uint32_t mask) { return (features & mask) != 0; }
void featureEnableImmediate(uint32_t mask) { features |= mask; }
bool motorIsEnabled(void) { return false; }
void motorDisable(void) { }
IO_t IOGetByTag(ioTag_t tag) { (void)tag; return NULL; }
void IOInit(IO_t io, resourceOwner_e owner, uint8_t index) { (void)io; (void)owner; (void)index; }
resourceOwner_e IOGetOwner(IO_t io) { (void)io; return OWNER_FREE; }
void IOConfigGPIO(IO_t io, ioConfig_t cfg) { (void)io; (void)cfg; }
void IOConfigGPIOAF(IO_t io, ioConfig_t cfg, uint8_t af) { (void)io; (void)cfg; (void)af; }
void timerChannelEdgeHandlerInit(timerEdgeHandlerRec_t *self, timerEdgeHandlerCallback *fn) { self->fn = fn; }
void timerChannelConfigCallbacks(const timerHardware_t *timHw, timerEdgeHandlerRec_t *edge, void *overflow)
{
    (void)timHw; (void)edge; (void)overflow;
}
uint32_t timerClock(const timerHardware_t *timHw) { (void)timHw; return HOST_CPU_HZ; }
void dmaSetHandler(int identifier, dmaCallbackHandlerFuncPtr callback, uint32_t priority, uint32_t userParam)
{
    (void)identifier; (void)callback; (void)priority; (void)userParam;
}
void ledSet(int led, bool state) { (void)led; (void)state; }
void ledToggle(int led) { (void)led; }
void setTaskEnabled(taskId_e taskId, bool on) { (void)taskId; (void)on; }
void rescheduleTask(taskId_e taskId, int32_t periodUs) { (void)taskId; (void)periodUs; }
void getTaskInfo(taskId_e taskId, taskInfo_t *info) { (void)taskId; info->isEnabled = false; }
void EXTIDisable(IO_t io) { (void)io; }
void EXTIEnable(IO_t io) { (void)io; }
void cliPrintAsync(const char *line) { fprintf(stderr, "[demo] %s\n", line); }

// --- Display port / profiles -------------------------------------------------------------------
static displayPort_t hostDisplay = { .cols = 30, .rows = 16 };
displayPort_t *osdGetDisplayPort(osdDisplayPortDevice_e *device)
{
    if (device) {
        *device = OSD_DISPLAYPORT_DEVICE_MAX7456;
    }
    return &hostDisplay;
}
void osdInit(displayPort_t *dp, osdDisplayPortDevice_e dev) { (void)dp; (void)dev; }
static const vcdProfile_t hostVcd = { 1, 0, 0 };
const vcdProfile_t *vcdProfile(void) { return &hostVcd; }
bool max7456DisplayPortInit(const vcdProfile_t *vcd, displayPort_t **dp) { (void)vcd; *dp = &hostDisplay; return true; }
static const displayPortProfile_t hostDpProfile = { 0, 0, false, 0, 2 };
const displayPortProfile_t *displayPortProfileMax7456(void) { return &hostDpProfile; }
void displayGrab(displayPort_t *instance) { (void)instance; }
void displayReleaseAll(displayPort_t *instance) { (void)instance; }
void displayClearScreen(displayPort_t *instance, displayClearOption_e options) { (void)instance; (void)options; }
bool displayCheckReady(displayPort_t *instance, bool rescan) { (void)instance; (void)rescan; return true; }

// --- Audio path: the beeper's AY interface straight onto the synth ------------------------------
bool hostAudioOn;
bool ayPwmStart(void) { hostAudioOn = true; aySynthSetFrameCallback(NULL); return true; }
void ayPwmStop(void) { hostAudioOn = false; aySynthSetFrameCallback(NULL); }
bool ayPwmIsRunning(void) { return hostAudioOn; }
void ayPwmSetFrameCallback(void (*cb)(void)) { aySynthSetFrameCallback(cb); }
void ayPwmResetFrameClock(void) { aySynthResetFrameClock(); }
void ayPwmSetRegs(const uint8_t regs[16]) { aySynthApplyRegs(regs); }
void ayPwmApplyRegsIsr(const uint8_t regs[16]) { aySynthApplyRegs(regs); }
void ayPwmSetClockErrorPpm(int32_t ppm) { (void)ppm; }
bool ayPwmClaim(void) { return true; }
ioTag_t ayPwmPin(void) { return 0; }

// --- The chip ----------------------------------------------------------------------------------
hostChip_t chip;
static uint8_t layer[CHIP_CELLS], layerInv[CHIP_CELLS]; // the driver's screen buffer
static uint8_t rbWanted[16];
static bool rbHold;
static int8_t hudX, hudY;
static uint8_t osdmReg;

static uint8_t vm1Of(bool gray, uint8_t level)
{
    return (uint8_t)(0x04 | 0x03 | ((gray ? (level & 7u) : 4u) << 4) | (gray ? 0x80u : 0u));
}

void hostChipInit(void)
{
    memset(&chip, 0, sizeof(chip));
    memset(layer, 0x20, sizeof(layer));
    memcpy(chip.code, layer, sizeof(layer));
    chip.vm1 = vm1Of(false, 4);
    chip.hos = 32;
    chip.vos = 16;
    memset(chip.rb, (0u << 2) | (3u - 2u), sizeof(chip.rb)); // Betaflight default: black 0, white 2
    memcpy(rbWanted, chip.rb, sizeof(rbWanted));
}

bool max7456IsDeviceDetected(void) { return true; }
bool max7456IsAt7456(void) { return true; }
bool max7456DmaInProgress(void) { return false; }
uint8_t max7456DmmBase(void) { return 0; }
void max7456Invalidate(void) { }
void max7456EndFontWrite(void) { }
bool max7456MidGlyphSpiIsHot(void) { return false; }
void max7456MidGlyphSpiBegin(void) { }
void max7456MidGlyphSpiBoost(bool enable) { (void)enable; }
void max7456MidGlyphSpiEnd(void) { }
uint32_t max7456SessionWordTicks(void) { return 60; } // 16 bits at 27 MHz
static uint32_t spiDr;
volatile void *max7456SpiDataRegister(void) { return &spiDr; }
bool max7456Zone16Active(void) { return false; }
void max7456Zone16Begin(void) { }
void max7456Zone16End(void) { }
void max7456Osdm(uint8_t value) { osdmReg = value; }
void max7456SetSyncMode(uint8_t mode) { chip.syncMode = mode; }

bool max7456WaitVsyncFallingEdge(uint32_t *edgeTicks, timeUs_t timeoutUs)
{
    (void)timeoutUs;
    if (edgeTicks) {
        *edgeTicks = getCycleCounter();
    }
    return true;
}

bool max7456WaitHsyncFallingEdge(uint32_t *edgeTicks, timeUs_t timeoutUs)
{
    return max7456WaitVsyncFallingEdge(edgeTicks, timeoutUs);
}

// Font NVM: 54 bytes a glyph, 4 pixels a byte (MSB first): 00 black, 10 white, x1 transparent
bool max7456WriteNvm(uint8_t addr, const uint8_t *data) { memcpy(chip.nvm[addr], data, 54); return true; }
bool max7456NvmWrite9(uint16_t addr, const uint8_t data[54]) { memcpy(chip.nvm[addr & 511u], data, 54); return true; }
bool max7456NvmRead9(uint16_t addr, uint8_t out[54]) { memcpy(out, chip.nvm[addr & 511u], 54); return true; }
bool max7456NvmMatches(uint8_t addr, const uint8_t *data) { return memcmp(chip.nvm[addr], data, 54) == 0; }

// Screen buffer → display memory
static void flushLayer(void)
{
    memcpy(chip.code, layer, sizeof(layer));
    memcpy(chip.inv, layerInv, sizeof(layerInv));
}

static void applyHudMotion(void)
{
    chip.hos = (uint8_t)constrain(32 + hudX, 0, 63);
    chip.vos = (uint8_t)constrain(16 + hudY, 0, 31);
}

void max7456WriteChar(uint8_t x, uint8_t y, uint8_t c)
{
    if (x < 30 && y < 16) {
        layer[y * 30 + x] = c;
        layerInv[y * 30 + x] = 0;
    }
}

void max7456WriteCharEx(uint8_t x, uint8_t y, uint8_t c, bool invert)
{
    if (x < 30 && y < 16) {
        layer[y * 30 + x] = c;
        layerInv[y * 30 + x] = invert ? 1u : 0u;
    }
}

void max7456CopyLayer(uint8_t *codes, uint8_t *inv, uint16_t n)
{
    n = MIN(n, (uint16_t)CHIP_CELLS);
    memcpy(codes, layer, n);
    memcpy(inv, layerInv, n);
}

void max7456FillScreen(uint8_t c)
{
    memset(layer, c, sizeof(layer));
    memset(layerInv, 0, sizeof(layerInv));
}

uint16_t max7456FlushDirtyBurst(void) { flushLayer(); applyHudMotion(); return 1; }
void max7456RefreshAll(void) { flushLayer(); applyHudMotion(); }
bool max7456DrawScreen(void) { flushLayer(); applyHudMotion(); return false; } // all sent in one call

bool max7456WriteDisplaySramRowFillEx(uint16_t addr, uint8_t glyph, uint8_t count, bool commitShadow)
{
    (void)commitShadow;
    for (uint16_t i = 0; i < count && addr + i < CHIP_CELLS; i++) {
        chip.code[addr + i] = glyph;
        chip.inv[addr + i] = 0;
        layer[addr + i] = glyph;
        layerInv[addr + i] = 0;
    }
    return true;
}

// Row writes the engine encodes then sends: applied at the send
static struct { uint16_t rowAddr; uint8_t glyphs[30]; uint32_t inv; max7456SramSeg_t seg[16]; uint8_t nSeg; bool pending; } enc;

uint16_t max7456EncodeDisplaySramRowAttr(uint16_t rowAddr, const uint8_t *glyphs, uint32_t invMask,
                                         const max7456SramSeg_t *seg, uint8_t nSeg,
                                         uint16_t *segFirstByte, uint16_t *segLastByte)
{
    enc.rowAddr = rowAddr;
    memcpy(enc.glyphs, glyphs, 30);
    enc.inv = invMask;
    enc.nSeg = (uint8_t)MIN(nSeg, 16u);
    memcpy(enc.seg, seg, enc.nSeg * sizeof(max7456SramSeg_t));
    enc.pending = true;
    for (uint8_t i = 0; i < nSeg; i++) {
        if (segFirstByte) {
            segFirstByte[i] = (uint16_t)(i * 4u);
        }
        segLastByte[i] = (uint16_t)(i * 4u + 3u);
    }
    return (uint16_t)(nSeg * 4u + 1u);
}

bool max7456SendEncodedDisplaySram(uint16_t len)
{
    (void)len;
    if (!enc.pending) {
        return false;
    }
    for (uint8_t i = 0; i < enc.nSeg; i++) {
        for (uint8_t k = 0; k < enc.seg[i].len; k++) {
            const uint8_t col = (uint8_t)(enc.seg[i].col + k);
            if (col < 30 && enc.rowAddr + col < CHIP_CELLS) {
                chip.code[enc.rowAddr + col] = enc.glyphs[col];
                chip.inv[enc.rowAddr + col] = (uint8_t)((enc.inv >> col) & 1u);
            }
        }
    }
    enc.pending = false;
    return true;
}

// Row brightness: RB = black << 2 | (3 − white)
void max7456Brightness(uint8_t black, uint8_t white)
{
    const uint8_t reg = (uint8_t)((black << 2) | (3 - white));
    memset(rbWanted, reg, sizeof(rbWanted));
    if (!rbHold) {
        memset(chip.rb, reg, sizeof(chip.rb));
    }
}

void max7456BrightnessRow(uint8_t row, uint8_t black, uint8_t white)
{
    if (row > 15) {
        return;
    }
    const uint8_t reg = (uint8_t)(((black & 3) << 2) | (3 - (white & 3)));
    rbWanted[row] = reg;
    if (!rbHold) {
        chip.rb[row] = reg;
    }
}

void max7456WriteRbRaw(uint8_t row, uint8_t reg)
{
    if (row > 15) {
        return;
    }
    rbWanted[row] = reg & 0x0Fu;
    if (!rbHold) {
        chip.rb[row] = reg & 0x0Fu;
    }
}

void max7456RbHold(bool hold, uint8_t flashReg)
{
    rbHold = hold;
    for (uint8_t i = 0; i < 16; i++) {
        chip.rb[i] = hold ? (uint8_t)(flashReg & 0x0Fu) : rbWanted[i];
    }
}

void max7456RbResetWanted(uint8_t reg) { memset(rbWanted, reg & 0x0Fu, sizeof(rbWanted)); }
bool max7456RbHeld(void) { return rbHold; }
void max7456ReadRowBrightness(uint8_t out[16]) { memcpy(out, chip.rb, 16); }
void max7456ReadRegs(uint8_t reg, uint8_t *out, uint8_t count) { (void)reg; memset(out, 0, count); }

// Background (VM1)
void max7456SetBackgroundType(displayPortBackground_e type)
{
    switch (type) {
    case DISPLAY_BACKGROUND_BLACK: chip.vm1 = vm1Of(true, 0); break;
    case DISPLAY_BACKGROUND_LTGRAY: chip.vm1 = vm1Of(true, 7); break;
    case DISPLAY_BACKGROUND_GRAY: chip.vm1 = vm1Of(true, 4); break;
    default: chip.vm1 = vm1Of(false, 4); break;
    }
}
void max7456WriteBackgroundNow(bool gray, uint8_t level) { chip.vm1 = vm1Of(gray, level); }
int16_t max7456BackgroundFix(bool gray, uint8_t level)
{
    const uint8_t want = vm1Of(gray, level);
    if (chip.vm1 == want) {
        return -1;
    }
    const uint8_t got = chip.vm1;
    chip.vm1 = want;
    return got;
}
void max7456WriteVm1Raw(uint8_t reg) { chip.vm1 = reg; }

// HOS / VOS
void max7456SetHudMotionOffset(int8_t x, int8_t y) { hudX = x; hudY = y; }
void max7456ResetHudMotionOffset(void) { hudX = hudY = 0; }
void max7456ApplyHudMotionNow(void) { applyHudMotion(); }
uint8_t max7456GetHosApplied(void) { return (uint8_t)constrain(32 + hudX, 0, 63); }
void max7456GetHudMotionXLimits(int8_t *minX, int8_t *maxX)
{
    if (minX) {
        *minX = -32;
    }
    if (maxX) {
        *maxX = 31;
    }
}
void max7456GetHudMotionYLimits(int8_t *minY, int8_t *maxY)
{
    if (minY) {
        *minY = -16;
    }
    if (maxY) {
        *maxY = 15;
    }
}
void max7456WriteHosNow(uint8_t hos) { chip.hos = (uint8_t)MIN(hos, 63u); }
void max7456WriteHosSigned(int8_t offsetPx) { chip.hos = (uint8_t)constrain(32 + offsetPx, 0, 63); }
int8_t max7456WriteVosOffsetNow(int8_t offsetPx)
{
    chip.vos = (uint8_t)constrain(16 + offsetPx, 0, 31);
    return (int8_t)(chip.vos - 16);
}
