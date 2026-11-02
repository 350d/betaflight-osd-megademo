/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <strings.h>
#include <math.h>

#include "platform.h"

#ifdef USE_MAX7456

#include "build/debug.h"
#include "common/maths.h"

#include "pg/max7456.h"
#include "pg/vcd.h"

#include "drivers/bus_spi.h"
#include "drivers/dma.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/light_led.h"
#include "drivers/max7456.h"
#include "drivers/nvic.h"
#include "drivers/osd.h"
#include "drivers/osd_symbols.h"
#include "drivers/system.h"
#include "drivers/time.h"

static void max7456CommitShadowCell(uint16_t addr, uint8_t glyph, bool invert);

// 10 MHz max SPI frequency
#define MAX7456_MAX_SPI_CLK_HZ 10000000
#define MAX7456_INIT_MAX_SPI_CLK_HZ 5000000

// OSD demo zone session (HSYNC wired): the slot engine's DMA owns SPI and holds CS low for the
// whole active field. Any other driver access then would cut into its word stream and raise CS
// (the chip loses its state: random breaks even on a still picture) — dropped and counted.
#if defined(STM32F4) || defined(STM32F7)
static bool zone16;
static inline bool max7456BusOwnedBySession(void)
{
    return zone16;
}
#define MAX7456_SESSION_GUARD(ret) do { if (max7456BusOwnedBySession()) { return ret; } } while (0)
static inline void max7456SpiWriteReg(const extDevice_t *d, uint8_t r, uint8_t v)
{
    MAX7456_SESSION_GUARD();
    spiWriteReg(d, r, v);
}
static inline uint8_t max7456SpiReadRegMsk(const extDevice_t *d, uint8_t r)
{
    MAX7456_SESSION_GUARD(0);
    return spiReadRegMsk(d, r);
}
static inline void max7456SpiSequence(const extDevice_t *d, busSegment_t *seg)
{
    MAX7456_SESSION_GUARD();
    spiSequence(d, seg);
}
static inline void max7456SpiReadWriteBuf(const extDevice_t *d, uint8_t *tx, uint8_t *rx, int len)
{
    MAX7456_SESSION_GUARD();
    spiReadWriteBuf(d, tx, rx, len);
}
static inline void max7456SpiWrite(const extDevice_t *d, uint8_t b)
{
    MAX7456_SESSION_GUARD();
    spiWrite(d, b);
}
#define spiWriteReg max7456SpiWriteReg
#define spiReadRegMsk max7456SpiReadRegMsk
#define spiSequence max7456SpiSequence
#define spiReadWriteBuf max7456SpiReadWriteBuf
#define spiWrite max7456SpiWrite

#endif

// DEBUG_MAX7456_SIGNAL
#define DEBUG_MAX7456_SIGNAL_MODEREG       0
#define DEBUG_MAX7456_SIGNAL_SENSE         1
#define DEBUG_MAX7456_SIGNAL_REINIT        2
#define DEBUG_MAX7456_SIGNAL_ROWS          3

// DEBUG_MAX7456_SPICLOCK
#define DEBUG_MAX7456_SPICLOCK_OVERCLOCK   0
#define DEBUG_MAX7456_SPICLOCK_DEVTYPE     1
#define DEBUG_MAX7456_SPICLOCK_DIVISOR     2
#define DEBUG_MAX7456_SPICLOCK_X100        3

// VM0 bits
#define VIDEO_BUFFER_DISABLE        0x01
#define MAX7456_RESET               0x02
#define VERTICAL_SYNC_NEXT_VSYNC    0x04
#define OSD_ENABLE                  0x08

#define SYNC_MODE_AUTO              0x00
#define SYNC_MODE_INTERNAL          0x30
#define SYNC_MODE_EXTERNAL          0x20

#define VIDEO_MODE_PAL              0x40
#define VIDEO_MODE_NTSC             0x00
#define VIDEO_MODE_MASK             0x40
#define VIDEO_MODE_IS_PAL(val)      (((val) & VIDEO_MODE_MASK) == VIDEO_MODE_PAL)
#define VIDEO_MODE_IS_NTSC(val)     (((val) & VIDEO_MODE_MASK) == VIDEO_MODE_NTSC)

#define VIDEO_SIGNAL_DEBOUNCE_MS    100 // Time to wait for input to stabilize

// VM1 bits

// duty cycle is on_off
#define BLINK_DUTY_CYCLE_50_50 0x00
#define BLINK_DUTY_CYCLE_33_66 0x01
#define BLINK_DUTY_CYCLE_25_75 0x02
#define BLINK_DUTY_CYCLE_75_25 0x03

// blinking time
#define BLINK_TIME_0 0x00
#define BLINK_TIME_1 0x04
#define BLINK_TIME_2 0x08
#define BLINK_TIME_3 0x0C

// background mode brightness (percent)
#define BACKGROUND_BRIGHTNESS_0 0x00
#define BACKGROUND_BRIGHTNESS_7 0x01
#define BACKGROUND_BRIGHTNESS_14 0x02
#define BACKGROUND_BRIGHTNESS_21 0x03
#define BACKGROUND_BRIGHTNESS_28 0x04
#define BACKGROUND_BRIGHTNESS_35 0x05
#define BACKGROUND_BRIGHTNESS_42 0x06
#define BACKGROUND_BRIGHTNESS_49 0x07

#define BACKGROUND_MODE_GRAY 0x80

// STAT register bits

#define STAT_PAL      0x01
#define STAT_NTSC     0x02
#define STAT_LOS      0x04
#define STAT_HSYNC    0x08  // STAT[3] — HSYNC output level (~HSYNC)
#define STAT_VSYNC    0x10  // STAT[4] — VSYNC output level (~VSYNC, falls at VSYNC start)
#define STAT_NVR_BUSY 0x20

#define STAT_IS_PAL(val)  ((val) & STAT_PAL)
#define STAT_IS_NTSC(val) ((val) & STAT_NTSC)
#define STAT_IS_LOS(val)  ((val) & STAT_LOS)
#define STAT_IS_VSYNC_HIGH(val)  (((val) & STAT_VSYNC) != 0)
#define STAT_IS_HSYNC_HIGH(val)  (((val) & STAT_HSYNC) != 0)

#define VIN_IS_PAL(val)  (!STAT_IS_LOS(val) && STAT_IS_PAL(val))
#define VIN_IS_NTSC(val)  (!STAT_IS_LOS(val) && STAT_IS_NTSC(val))

// DMM register bits
#define DMM_AUTO_INC 0x01

// Kluege warning!
// There are occasions that NTSC is not detected even with !LOS (AB7456 specific?)
// When this happens, lower 3 bits of STAT register is read as zero.
// To cope with this case, this macro defines !LOS && !PAL as NTSC.
// Should be compatible with MAX7456 and non-problematic case.

#define VIN_IS_NTSC_alt(val)  (!STAT_IS_LOS(val) && !STAT_IS_PAL(val))

#define MAX7456_SIGNAL_CHECK_INTERVAL_MS 1000 // msec
#define MAX7456_STALL_CHECK_INTERVAL_MS  1000 // msec

// DMM special bits
#define CLEAR_DISPLAY 0x04
#define CLEAR_DISPLAY_VERT 0x06
#define INVERT_PIXEL_COLOR 0x08
#define DMM_8BIT_MODE 0x40          // DMM[6]: 1 = 8-bit display-memory ops
#define DMAH_ATTR_SELECT 0x02       // DMAH[1]: 1 = attribute byte (8-bit mode only)
#define CHAR_ATTR_INV 0x01          // per-cell attribute bit0

// Special address for terminating incremental write
#define END_STRING 0xff

#define MAX7456ADD_READ         0x80
#define MAX7456ADD_VM0          0x00  //0b0011100// 00 // 00             ,0011100
#define MAX7456ADD_VM1          0x01
#define MAX7456ADD_HOS          0x02
#define MAX7456ADD_VOS          0x03
#define MAX7456ADD_DMM          0x04
#define MAX7456ADD_DMAH         0x05
#define MAX7456ADD_DMAL         0x06
#define MAX7456ADD_DMDI         0x07
#define MAX7456ADD_CMM          0x08
#define MAX7456ADD_CMAH         0x09
#define MAX7456ADD_CMAL         0x0a
#define MAX7456ADD_CMDI         0x0b
#define MAX7456ADD_OSDM         0x0c
#define MAX7456ADD_RB0          0x10
#define MAX7456ADD_RB1          0x11
#define MAX7456ADD_RB2          0x12
#define MAX7456ADD_RB3          0x13
#define MAX7456ADD_RB4          0x14
#define MAX7456ADD_RB5          0x15
#define MAX7456ADD_RB6          0x16
#define MAX7456ADD_RB7          0x17
#define MAX7456ADD_RB8          0x18
#define MAX7456ADD_RB9          0x19
#define MAX7456ADD_RB10         0x1a
#define MAX7456ADD_RB11         0x1b
#define MAX7456ADD_RB12         0x1c
#define MAX7456ADD_RB13         0x1d
#define MAX7456ADD_RB14         0x1e
#define MAX7456ADD_RB15         0x1f
#define MAX7456ADD_OSDBL        0x6c
#define MAX7456ADD_STAT         0xA0

#define NVM_RAM_SIZE            54
#define WRITE_NVR               0xA0
#define READ_NVR                0x50
#define MAX7456ADD_CMDO_MSK     0x40 // CMDO is read at 0xC0 (spiReadRegMsk adds 0x80)

// Device type. An enum rather than two #defines so that the debug annotation
// on DEBUG_MAX7456_SPICLOCK_DEVTYPE can name it, and a consumer reads the
// device name instead of the number.
typedef enum {
    MAX7456_DEVICE_TYPE_MAX = 0,
    MAX7456_DEVICE_TYPE_AT
} max7456DeviceType_e;

#define CHARS_PER_LINE      30 // XXX Should be related to VIDEO_BUFFER_CHARS_*?

#define MAX7456_SUPPORTED_LAYER_COUNT (DISPLAYPORT_LAYER_BACKGROUND + 1)

typedef struct max7456Layer_s {
    uint8_t buffer[VIDEO_BUFFER_CHARS_PAL];
} max7456Layer_t;

static max7456Layer_t displayLayers[MAX7456_SUPPORTED_LAYER_COUNT];
static displayPortLayer_e activeLayer = DISPLAYPORT_LAYER_FOREGROUND;

extDevice_t max7456Device;
extDevice_t *dev = &max7456Device;

static bool max7456DeviceDetected = false;
static uint16_t max7456SpiClockDiv;
static volatile bool max7456ActiveDma = false;

uint16_t maxScreenSize = VIDEO_BUFFER_CHARS_PAL;

// We write everything to the active layer and then compare
// it with shadowBuffer to update only changed chars.
// This solution is faster then redrawing entire screen.

static uint8_t shadowBuffer[VIDEO_BUFFER_CHARS_PAL];
// Per-cell INV attribute (MAX7456 DMM[3] / char attr). Parallel to char buffers.
static uint8_t invertLayer[VIDEO_BUFFER_CHARS_PAL];
static uint8_t shadowInvert[VIDEO_BUFFER_CHARS_PAL];

//Max bytes to update in one call to max7456DrawScreen()

#define MAX_BYTES2SEND          250
#define MAX_BYTES2SEND_POLLED   12
#define MAX_ENCODE_US           20
#define MAX_ENCODE_US_POLLED    10

static DMA_DATA uint8_t spiBuf[MAX_BYTES2SEND];

// Screen-draw cursor; exposed via max7456IsFrameIdle().
static uint16_t max7456DrawPos = 0;

static uint8_t  videoSignalCfg;
static uint8_t  videoSignalReg  = OSD_ENABLE; // OSD_ENABLE required to trigger first ReInit
static uint8_t  syncModeBits    = SYNC_MODE_AUTO; // VM0[5:4]; the OSD demo may run on internal sync
static uint8_t  displayMemoryModeReg = 0;

static uint8_t  hosRegValue; // HOS (Horizontal offset register) value
static bool midGlyphSpiHot; // a mid-glyph engine owns chip + bus (defined with its helpers below)
static uint8_t  vosRegValue; // VOS (Vertical offset register) value

// Transient HUD inertia offset in screen pixels (+x right, +y down). Not part of vcd base.
static int8_t hudMotionOffsetX;
static int8_t hudMotionOffsetY;

static bool fontIsLoading       = false;

static uint8_t max7456DeviceType;

static displayPortBackground_e deviceBackgroundType = DISPLAY_BACKGROUND_TRANSPARENT;

// previous states initialized outside the valid range to force update on first call
#define INVALID_PREVIOUS_REGISTER_STATE 255
static uint8_t previousBlackWhiteRegister = INVALID_PREVIOUS_REGISTER_STATE;
static uint8_t previousInvertRegister = INVALID_PREVIOUS_REGISTER_STATE;
static uint8_t previousHosRegister = INVALID_PREVIOUS_REGISTER_STATE;
static uint8_t previousVosRegister = INVALID_PREVIOUS_REGISTER_STATE;
static uint8_t previousOsdmRegister = INVALID_PREVIOUS_REGISTER_STATE;

static uint8_t *getLayerBuffer(displayPortLayer_e layer)
{
    return displayLayers[layer].buffer;
}

static uint8_t *getActiveLayerBuffer(void);

// The screen buffer the next flush sends (codes, INV): what a scene drew last (the OSD demo's
// scene transitions take the picture over)
void max7456CopyLayer(uint8_t *codes, uint8_t *inv, uint16_t n)
{
    n = MIN(n, (uint16_t)maxScreenSize);
    memcpy(codes, getActiveLayerBuffer(), n);
    memcpy(inv, invertLayer, n);
}

static uint8_t *getActiveLayerBuffer(void)
{
    return getLayerBuffer(activeLayer);
}

// HOS 0..63, VOS 0..31. Higher HOS/VOS shifts OSD right/down on screen.
static void max7456ApplyHosVos(void)
{
    const int hos = constrain((int)hosRegValue + hudMotionOffsetX, 0, 63);
    const int vos = constrain((int)vosRegValue + hudMotionOffsetY, 0, 31);

    if (hos != previousHosRegister) {
        previousHosRegister = hos;
        spiWriteReg(dev, MAX7456ADD_HOS, hos);
    }
    if (vos != previousVosRegister) {
        previousVosRegister = vos;
        spiWriteReg(dev, MAX7456ADD_VOS, vos);
    }
}

void max7456SetHudMotionOffset(int8_t x, int8_t y)
{
    hudMotionOffsetX = x;
    hudMotionOffsetY = y;
}

void max7456ResetHudMotionOffset(void)
{
    hudMotionOffsetX = 0;
    hudMotionOffsetY = 0;
}

void max7456GetHudMotionYLimits(int8_t *minY, int8_t *maxY)
{
    // Final VOS = vosRegValue + offset, clamped to 0..31.
    // +offset shifts OSD downward on screen.
    if (minY) {
        *minY = (int8_t)(-vosRegValue);           // most upward (smallest VOS)
    }
    if (maxY) {
        *maxY = (int8_t)(31 - vosRegValue);        // most downward (largest VOS) = floor
    }
}

void max7456GetHudMotionXLimits(int8_t *minX, int8_t *maxX)
{
    // Final HOS = hosRegValue + offset, clamped to 0..63.
    // +offset shifts OSD rightward on screen.
    if (minX) {
        *minX = (int8_t)(-hosRegValue);           // most leftward
    }
    if (maxX) {
        *maxX = (int8_t)(63 - hosRegValue);       // most rightward
    }
}

static void max7456SetRegisterVM1(void)
{
    uint8_t backgroundGray = BACKGROUND_BRIGHTNESS_28; // this is the device default background gray level
    uint8_t vm1Register = BLINK_TIME_1 | BLINK_DUTY_CYCLE_75_25; // device defaults
    if (deviceBackgroundType != DISPLAY_BACKGROUND_TRANSPARENT) {
        vm1Register |= BACKGROUND_MODE_GRAY;
        switch (deviceBackgroundType) {
        case DISPLAY_BACKGROUND_BLACK:
            backgroundGray = BACKGROUND_BRIGHTNESS_0;
            break;
        case DISPLAY_BACKGROUND_LTGRAY:
            backgroundGray = BACKGROUND_BRIGHTNESS_49;
            break;
        case DISPLAY_BACKGROUND_GRAY:
        default:
            backgroundGray = BACKGROUND_BRIGHTNESS_28;
            break;
        }
    }
    vm1Register |= (backgroundGray << 4);
    spiWriteReg(dev, MAX7456ADD_VM1, vm1Register);
}

// HOS as last applied (configured offset + HUD motion, 0..63; 32 = centre).
uint8_t max7456GetHosApplied(void)
{
    return (uint8_t)constrain((int)hosRegValue + hudMotionOffsetX, 0, 63);
}

static void max7456NvmWaitIdle(void);
static bool max7456WaitSpiIdle(void);

// Sync source, VM0[5:4]: SYNC_MODE_AUTO (BF default), _INTERNAL (the chip's own PAL/NTSC timing,
// no camera picture — a steady signal for capture / HDMI adapters that lose lock on the camera's)
// or _EXTERNAL. Applied now when the bus is free, and by every re-init.
void max7456SetSyncMode(uint8_t mode)
{
    syncModeBits = (mode == 1u) ? SYNC_MODE_INTERNAL : (mode == 2u) ? SYNC_MODE_EXTERNAL : SYNC_MODE_AUTO;
    if (max7456DeviceDetected && max7456WaitSpiIdle()) {
        videoSignalReg = (uint8_t)((videoSignalReg & (uint8_t)~SYNC_MODE_INTERNAL) | syncModeBits);
        spiWriteReg(dev, MAX7456ADD_VM0, videoSignalReg);
    }
}

// 512 glyphs (AT7456): CMAL[6] = CA[8] sticks. Probed with the OSD off — the init probe writes
// CMAL with the OSD on, which the AT7456 ignores, and takes it for a MAX7456.
// Measured: the upper 256 glyphs are RAM, not NVM — 0.76 ms per glyph
// write vs 4.1 ms for a real NVM slot, lost at power-off (power-up content is garbage), kept
// across a VM0 soft reset and an FC reflash. A per-glyph store: three slots hold three patterns.
bool max7456IsAt7456(void)
{
    static int8_t at = -1;
    if (at < 0) {
        if (!max7456DeviceDetected) {
            return false;
        }
        spiWait(dev);
        fontIsLoading = true;
        spiWriteReg(dev, MAX7456ADD_VM0, (uint8_t)(videoSignalReg & (uint8_t)~OSD_ENABLE)); // OSD off, sync kept
        max7456NvmWaitIdle();
        spiWriteReg(dev, MAX7456ADD_CMAL, 0x40);
        at = (spiReadRegMsk(dev, MAX7456ADD_CMAL) & 0x40u) ? 1 : 0;
        spiWriteReg(dev, MAX7456ADD_CMAL, 0);
        max7456EndFontWrite();
    }
    return at > 0;
}

uint8_t max7456GetRowsCount(void)
{
    return (videoSignalReg & VIDEO_MODE_PAL) ? VIDEO_LINES_PAL : VIDEO_LINES_NTSC;
}

// When clearing the shadow buffer we fill with 0 so that the characters will
// be flagged as changed when compared to the 0x20 used in the layer buffers.
static void max7456ClearShadowBuffer(void)
{
    memset(shadowBuffer, 0, maxScreenSize);
    memset(shadowInvert, 0, maxScreenSize);
}

// Buffer is filled with the whitespace character (0x20)
static void max7456ClearLayer(displayPortLayer_e layer)
{
    memset(getLayerBuffer(layer), 0x20, VIDEO_BUFFER_CHARS_PAL);
    if (layer == activeLayer) {
        memset(invertLayer, 0, VIDEO_BUFFER_CHARS_PAL);
    }
}

static void max7456ReInit(void)
{
    uint8_t srdata = 0;

    switch (videoSignalCfg) {
    case VIDEO_SYSTEM_PAL:
        videoSignalReg = VIDEO_MODE_PAL | OSD_ENABLE;
        break;

    case VIDEO_SYSTEM_NTSC:
        videoSignalReg = VIDEO_MODE_NTSC | OSD_ENABLE;
        break;

    case VIDEO_SYSTEM_AUTO:
        srdata = spiReadRegMsk(dev, MAX7456ADD_STAT);

        if (VIN_IS_NTSC(srdata)) {
            videoSignalReg = VIDEO_MODE_NTSC | OSD_ENABLE;
        } else if (VIN_IS_PAL(srdata)) {
            videoSignalReg = VIDEO_MODE_PAL | OSD_ENABLE;
        } else {
            // No valid input signal, fallback to default (PAL)
            videoSignalReg = VIDEO_MODE_PAL | OSD_ENABLE;
        }
        break;
    }

    videoSignalReg |= syncModeBits;

    if (videoSignalReg & VIDEO_MODE_PAL) { //PAL
        maxScreenSize = VIDEO_BUFFER_CHARS_PAL;
    } else {              // NTSC
        maxScreenSize = VIDEO_BUFFER_CHARS_NTSC;
    }

    // Set all rows to same charactor black/white level
    previousBlackWhiteRegister = INVALID_PREVIOUS_REGISTER_STATE;
    max7456Brightness(0, 2);
    // Re-enable MAX7456 (last function call disables it)

    // Make sure the Max7456 is enabled
    spiWriteReg(dev, MAX7456ADD_VM0, videoSignalReg);
    // Reinit restores configured base position; drop any transient HUD offset.
    max7456ResetHudMotionOffset();
    previousHosRegister = INVALID_PREVIOUS_REGISTER_STATE;
    previousVosRegister = INVALID_PREVIOUS_REGISTER_STATE;
    previousOsdmRegister = INVALID_PREVIOUS_REGISTER_STATE;
    max7456ApplyHosVos();

    max7456SetRegisterVM1();

    // Clear shadow to force redraw all screen
    max7456ClearShadowBuffer();
}

void max7456Preinit(const max7456Config_t *max7456Config)
{
    ioPreinitByTag(max7456Config->csTag, max7456Config->preInitOPU ? IOCFG_OUT_PP : IOCFG_IPU, PREINIT_PIN_STATE_HIGH);
}

// Here we init only CS and try to init MAX for first time.
// Also detect device type (MAX v.s. AT)

max7456InitStatus_e max7456Init(const max7456Config_t *max7456Config, const vcdProfile_t *pVcdProfile, bool cpuOverclock)
{
    max7456DeviceDetected = false;
    deviceBackgroundType = DISPLAY_BACKGROUND_TRANSPARENT;

    // initialize all layers
    for (unsigned i = 0; i < MAX7456_SUPPORTED_LAYER_COUNT; i++) {
        max7456ClearLayer(i);
    }

    max7456HardwareReset();

    if (!max7456Config->csTag || !spiSetBusInstance(dev, max7456Config->spiDevice)) {
        return MAX7456_INIT_NOT_CONFIGURED;
    }

    dev->busType_u.spi.csnPin = IOGetByTag(max7456Config->csTag);

    // Allow re-probe when a prior NOT_FOUND already claimed CS (checkReady rescan).
    // Any other owner means the pin is not ours — abort.
    if (!IOIsFreeOrPreinit(dev->busType_u.spi.csnPin)
        && IOGetOwner(dev->busType_u.spi.csnPin) != OWNER_OSD_CS) {
        return MAX7456_INIT_NOT_CONFIGURED;
    }

    IOInit(dev->busType_u.spi.csnPin, OWNER_OSD_CS, 0);
    IOConfigGPIO(dev->busType_u.spi.csnPin, SPI_IO_CS_CFG);
    IOHi(dev->busType_u.spi.csnPin);

    // Detect MAX7456 / AT7456 by an OSDM write/read-back.
    // Do NOT assume power-on default 0x1B: osd_demo (and max7456Osdm) change OSDM
    // at runtime, and a soft FC reboot does not power-cycle the OSD chip — a stale
    // non-0x1B value made the old "read default only" probe fail forever.

    spiSetClkDivisor(dev, spiCalculateDivider(MAX7456_INIT_MAX_SPI_CLK_HZ));

    // Write 0xff to conclude any current SPI transaction the MAX7456 is expecting
    spiWrite(dev, END_STRING);

    spiWriteReg(dev, MAX7456ADD_OSDM, 0x1B);
    previousOsdmRegister = 0x1B;
    uint8_t osdm = spiReadRegMsk(dev, MAX7456ADD_OSDM);

    if (osdm != 0x1B) {
        // The chip keeps power across FC reflash / soft reboot, so it can still be inside
        // an auto-increment session (16-bit framed or 8-bit data-only, datasheet p.29) and
        // eat every register write. Escape in both framings, then VM0[1] software reset
        // (all registers to defaults, display memory cleared, ~100 us; p.25).
        spiWrite(dev, END_STRING);
        spiWriteReg(dev, MAX7456ADD_DMDI, END_STRING);
        spiWrite(dev, END_STRING);
        spiWriteReg(dev, MAX7456ADD_VM0, MAX7456_RESET);
        delayMicroseconds(1000);
        spiWriteReg(dev, MAX7456ADD_OSDM, 0x1B);
        osdm = spiReadRegMsk(dev, MAX7456ADD_OSDM);
    }

    if (osdm != 0x1B) {
        IOConfigGPIO(dev->busType_u.spi.csnPin, IOCFG_IPU);
        IORelease(dev->busType_u.spi.csnPin);
        return MAX7456_INIT_NOT_FOUND;
    }

    // At this point, we can claim the ownership of the CS pin
    max7456DeviceDetected = true;
    IOInit(dev->busType_u.spi.csnPin, OWNER_OSD_CS, 0);

    // Detect device type by writing and reading CA[8] bit at CMAL[6].
    // This is a bit for accessing second half of character glyph storage, supported only by AT variant.

    spiWriteReg(dev, MAX7456ADD_CMAL, (1 << 6)); // CA[8] bit

    if (spiReadRegMsk(dev, MAX7456ADD_CMAL) & (1 << 6)) {
        max7456DeviceType = MAX7456_DEVICE_TYPE_AT;
    } else {
        max7456DeviceType = MAX7456_DEVICE_TYPE_MAX;
    }

#if defined(USE_OVERCLOCK)
    // Determine SPI clock divisor based on config and the device type.

    switch (max7456Config->clockConfig) {
    case MAX7456_CLOCK_CONFIG_HALF:
        max7456SpiClockDiv = spiCalculateDivider(MAX7456_MAX_SPI_CLK_HZ / 2);
        break;

    case MAX7456_CLOCK_CONFIG_NOMINAL:
    default:
        max7456SpiClockDiv = spiCalculateDivider(MAX7456_MAX_SPI_CLK_HZ);
        break;

    case MAX7456_CLOCK_CONFIG_DOUBLE:
        max7456SpiClockDiv = spiCalculateDivider(MAX7456_MAX_SPI_CLK_HZ * 2);
        break;
    }

    DEBUG_SET(DEBUG_MAX7456_SPICLOCK, DEBUG_MAX7456_SPICLOCK_OVERCLOCK, cpuOverclock);                              //!< CPU Overclocked
    DEBUG_SET(DEBUG_MAX7456_SPICLOCK, DEBUG_MAX7456_SPICLOCK_DEVTYPE, max7456DeviceType);                           //!< Device Type [enum:max7456DeviceType_e]
    DEBUG_SET(DEBUG_MAX7456_SPICLOCK, DEBUG_MAX7456_SPICLOCK_DIVISOR, max7456SpiClockDiv);                          //!< SPI Clock Divisor
    DEBUG_SET(DEBUG_MAX7456_SPICLOCK, DEBUG_MAX7456_SPICLOCK_X100, spiCalculateClock(max7456SpiClockDiv) / 10000);  //!< SPI Clock [unit:0.01MHz]
#else
    UNUSED(max7456Config);
    UNUSED(cpuOverclock);
    max7456SpiClockDiv = spiCalculateDivider(MAX7456_MAX_SPI_CLK_HZ);
#endif

    spiSetClkDivisor(dev, max7456SpiClockDiv);

    // force soft reset on Max7456
    spiWriteReg(dev, MAX7456ADD_VM0, MAX7456_RESET);

    // Wait for 200us before polling for completion of reset
    delayMicroseconds(200);

    // Wait for reset to complete
    while ((spiReadRegMsk(dev, MAX7456ADD_VM0) & MAX7456_RESET) != 0x00);

    // Setup values to write to registers
    videoSignalCfg = pVcdProfile->video_system;
    hosRegValue = 32 - pVcdProfile->h_offset;
    vosRegValue = 16 - pVcdProfile->v_offset;

    // Real init will be made later when driver detect idle.
    return MAX7456_INIT_OK;
}

/**
 * Sets inversion of black and white pixels.
 */
void max7456Invert(bool invert)
{
    if (invert) {
        displayMemoryModeReg |= INVERT_PIXEL_COLOR;
    } else {
        displayMemoryModeReg &= ~INVERT_PIXEL_COLOR;
    }

    if (displayMemoryModeReg != previousInvertRegister) {
        // clear the shadow buffer so all characters will be
        // redrawn with the proper invert state
        max7456ClearShadowBuffer();
        previousInvertRegister = displayMemoryModeReg;
        spiWriteReg(dev, MAX7456ADD_DMM, displayMemoryModeReg);
    }
}

/**
 * Sets the brightness of black and white pixels.
 *
 * @param black Black brightness (0-3, 0 is darkest)
 * @param white White brightness (0-3, 0 is darkest)
 */
// Row brightness wanted per row (what the last writes asked for) and the hold: while held (the
// OSD demo's scene-start flash) writes only update the wanted values; the release writes them.
static uint8_t rbWanted[16];
static bool rbHold;

// Register by register (polled, like max7456BrightnessRow): also called from the demo's VBLANK
// interrupt, where a DMA transfer with its completion interrupt below could not finish.
static void max7456RbWriteAll(const uint8_t *regs, uint8_t same)
{
    for (uint8_t r = 0; r < 16u; r++) {
        spiWriteReg(dev, (uint8_t)(MAX7456ADD_RB0 + r), regs ? regs[r] : same);
    }
}

// Scene start (OSD demo): every row's wanted level back to `reg` — rows a scene never sets end
// at it, not at what the previous scene (or a DMA level run) left in the register.
void max7456RbResetWanted(uint8_t reg)
{
    memset(rbWanted, reg & 0x0Fu, sizeof(rbWanted));
    previousBlackWhiteRegister = INVALID_PREVIOUS_REGISTER_STATE;
}

// hold: all rows to `flashReg` now and keep it (writes only update the wanted values);
// release: every row to its wanted value. Call in VBLANK (or with the OSD idle).
void max7456RbHold(bool hold, uint8_t flashReg)
{
    if (!max7456DeviceDetected) {
        return;
    }
    rbHold = hold;
    previousBlackWhiteRegister = INVALID_PREVIOUS_REGISTER_STATE;
    max7456RbWriteAll(hold ? NULL : rbWanted, (uint8_t)(flashReg & 0x0Fu));
}

bool max7456RbHeld(void)
{
    return rbHold;
}

void max7456Brightness(uint8_t black, uint8_t white)
{
    const uint8_t reg = (black << 2) | (3 - white);
    memset(rbWanted, reg, sizeof(rbWanted));
    if (rbHold) {
        return;
    }

    if (reg != previousBlackWhiteRegister) {
        previousBlackWhiteRegister = reg;
        STATIC_DMA_DATA_AUTO uint8_t buf[32];
        for (int i = MAX7456ADD_RB0, j = 0; i <= MAX7456ADD_RB15; i++) {
            buf[j++] = i;
            buf[j++] = reg;
        }
        spiReadWriteBuf(dev, buf, NULL, sizeof(buf));
    }
}

/**
 * Per-row black/white brightness (MAX7456 RB0..RB15).
 * black/white: 0 = darkest, 3 = brightest.
 */

void max7456BrightnessRow(uint8_t row, uint8_t black, uint8_t white)
{
    if (row > 15) {
        return;
    }
    const uint8_t reg = (uint8_t)(((black & 3) << 2) | (3 - (white & 3)));
    rbWanted[row] = reg;
    if (rbHold) {
        return;
    }
    // Invalidate the "all rows equal" cache so a later max7456Brightness() rewrites.
    previousBlackWhiteRegister = INVALID_PREVIOUS_REGISTER_STATE;
    spiWriteReg(dev, (uint8_t)(MAX7456ADD_RB0 + row), reg);
}

/**
 * OSD Insertion Mux (OSDM): trade sharpness vs cross-color/cross-luma.
 * value bits[5:3] = OSD pixel rise/fall (0=20ns .. 5=110ns),
 *       bits[2:0] = video↔OSD mux switch (0=30ns .. 5=120ns).
 * Datasheet default is 0x1B (both fields = 3).
 */
void max7456Osdm(uint8_t value)
{
    // Only codes 0..5 are valid in each field; clamp nibble-wise.
    const uint8_t rise = (uint8_t)MIN(value >> 3, 5);
    const uint8_t mux = (uint8_t)MIN(value & 7, 5);
    const uint8_t reg = (uint8_t)((rise << 3) | mux);
    if (reg != previousOsdmRegister) {
        previousOsdmRegister = reg;
        spiWriteReg(dev, MAX7456ADD_OSDM, reg);
    }
}

void max7456ClearScreen(void)
{
    max7456ClearLayer(activeLayer);
}

void max7456FillScreen(uint8_t c)
{
    memset(getActiveLayerBuffer(), c, maxScreenSize);
    memset(invertLayer, 0, maxScreenSize);
}

void max7456Invalidate(void)
{
    // Force every cell dirty vs shadow so drawScreen rewrites the whole screen.
    memset(shadowBuffer, 0xFF, maxScreenSize);
    memset(shadowInvert, 0xFF, maxScreenSize);
    max7456DrawPos = 0;
}

void max7456WriteCharEx(uint8_t x, uint8_t y, uint8_t c, bool invert)
{
    if (x < CHARS_PER_LINE && y < VIDEO_LINES_PAL) {
        const uint16_t pos = (uint16_t)(y * CHARS_PER_LINE + x);
        getActiveLayerBuffer()[pos] = c;
        invertLayer[pos] = invert ? 1 : 0;
    }
}

void max7456WriteChar(uint8_t x, uint8_t y, uint8_t c)
{
    max7456WriteCharEx(x, y, c, false);
}

void max7456Write(uint8_t x, uint8_t y, const char *text)
{
    if (y < VIDEO_LINES_PAL) {
        uint8_t *buffer = getActiveLayerBuffer();
        const uint32_t bufferYOffset = y * CHARS_PER_LINE;
        for (int i = 0, bufferXOffset = x; text[i] && bufferXOffset < CHARS_PER_LINE; i++, bufferXOffset++) {
            const uint16_t pos = (uint16_t)(bufferYOffset + bufferXOffset);
            buffer[pos] = text[i];
            invertLayer[pos] = 0;
        }
    }
}

bool max7456LayerSupported(displayPortLayer_e layer)
{
    if (layer == DISPLAYPORT_LAYER_FOREGROUND) {
        return true;
    } else {
        return false;
    }
}

bool max7456LayerSelect(displayPortLayer_e layer)
{
    if (max7456LayerSupported(layer)) {
        activeLayer = layer;
        return true;
    } else {
        return false;
    }
}

bool max7456LayerCopy(displayPortLayer_e destLayer, displayPortLayer_e sourceLayer)
{
    if ((sourceLayer != destLayer) && max7456LayerSupported(sourceLayer) && max7456LayerSupported(destLayer)) {
        memcpy(getLayerBuffer(destLayer), getLayerBuffer(sourceLayer), VIDEO_BUFFER_CHARS_PAL);
        return true;
    } else {
        return false;
    }
}

bool max7456DmaInProgress(void)
{
    return max7456ActiveDma;
}

bool max7456BuffersSynced(void)
{
    for (int i = 0; i < maxScreenSize; i++) {
        if (displayLayers[DISPLAYPORT_LAYER_FOREGROUND].buffer[i] != shadowBuffer[i]
            || invertLayer[i] != shadowInvert[i]) {
            return false;
        }
    }
    return true;
}

bool max7456ReInitIfRequired(bool forceStallCheck)
{
    static timeMs_t lastSigCheckMs = 0;
    static timeMs_t videoDetectTimeMs = 0;
    static uint16_t reInitCount = 0;
    static timeMs_t lastStallCheckMs = MAX7456_STALL_CHECK_INTERVAL_MS / 2; // offset so that it doesn't coincide with the signal check

    // A mid-glyph engine owns the chip and the bus (scene 6 bursts from a TIM5 interrupt while
    // this runs in the OSD task): a VM0 read here could interleave with a raster burst, read
    // garbage, "detect a stall" and re-init the chip mid-scene. Skip checks while hot.
    if (midGlyphSpiHot) {
        return false;
    }

    const timeMs_t nowMs = millis();

    bool stalled = false;
    if (forceStallCheck || (lastStallCheckMs + MAX7456_STALL_CHECK_INTERVAL_MS < nowMs)) {
        lastStallCheckMs = nowMs;

        // Write 0xff to conclude any current SPI transaction the MAX7456 is expecting
        spiWrite(dev, END_STRING);

        stalled = (spiReadRegMsk(dev, MAX7456ADD_VM0) != videoSignalReg);
    }

    if (stalled) {
        max7456ReInit();
    } else if ((videoSignalCfg == VIDEO_SYSTEM_AUTO)
              && ((nowMs - lastSigCheckMs) > MAX7456_SIGNAL_CHECK_INTERVAL_MS)) {

        // Write 0xff to conclude any current SPI transaction the MAX7456 is expecting
        spiWrite(dev, END_STRING);

        // Adjust output format based on the current input format.

        const uint8_t videoSense = spiReadRegMsk(dev, MAX7456ADD_STAT);

        DEBUG_SET(DEBUG_MAX7456_SIGNAL, DEBUG_MAX7456_SIGNAL_MODEREG, videoSignalReg & VIDEO_MODE_MASK);  //!< Video Mode Register
        DEBUG_SET(DEBUG_MAX7456_SIGNAL, DEBUG_MAX7456_SIGNAL_SENSE, videoSense & 0x7);                    //!< Video Sense
        DEBUG_SET(DEBUG_MAX7456_SIGNAL, DEBUG_MAX7456_SIGNAL_ROWS, max7456GetRowsCount());                //!< Row Count

        if (videoSense & STAT_LOS) {
            videoDetectTimeMs = 0;
        } else {
            if ((VIN_IS_PAL(videoSense) && VIDEO_MODE_IS_NTSC(videoSignalReg))
              || (VIN_IS_NTSC_alt(videoSense) && VIDEO_MODE_IS_PAL(videoSignalReg))) {
                if (videoDetectTimeMs) {
                    if (millis() - videoDetectTimeMs > VIDEO_SIGNAL_DEBOUNCE_MS) {
                        max7456ReInit();
                        DEBUG_SET(DEBUG_MAX7456_SIGNAL, DEBUG_MAX7456_SIGNAL_REINIT, ++reInitCount);  //!< Reinit Count
                    }
                } else {
                    // Wait for signal to stabilize
                    videoDetectTimeMs = millis();
                }
            }
        }

        lastSigCheckMs = nowMs;
    }

    return stalled;
}

// Called in ISR context
static busStatus_e max7456_callbackReady(uintptr_t arg)
{
    UNUSED(arg);

    max7456ActiveDma = false;

    return BUS_READY;
}

// Return true if screen still being transferred
bool max7456DrawScreen(void)
{
    // This routine doesn't block so need to use static data
    static busSegment_t segments[] = {
            {.u.link = {NULL, NULL}, 0, true, max7456_callbackReady},
            {.u.link = {NULL, NULL}, 0, true, NULL},
    };

    if (!fontIsLoading) {
        uint8_t *buffer = getActiveLayerBuffer();
        int spiBufIndex = 0;
        int maxSpiBufStartIndex;
        timeDelta_t maxEncodeTime;
        bool setAddress = true;
        bool autoInc = false;
        int posLimit = max7456DrawPos + (maxScreenSize / 2);

#ifdef USE_DMA
        const bool useDma = spiUseSDO_DMA(dev);
#else
        const bool useDma = false;
#endif
        maxSpiBufStartIndex = useDma ? MAX_BYTES2SEND : MAX_BYTES2SEND_POLLED;
        maxEncodeTime = useDma ? MAX_ENCODE_US : MAX_ENCODE_US_POLLED;

        // Abort for now if the bus is still busy
        if (spiIsBusy(dev)) {
            // Not finished yet
            return true;
        }

        // NOTE: Do NOT apply HOS/VOS here at pass start. On soft-scroll wrap
        // (HOS → 0 + cell advance) that would snap the old character grid right
        // before the new cells are written. Apply after the pass completes.

        timeUs_t startTime = micros();

        // Allow for an ESCAPE, a reset of DMM and a two byte MAX7456ADD_DMM command at end of buffer
        // Extra headroom: DMM updates when per-char INV attribute changes mid-pass.
        maxSpiBufStartIndex -= 8;

        // 16-bit mode copies DMM[5:3] (LBC/BLK/INV) into each written character's attribute.
        uint8_t drawDmm = displayMemoryModeReg & (uint8_t)~INVERT_PIXEL_COLOR;

        // Initialise the transfer buffer
        while ((spiBufIndex < maxSpiBufStartIndex) && (max7456DrawPos < posLimit) && (cmpTimeUs(micros(), startTime) < maxEncodeTime)) {
            const bool charDirty = buffer[max7456DrawPos] != shadowBuffer[max7456DrawPos];
            const bool invDirty = invertLayer[max7456DrawPos] != shadowInvert[max7456DrawPos];
            if (charDirty || invDirty) {
                if (buffer[max7456DrawPos] == 0xff) {
                    buffer[max7456DrawPos] = ' ';
                }

                const uint8_t wantDmm = invertLayer[max7456DrawPos]
                    ? (uint8_t)(displayMemoryModeReg | INVERT_PIXEL_COLOR)
                    : (uint8_t)(displayMemoryModeReg & (uint8_t)~INVERT_PIXEL_COLOR);

                if (setAddress || !autoInc || wantDmm != drawDmm) {
                    if (autoInc && !setAddress) {
                        spiBuf[spiBufIndex++] = MAX7456ADD_DMDI;
                        spiBuf[spiBufIndex++] = END_STRING;
                    }

                    // Peek ahead: auto-inc only when next cell shares char+inv dirty streak and same INV.
                    bool nextDirty = false;
                    bool nextSameInv = false;
                    if (max7456DrawPos + 1 < maxScreenSize) {
                        nextDirty = (buffer[max7456DrawPos + 1] != shadowBuffer[max7456DrawPos + 1])
                            || (invertLayer[max7456DrawPos + 1] != shadowInvert[max7456DrawPos + 1]);
                        nextSameInv = invertLayer[max7456DrawPos + 1] == invertLayer[max7456DrawPos];
                    }

                    drawDmm = wantDmm;
                    if (nextDirty && nextSameInv) {
                        spiBuf[spiBufIndex++] = MAX7456ADD_DMM;
                        spiBuf[spiBufIndex++] = drawDmm | DMM_AUTO_INC;
                        autoInc = true;
                    } else {
                        spiBuf[spiBufIndex++] = MAX7456ADD_DMM;
                        spiBuf[spiBufIndex++] = drawDmm;
                        autoInc = false;
                    }

                    spiBuf[spiBufIndex++] = MAX7456ADD_DMAH;
                    spiBuf[spiBufIndex++] = max7456DrawPos >> 8;
                    spiBuf[spiBufIndex++] = MAX7456ADD_DMAL;
                    spiBuf[spiBufIndex++] = max7456DrawPos & 0xff;

                    setAddress = false;
                }

                spiBuf[spiBufIndex++] = MAX7456ADD_DMDI;
                spiBuf[spiBufIndex++] = buffer[max7456DrawPos];

                shadowBuffer[max7456DrawPos] = buffer[max7456DrawPos];
                shadowInvert[max7456DrawPos] = invertLayer[max7456DrawPos];
            } else {
                if (!setAddress) {
                    setAddress = true;
                    if (autoInc) {
                        spiBuf[spiBufIndex++] = MAX7456ADD_DMDI;
                        spiBuf[spiBufIndex++] = END_STRING;
                    }
                }
            }

            if (++max7456DrawPos >= maxScreenSize) {
                max7456DrawPos = 0;
                // All dirty cells for this buffer are now queued/sent — apply HUD
                // motion so HOS wrap lands with the matching character grid.
                max7456ApplyHosVos();
                break;
            }
        }

        if (autoInc) {
            if (!setAddress) {
                spiBuf[spiBufIndex++] = MAX7456ADD_DMDI;
                spiBuf[spiBufIndex++] = END_STRING;
            }

            spiBuf[spiBufIndex++] = MAX7456ADD_DMM;
            spiBuf[spiBufIndex++] = displayMemoryModeReg;
        }

        if (spiBufIndex) {
            segments[0].u.buffers.txData = spiBuf;
            segments[0].len = spiBufIndex;

            max7456ActiveDma = true;

            spiSequence(dev, &segments[0]);

            // Non-blocking, so transfer still in progress if using DMA
        }
    }

    return (max7456DrawPos != 0);
}

// should not be used when armed
void max7456RefreshAll(void)
{
    max7456ReInitIfRequired(true);
    while (max7456DrawScreen());
}

bool max7456WriteNvm(uint8_t char_address, const uint8_t *font_data)
{
    if (!max7456DeviceDetected) {
        return false;
    }

    // Block pending completion of any prior SPI access
    spiWait(dev);

    // disable display
    fontIsLoading = true;
    spiWriteReg(dev, MAX7456ADD_VM0, (uint8_t)(videoSignalReg & (uint8_t)~OSD_ENABLE)); // OSD off, sync kept

    spiWriteReg(dev, MAX7456ADD_CMAH, char_address); // set start address high

    for (int x = 0; x < 54; x++) {
        spiWriteReg(dev, MAX7456ADD_CMAL, x); //set start address low
        spiWriteReg(dev, MAX7456ADD_CMDI, font_data[x]);
#ifdef LED0_TOGGLE
        LED0_TOGGLE;
#else
        LED1_TOGGLE;
#endif
    }

    // Transfer 54 bytes from shadow ram to NVM

    spiWriteReg(dev, MAX7456ADD_CMM, WRITE_NVR);

    // Wait until bit 5 in the status register returns to 0 (12ms)

    while ((spiReadRegMsk(dev, MAX7456ADD_STAT) & STAT_NVR_BUSY) != 0x00);

    return true;
}

// True when NVM character `char_address` already holds `font_data` (54 bytes): lets callers
// skip a 12 ms write that also wears the NVM. Leaves the OSD disabled like WriteNvm — finish
// with max7456EndFontWrite(). A failed read only makes the caller write.
bool max7456NvmMatches(uint8_t char_address, const uint8_t *font_data)
{
    if (!max7456DeviceDetected) {
        return false;
    }
    spiWait(dev);
    fontIsLoading = true;
    spiWriteReg(dev, MAX7456ADD_VM0, (uint8_t)(videoSignalReg & (uint8_t)~OSD_ENABLE)); // OSD off, sync kept
    spiWriteReg(dev, MAX7456ADD_CMAH, char_address);
    spiWriteReg(dev, MAX7456ADD_CMAL, 0); // CMAL[6] = CA[8] on the AT7456: the lower 256
    spiWriteReg(dev, MAX7456ADD_CMM, READ_NVR); // NVM → shadow RAM (< 0.5 us)
    delayMicroseconds(1);
    for (int x = 0; x < 54; x++) {
        spiWriteReg(dev, MAX7456ADD_CMAL, x);
        if (spiReadRegMsk(dev, MAX7456ADD_CMDO_MSK) != font_data[x]) {
            return false;
        }
    }
    return true;
}

// 9-bit glyph address (AT7456: 512 glyphs, CA[8] = CMAL[6]; the upper 256 are RAM, see
// max7456IsAt7456). Read: 54 bytes into `out`. Both leave the OSD disabled like WriteNvm —
// finish with max7456EndFontWrite().
static void max7456NvmWaitIdle(void)
{
    while ((spiReadRegMsk(dev, MAX7456ADD_STAT) & STAT_NVR_BUSY) != 0x00);
}

bool max7456NvmRead9(uint16_t addr, uint8_t out[54])
{
    if (!max7456DeviceDetected) {
        return false;
    }
    spiWait(dev);
    fontIsLoading = true;
    spiWriteReg(dev, MAX7456ADD_VM0, (uint8_t)(videoSignalReg & (uint8_t)~OSD_ENABLE)); // OSD off, sync kept
    max7456NvmWaitIdle();
    const uint8_t ca8 = (addr & 0x100u) ? 0x40u : 0u;
    spiWriteReg(dev, MAX7456ADD_CMAH, (uint8_t)addr);
    spiWriteReg(dev, MAX7456ADD_CMAL, ca8);
    spiWriteReg(dev, MAX7456ADD_CMM, READ_NVR);
    max7456NvmWaitIdle();
    for (uint8_t x = 0; x < 54u; x++) {
        spiWriteReg(dev, MAX7456ADD_CMAL, (uint8_t)(ca8 | x));
        out[x] = spiReadRegMsk(dev, MAX7456ADD_CMDO_MSK);
    }
    spiWriteReg(dev, MAX7456ADD_CMAL, 0);
    return true;
}

bool max7456NvmWrite9(uint16_t addr, const uint8_t data[54])
{
    if (!max7456DeviceDetected) {
        return false;
    }
    spiWait(dev);
    fontIsLoading = true;
    spiWriteReg(dev, MAX7456ADD_VM0, (uint8_t)(videoSignalReg & (uint8_t)~OSD_ENABLE)); // OSD off, sync kept
    max7456NvmWaitIdle();
    const uint8_t ca8 = (addr & 0x100u) ? 0x40u : 0u;
    spiWriteReg(dev, MAX7456ADD_CMAH, (uint8_t)addr);
    for (uint8_t x = 0; x < 54u; x++) {
        spiWriteReg(dev, MAX7456ADD_CMAL, (uint8_t)(ca8 | x));
        spiWriteReg(dev, MAX7456ADD_CMDI, data[x]);
    }
    spiWriteReg(dev, MAX7456ADD_CMM, WRITE_NVR); // at (CMAH, CMAL[6]): the last CMAL keeps CA[8]
    max7456NvmWaitIdle();
    spiWriteReg(dev, MAX7456ADD_CMAL, 0);
    return true;
}

void max7456EndFontWrite(void)
{
    // WriteNvm leaves OSD disabled (VM0=0) and blocks drawScreen via fontIsLoading.
    fontIsLoading = false;
    max7456ReInit();
}

#ifdef MAX7456_NRST_PIN
static IO_t max7456ResetPin        = IO_NONE;
#endif

void max7456HardwareReset(void)
{
#ifdef MAX7456_NRST_PIN
#define IO_RESET_CFG      IO_CONFIG(GPIO_Mode_OUT, GPIO_Speed_2MHz, GPIO_OType_PP, GPIO_PuPd_DOWN)

    max7456ResetPin = IOGetByTag(IO_TAG(MAX7456_NRST_PIN));
    IOInit(max7456ResetPin, OWNER_OSD, 0);
    IOConfigGPIO(max7456ResetPin, IO_RESET_CFG);

    // RESET 50ms long pulse, followed by 100us pause
    IOLo(max7456ResetPin);
    delay(50);
    IOHi(max7456ResetPin);
    delayMicroseconds(100);
#else
    // Allow device 50ms to powerup
    delay(50);
#endif
}

bool max7456IsDeviceDetected(void)
{
    return max7456DeviceDetected;
}

void max7456SetBackgroundType(displayPortBackground_e backgroundType)
{
    deviceBackgroundType = backgroundType;

    max7456SetRegisterVM1();
}

// Wait until the draw DMA/SPI path is idle enough for a polled register poke.
static bool max7456WaitSpiIdle(void)
{
    if (!max7456DeviceDetected || fontIsLoading) {
        return false;
    }
    // Finish any in-flight DMA segment before stealing the bus. DWT timeout: this also runs
    // inside the scene-10 TIM5 interrupt, where micros() does not advance past 1 ms.
    const uint32_t spinStart = getCycleCounter();
    const uint32_t spinTicks = clockMicrosToCycles(5000);
    while (max7456ActiveDma || spiIsBusy(dev)) {
        if (getCycleCounter() - spinStart > spinTicks) {
            return false;
        }
    }
    // Conclude any dangling auto-increment transaction the chip may expect.
    spiWrite(dev, END_STRING);
    return true;
}

void max7456ApplyHudMotionNow(void)
{
    if (!max7456WaitSpiIdle()) {
        return;
    }
    max7456ApplyHosVos();
}

// Burst-fill `count` consecutive character cells with the same glyph.
// 16-bit auto-inc like max7456DrawScreen: address → DMM|AI → DMDI writes → END, one CS frame.
// commitShadow=false: raster hot path skips the shadow walk.
bool max7456WriteDisplaySramRowFillEx(uint16_t addr, uint8_t glyph, uint8_t count, bool commitShadow)
{
    if (count == 0 || addr >= maxScreenSize) {
        return false;
    }
    if ((uint16_t)(addr + count) > maxScreenSize) {
        count = (uint8_t)(maxScreenSize - addr);
    }
    if (count > CHARS_PER_LINE) {
        count = CHARS_PER_LINE;
    }
    if (commitShadow) {
        if (!max7456WaitSpiIdle()) {
            return false;
        }
    } else if (max7456ActiveDma || spiIsBusy(dev)) {
        if (!max7456WaitSpiIdle()) {
            return false;
        }
    }

    // The whole auto-increment run in ONE CS frame. Measured on AT7456: an
    // AI run split into per-register CS frames desyncs the chip - it stores the first byte of
    // every later frame as display data and eats register writes until the 0xFF escape.
    static DMA_DATA uint8_t buf[CHARS_PER_LINE * 2 + 10];
    const uint8_t drawDmm = (uint8_t)((displayMemoryModeReg & (uint8_t)~INVERT_PIXEL_COLOR) | DMM_AUTO_INC);
    uint16_t idx = 0;
    buf[idx++] = MAX7456ADD_DMAH; buf[idx++] = (uint8_t)((addr >> 8) & 0x01);
    buf[idx++] = MAX7456ADD_DMAL; buf[idx++] = (uint8_t)(addr & 0xff);
    buf[idx++] = MAX7456ADD_DMM;  buf[idx++] = drawDmm;
    for (uint8_t i = 0; i < count; i++) {
        buf[idx++] = MAX7456ADD_DMDI; buf[idx++] = glyph;
    }
    buf[idx++] = MAX7456ADD_DMDI; buf[idx++] = END_STRING;
    buf[idx++] = MAX7456ADD_DMM;  buf[idx++] = displayMemoryModeReg;
    spiReadWriteBuf(dev, buf, NULL, idx);
    previousInvertRegister = displayMemoryModeReg;

    if (commitShadow) {
        for (uint8_t i = 0; i < count; i++) {
            max7456CommitShadowCell((uint16_t)(addr + i), glyph, false);
        }
    }
    return true;
}

static bool midGlyphSpiHot;
static uint16_t midGlyphSavedDiv;
#ifdef USE_DMA
static bool midGlyphSavedDma;
#endif

#if defined(STM32F4) || defined(STM32F7)
static void max7456SpiDrain(SPI_TypeDef *spi)
{
    while (!(spi->SR & SPI_SR_TXE)) {
    }
#if defined(STM32F7)
    while (spi->SR & SPI_SR_FTLVL) {
    }
#endif
    while (spi->SR & SPI_SR_BSY) {
    }
}

// Received data nobody reads (the F7 keeps up to 4 bytes in its RX FIFO); then an SR read
// clears OVR.
static void max7456SpiDropRx(SPI_TypeDef *spi)
{
#if defined(STM32F7)
    while (spi->SR & SPI_SR_FRLVL) {
        (void)*(volatile uint8_t *)&spi->DR;
    }
#else
    (void)spi->DR;
#endif
}

// Mid-glyph hot path, write-only. The generic polled transfer sends a byte, waits for its
// RX byte, reads it, and does it through stdperiph flag calls — ~1.25 us/byte at 13.5 MHz
// where the wire needs 0.59 us. Writes to the MAX7456 return nothing useful, so keep TXE
// fed back-to-back and drop RX once at the end (DR then SR read clears OVR).
// Only used when the bus already carries this device's clock/mode (the generic path
// applies a pending divisor change lazily) — otherwise fall back.
static bool max7456SpiTxOnlyBurst(const uint8_t *buf, uint16_t len)
{
    MAX7456_SESSION_GUARD(false);
    busDevice_t *bus = dev->bus;
    if (dev->busType_u.spi.speed != bus->busType_u.spi.speed
        || dev->busType_u.spi.leadingEdge != bus->busType_u.spi.leadingEdge) {
        return false;
    }
    SPI_TypeDef *spi = (SPI_TypeDef *)bus->busType_u.spi.instance;
    IOLo(dev->busType_u.spi.csnPin);
    for (uint16_t i = 0; i < len; i++) {
        while (!(spi->SR & SPI_SR_TXE)) {
        }
        *(volatile uint8_t *)&spi->DR = buf[i];
    }
    max7456SpiDrain(spi);
    max7456SpiDropRx(spi);
    (void)spi->SR;
    IOHi(dev->busType_u.spi.csnPin);
    return true;
}

// Zone session (OSD demo, HSYNC wired): over the active field SPI runs 16-bit inside one CS
// frame (DMA1 cannot write GPIO, so no per-write CS; a long frame is fine on the AT7456). The
// demo's slot engine then writes every word by timer-paced DMA straight into SPI DR — glyphs,
// zone registers and fillers alike — so nothing else may touch the bus until End.
static bool zone16;
static uint16_t zone16SavedBr; // CR1 baud bits before the session
#define ZONE16_LIMIT_HZ 30000000u // session SPI clock limit: 27 MHz on F411 / F7, 21 MHz on F405

static uint16_t zone16SavedCr2; // CR2 DMA request bits before the session
#if defined(STM32F7)
static uint32_t zone16SavedDs;  // CR2 frame size and RX threshold before the session
#endif

// The bus driver's SPI streams stay off in the session.
static void max7456SpiStreamOff(dmaChannelDescriptor_t *dma)
{
    if (dma) {
        DMA_Stream_TypeDef *stream = (DMA_Stream_TypeDef *)dma->ref;
        stream->CR &= ~DMA_SxCR_EN;
        while (stream->CR & DMA_SxCR_EN) {
        }
    }
}

// The session's own SPI clock: the fastest APB / 2^(BR+1) at or under the limit.
static uint16_t max7456SessionBr(SPI_TypeDef *spi)
{
    const bool apb1 = (spi == SPI2) || (spi == SPI3);
    const uint32_t ppre = (RCC->CFGR >> (apb1 ? 10u : 13u)) & 7u;
    const uint32_t apb = SystemCoreClock / ((ppre < 4u) ? 1u : (1u << (ppre - 3u)));
    const uint32_t limit = ZONE16_LIMIT_HZ;
    uint16_t br = 0;
    while (br < 7u && apb / (2u << br) > limit) {
        br++;
    }
    return br;
}

bool max7456Zone16Active(void)
{
    return zone16;
}

void max7456Zone16Begin(void)
{
    SPI_TypeDef *spi = (SPI_TypeDef *)dev->bus->busType_u.spi.instance;
    max7456SpiDrain(spi);
    spi->CR1 &= (uint16_t)~SPI_CR1_SPE;
    zone16SavedBr = spi->CR1 & SPI_CR1_BR;
#if defined(STM32F7)
    zone16SavedDs = spi->CR2 & (SPI_CR2_DS | SPI_CR2_FRXTH);
    spi->CR2 = (spi->CR2 & ~(SPI_CR2_DS | SPI_CR2_FRXTH)) | SPI_CR2_DS; // 16-bit frames
    spi->CR1 = (uint16_t)((spi->CR1 & ~SPI_CR1_BR) | (max7456SessionBr(spi) << 3));
#else
    spi->CR1 = (uint16_t)((spi->CR1 & ~SPI_CR1_BR) | (max7456SessionBr(spi) << 3) | SPI_CR1_DFF);
#endif
    spi->CR1 |= SPI_CR1_SPE;
    max7456SpiStreamOff(dev->bus->dmaTx);
    // Neither SPI DMA request in the session: the slot DMA writes DR itself, and an RX stream left
    // armed by the bus driver would copy every received word into someone's buffer.
    zone16SavedCr2 = spi->CR2 & (SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);
    spi->CR2 &= (uint16_t)~(SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);
    max7456SpiStreamOff(dev->bus->dmaRx);
    IOLo(dev->busType_u.spi.csnPin);
    zone16 = true;
}

void max7456Zone16End(void)
{
    if (!zone16) {
        return;
    }
    SPI_TypeDef *spi = (SPI_TypeDef *)dev->bus->busType_u.spi.instance;
    max7456SpiDrain(spi);
    IOHi(dev->busType_u.spi.csnPin);
    spi->CR1 &= (uint16_t)~SPI_CR1_SPE;
#if defined(STM32F7)
    spi->CR1 = (uint16_t)((spi->CR1 & ~SPI_CR1_BR) | zone16SavedBr);
    spi->CR2 = (spi->CR2 & ~(SPI_CR2_DS | SPI_CR2_FRXTH)) | zone16SavedDs;
    max7456SpiDropRx(spi); // the session's received words
#else
    spi->CR1 = (uint16_t)((spi->CR1 & ~(SPI_CR1_DFF | SPI_CR1_BR)) | zone16SavedBr);
    while (spi->SR & SPI_SR_RXNE) {
        (void)spi->DR; // the session's received words
    }
#endif
    spi->SR = 0; // OVR from never reading them
    spi->CR2 = (uint16_t)(spi->CR2 | zone16SavedCr2);
    spi->CR1 |= SPI_CR1_SPE;
    (void)spi->DR;
    (void)spi->SR;
    zone16 = false;
}

// The 16-bit word time of the session, in core clocks (slot planning).
uint32_t max7456SessionWordTicks(void)
{
    SPI_TypeDef *spi = (SPI_TypeDef *)dev->bus->busType_u.spi.instance;
    const bool apb1 = (spi == SPI2) || (spi == SPI3);
    const uint32_t ppre = (RCC->CFGR >> (apb1 ? 10u : 13u)) & 7u;
    const uint32_t apbDiv = (ppre < 4u) ? 1u : (1u << (ppre - 3u));
    return 16u * apbDiv * (2u << max7456SessionBr(spi));
}

volatile void *max7456SpiDataRegister(void)
{
    return &((SPI_TypeDef *)dev->bus->busType_u.spi.instance)->DR;
}
#endif

// DMM as the driver sets it (attribute bits for new characters), for writers of their own.
uint8_t max7456DmmBase(void)
{
    return displayMemoryModeReg;
}

static DMA_DATA uint8_t midGlyphRowBuf[256];

// Mid-glyph hot path: encode one row burst from a segment plan. Two framings, both 16-bit:
//  - autoInc: DMM|AI, [DMAH], DMAL, (DMDI,ca)×n, (DMDI,0xFF)   = 6 (+2) + 2n bytes
//  - single:  [DMM non-AI], [DMAH], DMAL, (DMDI,ca) per cell  = 4 (+2) bytes per cell
// Isolated changed cells are far cheaper as plain addressed writes than as an AI run, and
// nothing unchanged gets rewritten (fewer SRAM writes → fewer internal-read collisions,
// datasheet p.40). DMAH is only sent when address bit 8 changes. segLastByte[i] = offset just
// past the last character byte of segment i (when that cell lands) for beam racing.
// Framing rule measured on AT7456: an AI run must be ONE CS frame of
// (register, data) pairs. Data-only bytes after DMDI (datasheet Fig. 20/21) or a CS toggle
// between the pairs desync the chip: the data is dropped, the first byte of every later frame
// is stored as display data, and only the 0xFF escape (bare, then via DMDI) brings it back.
// Per-cell INV: invMask bit c inverts column c (black ↔ white). The attribute comes from DMM
// at each DMDI write, and rewriting DMM inside an auto-increment run keeps the run going
// (checked on AT7456), so an attribute change costs 2 bytes.
// segFirstByte (optional) = offset just past the first cell of each segment.
uint16_t max7456EncodeDisplaySramRowAttr(uint16_t rowAddr, const uint8_t *glyphs, uint32_t invMask,
                                         const max7456SramSeg_t *seg, uint8_t nSeg,
                                         uint16_t *segFirstByte, uint16_t *segLastByte)
{
    const uint8_t dmmAi = (uint8_t)((displayMemoryModeReg & (uint8_t)~INVERT_PIXEL_COLOR) | DMM_AUTO_INC);
    const uint8_t dmmSingle = (uint8_t)(dmmAi & (uint8_t)~DMM_AUTO_INC);
    uint16_t idx = 0;
    int8_t singleInv = -1; // INV of the non-AI DMM currently in the chip, -1 = unknown
    int16_t high = -1; // DMAH bit 8 currently in the chip, -1 = unknown
    for (uint8_t i = 0; i < nSeg; i++) {
        const uint16_t addr = (uint16_t)(rowAddr + seg[i].col);
        uint8_t len = seg[i].len;
        if (len == 0 || addr >= maxScreenSize) {
            segLastByte[i] = idx;
            if (segFirstByte) {
                segFirstByte[i] = idx;
            }
            continue;
        }
        if ((uint16_t)(addr + len) > maxScreenSize) {
            len = (uint8_t)(maxScreenSize - addr);
        }
        const uint16_t need = (uint16_t)(seg[i].autoInc ? 10u + 4u * len : 8u + 8u * len);
        if ((uint16_t)(idx + need) > sizeof(midGlyphRowBuf)) {
            return 0;
        }
        if (seg[i].autoInc) {
            uint8_t inv = (uint8_t)((invMask >> seg[i].col) & 1u);
            midGlyphRowBuf[idx++] = MAX7456ADD_DMM;
            midGlyphRowBuf[idx++] = (uint8_t)(dmmAi | (inv ? INVERT_PIXEL_COLOR : 0u));
            if (high != (int16_t)(addr >> 8)) {
                midGlyphRowBuf[idx++] = MAX7456ADD_DMAH;
                midGlyphRowBuf[idx++] = (uint8_t)((addr >> 8) & 0x01);
                high = (int16_t)(addr >> 8);
            }
            midGlyphRowBuf[idx++] = MAX7456ADD_DMAL;
            midGlyphRowBuf[idx++] = (uint8_t)(addr & 0xff);
            for (uint8_t k = 0; k < len; k++) {
                const uint8_t want = (uint8_t)((invMask >> (seg[i].col + k)) & 1u);
                if (want != inv) {
                    inv = want;
                    midGlyphRowBuf[idx++] = MAX7456ADD_DMM; // attribute change inside the run
                    midGlyphRowBuf[idx++] = (uint8_t)(dmmAi | (inv ? INVERT_PIXEL_COLOR : 0u));
                }
                midGlyphRowBuf[idx++] = MAX7456ADD_DMDI;
                midGlyphRowBuf[idx++] = glyphs[seg[i].col + k];
                if (k == 0u && segFirstByte) {
                    segFirstByte[i] = idx;
                }
            }
            segLastByte[i] = idx;
            midGlyphRowBuf[idx++] = MAX7456ADD_DMDI;
            midGlyphRowBuf[idx++] = END_STRING;
            singleInv = -1;
            if ((addr >> 8) != ((addr + len - 1u) >> 8)) {
                high = -1; // the AI counter crossed 255→256; chip DMAH state unknown
            }
        } else {
            for (uint8_t k = 0; k < len; k++) {
                const uint16_t a = (uint16_t)(addr + k);
                const int8_t want = (int8_t)((invMask >> (seg[i].col + k)) & 1u);
                if (want != singleInv) {
                    midGlyphRowBuf[idx++] = MAX7456ADD_DMM;
                    midGlyphRowBuf[idx++] = (uint8_t)(dmmSingle | (want ? INVERT_PIXEL_COLOR : 0u));
                    singleInv = want;
                }
                if (high != (int16_t)(a >> 8)) {
                    midGlyphRowBuf[idx++] = MAX7456ADD_DMAH;
                    midGlyphRowBuf[idx++] = (uint8_t)((a >> 8) & 0x01);
                    high = (int16_t)(a >> 8);
                }
                midGlyphRowBuf[idx++] = MAX7456ADD_DMAL;
                midGlyphRowBuf[idx++] = (uint8_t)(a & 0xff);
                midGlyphRowBuf[idx++] = MAX7456ADD_DMDI;
                midGlyphRowBuf[idx++] = glyphs[seg[i].col + k];
                if (k == 0u && segFirstByte) {
                    segFirstByte[i] = idx;
                }
            }
            segLastByte[i] = idx;
        }
    }
    return idx;
}

// Send what max7456EncodeDisplaySramRow() prepared. Only valid between MidGlyphSpiBegin/End.
bool max7456SendEncodedDisplaySram(uint16_t len)
{
    if (!midGlyphSpiHot || len == 0 || len > sizeof(midGlyphRowBuf)) {
        return false;
    }
#if defined(STM32F4) || defined(STM32F7)
    if (zone16) {
        return false; // the slot engine owns the bus
    }
#endif
    if (max7456ActiveDma || spiIsBusy(dev)) {
        if (!max7456WaitSpiIdle()) {
            return false;
        }
    }
#if defined(STM32F4) || defined(STM32F7)
    if (max7456SpiTxOnlyBurst(midGlyphRowBuf, len)) {
        return true;
    }
#endif
    busSegment_t segments[] = {
        {.u.buffers = {midGlyphRowBuf, NULL}, len, true, NULL},
        {.u.link = {NULL, NULL}, 0, true, NULL},
    };
    spiSequence(dev, &segments[0]);
    spiWait(dev);
    return true;
}

// Demo flush: every dirty cell of the active layer now, top row first, one polled burst per
// row (TX-only on F4), then HOS/VOS. Auto-increment runs per INV attribute; clean gaps of up
// to 3 cells inside a run are rewritten (2 B each) rather than restarting the run (8 B).
// The generic DrawScreen sends ≤ 12 B per spiSequence with a fresh DMM/DMAH/DMAL header each
// time — several ms for a scroller frame, i.e. into the visible field. Blocking. Returns the
// number of bytes sent.
uint16_t max7456FlushDirtyBurst(void)
{
    static DMA_DATA uint8_t buf[CHARS_PER_LINE * 10 + 8];
    if (fontIsLoading || !max7456DeviceDetected || !max7456WaitSpiIdle()) {
        return 0;
    }
    uint8_t *layer = getActiveLayerBuffer();
    const uint8_t dmmBase = (uint8_t)(displayMemoryModeReg & (uint8_t)~INVERT_PIXEL_COLOR);
    uint16_t total = 0;
    uint8_t hi = 0xFF;
    for (uint16_t rowAddr = 0; rowAddr < maxScreenSize; rowAddr += CHARS_PER_LINE) {
        uint16_t idx = 0;
        uint8_t c = 0;
        while (c < CHARS_PER_LINE) {
            const uint16_t a = (uint16_t)(rowAddr + c);
            if (layer[a] == shadowBuffer[a] && invertLayer[a] == shadowInvert[a]) {
                c++;
                continue;
            }
            const uint8_t inv = invertLayer[a];
            uint8_t end = c;
            for (uint8_t k = (uint8_t)(c + 1u); k < CHARS_PER_LINE; k++) {
                const uint16_t b = (uint16_t)(rowAddr + k);
                if (invertLayer[b] != inv || k - end > 3) {
                    break;
                }
                if (layer[b] != shadowBuffer[b] || invertLayer[b] != shadowInvert[b]) {
                    end = k;
                }
            }
            buf[idx++] = MAX7456ADD_DMM;
            buf[idx++] = (uint8_t)(dmmBase | (inv ? INVERT_PIXEL_COLOR : 0) | DMM_AUTO_INC);
            if (((a >> 8) & 1u) != hi) {
                hi = (uint8_t)((a >> 8) & 1u);
                buf[idx++] = MAX7456ADD_DMAH;
                buf[idx++] = hi;
            }
            buf[idx++] = MAX7456ADD_DMAL;
            buf[idx++] = (uint8_t)(a & 0xff);
            for (uint8_t k = c; k <= end; k++) {
                const uint16_t b = (uint16_t)(rowAddr + k);
                if (layer[b] == END_STRING) {
                    layer[b] = ' '; // 0xFF ends an auto-increment run
                }
                buf[idx++] = MAX7456ADD_DMDI;
                buf[idx++] = layer[b];
                shadowBuffer[b] = layer[b];
                shadowInvert[b] = invertLayer[b];
            }
            buf[idx++] = MAX7456ADD_DMDI;
            buf[idx++] = END_STRING;
            c = (uint8_t)(end + 1u);
        }
        if (idx) {
#if defined(STM32F4) || defined(STM32F7)
            if (!max7456SpiTxOnlyBurst(buf, idx))
#endif
            {
                busSegment_t segments[] = {
                    {.u.buffers = {buf, NULL}, idx, true, NULL},
                    {.u.link = {NULL, NULL}, 0, true, NULL},
                };
                spiSequence(dev, &segments[0]);
                spiWait(dev);
            }
            total = (uint16_t)(total + idx);
        }
    }
    spiWriteReg(dev, MAX7456ADD_DMM, displayMemoryModeReg);
    previousInvertRegister = displayMemoryModeReg;
    max7456DrawPos = 0;
    max7456ApplyHosVos();
    return total;
}

void max7456MidGlyphSpiBegin(void)
{
    (void)max7456WaitSpiIdle();
    midGlyphSavedDiv = max7456SpiClockDiv;
#ifdef USE_DMA
    midGlyphSavedDma = dev->useDMA;
    spiDmaEnable(dev, false);
#endif
    // Keep nominal SPI clock here — WaitVsync STAT polling breaks at 20 MHz on many boards.
    midGlyphSpiHot = true;
}

void max7456MidGlyphSpiEnd(void)
{
    if (!midGlyphSpiHot) {
        return;
    }
    midGlyphSpiHot = false;
    (void)max7456WaitSpiIdle();
    spiWriteReg(dev, MAX7456ADD_DMM, displayMemoryModeReg);
    previousInvertRegister = displayMemoryModeReg;
    spiSetClkDivisor(dev, midGlyphSavedDiv);
#ifdef USE_DMA
    spiDmaEnable(dev, midGlyphSavedDma);
#endif
}

// Boost only around Display-SRAM bursts (call after VSYNC, unboost before next VSYNC).
void max7456MidGlyphSpiBoost(bool enable)
{
    if (!midGlyphSpiHot) {
        return;
    }
    if (enable) {
        // 20 MHz for both AT and MAX — needed to finish a row rewrite inside a PAL line.
        spiSetClkDivisor(dev, spiCalculateDivider(MAX7456_MAX_SPI_CLK_HZ * 2));
        // The bus layer applies a divisor lazily on the next spiSequence. Latch it now with a
        // write that cannot touch the picture (~1 us) so the first hot burst after a STAT poll
        // takes the TX-only path instead of the 2× slower generic one. DMAH only feeds the
        // next display-memory address, and every hot burst rewrites it first — unlike HOS,
        // which re-latches the horizontal position mid-line.
        if (!max7456ActiveDma && !spiIsBusy(dev)) {
            spiWriteReg(dev, MAX7456ADD_DMAH, 0);
        }
    } else {
        spiSetClkDivisor(dev, midGlyphSavedDiv);
    }
}

// Background of transparent pixels, written now (mid-glyph field hooks, in VBLANK):
// gray=false → video, gray=true → grey at level 0..7 (0..49 %). The configured background
// type is not changed; the next SetBackgroundType / re-init restores it.
static uint8_t max7456BackgroundReg(bool gray, uint8_t level)
{
    // Video keeps the exact VM1 SetRegisterVM1 uses for TRANSPARENT (device default level
    // 28 %): VM1 = 0x07 (level 0, gray off) showed black background on AT7456.
    uint8_t reg = (uint8_t)(BLINK_TIME_1 | BLINK_DUTY_CYCLE_75_25
                            | ((gray ? (level & 7u) : BACKGROUND_BRIGHTNESS_28) << 4));
    if (gray) {
        reg |= BACKGROUND_MODE_GRAY;
    }
    return reg;
}

void max7456WriteBackgroundNow(bool gray, uint8_t level)
{
    spiWriteReg(dev, MAX7456ADD_VM1, max7456BackgroundReg(gray, level));
}

// VM1 read back against (gray, level) and written again when it differs (polled, like
// WriteBackgroundNow). Returns the value read when it had to be fixed, −1 when it was right.
int16_t max7456BackgroundFix(bool gray, uint8_t level)
{
    if (!max7456DeviceDetected || fontIsLoading) {
        return -1;
    }
    if (!midGlyphSpiHot && !max7456WaitSpiIdle()) {
        return -1; // the task side: not with a draw transfer under way
    }
    const uint8_t want = max7456BackgroundReg(gray, level);
    const uint8_t got = spiReadRegMsk(dev, MAX7456ADD_VM1);
    if (got == want) {
        return -1;
    }
    spiWriteReg(dev, MAX7456ADD_VM1, want);
    return got;
}

void max7456WriteHosNow(uint8_t hos)
{
    if (hos > 63) {
        hos = 63;
    }
    if (!midGlyphSpiHot) {
        if (!max7456WaitSpiIdle()) {
            return;
        }
    } else if (spiIsBusy(dev)) {
        spiWait(dev);
    }
    // Keep hosRegValue in sync so a later ApplyHosVos cannot snap by ~1 cell.
    hosRegValue = hos;
    previousHosRegister = hos;
    spiWriteReg(dev, MAX7456ADD_HOS, hos);
}

// Mid-glyph hot path: vertical shift relative to the configured base VOS, now (+ = down).
// Returns the offset actually applied after clamping to the VOS range, so a raster engine
// can move its line schedule by exactly that many lines.
int8_t max7456WriteVosOffsetNow(int8_t offsetPx)
{
    const int vos = constrain((int)vosRegValue + (int)offsetPx, 0, 31);
    if (!midGlyphSpiHot) {
        if (!max7456WaitSpiIdle()) {
            return (int8_t)(vos - (int)vosRegValue);
        }
    } else if (spiIsBusy(dev)) {
        spiWait(dev);
    }
    if (vos != previousVosRegister) {
        previousVosRegister = (uint8_t)vos;
        spiWriteReg(dev, MAX7456ADD_VOS, (uint8_t)vos);
    }
    return (int8_t)(vos - (int)vosRegValue);
}

void max7456WriteHosSigned(int8_t offsetPx)
{
    // Register center 32 == neutral. Clamp BEFORE encode — never wrap 0↔63.
    int hos = 32 + (int)offsetPx;
    if (hos < 0) {
        hos = 0;
    }
    if (hos > 63) {
        hos = 63;
    }
    max7456WriteHosNow((uint8_t)hos);
}

static void max7456CommitShadowCell(uint16_t addr, uint8_t glyph, bool invert)
{
    if (addr >= maxScreenSize) {
        return;
    }
    getActiveLayerBuffer()[addr] = glyph;
    invertLayer[addr] = invert ? 1 : 0;
    shadowBuffer[addr] = glyph;
    shadowInvert[addr] = invert ? 1 : 0;
}

bool max7456WaitVsyncFallingEdge(uint32_t *edgeTicks, timeUs_t timeoutUs)
{
    if (!max7456WaitSpiIdle()) {
        return false;
    }

    // DWT, not micros(): these run inside the scene-10 TIM5 interrupt, where SysTick cannot
    // fire — micros() then wraps back every 1 ms and a >1 ms timeout never expired (hang).
    const uint32_t t0 = getCycleCounter();
    const uint32_t timeoutTicks = clockMicrosToCycles(timeoutUs);
    uint8_t prev = spiReadRegMsk(dev, MAX7456ADD_STAT);

    // Ensure we start from VSYNC high so the next 1→0 is a real edge.
    while (!STAT_IS_VSYNC_HIGH(prev)) {
        if (getCycleCounter() - t0 > timeoutTicks) {
            return false;
        }
        prev = spiReadRegMsk(dev, MAX7456ADD_STAT);
    }

    for (;;) {
        const uint8_t s = spiReadRegMsk(dev, MAX7456ADD_STAT);
        if (STAT_IS_VSYNC_HIGH(prev) && !STAT_IS_VSYNC_HIGH(s)) {
            // Falling edge of STAT[4] ≈ start of VSYNC (active-low ~VSYNC).
            if (edgeTicks) {
                *edgeTicks = getCycleCounter();
            }
            return true;
        }
        prev = s;
        if (getCycleCounter() - t0 > timeoutTicks) {
            return false;
        }
    }
}

bool max7456WaitHsyncFallingEdge(uint32_t *edgeTicks, timeUs_t timeoutUs)
{
    if (!max7456WaitSpiIdle()) {
        return false;
    }

    // DWT, not micros(): these run inside the scene-10 TIM5 interrupt, where SysTick cannot
    // fire — micros() then wraps back every 1 ms and a >1 ms timeout never expired (hang).
    const uint32_t t0 = getCycleCounter();
    const uint32_t timeoutTicks = clockMicrosToCycles(timeoutUs);
    uint8_t prev = spiReadRegMsk(dev, MAX7456ADD_STAT);

    // Start from HSYNC high so the next 1→0 is a real line edge.
    while (!STAT_IS_HSYNC_HIGH(prev)) {
        if (getCycleCounter() - t0 > timeoutTicks) {
            return false;
        }
        prev = spiReadRegMsk(dev, MAX7456ADD_STAT);
    }

    for (;;) {
        const uint8_t s = spiReadRegMsk(dev, MAX7456ADD_STAT);
        if (STAT_IS_HSYNC_HIGH(prev) && !STAT_IS_HSYNC_HIGH(s)) {
            if (edgeTicks) {
                *edgeTicks = getCycleCounter();
            }
            return true;
        }
        prev = s;
        if (getCycleCounter() - t0 > timeoutTicks) {
            return false;
        }
    }
}

#endif // USE_MAX7456
