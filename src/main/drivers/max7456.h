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

#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "common/time.h"
#include "drivers/display.h"

/** PAL or NTSC, value is number of chars total */
#define VIDEO_BUFFER_CHARS_NTSC   390
#define VIDEO_BUFFER_CHARS_PAL    480

typedef enum {
    // IO defined and MAX7456 was detected
    MAX7456_INIT_OK = 0,
    // IO defined, but MAX7456 could not be detected (maybe not yet
    // powered on)
    MAX7456_INIT_NOT_FOUND = -1,
    // No MAX7456 IO defined, which means either the we don't have it or
    // it's not properly configured
    MAX7456_INIT_NOT_CONFIGURED = -2,
} max7456InitStatus_e;

extern uint16_t maxScreenSize;
struct vcdProfile_s;
void    max7456HardwareReset(void);
struct max7456Config_s;
void    max7456Preinit(const struct max7456Config_s *max7456Config);
max7456InitStatus_e max7456Init(const struct max7456Config_s *max7456Config, const struct vcdProfile_s *vcdProfile, bool cpuOverclock);
void    max7456Invert(bool invert);
void    max7456Brightness(uint8_t black, uint8_t white);
// Per-row brightness via RB0..RB15 (black/white each 0..3, 0 = darkest).
void    max7456BrightnessRow(uint8_t row, uint8_t black, uint8_t white);
#define MAX7456_REG_RB0 0x10
// OSD demo zones by DMA (F4): 16-bit SPI over the active field in one CS frame; bursts go out
// by SPI2_TX DMA, paused around the zone words by the zone timer.
void    max7456Zone16Begin(void);
void    max7456Zone16End(void);
bool    max7456Zone16Active(void);
volatile void *max7456SpiDataRegister(void);
uint8_t max7456DmmBase(void);
uint32_t max7456SessionWordTicks(void);
// OSDM (0x0C): bits[5:3]=rise/fall, bits[2:0]=mux switch; each 0=sharpest .. 5=softest. Default 0x1B.
void    max7456Osdm(uint8_t value);
bool    max7456ReInitIfRequired(bool forceStallCheck);
bool     max7456DrawScreen(void);
bool    max7456WriteNvm(uint8_t char_address, const uint8_t *font_data);
uint8_t max7456GetRowsCount(void);
uint8_t max7456GetHosApplied(void); // HOS in the chip (32 = centre)
void    max7456Write(uint8_t x, uint8_t y, const char *text);
void    max7456WriteChar(uint8_t x, uint8_t y, uint8_t c);
// Per-character INV attribute (MAX7456 DMM[3] copied in 16-bit display-memory writes).
void    max7456WriteCharEx(uint8_t x, uint8_t y, uint8_t c, bool invert);
void    max7456ClearScreen(void);
void    max7456RefreshAll(void);
bool    max7456DmaInProgress(void);
bool    max7456BuffersSynced(void);
// True when the previous screen pass finished and SPI is idle (safe to rewrite the layer buffer).
bool    max7456LayerSupported(displayPortLayer_e layer);
bool    max7456LayerSelect(displayPortLayer_e layer);
bool    max7456LayerCopy(displayPortLayer_e destLayer, displayPortLayer_e sourceLayer);
bool    max7456IsDeviceDetected(void);
void    max7456SetBackgroundType(displayPortBackground_e backgroundType);
// Visual screen-space offset in MAX7456 pixel steps: +x right, +y down. Combined with vcd base HOS/VOS.
void    max7456SetHudMotionOffset(int8_t x, int8_t y);
void    max7456ResetHudMotionOffset(void);
// Push pending HOS/VOS to the chip now (SPI must be idle).
void    max7456ApplyHudMotionNow(void);
// Absolute HOS register poke (0..63). Display timing only — no Display SRAM / NVM.
// Used by mid-scanline HOS realtime probes; updates the driver's HOS shadow.
void    max7456WriteHosNow(uint8_t hos);
// Signed pixel offset relative to register center 32. Clamps before encode — no wrap.
// Accepts -32..+31 (+ = right on screen). Scene code must not invent "32 + x" itself.
void    max7456WriteHosSigned(int8_t offsetPx);
// Available HUD offset before HOS/VOS clamps ( +X = right, +Y = down on screen ).
void    max7456GetHudMotionXLimits(int8_t *minX, int8_t *maxX);
void    max7456GetHudMotionYLimits(int8_t *minY, int8_t *maxY);
void    max7456FillScreen(uint8_t c);
void    max7456CopyLayer(uint8_t *codes, uint8_t *inv, uint16_t n); // the screen buffer (codes, INV)
void    max7456Invalidate(void); // force full dirty screen (FX switches)
// Call after max7456WriteNvm(): clears fontIsLoading and re-enables the OSD.
void    max7456EndFontWrite(void);

// Experimental: blocking 8-bit Display SRAM poke (character index only).
// Does not touch NVM/font. Safe vs DMA only when SPI is idle (caller must wait).
// Uses DMM 8-bit mode so the cell's attribute byte is not rewritten from DMM[5:3].
// Experimental: blocking 8-bit Display SRAM poke of per-cell attribute byte.
// attr bit7=LBC, bit6=BLK, bit5=INV, bit4=CA[8] (AT7456 upper glyph; it only takes effect when
// the character is written in 8-bit mode after it — a lone attribute write drops it). Verified on
// an AT7456E: INV = 0x20, CA[8] = 0x10.
// Experimental: char + attr in one 8-bit DMM session (lower SPI cost).
// Burst same glyph into `count` consecutive cells (8-bit auto-increment).
// Raster hot path: same as RowFill but skips shadow walk (SPI assumed idle).
bool    max7456WriteDisplaySramRowFillEx(uint16_t addr, uint8_t glyph, uint8_t count, bool commitShadow);
// Mid-glyph 2×2 plasma: per-cell glyphs along one character row (AI burst).
// Same, with optional per-cell INV (NULL = all non-inverted). Groups runs by INV for 16-bit DMM copy.
// One SPI burst: signed HOS (center=32) then row glyphs. Avoids a second transaction in the
// mid-scanline tear window (twister HOS bend + silhouette rewrite).
// Mid-glyph hot path: one row burst built from a segment plan (hot mode only).
typedef struct max7456SramSeg_s {
    uint8_t col;      // first column in the row
    uint8_t len;      // cells
    bool autoInc;     // true: one auto-increment run; false: addressed write per cell
} max7456SramSeg_t;
uint16_t max7456EncodeDisplaySramRowAttr(uint16_t rowAddr, const uint8_t *glyphs, uint32_t invMask,
                                         const max7456SramSeg_t *seg, uint8_t nSeg,
                                         uint16_t *segFirstByte, uint16_t *segLastByte);
bool    max7456SendEncodedDisplaySram(uint16_t len);
// Lock SPI for a mid-glyph field (20 MHz polled on AT, sticky DMM). Call End after the field.
// Mid-glyph hot path: VOS = base + offset now (+ = down); returns the clamped offset applied.
int8_t  max7456WriteVosOffsetNow(int8_t offsetPx);
void    max7456MidGlyphSpiBegin(void);
void    max7456MidGlyphSpiEnd(void);
void    max7456MidGlyphSpiBoost(bool enable);
bool    max7456NvmMatches(uint8_t char_address, const uint8_t *font_data);
void    max7456RbResetWanted(uint8_t reg);                       // OSD demo: row levels at a scene start
void    max7456RbHold(bool hold, uint8_t flashReg);
bool    max7456RbHeld(void);
void    max7456SetSyncMode(uint8_t mode);                       // 0 auto, 1 internal, 2 external
bool    max7456HasVideoIn(void);                                // the chip sees a camera (no loss of sync)
bool    max7456IsAt7456(void);                                  // 512 glyphs (upper 256 are RAM)
// Upper-bank test suite (`osd_demo bank ...`): prints `bank <test>: PASS|FAIL|INFO ...` lines.
typedef void (*max7456PrintFn)(const char *fmt, ...);
bool    max7456NvmRead9(uint16_t addr, uint8_t out[54]);         // AT7456: glyphs 0..511
bool    max7456NvmWrite9(uint16_t addr, const uint8_t data[54]);
void    max7456WriteBackgroundNow(bool gray, uint8_t level);
int16_t max7456BackgroundFix(bool gray, uint8_t level);
uint16_t max7456FlushDirtyBurst(void); // demo: all dirty cells + HOS/VOS now, row order
// Keep layer+shadow in sync for one cell so the normal draw pass will not fight a test.

// STAT[4]=VSYNC level, STAT[3]=HSYNC level (SPI; no pin wiring).
// Poll STAT until bit4 falls 1→0 (start of VSYNC). *edgeTicks ≈ DWT at detect.
bool    max7456WaitVsyncFallingEdge(uint32_t *edgeTicks, timeUs_t timeoutUs);
// Poll STAT until bit3 falls 1→0 (start of HSYNC / line).
bool    max7456WaitHsyncFallingEdge(uint32_t *edgeTicks, timeUs_t timeoutUs);
// Count `count` HSYNC falling edges (for line-accurate mid-glyph scheduling).
