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
 *
 * MAX7456 demoscene — uses the dedicated osd_demo font
 * (tmp/osd_demo.mcm / osd_demo_glyphs.inc from generate_osd_cellfont.py).
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#ifdef USE_OSD_DEMO

#include "common/maths.h"
#include "common/printf.h"
#include "common/time.h"
#include "common/utils.h"

#include "drivers/display.h"
#include "drivers/max7456.h"
#include "drivers/motor.h"
#include "drivers/system.h"
#include "drivers/time.h"
#include "drivers/timer.h"
#include "drivers/nvic.h"
#include "drivers/io.h"
#include "drivers/serial_resource.h"
#include "drivers/ay_pwm.h"

#include "osd/osd.h"
#include "drivers/osd_symbols.h"
#include "flight/imu.h"

#include "config/config.h"
#include "config/feature.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#ifdef USE_I2C
#include "pg/bus_i2c.h"
#endif
#include "pg/displayport_profiles.h"
#include "pg/max7456.h"
#include "drivers/serial.h"
#include "pg/vcd.h"

#include "io/osd_demo.h"
#include "io/displayport_max7456.h"
#include "io/ledstrip.h"

#ifdef USE_CHIPTUNE
#include "io/chiptune.h"
#endif


#ifdef USE_CLI
#include "cli/cli.h"
#endif

#include "osd/osd_demo_font.inc"
#include "osd/osd_demo_glyphs.inc"

#define OSD_DEMO_SCROLL_HZ          72  // fine px steps
#define OSD_DEMO_BOUNCE_PERIOD_MS   900
#define OSD_DEMO_CELL_W             12
#define OSD_DEMO_CELL_H             18
#define OSD_DEMO_HOS_SOFT_PX        6   // soft-scroll amplitude (halfway to a full cell)
#define OSD_DEMO_HOP_CELLS          3

#define OSD_DEMO_PUNCH_KICK         96
#define OSD_DEMO_PUNCH_SPRING       5
#define OSD_DEMO_PUNCH_DAMP_NUM     14
#define OSD_DEMO_PUNCH_DAMP_DEN     16
#define OSD_DEMO_PUNCH_MAX_PX       12

#define OSD_DEMO_SHIMMER_PERIOD_MS  1100
#define OSD_DEMO_STAR_BRIGHT_MS     90   // per-step for starfield row white wave
#define OSD_DEMO_RB_ROWS            16

#define OSD_DEMO_STAR_LAYERS        3
#define OSD_DEMO_STARS_PER_LAYER    12
#define OSD_DEMO_PLASMA_STAR_ROWS   3   // starfield band above/below plasma

#define OSD_DEMO_PLASMA_T_HZ        50  // scene 2 plasma phase steps / s
// Wall-clock effect durations (Craft plasma block ~8s).
#define OSD_DEMO_PLASMA_FADE_MS     1200
// Scene lengths in song patterns (64 rows; Summer Steps 3.84 s each). Every scene ends on a
// pattern boundary of the music and its start in the timeline is the sum of the scenes before
// it in the order. Without music a pattern is 64 × 60 ms.
#define OSD_DEMO_PAT_SCROLLER    8    // intro 2 + star ramp 2 + text with the ridge 4
#define OSD_DEMO_PAT_PLASMA      4
#define OSD_DEMO_PAT_FIRE        4
#define OSD_DEMO_PAT_WIPE        4
#define OSD_DEMO_PAT_TUNNEL      4
#define OSD_DEMO_PAT_PLASMA2X2   4
#define OSD_DEMO_PAT_TWISTER     4
#define OSD_DEMO_PAT_SHOUT       32
#define OSD_DEMO_PAT_DONUT       4
#define OSD_DEMO_PAT_MEGA        1
#define OSD_DEMO_PAT_ROTO        4
#define OSD_DEMO_PAT_CUBE        4
#define OSD_DEMO_PAT_BITMAP      3
#define OSD_DEMO_PAT_DISARM      2
#define OSD_DEMO_PATTERN_ROWS    64
#define OSD_DEMO_FX_ROW_GRACE_MS 4000 // clock lost: fall back to the ms length
// Scene 3 on the beat (tracker lines nearest FIRE_BEAT_MS per beat, counted from the scene's
// first beat): the plasma collapses over COLLAPSE beats, the fire bursts up over IGNITE beats
// (physics every tick) and burns at its own pace from the following beat — a downbeat.
#define OSD_DEMO_FIRE_BEAT_MS       500
#define OSD_DEMO_FIRE_COLLAPSE_BEATS 1
#define OSD_DEMO_FIRE_IGNITE_BEATS  1
#define OSD_DEMO_FIRE_IGNITE_STEP_TICKS 1
#define OSD_DEMO_FIRE_STEP_TICKS    3   // flame physics ~24Hz @ SCROLL_HZ (was every tick)
#define OSD_DEMO_FIRE_COLS          30
#define OSD_DEMO_FIRE_ROWS          16
#define OSD_DEMO_FIRE_SPARKS        4
#define OSD_DEMO_FIRE_CORE_HALF     5

#define OSD_DEMO_CHARS_PER_LINE         30

typedef enum {
    OSD_DEMO_FX_SCROLLER = 0, // scene 1 — bounce scroller (scene 0 is the intro, not an effect)
    OSD_DEMO_FX_PLASMA,     // scene 2 — dither-ladder plasma
    OSD_DEMO_FX_FIRE,       // scene 3 — fire
    OSD_DEMO_FX_WIPE,       // scene 4 — wipe and the music analyser
    OSD_DEMO_FX_TUNNEL,     // scene 5 — tunnel
    OSD_DEMO_FX_DONUT,      // scene 6 — donut tunnel (rings of dashes, 2×2 raster)
    OSD_DEMO_FX_PLASMA2X2,  // scene 7 — 2×2 mid-glyph plasma
    OSD_DEMO_FX_TWISTER,    // scene 8 — classic B/W ribbon twister
    OSD_DEMO_FX_MEGA,       // scene 9 — megacolor: 4-level pictures on the 2×2 raster
    OSD_DEMO_FX_SHOUT,      // scene 10 — FPV community shoutouts (3×3-px raster text)
    OSD_DEMO_FX_ROTO,       // scene 11 — rotozoom: scene 9's sprite tiled over grey
    OSD_DEMO_FX_CUBE,       // scene 12 — rotating 3D cube on the bitmap engine
    OSD_DEMO_FX_BITMAP,     // scene 13 — 2 px × 1 line bitmap plasma, one grey pixel a cell line
    OSD_DEMO_FX_DISARM,     // scene 14 — the end: *** DISARM *** blinking, no music
    OSD_DEMO_FX_COUNT,
} osdDemoFx_e;

static uint8_t fxPatterns[OSD_DEMO_FX_COUNT] = {
    [OSD_DEMO_FX_SCROLLER] = OSD_DEMO_PAT_SCROLLER, [OSD_DEMO_FX_PLASMA] = OSD_DEMO_PAT_PLASMA,
    [OSD_DEMO_FX_FIRE] = OSD_DEMO_PAT_FIRE,         [OSD_DEMO_FX_WIPE] = OSD_DEMO_PAT_WIPE,
    [OSD_DEMO_FX_TUNNEL] = OSD_DEMO_PAT_TUNNEL,     [OSD_DEMO_FX_PLASMA2X2] = OSD_DEMO_PAT_PLASMA2X2,
    [OSD_DEMO_FX_TWISTER] = OSD_DEMO_PAT_TWISTER,   [OSD_DEMO_FX_SHOUT] = OSD_DEMO_PAT_SHOUT,
    [OSD_DEMO_FX_DONUT] = OSD_DEMO_PAT_DONUT,     [OSD_DEMO_FX_MEGA] = OSD_DEMO_PAT_MEGA,
    [OSD_DEMO_FX_ROTO] = OSD_DEMO_PAT_ROTO, [OSD_DEMO_FX_CUBE] = OSD_DEMO_PAT_CUBE,
    [OSD_DEMO_FX_BITMAP] = OSD_DEMO_PAT_BITMAP, [OSD_DEMO_FX_DISARM] = OSD_DEMO_PAT_DISARM,
};
// Scene order of the demo (scene numbers).
static const uint8_t demoOrder[] = { 1, 3, 2, 4, 5, 8, 6, 7, 11, 12, 13, 9, 10, 14 };
#define demoOrderLen ((uint8_t)ARRAYLEN(demoOrder))
static uint8_t fxOrderIdx;       // position of the running scene in demoOrder
static uint32_t s1BaseRowQ8;     // song row where scene 1's own timeline (intro) starts

// CLI scene number → effect (-1 = none): the scenes numbered in the demo's order, 0 the intro.
static const int8_t osdDemoSceneFx[OSD_DEMO_SCENE_COUNT] = {
    -1, OSD_DEMO_FX_SCROLLER, OSD_DEMO_FX_PLASMA, OSD_DEMO_FX_FIRE, OSD_DEMO_FX_WIPE,
    OSD_DEMO_FX_TUNNEL, OSD_DEMO_FX_DONUT, OSD_DEMO_FX_PLASMA2X2, OSD_DEMO_FX_TWISTER,
    OSD_DEMO_FX_MEGA, OSD_DEMO_FX_SHOUT, OSD_DEMO_FX_ROTO, OSD_DEMO_FX_CUBE,
    OSD_DEMO_FX_BITMAP, OSD_DEMO_FX_DISARM,
};

typedef enum {
    OSD_DEMO_FIRE_COLLAPSE = 0,
    OSD_DEMO_FIRE_IGNITE,
    OSD_DEMO_FIRE_BURN,
} osdDemoFirePhase_e;

typedef struct {
    uint8_t x;
    uint8_t y;
    uint8_t size;
} osdDemoStar_t;

static bool active;
static displayPort_t *demoDisplay;
static const char *startLastError;
static uint16_t scrollCol;       // whole-cell text offset
static uint8_t scrollFine;       // 0..HOS_SOFT_PX-1 hardware HOS phase
static uint16_t textPixelCols;
static timeUs_t lastStepUs;
static timeMs_t bounceStartMs;
static timeMs_t fxStartMs;
static bool fxEndByRow;          // the scene ends at fxEndRowQ8 of the song
static uint32_t fxEndRowQ8;
// Started by scene name from the CLI: stay on this scene (no auto-cycle). Mid-glyph scenes
// (7, 8) just keep running; time-phased scenes restart themselves every period.
static bool fxHold;
static bool demoStartedChiptune; // stop the music on osd_demo stop only if the demo started it
static osdDemoFx_e fx;
static uint16_t prevBouncePhase;
static bool punchArmed;

static int16_t punchY;
static int16_t punchVel;
static uint8_t prevRowBright[OSD_DEMO_RB_ROWS];

static osdDemoStar_t stars[OSD_DEMO_STAR_LAYERS][OSD_DEMO_STARS_PER_LAYER];
static uint16_t starFine[OSD_DEMO_STAR_LAYERS]; // absolute leftward px; synced to HOS

// PAL timing shared by the mid-glyph engines (scene 7 / scene 8).
#define OSD_DEMO_PAL_VSYNC_TIMEOUT_US  30000
#define OSD_DEMO_PAL_LINE_US           64    // PAL line ≈ 64 us
// VSYNC→row0 top (calibrated: Y=8, line=64 → 1504 + 8*18*64 = 10720 us).
// PAL field = 312.5 lines.
#define OSD_DEMO_PAL_FIELD_LINES_X2    625

static uint8_t hosSteps; // 1..HOS_SOFT_PX, right-bias soft-scroll range
static uint8_t hosStepPx = 1; // HOS px per soft step (2: 6 steps cover a 12-px cell)
// Star x phase wraps on a whole screen of cells, so rel % CELL_W and the cell shift stay
// continuous (a uint16 wrap at 65536 is not a multiple of 12).
#define OSD_DEMO_STAR_WRAP (OSD_DEMO_CELL_W * OSD_DEMO_CHARS_PER_LINE)

// Scene 1 timeline on the song (rows of 64-row patterns; Summer Steps: 3.84 s per pattern).
//  rows   0..127 (pos 0-1, lead solo): stars appear one by one, static, twinkling
//  rows 128..255 (pos 2-3, bass):      stars accelerate to their normal speed
//  rows 256..511 (pos 4-7, drums):     the text enters from the right while dropping from the
//                                      top of the bounce — first floor hit on the first kick,
//                                      the VOS punch starts there; the brightness ridge sweeps
//                                      over it from the start (a flat-brightness part made the
//                                      scene too long)
//  row  512      (pos 8):              next scene
// Without music the same rows run on a nominal 60 ms clock (speed 3 × 20 ms).
#define OSD_DEMO_S1_STARS_ROWS   120  // all intro stars shown by this row
#define OSD_DEMO_S1_BASS_ROW     128
#define OSD_DEMO_S1_SCROLL_ROW   256
#define OSD_DEMO_S1_SHIMMER_ROW  256
#define OSD_DEMO_S1_ROW_MS       60
#define OSD_DEMO_S1_HOS_STEPS    4    // soft-scroll steps per 12-px cell (3 px each)
static bool s1Timeline;               // first scene after `osd_demo`: run the song timeline
static timeMs_t s1TimelineMs;         // no-music clock start
static bool s1TextOn = true;          // text, bounce and punch live
static bool s1TextArmed = true;       // show the text at the next bounce peak
static bool s1Shimmer = true;         // brightness ridge on the text rows
static uint8_t s1StarsShown = 255;    // intro: stars visible so far (in appearance order)
static uint8_t s1LeadIn;              // blank cells before the text: it enters from the right
static uint16_t s1StarScale = 256;    // star speed ×/256
static uint8_t starAcc[OSD_DEMO_STAR_LAYERS]; // sub-pixel star motion while accelerating
static uint32_t s1PosQ8;              // timeline position (rows, Q8)

// Intro titles in 6×6-px mega-pixels (2 × 3 per cell, 32 mask glyphs at MP_BASE + the cell
// INV attribute: mask m with bit 5 set is glyph(63 − m) inverted — B/W only, so exact): one
// title per pattern (rows 0..63, 64..127), two centred 5-mp lines with a 2-mp gap = 4 char
// rows. 60 mega-pixels per line → ≤ 10 characters, so "OSD MEGADEMO" takes two lines.
// Fade = blinking on an easing curve: in, fast blinks slowing down until the title stays; out,
// slow blinks speeding up until it is gone (was a pixel dissolve). The third title, the music
// credit, runs while the stars accelerate (the bass, rows 128..255).
#define OSD_DEMO_S1_TITLE_ROW0   6
#define OSD_DEMO_S1_TITLE_SHIFT  1    // mega-pixels right of the centred layout (on the glass)
#define OSD_DEMO_S1_TITLES       3
#define OSD_DEMO_S1_FADE_IN0     2    // of 64 rows (a longer title stretches them)
#define OSD_DEMO_S1_FADE_IN1     14
#define OSD_DEMO_S1_FADE_OUT0    50
#define OSD_DEMO_S1_FADE_OUT1    62
#define OSD_DEMO_S1_BLINKS       9    // visibility toggles over a fade (odd: in ends shown, out hidden)
static const char *const s1Titles[OSD_DEMO_S1_TITLES][2] = {
    { "X.COM/350D", "PRESENTS" },
    { "OSD", "MEGADEMO" },
    { "MUSIC BY", "E.J.KOT" },
};
static const uint8_t s1TitleRow0[OSD_DEMO_S1_TITLES] = { 0, 64, 128 };  // song rows
static const uint8_t s1TitleRows[OSD_DEMO_S1_TITLES] = { 64, 64, 128 };
// White of each title line (row brightness: 0 = 80 % .. 3 = 120 %), the blink's ramp capped at it
static const uint8_t s1TitleLineWhite[OSD_DEMO_S1_TITLES][2] = { { 3, 0 }, { 3, 3 }, { 0, 3 } };
static uint32_t rngState;
static uint8_t plasmaT; // Craft framecount>>1 analogue
static uint8_t tunnelRot;  // angle scroll

#define OSD_DEMO_TUNNEL_STATIC_MS  2500  // look forward before first look-back
#define OSD_DEMO_TUNNEL_FWD_MS     4000  // time facing forward
#define OSD_DEMO_TUNNEL_TURN_MS    4000  // 180° Y-rotation (Craft scene angle)
#define OSD_DEMO_TUNNEL_BACK_MS    2000  // time facing back
#define OSD_DEMO_TUNNEL_SWAY_AMP   10    // ±yaw during straight holds (~14°)
#define OSD_DEMO_TUNNEL_SWAY_MS    2200  // full left-right sway period
#define OSD_DEMO_TUNNEL_SWAY_FADE  500   // fade sway out before/after turns
#define OSD_DEMO_TUNNEL_Z         32     // Craft ray z (3.5 fixed forward)
#define OSD_DEMO_TUNNEL_HOLE2     81     // Craft r² black-centre threshold
#define OSD_DEMO_TUNNEL_ANIM_DIV  4      // depth fly (2=fast, 8=slow → mid)
#define OSD_DEMO_TUNNEL_SPIN_DIV  3      // wedge spin (1=fast, 6=slow → mid)
#define OSD_DEMO_TUNNEL_TEXT_DIV  4      // text mega-pixel scroll (keep)
#define OSD_DEMO_TUNNEL_UV_SHIFT  4      // Craft bit4 — same on U and V (even wedges)
#define OSD_DEMO_TUNNEL_DEP_MUL   2      // ×V before shift → depth rings half as long
#define OSD_DEMO_TUNNEL_TEXT_ROWS 3      // center band for 5×5 @ 6×6 mega-pixels
#define OSD_DEMO_TUNNEL_INK_Y0    1      // Visitor 5×5 ink starts at font row 1

// Scene 7 — mid-glyph 2×2 plasma (6 cols × 9 bands per cell) via PAL VSYNC/DWT.
// 2×2 megapixels: rewrite every 2 video lines (half the SPI of full line rate).
#define OSD_DEMO_CHECKER_COLS     30
#define OSD_DEMO_CHECKER_ROWS     16
#define OSD_DEMO_CHECKER_CELL_H   18
#define OSD_DEMO_CHECKER_CELL_W2  6
#define OSD_DEMO_PLASMA2X2_STEP   2   // video lines per megapixel (DIAG-style)
#define OSD_DEMO_PLASMA2X2_BANDS  (OSD_DEMO_CHECKER_CELL_H / OSD_DEMO_PLASMA2X2_STEP) // 9
// Odd-line tear window: full 30-cell row @20 MHz ≈ 28–36 us. Budget must
// cover overhead so the burst FINISHES before the lit even line (due).
// Build the next band only if at least this long remains before the HSYNC of line due−1.
// Scene 10 — FPV shoutouts on the raster engine: 3×3-px logical pixels.
// Scene 6 — scene 7's plasma on the TIM5 interrupt engine.
#define OSD_DEMO_IRQ_NOMINAL_FIELD_US 20000
#define OSD_DEMO_IRQ_VSYNC_LEAD_US  300   // open the STAT window this early before the prediction
#define OSD_DEMO_IRQ_VSYNC_WINDOW_US 1200 // past the predicted edge before giving up (dead-reckon)
#define OSD_DEMO_IRQ_VSYNC_LEAD_MAX_US 4000
#define OSD_DEMO_SHOUT_BAND_LINES   3
#define OSD_DEMO_SHOUT_BANDS        (OSD_DEMO_CELL_H / OSD_DEMO_SHOUT_BAND_LINES) // 6
#define OSD_DEMO_SHOUT_LP_PER_CELL  (OSD_DEMO_CELL_W / 3)                         // 4
#define OSD_DEMO_SHOUT_LP_COLS      (OSD_DEMO_CHARS_PER_LINE * OSD_DEMO_SHOUT_LP_PER_CELL) // 120
#define OSD_DEMO_SHOUT_LP_ROWS      (16 * OSD_DEMO_SHOUT_BANDS)                   // 96
#define OSD_DEMO_SHOUT_FONT_ROWS    5    // 5×5 ink → bands 0..4, band 5 = 3-line gap
#define OSD_DEMO_SHOUT_GREY_LEVEL   5    // VM1 grey of the fading names, stars and bars: 35 % (darker than 49 %)
#define OSD_DEMO_SHOUT_FADE_MS      1500 // after LIFE_BEATS a name blinks grey / black this long, then is gone
#define OSD_DEMO_SHOUT_BLINKS       15   // its toggles, faster and faster (odd: it ends hidden)
#define OSD_DEMO_SHOUT_LIFE_BEATS   6    // quarter notes before a name starts to blink out
#define OSD_DEMO_SHOUT_TRIES        48
#define OSD_DEMO_SHOUT_MAX_EVICT    4    // oldest names dropped per beat to make room
#define OSD_DEMO_SHOUT_ENV_LEVEL    13   // envelope-mode channels count as this volume
#define OSD_DEMO_SHOUT_BAR_DECAY_Q8 256  // bar fall per frame (px << 8): full bar in ~0.5 s
#define OSD_DEMO_SHOUT_MAX_WORDS    64
#define OSD_DEMO_SHOUT_FALLBACK_BEAT_MS 500 // 120 BPM when no chiptune is playing
// Classic plasma wave coeffs — wide contour bands on 180×144 2×2 grid.
#define OSD_DEMO_PLASMA2X2_KX     3
#define OSD_DEMO_PLASMA2X2_KY     2
#define OSD_DEMO_PLASMA2X2_KD     2
#define OSD_DEMO_PLASMA_ZEBRA_BANDS  8


static uint8_t tunnelAnim; // Craft framecount>>1 analogue
static uint8_t tunnelAnimDiv;
static uint8_t tunnelSpinDiv;
static uint8_t tunnelTextDiv;
static uint16_t tunnelScrollMx; // text offset in mega-pixels (1 font px = 1 MP)
static uint16_t tunnelTextCols; // len(tunnel text) * FONT_ADVANCE
static bool hosWrappedThisStep; // scrollFine wrap → defer HOS until SPI pass done

static uint8_t plasmaPhase;    // uint8 sine phase — wrap 255→0 is seamless
static bool plasma2x2Armed; // mid-glyph PAL engine for scene 7

// Scene 8 — classic vertical twister: rotating square cross-section → 4 projected edges.
// Mid-glyph rewrite: 9 vertical logical px/cell (12×2). Bend is geometric (centerX), not
// mid-field HOS. Always rewrite the full silhouette window — trimming the burst leaves
// stale wide glyphs on the right and reads as torn corners at the end of each band.
#define OSD_DEMO_TWISTER_AMP_PX         56
#define OSD_DEMO_TWISTER_LINE_SPAN      288
#define OSD_DEMO_TWISTER_TWIST_NUM      256
#define OSD_DEMO_TWISTER_ROT_STEP       1
// Geometric sway = two fixed-wavelength sines drifting at different speeds (A + B ≤ AMP).
#define OSD_DEMO_TWISTER_BEND_AMP       12   // total sway px (±), sizes the SPI window margin
#define OSD_DEMO_TWISTER_BEND_AMP_A     8
#define OSD_DEMO_TWISTER_BEND_AMP_B     4
#define OSD_DEMO_TWISTER_BEND_WAVE_A    140  // lines per period
#define OSD_DEMO_TWISTER_BEND_WAVE_B    90
#define OSD_DEMO_TWISTER_BEND_ROT_STEP  3    // wave A phase per field
#define OSD_DEMO_TWISTER_BEND_FLOAT_STEP 2   // wave B phase per field (opposite direction)
#define OSD_DEMO_TWISTER_BAND_PX        3    // 12×3: also the line height of the 3×3 scroller
// Vertical scroller at the right edge: the 5×5 font turned 90° CCW, font pixels 6×6 (two 3×3
// pixels of the shared 3-colour glyph set, black / white only), three cells wide, moving down.
// The line reads bottom to top; one letter + gap = 6 font pixels = 2 OSD rows.
#define OSD_DEMO_TWISTER_TEXT_CELLS     3
#define OSD_DEMO_TWISTER_TEXT_BANDS_PER_S 50 // one 3-line band a field
#define OSD_DEMO_TWISTER_WIN_MARGIN_PX  (12 + OSD_DEMO_TWISTER_BEND_AMP)
// Scene bank (0xBF..0xFE) during the scene: k=1..11 partials (solids via FILL/OFF/plasma dither).
#define OSD_DEMO_TWISTER_GLYPH_W        OSD_DEMO_FILL_BASE
#define OSD_DEMO_TWISTER_GLYPH_B        OSD_DEMO_PIXEL_OFF
// Same TL=WHITE checker as our XFILL dither builders (plasma 0xEE).
#define OSD_DEMO_TWISTER_GLYPH_D        OSD_DEMO_CLEAR // the middle face: real grey (VM1), was a 1-px checker
enum {
    OSD_DEMO_TWISTER_C_B = 0,
    OSD_DEMO_TWISTER_C_W = 1,
    OSD_DEMO_TWISTER_C_D = 2,
};
static bool twisterArmed;
static uint8_t twisterPhase;
static uint8_t twisterBendPhase;
static uint8_t twisterBendFloat;
// Display-SRAM mirror for the twister rows: lets each band send only the changed cells.
#define OSD_DEMO_MG_ROWS_MAX       16
#define OSD_DEMO_MG_RUN_MERGE_GAP  2    // bridge ≤2 clean cells instead of a new run header
#define OSD_DEMO_MG_BURST_FIXED_US 2    // spiSequence + CS overhead per burst
#define OSD_DEMO_MG_LINE_IIR_SHIFT 5    // ~32-field time constant on the measured line period
static uint8_t mgSram[OSD_DEMO_MG_ROWS_MAX][OSD_DEMO_CHARS_PER_LINE];
static uint32_t mgSramInv[OSD_DEMO_MG_ROWS_MAX]; // INV attribute mirror, bit c = column c
static uint32_t mgBandInv; // INV mask a band hook returns with its glyphs (0 = none)
static uint8_t zdmaSpc;      // slots a cell: 1, or 2 when a word fits half a cell (27 MHz SPI)
static void osdDemoZdmaTiming(void);
static const char *osdDemoHsCapHint(void);
static void osdDemoUpdate(timeUs_t currentTimeUs);
static uint32_t zdmaGate;

static void osdDemoZdmaInit(void);
static void osdDemoPbBuildIrqOn(bool on);
static void osdDemoZdmaField(void);
static void osdDemoZdmaStart(void);
static void osdDemoZdmaEnd(void);
static uint32_t zdmaStartAt, zdmaEndAt;
static bool zdmaStartPending, zdmaEndPending;
static uint32_t mgLineQ16;       // measured PAL line period, DWT ticks << 16
// Lines per field ×2 used to turn VSYNC→VSYNC into a line period. 625 = interlaced PAL;
// many FPV cameras send progressive 313-line (626) or 312-line (624) fields: same 20 ms,
// different line → row timing drifts linearly away from the calibration row.
static uint32_t mgFieldLastTicks; // raw VSYNC→VSYNC, accepted or not
static bool mgLineSeeded;
// VBLANK_US was calibrated at the top of char row 8 — pivot the line-period model there so
// changing the period does not move that calibrated row.
#define OSD_DEMO_MG_PIVOT_LINE     (8u * OSD_DEMO_CELL_H)
#define OSD_DEMO_MG_PIVOT_AT_Q8    42020u // VSYNC → the pivot line, video lines Q8 (164.14)
static uint32_t mgLastEdgeTicks;
static uint32_t mgByteTicksQ8;   // learned SPI cost per byte, DWT ticks << 8
static uint32_t mgBurstFixedTicks;
static uint32_t mgStatFields;
static uint8_t mgSlowRun;
// Beam position model for race-the-beam writes (scene 7): HSYNC falling edge → first OSD
// pixel, and one 12-px cell. 360 OSD px span ~53 us of the active line (27 MHz / 4 pixel
// clock) → 1.778 us per cell; the window per cell is a whole line, so ±15 us of error here is
// tolerated.
static int32_t mgHsyncPhase;     // THIS field: HSYNC edge − model line start, wrapped ±½ line
static int32_t mgHsyncPhaseBy[2]; // last measured phase per field parity (fallback on a miss)
static bool mgHsyncPhaseByValid[2];
static bool mgFieldFirst;        // this field is field 1 (HSYNC aligned with the VSYNC edge)
#define OSD_DEMO_MG_FIELD_PHASE_LINES 13 // measure in VBLANK, after the equalizing pulses


static uint8_t fireHeat[OSD_DEMO_FIRE_ROWS][OSD_DEMO_FIRE_COLS];
static uint8_t fireTipFine[OSD_DEMO_FIRE_COLS]; // 0..STAR_FRAMES-1 pixel phase for tip stars
static uint8_t fireTipMask[OSD_DEMO_FIRE_COLS]; // row of tip star this frame (0xFF = none)
static osdDemoFirePhase_e firePhase;
static uint8_t fireFall;     // rows already collapsed toward bottom
static uint8_t fireTick;     // collapse / step sub-tick
static uint32_t fireBeat0Q8; // song beat the scene started on (Q8)

typedef struct {
    uint8_t x;
    uint8_t y;
    uint8_t fine;  // horizontal star frame
    uint8_t life;  // 0 = dead
    uint8_t size;  // 2 or 3
    uint8_t spd;   // rise period in ticks (1=fast .. 3=slow)
    uint8_t acc;   // ticks since last rise
} osdDemoFireSpark_t;

static osdDemoFireSpark_t fireSparks[OSD_DEMO_FIRE_SPARKS];
static uint8_t fireFlameTop;       // smoothed highest tongue row (for RB gradient)
static uint8_t fireFlameTopRaw;    // instantaneous highest tongue
static uint8_t fireTopHold;        // lag counter when tip falls
static uint8_t fireFuelAvg[OSD_DEMO_FIRE_COLS]; // smoothed base fuel per column
static uint8_t fireNext[OSD_DEMO_FIRE_ROWS][OSD_DEMO_FIRE_COLS]; // rise scratch
static uint8_t fireDrawn[OSD_DEMO_FIRE_ROWS][OSD_DEMO_FIRE_COLS]; // last painted cell code

// Craft data.S sine table (256 signed bytes). Stored raw; cast on lookup.
static const uint8_t osdDemoSineU8[256] = {
    0x00, 0x03, 0x06, 0x09, 0x0c, 0x10, 0x13, 0x16, 0x19, 0x1c, 0x1f, 0x22, 0x25, 0x28, 0x2b, 0x2e,
    0x31, 0x33, 0x36, 0x39, 0x3c, 0x3f, 0x41, 0x44, 0x47, 0x49, 0x4c, 0x4e, 0x51, 0x53, 0x55, 0x58,
    0x5a, 0x5c, 0x5e, 0x60, 0x62, 0x64, 0x66, 0x68, 0x6a, 0x6b, 0x6d, 0x6f, 0x70, 0x71, 0x73, 0x74,
    0x75, 0x76, 0x78, 0x79, 0x7a, 0x7a, 0x7b, 0x7c, 0x7d, 0x7d, 0x7e, 0x7e, 0x7e, 0x7f, 0x7f, 0x7f,
    0x7f, 0x7f, 0x7f, 0x7f, 0x7e, 0x7e, 0x7e, 0x7d, 0x7d, 0x7c, 0x7b, 0x7a, 0x7a, 0x79, 0x78, 0x76,
    0x75, 0x74, 0x73, 0x71, 0x70, 0x6f, 0x6d, 0x6b, 0x6a, 0x68, 0x66, 0x64, 0x62, 0x60, 0x5e, 0x5c,
    0x5a, 0x58, 0x55, 0x53, 0x51, 0x4e, 0x4c, 0x49, 0x47, 0x44, 0x41, 0x3f, 0x3c, 0x39, 0x36, 0x33,
    0x31, 0x2e, 0x2b, 0x28, 0x25, 0x22, 0x1f, 0x1c, 0x19, 0x16, 0x13, 0x10, 0x0c, 0x09, 0x06, 0x03,
    0x00, 0xfd, 0xfa, 0xf7, 0xf4, 0xf0, 0xed, 0xea, 0xe7, 0xe4, 0xe1, 0xde, 0xdb, 0xd8, 0xd5, 0xd2,
    0xcf, 0xcd, 0xca, 0xc7, 0xc4, 0xc1, 0xbf, 0xbc, 0xb9, 0xb7, 0xb4, 0xb2, 0xaf, 0xad, 0xab, 0xa8,
    0xa6, 0xa4, 0xa2, 0xa0, 0x9e, 0x9c, 0x9a, 0x98, 0x96, 0x95, 0x93, 0x91, 0x90, 0x8f, 0x8d, 0x8c,
    0x8b, 0x8a, 0x88, 0x87, 0x86, 0x86, 0x85, 0x84, 0x83, 0x83, 0x82, 0x82, 0x82, 0x81, 0x81, 0x81,
    0x81, 0x81, 0x81, 0x81, 0x82, 0x82, 0x82, 0x83, 0x83, 0x84, 0x85, 0x86, 0x86, 0x87, 0x88, 0x8a,
    0x8b, 0x8c, 0x8d, 0x8f, 0x90, 0x91, 0x93, 0x95, 0x96, 0x98, 0x9a, 0x9c, 0x9e, 0xa0, 0xa2, 0xa4,
    0xa6, 0xa8, 0xab, 0xad, 0xaf, 0xb2, 0xb4, 0xb7, 0xb9, 0xbc, 0xbf, 0xc1, 0xc4, 0xc7, 0xca, 0xcd,
    0xcf, 0xd2, 0xd5, 0xd8, 0xdb, 0xde, 0xe1, 0xe4, 0xe7, 0xea, 0xed, 0xf0, 0xf4, 0xf7, 0xfa, 0xfd,
};
// Total leftward px/tick (layer 0 matches scroller HOS rate = 1).
static const uint8_t starSpeedPx[OSD_DEMO_STAR_LAYERS] = { 1, 2, 3 };
static const uint8_t starLayerSize[OSD_DEMO_STAR_LAYERS] = { 2, 2, 3 };

static uint16_t osdDemoTextLen(void)
{
    return (uint16_t)strlen(osdDemoScrollText);
}

static uint16_t osdDemoTunnelTextLen(void)
{
    return (uint16_t)strlen(osdDemoTunnelScrollText);
}

static bool osdDemoFontPixel(uint8_t code, uint8_t px, uint8_t py)
{
    if (code >= OSD_DEMO_FONT_GLYPHS || px >= OSD_DEMO_FONT_W || py >= OSD_DEMO_FONT_H) {
        return false;
    }
    return (osdDemoFontBits[code][py] & (0x80 >> px)) != 0;
}

// Scene 1 kerning: a glyph takes only its inked font columns (first..last of the 5), a space 3
// — '.' 1, '-' / '*' / 'I' / '1' 3, the rest 5; one column of gap after each.
static void osdDemoS1CharInk(char ch, uint8_t *col0, uint8_t *w)
{
    const uint8_t code = (uint8_t)ch;
    uint8_t bits = 0;
    if (code < OSD_DEMO_FONT_GLYPHS) {
        for (uint8_t r = 0; r < OSD_DEMO_FONT_H; r++) {
            bits |= osdDemoFontBits[code][r];
        }
    }
    bits &= 0xF8u; // the 5 font columns
    if (!bits) {
        *col0 = 0;
        *w = 3; // a space
        return;
    }
    uint8_t first = 0, last = 4;
    while (!(bits & (0x80u >> first))) {
        first++;
    }
    while (!(bits & (0x80u >> last))) {
        last--;
    }
    *col0 = first;
    *w = (uint8_t)(last - first + 1u);
}

// The scroll text's character start columns (kerned) and its width; set at the demo start.
static uint16_t s1CharStart[64];
static uint16_t osdDemoS1TextLayout(void)
{
    uint16_t x = 0;
    const uint16_t len = (uint16_t)MIN(strlen(osdDemoScrollText), ARRAYLEN(s1CharStart));
    for (uint16_t i = 0; i < len; i++) {
        uint8_t c0, w;
        osdDemoS1CharInk(osdDemoScrollText[i], &c0, &w);
        s1CharStart[i] = x;
        x = (uint16_t)(x + w + 1u);
    }
    return x;
}

static bool osdDemoTextOnFrom(const char *text, uint16_t len, uint16_t pixelCol, uint8_t pixelRow)
{
    if (len == 0 || pixelRow >= OSD_DEMO_FONT_H) {
        return false;
    }
    const uint16_t advance = OSD_DEMO_FONT_ADVANCE;
    const uint8_t px = (uint8_t)(pixelCol % advance);
    if (px >= OSD_DEMO_FONT_W) {
        return false;
    }
    const uint8_t code = (uint8_t)text[(pixelCol / advance) % len];
    return osdDemoFontPixel(code, px, pixelRow);
}

// Scene 1's scroller, kerned (osdDemoS1CharInk): the character under pixelCol (mod the text).
static bool osdDemoTextOn(uint16_t pixelCol, uint8_t pixelRow)
{
    const uint16_t len = (uint16_t)MIN(osdDemoTextLen(), ARRAYLEN(s1CharStart));
    if (!len || !textPixelCols || pixelRow >= OSD_DEMO_FONT_H) {
        return false;
    }
    const uint16_t col = (uint16_t)(pixelCol % textPixelCols);
    uint16_t i = 0;
    while (i + 1u < len && s1CharStart[i + 1u] <= col) {
        i++;
    }
    uint8_t c0, w;
    osdDemoS1CharInk(osdDemoScrollText[i], &c0, &w);
    const uint16_t px = (uint16_t)(col - s1CharStart[i]);
    return px < w && osdDemoFontPixel((uint8_t)osdDemoScrollText[i], (uint8_t)(c0 + px), pixelRow);
}

static bool osdDemoTunnelTextOn(uint16_t pixelCol, uint8_t pixelRow)
{
    return osdDemoTextOnFrom(osdDemoTunnelScrollText, osdDemoTunnelTextLen(), pixelCol, pixelRow);
}

static uint32_t osdDemoRand(void)
{
    uint32_t x = rngState ? rngState : 0xA5A5u;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    rngState = x;
    return x;
}

// Every demo glyph is a simple mask — build them at install time instead of
// storing 54-byte NVM payloads in flash. Pixels: 00 black, 10 white, 01 transparent.
#define OSD_DEMO_PX_B 0u
#define OSD_DEMO_PX_T 1u
#define OSD_DEMO_PX_W 2u

typedef enum {
    OSD_DEMO_GEN_ASCII = 0, // idx = code: the 5×5 font at 2× (10×10) centred, black outline
    OSD_DEMO_GEN_EFFECT,    // 0x90.. : off, 18 fills, stars ×2 sizes, plasma dithers
    OSD_DEMO_GEN_TUNNEL,    // 2×3 mega-pixels of 6×6, bit = 2*row + col
    OSD_DEMO_GEN_PLASMA8,   // idx 5..7: the dither ladder's top without INV (inverted 2, 0, off)
    OSD_DEMO_GEN_3C,        // idx 0..80: four 3-px columns, base-3 digit each: 0 black, 1 clear, 2 white
    OSD_DEMO_GEN_PX22,      // 6 columns of 2 px, all rows
    OSD_DEMO_GEN_XFILL,     // idx 0..32: k = idx % 11 + 1 px from the left; idx / 11: 0 white | black,
                            // 1 clear | black, 2 black | clear
    OSD_DEMO_GEN_BM,        // scene 13: idx 0..31 six 2-px columns black / white (bit 5 = left,
                            // 1 = white, the left one black)
    OSD_DEMO_GEN_GREY,      // idx 0..47: osdDemoGreyMask[idx] columns clear, the rest black
    OSD_DEMO_GEN_GREY3,     // idx 0..7: osdDemoGrey3Pat[idx], 2-px columns black / clear / white
    OSD_DEMO_GEN_PEAK,      // idx 0..11: px columns idx, idx + 1 white (11: alone), the rest black
} osdDemoGlyphGen_e;

static uint8_t osdDemoEffectPixel(uint8_t idx, uint8_t r, uint8_t c)
{
    if (idx == 0) {
        return OSD_DEMO_PX_B;                                   // PIXEL_OFF
    }
    if (idx <= OSD_DEMO_SOFT_STEPS) {
        return (r < idx - 1u) ? OSD_DEMO_PX_B : OSD_DEMO_PX_W;  // FILL_BASE + k: k black rows on top
    }
    const int8_t col = (int8_t)c;
    if (idx < OSD_DEMO_STAR3_BASE - OSD_DEMO_GLYPH_BASE) {      // 2×2 star, 12 frames
        const int8_t x0 = (int8_t)(10 - (idx - (OSD_DEMO_STAR2_BASE - OSD_DEMO_GLYPH_BASE)));
        return (r >= 8 && r <= 9 && col >= x0 && col < x0 + 2) ? OSD_DEMO_PX_W : OSD_DEMO_PX_B;
    }
    if (idx < OSD_DEMO_PLASMA_BASE - OSD_DEMO_GLYPH_BASE) {     // 3×3 star, 12 frames
        const int8_t x0 = (int8_t)(9 - (idx - (OSD_DEMO_STAR3_BASE - OSD_DEMO_GLYPH_BASE)));
        return (r >= 7 && r <= 9 && col >= x0 && col < x0 + 3) ? OSD_DEMO_PX_W : OSD_DEMO_PX_B;
    }
    bool on;
    switch (idx - (OSD_DEMO_PLASMA_BASE - OSD_DEMO_GLYPH_BASE)) {
    case 0: on = (r % 3u == 0) && (c % 3u == 0); break;        // plasma dither levels
    case 1: on = (r % 2u == 0) && (c % 3u == 0); break;
    case 2: on = (r % 2u == 0) && (c % 2u == 0); break;
    default: on = ((r + c) & 1u) == 0; break;
    }
    return on ? OSD_DEMO_PX_W : OSD_DEMO_PX_B;
}

// ASCII glyph ink: font pixel (x, y) is a 2×2 block at cell (1 + 2x, 4 + 2y) — 10×10 centred.
static bool osdDemoAsciiInk(uint8_t code, int8_t c, int8_t r)
{
    const int8_t x = (int8_t)(c - 1);
    const int8_t y = (int8_t)(r - 4);
    if (x < 0 || y < 0 || x >= 2 * OSD_DEMO_FONT_W || y >= 2 * 5) {
        return false;
    }
    return osdDemoFontPixel(code, (uint8_t)(x / 2), (uint8_t)(OSD_DEMO_TUNNEL_INK_Y0 + y / 2));
}

// The grey set's masks in glyph order (OSD_DEMO_GREY_* families, osd_demo_font.inc)
static const uint8_t osdDemoGreyMask[OSD_DEMO_GREY_ALL] = {
    0x20, 0x10, 0x08, 0x04, 0x02, 0x01, // one clear
    0x1F, 0x2F, 0x37, 0x3B, 0x3D, 0x3E, // one not
    0x28, 0x14, 0x0A, 0x05,             // two, 2 apart
    0x17, 0x2B, 0x35, 0x3A,             // all but two, 2 apart
    0x22, 0x11,                         // two, 4 apart
    0x1D, 0x2E,                         // all but two, 4 apart
    0x2A, 0x15,                         // checker
    0x38, 0x07,                         // left half, right half
    0x30, 0x3C, 0x0F, 0x03,             // edges: left 2, left 4, right 4, right 2
    0x18, 0x0C, 0x06,                   // a grey run inside: 2 px at 1, 2, 3
    0x1C, 0x0E,                         //                    3 px at 1, 2
    0x1E,                               //                    4 px at 1
    0x27, 0x33, 0x39,                   // all grey but a run inside: 2 px at 1, 2, 3
    0x23, 0x31,                         //                            3 px at 1, 2
    0x21,                               //                            4 px at 1
    0x32, 0x13, 0x0D, 0x2C,             // two runs: GG..G. and its mirror, ..GG.G and its mirror
};

// Three-colour cell lines — black, grey (clear) and white in one — that the grey set cannot show:
// the eight most frequent (statistics of scenes 9, 11, 13: the sprite's black outline on the grey,
// ~15 % of the rotozoom's black / grey / white transitions; the 14 grey masks the set lacks: 0.5 %). Base-3
// digits a 2-px column, left first (0 black, 1 clear, 2 white); INV gives the black ↔ white swap.
#define OSD_DEMO_P3D(a, b, c, d, e, f) ((a) * 243 + (b) * 81 + (c) * 27 + (d) * 9 + (e) * 3 + (f))
#define OSD_DEMO_GREY3_COUNT 13
static const uint16_t osdDemoGrey3Pat[OSD_DEMO_GREY3_COUNT] = {
    OSD_DEMO_P3D(1, 1, 1, 1, 0, 2), OSD_DEMO_P3D(0, 2, 1, 1, 1, 1), // GGGGBW, BWGGGG
    OSD_DEMO_P3D(1, 1, 1, 0, 0, 2), OSD_DEMO_P3D(0, 2, 2, 1, 1, 1), // GGGBBW, BWWGGG
    OSD_DEMO_P3D(1, 1, 1, 0, 2, 2), OSD_DEMO_P3D(0, 0, 2, 1, 1, 1), // GGGBWW, BBWGGG
    OSD_DEMO_P3D(0, 0, 2, 2, 1, 1), OSD_DEMO_P3D(1, 1, 0, 0, 2, 2), // BBWWGG, GGBBWW
    OSD_DEMO_P3D(1, 0, 0, 2, 2, 2), OSD_DEMO_P3D(0, 2, 2, 2, 1, 1), // GBBWWW, BWWWGG
    OSD_DEMO_P3D(0, 0, 0, 2, 2, 1), OSD_DEMO_P3D(1, 0, 0, 0, 2, 2), // BBBWWG, GBBBWW
    OSD_DEMO_P3D(1, 1, 0, 0, 0, 2),                                  // GGBBBW
};
static const uint8_t osdDemoGrey3Code[OSD_DEMO_GREY3_COUNT] = {
    0x38, 0x39, 0x3A, 0x3B, 0x5F, 0x60, 0x61, 0x62, 0x57, 0x58, 0x59, 0x5A, 0x5B,
};

static uint16_t osdDemoP3Swap(uint16_t idx) // black ↔ white in every column
{
    uint16_t q = 0;
    for (uint16_t w = 243; w; w = (uint16_t)(w / 3u)) {
        const uint16_t d = (uint16_t)((idx / w) % 3u);
        q = (uint16_t)(q + ((d == 1u) ? 1u : 2u - d) * w);
    }
    return q;
}

// A three-colour pattern (base 3, left first) → its glyph | INV << 8, or 0xFFFF when none
static uint16_t osdDemoGrey3Of(uint16_t idx)
{
    const uint16_t sw = osdDemoP3Swap(idx);
    for (uint8_t i = 0; i < OSD_DEMO_GREY3_COUNT; i++) {
        if (osdDemoGrey3Pat[i] == idx) {
            return osdDemoGrey3Code[i];
        }
        if (osdDemoGrey3Pat[i] == sw) {
            return (uint16_t)(osdDemoGrey3Code[i] | 0x100u);
        }
    }
    return 0xFFFFu;
}

// The glyph code of the set's pattern i (the edges and the runs sit outside the block)
static inline uint8_t osdDemoGreyCodeAt(uint8_t i)
{
    static const uint8_t edge[OSD_DEMO_GREY_RUNS - OSD_DEMO_GREY_GLYPHS] = { 0xFC, 0xFD, 0xFE, 0x8F };
    if (i < OSD_DEMO_GREY_GLYPHS) {
        return (uint8_t)(OSD_DEMO_GREY_BASE + i);
    }
    return (i < OSD_DEMO_GREY_RUNS) ? edge[i - OSD_DEMO_GREY_GLYPHS] : (uint8_t)(OSD_DEMO_GREY_RUN_BASE + i - OSD_DEMO_GREY_RUNS);
}

static uint8_t osdDemoGenPixel(osdDemoGlyphGen_e kind, uint8_t idx, uint8_t r, uint8_t c)
{
    switch (kind) {
    case OSD_DEMO_GEN_ASCII:
        if (osdDemoAsciiInk(idx, (int8_t)c, (int8_t)r)) {
            return OSD_DEMO_PX_W;
        }
        for (int8_t dy = -1; dy <= 1; dy++) {
            for (int8_t dx = -1; dx <= 1; dx++) {
                if (osdDemoAsciiInk(idx, (int8_t)(c + dx), (int8_t)(r + dy))) {
                    return OSD_DEMO_PX_B; // 1-px black outline, like the stock font
                }
            }
        }
        return OSD_DEMO_PX_T;
    case OSD_DEMO_GEN_EFFECT:
        return osdDemoEffectPixel(idx, r, c);
    case OSD_DEMO_GEN_3C: {
        static const uint8_t pow3[4] = { 27, 9, 3, 1 };
        static const uint8_t px[3] = { OSD_DEMO_PX_B, OSD_DEMO_PX_T, OSD_DEMO_PX_W };
        UNUSED(r);
        return px[(idx / pow3[c / 3u]) % 3u];
    }
    case OSD_DEMO_GEN_PLASMA8: {
        static const uint8_t base[8] = { 0, 0, 1, 2, 3, 2, 0, 0 }; // PLASMA_BASE + base
        if (idx >= 7u) {
            return OSD_DEMO_PX_W;
        }
        const uint8_t px = osdDemoEffectPixel((uint8_t)(OSD_DEMO_PLASMA_BASE - OSD_DEMO_GLYPH_BASE + base[idx]), r, c);
        if (idx <= 4u) {
            return px;
        }
        return (px == OSD_DEMO_PX_W) ? OSD_DEMO_PX_B : OSD_DEMO_PX_W; // what INV showed
    }
    case OSD_DEMO_GEN_TUNNEL:
        return ((idx >> (2u * (r / 6u) + c / 6u)) & 1u) ? OSD_DEMO_PX_W : OSD_DEMO_PX_B;
    case OSD_DEMO_GEN_XFILL: {
        UNUSED(r);
        const bool left = c < (uint8_t)(idx % 11u + 1u);
        switch (idx / 11u) {
        case 0:  return left ? OSD_DEMO_PX_W : OSD_DEMO_PX_B;
        case 1:  return left ? OSD_DEMO_PX_T : OSD_DEMO_PX_B;
        default: return left ? OSD_DEMO_PX_B : OSD_DEMO_PX_T;
        }
    }
    case OSD_DEMO_GEN_BM:
        UNUSED(r);
        return ((idx >> (5u - c / 2u)) & 1u) ? OSD_DEMO_PX_W : OSD_DEMO_PX_B;
    case OSD_DEMO_GEN_PEAK:
        UNUSED(r);
        return (c == idx || c == idx + 1u) ? OSD_DEMO_PX_W : OSD_DEMO_PX_B;
    case OSD_DEMO_GEN_GREY:
        return ((osdDemoGreyMask[idx] >> (5u - c / 2u)) & 1u) ? OSD_DEMO_PX_T : OSD_DEMO_PX_B;
    case OSD_DEMO_GEN_GREY3: {
        static const uint16_t pow3[6] = { 243, 81, 27, 9, 3, 1 };
        static const uint8_t px[3] = { OSD_DEMO_PX_B, OSD_DEMO_PX_T, OSD_DEMO_PX_W };
        UNUSED(r);
        return px[(osdDemoGrey3Pat[idx] / pow3[c / 2u]) % 3u];
    }
    case OSD_DEMO_GEN_PX22:
        return ((idx >> (c / 2u)) & 1u) ? OSD_DEMO_PX_W : OSD_DEMO_PX_B;
    default:
        return OSD_DEMO_PX_T;
    }
}

// NVM glyph write that skips glyphs already in place (reads them back first): most demo
// starts and scene entries rewrite the same sets, ~12 ms each and NVM wear.

static bool osdDemoNvmPut(uint8_t addr, const uint8_t *nvm)
{
    if (max7456NvmMatches(addr, nvm)) {
        return true;
    }
    return max7456WriteNvm(addr, nvm);
}


static void osdDemoRenderGlyph(osdDemoGlyphGen_e kind, uint8_t idx, uint8_t nvm[OSD_DEMO_GLYPH_BYTES])
{
    uint8_t *dst = nvm;
    for (uint8_t r = 0; r < OSD_DEMO_CELL_H; r++) {
        for (uint8_t c = 0; c < 12; c += 4) {
            uint8_t v = 0;
            for (uint8_t k = 0; k < 4; k++) {
                v = (uint8_t)((v << 2) | osdDemoGenPixel(kind, idx, r, (uint8_t)(c + k)));
            }
            *dst++ = v;
        }
    }
}

static bool osdDemoWriteGenGlyph(uint8_t addr, osdDemoGlyphGen_e kind, uint8_t idx)
{
    uint8_t nvm[OSD_DEMO_GLYPH_BYTES];
    osdDemoRenderGlyph(kind, idx, nvm);
    return osdDemoNvmPut(addr, nvm);
}

// End of an NVM glyph write. max7456EndFontWrite() re-inits the chip, and the re-init forces
// row brightness to (0, 2): scenes 7, 8, 10 set (0, 3) before installing their glyphs and ran at
// white 100 % instead of 120 % (every RB read back 0x01). Restore the demo's full white.
static void osdDemoFontWriteDone(void)
{
    max7456EndFontWrite();
    max7456Brightness(0, 3);
}

// --- The Betaflight font is kept in the AT7456's upper 256 glyphs while the demo runs ------
// The demo rewrites most of the lower 256 (letters, symbols, the logo) in NVM. At start the
// lower glyphs 0x00..0xFE go to 0x100 + code (only those that differ), at stop they come back.
// Glyph 0x1FF marks the state: CLEAN = backup valid and the lower half is the user's font,
// DIRTY = backup valid, the lower half may hold demo glyphs.
// The upper bank of this clone is RAM, not NVM (measured: 0.76 ms a glyph against 4.07 ms, lost at
// power-off, kept while the chip stays powered). So the backup survives an FC reset, a reflash or
// a crash mid-demo (DIRTY at boot → restored), but NOT a power cut: the backup and the mark are
// gone together and the lower NVM half keeps the demo font. The next start sees no mark and the
// demo font in the lower half, refuses to back that up as the user's, and the font has to be
// uploaded again from the Configurator. MAX7456 (256 glyphs): no backup, as before.
#define OSD_DEMO_FONT_BACKUP   0x100u
#define OSD_DEMO_FONT_CODES    0xFFu  // 0x00..0xFE (0xFF: the auto-increment escape, never written)
#define OSD_DEMO_FONT_MARK     0x1FFu
#define OSD_DEMO_FONT_CLEAN    0xC1u
#define OSD_DEMO_FONT_DIRTY    0xD1u

static const char *fontBackupNote = "not run yet";
// The chip's sync source while the demo runs: external, the camera's timing, so a video grabber
// that cannot lock to the chip's own PAL keeps its sync (cap the camera after the intro: a black
// picture wherever VOS / HOS move the OSD window off an edge or pixels are clear). Back to auto
// (Betaflight's) when the demo stops.

static uint16_t fontBackupWrites, fontRestoreWrites;
static bool fontRestorePending; // stopped while armed: restore at the next disarmed heartbeat

static void osdDemoFontMarkBytes(uint8_t state, uint8_t out[OSD_DEMO_GLYPH_BYTES])
{
    out[0] = state;
    for (uint8_t i = 1; i < OSD_DEMO_GLYPH_BYTES; i++) {
        out[i] = (uint8_t)(i * 37u + 11u);
    }
}

// OSD_DEMO_FONT_CLEAN / _DIRTY, or 0: no valid backup (or not an AT7456).
static uint8_t osdDemoFontMark(void)
{
    uint8_t got[OSD_DEMO_GLYPH_BYTES], want[OSD_DEMO_GLYPH_BYTES];
    if (!max7456NvmRead9(OSD_DEMO_FONT_MARK, got)) {
        return 0;
    }
    osdDemoFontMarkBytes(got[0], want);
    if (memcmp(got, want, sizeof(got)) != 0 || (got[0] != OSD_DEMO_FONT_CLEAN && got[0] != OSD_DEMO_FONT_DIRTY)) {
        return 0;
    }
    return got[0];
}

static void osdDemoFontSetMark(uint8_t state)
{
    uint8_t m[OSD_DEMO_GLYPH_BYTES];
    osdDemoFontMarkBytes(state, m);
    (void)max7456NvmWrite9(OSD_DEMO_FONT_MARK, m);
}

// Copies glyph `from` → `to` (9-bit codes) unless `to` already holds it. False: a read failed.
static bool osdDemoFontCopy(uint16_t from, uint16_t to, uint16_t *writes)
{
    uint8_t a[OSD_DEMO_GLYPH_BYTES], b[OSD_DEMO_GLYPH_BYTES];
    if (!max7456NvmRead9(from, a) || !max7456NvmRead9(to, b)) {
        return false;
    }
    if (memcmp(a, b, sizeof(a)) != 0) {
        (*writes)++;
        return max7456NvmWrite9(to, a);
    }
    return true;
}

static void osdDemoFontBackup(void)
{
    fontBackupWrites = 0;
    if (!max7456IsAt7456()) {
        fontBackupNote = "MAX7456 (256 glyphs): no backup";
        return;
    }
    const uint8_t mark = osdDemoFontMark();
    if (mark == OSD_DEMO_FONT_DIRTY) {
        fontBackupNote = "backup kept (the last demo did not restore the font yet)";
        return;
    }
    if (mark == 0u) {
        // No backup yet: back up only a real font, not one the demo already overwrote.
        uint8_t off[OSD_DEMO_GLYPH_BYTES], got[OSD_DEMO_GLYPH_BYTES];
        osdDemoRenderGlyph(OSD_DEMO_GEN_EFFECT, 0, off);
        if (max7456NvmRead9(OSD_DEMO_PIXEL_OFF, got) && memcmp(off, got, sizeof(off)) == 0) {
            fontBackupNote = "NO BACKUP: the lower glyphs hold the demo font (power cut mid-demo? the backup bank is RAM) - upload the Betaflight font in the Configurator";
            max7456EndFontWrite();
            return;
        }
    }
    for (uint16_t code = 0; code < OSD_DEMO_FONT_CODES; code++) {
        if (!osdDemoFontCopy(code, (uint16_t)(OSD_DEMO_FONT_BACKUP + code), &fontBackupWrites)) {
            fontBackupNote = "backup failed (NVM read)";
            max7456EndFontWrite();
            return;
        }
    }
    osdDemoFontSetMark(OSD_DEMO_FONT_DIRTY);
    fontBackupNote = "backed up";
    max7456EndFontWrite();
}

static void osdDemoFontRestore(void)
{
    fontRestorePending = false;
    fontRestoreWrites = 0;
    if (!max7456IsAt7456() || osdDemoFontMark() != OSD_DEMO_FONT_DIRTY) {
        max7456EndFontWrite();
        return;
    }
    for (uint16_t code = 0; code < OSD_DEMO_FONT_CODES; code++) {
        if (!osdDemoFontCopy((uint16_t)(OSD_DEMO_FONT_BACKUP + code), code, &fontRestoreWrites)) {
            max7456EndFontWrite();
            return; // mark stays DIRTY: the next stop tries again
        }
    }
    osdDemoFontSetMark(OSD_DEMO_FONT_CLEAN);
    max7456EndFontWrite();
}

// Display heartbeat while the demo is off: a restore deferred by arming runs once disarmed, and
// once after boot a demo that never got to restore (FC reset / power cut while it ran: the mark
// still says DIRTY) gets its font back.
static void osdDemoFontRestorePoll(void)
{
    static bool bootChecked;
    if (active || ARMING_FLAG(ARMED)) {
        return;
    }
    if (!bootChecked) {
        bootChecked = true;
        if (max7456IsAt7456() && osdDemoFontMark() == OSD_DEMO_FONT_DIRTY) {
            fontRestorePending = true;
            fontBackupNote = "restored at boot (the demo was cut off)";
        } else {
            max7456EndFontWrite(); // the mark read turned the OSD off
            return;
        }
    }
    if (fontRestorePending) {
        osdDemoFontRestore();
        max7456RefreshAll();
    }
}

// The font backup state for `osd_demo font [restore]`.
void osdDemoFontStatus(bool restoreNow, const char **note, uint8_t *mark, uint16_t *backupWrites, uint16_t *restoreWrites)
{
    if (restoreNow && !active) {
        osdDemoFontRestore();
        max7456RefreshAll();
    }
    *note = fontBackupNote;
    *mark = max7456IsAt7456() ? osdDemoFontMark() : 0u;
    if (!active) {
        max7456EndFontWrite(); // the mark read disabled the OSD
    }
    *backupWrites = fontBackupWrites;
    *restoreWrites = fontRestoreWrites;
}

// 2×2 raster cell masks (bit c = 2-px column c, 1 = white) → the 2-px set: glyph | INV << 8.
static uint16_t px22Map[64];

// A grey-set glyph (or the all-clear space)?
static bool osdDemoIsGreyCode(uint8_t code)
{
    return code == OSD_DEMO_CLEAR || (code >= OSD_DEMO_GREY_BASE && code <= 0xFEu) || code == 0x8Fu
        || (code >= OSD_DEMO_GREY_RUN_BASE && code < OSD_DEMO_GREY_RUN_BASE + OSD_DEMO_GREY_ALL - OSD_DEMO_GREY_RUNS);
}

// A mask of clear 2-px columns (bit 5 = left) on one colour → its code (0: no glyph); 0x3F = the
// space. Reusable by any scene drawing grey on black / white (INV for white).
static uint8_t osdDemoGreyCode(uint8_t mask)
{
    if (mask == 0x3Fu) {
        return OSD_DEMO_CLEAR;
    }
    for (uint8_t i = 0; i < OSD_DEMO_GREY_ALL; i++) {
        if (osdDemoGreyMask[i] == mask) {
            return osdDemoGreyCodeAt(i);
        }
    }
    return 0;
}

static void osdDemoPx22MapInit(void)
{
    for (uint8_t m = 0; m < 64u; m++) {
        uint8_t bits = 0; // the set's order: bit 5 = the left column
        for (uint8_t c = 0; c < 6u; c++) {
            bits = (uint8_t)((bits << 1) | ((m >> c) & 1u));
        }
        px22Map[m] = (bits & 0x20u) ? (uint16_t)((OSD_DEMO_BM_BASE + (~bits & 0x1Fu)) | 0x100u)
                                    : (uint16_t)(OSD_DEMO_BM_BASE + bits);
    }
}
// The 3-colour 3×3 set: every pattern of four 3-px columns in black / clear / white — all
// 81, exact. Clear pixels show the background, which a scene can make a hardware grey (VM1
// background mode, one level for the screen): black, grey, white per logical pixel, no
// flicker, no extra SPI. A pattern and its black ↔ white swap share one glyph, the swap shown
// with the cell's INV attribute (clear stays clear; INV changes inside an auto-increment run
// cost 2 bytes), so 81 patterns need 41 glyphs: all black = PIXEL_OFF
// (INV = all white), all clear = the ASCII space, 39 in 0x00..0x1F and 0x60..0x66. The
// letters at 0x20..0x5F stay. Installed with the demo font, kept by every scene.
static uint8_t osdDemo3c[81];    // pattern (base 3, left column most significant) → glyph
static uint8_t osdDemo3cInv[81]; // 1: show that glyph with INV
static uint16_t bmLut[2][729]; // 2-px cell pattern (6 base-3 digits) → code | INV << 8, per line parity

// Two 6-px halves (3c digits: 0 black, 1 clear, 2 white) → glyph | INV << 8 from the 3c set.
static inline uint16_t osdDemo3cHalves(uint8_t l, uint8_t r)
{
    const uint8_t p = (uint8_t)(l * 36u + r * 4u); // l·27 + l·9 + r·3 + r
    return (uint16_t)(osdDemo3c[p] | ((uint16_t)osdDemo3cInv[p] << 8));
}

static uint8_t osdDemo3cSwap(uint8_t p) // black ↔ white in every column
{
    uint8_t q = 0;
    for (uint8_t k = 0, w = 27; k < 4u; k++, w = (uint8_t)(w / 3u)) {
        const uint8_t d = (uint8_t)((p / w) % 3u);
        q = (uint8_t)(q + (uint8_t)(2u - d) * w);
    }
    return q;
}

static void osdDemo3cLayout(void)
{
    uint8_t slot = 0x00;
    for (uint8_t p = 0; p < 81u; p++) {
        const uint8_t q = osdDemo3cSwap(p);
        if (q < p) { // the swap is the canonical one: same glyph, INV
            osdDemo3c[p] = osdDemo3c[q];
            osdDemo3cInv[p] = 1;
            continue;
        }
        osdDemo3cInv[p] = 0;
        if (p == 0u) {
            osdDemo3c[p] = OSD_DEMO_PIXEL_OFF;
        } else if (p == 40u) {
            osdDemo3c[p] = 0x20; // all clear: the ASCII space
        } else {
            osdDemo3c[p] = slot;
            slot = (slot == 0x1Fu) ? 0x21u : (uint8_t)(slot + 1u); // 0x20: the all-clear glyph
        }
    }
}

// The whole demo font, once at the demo start (layout in osd_demo_font.inc): no glyph is written
// at a scene change, so scene changes never turn the OSD off (that was the flash). Glyphs that
// already match are only read (a restart is quick). The letters 0x20..0x5F are not written:
// no scene shows text glyphs, and the Betaflight font comes back from its backup at the stop.
static void osdDemoS1TitleGlyphs(void);
static uint8_t s1MpMask[OSD_DEMO_S1_MP_SLOTS]; // the title glyph slots' masks
static uint8_t s1MpCount;

static bool osdDemoInstallFont(void)
{
    bool ok = osdDemoWriteGenGlyph(OSD_DEMO_CLEAR, OSD_DEMO_GEN_3C, 40); // all clear
    for (uint8_t i = 0; ok && i < OSD_DEMO_GLYPH_COUNT; i++) {
        ok = osdDemoWriteGenGlyph((uint8_t)(OSD_DEMO_GLYPH_BASE + i), OSD_DEMO_GEN_EFFECT, i);
    }
    osdDemo3cLayout();
    for (uint8_t p = 0; ok && p < 81u; p++) {
        const uint8_t g = osdDemo3c[p];
        if (!osdDemo3cInv[p] && g != OSD_DEMO_PIXEL_OFF && g != OSD_DEMO_CLEAR) {
            ok = osdDemoWriteGenGlyph(g, OSD_DEMO_GEN_3C, p);
        }
    }
    // Scene-1 title mega-pixel masks: those the texts use (a mask m ≥ 32 shows as the INV of 63 − m)
    osdDemoS1TitleGlyphs();
    for (uint8_t i = 0; ok && i < s1MpCount; i++) {
        ok = osdDemoWriteGenGlyph((uint8_t)(OSD_DEMO_S1_MP_BASE + i), OSD_DEMO_GEN_TUNNEL, s1MpMask[i]);
    }
    for (uint8_t l = 5; ok && l < OSD_DEMO_PLASMA_LEVELS; l++) { // the dither ladder's top: scenes 2, 3
        ok = osdDemoWriteGenGlyph((uint8_t)(OSD_DEMO_PLASMA8_BASE + l - 5u), OSD_DEMO_GEN_PLASMA8, l);
    }
    for (uint8_t i = 0; ok && i < OSD_DEMO_BM_GLYPHS; i++) { // 2-px columns: scenes 6, 7, 9, 13
        ok = osdDemoWriteGenGlyph((uint8_t)(OSD_DEMO_BM_BASE + i), OSD_DEMO_GEN_BM, i);
    }
    for (uint8_t i = 0; ok && i < OSD_DEMO_PEAK_GLYPHS; i++) { // peak marks: scene 10
        ok = osdDemoWriteGenGlyph((uint8_t)(OSD_DEMO_PEAK_BASE + i), OSD_DEMO_GEN_PEAK, i);
    }
    for (uint8_t i = 0; ok && i < OSD_DEMO_GREY_ALL; i++) { // grey set: scenes 6, 9, 11, 12, 13
        ok = osdDemoWriteGenGlyph(osdDemoGreyCodeAt(i), OSD_DEMO_GEN_GREY, i);
    }
    for (uint8_t i = 0; ok && i < OSD_DEMO_GREY3_COUNT; i++) { // three-colour cell lines: scenes 6, 9, 11, 13
        ok = osdDemoWriteGenGlyph(osdDemoGrey3Code[i], OSD_DEMO_GEN_GREY3, i);
    }
    for (uint8_t i = 0; ok && i < 33u; i++) { // 1-px partial cells: scenes 8, 10
        ok = osdDemoWriteGenGlyph((uint8_t)(OSD_DEMO_XFILL_W_BASE + i), OSD_DEMO_GEN_XFILL, i);
    }
    // Always clear fontIsLoading — WriteNvm leaves it set until EndFontWrite.
    osdDemoFontWriteDone();
    return ok;
}

static void osdDemoInitStars(uint8_t cols, uint8_t rows)
{
    for (uint8_t layer = 0; layer < OSD_DEMO_STAR_LAYERS; layer++) {
        starFine[layer] = 0;
        for (uint8_t i = 0; i < OSD_DEMO_STARS_PER_LAYER; i++) {
            stars[layer][i].x = (uint8_t)(osdDemoRand() % cols);
            stars[layer][i].y = (uint8_t)(osdDemoRand() % rows);
            stars[layer][i].size = starLayerSize[layer];
        }
    }
}

// The drum the scenes react to (scene 4 flashes, scene 7 grey pulse, scene 10 bounce gate).
static uint8_t osdDemoDrumPulse(void)
{
    return chiptuneGetSnarePulse();
}

// Tracker-locked period: n lines (power of 2) closest to targetMs.
static uint8_t osdDemoMusicLines(uint8_t speed, uint16_t targetMs)
{
    uint8_t best = 1;
    uint32_t bestErr = UINT32_MAX;
    for (uint8_t n = 1; n <= 32u; n = (uint8_t)(n * 2u)) {
        const int32_t ms = (int32_t)n * speed * 20;
        const uint32_t err = (uint32_t)ABS(ms - (int32_t)targetMs);
        if (err < bestErr) {
            bestErr = err;
            best = n;
        }
    }
    return best;
}

// Phase 0..255 of a period locked to the music (n tracker lines nearest targetMs); without
// music a free-running targetMs clock from sinceMs. Safe from the engine ISR.
static uint16_t osdDemoMusicPhase(uint16_t targetMs, timeMs_t sinceMs)
{
    chiptuneClock_t ck;
    if (chiptuneGetClock(&ck)) {
        const uint8_t line = ck.line;
        const uint8_t tick = ck.tick;
        const uint8_t speed = ck.speed;
        const timeUs_t tickUs = ck.tickUs;
        const uint8_t n = osdDemoMusicLines(speed, targetMs);
        // Frames since the line start in Q8: the frame applied at tickUs is (tick − 1), plus
        // the fraction of the 20 ms tick elapsed since; tick 0 → last frame of the previous line.
        uint32_t frac = (uint32_t)(micros() - tickUs) * 256u / 20000u;
        if (frac > 255u) {
            frac = 255u;
        }
        const int32_t framesQ8 = ((int32_t)tick - 1) * 256 + (int32_t)frac;
        const uint32_t period = (uint32_t)n * speed * 256u;           // bounce in frames Q8
        const uint32_t pos = ((uint32_t)(line % n) * speed * 256u + period
                              + (uint32_t)framesQ8) % period;
        return (uint16_t)(pos * 256u / period);
    }
    const timeMs_t elapsed = millis() - sinceMs;
    return (uint16_t)((elapsed % targetMs) * 256u / targetMs);
}

static uint16_t osdDemoBouncePhase(void)
{
    return osdDemoMusicPhase(OSD_DEMO_BOUNCE_PERIOD_MS, bounceStartMs);
}

// Song position in rows (Q8) assuming 64-row patterns, and the speed; false without music.
static bool osdDemoSongClockQ8(uint32_t *rowQ8, uint8_t *speed)
{
    chiptuneClock_t ck;
    if (chiptuneGetClock(&ck)) {
        uint32_t frac = (uint32_t)(micros() - ck.tickUs) * 256u / 20000u;
        if (frac > 255u) {
            frac = 255u;
        }
        const int32_t framesQ8 = ((int32_t)ck.tick - 1) * 256 + (int32_t)frac;
        const int32_t pos = (int32_t)ck.lines * 256 + framesQ8 / ck.speed; // monotonic over loops
        *rowQ8 = (pos > 0) ? (uint32_t)pos : 0u;
        *speed = ck.speed;
        return true;
    }
    UNUSED(rowQ8);
    UNUSED(speed);
    return false;
}

// Beats elapsed (Q8): tracker lines nearest beatMs per beat, or a free clock from sinceMs.
static uint32_t osdDemoBeatsQ8(uint16_t beatMs, timeMs_t sinceMs)
{
    uint32_t rowQ8;
    uint8_t speed;
    if (osdDemoSongClockQ8(&rowQ8, &speed)) {
        return rowQ8 / osdDemoMusicLines(speed, beatMs);
    }
    return (uint32_t)(millis() - sinceMs) * 256u / beatMs;
}

static uint16_t osdDemoBounceFine(uint16_t hopFine, uint16_t phase)
{
    const uint32_t h = (4u * (uint32_t)phase * (256u - phase) * hopFine) / (256u * 256u);
    return (uint16_t)constrain(h, 0, hopFine);
}

static void osdDemoUpdatePunch(uint16_t fine, uint16_t phase)
{
    // Re-arm once the soft-hop is clearly airborne.
    if (fine >= 3) {
        punchArmed = true;
    }

    // Floor impact: phase wraps 255→0 once per bounce. Also catch a descending
    // sample into the floor zone in case a paint lands exactly on the trough.
    const bool phaseWrapped = (phase < prevBouncePhase);
    const bool troughHit = punchArmed && (fine <= 1) && !phaseWrapped;
    if (punchArmed && (phaseWrapped || troughHit)) {
        punchVel += OSD_DEMO_PUNCH_KICK;
        punchArmed = false;
    }
    prevBouncePhase = phase;

    punchVel -= (int16_t)(punchY / OSD_DEMO_PUNCH_SPRING);
    punchVel = (int16_t)((punchVel * OSD_DEMO_PUNCH_DAMP_NUM) / OSD_DEMO_PUNCH_DAMP_DEN);
    punchY += punchVel;
    if (punchY < 0) {
        punchY = 0;
        punchVel = 0;
    }

    int8_t maxY = 0;
    max7456GetHudMotionYLimits(NULL, &maxY);
    const int maxPunch = (int)constrain(OSD_DEMO_PUNCH_MAX_PX, 0, (int)maxY) * 8;
    if (punchY > maxPunch) {
        punchY = (int16_t)maxPunch;
        if (punchVel > 0) {
            punchVel = 0;
        }
    }
}

// HOS soft-scroll: +6..+1 right bias (half-cell staging), ease toward +1, then
// cell-step + snap back to +6. Smaller than a full cell so the left margin stays sane.
// HOS is applied by max7456DrawScreen at end-of-pass (not immediately).
static uint8_t osdDemoHosSteps(void)
{
    int8_t minX = 0;
    int8_t maxX = 0;
    max7456GetHudMotionXLimits(&minX, &maxX);
    // S1_HOS_STEPS × 3 px = one 12-px cell → even motion. 1-px steps over only 6 px moved
    // 1,1,1,1,1 px and then 7 px on every cell step (visible jerk).
    if (maxX >= OSD_DEMO_CELL_W / 2 && minX <= -(OSD_DEMO_CELL_W / 2)) {
        hosStepPx = OSD_DEMO_CELL_W / OSD_DEMO_S1_HOS_STEPS;
        return OSD_DEMO_S1_HOS_STEPS;
    }
    hosStepPx = 1;
    return (uint8_t)constrain((int)maxX, 1, (int)OSD_DEMO_HOS_SOFT_PX);
}

// Star x phase relative to the HOS soft-scroll, so a star's screen motion is starFine alone.
static uint16_t osdDemoStarRel(uint8_t layer)
{
    return (uint16_t)(starFine[layer] + OSD_DEMO_STAR_WRAP - (uint16_t)scrollFine * hosStepPx);
}

static void osdDemoApplyHudMotion(void)
{
    hosSteps = osdDemoHosSteps();
    const uint8_t fine = (scrollFine >= hosSteps) ? (uint8_t)(hosSteps - 1) : scrollFine;
    // fine=0 → hos=+hosSteps·step; fine→hosSteps-1 → hos=+step. The multi-px range (12..3)
    // is centred (+6..−3) so the OSD edges move no further than the old 6-px range did.
    int hosX = ((int)hosSteps - (int)fine) * hosStepPx;
    if (hosStepPx > 1) {
        hosX -= OSD_DEMO_CELL_W / 2;
    }
    // Hard bounce: the floor punch shakes the whole OSD down via VOS.
    max7456SetHudMotionOffset((int8_t)hosX, (int8_t)(punchY / 8));
}

// Scene 1 row brightness wanted for the next field; written right after VSYNC with the
// cells, so a level never changes mid-field. Changed rows + one refresh row per field.
static uint8_t s1RowWhite[OSD_DEMO_RB_ROWS];
static uint8_t s1RowWritten[OSD_DEMO_RB_ROWS];
static uint8_t s1RbNext;
// OSD insertion mux (OSDM: rise / fall time bits 5..3, mux switching 2..0; 0 sharpest .. 5
// softest) — one value for every scene, 5 / 4 (chosen on the glass); Betaflight's OSD gets the
// chip default 3 / 3 (0x1B) back at the stop. Written in VBLANK when changed.
#define OSD_DEMO_OSDM_DEFAULT 0x1Bu // the chip's
#define OSD_DEMO_OSDM_DEMO    ((5u << 3) | 4u)
// Scenes with clear pixels on the VM1 background (grey, or black in scene 1) get their own:
// the AT7456 does not insert that background at the softer 5 / 4 (scenes 5, 8, 9, 10, 11 lost
// their grey, scene 1 showed the video input), at 3 / 3 it does.
static uint8_t osdmApplied = 0xFF;

// The background (VM1: clear pixels black, grey 0..7 or video) the running scene wants. Every
// write goes through here, and the engine reads VM1 back each field in VBLANK and writes it
// again when the chip holds something else (scenes 5, 8, 9, 10, 11 set their grey at entry and
// lost it; the ones setting it in their first field kept it).
static bool demoBgManaged;
static bool demoBgGray = true;
static uint8_t demoBgLevel;

static void osdDemoBg(bool gray, uint8_t level)
{
    demoBgGray = gray;
    demoBgLevel = (uint8_t)(level & 7u);
    demoBgManaged = true;
    max7456WriteBackgroundNow(gray, level);
}

static void osdDemoBgType(displayPortBackground_e type)
{
    max7456SetBackgroundType(type); // the driver's type too: a re-init restores it
    demoBgManaged = true;
    switch (type) {
    case DISPLAY_BACKGROUND_TRANSPARENT:
        demoBgGray = false;
        demoBgLevel = 4;
        break;
    case DISPLAY_BACKGROUND_BLACK:
        demoBgGray = true;
        demoBgLevel = 0;
        break;
    case DISPLAY_BACKGROUND_LTGRAY:
        demoBgGray = true;
        demoBgLevel = 7;
        break;
    default:
        demoBgGray = true;
        demoBgLevel = 4;
        break;
    }
}

// In VBLANK (or from the task for the character scenes): the wanted background kept
static void osdDemoBgKeep(void)
{
    if (!demoBgManaged) {
        return;
    }
    (void)max7456BackgroundFix(demoBgGray, demoBgLevel);
}

static bool osdDemoFxUsesGrey(osdDemoFx_e f)
{
    switch (f) {
    case OSD_DEMO_FX_SCROLLER: // its clear pixels on the black VM1 background: video at 5 / 4
    case OSD_DEMO_FX_FIRE:     // the glow round the core: clear pixels on the VM1 grey
    case OSD_DEMO_FX_WIPE:
    case OSD_DEMO_FX_TUNNEL:
    case OSD_DEMO_FX_TWISTER:
    case OSD_DEMO_FX_SHOUT:
    case OSD_DEMO_FX_DONUT:
    case OSD_DEMO_FX_MEGA:
    case OSD_DEMO_FX_ROTO:
    case OSD_DEMO_FX_CUBE:
    case OSD_DEMO_FX_BITMAP:
        return true;
    default:
        return false;
    }
}

static uint8_t osdDemoOsdmWanted(void)
{
    return osdDemoFxUsesGrey(fx) ? OSD_DEMO_OSDM_DEFAULT : OSD_DEMO_OSDM_DEMO;
}

static void osdDemoApplyOsdmNow(void)
{
    const uint8_t v = osdDemoOsdmWanted();
    max7456Osdm(v);
    osdmApplied = v;
}

static void osdDemoS1WriteRowBrightness(uint8_t rows)
{
    const uint8_t n = (rows < OSD_DEMO_RB_ROWS) ? rows : OSD_DEMO_RB_ROWS;
    for (uint8_t y = 0; y < n; y++) {
        if (s1RowWhite[y] != s1RowWritten[y]) {
            s1RowWritten[y] = s1RowWhite[y];
            max7456BrightnessRow(y, 0, s1RowWhite[y]);
        }
    }
    if (n) {
        if (s1RbNext >= n) {
            s1RbNext = 0;
        }
        max7456BrightnessRow(s1RbNext, 0, s1RowWhite[s1RbNext]);
        s1RbNext++;
    }
}

// Hardware per-row brightness: scroller ridge + starfield white breathing.
static void osdDemoApplyRowShimmer(uint8_t y0, uint8_t drawH, uint8_t rows)
{
    const uint8_t span = (drawH > 1) ? (uint8_t)(drawH - 1) : 0;
    const timeMs_t elapsed = millis() - bounceStartMs;
    uint8_t peak = 0;
    if (span > 0) {
        const uint16_t path = (uint16_t)(span * 2); // up then down
        uint16_t p = (uint16_t)((elapsed % OSD_DEMO_SHIMMER_PERIOD_MS) * path / OSD_DEMO_SHIMMER_PERIOD_MS);
        peak = (p <= span) ? (uint8_t)p : (uint8_t)(path - p);
    }

    const uint8_t limit = (rows < OSD_DEMO_RB_ROWS) ? rows : OSD_DEMO_RB_ROWS;
    for (uint8_t row = 0; row < limit; row++) {
        uint8_t white;
        if (row >= y0 && row < (uint8_t)(y0 + drawH)) {
            const uint8_t local = (uint8_t)(row - y0);
            const uint8_t dist = (local > peak) ? (uint8_t)(local - peak) : (uint8_t)(peak - local);
            if (dist == 0) {
                white = 3;
            } else if (dist == 1) {
                white = 2;
            } else if (dist == 2) {
                white = 1;
            } else {
                white = 0;
            }
        } else {
            // Starfield rows: white level waves independently per row.
            const uint8_t phase = (uint8_t)((elapsed / OSD_DEMO_STAR_BRIGHT_MS) + row * 3);
            static const uint8_t starWhiteWave[8] = { 0, 1, 2, 3, 2, 1, 0, 1 };
            white = starWhiteWave[phase & 7];
        }
        s1RowWhite[row] = white;
    }
}

static void osdDemoWriteCell(uint8_t x, uint8_t y, uint8_t rows, uint8_t glyph, bool invert)
{
    if (y < rows) {
        max7456WriteCharEx(x, y, glyph, invert);
    }
}

static uint8_t osdDemoStarGlyph(uint8_t size, uint8_t frame)
{
    const uint8_t base = (size >= 3) ? OSD_DEMO_STAR3_BASE : OSD_DEMO_STAR2_BASE;
    return (uint8_t)(base + (frame % OSD_DEMO_STAR_FRAMES));
}

// Craft plasma.S calcpixel — 3 radial sines, (sum>>5)&3 expanded to 8 dither levels.
static uint8_t osdDemoSqHi(int8_t v)
{
    const int16_t p = (int16_t)v * (int16_t)v;
    return (uint8_t)((uint16_t)p >> 8);
}

static int8_t osdDemoSineAt(uint8_t idx)
{
    return (int8_t)osdDemoSineU8[idx];
}

static uint8_t osdDemoPlasmaLevel(int8_t x, int8_t y, uint8_t t)
{
    // Blob 1: (x-t, y-14), r²>>8 *4 → sine>>2
    uint8_t idx = (uint8_t)(osdDemoSqHi((int8_t)(x - (int8_t)t)) + osdDemoSqHi((int8_t)(y - 14)));
    idx = (uint8_t)(idx << 2);
    int16_t sum = (int16_t)(osdDemoSineAt(idx) >> 2);

    // Blob 2: (x-54, y-t), *8
    idx = (uint8_t)(osdDemoSqHi((int8_t)(x - 54)) + osdDemoSqHi((int8_t)(y - (int8_t)t)));
    idx = (uint8_t)(idx << 3);
    sum += (int16_t)(osdDemoSineAt(idx) >> 2);

    // Blob 3: (x-97, y-16), *8
    idx = (uint8_t)(osdDemoSqHi((int8_t)(x - 97)) + osdDemoSqHi((int8_t)(y - 16)));
    idx = (uint8_t)(idx << 3);
    sum += (int16_t)(osdDemoSineAt(idx) >> 2);

    // Craft: (sum>>5)&3 for 4 palette slots. Stretch onto 8 tileable dithers.
    int16_t v = sum + 96; // ~0..189
    if (v < 0) {
        v = 0;
    }
    if (v > 189) {
        v = 189;
    }
    return (uint8_t)((v * (OSD_DEMO_PLASMA_LEVELS - 1)) / 189);
}

static void osdDemoPaintPlasma(void);
// Scene 2's start: the last picture on screen (the fire's) gathered into the plasma's band, fast,
// in a flash: every row slides to its place in the band (the order kept), all rows at 120 % white
// with black lifted to 30 % and falling back to 0 meanwhile; then the plasma.
#define OSD_DEMO_PLASMA_GATHER_MS 320
static uint8_t plasmaGatherCode[OSD_DEMO_MG_ROWS_MAX][OSD_DEMO_CHARS_PER_LINE];
static uint8_t plasmaGatherInv[OSD_DEMO_MG_ROWS_MAX][OSD_DEMO_CHARS_PER_LINE];
static bool plasmaGather;
static timeMs_t plasmaGatherMs;
static bool plasmaSnakeOn;             // scene 2's snake (osdDemoPlasmaSnake)
static timeMs_t plasmaSnakeMs;
static int16_t plasmaSnakeOrder = -1; // track section seen last (-1: none yet)
static uint8_t plasmaSnakeCount;      // snakes in the flight now (one more each section)
static void osdDemoPaintScroller(void);
static int8_t osdDemoS1Title(uint16_t *fade, uint8_t *white);
static void osdDemoS1Reset(void);
static void osdDemoTunnelEnter(void);
static void osdDemoShoutEnter(void);
static void osdDemoShoutStarsStep(uint8_t cols, uint8_t rows);
static void osdDemoIrqStop(void);
static void osdDemoTwisterSetupGeometry(uint8_t cols, uint8_t rows);
static bool osdDemoFxIsMidGlyph(osdDemoFx_e f);
static uint8_t osdDemoFxPatterns(osdDemoFx_e f);
static timeMs_t osdDemoFxDurationMs(osdDemoFx_e f);
static uint32_t osdDemoOrderRows(uint8_t idx);
static osdDemoFx_e osdDemoOrderFx(uint8_t idx);
static void osdDemoPaintTwister(void);
static void osdDemoWaitCycles(uint32_t deadlineTicks);
// One planned race-the-beam burst and its start window (DWT ticks).
typedef struct osdDemoMgBurst_s {
    max7456SramSeg_t seg[OSD_DEMO_CHARS_PER_LINE];
    const uint8_t *glyphs;
    uint8_t row;
    uint8_t nSeg;
    uint16_t bytes;
    uint32_t startAt;
    uint32_t latest;
    uint32_t inv;      // per-column INV of this band
// Used only by the TIM5 engine (STM32F4); other MCUs keep them compiled but unused.
} osdDemoMgBurst_t;
static void osdDemoMgTrackLinePeriod(uint32_t edgeTicks, uint32_t nominalLineTicks);
static inline int32_t osdDemoMgLineOffset(uint16_t line, uint32_t lineQ16);
static bool osdDemoMgWriteRow(uint8_t row, const uint8_t *glyphs, uint8_t winL, uint8_t winR,
                              uint32_t finishBy, bool checkDeadline, bool maskIrq);
static void osdDemoMgMeasureFieldPhase(uint32_t vsyncEdge, uint32_t row0, uint32_t lineTicks);
// HSYNC captured: the phase line's edge this field and the line period from it.
static uint32_t mgCapEdge;     // DWT ticks, this field
static uint32_t mgCapLineQ16;  // line period, ticks Q16 (0 = not yet)
static uint32_t mgCapFields;

// Drain SPI after the CPU shadow is ready. waitVsync=true: wait then flush
// (EnterFx / post-paint). Never put heavy paint BETWEEN vsync and flush —
// tunnel math alone burns blanking and the beam tears the text band.
static void osdDemoSyncFlush(bool waitVsync, bool applyHosEarly)
{
    if (waitVsync) {
        (void)max7456WaitVsyncFallingEdge(NULL, OSD_DEMO_PAL_VSYNC_TIMEOUT_US);
    }
    if (applyHosEarly && !hosWrappedThisStep) {
        max7456ApplyHudMotionNow();
    }
    while (max7456DrawScreen()) {
    }
}

// Shared soft-scroll phase: HOS + star parallax stay continuous across effects.
// Plasma/wipe: freeze scrollFine and clear HOS. Fire: stars only — paintFire owns HUD.
static void osdDemoAdvanceHosAndStars(void)
{
    hosSteps = osdDemoHosSteps();
    hosWrappedThisStep = false;
    if (fx == OSD_DEMO_FX_TWISTER) {
        max7456SetHudMotionOffset(0, 0);
        return;
    }
    if (fx == OSD_DEMO_FX_PLASMA || fx == OSD_DEMO_FX_WIPE || fx == OSD_DEMO_FX_TUNNEL
        || fx == OSD_DEMO_FX_PLASMA2X2) {
        for (uint8_t layer = 0; layer < OSD_DEMO_STAR_LAYERS; layer++) {
            starFine[layer] = (uint16_t)((starFine[layer] + starSpeedPx[layer]) % OSD_DEMO_STAR_WRAP);
        }
        max7456SetHudMotionOffset(0, 0);
        return;
    }
    if (fx == OSD_DEMO_FX_FIRE) {
        for (uint8_t layer = 0; layer < OSD_DEMO_STAR_LAYERS; layer++) {
            starFine[layer] = (uint16_t)((starFine[layer] + starSpeedPx[layer]) % OSD_DEMO_STAR_WRAP);
        }
        return;
    }
    // Stars at s1StarScale (the intro ramp; 256 = normal), sub-pixel accumulated.
    for (uint8_t layer = 0; layer < OSD_DEMO_STAR_LAYERS; layer++) {
        const uint16_t acc = (uint16_t)(starAcc[layer] + starSpeedPx[layer] * s1StarScale);
        starAcc[layer] = (uint8_t)(acc & 0xFFu);
        starFine[layer] = (uint16_t)((starFine[layer] + (acc >> 8)) % OSD_DEMO_STAR_WRAP);
    }
    if (!s1TextOn) {
        osdDemoApplyHudMotion(); // intro: HOS parked
        return;
    }
    scrollFine++;
    if (scrollFine >= hosSteps) {
        scrollFine = 0;
        scrollCol++;
        hosWrappedThisStep = true; // new cells must land before HOS snap
        if (s1LeadIn && scrollCol >= s1LeadIn) {
            scrollCol = (uint16_t)(scrollCol - s1LeadIn); // same screen, lead-in consumed
            s1LeadIn = 0;
        } else if (textPixelCols && scrollCol >= textPixelCols) {
            scrollCol = 0;
        }
    }
    osdDemoApplyHudMotion();
}

// The dither ladder of scenes 2 (plasma) and 3 (fire): shade 0..7 = 0, 11, 17, 25, 50, 75, 89,
// 100 % white → its glyph, never INV (a row of them goes out as one auto-increment run): 1..4 the
// effect set's dithers, 5..7 their inverses as glyphs of their own.
static inline uint8_t osdDemoLadderGlyph(uint8_t level)
{
    if (level == 0u) {
        return OSD_DEMO_PIXEL_OFF;
    }
    if (level <= 4u) {
        return (uint8_t)(OSD_DEMO_PLASMA_BASE + level - 1u);
    }
    return (uint8_t)(OSD_DEMO_PLASMA8_BASE + MIN(level, (uint8_t)(OSD_DEMO_PLASMA_LEVELS - 1u)) - 5u);
}

static void osdDemoWritePlasmaCell(uint8_t x, uint8_t y, uint8_t rows, uint8_t level)
{
    osdDemoWriteCell(x, y, rows, osdDemoLadderGlyph(level), false);
}

// Fade in only — exit is the fire collapse, not an RB fade-out.
static uint8_t osdDemoPlasmaFadeWhite(void)
{
    const timeMs_t elapsed = millis() - fxStartMs;
    const timeMs_t fade = OSD_DEMO_PLASMA_FADE_MS;

    if (elapsed < fade) {
        return (uint8_t)((elapsed * 3) / (fade ? fade : 1));
    }
    return 3;
}

static void osdDemoSnapshotToFire(uint8_t cols, uint8_t rows)
{
    memset(fireHeat, 0, sizeof(fireHeat));
    if (cols > OSD_DEMO_FIRE_COLS) {
        cols = OSD_DEMO_FIRE_COLS;
    }
    if (rows > OSD_DEMO_FIRE_ROWS) {
        rows = OSD_DEMO_FIRE_ROWS;
    }
    const uint8_t band = OSD_DEMO_PLASMA_STAR_ROWS;
    const uint8_t plasmaY0 = band;
    const uint8_t plasmaY1 = (rows > (uint8_t)(band * 2)) ? (uint8_t)(rows - band) : rows;
    const uint8_t plasmaH = (plasmaY1 > plasmaY0) ? (uint8_t)(plasmaY1 - plasmaY0) : 1;
    const uint8_t xDiv = (cols > 1) ? (uint8_t)(cols - 1) : 1;
    const uint8_t yDiv = (plasmaH > 1) ? (uint8_t)(plasmaH - 1) : 1;

    for (uint8_t y = plasmaY0; y < plasmaY1; y++) {
        const uint8_t local = (uint8_t)(y - plasmaY0);
        const int8_t cy = (int8_t)(((uint16_t)local * 31u) / yDiv);
        for (uint8_t x = 0; x < cols; x++) {
            const int8_t cx = (int8_t)(((uint16_t)x * 59u) / xDiv);
            uint8_t level = osdDemoPlasmaLevel(cx, cy, plasmaT);
            if (level >= OSD_DEMO_PLASMA_LEVELS) {
                level = OSD_DEMO_PLASMA_LEVELS - 1;
            }
            fireHeat[y][x] = level;
        }
    }

    // Fold visible stars into the heat map so the whole screen falls together.
    for (uint8_t layer = 0; layer < OSD_DEMO_STAR_LAYERS; layer++) {
        const uint16_t rel = osdDemoStarRel(layer);
        const uint16_t cellShift = rel / OSD_DEMO_CELL_W;
        for (uint8_t i = 0; i < OSD_DEMO_STARS_PER_LAYER; i++) {
            const uint8_t y = stars[layer][i].y;
            if (y >= rows) {
                continue;
            }
            const int sx = (int)stars[layer][i].x - (int)cellShift;
            const uint8_t x = (uint8_t)(((sx % cols) + cols) % cols);
            const uint8_t spark = (stars[layer][i].size >= 3) ? 4 : 3;
            if (fireHeat[y][x] < spark) {
                fireHeat[y][x] = spark;
            }
        }
    }
}

// The fire grows with the track: every new section a stage up (0..3) — a wider hot middle at the
// floor, more fuel at the sides, slower cooling with height: taller flames, more white core.
#define OSD_DEMO_FIRE_STAGES 4
static uint8_t fireStage;
static int16_t fireStageOrder = -1; // track section seen last (-1: none yet)
static timeMs_t fireStageFlashMs;   // a stage up flashes: every row's black up to 30 % and back
static bool fireStageFlash;
#define OSD_DEMO_FIRE_STAGE_FLASH_MS 240

// (VOS soft-rise + cell shift-up removed — fire stays screen-locked.)

static void osdDemoFireCollapseStep(uint8_t cols, uint8_t rows)
{
    for (uint8_t x = 0; x < cols; x++) {
        for (uint8_t y = (uint8_t)(rows - 1); y > 0; y--) {
            uint16_t sum = (uint16_t)fireHeat[y][x] + (uint16_t)fireHeat[y - 1][x];
            if (sum > (OSD_DEMO_PLASMA_LEVELS - 1)) {
                sum = OSD_DEMO_PLASMA_LEVELS - 1;
            }
            fireHeat[y][x] = (uint8_t)sum;
            fireHeat[y - 1][x] = 0;
        }
    }
}

static void osdDemoFireStageUpdate(void)
{
    chiptuneClock_t ck;
    if (chiptuneGetClock(&ck)) {
        if ((int16_t)ck.order != fireStageOrder) {
            if (fireStageOrder >= 0 && fireStage + 1u < OSD_DEMO_FIRE_STAGES) {
                fireStage++;
                fireStageFlash = true;
                fireStageFlashMs = millis();
            }
            fireStageOrder = (int16_t)ck.order;
        }
        return;
    }
    const timeMs_t len = osdDemoFxDurationMs(OSD_DEMO_FX_FIRE); // no music: quarters of the scene
    const uint32_t q = len ? (uint32_t)(millis() - fxStartMs) * OSD_DEMO_FIRE_STAGES / len : 0u;
    const uint8_t stage = (uint8_t)MIN(q, (uint32_t)(OSD_DEMO_FIRE_STAGES - 1u));
    if (stage != fireStage) {
        fireStage = stage;
        fireStageFlash = true;
        fireStageFlashMs = millis();
    }
}

static void osdDemoFireStep(uint8_t cols, uint8_t rows)
{
    memset(fireNext, 0, sizeof(fireNext));
    const uint8_t coreHalf = (uint8_t)(OSD_DEMO_FIRE_CORE_HALF + fireStage);

    const uint8_t cx = (uint8_t)(cols / 2);
    const uint8_t maxDist = cx ? cx : 1;

    for (uint8_t y = 0; y + 1 < rows; y++) {
        for (uint8_t x = 0; x < cols; x++) {
            const uint8_t xl = (x > 0) ? (uint8_t)(x - 1) : 0;
            const uint8_t xr = (x + 1 < cols) ? (uint8_t)(x + 1) : (uint8_t)(cols - 1);
            const uint8_t c = fireHeat[y + 1][x];
            const uint16_t sum = (uint16_t)fireHeat[y + 1][xl] + (uint16_t)c * 2u
                + (uint16_t)fireHeat[y + 1][xr];
            uint8_t below = (uint8_t)(sum / 4u);
            if (c > 0 && below < (uint8_t)(c - 1)) {
                below = (uint8_t)(c - 1);
            }
            if ((osdDemoRand() % 10) == 0 && below > 0) {
                below--;
            }
            // Cool faster with height; edges die toward dark sooner than center.
            const uint8_t fromFloor = (uint8_t)(rows - 1 - y);
            const uint8_t dist = (x > cx) ? (uint8_t)(x - cx) : (uint8_t)(cx - x);
            const uint8_t edge = (uint8_t)((dist * 30u) / maxDist); // 0..30
            uint8_t die = 8;
            if (fromFloor >= 11) {
                die = 35;
            } else if (fromFloor >= 8) {
                die = 22;
            } else if (fromFloor >= 5) {
                die = 12;
            }
            die = (uint8_t)(die + edge);
            if (die > 90) {
                die = 90;
            }
            die = (uint8_t)(die * (8u - fireStage) / 8u); // a stage up: cooling 1/8 slower
            // Extra edge snuff above mid-tongue height.
            if (dist > coreHalf && fromFloor >= 4 && below > 0
                && (osdDemoRand() % 3) == 0) {
                below--;
            }
            if (below > 0 && (osdDemoRand() % 100) < die) {
                below--;
            }
            fireNext[y][x] = below;
        }
    }

    for (uint8_t x = 0; x < cols; x++) {
        const uint8_t dist = (x > cx) ? (uint8_t)(x - cx) : (uint8_t)(cx - x);
        uint8_t target;
        if (dist <= coreHalf) {
            target = (uint8_t)(6 + (osdDemoRand() % 2)); // 6..7
            if ((osdDemoRand() % 3) != 0) {
                target = 7; // mostly white-hot floor in the core
            }
        } else {
            const uint8_t roll = (uint8_t)(osdDemoRand() % 10 + fireStage); // a stage up: more fuel
            if (roll < 5) {
                target = (uint8_t)(osdDemoRand() % 3); // 0..2 more edge gaps
            } else if (roll < 8) {
                target = (uint8_t)(2 + (osdDemoRand() % 3)); // 2..4
            } else {
                target = (uint8_t)(4 + (osdDemoRand() % 2)); // 4..5
            }
        }
        fireFuelAvg[x] = (uint8_t)((fireFuelAvg[x] * 2u + target * 2u) / 4u);
        if (dist <= coreHalf && fireFuelAvg[x] + 1 < target) {
            fireFuelAvg[x] = (uint8_t)((fireFuelAvg[x] + target) / 2u);
        }
        fireNext[rows - 1][x] = fireFuelAvg[x];
        // Extra white flicker on the floor — mostly in the core.
        if (dist <= coreHalf) {
            if ((osdDemoRand() % 3) == 0) {
                fireNext[rows - 1][x] = 7;
            } else if ((osdDemoRand() & 1) != 0) {
                fireNext[rows - 1][x] = 7;
            }
        } else if ((osdDemoRand() % 6) == 0) {
            fireNext[rows - 1][x] = (uint8_t)(4 + (osdDemoRand() % 3));
        }

        if (dist <= coreHalf && rows >= 2 && (osdDemoRand() % 100) < 80) {
            const uint8_t v = (uint8_t)(fireFuelAvg[x] > 0 ? (fireFuelAvg[x] - 1) : 0);
            if (fireNext[rows - 2][x] < v) {
                fireNext[rows - 2][x] = v;
            }
        }
        if (dist <= coreHalf && rows >= 3 && (osdDemoRand() % 100) < 40) {
            const uint8_t burst = (uint8_t)(3 + (osdDemoRand() % 3)); // 3..5
            if (fireNext[rows - 3][x] < burst) {
                fireNext[rows - 3][x] = burst;
            }
        }

        const uint8_t mid = (uint8_t)(rows / 2);
        const uint8_t xl = (x > 0) ? (uint8_t)(x - 1) : 0;
        const uint8_t xr = (x + 1 < cols) ? (uint8_t)(x + 1) : (uint8_t)(cols - 1);
        if (fireHeat[mid][xl] > fireHeat[mid][xr]) {
            fireTipFine[x] = (uint8_t)((fireTipFine[x] + 1) % OSD_DEMO_STAR_FRAMES);
        } else if (fireHeat[mid][xr] > fireHeat[mid][xl]) {
            fireTipFine[x] = (uint8_t)((fireTipFine[x] + OSD_DEMO_STAR_FRAMES - 1) % OSD_DEMO_STAR_FRAMES);
        } else if ((osdDemoRand() & 7) == 0) {
            fireTipFine[x] = (uint8_t)((fireTipFine[x] + 1) % OSD_DEMO_STAR_FRAMES);
        }
    }

    for (uint8_t y = 0; y < rows; y++) {
        for (uint8_t x = 0; x < cols; x++) {
            const uint8_t n = fireNext[y][x];
            const uint8_t o = fireHeat[y][x];
            if (n >= o) {
                fireHeat[y][x] = n;
            } else {
                fireHeat[y][x] = (uint8_t)((o * 2u + n) / 3u);
            }
        }
    }

    for (uint8_t i = 0; i < OSD_DEMO_FIRE_SPARKS; i++) {
        if (fireSparks[i].life == 0) {
            continue;
        }
        fireSparks[i].acc++;
        if (fireSparks[i].acc < fireSparks[i].spd) {
            continue;
        }
        fireSparks[i].acc = 0;
        if (fireSparks[i].y == 0) { // gone at the top row
            fireSparks[i].life = 0;
            continue;
        }
        fireSparks[i].y--;
        if ((osdDemoRand() & 3) == 0) {
            fireSparks[i].fine = (uint8_t)((fireSparks[i].fine + 1) % OSD_DEMO_STAR_FRAMES);
        }
    }
    // Sparks peel off the tongue tip and fly up to the top.
    if ((osdDemoRand() % 12) == 0 && cols > 0 && rows > 4) {
        const uint8_t x = (uint8_t)(osdDemoRand() % cols);
        uint8_t tipY = 0xFF;
        for (uint8_t y = 0; y < rows; y++) {
            if (fireHeat[y][x] != 0) {
                tipY = y;
                break;
            }
        }
        // Need a real climb path (at least a few rows).
        if (tipY != 0xFF && tipY > 2u) {
            const uint8_t yPick = tipY; // spawn on the tip, then rise
            bool rowTaken = false;
            for (uint8_t i = 0; i < OSD_DEMO_FIRE_SPARKS; i++) {
                if (fireSparks[i].life && fireSparks[i].x == x
                    && fireSparks[i].y <= yPick
                    && fireSparks[i].y + 2 >= yPick) {
                    rowTaken = true;
                    break;
                }
            }
            if (!rowTaken) {
                for (uint8_t i = 0; i < OSD_DEMO_FIRE_SPARKS; i++) {
                    if (fireSparks[i].life != 0) {
                        continue;
                    }
                    fireSparks[i].x = x;
                    fireSparks[i].y = yPick;
                    fireSparks[i].fine = fireTipFine[x];
                    // Survive the whole climb to the ribbon regardless of spd.
                    fireSparks[i].life = 255;
                    fireSparks[i].size = 2;
                    fireSparks[i].spd = (uint8_t)(2 + (osdDemoRand() % 3)); // 2..4 — readable rise
                    fireSparks[i].acc = 0;
                    break;
                }
            }
        }
    }
}

static void osdDemoFireFindTips(uint8_t cols, uint8_t rows)
{
    memset(fireTipMask, 0xFF, sizeof(fireTipMask));
    fireFlameTopRaw = (uint8_t)(rows - 1);
    if (firePhase == OSD_DEMO_FIRE_COLLAPSE) {
        fireFlameTop = fireFlameTopRaw;
        return;
    }
    uint8_t tipY[OSD_DEMO_FIRE_COLS];
    memset(tipY, 0xFF, sizeof(tipY));
    bool any = false;
    for (uint8_t x = 0; x < cols; x++) {
        for (uint8_t y = 0; y < rows; y++) {
            const uint8_t h = fireHeat[y][x];
            if (h == 0) {
                continue;
            }
            if (!any || y < fireFlameTopRaw) {
                fireFlameTopRaw = y;
                any = true;
            }
            const uint8_t flameH = (uint8_t)(rows - y);
            if (h <= 2 && flameH >= 4) {
                tipY[x] = y;
            }
            break;
        }
    }
    if (!any) {
        fireFlameTopRaw = (uint8_t)(rows - 1);
    }

    // Only local height maxima — breaks the flat mid-screen tip line.
    uint8_t placed = 0;
    for (uint8_t x = 0; x < cols && placed < 3; x++) {
        if (tipY[x] == 0xFF) {
            continue;
        }
        const uint8_t xl = (x > 0) ? (uint8_t)(x - 1) : x;
        const uint8_t xr = (x + 1 < cols) ? (uint8_t)(x + 1) : x;
        const uint8_t yl = tipY[xl];
        const uint8_t yr = tipY[xr];
        const bool tallerL = (yl == 0xFF) || (tipY[x] < yl);
        const bool tallerR = (yr == 0xFF) || (tipY[x] < yr);
        if (tallerL && tallerR && (osdDemoRand() & 1) != 0) {
            fireTipMask[x] = tipY[x];
            placed++;
        }
    }

    // Smooth RB boundary: follow rising tips immediately, lag when they fall.
    if (fireFlameTopRaw < fireFlameTop) {
        fireFlameTop = fireFlameTopRaw;
        fireTopHold = 0;
    } else if (fireFlameTopRaw > fireFlameTop) {
        fireTopHold++;
        if (fireTopHold >= 3) {
            fireTopHold = 0;
            if (fireFlameTop < (uint8_t)(rows - 1)) {
                fireFlameTop++;
            }
        }
    } else {
        fireTopHold = 0;
    }
}

// White-hot cores use the same dither ladder as plasma (no soft-fill glyphs).
static void osdDemoWriteFireCell(uint8_t x, uint8_t y, uint8_t rows, uint8_t level)
{
    if (level == 0) {
        return;
    }
    osdDemoWritePlasmaCell(x, y, rows, level);
}

// The glow flickers: at each physics step every row in the fire takes its white a step up or down
// at random, the hotter the row the more often (a row's mean heat), held until the next step.
static int8_t fireRowFlick[OSD_DEMO_RB_ROWS];
#define OSD_DEMO_FIRE_CORE 6 // heat from which a cell is the white core
#define OSD_DEMO_FIRE_HALO_MAX 4 // ladder level the halo stops at (50 %)

static uint8_t fireGlowLevel = 0xFF; // VM1 grey level written for the glow

static void osdDemoFireFlickerStep(uint8_t cols, uint8_t rows)
{
    const uint8_t limit = MIN(rows, (uint8_t)OSD_DEMO_RB_ROWS);
    for (uint8_t y = 0; y < limit; y++) {
        uint16_t sum = 0;
        for (uint8_t x = 0; x < cols; x++) {
            sum = (uint16_t)(sum + fireHeat[y][x]);
        }
        const uint16_t mean4 = cols ? (uint16_t)(sum * 4u / cols) : 0u; // mean heat × 4
        const uint8_t r = (uint8_t)(osdDemoRand() % 16u);
        if (mean4 >= 12u) {
            fireRowFlick[y] = (r < 6u) ? 1 : ((r < 10u) ? -1 : 0);
        } else if (mean4 >= 2u) {
            fireRowFlick[y] = (r < 3u) ? 1 : ((r < 7u) ? -1 : 0);
        } else {
            fireRowFlick[y] = 0;
        }
    }
}

static void osdDemoApplyFireRowBright(uint8_t rows)
{
    const uint8_t limit = (rows < OSD_DEMO_RB_ROWS) ? rows : OSD_DEMO_RB_ROWS;
    uint8_t sparkRow[OSD_DEMO_RB_ROWS];
    memset(sparkRow, 0, sizeof(sparkRow));
    for (uint8_t i = 0; i < OSD_DEMO_FIRE_SPARKS; i++) {
        if (fireSparks[i].life && fireSparks[i].y < limit) {
            sparkRow[fireSparks[i].y] = 1;
        }
    }

    // Highest tongue tip → start RB; +1 white per row down to max (3).
    // Start at max-3 so the darkest band is only one row, not a tall void.
    const uint8_t top = (fireFlameTop < rows) ? fireFlameTop : (uint8_t)(rows - 1);
    const uint8_t rbMax = 3;
    const uint8_t rbStart = (uint8_t)(rbMax - 3); // tip row brightness

    // A stage up: every row's black 30 % → 0 over the flash (steps of 10 %)
    uint8_t black = 0;
    if (fireStageFlash) {
        const uint32_t el = millis() - fireStageFlashMs;
        if (el >= OSD_DEMO_FIRE_STAGE_FLASH_MS) {
            fireStageFlash = false;
        } else {
            black = (uint8_t)(3u - el * 3u / OSD_DEMO_FIRE_STAGE_FLASH_MS);
        }
    }
    for (uint8_t y = 0; y < limit; y++) {
        uint8_t white = rbMax;
        if (firePhase == OSD_DEMO_FIRE_COLLAPSE) {
            if (y < fireFall) {
                white = 0;
            } else if (y == (uint8_t)(rows - 1)) {
                white = rbMax;
            } else {
                // 1 step per collapsed row, same start as burn.
                const uint8_t pos = (uint8_t)(y - fireFall);
                white = (uint8_t)(rbStart + pos);
                if (white > rbMax) {
                    white = rbMax;
                }
            }
        } else if (y < top) {
            // Above the tallest tongue: dark, sparks still punch through brighter.
            white = sparkRow[y] ? rbMax : 0;
        } else {
            // 1 RB step per row: tip = max-3, then +1 each row → clamp at max.
            const uint8_t pos = (uint8_t)(y - top);
            white = (uint8_t)(rbStart + pos);
            if (white > rbMax) {
                white = rbMax;
            }
            if (sparkRow[y] && white < rbMax) {
                white++;
            }
            white = (uint8_t)constrain((int)white + fireRowFlick[y], 0, (int)rbMax);
            for (uint8_t x = 0; x < OSD_DEMO_FIRE_COLS; x++) {
                if (fireHeat[y][x] >= OSD_DEMO_FIRE_CORE) {
                    white = rbMax; // a row with core: its white even (bands showed on the core)
                    break;
                }
            }
        }
        const uint8_t reg = (uint8_t)((black << 2) | (rbMax - white));
        if (prevRowBright[y] != reg) {
            prevRowBright[y] = reg;
            max7456BrightnessRow(y, black, white);
        }
    }
}

// Cell codes for dirty redraw: 0=blank, 1..7=heat, 0x40|f tip2, 0x50|f tip3, 0x60|f spark,
// 0xC0|k / 0xD0|k the core's left / right edge, 0x80|mask the scroller.
// Scene 3's scroller: a line of the 5×5 font in 6×6-px dots (scene 1's title masks) over rows 3
// and 4, right to left in dot steps, through the scene once, hopping up to the beat like scene 1's
// (a dot a step, no VOS: the fire stays). The fire covers it (any cell with heat); the sparks
// and the tips' stars pass under it.
#define OSD_DEMO_FIRE_TEXT_ROW0  3
#define OSD_DEMO_FIRE_TEXT_TOP   (OSD_DEMO_FIRE_TEXT_ROW0 * 3) // its top dot row at rest
#define OSD_DEMO_FIRE_TEXT_HOP   8    // dots of the hop (~2.7 rows)
#define OSD_DEMO_FIRE_TEXT_COVER 1    // fire level from which a cell covers the text
#define OSD_DEMO_FIRE_TEXT_MPS   60   // screen width in dots (30 cells × 2)
static uint16_t fireCharStart[sizeof(osdDemoFireScrollText)];
static uint16_t fireTextW;            // dots
static uint16_t fireScrollPos;        // dots the text has moved in from the right edge
static uint16_t fireScrollDrawn = 0xFFFF;
static uint8_t fireTextTop = OSD_DEMO_FIRE_TEXT_TOP; // its top dot row now (the hop)
static uint8_t fireTextTopDrawn = 0xFF;
static uint16_t osdDemoS1MaskCode(uint8_t mask);

static void osdDemoFireTextLayout(void)
{
    uint16_t x = 0;
    for (uint16_t i = 0; osdDemoFireScrollText[i]; i++) {
        uint8_t c0, w;
        osdDemoS1CharInk(osdDemoFireScrollText[i], &c0, &w);
        fireCharStart[i] = x;
        x = (uint16_t)(x + w + 1u);
    }
    fireTextW = x;
}

// The mask (bit 2r + c: dot column c, row r) of cell (x, y), the text moved in by pos dots, its
// top at dot row top
static uint8_t osdDemoFireTextMask(uint8_t x, uint8_t y, uint16_t pos, uint8_t top)
{
    if ((uint16_t)y * 3u + 2u < top || (uint16_t)y * 3u > (uint16_t)top + 4u) {
        return 0; // a row the text does not reach
    }
    uint8_t mask = 0;
    for (uint8_t c = 0; c < 2u; c++) {
        const int32_t tc = (int32_t)(x * 2u + c) + (int32_t)pos - OSD_DEMO_FIRE_TEXT_MPS;
        if (tc < 0 || tc >= (int32_t)fireTextW) {
            continue;
        }
        uint16_t i = 0;
        while (osdDemoFireScrollText[i + 1u] && fireCharStart[i + 1u] <= (uint16_t)tc) {
            i++;
        }
        uint8_t c0, w;
        osdDemoS1CharInk(osdDemoFireScrollText[i], &c0, &w);
        const uint16_t k = (uint16_t)((uint16_t)tc - fireCharStart[i]);
        if (k >= w) {
            continue; // the gap after the character
        }
        for (uint8_t r = 0; r < 3u; r++) {
            const int16_t fr = (int16_t)(y * 3u + r) - (int16_t)top;
            if (fr >= 0 && fr < 5 && osdDemoFontPixel((uint8_t)osdDemoFireScrollText[i], (uint8_t)(c0 + k),
                                                      (uint8_t)(OSD_DEMO_TUNNEL_INK_Y0 + fr))) {
                mask |= (uint8_t)(1u << (2u * r + c));
            }
        }
    }
    return mask;
}

static uint8_t osdDemoFireCellCode(uint8_t x, uint8_t y, uint8_t rows)
{
    if (fireHeat[y][x] < OSD_DEMO_FIRE_TEXT_COVER) {
        const uint8_t mask = osdDemoFireTextMask(x, y, fireScrollPos, fireTextTop);
        if (mask) {
            return (uint8_t)(0x80u | mask); // over the sparks and stars, under the fire
        }
    }
    for (uint8_t i = 0; i < OSD_DEMO_FIRE_SPARKS; i++) {
        if (fireSparks[i].life && fireSparks[i].x == x && fireSparks[i].y == y) {
            return (uint8_t)(0x60 | (fireSparks[i].fine % OSD_DEMO_STAR_FRAMES));
        }
    }
    if (fireTipMask[x] == y) {
        const uint8_t h = fireHeat[y][x];
        const uint8_t size = (h >= 3 || (y + 1 < rows && fireHeat[y + 1][x] >= 5)) ? 3 : 2;
        const uint8_t tag = (size >= 3) ? 0x50 : 0x40;
        return (uint8_t)(tag | (fireTipFine[x] % OSD_DEMO_STAR_FRAMES));
    }
    uint8_t level = fireHeat[y][x];
    if (level >= OSD_DEMO_PLASMA_LEVELS) {
        level = OSD_DEMO_PLASMA_LEVELS - 1;
    }
    // The core: solid white; a cell beside it white on the core's side, its share by the cell's
    // own heat (3 / 6 / 9 px of 12), clear the rest — the glow, the VM1 grey near the core's
    // white — at any height; over the core and further out the dither ladder down to the
    // tongue's edge. (Black at the edge looked too hard; an all-glow cell over the core looked
    // bad, and taking the cells over the core first left the soft sides to the bottom row.)
    if (level >= OSD_DEMO_FIRE_CORE) {
        // White only where it is hottest all round; towards the core's edge two darker whites
        // (89 %, 75 % dithers): the mean heat of the 3 × 3 cells around
        uint16_t sum = 0;
        for (int8_t dy = -1; dy <= 1; dy++) {
            for (int8_t dx = -1; dx <= 1; dx++) {
                const int16_t yy = (int16_t)y + dy, xx = (int16_t)x + dx;
                if (yy >= 0 && yy < (int16_t)rows && xx >= 0 && xx < OSD_DEMO_FIRE_COLS) {
                    sum = (uint16_t)(sum + MIN(fireHeat[yy][xx], (uint8_t)(OSD_DEMO_PLASMA_LEVELS - 1u)));
                } else {
                    sum = (uint16_t)(sum + (OSD_DEMO_PLASMA_LEVELS - 1u)); // off screen: as hot (the floor)
                }
            }
        }
        return (sum >= 57u) ? 7u : ((sum >= 50u) ? 6u : 5u); // mean ≥ 6.3 / 5.6
    }
    if (level >= OSD_DEMO_FIRE_CORE - 3u) {
        const uint8_t share = (uint8_t)(level - (OSD_DEMO_FIRE_CORE - 4u)); // 1..3
        const bool left = x > 0u && fireHeat[y][x - 1u] >= OSD_DEMO_FIRE_CORE;
        const bool right = x + 1u < OSD_DEMO_FIRE_COLS && fireHeat[y][x + 1u] >= OSD_DEMO_FIRE_CORE;
        if (left != right) {
            return (uint8_t)((left ? 0xC0u : 0xD0u) | (share * OSD_DEMO_CELL_W / 4u)); // side, px
        }
    }
    // the halo no brighter than the glow (50 % at most): white, grey, then dimmer outwards
    return MIN(level, (uint8_t)OSD_DEMO_FIRE_HALO_MAX);
}

static void osdDemoFireWriteCoded(uint8_t x, uint8_t y, uint8_t rows, uint8_t code)
{
    if ((code & 0xC0u) == 0x80u) { // the scroller's dots
        const uint16_t g = osdDemoS1MaskCode((uint8_t)(code & 0x3Fu));
        osdDemoWriteCell(x, y, rows, (uint8_t)g, (g >> 8) != 0u);
        return;
    }
    if (code == 0) {
        max7456WriteChar(x, y, OSD_DEMO_PIXEL_OFF);
        return;
    }
    if ((code & 0xE0) == 0xC0) { // the core's side edge: k px white on its side, glow the rest
        const uint8_t k = (uint8_t)(code & 0x0Fu);
        if ((code & 0x10u) == 0u) { // core on the left: INV of (k black | clear)
            osdDemoWriteCell(x, y, rows, (uint8_t)(OSD_DEMO_XFILL_CT_BASE + k - 1u), true);
        } else {                    // core on the right: INV of (12 − k clear | black)
            osdDemoWriteCell(x, y, rows, (uint8_t)(OSD_DEMO_XFILL_C_BASE + (OSD_DEMO_CELL_W - k) - 1u), true);
        }
        return;
    }
    if ((code & 0xF0) == 0x60) {
        max7456WriteChar(x, y, osdDemoStarGlyph(2, (uint8_t)(code & 0x0F)));
        return;
    }
    if ((code & 0xF0) == 0x50) {
        max7456WriteChar(x, y, osdDemoStarGlyph(3, (uint8_t)(code & 0x0F)));
        return;
    }
    if ((code & 0xF0) == 0x40) {
        max7456WriteChar(x, y, osdDemoStarGlyph(2, (uint8_t)(code & 0x0F)));
        return;
    }
    osdDemoWriteFireCell(x, y, rows, code);
}

static void osdDemoFireDrawDirty(uint8_t cols, uint8_t rows, bool force)
{
    for (uint8_t y = 0; y < rows; y++) {
        for (uint8_t x = 0; x < cols; x++) {
            const uint8_t code = osdDemoFireCellCode(x, y, rows);
            if (!force && fireDrawn[y][x] == code) {
                continue;
            }
            fireDrawn[y][x] = code;
            osdDemoFireWriteCoded(x, y, rows, code);
        }
    }
}

// True-angle LUT: idx = min*64/max → 0..32 for first octant (even wedges).
static const uint8_t osdDemoAtanLut[65] = {
    0,  1,  1,  2,  3,  3,  4,  4,  5,  6,  6,  7,  8,  8,  9,  9,
    10, 11, 11, 12, 12, 13, 13, 14, 15, 15, 16, 16, 17, 17, 18, 18,
    19, 19, 20, 20, 21, 21, 22, 22, 23, 23, 24, 24, 25, 25, 25, 26,
    26, 27, 27, 27, 28, 28, 29, 29, 29, 30, 30, 30, 31, 31, 31, 32, 32
};

static void osdDemoPaintFire(void)
{
    if (!demoDisplay) {
        return;
    }
    uint8_t cols = demoDisplay->cols;
    uint8_t rows = demoDisplay->rows;
    if (cols > OSD_DEMO_FIRE_COLS) {
        cols = OSD_DEMO_FIRE_COLS;
    }
    if (rows > OSD_DEMO_FIRE_ROWS) {
        rows = OSD_DEMO_FIRE_ROWS;
    }

    osdDemoFireStageUpdate();
    bool redraw = false;

    const uint32_t nowQ8 = osdDemoBeatsQ8(OSD_DEMO_FIRE_BEAT_MS, fxStartMs);
    const uint32_t beatQ8 = (nowQ8 > fireBeat0Q8) ? nowQ8 - fireBeat0Q8 : 0u; // beats into the scene
    max7456SetHudMotionOffset(0, 0);
    if (firePhase == OSD_DEMO_FIRE_COLLAPSE) {
        // Rows fallen = share of the collapse beat(s) gone by.
        const uint32_t want = MIN((uint32_t)rows,
                                  beatQ8 * rows / (OSD_DEMO_FIRE_COLLAPSE_BEATS * 256u) + 1u);
        while (fireFall < want && fireFall < rows) {
            osdDemoFireCollapseStep(cols, rows);
            fireFall++;
            redraw = true;
        }
        if (fireFall >= rows) {
            firePhase = OSD_DEMO_FIRE_IGNITE;
            redraw = true;
        }
    } else {
        fireTick++;
        const uint8_t stepTicks = (firePhase == OSD_DEMO_FIRE_IGNITE)
            ? OSD_DEMO_FIRE_IGNITE_STEP_TICKS : OSD_DEMO_FIRE_STEP_TICKS;
        if (fireTick >= stepTicks) {
            fireTick = 0;
            osdDemoFireStep(cols, rows);
            redraw = true;
        }
        if (firePhase == OSD_DEMO_FIRE_IGNITE
            && beatQ8 >= (OSD_DEMO_FIRE_COLLAPSE_BEATS + OSD_DEMO_FIRE_IGNITE_BEATS) * 256u) {
            firePhase = OSD_DEMO_FIRE_BURN;
            memset(prevRowBright, 0xFF, sizeof(prevRowBright));
        }
    }

    // The scroller: in from the right edge to out at the left over the scene
    {
        const timeMs_t len = osdDemoFxDurationMs(OSD_DEMO_FX_FIRE);
        const uint32_t el = MIN((uint32_t)(millis() - fxStartMs), (uint32_t)len);
        fireScrollPos = (uint16_t)(len ? el * (OSD_DEMO_FIRE_TEXT_MPS + fireTextW) / len : 0u);
    }
    { // the hop: a parabola landing on the beat (scene 1's period)
        const uint32_t ph = osdDemoMusicPhase(OSD_DEMO_BOUNCE_PERIOD_MS, fxStartMs);
        const uint32_t up = OSD_DEMO_FIRE_TEXT_HOP * 4u * ph * (256u - ph) / 65536u;
        fireTextTop = (uint8_t)(OSD_DEMO_FIRE_TEXT_TOP - MIN(up, (uint32_t)OSD_DEMO_FIRE_TEXT_TOP));
    }
    const bool scrolled = fireScrollPos != fireScrollDrawn || fireTextTop != fireTextTopDrawn;
    fireScrollDrawn = fireScrollPos;
    fireTextTopDrawn = fireTextTop;
    if (!redraw) {
        osdDemoApplyFireRowBright(rows); // row levels (cached RB writes)
        if (scrolled) {
            osdDemoFireDrawDirty(cols, rows, false); // the scroller's cells only (the rest unchanged)
        }
    }
    if (redraw) {
        osdDemoFireFindTips(cols, rows);
        if (firePhase != OSD_DEMO_FIRE_COLLAPSE) {
            osdDemoFireFlickerStep(cols, rows);
        }
        // The glow (VM1 grey of the clear pixels) near the core's white: the top grey (49 %), a
        // step lower while the flames are low (42 %)
        const uint8_t height = (fireFlameTop < rows) ? (uint8_t)(rows - fireFlameTop) : 0u;
        const uint8_t glow = (firePhase == OSD_DEMO_FIRE_COLLAPSE) ? 0u : ((height * 2u >= rows) ? 7u : 6u);
        if (glow != fireGlowLevel) {
            fireGlowLevel = glow;
            osdDemoBg(true, glow);
        }
        osdDemoApplyFireRowBright(rows);
        osdDemoFireDrawDirty(cols, rows, false);
    }
}

static uint8_t osdDemoAtan2u8(int16_t y, int16_t x)
{
    if (x == 0 && y == 0) {
        return 0;
    }
    const uint16_t ax = (x < 0) ? (uint16_t)(-x) : (uint16_t)x;
    const uint16_t ay = (y < 0) ? (uint16_t)(-y) : (uint16_t)y;
    uint8_t n;
    if (ax >= ay) {
        uint8_t idx = (ax == 0) ? 0u : (uint8_t)(((uint32_t)ay * 64u) / ax);
        if (idx > 64u) {
            idx = 64u;
        }
        n = osdDemoAtanLut[idx];
    } else {
        uint8_t idx = (ay == 0) ? 0u : (uint8_t)(((uint32_t)ax * 64u) / ay);
        if (idx > 64u) {
            idx = 64u;
        }
        n = (uint8_t)(64u - osdDemoAtanLut[idx]);
    }
    if (x >= 0) {
        return (y >= 0) ? n : (uint8_t)(256u - n);
    }
    return (y >= 0) ? (uint8_t)(128u - n) : (uint8_t)(128u + n);
}

// Cosine ease 0→128 over dur (soft start/stop for look-back turns).
static uint8_t osdDemoTunnelEase128(timeMs_t t, timeMs_t dur)
{
    if (dur == 0) {
        return 128;
    }
    if (t >= dur) {
        return 128;
    }
    const uint8_t a = (uint8_t)((t * 128u) / dur); // 0..127 → half-turn of cos
    const int8_t c = osdDemoSineAt((uint8_t)(a + 64)); // cos: +127 → -127
    return (uint8_t)((((int16_t)127 - (int16_t)c) * 128) / 254);
}

// Light L/R yaw while holding a straight heading; fades at hold edges so turns connect cleanly.
static int8_t osdDemoTunnelSway(timeMs_t elapsed, timeMs_t holdPos, timeMs_t holdLen)
{
    uint16_t amp = OSD_DEMO_TUNNEL_SWAY_AMP;
    const timeMs_t fade = OSD_DEMO_TUNNEL_SWAY_FADE;
    if (holdLen <= (timeMs_t)(2u * fade)) {
        amp = (uint16_t)((amp * holdLen) / (2u * fade + 1u));
    } else if (holdPos < fade) {
        amp = (uint16_t)((amp * holdPos) / fade);
    } else if (holdPos > (timeMs_t)(holdLen - fade)) {
        amp = (uint16_t)((amp * (holdLen - holdPos)) / fade);
    }
    const uint8_t ph = (uint8_t)((elapsed * 256u) / OSD_DEMO_TUNNEL_SWAY_MS);
    return (int8_t)(((int16_t)osdDemoSineAt(ph) * (int16_t)amp) >> 7);
}

// Craft tunnel.S scene angle: 0=forward, 128=back, 256=forward (full Y spin).
// Straight holds get a soft L/R sway; 180° turns use cosine ease (no hard kick).
static uint8_t osdDemoTunnelSceneAngle(timeMs_t elapsed)
{
    if (elapsed < OSD_DEMO_TUNNEL_STATIC_MS) {
        return (uint8_t)osdDemoTunnelSway(elapsed, elapsed, OSD_DEMO_TUNNEL_STATIC_MS);
    }
    const timeMs_t after = (timeMs_t)(elapsed - OSD_DEMO_TUNNEL_STATIC_MS);
    const timeMs_t turn = OSD_DEMO_TUNNEL_TURN_MS;
    const timeMs_t fwd = OSD_DEMO_TUNNEL_FWD_MS;
    const timeMs_t back = OSD_DEMO_TUNNEL_BACK_MS;
    const timeMs_t cycle = (timeMs_t)(fwd + turn + back + turn);
    const timeMs_t phase = (timeMs_t)(after % cycle);

    if (phase < fwd) {
        return (uint8_t)osdDemoTunnelSway(elapsed, phase, fwd);
    }
    if (phase < (fwd + turn)) {
        return osdDemoTunnelEase128((timeMs_t)(phase - fwd), turn); // 0→128 eased
    }
    if (phase < (fwd + turn + back)) {
        const timeMs_t hp = (timeMs_t)(phase - fwd - turn);
        return (uint8_t)(128 + osdDemoTunnelSway(elapsed, hp, back));
    }
    return (uint8_t)(128u + osdDemoTunnelEase128((timeMs_t)(phase - fwd - turn - back), turn));
}

// Row-burst Display SRAM fill — much fewer SPI transactions than per-cell WriteChar.
static void osdDemoFillRowsGlyphFast(uint8_t cols, uint8_t rows, uint8_t glyphEven, uint8_t glyphOdd)
{
    if (cols == 0 || rows == 0) {
        return;
    }
    if (cols > OSD_DEMO_CHARS_PER_LINE) {
        cols = OSD_DEMO_CHARS_PER_LINE;
    }
    for (uint8_t y = 0; y < rows; y++) {
        const uint16_t addr = (uint16_t)((uint16_t)y * (uint16_t)OSD_DEMO_CHARS_PER_LINE);
        const uint8_t g = (y & 1u) ? glyphOdd : glyphEven;
        (void)max7456WriteDisplaySramRowFillEx(addr, g, cols, true);
    }
}


// Scene 7 — mid-glyph 2×2 plasma:
// Solid PX22 (all 18 rows) → both lines of each band lit. Early-odd SPI + prefetch
// so the burst stays ahead of the beam (avoids right-side 1px shear). No INV.

static int8_t plasmaSx[OSD_DEMO_CHECKER_COLS * OSD_DEMO_CHECKER_CELL_W2];
static int8_t plasmaSy[OSD_DEMO_CHECKER_ROWS * OSD_DEMO_PLASMA2X2_BANDS];
static int8_t plasmaSd[512]; // second half mirrors the first: index my+mx without & 255

static void osdDemoPlasma2x2BuildLuts(uint8_t cols, uint8_t rows, uint8_t t)
{
    const uint16_t mxMax = (uint16_t)cols * OSD_DEMO_CHECKER_CELL_W2;
    const uint16_t myMax = (uint16_t)rows * OSD_DEMO_PLASMA2X2_BANDS;
    for (uint16_t mx = 0; mx < mxMax; mx++) {
        plasmaSx[mx] = osdDemoSineAt((uint8_t)(mx * OSD_DEMO_PLASMA2X2_KX + t));
    }
    for (uint16_t my = 0; my < myMax; my++) {
        plasmaSy[my] = osdDemoSineAt((uint8_t)(my * OSD_DEMO_PLASMA2X2_KY + (uint8_t)(t << 1)));
    }
    for (uint16_t i = 0; i < 256u; i++) {
        plasmaSd[i] = osdDemoSineAt((uint8_t)(i * OSD_DEMO_PLASMA2X2_KD + t));
        plasmaSd[i + 256u] = plasmaSd[i];
    }
}

// v = Sx + Sy + Sd ∈ [−384, 381] (three int8 sines) → s = v + 384 ∈ [0, 765]: the old
// 0..767 clamp never fired, and (s·ZEBRA) >> 9 with ZEBRA = 8 is s >> 6. Sd is mirrored to
// 512 entries so the diagonal index needs no wrap. Same masks, ~half the inner-loop work.
#if OSD_DEMO_PLASMA_ZEBRA_BANDS != 8
#error "osdDemoPlasma2x2BuildMasks assumes 8 zebra bands (s >> 6)"
#endif
// Cells hold a 2-bit level per mega-pixel (bits 2·bc): 0 black, 1 dark grey, 2 grey, 3 white
// (raw, no glyph base); the band hook turns them into the glyph mask of this field.
// Zebra palette over the 8 eighth-bands of each 128-step period:
// levels 0 black, 1 dark grey, 2 grey, 3 white.
static const uint8_t mgZebraPal[8] = { 0, 1, 2, 3, 3, 2, 1, 0 }; // black → white and back

static void osdDemoPlasma2x2BuildMasks(uint8_t y, uint8_t b, uint8_t cols, uint16_t *cells)
{
    const uint16_t my = (uint16_t)y * OSD_DEMO_PLASMA2X2_BANDS + b;
    const int16_t base = (int16_t)plasmaSy[my] + 384;
    const int8_t *sx = plasmaSx;
    const int8_t *sd = &plasmaSd[my & 0xFFu];
    for (uint8_t x = 0; x < cols; x++) {
        uint16_t pat = 0;
        for (uint8_t bc = 0; bc < OSD_DEMO_CHECKER_CELL_W2; bc++) {
            const int16_t s6 = (int16_t)(base + (int16_t)sx[bc] + (int16_t)sd[bc]);
            // Zebra by eighth-band through the palette; 2-bit level per mega-pixel.
            const uint8_t lvl = mgZebraPal[((uint16_t)s6 >> 4) & 7u];
            pat |= (uint16_t)((uint16_t)lvl << (2u * bc));
        }
        sx += OSD_DEMO_CHECKER_CELL_W2;
        sd += OSD_DEMO_CHECKER_CELL_W2;
        cells[x] = pat;
    }
}

// Universal mid-glyph raster engine (scenes 7 and 10). Same video model as scene 8 (measured
// line period, per-field HSYNC phase, HSYNC re-lock per char row, SRAM mirror + planned
// bursts) with writes racing the beam, so every changed cell has a whole line of window.
// A scene supplies the band geometry (bandLines × bands = 18), a per-field hook (animation +
// VOS motion for the next field) and a per-band glyph builder.
typedef struct osdDemoRasterMotion_s {
    int8_t vos; // whole-OSD vertical offset, lines (+ = down)
    int8_t hos; // whole-OSD horizontal offset from the HOS at the engine start, px (+ = right)
    // Bitmap scenes: per OSD line its own HOS offset (NULL: none), written by the slot engine
    // just before each line's left edge; line 0 takes `hos` in VBLANK.
    const int8_t *hosLine;
    // Bitmap scenes: per OSD line the RB register value (black << 2 | 3 − white) it shows (NULL:
    // none) — in VBLANK the first line each row register governs, a change by the slot engine
    // just before that line's left edge: the row levels line by line (copper). Mid-row the chip
    // switches cleanly only white 80 ↔ 120 % and black 0 ↔ one level; other steps flicker.
    const uint8_t *rbLine;
} osdDemoRasterMotion_t;

// --- Scene 10: FPV shoutouts ----------------------------------------------------------------
// 3×3-px logical pixels: 4 per cell × 6 bands of 3 lines → 120×96 over the screen. The 5×5
// font fills 5 bands of a character row and leaves exactly one 3-line band between rows.
// A new name appears on every quarter note, blinks white/black every frame until the next one,
// then stays white; every beat bounces the whole OSD up via VOS (hop + small rebound).

static const char *const osdDemoShoutNames[] = {
    "CHARPU", "MR.STEELE", "SKITZO", "LE.DRIB", "UMMAGAWD", "FINALGLIDEAUS", "STINGERSWARM",
    "MATTYSTUNTZ", "JOHNNYFPV", "NURKFPV", "CRICKETFPV", "BOTGRINDER", "ZOEFPV", "VORT3X",
    "BARDWELL", "VANOVER", "HEADSUP", "MCKFPV", "NYTFURY", "JET", "BMSTHOMAS", "PHATKID",
    "WILDWILLY", "NUBB", "GAB707", "JBOX", "PAWELOSFPV", "YUKI.FPV", "DARKEX",
    "QUADMOVR", "J-TRUE", "VIKFPV", "NOICAL", "SKYWAKKA", "WESTPYSDE", "HIFLITE",
    "NATHANLOOPZ", "ERODYO", "RECKLESS.FPV", "SLATTFPV", "JACUZZI.JAY", "MARIUSFPV",
    "FENIXFPV", "AUXPLUMES", "TINE.XD", "LUMPYFPV", "PDEVX", "BUBBYFPV", "YOUDONTKNOWME",
    "PATRICK.WATKINS", "MAGIC.CARPET", "FPVEGAN", "CIOTTIFPV", "INFINITYLOOPS", "NICK.BURNS",
    "OSCAR.LIANG", "CHRIS.ROSSER", "UAV.TECH", "MAD'S.TECH",
    "BORISBSTYLE", "HYDRA", "TIMECOP", "BLCKMN", "MIKELLER", "CTZSNOOZE", "LEDVINAP",
    "JFLYPER", "MARTINBUDDEN", "SKAMAN82", "STEVECEVANS", "HASLINGHUIS", "CAPNBRY",
    "DIGITALENTITY", "ALEXINPARIS", "KILRAH",
    "ALYXFPV", "BORODA", "MARTINOSFPV", "BOGDAN", "ANIKFPV", "RECOPTER",
};
#define OSD_DEMO_SHOUT_COUNT      ARRAYLEN(osdDemoShoutNames)

// The 2×2 raster's grey checker (defined further down), shared by scene 10's fades.
static inline uint16_t osdDemoMgGraySelRow(uint16_t row);

// Names go to random free spots of the text area (columns 1..28, ≥ 1 character of air) on
// every quarter note; after LIFE_BEATS beats a name fades out through the greys, a step per beat. Columns 0 and 29 carry a 16-band AY
// "spectrum": per character row, a horizontal bar with 1-px resolution growing from the
// screen edge (XFILL-style partial cells, as in scene 8).
typedef struct {
    uint8_t row;
    uint8_t x;      // logical px, absolute (text area starts at column 1)
    uint8_t w;      // logical px
    uint8_t name;   // index into osdDemoShoutNames
    uint16_t born;  // beat number it appeared on
    uint8_t squash; // pixel rows folded away: +1 per bounce impact while fading (0..4)
    uint8_t step;   // fade blink phase shown: 0 alive, odd grey, even hidden
    timeMs_t fadeMs; // when the fade began (0 = alive)
} osdDemoShoutWord_t;

static bool shoutArmed;
#define OSD_DEMO_P3_BANDS       6
#define OSD_DEMO_PB_BUFS     3
#define OSD_DEMO_TUNNEL6_BANDS       3
#define OSD_DEMO_TUNNEL6_LP_ROWS     (16 * OSD_DEMO_TUNNEL6_BANDS)                // 48

// Scene arena: one scene runs at a time (scene entry stops the engine, then builds frame 0
// before it starts again), so the big frame buffers of the scenes share one block of RAM.
// Nothing in it survives a scene change.
static union {
    // Scenes 7, 10, 11, 12 and the donut: per cell and band glyph | INV << 8, double buffered.
    uint16_t plasmaMasks[2][OSD_DEMO_CHECKER_ROWS][OSD_DEMO_PLASMA2X2_BANDS][OSD_DEMO_CHECKER_COLS];
    struct { // shoutouts
        uint8_t fb[OSD_DEMO_SHOUT_LP_ROWS][OSD_DEMO_SHOUT_LP_COLS / 8u];
        uint8_t stars[OSD_DEMO_SHOUT_LP_ROWS][OSD_DEMO_SHOUT_LP_COLS / 8u]; // grey star pixels
        uint8_t wstars[OSD_DEMO_SHOUT_LP_ROWS][OSD_DEMO_SHOUT_LP_COLS / 8u]; // white star pixels
        struct { int16_t x, y, z; } parts[256]; // pixels of dissolved names, flying like the stars
        osdDemoShoutWord_t words[OSD_DEMO_SHOUT_MAX_WORDS];
    } shout;
    uint8_t tunnelFb[2][OSD_DEMO_TUNNEL6_LP_ROWS][OSD_DEMO_CHARS_PER_LINE]; // scene 5
    struct { // scenes 9, 11, 12, 13 (bitmap engine), triple buffered: per line the 30 codes and the INV bits
        uint8_t code[OSD_DEMO_PB_BUFS][OSD_DEMO_MG_ROWS_MAX * OSD_DEMO_CELL_H][OSD_DEMO_CHARS_PER_LINE];
        uint32_t inv[OSD_DEMO_PB_BUFS][OSD_DEMO_MG_ROWS_MAX * OSD_DEMO_CELL_H];
    } bm;
} osdDemoArena;

#define plasmaMasks osdDemoArena.plasmaMasks
#define shoutFb     osdDemoArena.shout.fb
#define shoutWords  osdDemoArena.shout.words
#define shoutStarFb osdDemoArena.shout.stars
#define shoutStarWFb osdDemoArena.shout.wstars
#define shoutParts  osdDemoArena.shout.parts
#define tunnelFb    osdDemoArena.tunnelFb
#define bmCode      osdDemoArena.bm.code
#define bmInv       osdDemoArena.bm.inv

static uint8_t shoutWordCount;
static uint16_t shoutBeatNo;
static uint8_t shoutOrder[OSD_DEMO_SHOUT_COUNT];
static uint8_t shoutNext;
static int8_t shoutNewest = -1;  // word index that blinks
static bool shoutBlinkBlack;
static uint8_t shoutLastPulse; // tracker line counter seen last (beat clock)
static uint8_t shoutLineAcc;
static timeMs_t shoutLastBeatMs;
static bool shoutBeatPending;
static uint8_t shoutRowWhite[OSD_DEMO_MG_ROWS_MAX]; // RB value (black, white) last written per row
static uint16_t shoutBarQ8[2][OSD_DEMO_MG_ROWS_MAX]; // spectrum bars, px << 8 (left, right)
// Peak hold: a white 2-px mark at each bar's peak, held OSD_DEMO_SHOUT_PEAK_HOLD frames, then
// falling at half the bars' speed.
static uint16_t shoutPeakQ8[2][OSD_DEMO_MG_ROWS_MAX];
static uint8_t shoutPeakHold[2][OSD_DEMO_MG_ROWS_MAX];
#define OSD_DEMO_SHOUT_PEAK_HOLD 10 // frames (25 a second)
static uint8_t shoutCellLevel[OSD_DEMO_MG_ROWS_MAX][OSD_DEMO_CHARS_PER_LINE]; // 4 white .. 1 darkest
static bool shoutHopPending; // a bounce hop started this frame (the impact)
// The beat hop only while the drums play: the song restarts around scene 10 with its drum-less
// intro, so the bounce starts with the first kick and stops when kicks pause > DRUMS_MS.
#define OSD_DEMO_SHOUT_DRUMS_MS 2000
static uint8_t shoutKickSeen;
static uint8_t shoutSnareSeen;
static bool shoutSnarePending;
static bool shoutKickEver;
static timeMs_t shoutLastKickMs;

static bool osdDemoShoutDrumsOn(void)
{
    if (chiptuneIsPlaying()) {
        return shoutKickEver && (timeDelta_t)(millis() - shoutLastKickMs) < OSD_DEMO_SHOUT_DRUMS_MS;
    }
    return true;
}

// Kerning: '.' and '\'' have ink in font column 2 only — that column alone, not 5.
static inline uint8_t osdDemoShoutCharCol0(char c)
{
    return (c == '.' || c == '\'') ? 2u : 0u;
}

static inline uint8_t osdDemoShoutCharW(char c)
{
    return (c == '.' || c == '\'') ? 1u : OSD_DEMO_FONT_W;
}

static uint8_t osdDemoShoutWidth(uint8_t name)
{
    uint8_t w = 0;
    for (const char *t = osdDemoShoutNames[name]; *t; t++) {
        w = (uint8_t)(w + osdDemoShoutCharW(*t) + 1u);
    }
    return w ? (uint8_t)(w - 1u) : 0u;
}

static void osdDemoShoutShuffle(void)
{
    for (uint8_t i = 0; i < OSD_DEMO_SHOUT_COUNT; i++) {
        shoutOrder[i] = i;
    }
    for (uint8_t i = (uint8_t)(OSD_DEMO_SHOUT_COUNT - 1u); i > 0u; i--) {
        const uint8_t j = (uint8_t)(osdDemoRand() % (uint32_t)(i + 1u));
        const uint8_t t = shoutOrder[i];
        shoutOrder[i] = shoutOrder[j];
        shoutOrder[j] = t;
    }
    shoutNext = 0;
}

// Fade step by the name's own clock: 0 alive, 1..3 greys, FADE_STEPS = gone.
// A fading name's blink phase: 0 alive, then 1, 2, 3 … (odd shown in grey, even hidden),
// toggles so far = BLINKS · f² (f = share of the fade): slow at first, faster and faster;
// 0xFF once its fade is over (gone).
static uint8_t osdDemoShoutFadeStep(const osdDemoShoutWord_t *wd, timeMs_t now)
{
    if (!wd->fadeMs) {
        return 0;
    }
    const timeMs_t t = now - wd->fadeMs;
    if (t >= OSD_DEMO_SHOUT_FADE_MS) {
        return 0xFF;
    }
    const uint32_t f = (uint32_t)t * 256u / OSD_DEMO_SHOUT_FADE_MS; // 0..255
    return (uint8_t)(1u + OSD_DEMO_SHOUT_BLINKS * f * f / 65536u);
}

// Paint a name; squashed names are flattened onto their baseline inside their own row: font
// rows merge (OR) into fewer pixel rows, so the name flattens without losing a pixel.
static void osdDemoShoutPaint(const osdDemoShoutWord_t *wd)
{
    const uint8_t h = (uint8_t)(OSD_DEMO_SHOUT_FONT_ROWS - MIN(wd->squash, OSD_DEMO_SHOUT_FONT_ROWS - 1u));
    const uint16_t lpRow0 = (uint16_t)wd->row * OSD_DEMO_SHOUT_BANDS + (OSD_DEMO_SHOUT_FONT_ROWS - h);
    const char *s = osdDemoShoutNames[wd->name];
    uint16_t cx0 = wd->x;
    for (uint8_t ci = 0; s[ci]; cx0 = (uint16_t)(cx0 + osdDemoShoutCharW(s[ci]) + 1u), ci++) {
        const uint8_t code = (uint8_t)s[ci];
        if (code >= OSD_DEMO_FONT_GLYPHS) {
            continue;
        }
        const uint8_t col0 = osdDemoShoutCharCol0(s[ci]), cw = osdDemoShoutCharW(s[ci]);
        for (uint8_t r = 0; r < OSD_DEMO_SHOUT_FONT_ROWS; r++) {
            const uint8_t bits = osdDemoFontBits[code][OSD_DEMO_TUNNEL_INK_Y0 + r];
            for (uint8_t px = 0; px < cw; px++) {
                if (bits & (0x80u >> (col0 + px))) {
                    const uint16_t x = (uint16_t)(cx0 + px);
                    const uint16_t yy = (uint16_t)(lpRow0 + (r * h) / OSD_DEMO_SHOUT_FONT_ROWS);
                    if (yy < OSD_DEMO_SHOUT_LP_ROWS) {
                        shoutFb[yy][x >> 3] |= (uint8_t)(0x80u >> (x & 7u));
                    }
                }
            }
        }
    }
}

static bool osdDemoShoutOnScreen(uint8_t name)
{
    for (uint8_t i = 0; i < shoutWordCount; i++) {
        if (shoutWords[i].name == name) {
            return true;
        }
    }
    return false;
}

// The side bars as arcs: a row's longest bar grows from one cell mid-screen to six at the
// top and bottom (parabola) — the bars there swing wider; names keep out of those cells.
#define OSD_DEMO_SHOUT_BAR_ARC_PX  60 // extra px at the corners (on top of a cell): 72 px = 6 cells there
static uint8_t shoutBarRows = OSD_DEMO_MG_ROWS_MAX; // the rows the arcs span (set with the bars)

static uint8_t osdDemoShoutBarMaxPx(uint8_t y, uint8_t rows)
{
    const int32_t span = (rows > 1u) ? (int32_t)rows - 1 : 1;
    const int32_t dq8 = ABS(2 * (int32_t)y - span) * 256 / span; // 0 mid .. 256 edge
    return (uint8_t)(OSD_DEMO_CELL_W + (OSD_DEMO_SHOUT_BAR_ARC_PX * dq8 * dq8 + 32768) / 65536);
}

static uint8_t osdDemoShoutBarCells(uint8_t y, uint8_t rows)
{
    return (uint8_t)((osdDemoShoutBarMaxPx(y, rows) + OSD_DEMO_CELL_W - 1u) / OSD_DEMO_CELL_W);
}

static bool osdDemoShoutFits(uint8_t row, uint8_t x, uint8_t w)
{
    for (uint8_t i = 0; i < shoutWordCount; i++) {
        const osdDemoShoutWord_t *o = &shoutWords[i];
        if (o->row != row) {
            continue;
        }
        // At least one character (6 logical px) of air between names.
        const bool leftOk = (uint16_t)x + w + OSD_DEMO_FONT_ADVANCE <= o->x;
        const bool rightOk = (uint16_t)o->x + o->w + OSD_DEMO_FONT_ADVANCE <= x;
        if (!leftOk && !rightOk) {
            return false;
        }
    }
    return true;
}

static bool shoutDirty; // repaint the name raster from the word list

static void osdDemoShoutRemove(uint8_t i)
{
    shoutDirty = true;
    if (shoutNewest == (int8_t)i) {
        shoutNewest = -1;
    } else if (shoutNewest == (int8_t)(shoutWordCount - 1u)) {
        shoutNewest = (int8_t)i; // the last entry moves into the hole
    }
    shoutWords[i] = shoutWords[--shoutWordCount];
}

static void osdDemoShoutDissolve(const osdDemoShoutWord_t *wd);
static void osdDemoShoutRemove(uint8_t i);

// Names past their life turn into stars (on a snare, or on the beat without music).
static void osdDemoShoutDissolveOld(void)
{
    for (uint8_t i = 0; i < shoutWordCount;) {
        if ((uint16_t)(shoutBeatNo - shoutWords[i].born) >= OSD_DEMO_SHOUT_LIFE_BEATS) {
            osdDemoShoutDissolve(&shoutWords[i]);
            osdDemoShoutRemove(i); // re-check slot i
            continue;
        }
        i++;
    }
}

// One quarter note: expire old names, then place the next one at a random free spot.
static void osdDemoShoutBeat(uint8_t cols, uint8_t rows)
{
    shoutBeatNo++;
    // Past its life a name starts to fade on its own clock (grey, thinning, gone) — a snare
    // may still turn it into stars first; without snares the names no longer pile up.
    const timeMs_t nowMs = millis();
    for (uint8_t i = 0; i < shoutWordCount; i++) {
        if (!shoutWords[i].fadeMs && (uint16_t)(shoutBeatNo - shoutWords[i].born) >= OSD_DEMO_SHOUT_LIFE_BEATS) {
            shoutWords[i].fadeMs = nowMs ? nowMs : 1u;
            shoutDirty = true;
        }
    }
    if (!chiptuneIsPlaying())
    {
        osdDemoShoutDissolveOld(); // no music: old names turn into stars on the beat
    }
    if (shoutWordCount >= OSD_DEMO_SHOUT_MAX_WORDS) {
        osdDemoShoutRemove(0); // table full (cannot happen on a 30×16 screen, but be exact)
    }
    // Next name not already on screen.
    uint8_t name = shoutOrder[shoutNext];
    for (uint8_t n = 0; n < OSD_DEMO_SHOUT_COUNT && osdDemoShoutOnScreen(name); n++) {
        if (++shoutNext >= OSD_DEMO_SHOUT_COUNT) {
            osdDemoShoutShuffle();
        }
        name = shoutOrder[shoutNext];
    }
    const uint8_t w = osdDemoShoutWidth(name);
    if (w > (uint8_t)((cols - 2u) * OSD_DEMO_SHOUT_LP_PER_CELL)) {
        return;
    }
    osdDemoShoutWord_t wd = { 0, 0, w, name, shoutBeatNo, 0, 0, 0 };
    // No room → drop the oldest name and try again, so a new name appears on every beat.
    for (uint8_t evict = 0; evict <= OSD_DEMO_SHOUT_MAX_EVICT; evict++) {
        for (uint8_t tries = 0; tries < OSD_DEMO_SHOUT_TRIES; tries++) {
            // Rows 1 .. rows − 2: a row of air above and below (row 0 sat on the screen's edge).
            wd.row = (rows > 2u) ? (uint8_t)(1u + osdDemoRand() % (uint32_t)(rows - 2u)) : 0u;
            // Between the row's bar cells (the arcs)
            const uint8_t bc = osdDemoShoutBarCells(wd.row, rows);
            const uint8_t x0 = (uint8_t)(bc * OSD_DEMO_SHOUT_LP_PER_CELL);
            const uint8_t x1 = (uint8_t)((cols - bc) * OSD_DEMO_SHOUT_LP_PER_CELL);
            if (x1 <= x0 || w > (uint8_t)(x1 - x0)) {
                continue;
            }
            wd.x = (uint8_t)(x0 + osdDemoRand() % (uint32_t)(x1 - x0 - w + 1u));
            if (osdDemoShoutFits(wd.row, wd.x, w)) {
                if (++shoutNext >= OSD_DEMO_SHOUT_COUNT) {
                    osdDemoShoutShuffle();
                }
                shoutWords[shoutWordCount] = wd;
                shoutNewest = (int8_t)shoutWordCount;
                shoutWordCount++;
                shoutDirty = true;
                shoutBlinkBlack = false;
                if (osdDemoShoutDrumsOn()) {
                    shoutHopPending = true; // its impact folds the fading names one more row
                }
                return;
            }
        }
        if (shoutWordCount == 0u) {
            return;
        }
        uint8_t oldest = 0;
        for (uint8_t i = 1; i < shoutWordCount; i++) {
            if ((uint16_t)(shoutBeatNo - shoutWords[i].born) > (uint16_t)(shoutBeatNo - shoutWords[oldest].born)) {
                oldest = i;
            }
        }
        osdDemoShoutRemove(oldest);
    }
}

// Beats from the tracker: PT3 lines at 50/speed Hz, a quarter ≈ 0.5 s → 2/4/8/16 lines.
// Without music: a fixed 120 BPM clock.
static uint8_t osdDemoShoutBeats(void)
{
    const uint8_t speed = chiptuneGetSpeed();
    if (speed > 0u) {
        uint8_t lpb = 2;
        while (lpb < 16u && (uint16_t)lpb * speed * 2u < 50u) {
            lpb = (uint8_t)(lpb * 2u);
        }
        const uint8_t pulse = chiptuneGetLinePulse();
        shoutLineAcc = (uint8_t)(shoutLineAcc + (uint8_t)(pulse - shoutLastPulse));
        shoutLastPulse = pulse;
        uint8_t beats = 0;
        while (shoutLineAcc >= lpb) {
            shoutLineAcc = (uint8_t)(shoutLineAcc - lpb);
            beats++;
        }
        return beats;
    }
    const timeMs_t now = millis();
    if ((timeDelta_t)(now - shoutLastBeatMs) >= OSD_DEMO_SHOUT_FALLBACK_BEAT_MS) {
        shoutLastBeatMs = now;
        return 1;
    }
    return 0;
}

// AY "spectrum" from the live register frame (no FFT): each tone channel lands in an octave band
// by its period, mirrored top and bottom (bass at the edges, highs meeting mid-screen), its
// volume sets the bar; noise lights the highest bands. Stereo like the Spectrum's ABC: left = A + ½B, right = C + ½B.
static void osdDemoShoutUpdateBars(uint8_t rows)
{
    shoutBarRows = MIN(rows, (uint8_t)OSD_DEMO_MG_ROWS_MAX);
    uint8_t target[2][OSD_DEMO_MG_ROWS_MAX];
    memset(target, 0, sizeof(target));
    uint8_t regs[16];
    if (chiptuneGetAyRegs(regs)) {
        const uint8_t mixer = regs[7];
        for (uint8_t ch = 0; ch < 3u; ch++) {
            const uint8_t volReg = regs[8u + ch];
            const uint8_t vol = (volReg & 0x10u) ? (uint8_t)OSD_DEMO_SHOUT_ENV_LEVEL : (uint8_t)(volReg & 0x0Fu);
            if (vol == 0u) {
                continue;
            }
            const uint8_t len = (uint8_t)((vol * OSD_DEMO_CELL_W + 7u) / 15u); // 0..12 px
            const bool tone = (mixer & (1u << ch)) == 0u;
            const bool noise = (mixer & (8u << ch)) == 0u;
            // Mirrored: band k (an octave, 0 = the bass) on rows k and rows − 1 − k — the bass at
            // the top and bottom edges (the widest arc), the highs meeting in the middle.
            const uint8_t half = (uint8_t)(rows / 2u);
            int8_t band = -1;
            if (tone) {
                const uint16_t period = (uint16_t)(regs[ch * 2u] | ((regs[ch * 2u + 1u] & 0x0Fu) << 8));
                if (period > 0u) {
                    uint8_t msb = 0;
                    while ((uint16_t)(period >> (msb + 1u)) != 0u) {
                        msb++;
                    }
                    // period 2048..4095 (≈ 27..55 Hz) → band 0 … 16..31 (≈ 3.5..6.9 kHz) → band 7
                    band = (int8_t)constrain(11 - (int16_t)msb, 0, (int16_t)half - 1);
                }
            } else if (noise) {
                band = (int8_t)((int16_t)half - 1 - (int16_t)(osdDemoRand() % 2u)); // the highs
            }
            if (band < 0) {
                continue;
            }
            const uint8_t side[3][2] = { { 2, 0 }, { 1, 1 }, { 0, 2 } }; // A, B, C weights /2
            for (uint8_t sd = 0; sd < 2u; sd++) {
                const uint8_t l = (uint8_t)((len * side[ch][sd]) / 2u);
                for (int8_t dk = -1; dk <= 1; dk++) { // soft neighbours
                    const int8_t k = (int8_t)(band + dk);
                    if (k < 0 || k >= (int8_t)half) {
                        continue;
                    }
                    const uint8_t v = dk ? (uint8_t)(l / 2u) : l;
                    const uint8_t r0 = (uint8_t)k, r1 = (uint8_t)(rows - 1u - (uint8_t)k);
                    if (v > target[sd][r0]) {
                        target[sd][r0] = v;
                    }
                    if (v > target[sd][r1]) {
                        target[sd][r1] = v;
                    }
                }
            }
        }
    }
    for (uint8_t sd = 0; sd < 2u; sd++) {
        for (uint8_t y = 0; y < rows && y < OSD_DEMO_MG_ROWS_MAX; y++) {
            // A cell's worth of level stretched to the row's arc length, the fall with it
            const uint8_t maxPx = osdDemoShoutBarMaxPx(y, rows);
            const uint16_t t = (uint16_t)(((uint32_t)target[sd][y] * maxPx << 8) / OSD_DEMO_CELL_W);
            const uint16_t decay = (uint16_t)((uint32_t)OSD_DEMO_SHOUT_BAR_DECAY_Q8 * maxPx / OSD_DEMO_CELL_W);
            uint16_t *b = &shoutBarQ8[sd][y];
            // Attack at once, fall back slowly (classic analyser feel).
            if (t >= *b) {
                *b = t;
            } else {
                *b = (*b - t > decay) ? (uint16_t)(*b - decay) : t;
            }
            uint16_t *pk = &shoutPeakQ8[sd][y];
            if (*b >= *pk) {
                *pk = *b;
                shoutPeakHold[sd][y] = OSD_DEMO_SHOUT_PEAK_HOLD;
            } else if (shoutPeakHold[sd][y]) {
                shoutPeakHold[sd][y]--;
            } else {
                const uint16_t fall = (uint16_t)(decay / 2u);
                *pk = (*pk - *b > fall) ? (uint16_t)(*pk - fall) : *b;
            }
        }
    }
}

// Once per frame: every expired name moves one random letter a step towards gone.
// The newest name blinks white / black; once the next one comes it turns grey. LIFE_BEATS after
// it appeared a name blinks grey / black, faster and faster, on its own clock (FADE_MS) and is gone — always,
// independent of new names arriving (a snare may turn it into stars first). The impact of each bounce hop (only with
// the drums) folds a fading name one more pixel row inside its own row, and it stays folded.
// No pixel is ever dropped. The raster
// is repainted from the word list whenever it changes; per-cell level for the band hook
// (darkest name wins where a slid column reaches the row below).
static void osdDemoShoutFade(uint8_t rows, bool hop)
{
    const timeMs_t now = millis();
    for (uint8_t i = 0; i < shoutWordCount;) {
        osdDemoShoutWord_t *wd = &shoutWords[i];
        const uint8_t step = osdDemoShoutFadeStep(wd, now);
        if (step == 0xFFu) {
            osdDemoShoutRemove(i); // re-check slot i
            shoutDirty = true;
            continue;
        }
        if (step != wd->step) {
            wd->step = step;
            shoutDirty = true;
        }
        UNUSED(hop);
        i++;
    }
    if (!shoutDirty) {
        return;
    }
    shoutDirty = false;
    memset(shoutFb, 0, sizeof(shoutFb));
    memset(shoutCellLevel, 4, sizeof(shoutCellLevel));
    for (uint8_t i = 0; i < shoutWordCount; i++) {
        const osdDemoShoutWord_t *wd = &shoutWords[i];
        if (wd->step && !(wd->step & 1u)) {
            continue; // fading, the black half of its blink
        }
        osdDemoShoutPaint(wd);
        if ((int8_t)i == shoutNewest || wd->row >= rows || wd->row >= OSD_DEMO_MG_ROWS_MAX) {
            continue; // the newest name blinks white / black; every other one is grey
        }
        const uint8_t lvl = 3u; // grey
        const uint8_t c0 = (uint8_t)(wd->x / OSD_DEMO_SHOUT_LP_PER_CELL);
        const uint8_t c1 = (uint8_t)((wd->x + wd->w - 1u) / OSD_DEMO_SHOUT_LP_PER_CELL);
        for (uint8_t c = c0; c <= c1 && c < OSD_DEMO_CHARS_PER_LINE; c++) {
            shoutCellLevel[wd->row][c] = lvl;
        }
    }
}

// Row brightness gradient from the active (newest) row: RB white 120 / 100 / 90 / 80 %; on a
// snare the black of every row flashes up to 30 % and back over OSD_DEMO_SHOUT_SNARE_FIELDS
// fields (30, 20, 10, 0 %), as in scene 10.
#define OSD_DEMO_SHOUT_SNARE_FIELDS 8
static uint8_t shoutSnareFlash; // fields left

static void osdDemoShoutApplyRowBrightness(uint8_t rows)
{
    const int8_t active = (shoutNewest >= 0) ? (int8_t)shoutWords[shoutNewest].row : -1;
    const uint8_t black = (uint8_t)((shoutSnareFlash * 3u + OSD_DEMO_SHOUT_SNARE_FIELDS - 1u) / OSD_DEMO_SHOUT_SNARE_FIELDS);
    if (shoutSnareFlash) {
        shoutSnareFlash--;
    }
    for (uint8_t y = 0; y < rows && y < OSD_DEMO_MG_ROWS_MAX; y++) {
        uint8_t white = 3;
        if (active >= 0) {
            const uint8_t d = (uint8_t)((y > active) ? (y - active) : (active - y));
            white = (d >= 3u) ? 0u : (uint8_t)(3u - d);
        }
        const uint8_t reg = (uint8_t)((black << 2) | (3u - white)); // the RB value, cached per row
        if (reg != shoutRowWhite[y]) {
            shoutRowWhite[y] = reg;
            max7456BrightnessRow(y, black, white);
        }
    }
}

static void osdDemoShoutField(uint8_t cols, uint8_t rows, bool frameStart,
                              osdDemoRasterMotion_t *next)
{
    // Picture changes only on the field that starts a frame (both woven fields identical):
    // a beat that lands on the second field waits for the next frame start.
    if (osdDemoShoutBeats() > 0u) {
        shoutBeatPending = true;
    }
    {
        const uint8_t snare = chiptuneGetSnarePulse();
        if (snare != shoutSnareSeen) {
            shoutSnareSeen = snare;
            shoutSnarePending = true;
            shoutSnareFlash = OSD_DEMO_SHOUT_SNARE_FIELDS; // the blacks flash at once (every field)
        }
    }
    if (frameStart) {
        if (shoutSnarePending) {
            shoutSnarePending = false;
            osdDemoShoutDissolveOld(); // old names turn into stars on the snare
        }
        if (shoutBeatPending) {
            shoutBeatPending = false;
            osdDemoShoutBeat(cols, rows);
        } else if (shoutNewest >= 0) {
            shoutBlinkBlack = !shoutBlinkBlack; // newest name: white/black every frame
        }
        osdDemoShoutFade(rows, shoutHopPending);
        shoutHopPending = false;
        osdDemoShoutUpdateBars(rows);
        osdDemoShoutStarsStep(cols, rows);
    }
    {
        const uint8_t kick = osdDemoDrumPulse();
        if (kick != shoutKickSeen) {
            shoutKickSeen = kick;
            shoutKickEver = true;
            shoutLastKickMs = millis();
        }
    }
    // No temporal grey any more (pixels flipping black / white per field looked bad on the
    // glass): the fading names are grey.
    osdDemoShoutApplyRowBrightness(rows); // RB registers, in VBLANK, only rows that change
    next->vos = 0; // no bounce: the names dissolve into stars on the beat instead
}

// Grey stars flying at the viewer (3D): spawned far away anywhere over the whole field of view
// (a uniform spread, not a point in the middle), projected around a centre that drifts slowly,
// one or two logical pixels (3×3 px) by distance; behind the names (only where no text pixel).
#define OSD_DEMO_SHOUT_STARS     40
#define OSD_DEMO_SHOUT_WHITE_EVERY 8   // one star in this many is white (rare), the rest grey
#define OSD_DEMO_SHOUT_STAR_F    64    // focal length, logical px
#define OSD_DEMO_SHOUT_STAR_ZMAX 1024
#define OSD_DEMO_SHOUT_STAR_ZMIN 40
#define OSD_DEMO_SHOUT_STAR_DZ   14    // per frame (25 a second): ~2.8 s from far to near
#define OSD_DEMO_SHOUT_STAR_NEAR 280   // closer than this: 2 × 2 logical px

typedef struct {
    int16_t x, y; // world, logical px at z = F
    int16_t z;
} osdDemoShoutStar_t;

static osdDemoShoutStar_t shoutStars[OSD_DEMO_SHOUT_STARS];
static uint16_t shoutStarT;
static uint16_t shoutPartCount;
static int16_t shoutStarCx, shoutStarCy; // the projection centre now (logical px)

static void osdDemoShoutStarSpawn(osdDemoShoutStar_t *st, bool anyDepth)
{
    // Far plane covers the screen (120 × 96 logical px) with a margin: x·F/z within ±75, ±60.
    const int32_t z = anyDepth ? (int32_t)(OSD_DEMO_SHOUT_STAR_ZMIN + 60 + osdDemoRand() % (OSD_DEMO_SHOUT_STAR_ZMAX - 100))
                               : OSD_DEMO_SHOUT_STAR_ZMAX;
    const int32_t xs = 75 * OSD_DEMO_SHOUT_STAR_ZMAX / OSD_DEMO_SHOUT_STAR_F;
    const int32_t ys = 60 * OSD_DEMO_SHOUT_STAR_ZMAX / OSD_DEMO_SHOUT_STAR_F;
    st->x = (int16_t)((int32_t)(osdDemoRand() % (uint32_t)(2 * xs + 1)) - xs);
    st->y = (int16_t)((int32_t)(osdDemoRand() % (uint32_t)(2 * ys + 1)) - ys);
    st->z = (int16_t)z;
}

static void osdDemoShoutStarsInit(void)
{
    for (uint8_t i = 0; i < OSD_DEMO_SHOUT_STARS; i++) {
        osdDemoShoutStarSpawn(&shoutStars[i], true);
    }
    shoutStarT = 0;
    shoutPartCount = 0;
    shoutStarCx = OSD_DEMO_SHOUT_LP_COLS / 2;
    shoutStarCy = OSD_DEMO_SHOUT_LP_ROWS / 2;
    memset(shoutStarFb, 0, sizeof(shoutStarFb));
    memset(shoutStarWFb, 0, sizeof(shoutStarWFb));
}

// A name dissolves on the beat: every lit pixel becomes a star at a middle depth, placed so it
// projects where the pixel was, then flies at the viewer like the background stars.
static void osdDemoShoutDissolve(const osdDemoShoutWord_t *wd)
{
    const char *s = osdDemoShoutNames[wd->name];
    uint16_t cx0 = wd->x;
    for (uint8_t ci = 0; s[ci]; cx0 = (uint16_t)(cx0 + osdDemoShoutCharW(s[ci]) + 1u), ci++) {
        const uint8_t code = (uint8_t)s[ci];
        if (code >= OSD_DEMO_FONT_GLYPHS) {
            continue;
        }
        const uint8_t col0 = osdDemoShoutCharCol0(s[ci]), cw = osdDemoShoutCharW(s[ci]);
        for (uint8_t r = 0; r < OSD_DEMO_SHOUT_FONT_ROWS; r++) {
            const uint8_t bits = osdDemoFontBits[code][OSD_DEMO_TUNNEL_INK_Y0 + r];
            for (uint8_t px = 0; px < cw; px++) {
                // One pixel in four (a sparse diagonal grid): a quarter as many stars as the
                // name has pixels.
                if (!(bits & (0x80u >> (col0 + px))) || ((cx0 + px + 2u * r) & 3u) || shoutPartCount >= ARRAYLEN(shoutParts)) {
                    continue;
                }
                const int32_t lx = (int32_t)(cx0 + px);
                const int32_t ly = (int32_t)wd->row * OSD_DEMO_SHOUT_BANDS + r;
                const int32_t z = 220 + (int32_t)(osdDemoRand() % 140u);
                shoutParts[shoutPartCount].x = (int16_t)((lx - shoutStarCx) * z / OSD_DEMO_SHOUT_STAR_F);
                shoutParts[shoutPartCount].y = (int16_t)((ly - shoutStarCy) * z / OSD_DEMO_SHOUT_STAR_F);
                shoutParts[shoutPartCount].z = (int16_t)z;
                shoutPartCount++;
            }
        }
    }
}

// Once a frame: move, project into the star layer (the bar columns at the edges stay clear).
static void osdDemoShoutStarsStep(uint8_t cols, uint8_t rows)
{
    memset(shoutStarFb, 0, sizeof(shoutStarFb));
    memset(shoutStarWFb, 0, sizeof(shoutStarWFb));
    shoutStarT++;
    const int16_t w = (int16_t)((uint16_t)cols * OSD_DEMO_SHOUT_LP_PER_CELL);
    const int16_t h = (int16_t)((uint16_t)rows * OSD_DEMO_SHOUT_BANDS);
    const int16_t cx = (int16_t)(w / 2 + (osdDemoSineAt((uint8_t)(shoutStarT / 2u)) >> 4));
    const int16_t cy = (int16_t)(h / 2 + (osdDemoSineAt((uint8_t)(shoutStarT / 3u + 64u)) >> 5));
    shoutStarCx = cx;
    shoutStarCy = cy;
    const int16_t x0 = OSD_DEMO_SHOUT_LP_PER_CELL, x1 = (int16_t)(w - OSD_DEMO_SHOUT_LP_PER_CELL);
    // Dissolved names' pixels: same flight, gone when near or off the screen.
    for (uint16_t i = 0; i < shoutPartCount;) {
        int16_t *pz = &shoutParts[i].z;
        *pz = (int16_t)(*pz - OSD_DEMO_SHOUT_STAR_DZ);
        const int16_t sx = (*pz > 0) ? (int16_t)(cx + (int32_t)shoutParts[i].x * OSD_DEMO_SHOUT_STAR_F / *pz) : -1000;
        const int16_t sy = (*pz > 0) ? (int16_t)(cy + (int32_t)shoutParts[i].y * OSD_DEMO_SHOUT_STAR_F / *pz) : -1000;
        if (*pz < OSD_DEMO_SHOUT_STAR_ZMIN || sx < x0 || sx >= x1 || sy < 0 || sy >= h) {
            shoutParts[i] = shoutParts[--shoutPartCount];
            continue;
        }
        shoutStarFb[sy][sx >> 3] |= (uint8_t)(0x80u >> (sx & 7));
        i++;
    }
    for (uint8_t i = 0; i < OSD_DEMO_SHOUT_STARS; i++) {
        osdDemoShoutStar_t *st = &shoutStars[i];
        st->z = (int16_t)(st->z - OSD_DEMO_SHOUT_STAR_DZ);
        if (st->z < OSD_DEMO_SHOUT_STAR_ZMIN) {
            osdDemoShoutStarSpawn(st, false);
        }
        const int16_t sx = (int16_t)(cx + (int32_t)st->x * OSD_DEMO_SHOUT_STAR_F / st->z);
        const int16_t sy = (int16_t)(cy + (int32_t)st->y * OSD_DEMO_SHOUT_STAR_F / st->z);
        if (sx < x0 - 1 || sx >= x1 || sy < -1 || sy >= h) {
            osdDemoShoutStarSpawn(st, false); // left the screen
            continue;
        }
        const uint8_t size = (st->z < OSD_DEMO_SHOUT_STAR_NEAR) ? 2u : 1u;
        // One star in OSD_DEMO_SHOUT_WHITE_EVERY is white, the rest grey
        uint8_t (*fb)[OSD_DEMO_SHOUT_LP_COLS / 8u] = (i % OSD_DEMO_SHOUT_WHITE_EVERY) ? shoutStarFb : shoutStarWFb;
        for (uint8_t dy = 0; dy < size; dy++) {
            for (uint8_t dx = 0; dx < size; dx++) {
                const int16_t px = (int16_t)(sx + dx), py = (int16_t)(sy + dy);
                if (px >= x0 && px < x1 && py >= 0 && py < h) {
                    fb[py][px >> 3] |= (uint8_t)(0x80u >> (px & 7));
                }
            }
        }
    }
}

// A bar cell (1-px partial set): the left bar `px` clear px from the left, the right bar from the
// right, black on the rest; full = all clear (the VM1 grey). Never INV.
static uint8_t osdDemoShoutBarGlyph(uint16_t q8, bool right)
{
    const uint8_t px = (uint8_t)MIN(q8 >> 8, OSD_DEMO_CELL_W);
    if (px == 0u) {
        return OSD_DEMO_PIXEL_OFF;
    }
    if (px >= OSD_DEMO_CELL_W) {
        return OSD_DEMO_CLEAR;
    }
    return right ? (uint8_t)(OSD_DEMO_XFILL_CT_BASE + (OSD_DEMO_CELL_W - px) - 1u) // left black px
                 : (uint8_t)(OSD_DEMO_XFILL_C_BASE + px - 1u);
}

// Text cells from the 3-colour set: 4 logical pixels (mask bit 3 = left) white, or grey (clear
// on the VM1 grey) while a name fades — [grey][mask] → glyph | INV << 8.
static uint16_t shoutMap[2][16];

static void osdDemoShoutMapInit(void)
{
    for (uint8_t grey = 0; grey < 2u; grey++) {
        for (uint8_t m = 0; m < 16u; m++) {
            uint8_t p = 0;
            for (uint8_t k = 0; k < 4u; k++) {
                const bool on = (m >> (3u - k)) & 1u;
                p = (uint8_t)(p * 3u + (on ? (grey ? 1u : 2u) : 0u));
            }
            shoutMap[grey][m] = (uint16_t)(osdDemo3c[p] | ((uint16_t)osdDemo3cInv[p] << 8));
        }
    }
}

static void osdDemoShoutBand(uint8_t y, uint8_t b, uint8_t cols, uint8_t *glyphs)
{
    // The gap band (b = 5) is normally empty but carries name pixels that slid down.
    const uint8_t *line = shoutFb[(uint16_t)y * OSD_DEMO_SHOUT_BANDS + b];
    uint8_t blackL = 0xFF;
    uint8_t blackR = 0;
    if (shoutNewest >= 0 && shoutBlinkBlack && shoutWords[shoutNewest].row == y) {
        const osdDemoShoutWord_t *wd = &shoutWords[shoutNewest];
        blackL = (uint8_t)(wd->x / OSD_DEMO_SHOUT_LP_PER_CELL);
        blackR = (uint8_t)((wd->x + wd->w - 1u) / OSD_DEMO_SHOUT_LP_PER_CELL);
    }
    // Fading names (levels 1..3): grey, the scene's dark VM1 grey — no checker, no blinking.
    // Stars: grey pixels (a rare one white) where the cell has no text pixel.
    const uint8_t *lvlRow = shoutCellLevel[(y < OSD_DEMO_MG_ROWS_MAX) ? y : (OSD_DEMO_MG_ROWS_MAX - 1u)];
    const uint8_t *starLine = shoutStarFb[(uint16_t)y * OSD_DEMO_SHOUT_BANDS + b];
    const uint8_t *wstarLine = shoutStarWFb[(uint16_t)y * OSD_DEMO_SHOUT_BANDS + b];
    uint32_t inv = 0;
    for (uint8_t x = 1; x + 1u < cols; x++) {
        const uint8_t byte = line[x >> 1];
        const uint8_t nib = (x & 1u) ? (uint8_t)(byte & 0x0Fu) : (uint8_t)(byte >> 4);
        const uint8_t sbyte = starLine[x >> 1];
        const uint8_t wbyte = wstarLine[x >> 1];
        const uint8_t wstar = (uint8_t)(((x & 1u) ? (wbyte & 0x0Fu) : (wbyte >> 4)) & (uint8_t)~nib);
        const uint8_t star = (uint8_t)((((x & 1u) ? (sbyte & 0x0Fu) : (sbyte >> 4)) | wstar) & (uint8_t)~nib);
        const bool black = x >= blackL && x <= blackR; // whole cell belongs to this name
        uint16_t g = OSD_DEMO_PIXEL_OFF;
        if (!black && (nib | star)) {
            if (!star) {
                g = shoutMap[(lvlRow[x] >= 1u && lvlRow[x] <= 3u) ? 1u : 0u][nib];
            } else {
                const uint8_t textD = (lvlRow[x] >= 1u && lvlRow[x] <= 3u) ? 1u : 2u;
                uint8_t p = 0;
                for (uint8_t k = 0; k < 4u; k++) {
                    const uint8_t bit = (uint8_t)(3u - k);
                    p = (uint8_t)(p * 3u + (((nib >> bit) & 1u) ? textD
                                            : (((wstar >> bit) & 1u) ? 2u : (((star >> bit) & 1u) ? 1u : 0u))));
                }
                g = (uint16_t)(osdDemo3c[p] | ((uint16_t)osdDemo3cInv[p] << 8));
            }
        }
        glyphs[x] = (uint8_t)g;
        inv |= (uint32_t)(g >> 8) << x;
    }
    const uint8_t row = (y < OSD_DEMO_MG_ROWS_MAX) ? y : (uint8_t)(OSD_DEMO_MG_ROWS_MAX - 1u);
    const bool gap = b >= OSD_DEMO_SHOUT_FONT_ROWS; // bars stay black in the gap band
    // The bars over their row's cells from each edge (an arc: up to three cells at the corners)
    const uint8_t bc = MIN(osdDemoShoutBarCells(row, shoutBarRows), (uint8_t)(cols / 2u));
    // Peak marks (px from the edge; kept inside the bar cells), shown in a cell the bar leaves
    // empty — where the bar ends in the same cell the bar wins (no glyph has both).
    const uint16_t maxMark = (uint16_t)(bc * OSD_DEMO_CELL_W - 2u);
    const uint16_t pkL = MIN((uint16_t)(shoutPeakQ8[0][row] >> 8), maxMark);
    const uint16_t pkR = MIN((uint16_t)(shoutPeakQ8[1][row] >> 8), maxMark);
    for (uint8_t c = 0; c < bc; c++) {
        const uint16_t off = (uint16_t)((uint16_t)c * OSD_DEMO_CELL_W << 8);
        const uint16_t qL = (shoutBarQ8[0][row] > off) ? (uint16_t)(shoutBarQ8[0][row] - off) : 0u;
        const uint16_t qR = (shoutBarQ8[1][row] > off) ? (uint16_t)(shoutBarQ8[1][row] - off) : 0u;
        const uint8_t xl = c, xr = (uint8_t)(cols - 1u - c);
        uint8_t gL = osdDemoShoutBarGlyph(qL, false);
        uint8_t gR = osdDemoShoutBarGlyph(qR, true);
        if (gL == OSD_DEMO_PIXEL_OFF && pkL > 0u && pkL / OSD_DEMO_CELL_W == c) {
            gL = (uint8_t)(OSD_DEMO_PEAK_BASE + pkL % OSD_DEMO_CELL_W);          // columns j, j + 1
        }
        if (gR == OSD_DEMO_PIXEL_OFF && pkR > 0u && pkR / OSD_DEMO_CELL_W == c) {
            const uint8_t q = (uint8_t)(pkR % OSD_DEMO_CELL_W);                  // from the right
            gR = (uint8_t)(OSD_DEMO_PEAK_BASE + ((q < OSD_DEMO_CELL_W - 1u) ? OSD_DEMO_CELL_W - 2u - q : 0u));
        }
        glyphs[xl] = gap ? OSD_DEMO_PIXEL_OFF : gL;
        glyphs[xr] = gap ? OSD_DEMO_PIXEL_OFF : gR;
        inv &= ~((1u << xl) | (1u << xr));
    }
    mgBandInv = inv;
}

static void osdDemoShoutEnter(void)
{
    osdDemoApplyOsdmNow();
    max7456Brightness(0, 3);
    osdDemoBgType(DISPLAY_BACKGROUND_LTGRAY); // fading names and bars: clear pixels
    max7456SetHudMotionOffset(0, 0);
    osdDemoShoutMapInit();
    osdDemoShoutStarsInit();
    if (demoDisplay) {
        osdDemoFillRowsGlyphFast(demoDisplay->cols, demoDisplay->rows,
                                 OSD_DEMO_PIXEL_OFF, OSD_DEMO_PIXEL_OFF);
    } else {
        max7456FillScreen(OSD_DEMO_PIXEL_OFF);
    }
    osdDemoBg(true, OSD_DEMO_SHOUT_GREY_LEVEL);
    memset(mgSram, 0xFF, sizeof(mgSram)); // 0xFF never in a row → first field rewrites all
    mgLastEdgeTicks = 0;
    osdDemoShoutShuffle();
    memset(shoutFb, 0, sizeof(shoutFb));
    shoutWordCount = 0;
    shoutBeatNo = 0;
    shoutNewest = -1;
    memset(shoutBarQ8, 0, sizeof(shoutBarQ8));
    memset(shoutPeakQ8, 0, sizeof(shoutPeakQ8));
    memset(shoutPeakHold, 0, sizeof(shoutPeakHold));
    shoutLastPulse = chiptuneGetLinePulse();
    shoutLineAcc = 0;
    shoutLastBeatMs = millis();
    shoutBeatPending = true; // first name right away
    memset(shoutCellLevel, 4, sizeof(shoutCellLevel));
    shoutDirty = true;
    shoutKickSeen = osdDemoDrumPulse();
    shoutKickEver = false;
    shoutSnareSeen = chiptuneGetSnarePulse();
    shoutSnarePending = false;
    shoutBlinkBlack = false;
    shoutSnareFlash = 0;
    memset(shoutRowWhite, 0xFF, sizeof(shoutRowWhite)); // force the first gradient write
}

// === Unified mid-glyph raster engine (scenes 7, 8, 10) =======================================
// TIM5 compare interrupts drive the whole field — nothing busy-waits between bands, so the
// scheduler, CLI and USB keep running:
//  - VSYNC: STAT polled only in a short window around the edge predicted from the measured
//    field period (window doubles after a miss); VOS for this field; the scene's field hook;
//  - field phase: HSYNC phase + field parity in VBLANK, band-0 preload of as many rows as fit;
//  - every band: send the burst prepared one band earlier (IRQs masked for the burst only),
//    then build + plan + encode the next band and arm its race-the-beam start.
// A scene supplies the band geometry (bandLines × bands = 18), a field hook (ISR, VBLANK:
// animation + VOS for the next field), a band builder (ISR) and optionally a task hook (OSD
// task: heavy per-frame work such as the plasma masks). SPI2 TX DMA is not usable on the
// F411 board (DMA1 Stream4 belongs to motor 1 DShot), hence the polled TX-only burst.
typedef struct osdDemoIrqScene_s {
    uint8_t bandLines;
    uint8_t bands;
    bool *armed;
    void (*field)(uint8_t cols, uint8_t rows, bool frameStart, osdDemoRasterMotion_t *next);
    void (*band)(uint8_t y, uint8_t b, uint8_t cols, uint8_t *glyphs);
    void (*task)(void);
    // Optional: right after band (y, b)'s burst went out — about one line before the band.
    void (*sent)(uint8_t y, uint8_t b);
} osdDemoIrqScene_t;

static const osdDemoIrqScene_t *irqScene;
static bool irqRunning;
// Scene start: every row's level is reset (rows a scene never sets got whatever the previous
// scene left — a scene once showed the previous one's levels): the screen is filled
// black, all rows held at the normal level for FLASH_FIELDS fields (the scene's own level
// writes only set the wanted values meanwhile), then every row to its wanted level. (A grey
// flash — black lifted to 30 % — looked worse than plain black.)
#define OSD_DEMO_RB_FLASH_FIELDS 2
#define OSD_DEMO_RB_NORMAL       0x00u // black 0 %, white 120 %
#define OSD_DEMO_RB_FLASH_REG    OSD_DEMO_RB_NORMAL
static uint8_t irqRbFlash;      // fields left of the flash (+1: not started)
static timeMs_t charRbFlashEndMs; // the same for the character scenes (task side), 0 = none


typedef enum {
    IRQ_EV_VSYNC = 0,
    IRQ_EV_PHASE,
    IRQ_EV_ZSTART,
    IRQ_EV_ZEND,
} osdDemoIrqEvent_e;

#if defined(STM32F7)
void *timerFindTimerHandle(timerResource_t *tim); // timer_hal.c (no header)
#endif
static const timerHardware_t osdDemoIrqTimHw = {
    .tim = (timerResource_t *)TIM5,
    .tag = IO_TAG_NONE,
    .channel = 0, // channel 1 (TIM_Channel_1 / TIM_CHANNEL_1)
};
static timerEdgeHandlerRec_t osdDemoIrqEdgeRec;
static volatile uint8_t irqEvent;
static uint8_t irqCols;
static uint8_t irqRows;
static uint8_t irqYPre;
static uint32_t irqEdge;
static uint32_t irqPivot;
static uint32_t irqRow0;
static uint32_t irqLineQ16;
static uint32_t irqLineTicks;

// Video time → CPU ticks. Beam positions are times on the chip's own clock (27 MHz crystal: a
// line is 64 us, an OSD pixel 4 / 27 MHz), so they go through the measured line period — never
// through nominal CPU microseconds: the MCU clock may be off (HAKRCF411D without SYSTEM_HSE_MHZ
// runs on its HSI, ~2 % fast; with the crystal it is exact) and every window then slid by
// ~1.3 us a line. CPU ticks a video microsecond, Q16 (nominal until a line is measured).
static uint32_t osdDemoVideoUsQ16(void)
{
    return irqLineQ16 ? irqLineQ16 / OSD_DEMO_PAL_LINE_US : (clockMicrosToCycles(1) << 16);
}

static uint32_t osdDemoVideoNsToTicks(uint32_t ns)
{
    return (uint32_t)(((uint64_t)ns * osdDemoVideoUsQ16() / 1000u) >> 16);
}
static uint32_t irqTimRatioQ16;      // TIM5 ticks per DWT tick, Q16
static uint32_t osdDemoHsSnap(uint32_t predDwt, uint32_t lineTicks);
static bool hsCapOn;
static uint8_t irqGlyphs[2][OSD_DEMO_CHARS_PER_LINE];
static uint8_t irqGlyphCur;
static osdDemoRasterMotion_t irqMotion;
static uint8_t irqHosBase;   // the chip's HOS when the engine started (irqMotion.hos is relative)
static int8_t irqHosWritten;
static int8_t zdmaHosVal;    // the HOS offset the line words left in the chip this field
static uint8_t zdmaRbSh[OSD_DEMO_MG_ROWS_MAX]; // RB registers as the line words left them (rbLine)
static uint16_t irqVsyncLeadUs = OSD_DEMO_IRQ_VSYNC_LEAD_US; // widened after a miss

static void osdDemoIrqArm(uint32_t dwtAt)
{
    TIM_TypeDef *tim = TIM5;
    int32_t d = (int32_t)(dwtAt - getCycleCounter());
    const int32_t minLead = (int32_t)clockMicrosToCycles(1);
    if (d < minLead) {
        d = minLead; // already due: fire as soon as possible
    }
    const uint32_t dt = (uint32_t)(((uint64_t)(uint32_t)d * irqTimRatioQ16) >> 16);
    tim->SR = (uint16_t)~TIM_SR_CC1IF;
    tim->CCR1 = tim->CNT + dt;
}

static uint32_t osdDemoIrqFieldTicks(void)
{
    return (mgFieldLastTicks != 0u) ? mgFieldLastTicks : clockMicrosToCycles(OSD_DEMO_IRQ_NOMINAL_FIELD_US);
}

// Zones (scene option, HSYNC wired → by DMA): geometry and the per-field event bookkeeping.
static bool irqWantVsync;   // bands done, VSYNC wait deferred until the zone session closed
// Beam times below are video ns (the chip's crystal), found on HAKRCF411D: there in CPU ticks of
// its ~2 % fast HSI (8100 "ns" then = 7938 ns; the line read 65.309 "us").
#define OSD_DEMO_PX_PS      148148       // one OSD pixel: 4 / 27 MHz, ps
#define OSD_DEMO_FETCH_NS   56838        // the chip fetches the next line's codes from here
#define OSD_DEMO_X0_NS      7938         // HSYNC → the screen's left edge at HOS 32
// The left edge as the chip has it now: HOS moves the whole OSD a pixel a step.
static int32_t osdDemoX0Ns(void)
{
    return OSD_DEMO_X0_NS + ((int32_t)max7456GetHosApplied() - 32) * OSD_DEMO_PX_PS / 1000;
}
// Slot engine calibration (a chip constant, both fields alike with line 0 counted in HSYNCs):
// the chip's rows sit one line after line 0 (at 0 every row start showed a wrong line).
#define OSD_DEMO_ROW_SHIFT 1
#define OSD_DEMO_LINE0_HSYNC 20 // line 0 = the 20th HSYNC after VSYNC (default VOS; rowshift for others)
static uint16_t zdmaLine0N; // line 0's HSYNC number after VSYNC

static void osdDemoIrqScheduleVsync(void);
static void osdDemoZdmaArm(void);
static uint32_t zdmaLine0At;    // line 0's HSYNC (DWT) this field
static int32_t zdmaStartVsLine0; // last field's start against line 0, DWT ticks
static uint8_t osdDemoMgPlanRow(uint8_t row, const uint8_t *glyphs, uint32_t inv, uint8_t winL,
                                uint8_t winR, max7456SramSeg_t *seg);
static uint8_t osdDemoMgPlanMask(uint32_t chg, uint8_t winL, uint8_t winR, max7456SramSeg_t *seg);

// Arm whichever comes first: the zone session start or end, then the next VSYNC.
static void osdDemoIrqArmNext(void)
{
    if (zdmaStartPending) {
        irqEvent = IRQ_EV_ZSTART;
        osdDemoIrqArm(zdmaStartAt);
    } else if (zdmaEndPending) {
        irqEvent = IRQ_EV_ZEND;
        osdDemoIrqArm(zdmaEndAt);
    } else if (irqWantVsync) {
        irqWantVsync = false;
        osdDemoIrqScheduleVsync();
    }
}

static void osdDemoIrqScheduleVsync(void)
{
    if (zdmaStartPending || zdmaEndPending) {
        irqWantVsync = true; // after the zone session has closed
        osdDemoIrqArmNext();
        return;
    }
    irqEvent = IRQ_EV_VSYNC;
    osdDemoIrqArm(irqEdge + osdDemoIrqFieldTicks() - clockMicrosToCycles(irqVsyncLeadUs));
}

static void osdDemoIrqVsync(void)
{
    uint32_t edge = 0;
    max7456MidGlyphSpiBoost(false);
    if (max7456WaitVsyncFallingEdge(&edge, (timeUs_t)irqVsyncLeadUs + OSD_DEMO_IRQ_VSYNC_WINDOW_US)) {
        osdDemoMgTrackLinePeriod(edge, clockMicrosToCycles(OSD_DEMO_PAL_LINE_US));
        irqVsyncLeadUs = OSD_DEMO_IRQ_VSYNC_LEAD_US;
    } else {
        // Lost it: dead-reckon one field and look again next time with a wider window.
        irqVsyncLeadUs = (uint16_t)MIN((uint32_t)irqVsyncLeadUs * 2u, (uint32_t)OSD_DEMO_IRQ_VSYNC_LEAD_MAX_US);
        edge = irqEdge + osdDemoIrqFieldTicks();
        mgLastEdgeTicks = 0;
    }
    irqEdge = edge;
    mgStatFields++;
    irqLineQ16 = mgLineQ16;
    irqLineTicks = irqLineQ16 >> 16;
    // VOS moves the whole OSD by whole lines: write it in VBLANK and move the schedule with it.
    const int8_t vos = max7456WriteVosOffsetNow(irqMotion.vos);
    // HOS too, in VBLANK, when it changes (per-line HOS: every field, the lines moved it)
    if (irqMotion.hos != irqHosWritten || irqMotion.hosLine) {
        irqHosWritten = irqMotion.hos;
        zdmaHosVal = irqMotion.hos;
        max7456WriteHosNow((uint8_t)constrain((int)irqHosBase + irqHosWritten, 0, 63));
    }
    // Per-line RB: every row's register the level of the first line it governs (a row's from
    // the line before its first; the lines changed them last field)
    if (irqMotion.rbLine) {
        for (uint8_t r = 0; r < irqRows && r < OSD_DEMO_MG_ROWS_MAX; r++) {
            const uint8_t v = irqMotion.rbLine[r ? (uint16_t)r * OSD_DEMO_CELL_H - 1u : 0u];
            if (v != zdmaRbSh[r]) {
                zdmaRbSh[r] = v;
                max7456BrightnessRow(r, (uint8_t)((v >> 2) & 3u), (uint8_t)(3u - (v & 3u)));
            }
        }
    }
    // The pivot row's time after VSYNC in measured lines (it was VBLANK + 144 lines in nominal
    // CPU us, found on the ~2 % fast HSI: 164.14 real lines), the shift in video us
    irqPivot = edge + (uint32_t)(((uint64_t)OSD_DEMO_MG_PIVOT_AT_Q8 * irqLineQ16) >> 24)
        + (uint32_t)((int32_t)vos * (int32_t)irqLineTicks);
    irqRow0 = irqPivot + (uint32_t)osdDemoMgLineOffset(0, irqLineQ16);

    // Animation steps once per frame, on the field that starts it (both woven fields carry the
    // same picture). Fields alternate; the phase measurement below confirms the parity.
    const bool frameStart = !mgFieldFirst;
    if (osdmApplied != osdDemoOsdmWanted()) {
        osdDemoApplyOsdmNow(); // in VBLANK
    }
    osdDemoBgKeep();
    if (irqRbFlash) { // scene-start flash and row level reset, in VBLANK
        if (irqRbFlash == OSD_DEMO_RB_FLASH_FIELDS + 1u) {
            max7456RbHold(true, OSD_DEMO_RB_FLASH_REG);
        }
        if (--irqRbFlash == 0u) {
            max7456RbHold(false, 0);
        }
    }
    irqScene->field(irqCols, irqRows, frameStart, &irqMotion);
    irqWantVsync = false;
    osdDemoZdmaField(); // timing again once this field's phase is measured
    zdmaStartPending = zdmaEndPending = false;

    irqEvent = IRQ_EV_PHASE;
    osdDemoIrqArm(irqRow0 - (uint32_t)OSD_DEMO_MG_FIELD_PHASE_LINES * irqLineTicks - irqLineTicks * 3u / 8u);
}

static void osdDemoIrqPhase(void)
{
    const uint32_t capBefore = mgCapFields;
    osdDemoMgMeasureFieldPhase(irqEdge, irqRow0, irqLineTicks);
    max7456MidGlyphSpiBoost(true);
    // Line 0 by its HSYNC's number after VSYNC, like the chip counts — not by time: the
    // n-th edge has floor((edge − VSYNC) / line + ¼) = n in both fields (field 2's edges sit
    // ½ line later, ¼ keeps clear of VSYNC jitter). The phase line's captured edge and the
    // captured line period give that edge exactly; the number is fixed at the scene start.
    // (A model time wrapped to ±½ line picked either neighbour of field 2's edge per start.)
    const uint32_t line = irqLineTicks;
    if (mgCapFields != capBefore && line) {
        // The n-th HSYNC after VSYNC, from this field's own edge (Q16 lines after VSYNC):
        // field 1's edges sit just after a whole line (fraction ~0) → round; field 2's
        // half a line on (~0.6) → whole part. Either way ±0.4 line from the boundary, and
        // nothing kept from other fields (a mean per parity drifted on one misread field
        // and moved line 0 by a line now and then).
        const uint32_t xq = (uint32_t)(((uint64_t)(mgCapEdge - irqEdge) << 32) / irqLineQ16);
        const uint16_t fr = (uint16_t)xq;
        const bool mid = fr >= 0x4CCCu && fr < 0xCCCCu; // 0.3 .. 0.8: field 2's edges
        const int32_t np = (int32_t)((mid ? xq : xq + 0x8000u) >> 16);
        // Line 0 is a fixed HSYNC of the field — the chip starts its OSD at a set count
        // after VSYNC (VOS). Taken from the model it sat at x.5 lines (the model counts
        // 64 us lines, the camera's run ~65.2) and came out 20 or 21 per start.
        zdmaLine0N = OSD_DEMO_LINE0_HSYNC;
        const uint32_t e0 = mgCapEdge
            + (uint32_t)(int32_t)(((int64_t)((int32_t)zdmaLine0N - np) * (int64_t)irqLineQ16) >> 16);
        int32_t d = (int32_t)(e0 - (irqRow0 + (uint32_t)mgHsyncPhase));
        if (d > 3 * (int32_t)line || d < -3 * (int32_t)line) {
            d = 0; // far off the model: this field on the model
        }
        irqPivot += (uint32_t)d;
        irqRow0 += (uint32_t)d;
    }
    // The chip's rows against that line (a chip constant, per field parity).
    const int32_t sh = OSD_DEMO_ROW_SHIFT * (int32_t)line;
    irqPivot += (uint32_t)sh;
    irqRow0 += (uint32_t)sh;
    // VBLANK preload of band 0 for as many rows as fit (the slot engine's setup after it).
    const uint32_t fullRowTicks = mgBurstFixedTicks
        + (((10u + 2u * (uint32_t)irqCols) * mgByteTicksQ8) >> 8)
        + clockMicrosToCycles(500);
    // The bitmap engine's runs change Display SRAM behind the shadow's back: the preload
    // writes every cell (it diffed against a stale shadow — row 0 kept cells of the field
    // before, and the engine, trusting the preload, never wrote them again).
    memset(mgSram, 0xFF, sizeof(mgSram));
    irqYPre = 0;
    for (; irqYPre < irqRows; irqYPre++) {
        if ((int32_t)(getCycleCounter() - (irqRow0 - fullRowTicks)) > 0) {
            break;
        }
        irqGlyphCur ^= 1u;
        mgBandInv = 0;
        irqScene->band(irqYPre, 0, irqCols, irqGlyphs[irqGlyphCur]);
        (void)osdDemoMgWriteRow(irqYPre, irqGlyphs[irqGlyphCur], 0, irqCols, 0, false, true);
    }
    // Slot engine: every glyph and zone word of the field goes by DMA — no band events.
    // The session opens at line 0's HSYNC itself.
    osdDemoZdmaArm();
    const uint16_t lines = (uint16_t)irqRows * OSD_DEMO_CELL_H;
    zdmaLine0At = irqPivot + (uint32_t)(osdDemoMgLineOffset(0, irqLineQ16) + mgHsyncPhase);
    zdmaStartAt = zdmaLine0At - clockMicrosToCycles(10); // in line −1, before its gate closes
    zdmaEndAt = irqPivot + (uint32_t)(osdDemoMgLineOffset((uint16_t)(lines - 1u), irqLineQ16) + mgHsyncPhase)
                + zdmaGate + clockMicrosToCycles(2);
    zdmaStartPending = zdmaEndPending = true;
    osdDemoIrqScheduleVsync();
}

static void osdDemoIrqIsr(timerEdgeHandlerRec_t *cbRec, captureCompare_t capture)
{
    UNUSED(cbRec);
    UNUSED(capture);
    if (!irqRunning) {
        return;
    }
    switch (irqEvent) {
    case IRQ_EV_VSYNC:
        osdDemoIrqVsync();
        break;
    case IRQ_EV_PHASE:
        osdDemoIrqPhase();
        break;
    case IRQ_EV_ZSTART:
        zdmaStartPending = false;
        osdDemoZdmaStart();
        osdDemoIrqArmNext();
        break;
    case IRQ_EV_ZEND:
        zdmaEndPending = false;
        osdDemoZdmaEnd();
        osdDemoIrqArmNext();
        break;

    }
}

static void osdDemoIrqStop(void)
{
    if (!irqRunning) {
        return;
    }
    irqRunning = false;
    *irqScene->armed = false;
    TIM5->DIER &= ~TIM_DIER_CC1IE;
    irqRbFlash = 0;
    if (max7456RbHeld()) {
        max7456RbHold(false, 0);
    }
    osdDemoPbBuildIrqOn(false); // triple-buffered frame build off (not mid-build: thread mode runs only when it is idle)
    timerChannelConfigCallbacks(&osdDemoIrqTimHw, NULL, NULL);
    osdDemoZdmaEnd();
    (void)max7456WriteVosOffsetNow(0);
    if (irqHosWritten || irqMotion.hosLine) {
        max7456WriteHosNow(irqHosBase); // the line words moved it behind the driver's back
        irqMotion.hosLine = NULL;
        irqHosWritten = 0;
    }
    irqMotion.rbLine = NULL;
    max7456MidGlyphSpiEnd();
    // Back to the configured HOS/VOS (scene 8 forces HOS neutral while it runs).
    max7456ResetHudMotionOffset();
    max7456ApplyHudMotionNow();
}

static bool osdDemoIrqStart(const osdDemoIrqScene_t *scene)
{
    osdDemoIrqStop();
    memset(mgSramInv, 0, sizeof(mgSramInv)); // scenes seed mgSram with 0xFF (rewrite all)
    // The field phases start over with every scene (nothing a long run may have bent carries on)
    mgHsyncPhaseByValid[0] = mgHsyncPhaseByValid[1] = false;
    if (!demoDisplay) {
        startLastError = "no OSD display";
        return false;
    }
    if (!hsCapOn) {
        startLastError = osdDemoHsCapHint();
        return false;
    }
    osdDemoZdmaTiming();
    if (zdmaSpc < 2u) {
        startLastError = "OSD SPI too slow: a 16-bit word must fit half a character (SPI2 / SPI3, >= 18 MHz)";
        return false;
    }
    irqScene = scene;
    max7456RbResetWanted(OSD_DEMO_RB_NORMAL);
    irqRbFlash = OSD_DEMO_RB_FLASH_FIELDS + 1u;
    zdmaLine0N = 0; // line 0's HSYNC number: taken at the first field
    zdmaStartPending = zdmaEndPending = false;
    irqCols = MIN(demoDisplay->cols, OSD_DEMO_CHARS_PER_LINE);
    irqRows = MIN(demoDisplay->rows, OSD_DEMO_MG_ROWS_MAX);
    if (mgBurstFixedTicks == 0) {
        mgBurstFixedTicks = clockMicrosToCycles(OSD_DEMO_MG_BURST_FIXED_US);
    }
    if (mgByteTicksQ8 == 0) {
        mgByteTicksQ8 = (clockMicrosToCycles(30) << 8) / 36u;
    }

    // TIM5: free-running 32-bit at the timer clock; CC1 = next event time (frozen, no pin).
    RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
    __DSB();
    TIM5->CR1 = 0;
    TIM5->PSC = 0;
    TIM5->ARR = 0xFFFFFFFFu;
    TIM5->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC1P);
    TIM5->CCMR1 &= ~0x00FFu;
    TIM5->CCR1 = 0;
    TIM5->EGR = TIM_EGR_UG;
    TIM5->CR1 = TIM_CR1_CEN;
#if defined(STM32F7)
    // The HAL timer driver reaches the timer through its handle, set only by its own time base.
    ((TIM_HandleTypeDef *)timerFindTimerHandle(osdDemoIrqTimHw.tim))->Instance = TIM5;
#endif
    irqTimRatioQ16 = (uint32_t)(((uint64_t)timerClock(&osdDemoIrqTimHw) << 16)
                                / ((uint64_t)clockMicrosToCycles(1) * 1000000u));

    // Above the audio IRQs (the boosted fallback audio IRQ drops to timer priority meanwhile).
    NVIC_SetPriority(TIM5_IRQn, NVIC_PRIO_MAX >> (8u - __NVIC_PRIO_BITS));
    NVIC_EnableIRQ(TIM5_IRQn);

    max7456MidGlyphSpiBegin();
    // One blocking VSYNC lock seeds the prediction; from then on only short windows.
    uint32_t edge = 0;
    if (!max7456WaitVsyncFallingEdge(&edge, OSD_DEMO_PAL_VSYNC_TIMEOUT_US)) {
        max7456MidGlyphSpiEnd();
        return false;
    }
    mgLastEdgeTicks = 0;
    osdDemoMgTrackLinePeriod(edge, clockMicrosToCycles(OSD_DEMO_PAL_LINE_US));
    irqEdge = edge;
    irqVsyncLeadUs = OSD_DEMO_IRQ_VSYNC_LEAD_US;
    irqMotion.vos = 0;
    irqMotion.hos = 0;
    irqMotion.hosLine = NULL;
    irqMotion.rbLine = NULL;
    memset(zdmaRbSh, 0xFF, sizeof(zdmaRbSh));
    irqHosBase = max7456GetHosApplied();
    irqHosWritten = 0;
    zdmaHosVal = 0;

    *scene->armed = true;
    irqRunning = true;
    osdDemoIrqScheduleVsync(); // arm CC1 first, then enable its interrupt (no stale match)
    timerChannelEdgeHandlerInit(&osdDemoIrqEdgeRec, osdDemoIrqIsr);
    timerChannelConfigCallbacks(&osdDemoIrqTimHw, &osdDemoIrqEdgeRec, NULL);
    osdDemoZdmaInit();
    return true;
}


// --- Scene 7: 2×2 plasma — masks for a whole frame built by the OSD task (double buffer) ---

// 2×2 frame: per cell six 2-bit levels (see osdDemoMgCellMask), double buffered — plasmaMasks
// in the scene arena.

// Grey = a static checker of black / white mega-pixels: the grey bits shown white in this band
// (bits = the pixel columns of a cell, even count, so column parity is the same in every cell;
// `row` is the global raster row).
static inline uint16_t osdDemoMgGraySelRow(uint16_t row)
{
    return (row & 1u) ? 0x2Au : 0x15u;
}

// OSD task side: the scene's background work (e.g. the next plasma frame).
static void osdDemoIrqTask(void)
{
    if (!irqRunning || !irqScene->task) {
        return;
    }
    irqScene->task();
}

static inline uint16_t osdDemoMgGraySel(uint8_t y, uint8_t b)
{
    return osdDemoMgGraySelRow((uint16_t)y * OSD_DEMO_PLASMA2X2_BANDS + b);
}

static volatile uint8_t plasmaPlay;   // buffer being displayed
static volatile bool plasmaNextReady; // the other buffer holds a complete new frame

// Scene 7 row brightness (edge ramp), per frame buffer, written by the field hook.
static uint8_t plasmaRowLevel[2][OSD_DEMO_CHECKER_ROWS];
static uint8_t plasmaRbWritten[OSD_DEMO_CHECKER_ROWS];
static uint8_t plasmaRbNext;

// Bottom scroller: white 5×5 font in mega-pixels on black, in the last char row (font rows on
// bands 2..6), 1 mega-pixel per frame, entering from the right edge.
#define OSD_DEMO_PLASMA7_TEXT_MP_PER_S 25

// One band b of the scroller row: 2-bit levels per mega-pixel (white text on black) into cells.
static void osdDemoPlasma7TextBand(uint8_t b, uint8_t cols, uint16_t *cells)
{
    static const uint16_t len = sizeof(osdDemoPlasma7ScrollText) - 1u;
    const uint16_t mpCols = (uint16_t)cols * OSD_DEMO_CHECKER_CELL_W2;
    const uint32_t pos = (uint32_t)(millis() - fxStartMs) * OSD_DEMO_PLASMA7_TEXT_MP_PER_S / 1000u;
    const uint32_t period = (uint32_t)len * OSD_DEMO_FONT_ADVANCE;
    {
        const int8_t fr = (int8_t)b - 2; // font row 0..4 on bands 2..6
        for (uint8_t cx = 0; cx < cols; cx++) {
            uint16_t pat = 0;
            for (uint8_t bc = 0; fr >= 0 && fr < 5 && bc < OSD_DEMO_CHECKER_CELL_W2; bc++) {
                // Text column under this mega-pixel; negative = still right of the text start.
                const int32_t tc = (int32_t)pos + (int32_t)(cx * OSD_DEMO_CHECKER_CELL_W2 + bc) - (int32_t)mpCols;
                if (tc < 0) {
                    continue;
                }
                const uint32_t t = (uint32_t)tc % period;
                const uint8_t px = (uint8_t)(t % OSD_DEMO_FONT_ADVANCE);
                const uint8_t code = (uint8_t)osdDemoPlasma7ScrollText[t / OSD_DEMO_FONT_ADVANCE];
                if (px < OSD_DEMO_FONT_W && osdDemoFontPixel(code, px, (uint8_t)(OSD_DEMO_TUNNEL_INK_Y0 + fr))) {
                    pat |= (uint16_t)(3u << (2u * bc)); // white
                }
            }
            cells[cx] = pat;
        }
    }
}

static void osdDemoPlasma7Text(uint8_t buf, uint8_t cols, uint8_t y)
{
    for (uint8_t b = 0; b < OSD_DEMO_PLASMA2X2_BANDS; b++) {
        osdDemoPlasma7TextBand(b, cols, plasmaMasks[buf][y][b]);
    }
}

static void osdDemoPlasma2x2BuildFrame(uint8_t buf, uint8_t cols, uint8_t rows)
{
    plasmaPhase++;
    osdDemoPlasma2x2BuildLuts(cols, rows, plasmaPhase);
    // Plasma above, the scroller in the last char row.
    const uint8_t plasmaRows = (rows > 1u) ? (uint8_t)(rows - 1u) : rows;
    for (uint8_t y = 0; y < plasmaRows; y++) {
        for (uint8_t b = 0; b < OSD_DEMO_PLASMA2X2_BANDS; b++) {
            osdDemoPlasma2x2BuildMasks(y, b, cols, plasmaMasks[buf][y][b]);
        }
        // Row brightness: one RB step per row from the plasma's top and bottom edges (0, 1, 2),
        // full white (3) in between.
        const uint8_t edge = MIN(y, (uint8_t)(plasmaRows - 1u - y));
        plasmaRowLevel[buf][y] = (uint8_t)MIN(edge, 3u);
    }
    if (plasmaRows < rows) {
        osdDemoPlasma7Text(buf, cols, plasmaRows);
        plasmaRowLevel[buf][plasmaRows] = 3; // the scroller at full white
    }
}

static void osdDemoPlasma2x2Task(void)
{
    if (plasmaNextReady || !demoDisplay) {
        return;
    }
    osdDemoPlasma2x2BuildFrame((uint8_t)(plasmaPlay ^ 1u),
                               MIN(demoDisplay->cols, OSD_DEMO_CHECKER_COLS),
                               MIN(demoDisplay->rows, OSD_DEMO_CHECKER_ROWS));
    __DSB();
    plasmaNextReady = true;
}

static void osdDemoPlasma2x2Field(uint8_t cols, uint8_t rows, bool frameStart,
                                  osdDemoRasterMotion_t *next)
{
    UNUSED(cols);
    UNUSED(rows);
    if (frameStart && plasmaNextReady) {
        plasmaPlay ^= 1u;
        plasmaNextReady = false;
    }
    next->vos = 0;
}

// Three mega-pixels of 2-bit levels → white / grey / dark-grey bit triples (bits 0-2 / 3-5 / 6-8).
static uint16_t mgLevelLut[64];

static void osdDemoMgBuildLevelLut(void)
{
    for (uint8_t v = 0; v < 64u; v++) {
        uint16_t out = 0;
        for (uint8_t k = 0; k < 3u; k++) {
            const uint8_t lvl = (uint8_t)((v >> (2u * k)) & 3u);
            if (lvl == 3u) {
                out |= (uint16_t)(1u << k);
            } else if (lvl == 2u) {
                out |= (uint16_t)(1u << (k + 3u));
            } else if (lvl == 1u) {
                out |= (uint16_t)(1u << (k + 6u));
            }
        }
        mgLevelLut[v] = out;
    }
}

static void osdDemoPlasma2x2Band(uint8_t y, uint8_t b, uint8_t cols, uint8_t *glyphs)
{
    const uint16_t *cell = plasmaMasks[plasmaPlay][y][b];
    const uint16_t sel = osdDemoMgGraySel(y, b);
    // Dark grey: the grey phase on a fixed half of the mega-pixels (checker), black on the rest.
    const uint16_t half = ((y * OSD_DEMO_PLASMA2X2_BANDS + b) & 1u) ? 0x2Au : 0x15u;
    const uint16_t selDark = sel & half;
    uint32_t inv = 0;
    for (uint8_t x = 0; x < cols; x++) {
        const uint16_t lo = mgLevelLut[cell[x] & 0x3Fu];
        const uint16_t hi = mgLevelLut[(cell[x] >> 6) & 0x3Fu];
        const uint16_t w = (uint16_t)((lo & 7u) | ((hi & 7u) << 3));
        const uint16_t g = (uint16_t)(((lo >> 3) & 7u) | (((hi >> 3) & 7u) << 3));
        const uint16_t d = (uint16_t)(((lo >> 6) & 7u) | (((hi >> 6) & 7u) << 3));
        const uint16_t gl = px22Map[(w | (g & sel) | (d & selDark)) & 0x3Fu];
        glyphs[x] = (uint8_t)gl;
        inv |= (uint32_t)(gl >> 8) << x;
    }
    mgBandInv = inv;
}

// Scene 7 field: frame swap, row brightness from the white peaks of the frame on screen
// (VBLANK, changed rows + 1).
static void osdDemoPlasma7Field(uint8_t cols, uint8_t rows, bool frameStart,
                                osdDemoRasterMotion_t *next)
{
    osdDemoPlasma2x2Field(cols, rows, frameStart, next);
    // Row brightness (cache holds the RB byte)
    const uint8_t n = MIN(rows, OSD_DEMO_CHECKER_ROWS);
    const uint8_t *lvl = plasmaRowLevel[plasmaPlay];
    for (uint8_t y = 0; y < n; y++) {
        const uint8_t white = lvl[y];
        const uint8_t black = 0u;
        const uint8_t reg = (uint8_t)((black << 2) | (3u - white));
        if (reg != plasmaRbWritten[y]) {
            plasmaRbWritten[y] = reg;
            max7456BrightnessRow(y, black, white);
        }
    }
    if (n) {
        if (plasmaRbNext >= n) {
            plasmaRbNext = 0;
        }
        const uint8_t reg = plasmaRbWritten[plasmaRbNext];
        max7456BrightnessRow(plasmaRbNext, (uint8_t)((reg >> 2) & 3u), (uint8_t)(3u - (reg & 3u)));
        plasmaRbNext++;
    }
}

static const osdDemoIrqScene_t osdDemoPlasmaScene = {
    .bandLines = OSD_DEMO_PLASMA2X2_STEP,
    .bands = OSD_DEMO_PLASMA2X2_BANDS,
    .armed = &plasma2x2Armed,
    .field = osdDemoPlasma7Field,
    .band = osdDemoPlasma2x2Band,
    .task = osdDemoPlasma2x2Task,
};

// --- Scene 6: donut tunnel — rings of dashes on the 2×2 raster (scene 7's engine) ---------
// 12 rings at unit depth spacing fly towards the camera, one per beat; screen radius = F / z.
// Ring points outside a nearer ring's opening are behind the tube wall and not drawn. The tube
// bends: a ring's centre is offset by bend · (z / zFar)², so the nearest ring stays centred
// and the far ones curve away; the bend direction / amount drift on slow sines (turns).
// Each ring is 20 dashes along the circle, thicker and longer when near, slowly spinning.
// Built into scene 7's PX22 mask double buffer by the OSD task; shown by its band hook.
#define OSD_DEMO_DONUT_RINGS      12
#define OSD_DEMO_DONUT_DASHES     20
#define OSD_DEMO_DONUT_DASH_Q8    112   // dash length as a share of the dash period (/256)
#define OSD_DEMO_DONUT_F          80    // mega-pixels: ring radius at depth 1
#define OSD_DEMO_DONUT_DOT_GAP    8     // mega-pixels between the grey wall dots along a circle
#define OSD_DEMO_DONUT_MIN_R      11    // smaller (farther) rings only add clutter
#define OSD_DEMO_DONUT_Z0         90    // near clip (Q8 depth): rings closer than this respawn
#define OSD_DEMO_DONUT_BEAT_MS    500   // one ring per beat (tracker lines nearest this)
#define OSD_DEMO_DONUT_BEAT_Z     205   // depth (Q8) a ring is at on the beat: r ≈ 100 mp, the screen edge
#define OSD_DEMO_DONUT_BEND       90    // mega-pixels: far-ring centre offset at full bend
#define OSD_DEMO_DONUT_SPIN       24    // ring spin, sine-table steps per second
#define OSD_DEMO_DONUT_TWIST      9     // extra spin per ring (screw look)
#define OSD_DEMO_DONUT_W          (OSD_DEMO_CHECKER_COLS * OSD_DEMO_CHECKER_CELL_W2)   // 180

static bool donutArmed;
static uint8_t donutRowWhite[OSD_DEMO_MG_ROWS_MAX];
static uint8_t donutRbWritten[OSD_DEMO_MG_ROWS_MAX]; // RB register value written per row
static uint8_t donutSnareSeen;
static uint8_t donutSnareFields; // snare flash: fields left
#define OSD_DEMO_DONUT_SNARE_FIELDS 8
static uint8_t donutRbNext;
// The deep bass (the track's bass instrument two octaves down in this quiet part: a gate of
// volume 15 for 2–4 lines, then 0) shifts the picture right like an analogue hum: its level
// through an attack / release follower, so the gate's square edges become a fast swell and fall.
#define OSD_DEMO_DONUT_BASS_PERIOD 1000 // AY tone period from which a note counts as the bass (< 110 Hz)
#define OSD_DEMO_DONUT_BASS_DEEP   2800 // an octave lower still: 1.5× the shift
#define OSD_DEMO_DONUT_BASS_PX     8    // HOS swing at full volume, px each way
#define OSD_DEMO_DONUT_BASS_ATTACK 160  // follower step a field to the louder level (/256): ~2 fields
#define OSD_DEMO_DONUT_BASS_RELEASE 80  // and to the quieter one: ~8 fields to zero
#define OSD_DEMO_AY_CLOCK_HZ       1773400u // the emulated AY (ZX Spectrum 128): tone = clock / (16 · period)
static uint16_t donutBassQ8;    // the follower: HOS swing, px << 8
static uint16_t donutBassPrevQ8;
static uint32_t donutHumPhase;  // the bass tone's phase at the field's line 0 (sine-table steps << 16)
static uint32_t donutHumStep;   // its phase a line (64 us), same units: moves the bend from field to field
static int8_t donutHosLine[OSD_DEMO_MG_ROWS_MAX * OSD_DEMO_CELL_H];
static bool donutBgBlack;       // VM1 black while the lines bend: the edges they bare show it, not grey
#define OSD_DEMO_DONUT_GLOW_ROWS   4    // bright band half-width (char rows) between beats
#define OSD_DEMO_DONUT_GLOW_PULSE  5    // extra half-width (char rows) at the beat, decaying

// Tube bend now (−127..127 per axis): two incommensurate slow sines per axis.
static void osdDemoDonutBend(uint32_t ms, int16_t *bx, int16_t *by)
{
    *bx = (int16_t)((osdDemoSineAt((uint8_t)(ms / 47u)) * 3 + osdDemoSineAt((uint8_t)(ms / 113u + 60u))) / 4);
    *by = (int16_t)((osdDemoSineAt((uint8_t)(ms / 61u + 90u)) * 3 + osdDemoSineAt((uint8_t)(ms / 151u))) / 4);
}

static inline void osdDemoDonutDot(uint16_t (*m)[OSD_DEMO_PLASMA2X2_BANDS][OSD_DEMO_CHECKER_COLS],
                                   int16_t x, int16_t y, int16_t w, int16_t h)
{
    for (int16_t yy = y; yy < y + w; yy++) {
        if (yy < 0 || yy >= h) {
            continue;
        }
        uint16_t *row = m[yy / OSD_DEMO_PLASMA2X2_BANDS][yy % OSD_DEMO_PLASMA2X2_BANDS];
        for (int16_t xx = x; xx < x + w; xx++) {
            if (xx >= 0 && xx < OSD_DEMO_DONUT_W) {
                row[xx / OSD_DEMO_CHECKER_CELL_W2] |= (uint16_t)(3u << (2u * (xx % OSD_DEMO_CHECKER_CELL_W2))); // white
            }
        }
    }
}

static void osdDemoDonutBuildFrame(uint8_t buf, uint8_t cols, uint8_t rows)
{
    uint16_t (*m)[OSD_DEMO_PLASMA2X2_BANDS][OSD_DEMO_CHECKER_COLS] = plasmaMasks[buf];
    memset(plasmaMasks[buf], 0, sizeof(plasmaMasks[buf]));
    const int16_t w = (int16_t)cols * OSD_DEMO_CHECKER_CELL_W2;
    const int16_t h = (int16_t)rows * OSD_DEMO_PLASMA2X2_BANDS;
    const uint32_t ms = millis() - fxStartMs;
    const uint32_t span = OSD_DEMO_DONUT_RINGS * 256u;
    // Camera on the music: one ring spacing per beat, a ring reaching the edge on the beat.
    const uint32_t cam = (osdDemoBeatsQ8(OSD_DEMO_DONUT_BEAT_MS, fxStartMs)
                          + OSD_DEMO_DONUT_BEAT_Z - OSD_DEMO_DONUT_Z0) % span;
    int16_t bendX, bendY;
    osdDemoDonutBend(ms, &bendX, &bendY);
    const uint8_t spin = (uint8_t)(ms * OSD_DEMO_DONUT_SPIN / 1000u);

    // Rings near → far, so each one can be clipped by the openings of all nearer ones.
    uint32_t zq[OSD_DEMO_DONUT_RINGS];
    uint8_t order[OSD_DEMO_DONUT_RINGS];
    for (uint8_t i = 0; i < OSD_DEMO_DONUT_RINGS; i++) {
        zq[i] = ((uint32_t)i * 256u + span - cam) % span + OSD_DEMO_DONUT_Z0;
        uint8_t j = i;
        while (j > 0 && zq[order[j - 1]] > zq[i]) {
            order[j] = order[j - 1];
            j--;
        }
        order[j] = i;
    }
    // Every ring's circle (near → far), drawn or not: the openings of the tube.
    int16_t ringCx[OSD_DEMO_DONUT_RINGS], ringCy[OSD_DEMO_DONUT_RINGS], ringR[OSD_DEMO_DONUT_RINGS];
    int32_t ringR2[OSD_DEMO_DONUT_RINGS];
    for (uint8_t o = 0; o < OSD_DEMO_DONUT_RINGS; o++) {
        const uint8_t i = order[o];
        const int16_t r = (int16_t)(((int32_t)OSD_DEMO_DONUT_F * 256) / (int32_t)zq[i]);
        const int32_t u = (int32_t)((zq[i] - OSD_DEMO_DONUT_Z0) * 256u / span); // 0..255 near → far
        const int32_t u2 = u * u;                                                 // Q16
        ringCx[o] = (int16_t)(w / 2 + (int32_t)bendX * OSD_DEMO_DONUT_BEND * u2 / (127 * 65536));
        ringCy[o] = (int16_t)(h / 2 + (int32_t)bendY * OSD_DEMO_DONUT_BEND * u2 / (127 * 65536));
        ringR[o] = r;
        ringR2[o] = (int32_t)r * r;
    }

    // Sparse grey 2×2 dots on the tube wall, halfway between neighbouring rings (clear of the
    // white dashes), ~OSD_DEMO_DONUT_DOT_GAP mega-pixels apart along the circle, turning with
    // the rings — a few hundred dots a frame.
    for (uint8_t o = 1; o < OSD_DEMO_DONUT_RINGS; o++) {
        const int16_t r = (int16_t)((ringR[o - 1u] + ringR[o]) / 2);
        if (r < OSD_DEMO_DONUT_MIN_R || r > w) {
            continue;
        }
        const int16_t cx = (int16_t)((ringCx[o - 1u] + ringCx[o]) / 2);
        const int16_t cy = (int16_t)((ringCy[o - 1u] + ringCy[o]) / 2);
        const uint8_t v = 2u; // grey
        const uint16_t n = (uint16_t)constrain((6 * r) / OSD_DEMO_DONUT_DOT_GAP, 6, 96); // ≈ 2πr / gap
        const uint8_t a0 = (uint8_t)(spin + order[o] * OSD_DEMO_DONUT_TWIST + 128u / n);
        for (uint16_t k = 0; k < n; k++) {
            const uint8_t ang = (uint8_t)(a0 + (k * 256u) / n);
            const int16_t px = (int16_t)(cx + (r * osdDemoSineAt((uint8_t)(ang + 64u))) / 127);
            const int16_t py = (int16_t)(cy + (r * osdDemoSineAt(ang)) / 127);
            if (px < 0 || px >= w || py < 0 || py >= h) {
                continue;
            }
            bool hidden = false; // behind the wall of a nearer ring
            for (uint8_t q = 0; q + 1u < o && !hidden; q++) {
                const int32_t dx = px - ringCx[q];
                const int32_t dy = py - ringCy[q];
                hidden = dx * dx + dy * dy > ringR2[q];
            }
            if (!hidden) {
                uint16_t *cell = &m[py / OSD_DEMO_PLASMA2X2_BANDS][py % OSD_DEMO_PLASMA2X2_BANDS][px / OSD_DEMO_CHECKER_CELL_W2];
                *cell |= (uint16_t)((uint16_t)v << (2u * (px % OSD_DEMO_CHECKER_CELL_W2)));
            }
        }
    }

    for (uint8_t o = 0; o < OSD_DEMO_DONUT_RINGS; o++) {
        const uint8_t i = order[o];
        const int16_t r = ringR[o];
        const int16_t cx = ringCx[o];
        const int16_t cy = ringCy[o];
        if (r >= OSD_DEMO_DONUT_MIN_R && r <= w) {
            const int16_t thick = (int16_t)constrain(r / 16, 1, 5);
            // Q8 angle (65536 = one turn). Each dash start is k·turn/DASHES exactly: a rounded
            // period (256 / 20 = 12) left a 16-step (22.5°) gap in every ring.
            const uint32_t period = 65536u / OSD_DEMO_DONUT_DASHES;
            const uint16_t dashLen = (uint16_t)(period * OSD_DEMO_DONUT_DASH_Q8 / 256u);
            uint16_t stepQ8 = (uint16_t)((256u * 256u) / (6u * (uint16_t)r + 1u)); // ≈ 1 mp of arc
            if (stepQ8 < 64u) {
                stepQ8 = 64u;
            }
            const uint16_t a0 = (uint16_t)((uint8_t)(spin + i * OSD_DEMO_DONUT_TWIST) * 256u);
            for (uint8_t k = 0; k < OSD_DEMO_DONUT_DASHES; k++) {
                const uint16_t start = (uint16_t)(a0 + ((uint32_t)k * 65536u) / OSD_DEMO_DONUT_DASHES);
                for (uint16_t a = 0; a < dashLen; a = (uint16_t)(a + stepQ8)) {
                    const uint8_t ang = (uint8_t)((uint16_t)(start + a) >> 8);
                    const int16_t px = (int16_t)(cx + (r * osdDemoSineAt((uint8_t)(ang + 64u))) / 127);
                    const int16_t py = (int16_t)(cy + (r * osdDemoSineAt(ang)) / 127);
                    // Behind the tube wall: outside the opening of any nearer ring.
                    bool hidden = false;
                    for (uint8_t n = 0; n < o && !hidden; n++) {
                        const int32_t dx = px - ringCx[n];
                        const int32_t dy = py - ringCy[n];
                        hidden = dx * dx + dy * dy > ringR2[n];
                    }
                    if (!hidden) {
                        osdDemoDonutDot(m, (int16_t)(px - thick / 2), (int16_t)(py - thick / 2), thick, h);
                    }
                }
            }
        }
    }
}

static void osdDemoDonutTask(void)
{
    if (plasmaNextReady || !demoDisplay) {
        return;
    }
    osdDemoDonutBuildFrame((uint8_t)(plasmaPlay ^ 1u),
                           MIN(demoDisplay->cols, OSD_DEMO_CHECKER_COLS),
                           MIN(demoDisplay->rows, OSD_DEMO_CHECKER_ROWS));
    __DSB();
    plasmaNextReady = true;
}

// Row brightness: a bright band centred on the tube's far end (follows the bend up / down),
// widening on every beat and shrinking back over it. In VBLANK: changed rows + 1 refresh.
static void osdDemoDonutField(uint8_t cols, uint8_t rows, bool frameStart,
                              osdDemoRasterMotion_t *next)
{
    osdDemoPlasma2x2Field(cols, rows, frameStart, next);
    const uint8_t n = MIN(rows, OSD_DEMO_MG_ROWS_MAX);
    int16_t bx, by;
    osdDemoDonutBend(millis() - fxStartMs, &bx, &by);
    // Far-end centre in char rows, Q8 (a 2×2 row is 2 lines, a char row 9 of them).
    const int32_t hMp = (int32_t)n * OSD_DEMO_PLASMA2X2_BANDS;
    const int32_t farY = hMp / 2 + (int32_t)by * OSD_DEMO_DONUT_BEND / 127;
    const int32_t peakQ8 = farY * 256 / OSD_DEMO_PLASMA2X2_BANDS;
    const uint32_t frac = osdDemoBeatsQ8(OSD_DEMO_DONUT_BEAT_MS, fxStartMs) & 0xFFu;
    const uint32_t pulse = (256u - frac) * (256u - frac) / 256u;            // fast decay
    const int32_t widthQ8 = OSD_DEMO_DONUT_GLOW_ROWS * 256 + (int32_t)(OSD_DEMO_DONUT_GLOW_PULSE * pulse);
    // Snare: the black pixels flash — every row's black level up to 30 % and back down over
    // OSD_DEMO_DONUT_SNARE_FIELDS fields (30, 20, 10, 0 %).
    {
        const uint8_t snare = chiptuneGetSnarePulse();
        if (snare != donutSnareSeen) {
            donutSnareSeen = snare;
            donutSnareFields = OSD_DEMO_DONUT_SNARE_FIELDS;
        }
    }
    const uint8_t black = (uint8_t)((donutSnareFields * 3u + OSD_DEMO_DONUT_SNARE_FIELDS - 1u) / OSD_DEMO_DONUT_SNARE_FIELDS);
    if (donutSnareFields) {
        donutSnareFields--;
    }
    // Bass → HOS a line: the loudest low tone in the AY frame now, its level through the
    // follower, one sine period down the screen rolled on by the tone's phase each field — an
    // analogue hum bending the lines both ways, swelling and falling with the note.
    uint32_t bassQ8 = 0;
    uint16_t bassPeriod = 0;
    {
        uint8_t regs[16];
        if (chiptuneGetAyRegs(regs)) {
            for (uint8_t k = 0; k < 3u; k++) {
                if ((regs[7] >> k) & 1u) {
                    continue; // tone off
                }
                const uint16_t period = (uint16_t)(regs[2u * k] | ((regs[2u * k + 1u] & 0x0Fu) << 8));
                if (period < OSD_DEMO_DONUT_BASS_PERIOD) {
                    continue;
                }
                const uint8_t vol = (regs[8u + k] & 0x10u) ? 15u : (regs[8u + k] & 0x0Fu); // envelope: full
                uint32_t q = (uint32_t)vol * OSD_DEMO_DONUT_BASS_PX * 256u / 15u;
                if (period >= OSD_DEMO_DONUT_BASS_DEEP) {
                    q = q * 3u / 2u;
                }
                if (q > bassQ8) {
                    bassQ8 = q;
                    bassPeriod = period;
                }
            }
        }
    }
    if (bassQ8 > donutBassQ8) {
        donutBassQ8 = (uint16_t)(donutBassQ8 + ((bassQ8 - donutBassQ8) * OSD_DEMO_DONUT_BASS_ATTACK + 255u) / 256u);
    } else {
        donutBassQ8 = (uint16_t)(donutBassQ8 - ((donutBassQ8 - bassQ8) * OSD_DEMO_DONUT_BASS_RELEASE + 255u) / 256u);
    }
    if (bassPeriod) {
        // sine-table steps (256 a turn) a line: 64 us · clock / (16 · period), << 16
        donutHumStep = (uint32_t)(((uint64_t)OSD_DEMO_AY_CLOCK_HZ * OSD_DEMO_PAL_LINE_US * 256u * 65536u)
                                  / (16ull * 1000000u * bassPeriod));
    }
    {
        const uint16_t lines = (uint16_t)n * OSD_DEMO_CELL_H;
        const int32_t e0 = donutBassPrevQ8;
        const int32_t de = (int32_t)donutBassQ8 - e0;
        uint32_t ph = donutHumPhase;
        // One bend period down the screen; the tone's own phase rolls it from field to field
        const uint32_t lineStep = lines ? (256u << 16) / lines : 0u;
        for (uint16_t l = 0; l < lines; l++) {
            const int32_t env = e0 + de * l / (int32_t)lines; // the level ramps over the field
            const int32_t s = osdDemoSineAt((uint8_t)(ph >> 16));
            donutHosLine[l] = (int8_t)((env * s + (s >= 0 ? 16384 : -16384)) / 32768);
            ph += lineStep;
        }
        // phase on by a whole field (312.5 lines) for the next one
        donutHumPhase += donutHumStep * (OSD_DEMO_PAL_FIELD_LINES_X2 / 2u) + donutHumStep / 2u;
        donutBassPrevQ8 = donutBassQ8;
        // half a pixel and up: below it every line rounds to 0
        next->hosLine = (donutBassQ8 >= 128u || e0 >= 128) ? donutHosLine : NULL;
        next->hos = next->hosLine ? donutHosLine[0] : 0;
        const bool black = next->hosLine != NULL;
        if (black != donutBgBlack) { // the field the bend starts / ends (in VBLANK)
            donutBgBlack = black;
            osdDemoBg(true, black ? 0u : 7u); // the wall's grey dots go dark meanwhile
        }
    }
    for (uint8_t y = 0; y < n; y++) {
        const int32_t d = ABS((int32_t)y * 256 + 128 - peakQ8);
        const int32_t lvl = 3 - (d * 4) / (widthQ8 + 1);
        donutRowWhite[y] = (uint8_t)constrain(lvl, 0, 3);
        const uint8_t reg = (uint8_t)((black << 2) | (3u - donutRowWhite[y]));
        if (reg != donutRbWritten[y]) {
            donutRbWritten[y] = reg;
            max7456BrightnessRow(y, black, donutRowWhite[y]);
        }
    }
    if (n) {
        if (donutRbNext >= n) {
            donutRbNext = 0;
        }
        max7456BrightnessRow(donutRbNext, black, donutRowWhite[donutRbNext]);
        donutRbNext++;
    }
}

// One band of scene 6 (a mega-pixel row: 2 lines): a mega-pixel is 2 px × 2 lines, its value
// 0 black, 2 grey (clear on the VM1 grey: the grey set, osdDemoGreyCode), 3 white. A cell line
// no glyph shows (grey between black and white, a grey mask the set lacks) gets its greys as
// its colour — never a white dot near the dashes. Per half cell (3 mega-pixels, 6 bits), built
// at the scene entry: its base-3 digits as is / greys black / greys white and what it holds —
// two table reads a cell (a per-pixel loop in the DMA interrupt slowed the frames down).
typedef struct {
    uint8_t idx, idxB, idxW; // 0..26
    uint8_t grey, black;
} osdDemoDonutHalf_t;
static osdDemoDonutHalf_t donutHalf[64];

static void osdDemoDonutHalves(void)
{
    for (uint8_t h = 0; h < 64u; h++) {
        osdDemoDonutHalf_t *e = &donutHalf[h];
        e->idx = e->idxB = e->idxW = 0;
        e->grey = e->black = 0;
        for (uint8_t k = 0; k < 3u; k++) { // left mega-pixel first (bits 1..0)
            const uint8_t v = (uint8_t)((h >> (2u * k)) & 3u);
            const uint8_t d = (v == 3u) ? 2u : ((v == 2u) ? 1u : 0u);
            e->grey |= (d == 1u);
            e->black |= (d == 0u);
            e->idx = (uint8_t)(e->idx * 3u + d);
            e->idxB = (uint8_t)(e->idxB * 3u + ((d == 1u) ? 0u : d));
            e->idxW = (uint8_t)(e->idxW * 3u + ((d == 1u) ? 2u : d));
        }
    }
}

static void osdDemoDonutBand(uint8_t y, uint8_t b, uint8_t cols, uint8_t *glyphs)
{
    const uint16_t *cell = plasmaMasks[plasmaPlay][y][b];
    const uint16_t *lut = bmLut[0];
    uint32_t inv = 0;
    for (uint8_t x = 0; x < cols; x++) {
        const osdDemoDonutHalf_t *l = &donutHalf[cell[x] & 0x3Fu];
        const osdDemoDonutHalf_t *r = &donutHalf[(cell[x] >> 6) & 0x3Fu];
        uint16_t g = lut[l->idx * 27u + r->idx];
        if (l->grey | r->grey) {
            const uint8_t code = (uint8_t)g;
            if (!osdDemoIsGreyCode(code)) {
                g = (l->black | r->black) ? lut[l->idxB * 27u + r->idxB] : lut[l->idxW * 27u + r->idxW];
            }
        }
        glyphs[x] = (uint8_t)g;
        inv |= (uint32_t)(g >> 8) << x;
    }
    mgBandInv = inv;
}

static const osdDemoIrqScene_t osdDemoDonutScene = {
    .bandLines = OSD_DEMO_PLASMA2X2_STEP,
    .bands = OSD_DEMO_PLASMA2X2_BANDS,
    .armed = &donutArmed,
    .field = osdDemoDonutField,
    .band = osdDemoDonutBand,
    .task = osdDemoDonutTask,
};

// --- Scene 9: megacolor — 4-level pictures on the 2×2 raster (scene 7's engine) -----------
// 180×144 pictures from tools/gen_osd_megacolor.py (RLE, 2-bit levels: black, dark grey, grey,
// white) decoded straight into the 2×2 frame buffer. They share the scene length equally and
// dissolve into each other (per-mega-pixel thresholds) over the first sixth of each slot. The
// Betaflight sprite (black / white, keyed) flies across at the height of the old picture's wasp:
// fast in from the left, braking to a slow drift past the centre, then speeding off to the right
// (x = cubic of time), OSD_DEMO_MEGA_FLY_MS from the scene start whatever the scene length. A
// frame is rebuilt only when the picture mix or the sprite x changes.
#include "io/osd_demo_mega.inc"

#define OSD_DEMO_MEGA_FADE_Q8  43    // dissolve length, share of a picture slot (/256)
#define OSD_DEMO_MEGA_SPRITE_Y      17   // mega-pixels: sprite top
#define OSD_DEMO_MEGA_FLY_MS        2880 // the flight from the scene start: in, braking past the centre, out
#define OSD_DEMO_MEGA_DRIFT         18   // mega-pixels each side of the centre in the slow middle
#define OSD_DEMO_MEGA_NO_SPRITE     INT16_MIN

static bool megaArmed;
static uint8_t megaBuildBuf; // the code buffer a frame is built into
static uint8_t megaBuiltA = 0xFF, megaBuiltB = 0xFF;
static uint16_t megaBuiltMix = 0xFFFF;
static int16_t megaBuiltX = OSD_DEMO_MEGA_NO_SPRITE;

typedef struct osdDemoRle_s {
    const uint8_t *p;
    uint8_t left;
    uint8_t lvl;
} osdDemoRle_t;

static inline uint8_t osdDemoRleNext(osdDemoRle_t *r)
{
    if (!r->left) {
        const uint8_t b = *r->p++;
        r->left = (uint8_t)((b >> 2) + 1u);
        r->lvl = (uint8_t)(b & 3u);
    }
    r->left--;
    return r->lvl;
}

static inline uint16_t osdDemoMegaBw(uint8_t white) // white mask → black / white glyph | INV << 8
{
    return (white & 0x20u) ? (uint16_t)((OSD_DEMO_BM_BASE + (~white & 0x1Fu)) | 0x100u)
                           : (uint16_t)(OSD_DEMO_BM_BASE + white);
}

static inline uint16_t osdDemoMegaGrey(uint8_t grey, bool onWhite) // a grey-set pattern on black / white
{
    if (!grey) {
        return osdDemoMegaBw(onWhite ? 0x3Fu : 0u);
    }
    const uint8_t code = osdDemoGreyCode(grey);
    return (uint16_t)(code | ((onWhite && code != OSD_DEMO_CLEAR) ? 0x100u : 0u));
}

static inline uint8_t osdDemoMegaBits(uint8_t m)
{
    uint8_t n = 0;
    for (; m; m &= (uint8_t)(m - 1u)) {
        n++;
    }
    return n;
}

// The pattern nearest a cell line the glyphs cannot show (masks: bit 5 = left): among black /
// white (any) and the grey set's patterns on black or on white, the least level error (black 0,
// grey 1, white 2), then the fewest pixels changed. One pattern for both lines of the 2×2 pixel —
// splitting it over the two lines showed as torn edges on trees, clouds and the sprite.
static uint16_t osdDemoMegaNearest(uint8_t white, uint8_t grey, uint8_t black)
{
    const uint8_t nGrey = osdDemoMegaBits(grey);
    uint16_t best = 0;
    uint16_t bestKey = 0xFFFF;
    // Black / white: a grey pixel is off by 1 either way; the white mask chosen = the source's
    {
        const uint16_t key = (uint16_t)(nGrey * 16u + nGrey); // err · 16 + changed
        bestKey = key;
        best = osdDemoMegaBw(white);
    }
    for (uint8_t i = 0; i <= OSD_DEMO_GREY_ALL; i++) {
        const uint8_t q = (i < OSD_DEMO_GREY_ALL) ? osdDemoGreyMask[i] : 0x3Fu;
        for (uint8_t onWhite = 0; onWhite < 2u; onWhite++) {
            const uint8_t wrongBase = onWhite ? (uint8_t)(black & (uint8_t)~q) : (uint8_t)(white & (uint8_t)~q);
            const uint8_t miss = (uint8_t)(grey & (uint8_t)~q);   // grey shown as the base: off by 1
            const uint8_t over = (uint8_t)((white | black) & q);  // black / white shown grey: off by 1
            const uint8_t err = (uint8_t)(osdDemoMegaBits(miss) + osdDemoMegaBits(over) + 2u * osdDemoMegaBits(wrongBase));
            const uint8_t chg = (uint8_t)(osdDemoMegaBits(miss) + osdDemoMegaBits(over) + osdDemoMegaBits(wrongBase));
            const uint16_t key = (uint16_t)(err * 16u + chg);
            if (key < bestKey) {
                bestKey = key;
                best = osdDemoMegaGrey(q, onWhite != 0u);
            }
        }
    }
    return best;
}

// One row of 2×2 pixels (cells: six 2-bit levels each, left first in bits 1..0) → the codes of
// its two lines (the same pattern on both). Black / grey / white (levels 1 and 2 both grey):
// black / white exactly, grey on one colour as the grey set has it, anything else the nearest
// pattern (osdDemoMegaNearest).
// The pattern of a half-cell line of black / grey / white (base-3 digits, left first) → glyph |
// INV << 8, | 0x8000 once known: the nearest search runs once per pattern, not per cell (the
// rotozoom mixes all three in most cells every frame).
static uint16_t megaPatCache[729];

static void osdDemoMegaRow(const uint16_t *cells, uint8_t cols, uint16_t line)
{
    uint8_t *c0 = bmCode[megaBuildBuf][line], *c1 = bmCode[megaBuildBuf][line + 1u];
    uint32_t inv = 0;
    for (uint8_t x = 0; x < cols; x++) {
        uint8_t white = 0, grey = 0, black = 0;
        uint16_t idx = 0;
        for (uint8_t k = 0; k < 6u; k++) {
            const uint8_t lvl = (uint8_t)((cells[x] >> (2u * k)) & 3u);
            const uint8_t bit = (uint8_t)(0x20u >> k);
            if (lvl == 3u) {
                white |= bit;
                idx = (uint16_t)(idx * 3u + 2u);
            } else if (lvl) {
                grey |= bit;
                idx = (uint16_t)(idx * 3u + 1u);
            } else {
                black |= bit;
                idx = (uint16_t)(idx * 3u);
            }
        }
        uint16_t g = megaPatCache[idx];
        if (g & 0x8000u) {
            g &= 0x1FFu;
        } else {
            if (!grey) {
                g = osdDemoMegaBw(white);
            } else if ((!white || !black) && osdDemoGreyCode(grey)) {
                g = osdDemoMegaGrey(grey, white != 0u);
            } else if (white && black && osdDemoGrey3Of(idx) != 0xFFFFu) {
                g = osdDemoGrey3Of(idx); // black, grey and white: a three-colour glyph
            } else {
                g = osdDemoMegaNearest(white, grey, black);
            }
            megaPatCache[idx] = (uint16_t)(g | 0x8000u);
        }
        c0[x] = c1[x] = (uint8_t)g;
        inv |= (uint32_t)(g >> 8) << x;
    }
    bmInv[megaBuildBuf][line] = inv;
    bmInv[megaBuildBuf][line + 1u] = inv;
}

// Picture a, dissolving into b by mix (0 = all a, 256 = all b), the sprite at x — row by row
// straight into the line codes of buffer buf (no mask frame).
static void osdDemoMegaBuildFrame(uint8_t buf, uint8_t a, uint8_t b, uint16_t mix, int16_t spriteX)
{
    osdDemoRle_t ra = { osdDemoMegaPics[a], 0, 0 };
    osdDemoRle_t rb = { osdDemoMegaPics[b], 0, 0 };
    const uint8_t cols = OSD_DEMO_CHECKER_COLS;
    const int32_t screenW = OSD_DEMO_CHECKER_COLS * OSD_DEMO_CHECKER_CELL_W2;
    megaBuildBuf = buf;
    for (uint16_t my = 0; my < OSD_DEMO_CHECKER_ROWS * OSD_DEMO_PLASMA2X2_BANDS; my++) {
        uint16_t cells[OSD_DEMO_CHECKER_COLS];
        for (uint8_t cx = 0; cx < cols; cx++) {
            uint16_t pat = 0;
            for (uint8_t bc = 0; bc < OSD_DEMO_CHECKER_CELL_W2; bc++) {
                uint8_t lvl = osdDemoRleNext(&ra);
                if (mix) {
                    const uint8_t lb = osdDemoRleNext(&rb);
                    const uint32_t mx = (uint32_t)cx * OSD_DEMO_CHECKER_CELL_W2 + bc;
                    if ((((mx * 31u + (uint32_t)my * 17u) * 2654435761u) >> 24) < mix) {
                        lvl = lb;
                    }
                }
                pat |= (uint16_t)((uint16_t)lvl << (2u * bc));
            }
            cells[cx] = pat;
        }
        // The sprite over this row
        const int32_t sy = (int32_t)my - OSD_DEMO_MEGA_SPRITE_Y;
        if (spriteX != OSD_DEMO_MEGA_NO_SPRITE && sy >= 0 && sy < OSD_DEMO_MEGA_SPRITE_H) {
            const uint8_t *src = &osdDemoMegaSprite[sy * OSD_DEMO_MEGA_SPRITE_STRIDE];
            for (uint8_t sx = 0; sx < OSD_DEMO_MEGA_SPRITE_W; sx++) {
                const uint8_t code = (uint8_t)((src[sx >> 2] >> (2u * (sx & 3u))) & 3u);
                const int32_t mx = spriteX + sx;
                if (!code || mx < 0 || mx >= screenW) {
                    continue;
                }
                const uint8_t sh = (uint8_t)(2u * (mx % OSD_DEMO_CHECKER_CELL_W2));
                uint16_t *c = &cells[mx / OSD_DEMO_CHECKER_CELL_W2];
                *c = (uint16_t)((*c & ~(3u << sh)) | ((code == 2u ? 3u : 0u) << sh));
            }
        }
        osdDemoMegaRow(cells, cols, (uint16_t)(my * OSD_DEMO_PLASMA2X2_STEP));
    }
}

// Which pictures now: slot by the scene's progress (song rows, or the clock without music).
static void osdDemoMegaWant(uint8_t *a, uint8_t *b, uint16_t *mix, uint32_t *sceneQ16)
{
    const uint32_t n = OSD_DEMO_MEGA_PICS;
    uint32_t progQ8 = 0; // picture slots × 256
    uint32_t rowQ8;
    uint8_t speed;
    const uint32_t lenQ8 = (uint32_t)osdDemoFxPatterns(OSD_DEMO_FX_MEGA) * OSD_DEMO_PATTERN_ROWS * 256u;
    if (fxEndByRow && lenQ8 && osdDemoSongClockQ8(&rowQ8, &speed)) {
        const uint32_t start = fxEndRowQ8 - lenQ8;
        *sceneQ16 = (rowQ8 > start) ? (uint32_t)(((uint64_t)(rowQ8 - start) * 65536u) / lenQ8) : 0u;
    } else {
        const timeMs_t len = osdDemoFxDurationMs(OSD_DEMO_FX_MEGA);
        *sceneQ16 = len ? (uint32_t)(((uint64_t)(millis() - fxStartMs) * 65536u) / len) : 0u;
    }
    progQ8 = (*sceneQ16 * n) >> 8;
    const uint32_t slot = (progQ8 >> 8) % n;   // held: keeps cycling
    const uint32_t within = progQ8 & 0xFFu;
    *b = (uint8_t)slot;
    *a = (uint8_t)slot;
    *mix = 0;
    if ((progQ8 >> 8) > 0u && within < OSD_DEMO_MEGA_FADE_Q8) {
        *a = (uint8_t)((slot + n - 1u) % n);
        *mix = (uint16_t)(within * 256u / OSD_DEMO_MEGA_FADE_Q8);
    }
}

// Sprite left edge `ms` after the scene start, or NO_SPRITE after the flight / off screen — its
// own time, not a share of the scene (the scene's length is set on the track).
// u runs -1..1 over the flight; x = centre + K·u³ + DRIFT·u, from fully left to fully right.
static int16_t osdDemoMegaSpriteX(uint32_t ms)
{
    const int32_t w = OSD_DEMO_MEGA_SPRITE_W;
    const int32_t screenW = OSD_DEMO_CHECKER_COLS * OSD_DEMO_CHECKER_CELL_W2;
    if (ms >= OSD_DEMO_MEGA_FLY_MS) {
        return OSD_DEMO_MEGA_NO_SPRITE;
    }
    const int64_t u = (int64_t)ms * 8192 / OSD_DEMO_MEGA_FLY_MS - 4096; // Q12
    const int64_t k2 = (screenW + w) - 2 * OSD_DEMO_MEGA_DRIFT;                        // 2·K
    const int64_t x2 = (screenW - w) + ((k2 * u * u * u) >> 36) + ((2 * OSD_DEMO_MEGA_DRIFT * u) >> 12);
    const int32_t x = (int32_t)(x2 >> 1);
    return (x <= -w || x >= screenW) ? OSD_DEMO_MEGA_NO_SPRITE : (int16_t)x;
}

static void osdDemoMegaTask(void)
{
    if (plasmaNextReady || !demoDisplay) {
        return;
    }
    uint8_t a, b;
    uint16_t mix;
    uint32_t sceneQ16;
    osdDemoMegaWant(&a, &b, &mix, &sceneQ16);
    if (mix == 0u) {
        a = b; // still picture: one decode stream
    }
    const int16_t x = osdDemoMegaSpriteX(millis() - fxStartMs);
    if (a == megaBuiltA && b == megaBuiltB && mix == megaBuiltMix && x == megaBuiltX) {
        return; // nothing new: the engine keeps showing the current buffer
    }
    osdDemoMegaBuildFrame((uint8_t)(plasmaPlay ^ 1u), a, b, mix, x);
    megaBuiltA = a;
    megaBuiltB = b;
    megaBuiltMix = mix;
    megaBuiltX = x;
    __DSB();
    plasmaNextReady = true;
}

static void osdDemoBmBand(uint8_t y, uint8_t b, uint8_t cols, uint8_t *glyphs);

// Scene 9 on the bitmap engine: every line its own codes (built by the task, osdDemoMegaRow)
static const osdDemoIrqScene_t osdDemoMegaScene = {
    .bandLines = 1,
    .bands = OSD_DEMO_CELL_H,
    .armed = &megaArmed,
    .field = osdDemoPlasma2x2Field,
    .band = osdDemoBmBand,
    .task = osdDemoMegaTask,
};

// --- Scene 11: rotozoom — scene 9's sprite tiled over grey, turned and zoomed -----------------
// The Betaflight sprite (black / white; clear = the VM1 grey) repeated on a tile, sampled through
// a turning, zooming map at the 2×2 raster's 180×144 mega-pixels and turned into line codes by
// scene 9's pipeline (osdDemoMegaRow) on the bitmap engine, all locked to the music (a beat = 8
// tracker lines here: kick on 0, snare on 8). The field turns evenly, a sixteenth a beat; it flies
// over the plane on a slowly turning heading; the zoom swings out and back over a bar and bounces
// in on every beat — accelerating into the hit, turning on it sharply (|sin|), never resting.
// The grey (the clear pixels on the VM1 background) lights to 49 % on every beat, a step below
// (42 %) between.
#define OSD_DEMO_P3_LP           4     // 3×3 logical pixels a cell (scene 10, the bitmap plasma's shape)
#define OSD_DEMO_ROTO_TILE_W     80    // texels: the sprite (65 × 52) and its gap
#define OSD_DEMO_ROTO_TILE_H     64
#define OSD_DEMO_ROTO_BEAT_MS    500   // a beat (the tracker lines nearest it)
#define OSD_DEMO_ROTO_TURNS_BEAT 16    // a beat turns 1/16
#define OSD_DEMO_ROTO_ZOOM_NEAR  0.7f  // texels a mega-pixel, near: the sprite at 1.4× its size
#define OSD_DEMO_ROTO_ZOOM_FAR   2.4f  // far: ~5 sprites across
#define OSD_DEMO_ROTO_BOUNCE     0.35f // the beat's bounce in, log zoom (×0.7 at the hit)
#define OSD_DEMO_ROTO_FLY        20.0f // texels a beat the centre travels over the plane
#define OSD_DEMO_ROTO_HEAD_BEATS 32    // beats the heading takes for a full turn
#define OSD_DEMO_ROTO_LEAD_Q8    20    // beat Q8: a frame is shown ~40 ms after it is built
#define OSD_DEMO_ROTO_GREY_BEAT  7     // VM1 grey (7 % steps) on the beat
#define OSD_DEMO_ROTO_GREY       6     // between
#define OSD_DEMO_ROTO_GREY_Q8    60    // beat Q8 the beat's grey lasts (~120 ms)

static bool rotoArmed;
static volatile uint8_t rotoGreyWant = OSD_DEMO_ROTO_GREY; // for the frame built last
static uint8_t rotoGreyLevel;        // written last

static void osdDemoRotoBuildFrame(uint8_t buf)
{
    // Beats since the scene start (Q8, music-locked; a free clock without music)
    const uint32_t beatsQ8 = osdDemoBeatsQ8(OSD_DEMO_ROTO_BEAT_MS, fxStartMs) + OSD_DEMO_ROTO_LEAD_Q8;
    const float pb = (float)(beatsQ8 & 0xFFu) * (1.0f / 256.0f);
    const float pbar = (float)(beatsQ8 & 0x3FFu) * (1.0f / 1024.0f);
    // the turn so far (whole turns dropped: the float keeps its precision); even, with the beat
    const float turns = (float)(beatsQ8 % (256u * OSD_DEMO_ROTO_TURNS_BEAT)) / (256.0f * OSD_DEMO_ROTO_TURNS_BEAT);
    const float a = (turns - floorf(turns + 0.5f)) * 2.0f * M_PIf; // −π..π for the approximations
    // zoom (log scale): out and back over the bar, and a bounce in on each beat — sin(π·pb) is 0
    // on the beat, so 1 − sin peaks there with a sharp turn
    const float k = 0.5f - 0.5f * cos_approx(2.0f * M_PIf * pbar);
    const float bounce = 1.0f - sin_approx(M_PIf * pb); // pb 0..1: sin ≥ 0
    const float ln0 = log_approx(OSD_DEMO_ROTO_ZOOM_NEAR), ln1 = log_approx(OSD_DEMO_ROTO_ZOOM_FAR);
    const float zoom = exp_approx(ln0 + (ln1 - ln0) * (0.15f + 0.85f * k) - OSD_DEMO_ROTO_BOUNCE * bounce);
    rotoGreyWant = ((beatsQ8 & 0xFFu) < OSD_DEMO_ROTO_GREY_Q8) ? OSD_DEMO_ROTO_GREY_BEAT : OSD_DEMO_ROTO_GREY;
    // the centre flies over the plane: a heading turning once in HEAD_BEATS, so the path is a
    // circle of FLY · HEAD_BEATS / 2π texels, a second harmonic bending it (closed form, no state)
    const float hb = (float)(beatsQ8 % (256u * OSD_DEMO_ROTO_HEAD_BEATS)) / (256.0f * OSD_DEMO_ROTO_HEAD_BEATS);
    const float hd = (hb - 0.5f) * 2.0f * M_PIf;    // −π..π for the approximations
    const float h2 = ((hb * 3.0f - floorf(hb * 3.0f)) - 0.5f) * 2.0f * M_PIf;
    const float r = OSD_DEMO_ROTO_FLY * OSD_DEMO_ROTO_HEAD_BEATS / (2.0f * M_PIf);
    const float cu = r * sin_approx(hd) + 0.3f * r * sin_approx(h2);
    const float cv = -r * cos_approx(hd) + 0.3f * r * cos_approx(h2);
    // texels a mega-pixel step along x and along y
    const float ux = cos_approx(a) * zoom, vx = sin_approx(a) * zoom;
    const float uy = -vx, vy = ux;
    const int32_t twQ = OSD_DEMO_ROTO_TILE_W << 16, thQ = OSD_DEMO_ROTO_TILE_H << 16;
    const int32_t duQ = (int32_t)(ux * 65536.0f), dvQ = (int32_t)(vx * 65536.0f);
    const int16_t w = OSD_DEMO_CHECKER_COLS * OSD_DEMO_CHECKER_CELL_W2;           // 180
    const int16_t h = OSD_DEMO_CHECKER_ROWS * OSD_DEMO_PLASMA2X2_BANDS;           // 144
    static const uint8_t lvlOf[4] = { 2, 0, 3, 2 }; // sprite code: clear → grey, black, white
    megaBuildBuf = buf;
    for (int16_t my = 0; my < h; my++) {
        // the row's left end in texels, folded into the tile
        const float dy = (float)(my - h / 2), dx = (float)(-w / 2);
        float u = cu + dx * ux + dy * uy, v = cv + dx * vx + dy * vy;
        u -= floorf(u * (1.0f / OSD_DEMO_ROTO_TILE_W)) * OSD_DEMO_ROTO_TILE_W;
        v -= floorf(v * (1.0f / OSD_DEMO_ROTO_TILE_H)) * OSD_DEMO_ROTO_TILE_H;
        int32_t uQ = (int32_t)(u * 65536.0f), vQ = (int32_t)(v * 65536.0f);
        uint16_t cells[OSD_DEMO_CHECKER_COLS];
        for (uint8_t cx = 0; cx < OSD_DEMO_CHECKER_COLS; cx++) {
            uint16_t pat = 0;
            for (uint8_t bc = 0; bc < OSD_DEMO_CHECKER_CELL_W2; bc++) {
                if (uQ >= twQ) { uQ -= twQ; } else if (uQ < 0) { uQ += twQ; }
                if (vQ >= thQ) { vQ -= thQ; } else if (vQ < 0) { vQ += thQ; }
                const uint16_t tx = (uint16_t)(uQ >> 16), ty = (uint16_t)(vQ >> 16);
                uint8_t lvl = 2u;
                if (tx < OSD_DEMO_MEGA_SPRITE_W && ty < OSD_DEMO_MEGA_SPRITE_H) {
                    lvl = lvlOf[(osdDemoMegaSprite[ty * OSD_DEMO_MEGA_SPRITE_STRIDE + (tx >> 2)] >> (2u * (tx & 3u))) & 3u];
                }
                pat |= (uint16_t)((uint16_t)lvl << (2u * bc));
                uQ += duQ;
                vQ += dvQ;
            }
            cells[cx] = pat;
        }
        osdDemoMegaRow(cells, OSD_DEMO_CHECKER_COLS, (uint16_t)(my * OSD_DEMO_PLASMA2X2_STEP));
    }
}

static void osdDemoRotoTask(void)
{
    if (plasmaNextReady || !demoDisplay) {
        return;
    }
    osdDemoRotoBuildFrame((uint8_t)(plasmaPlay ^ 1u));
    __DSB();
    plasmaNextReady = true;
}

// The frame swap, then the grey of the frame on screen (VBLANK)
static void osdDemoRotoField(uint8_t cols, uint8_t rows, bool frameStart, osdDemoRasterMotion_t *next)
{
    osdDemoPlasma2x2Field(cols, rows, frameStart, next);
    const uint8_t g = rotoGreyWant;
    if (g != rotoGreyLevel) {
        rotoGreyLevel = g;
        osdDemoBg(true, g);
    }
}

static const osdDemoIrqScene_t osdDemoRotoScene = {
    .bandLines = 1,
    .bands = OSD_DEMO_CELL_H,
    .armed = &rotoArmed,
    .field = osdDemoRotoField,
    .band = osdDemoBmBand,
    .task = osdDemoRotoTask,
};

static bool cubeArmed; // scene 12 (the cube on the bitmap engine, after scene 13)
static void osdDemoCubeEnter(void);

// --- Hardware HSYNC: the OSD chip's HSYNC output (MAX7456 / AT7456 pin 18) on TIM2 CH2 -------
// TIM2 in slave reset mode on the falling edge: CNT is the time into the current line, exact to
// a tick, no STAT polling (edge time = now − CNT). Checked at demo start. The APB1 timers count
// at the core clock on F411; elsewhere at a fraction of it, so their ticks are converted.
static bool hsCapClaimed; // the HSYNC pin and TIM2 taken by the demo (given back by osdDemoHsCapStop)

static uint32_t osdDemoCpuToTim(uint32_t cpu)
{
    return (uint32_t)(((uint64_t)cpu * irqTimRatioQ16 + 0x8000u) >> 16);
}

static uint32_t osdDemoTimToCpu(uint32_t tim)
{
    return (uint32_t)(((uint64_t)tim << 16) / irqTimRatioQ16);
}
static void osdDemoHsCapStop(void);
static bool osdDemoHsCapWait(uint32_t *edgeDwt, uint32_t timeoutTicks);

// The pins the OSD chip's HSYNC output (MAX7456 / AT7456 pin 18) can go to: TIM2 CH1 / CH2 (AF1
// on every F4 / F7). The first free one with HSYNC on it is used.
typedef struct osdDemoHsPin_s {
    ioTag_t tag;
    uint8_t ch;
} osdDemoHsPin_t;
static const osdDemoHsPin_t osdDemoHsPins[] = {
    { DEFIO_TAG_E(PA0), 1 }, { DEFIO_TAG_E(PA1), 2 }, { DEFIO_TAG_E(PA5), 1 },
    { DEFIO_TAG_E(PA15), 1 }, { DEFIO_TAG_E(PB3), 2 },
};
static const osdDemoHsPin_t *hsPin;

static void osdDemoHsCapStart(const osdDemoHsPin_t *pin)
{
    const IO_t io = IOGetByTag(pin->tag);
    IOInit(io, OWNER_TIMER, 0);
    hsPin = pin;
    hsCapClaimed = true;
    IOConfigGPIOAF(io, IOCFG_AF_PP_UP, 1); // AF1 = TIM2
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    __DSB();
    // TIM2 / TIM3 / TIM5 (all APB1) ticks per CPU tick, Q16
    irqTimRatioQ16 = (uint32_t)(((uint64_t)timerClock(&osdDemoIrqTimHw) << 16)
                                / ((uint64_t)clockMicrosToCycles(1) * 1000000u));
    TIM2->CR1 = 0;
    TIM2->PSC = 0;
    TIM2->ARR = 0xFFFFFFFFu;
    TIM2->CCER = 0;
    // The input capture on the pin's channel, filter fCK_INT N=8 (74 ns), falling edge. Sampled
    // at the timer clock: a divided sampling clock (fDTS/16) moved the reset — and every slot —
    // by a whole OSD pixel line to line.
    if (pin->ch == 1) {
        TIM2->CCMR1 = (TIM2->CCMR1 & ~0x00FFu) | TIM_CCMR1_CC1S_0 | (0x3u << 4);
        TIM2->CCER = TIM_CCER_CC1P | TIM_CCER_CC1E;
    } else {
        TIM2->CCMR1 = (TIM2->CCMR1 & ~0xFF00u) | TIM_CCMR1_CC2S_0 | (0x3u << 12);
        TIM2->CCER = TIM_CCER_CC2P | TIM_CCER_CC2E;
    }
    // Slave reset on TI1FP1 / TI2FP2: the counter restarts at every HSYNC — CNT is the time into
    // the line (zone DMA compares on it; edge time = now − CNT).
    TIM2->SMCR = ((pin->ch == 1 ? 5u : 6u) << 4) | 4u; // TS, SMS = reset
    TIM2->EGR = TIM_EGR_UG;
    TIM2->SR = 0;
    TIM2->CR1 = TIM_CR1_CEN;
    hsCapOn = true;
}

// Next HSYNC falling edge after now, as DWT time; false on timeout (no signal).
static bool osdDemoHsCapWait(uint32_t *edgeDwt, uint32_t timeoutTicks)
{
    const uint16_t flag = (hsPin->ch == 1) ? TIM_SR_CC1IF : TIM_SR_CC2IF;
    TIM2->SR = (uint16_t)~flag;
    const uint32_t t0 = getCycleCounter();
    while (!(TIM2->SR & flag)) {
        if (getCycleCounter() - t0 > timeoutTicks) {
            return false;
        }
    }
    const uint32_t cnt = osdDemoTimToCpu(TIM2->CNT); // reset at the edge: time since it
    *edgeDwt = getCycleCounter() - cnt;
    return true;
}

// HSYNC at a predicted time (±½ line), exact from the last captured edge + whole lines; the
// prediction itself without the wire.
static uint32_t osdDemoHsSnap(uint32_t predDwt, uint32_t lineTicks)
{
    if (!hsCapOn || !lineTicks) {
        return predDwt;
    }
    const uint32_t last = getCycleCounter() - osdDemoTimToCpu(TIM2->CNT); // restarts at each HSYNC
    const int32_t d = (int32_t)(predDwt - last);
    const int32_t n = (d + (int32_t)lineTicks / 2 * (d >= 0 ? 1 : -1)) / (int32_t)lineTicks;
    if (n < -2 || n > 4) {
        return predDwt; // stale capture (signal lost): keep the model
    }
    return last + (uint32_t)(n * (int32_t)lineTicks);
}

// --- Slot engine: zones and glyphs on one timer-paced DMA (HSYNC wired) -------------------
// Timing is the hardware's alone. TIM2 restarts at every HSYNC and opens a gate for S
// slots; TIM3 counts inside it with the slot period — one character cell (12 OSD px, longer than
// a 16-bit SPI word) — and on every CC2 match DMA1 Stream5 moves the slot's word from the line
// buffer into SPI2 DR. Word k of a line goes out at HSYNC + C + k·slot, every line, exactly;
// slot K0 + x at cell x's left edge.
// One CS frame over the active field (16-bit SPI); nothing else may use the bus meanwhile.
//
// A line of the buffer is a template (a STAT read in every slot: no write, the chip only drives
// MISO) with the brightness words (RBn, level) at the cells where the level changes (run-length,
// from the scene) and the glyph words of the coming band
// change in free slots, each cell inside its window: after the beam has left it on the band's
// last line, before the beam reaches it on the new band's first (the chip reads codes just ahead
// of the beam). The buffer (2 halves × ZDMA_LINES lines × S) is refilled by its half / full
// transfer interrupt — a copy, the zone words, and the glyph words placed by computed slot.
// TIM3 and Stream5 are a motor's (DShot is switched off for the demo; reboot before flying).
#define ZDMA_STREAM    DMA1_Stream5 // TIM3_CH2: the slot's word → SPI DR
#define ZDMA_CHANNEL   (5u << 25)
#define ZDMA_SLOTS_MAX 72            // 2 a cell at 27 MHz SPI
#define ZDMA_LINES     6            // lines a buffer half (~390 us for a refill to come)
#define ZDMA_W_HOS     0x0200u
#define ZDMA_W_RB0     0x10u        // RB0..RB15: the row brightness registers (address)
#define ZDMA_W_DMM     0x0400u      // MAX7456 write addresses in the high byte
#define ZDMA_W_DMAH    0x0500u
#define ZDMA_W_DMAL    0x0600u
#define ZDMA_W_DMDI    0x0700u
#define ZDMA_FILLER    0xA000u      // read STAT
#define ZDMA_DMM_AI    0x01u
#define ZDMA_DMM_INV   0x08u
// Geometry (osdDemoZdmaTiming).
static uint8_t zdmaS;        // slots a line
static uint8_t zdmaBmTail;   // and the first slot after it (≥ ZDMA_BM_TAIL_NS past HSYNC)
static uint8_t zdmaBmFetch;  // bitmap mode 2: the slot by which a line's run has ended (~58 us)
#define ZDMA_BM_TAIL_NS 784
static void osdDemoBmFillLine(uint16_t line, uint16_t *dst);
#define OSD_DEMO_HOS_SLOT_LEAD 4 // per-line HOS word: slots before the left edge (s0)
static uint16_t bmCacheGen;  // bitmap band cache: + 1 every field (osdDemoZdmaArm)
static uint32_t zdmaSlot;    // slot, ticks (a cell / zdmaSpc)
static uint32_t zdmaC;       // slot 0's time into the line (TIM3 CCR2)
static uint32_t zdmaSlotInvQ24, zdmaNsQ16; // slot division as a multiply; ns → ticks
// Buffer and stats.
static uint16_t zdmaLine;    // next line to load into the buffer
static DMA_DATA_ZERO_INIT uint16_t zdmaBuf[2 * ZDMA_LINES * ZDMA_SLOTS_MAX]; // F7: in DTCM, past the D-cache
// Glyph window of a cell: 980 ns after the beam left it in line L−1 … 4900 ns before it gets
// there in line L.
#define ZDMA_WIN_START_NS 980
#define ZDMA_CELL_X_NS(x) ((int32_t)((uint32_t)(x) * OSD_DEMO_CELL_W * OSD_DEMO_PX_PS / 1000u)) // cell x's left edge
static int32_t osdDemoZdmaSlotAfter(int32_t line, int32_t ns)
{
    const int32_t t = (int32_t)(((int64_t)ns * zdmaNsQ16) >> 16) - (int32_t)zdmaC;
    const int32_t k = (t <= 0) ? 0 : (int32_t)(((uint32_t)t * zdmaSlotInvQ24 + 0xFFFFFFu) >> 24);
    return line * zdmaS + MIN(k, (int32_t)zdmaS);
}

// Cell windows relative to a line, once a field (osdDemoZdmaWindows): cell x of the change at
// line L may go from slot (L − 1)·S + nb[x] to L·S + na[x].
static int8_t zdmaCellNbOff[OSD_DEMO_CHARS_PER_LINE];

static void osdDemoZdmaWindows(void)
{
    const int32_t x0 = osdDemoX0Ns();
    for (uint8_t x = 0; x < OSD_DEMO_CHARS_PER_LINE; x++) {
        zdmaCellNbOff[x] = (int8_t)osdDemoZdmaSlotAfter(0, x0 + ZDMA_CELL_X_NS(x + 1u) + ZDMA_WIN_START_NS);
    }
}

static void osdDemoZdmaFill(uint8_t half)
{
    for (uint8_t l = 0; l < ZDMA_LINES; l++) {
        osdDemoBmFillLine((uint16_t)(zdmaLine + l), &zdmaBuf[((uint16_t)half * ZDMA_LINES + l) * zdmaS]);
    }
    zdmaLine = (uint16_t)(zdmaLine + ZDMA_LINES);
}

static void osdDemoZdmaIrq(dmaChannelDescriptor_t *descriptor)
{
    if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_HTIF)) {
        DMA_CLEAR_FLAG(descriptor, DMA_IT_HTIF);
        osdDemoZdmaFill(0);
    }
    if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_TCIF)) {
        DMA_CLEAR_FLAG(descriptor, DMA_IT_TCIF);
        osdDemoZdmaFill(1);
    }
    DMA_CLEAR_FLAG(descriptor, DMA_IT_TEIF | DMA_IT_DMEIF | DMA_IT_FEIF);
}

static void osdDemoZdmaStreamOff(void)
{
    TIM3->DIER &= ~TIM_DIER_CC2DE;
    ZDMA_STREAM->CR &= ~DMA_SxCR_EN;
    while (ZDMA_STREAM->CR & DMA_SxCR_EN) {
    }
    DMA1->HIFCR = 0x3Du << 6; // stream 5 flags
}

// Slot geometry: one slot a cell, cell 0 at the screen's left edge (zonex0), the line period.

static void osdDemoZdmaTiming(void)
{
    // All in video time (osdDemoVideoNsToTicks): the slots sit on the beam whatever the MCU clock.
    const uint32_t vQ16 = osdDemoVideoUsQ16();
    // A cell (12 OSD px) or half of one when a 16-bit SPI word fits that (27 MHz): a slot only
    // has to hold one word — the TX buffer takes the next while the last one still shifts.
    const uint32_t cell = (uint32_t)(((uint64_t)OSD_DEMO_CELL_W * OSD_DEMO_PX_PS * vQ16 / 1000000u) >> 16);
    zdmaSpc = (max7456SessionWordTicks() * 2u <= cell) ? 2u : 1u;
    zdmaSlot = cell / zdmaSpc;
    const uint32_t x0 = osdDemoVideoNsToTicks((uint32_t)osdDemoX0Ns()); // the left edge (HOS included)
    zdmaC = x0 % zdmaSlot;
    // No register write after ~56.8 us into the line: from there the chip fetches the next line.
    const uint32_t lastAt = osdDemoVideoNsToTicks(OSD_DEMO_FETCH_NS);
    // Slots over the line but the last ~1.5 us (the gate closes before the next HSYNC).
    const uint32_t line = irqLineTicks ? irqLineTicks : clockMicrosToCycles(64);
    zdmaS = (uint8_t)MIN((uint32_t)ZDMA_SLOTS_MAX, (line - osdDemoVideoNsToTicks(1470)) / zdmaSlot);
    zdmaGate = zdmaSlot * zdmaS;
    // Bitmap runs keep off the end of the line (the chip fetches the next line there) and off
    // the first ~0.8 us after HSYNC.
    const uint32_t fetchAt = lastAt; // ~58 us: the chip fetches the next line
    zdmaBmFetch = (uint8_t)MIN((uint32_t)zdmaS, (fetchAt > zdmaC) ? (fetchAt - zdmaC) / zdmaSlot : 0u);
    const uint32_t tailAt = osdDemoVideoNsToTicks(ZDMA_BM_TAIL_NS);
    zdmaBmTail = (uint8_t)((tailAt > zdmaC) ? (tailAt - zdmaC + zdmaSlot - 1u) / zdmaSlot : 0u);
    zdmaSlotInvQ24 = (1u << 24) / zdmaSlot;
    zdmaNsQ16 = vQ16 / 1000u; // CPU ticks a video ns, Q16
}

// The geometry into the timers (TIM2 gate, TIM3 slot period and CC2); safe between fields.
static void osdDemoZdmaApply(void)
{
    TIM2->CCR3 = osdDemoCpuToTim(zdmaGate);
    TIM3->ARR = (uint16_t)(osdDemoCpuToTim(zdmaSlot) - 1u);
    TIM3->CCR2 = (uint16_t)osdDemoCpuToTim(zdmaC);
}

static void osdDemoZdmaInit(void)
{
    // DShot keeps writing the motors from the motor task (even disarmed, gyro muted) — it
    // reprograms TIM3 and its streams. Off for good: the motors need a reboot after the demo.
    if (motorIsEnabled()) {
        motorDisable();
    }
    osdDemoZdmaTiming();
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    __DSB();
    // TIM2: PWM 1 on CC3 (no pin) = high from the line start for the gate; TRGO = OC3REF.
    TIM2->CCMR2 = (TIM2->CCMR2 & ~0x00FFu) | (6u << 4);
    TIM2->CR2 = (TIM2->CR2 & ~TIM_CR2_MMS) | (6u << 4);
    // TIM3: gated by ITR1 (= TIM2 TRGO), the slot period, CC2 at C (frozen, no pin).
    TIM3->CR1 = 0;
    TIM3->DIER = 0;
    TIM3->CCER = 0;
    TIM3->PSC = 0;
    TIM3->CCMR1 = 0;
    TIM3->CCMR2 = 0;
    osdDemoZdmaApply();
    TIM3->SMCR = (1u << 4) | 5u; // TS = ITR1, SMS = gated
    TIM3->EGR = TIM_EGR_UG;
    TIM3->CR1 = TIM_CR1_CEN;
    osdDemoZdmaStreamOff();
    dmaSetHandler(DMA1_ST5_HANDLER, osdDemoZdmaIrq, NVIC_PRIO_MAX, 0);
}

// VBLANK: the stream stays off until this field's phase event arms it.
static void osdDemoZdmaField(void)
{
    osdDemoZdmaStreamOff();
}

// Self-healing: a word the chip did not take leaves the shadow (mgSram) ahead of Display SRAM,
// and the diff never writes that cell again while the picture keeps it — stale cells piled up
// over minutes (gone at a scene restart). Each field a few cells of the shadow are made unknown
// (0xFF, never a code: it ends auto-increment runs), so they are written at their next band
// change: all of Display SRAM once every ~2.5 s.
#define ZDMA_HEAL_CELLS 4u
static uint16_t zdmaHealPos;

static void osdDemoZdmaHeal(void)
{
    const uint16_t total = (uint16_t)irqRows * OSD_DEMO_CHARS_PER_LINE;
    for (uint8_t i = 0; i < ZDMA_HEAL_CELLS && total; i++) {
        mgSram[zdmaHealPos / OSD_DEMO_CHARS_PER_LINE][zdmaHealPos % OSD_DEMO_CHARS_PER_LINE] = 0xFF;
        if (++zdmaHealPos >= total) {
            zdmaHealPos = 0;
        }
    }
}

// Phase event (this field's line 0 known): geometry, scheduler reset, the first half.
static void osdDemoZdmaArm(void)
{
    bmCacheGen++; // the bitmap engine's band cache: bands of this field from now on
    osdDemoZdmaHeal();
    osdDemoZdmaTiming(); // the SPI clock the session runs at, this line period
    osdDemoZdmaApply();
    osdDemoZdmaWindows();
    zdmaLine = 0;
    osdDemoZdmaFill(0); // the second half right after the start
    ZDMA_STREAM->PAR = (uint32_t)max7456SpiDataRegister();
    ZDMA_STREAM->M0AR = (uint32_t)zdmaBuf;
    ZDMA_STREAM->NDTR = 2u * ZDMA_LINES * zdmaS;
    ZDMA_STREAM->FCR = 0;
    ZDMA_STREAM->CR = ZDMA_CHANNEL | DMA_SxCR_PL_1 | DMA_SxCR_PL_0 | DMA_SxCR_MSIZE_0 | DMA_SxCR_PSIZE_0
                      | DMA_SxCR_MINC | DMA_SxCR_CIRC | DMA_SxCR_DIR_0 | DMA_SxCR_HTIE | DMA_SxCR_TCIE;
    ZDMA_STREAM->CR |= DMA_SxCR_EN;
}

// Before line 0: SPI 16-bit, then — at the end of line −1, gate closed, TIM3 standing — TIM3
// from 0 and the DMA requests on. Line 0's HSYNC opens the gate: slot 0 at exactly C after it.
// Too late for line 0 (the setup ran long): no session this field, never one shifted by lines.
static void osdDemoZdmaStart(void)
{
    zdmaStartVsLine0 = (int32_t)(getCycleCounter() - zdmaLine0At);
    if (zdmaStartVsLine0 > -(int32_t)(clockMicrosToCycles(1) / 2u)) {
        zdmaEndPending = false;
        osdDemoZdmaStreamOff();
        return;
    }
    max7456Zone16Begin();
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const uint32_t t0 = getCycleCounter();
    const uint32_t gateTim = osdDemoCpuToTim(zdmaGate);
    while (TIM2->CNT < gateTim && getCycleCounter() - t0 < clockMicrosToCycles(14)) {
    }
    TIM3->CNT = 0;
    TIM3->SR = (uint16_t)~TIM_SR_CC2IF;
    TIM3->DIER |= TIM_DIER_CC2DE;
    __set_PRIMASK(primask);
    osdDemoZdmaFill(1); // read from line ZDMA_LINES on
}

static void osdDemoZdmaEnd(void)
{
    TIM3->DIER &= ~TIM_DIER_CC2DE;
    const uint32_t t0 = getCycleCounter(); // let a DMA word in flight finish
    while (getCycleCounter() - t0 < clockMicrosToCycles(3)) {
    }
    max7456Zone16End();
    osdDemoZdmaStreamOff();
}

// Give the HSYNC pin back (input, no owner) and stop TIM2.
static void osdDemoHsCapStop(void)
{
    if (!hsCapClaimed) {
        return;
    }
    TIM2->CR1 = 0;
    TIM2->CCER = 0;
    TIM2->SMCR = 0;
    const IO_t io = IOGetByTag(hsPin->tag);
    IOConfigGPIO(io, IOCFG_IN_FLOATING);
    IOInit(io, OWNER_FREE, 0);
    hsCapOn = false;
    hsCapClaimed = false;
}

// Demo start: where is HSYNC wired? Every free candidate pin is tried; ~2 lines of edges
// 55..75 us apart keep the capture on, anything else gives the pin back. The OSD chip makes
// HSYNC from the camera or its own sync, so it is there whenever the wire is.
static bool osdDemoHsCapDetect(void)
{
    if (hsCapOn) {
        return true;
    }
    const uint32_t lo = clockMicrosToCycles(55), hi = clockMicrosToCycles(75);
    for (unsigned i = 0; i < ARRAYLEN(osdDemoHsPins); i++) {
        const IO_t io = IOGetByTag(osdDemoHsPins[i].tag);
        if (!io || IOGetOwner(io) != OWNER_FREE) {
            continue;
        }
        osdDemoHsCapStart(&osdDemoHsPins[i]);
        uint32_t e0 = 0, e1 = 0, e2 = 0;
        if (osdDemoHsCapWait(&e0, clockMicrosToCycles(200))
            && osdDemoHsCapWait(&e1, clockMicrosToCycles(100))
            && osdDemoHsCapWait(&e2, clockMicrosToCycles(100))
            && (e1 - e0) > lo && (e1 - e0) < hi && (e2 - e1) > lo && (e2 - e1) < hi) {
            return true;
        }
        osdDemoHsCapStop();
    }
    return false;
}

// The board's name for a pad: its UART / soft serial / LED strip function, else the pin.
static void osdDemoPadName(ioTag_t tag, char *buf, size_t size)
{
    for (unsigned i = 0; i < RESOURCE_UART_COUNT; i++) {
        if (serialPinConfig()->ioTagTx[RESOURCE_UART_OFFSET + i] == tag) {
            tfp_sprintf(buf, "TX%u (P%c%u)", i + 1, 'A' + DEFIO_TAG_GPIOID(tag), DEFIO_TAG_PIN(tag));
            return;
        }
        if (serialPinConfig()->ioTagRx[RESOURCE_UART_OFFSET + i] == tag) {
            tfp_sprintf(buf, "RX%u (P%c%u)", i + 1, 'A' + DEFIO_TAG_GPIOID(tag), DEFIO_TAG_PIN(tag));
            return;
        }
    }
#ifdef USE_I2C
    for (unsigned i = 0; i < I2CDEV_COUNT; i++) {
        if (i2cConfig(i)->ioTagScl == tag || i2cConfig(i)->ioTagSda == tag) {
            tfp_sprintf(buf, "%s (P%c%u)", i2cConfig(i)->ioTagScl == tag ? "SCL" : "SDA", 'A' + DEFIO_TAG_GPIOID(tag), DEFIO_TAG_PIN(tag));
            return;
        }
    }
#endif
#ifdef USE_LED_STRIP
    if (ledStripConfig()->ioTag == tag) {
        tfp_sprintf(buf, "LED (P%c%u)", 'A' + DEFIO_TAG_GPIOID(tag), DEFIO_TAG_PIN(tag));
        return;
    }
#endif
    UNUSED(size);
    tfp_sprintf(buf, "P%c%u", 'A' + DEFIO_TAG_GPIOID(tag), DEFIO_TAG_PIN(tag));
}

// Where the sound goes (claimed now if the music has not started yet): HSYNC has its pad by then.
const char *osdDemoSoundPad(void)
{
    static char info[40];
    char pad[24];
    if (!ayPwmClaim()) {
        return "no sound: no free LED / I2C / UART pad on a timer with DMA";
    }
    osdDemoPadName(ayPwmPin(), pad, sizeof(pad));
    tfp_sprintf(info, "sound on %s", pad);
    return info;
}

// No HSYNC found: the free pads it could go to.
static const char *osdDemoHsCapHint(void)
{
    static char hint[128];
    char pad[24];
    strcpy(hint, "no HSYNC. Solder OSD chip pin 18 to one of:");
    bool any = false;
    for (unsigned i = 0; i < ARRAYLEN(osdDemoHsPins); i++) {
        const IO_t io = IOGetByTag(osdDemoHsPins[i].tag);
        if (io && IOGetOwner(io) == OWNER_FREE && strlen(hint) + sizeof(pad) + 2 < sizeof(hint)) {
            osdDemoPadName(osdDemoHsPins[i].tag, pad, sizeof(pad));
            strcat(hint, any ? ", " : " ");
            strcat(hint, pad);
            any = true;
        }
    }
    if (!any) {
        strcat(hint, " PA0 PA1 PA15 PB3 (all in use: free one with `resource`)");
    }
    return hint;
}


// --- Triple-buffered frame build (scenes 12, 13): the bitmap plasma's shape and tables ------
#define OSD_DEMO_PB_W       (OSD_DEMO_CHARS_PER_LINE * OSD_DEMO_P3_LP) // 120: the plasma shape's width
#define OSD_DEMO_PB_H_SHAPE 96                                         // and height (3-line units)

static uint8_t pbFold[256]; // two-ramp fold of the (sum + 508)·129 >> 8 phase → 0..120 %
static int8_t pbSine[256];  // RAM copy of the sine for the builder (filled at scene entry)
#define PB_SIN(i) (pbSine[(uint8_t)(i)])

static void osdDemoPbFoldInit(void)
{
    if (!pbFold[255] && !pbFold[128]) {
        for (uint16_t ph = 0; ph < 256u; ph++) {
            const uint16_t tri = (ph < 128u) ? (uint16_t)(ph * 2u) : (uint16_t)((255u - ph) * 2u);
            pbFold[ph] = (uint8_t)((tri * 121u) >> 8);
        }
    }
}

// Triple buffer (shown / ready / being built): a build never waits for the frame start to
// free a buffer, and the field hook shows the newest ready one.
// The frame is built in a spare interrupt at the lowest priority, pended by the field hook right
// at each frame start for the next frame — no task latency (the OSD task runs at osd_framerate:
// the build began up to a task period late and a 34 ms build then missed every other 40 ms
// frame). It takes the CPU from the main loop like the task did, and SysTick goes one level up
// meanwhile so time keeps running.
#define PB_NONE 0xFFu
static volatile uint8_t pbReady = PB_NONE;
static uint32_t pbBuiltT;
// A peripheral interrupt the MCU has but Betaflight never uses, pended by software.
#if defined(STM32F411xE)
#define PB_BUILD_IRQ         SPI5_IRQn
#define PB_BUILD_IRQ_HANDLER SPI5_IRQHandler
#elif defined(STM32F446xx)
#define PB_BUILD_IRQ         SPI4_IRQn
#define PB_BUILD_IRQ_HANDLER SPI4_IRQHandler
#elif defined(STM32F7)
#define PB_BUILD_IRQ         RNG_IRQn
#define PB_BUILD_IRQ_HANDLER RNG_IRQHandler
#else
#define PB_BUILD_IRQ         HASH_RNG_IRQn
#define PB_BUILD_IRQ_HANDLER HASH_RNG_IRQHandler
#endif
static volatile bool pbBuilding;
static volatile uint8_t pbTarget;
static volatile uint32_t pbTargetT;
static uint32_t pbFrameNo;

// The running triple-buffered scene's frame builder (scene 12 or 13).
static void (*pbBuildFn)(uint8_t buf, uint8_t cols, uint8_t rows, uint32_t t);

static void osdDemoPbBuildInto(uint8_t buf, uint32_t t)
{
    pbBuildFn(buf, MIN(demoDisplay->cols, OSD_DEMO_CHARS_PER_LINE), MIN(demoDisplay->rows, OSD_DEMO_MG_ROWS_MAX), t);
    pbBuiltT = t;
    __DSB();
    pbReady = buf; // a frame still waiting is dropped: this one is newer
}

void PB_BUILD_IRQ_HANDLER(void)
{
    if (pbBuilding && demoDisplay) {
        osdDemoPbBuildInto(pbTarget, pbTargetT);
    }
    pbBuilding = false;
}

static void osdDemoPbBuildIrqOn(bool on)
{
    NVIC_DisableIRQ(PB_BUILD_IRQ);
    NVIC_ClearPendingIRQ(PB_BUILD_IRQ);
    pbBuilding = false;
    if (on) {
        // CMSIS takes the 4-bit level, NVIC_BUILD_PRIORITY gives it in the register's top bits
        // (passed as is it shifts out to 0 = the top priority, over the raster interrupts).
        NVIC_SetPriority(SysTick_IRQn, NVIC_BUILD_PRIORITY(2, 3) >> (8u - __NVIC_PRIO_BITS));
        NVIC_SetPriority(PB_BUILD_IRQ, NVIC_BUILD_PRIORITY(3, 3) >> (8u - __NVIC_PRIO_BITS));
        NVIC_EnableIRQ(PB_BUILD_IRQ);
    } else {
        NVIC_SetPriority(SysTick_IRQn, (1u << __NVIC_PRIO_BITS) - 1u); // back to CMSIS' lowest
    }
}

static void osdDemoPbTask(void)
{
}

// VBLANK: new frame on a frame start, the grey on (first field), band 0's level of every row.
// Frame start (VBLANK): show the newest ready frame and start building the next.
static void osdDemoTriFrame(bool frameStart)
{
    if (frameStart && pbReady != PB_NONE) {
        plasmaPlay = pbReady;
        pbReady = PB_NONE;
    }
    if (frameStart) {
        pbFrameNo++;
        if (!pbBuilding) { // else the last one overran: this frame repeats, the next is late
            uint8_t buf = 0;
            while (buf == plasmaPlay || buf == pbReady) {
                buf++;
            }
            pbTarget = buf;
            pbTargetT = pbFrameNo + 1u; // shown at the next frame start
            pbBuilding = true;
            NVIC_SetPendingIRQ(PB_BUILD_IRQ);
        }
    }
}

// --- Scene 13: bitmap plasma — 2 px × 1 line, black / white and one grey pixel a cell line ---
// 180 × 288 logical pixels a field (both fields the same picture). A cell line is six 2-px
// pixels: any black / white pattern (32 glyphs + INV), or one colour with clear pixels (the VM1
// grey) as the grey set has them (28 glyphs + INV for white, all clear = the space). No level
// changes: RB fixed at black 0 % / white 120 %, so grey is the only middle tone.
// Every line all 30 codes go out (slot engine, HSYNC wired, 27 MHz SPI): one auto-increment run
// a line, DMM only where INV changes, each code behind the beam on the line before (its window),
// the end of the run after the next HSYNC (no glyph write in the last ~7 us of a line). The rows'
// first lines go out in VBLANK (the engine's preload of band 0).
// OSD_DEMO_BM_BASE (32 glyphs, with the font): black / white; greys: the grey set (GREY_BASE).
#define OSD_DEMO_BM_W      (OSD_DEMO_CHARS_PER_LINE * 6)            // 180
#define OSD_DEMO_BM_LINES  (OSD_DEMO_MG_ROWS_MAX * OSD_DEMO_CELL_H) // 288
#define OSD_DEMO_BM_GREY   49                                       // the clear pixels: VM1 grey (%)

static bool bmArmed;
static bool bmBgPending;
// Pattern of a cell line (6 base-3 digits, left first: 0 black, 1 grey, 2 white) → code | INV << 8,
// per line parity (a pattern the glyphs cannot show gets its greys as black / white alternately).
// Per 4×4 ordered-dither position: plasma value 0..120 % → 0 / 1 / 2 (black / grey / white).
static uint8_t bmQ[16][121];
// The plasma's terms per frame (the shape of the old 3×2 plasma: x in thirds of its 3-px pixels = 2 px each,
// y in thirds of its 3-line pixels = 1 line each).
static int8_t bmSx[OSD_DEMO_BM_W], bmSy[OSD_DEMO_BM_LINES];
static int8_t bmSxy3[2 * OSD_DEMO_BM_W + OSD_DEMO_BM_LINES];
static uint16_t bmDx2[OSD_DEMO_BM_W], bmDy2[OSD_DEMO_BM_LINES];

static void osdDemoBmTables(void)
{
    static const uint8_t bayer[16] = { 0, 8, 2, 10, 12, 4, 14, 6, 3, 11, 1, 9, 15, 7, 13, 5 };
    for (uint8_t i = 0; i < 16u; i++) {
        const uint32_t th = 2u * bayer[i] + 1u; // threshold (th / 32) of the step
        for (uint8_t v = 0; v <= 120u; v++) {
            if (v < OSD_DEMO_BM_GREY) {
                bmQ[i][v] = ((uint32_t)v * 32u > th * OSD_DEMO_BM_GREY) ? 1u : 0u;
            } else {
                bmQ[i][v] = ((uint32_t)(v - OSD_DEMO_BM_GREY) * 32u > th * (120u - OSD_DEMO_BM_GREY)) ? 2u : 1u;
            }
        }
    }
    // Each pattern → a glyph: exact where one fits — greys on one colour from the grey set (made
    // for this dither: nearly every pattern it gives), all grey (the space) — else black / white
    // with the greys alternately black and white (shifted every line). (A best-error choice over all glyphs gave
    // grey blocks and line-to-line flicker: much worse than the dither.)
    for (uint8_t par = 0; par < 2u; par++) {
        for (uint16_t idx = 0; idx < 729u; idx++) {
            uint8_t d[6];
            uint16_t r = idx;
            uint8_t greys = 0, colours = 0, greyMask = 0;
            for (int8_t k = 5; k >= 0; k--) {
                d[k] = (uint8_t)(r % 3u);
                r = (uint16_t)(r / 3u);
            }
            for (uint8_t k = 0; k < 6u; k++) {
                if (d[k] == 1u) {
                    greys++;
                    greyMask |= (uint8_t)(0x20u >> k);
                } else {
                    colours |= (uint8_t)(1u << (d[k] >> 1)); // bit 0 black, bit 1 white
                }
            }
            const uint16_t inv = (colours == 2u) ? 0x100u : 0u; // the rest white: INV
            uint16_t w;
            const uint8_t gc = (greys && colours != 3u) ? osdDemoGreyCode(greyMask) : 0u;
            const uint16_t g3 = (greys && colours == 3u) ? osdDemoGrey3Of(idx) : 0xFFFFu;
            if (g3 != 0xFFFFu) {
                w = g3; // black, grey and white: one of the three-colour glyphs
            } else if (gc) {
                w = (uint16_t)(gc | ((greys == 6u) ? 0u : inv)); // the grey set (or the space)
            } else {
                uint8_t bits = 0;
                for (uint8_t k = 0; k < 6u; k++) {
                    const bool white = (d[k] == 1u) ? (((k + par) & 1u) != 0u) : (d[k] == 2u);
                    bits = (uint8_t)((bits << 1) | (white ? 1u : 0u));
                }
                w = (bits & 0x20u) ? (uint16_t)((OSD_DEMO_BM_BASE + (~bits & 0x1Fu)) | 0x100u)
                                   : (uint16_t)(OSD_DEMO_BM_BASE + bits);
            }
            bmLut[par][idx] = w;
        }
    }
}

static void osdDemoBmBuildFrame(uint8_t buf, uint8_t cols, uint8_t rows, uint32_t t)
{
    osdDemoPbFoldInit();
    const int16_t cx = (int16_t)(OSD_DEMO_PB_W / 2 - (PB_SIN((uint8_t)(t / 2u)) >> 2));
    const int16_t cy = (int16_t)(OSD_DEMO_PB_H_SHAPE / 2 - (PB_SIN((uint8_t)(t / 3u + 64u)) >> 3));
    const uint16_t w = (uint16_t)cols * 6u;
    const uint16_t lines = (uint16_t)rows * OSD_DEMO_CELL_H;
    for (uint16_t x = 0; x < w; x++) {
        const int32_t x3 = 2 * (int32_t)x; // the shape's 3-px pixels × 3
        bmSx[x] = PB_SIN((uint8_t)((x3 * 9) / 12 + (int32_t)t));
        const int32_t dx3 = x3 - 3 * cx;
        bmDx2[x] = (uint16_t)((dx3 * dx3) / 9);
    }
    for (uint16_t l = 0; l < lines; l++) {
        bmSy[l] = PB_SIN((uint8_t)(((int32_t)l * 7) / 9 - (int32_t)(t * 2u)));
        const int32_t dy3 = (int32_t)l - 3 * cy;
        bmDy2[l] = (uint16_t)((dy3 * dy3) / 9);
    }
    for (uint16_t d3 = 0; d3 < ARRAYLEN(bmSxy3); d3++) {
        bmSxy3[d3] = PB_SIN((uint8_t)((d3 * 5) / 12 + (int32_t)(t * 3u / 2u)));
    }
    const uint8_t t2 = (uint8_t)(t * 2u);
    for (uint16_t l = 0; l < lines; l++) {
        const int16_t sy = bmSy[l];
        const uint16_t dy2 = bmDy2[l];
        const int8_t *sxy = &bmSxy3[l];
        const uint8_t (*q)[121] = &bmQ[(l & 3u) << 2];
        const uint16_t *lut = bmLut[l & 1u];
        uint8_t *code = bmCode[buf][l];
        uint32_t inv = 0;
        uint16_t x = 0;
        for (uint8_t c = 0; c < cols; c++) {
            uint16_t idx = 0;
            for (uint8_t k = 0; k < 6u; k++, x++) {
                int32_t s2 = bmSx[x] + sy + sxy[2u * x];
                s2 += pbSine[(uint8_t)((((uint32_t)bmDx2[x] + dy2) * 21u >> 10) - t2)];
                const uint8_t v = pbFold[(uint8_t)(((uint32_t)(s2 + 508) * 129u) >> 8)];
                idx = (uint16_t)(idx * 3u + q[x & 3u][v]);
            }
            const uint16_t g = lut[idx];
            code[c] = (uint8_t)g;
            inv |= (uint32_t)(g >> 8) << c;
        }
        bmInv[buf][l] = inv;
    }
}

// The codes of a line come from the scene's band hook — any band scene runs on this engine —
// computed once per band and field (two bands cached: a line's fill needs its own and the next).
typedef struct {
    uint16_t key;  // y · 32 + band
    uint16_t gen;  // the field it was made in
    uint32_t inv;
    uint8_t code[OSD_DEMO_CHARS_PER_LINE];
} osdDemoBmBandCache_t;
static osdDemoBmBandCache_t bmCache[2];
static uint8_t bmCacheNext;

static const uint8_t *osdDemoBmLineCodes(uint16_t line, uint32_t *inv)
{
    const uint8_t y = (uint8_t)(line / OSD_DEMO_CELL_H);
    const uint8_t b = (uint8_t)((line % OSD_DEMO_CELL_H) / irqScene->bandLines);
    const uint16_t key = (uint16_t)(y * 32u + b);
    for (uint8_t i = 0; i < 2u; i++) {
        if (bmCache[i].key == key && bmCache[i].gen == bmCacheGen) {
            *inv = bmCache[i].inv;
            return bmCache[i].code;
        }
    }
    osdDemoBmBandCache_t *c = &bmCache[bmCacheNext];
    bmCacheNext ^= 1u;
    if (irqCols < OSD_DEMO_CHARS_PER_LINE) {
        memset(c->code, OSD_DEMO_PIXEL_OFF, sizeof(c->code));
    }
    mgBandInv = 0;
    irqScene->band(y, b, irqCols, c->code);
    c->inv = mgBandInv;
    c->key = key;
    c->gen = bmCacheGen;
    *inv = c->inv;
    return c->code;
}

// What the chip's row holds (the engine picks what to write itself): a line writes only the span
// of cells whose code or INV differs from it — none when the line repeats the one above (a 6-line
// band rewrote all 30 cells on all 6 lines; its last columns, written after HSYNC, now and then
// missed a line). A row's first line is written whole (healing: a word the chip dropped is
// written again next field); cells a run had no room for stay unknown here and differ next line.
static uint8_t bmShCode[OSD_DEMO_CHARS_PER_LINE];
static uint32_t bmShInv;

static void osdDemoBmShadowSet(uint16_t line)
{
    uint32_t inv;
    const uint8_t *code = osdDemoBmLineCodes(line, &inv);
    memcpy(bmShCode, code, sizeof(bmShCode));
    bmShInv = inv;
}

// Where a line's cells go: one auto-increment run in the line before, back to back from its left
// edge, ended before ~58 us (the chip fetches the next line from there; codes written after
// HSYNC showed a line late).
static inline int16_t osdDemoBmS0(void)
{
    return (int16_t)MAX(0, (int16_t)zdmaCellNbOff[0] - 3);
}

// One auto-increment run of cells [x, end) from slot pos: DMM, DMAH, DMAL, the codes (INV
// switches by DMM), the escape at slot lim at the latest. buf NULL: only counted. Returns the
// first cell not written.
static uint8_t osdDemoBmRun(uint16_t *buf, int16_t pos, int16_t lim, uint16_t addr, uint8_t x,
                            uint8_t end, const uint8_t *code, uint32_t inv, uint16_t dmm)
{
    if (x >= end || pos + 4 > lim) {
        return x;
    }
    uint8_t cur = (uint8_t)((inv >> x) & 1u);
    if (buf) {
        const uint16_t a = (uint16_t)(addr + x);
        buf[pos] = (uint16_t)(dmm | (cur ? ZDMA_DMM_INV : 0u));
        buf[pos + 1] = ZDMA_W_DMAH | ((a >> 8) & 1u);
        buf[pos + 2] = ZDMA_W_DMAL | (a & 0xFFu);
    }
    pos = (int16_t)(pos + 3);
    for (; x < end; x++) {
        const uint8_t want = (uint8_t)((inv >> x) & 1u);
        const int16_t d = (int16_t)(pos + ((want != cur) ? 1 : 0));
        if (d + 1 > lim) {
            break;
        }
        if (want != cur) {
            cur = want;
            if (buf) {
                buf[pos] = (uint16_t)(dmm | (cur ? ZDMA_DMM_INV : 0u));
            }
        }
        if (buf) {
            buf[d] = (uint16_t)(ZDMA_W_DMDI | code[x]);
        }
        pos = (int16_t)(d + 1);
    }
    if (buf) {
        buf[pos] = ZDMA_W_DMDI | 0xFFu;
    }
    return x;
}

// head: the buffer of the line before (the run goes there), tail: the buffer of `line` itself
// (the line counts as placed); either NULL.
static void osdDemoBmPlace(uint16_t line, uint16_t *head, uint16_t *tail)
{
    uint32_t inv;
    const uint8_t *code = osdDemoBmLineCodes(line, &inv);
    // The span to write: [xa, xb]
    uint8_t xa = 0, xb = OSD_DEMO_CHARS_PER_LINE - 1u;
    const uint8_t rowY = (uint8_t)(line / OSD_DEMO_CELL_H);
    if ((line % OSD_DEMO_CELL_H) == 0u && rowY < irqYPre) {
        // A row's first line the VBLANK preload wrote whole this field (every cell, mgSram
        // cleared): the chip has it — nothing to write (a whole line with INV changes did not
        // fit its slots: the rows' first lines kept their right cells from the field before).
        if (tail) {
            memcpy(bmShCode, code, sizeof(bmShCode));
            bmShInv = inv;
        }
        return;
    }
    if ((line % OSD_DEMO_CELL_H) != 0u) {
        const uint32_t dInv = inv ^ bmShInv;
        while (xa < OSD_DEMO_CHARS_PER_LINE && code[xa] == bmShCode[xa] && !((dInv >> xa) & 1u)) {
            xa++;
        }
        if (xa >= OSD_DEMO_CHARS_PER_LINE) {
            return; // the line repeats the one above: nothing to write
        }
        while (code[xb] == bmShCode[xb] && !((dInv >> xb) & 1u)) {
            xb--;
        }
    }
    const uint8_t end = (uint8_t)(xb + 1u);
    const uint16_t addr = (uint16_t)((line / OSD_DEMO_CELL_H) * OSD_DEMO_CHARS_PER_LINE);
    const uint16_t dmm = (uint16_t)(ZDMA_W_DMM | (max7456DmmBase() & (uint8_t)~(ZDMA_DMM_AI | ZDMA_DMM_INV)) | ZDMA_DMM_AI);
    const int16_t s0 = osdDemoBmS0();
    uint8_t x;
    // From the left edge (s0, ~8 us). Not from just after HSYNC: there every scene broke
    // (content a line off) and cell 0's left column showed grey — the chip is still busy with
    // the line around HSYNC.
    x = osdDemoBmRun(head, s0, (int16_t)zdmaBmFetch - 1, addr, xa, end, code, inv, dmm);
    if (tail) {
        // The line is placed (its head went out a line earlier, this is its last call): the
        // chip's row now holds cells [xa, x) of it (a row's first line: the rest not known)
        if (xa == 0u && (line % OSD_DEMO_CELL_H) == 0u) {
            memset(bmShCode, 0xFF, sizeof(bmShCode));
        }
        for (uint8_t c = xa; c < x; c++) {
            bmShCode[c] = code[c];
        }
        const uint32_t m = ((x >= 32u) ? 0xFFFFFFFFu : ((1u << x) - 1u)) & ~((1u << xa) - 1u);
        bmShInv = (bmShInv & ~m) | (inv & m);
    }
}

// Slot engine refill, bitmap scene: line `line` carries the rest of its own run and the head of
// the next line's, each the span that differs from the line above (osdDemoBmPlace). Row first
// lines: their head sits in the row above — fine, other row; line 0 is the VBLANK preload.
static void osdDemoBmFillLine(uint16_t line, uint16_t *dst)
{
    for (uint8_t k = 0; k < zdmaS; k++) {
        dst[k] = ZDMA_FILLER;
    }
    const uint16_t lines = (uint16_t)irqRows * OSD_DEMO_CELL_H;
    if (zdmaSpc < 2u || line >= lines) {
        return; // one slot a cell (13.5 MHz SPI) has no room for a run a line
    }
    const int16_t s0 = osdDemoBmS0();
    if (line == 0u) {
        osdDemoBmShadowSet(0); // line 0: the VBLANK preload
        if (irqYPre == 0u) {
            memset(bmShCode, 0xFF, sizeof(bmShCode)); // no preload this field: not known
        }
    } else {
        osdDemoBmPlace(line, NULL, dst);
    }
    if (line + 1u < lines) {
        osdDemoBmPlace((uint16_t)(line + 1u), dst, NULL);
    }
    // Per-line HOS: this line's offset a few slots before its left edge (~4 us after HSYNC),
    // in a free slot (mode 2: the head of the next line starts at s0) — if the chip takes HOS
    // at once the line moves whole, if at the next HSYNC a line later.
    const int8_t *hl = irqMotion.hosLine;
    if (hl && line > 0u && hl[line] != zdmaHosVal) {
        const int16_t k = (int16_t)(s0 - OSD_DEMO_HOS_SLOT_LEAD);
        if (k > (int16_t)zdmaBmTail && dst[k] == ZDMA_FILLER) {
            zdmaHosVal = hl[line];
            dst[k] = (uint16_t)(ZDMA_W_HOS | (uint8_t)constrain((int)irqHosBase + zdmaHosVal, 0, 63));
        }
    }
    // Per-line RB: the level for this line, a free slot or two before the HOS one, into the
    // register the chip reads for it — the next row's from a row's last line (it takes the next
    // row's RB a line before the row boundary: zoneline −1)
    const uint8_t *rl = irqMotion.rbLine;
    const uint8_t row = (uint8_t)((line + 1u) / OSD_DEMO_CELL_H);
    if (rl && line > 0u && row < OSD_DEMO_MG_ROWS_MAX && rl[line] != zdmaRbSh[row]) {
        for (int16_t k = (int16_t)(s0 - OSD_DEMO_HOS_SLOT_LEAD - 1); k >= (int16_t)(s0 - OSD_DEMO_HOS_SLOT_LEAD - 3); k--) {
            if (k > (int16_t)zdmaBmTail && dst[k] == ZDMA_FILLER) {
                zdmaRbSh[row] = rl[line];
                dst[k] = (uint16_t)(((uint16_t)(ZDMA_W_RB0 + row) << 8) | rl[line]);
                break;
            }
        }
    }
}

static void osdDemoBmField(uint8_t cols, uint8_t rows, bool frameStart, osdDemoRasterMotion_t *next)
{
    UNUSED(cols);
    UNUSED(rows);
    osdDemoTriFrame(frameStart);
    // The runs change the chip behind the shadow's back: the VBLANK preload of the rows' first
    // lines must write every cell, not only the ones the shadow thinks changed.
    memset(mgSram, 0xFF, sizeof(mgSram));
    if (bmBgPending) {
        bmBgPending = false;
        osdDemoBg(true, 7);
    }
    next->vos = 0;
}

// VBLANK preload (band 0 = a row's first line) and the polled engine without the HSYNC wire.
static void osdDemoBmBand(uint8_t y, uint8_t b, uint8_t cols, uint8_t *glyphs)
{
    const uint16_t line = (uint16_t)(y * OSD_DEMO_CELL_H + b);
    memcpy(glyphs, bmCode[plasmaPlay][line], cols);
    mgBandInv = bmInv[plasmaPlay][line];
}

static osdDemoIrqScene_t osdDemoBitmapScene = {
    .bandLines = 1,
    .bands = OSD_DEMO_CELL_H,
    .armed = &bmArmed,
    .field = osdDemoBmField,
    .band = osdDemoBmBand,
    .task = osdDemoPbTask,
};

static void osdDemoBmEnter(void)
{
    osdDemoApplyOsdmNow();
    max7456Brightness(0, 3);
    osdDemoBgType(DISPLAY_BACKGROUND_LTGRAY);
    osdDemoBg(true, 0); // black until the screen is glyphs; grey from field 1
    bmBgPending = true;
    max7456SetHudMotionOffset(0, 0);
    if (!demoDisplay) {
        return;
    }
    osdDemoBmTables(); // the glyphs are in the font since the demo start
    for (uint16_t i = 0; i < 256u; i++) {
        pbSine[i] = (int8_t)osdDemoSineU8[i];
    }
    osdDemoFillRowsGlyphFast(demoDisplay->cols, demoDisplay->rows, OSD_DEMO_BM_BASE, OSD_DEMO_BM_BASE);
    memset(mgSram, 0xFF, sizeof(mgSram));
    mgLastEdgeTicks = 0;
    plasmaPlay = 0;
    plasmaNextReady = false;
    pbReady = PB_NONE;
    pbBuildFn = osdDemoBmBuildFrame;
    pbFrameNo = 0;
    pbBuiltT = 0;
    osdDemoBmBuildFrame(0, MIN(demoDisplay->cols, OSD_DEMO_CHARS_PER_LINE), MIN(demoDisplay->rows, OSD_DEMO_MG_ROWS_MAX), pbBuiltT);
    if (!osdDemoIrqStart(&osdDemoBitmapScene)) {
        return;
    }
    osdDemoPbBuildIrqOn(true);
}


// --- Scene 14: the end — "*** DISARM ***" blinking in the middle, no music -------------------
// Black screen, the music off (another scene asked for starts it again; the demo ends after it). The text in the 5×8 font
// with scene 1's kerning, a font pixel 3 px × 3 lines: per line the 3-colour set (four 3-px
// columns a cell, black / white) on the bitmap engine. Two frames built at the entry, text and
// none; the field hook shows one or the other.
#define OSD_DEMO_DISARM_TEXT     "*** DISARM ***"
#define OSD_DEMO_DISARM_BLINK_MS 500 // on, then off as long
#define OSD_DEMO_DISARM_DOT_LINES 3

static bool disarmArmed;
static bool demoMusicMuted; // the silent scene stopped the music: start it again after it

// Text (true) or black into code buffer buf.
static void osdDemoDisarmBuild(uint8_t buf, bool text)
{
    const uint8_t black = osdDemo3c[0];
    for (uint16_t l = 0; l < OSD_DEMO_MG_ROWS_MAX * OSD_DEMO_CELL_H; l++) {
        memset(bmCode[buf][l], black, OSD_DEMO_CHARS_PER_LINE);
        bmInv[buf][l] = osdDemo3cInv[0] ? 0x3FFFFFFFu : 0u;
    }
    if (!text) {
        return;
    }
    // The text's font pixels: 3-px columns (120 a line) × font rows, kerned
    enum { COLS = OSD_DEMO_CHARS_PER_LINE * 4 };
    uint8_t ink[OSD_DEMO_FONT_H][COLS / 8];
    memset(ink, 0, sizeof(ink));
    const char *t = OSD_DEMO_DISARM_TEXT;
    uint16_t w = 0;
    for (const char *c = t; *c; c++) {
        uint8_t c0, cw;
        osdDemoS1CharInk(*c, &c0, &cw);
        w = (uint16_t)(w + cw + (c[1] ? 1u : 0u));
    }
    uint16_t x = (uint16_t)((COLS - MIN(w, (uint16_t)COLS)) / 2u);
    uint8_t rowLo = OSD_DEMO_FONT_H, rowHi = 0;
    for (const char *c = t; *c; c++) {
        uint8_t c0, cw;
        osdDemoS1CharInk(*c, &c0, &cw);
        for (uint8_t fx = 0; fx < cw && x + fx < COLS; fx++) {
            for (uint8_t fy = 0; fy < OSD_DEMO_FONT_H; fy++) {
                if (osdDemoFontPixel((uint8_t)*c, (uint8_t)(c0 + fx), fy)) {
                    ink[fy][(x + fx) >> 3] |= (uint8_t)(0x80u >> ((x + fx) & 7u));
                    rowLo = MIN(rowLo, fy);
                    rowHi = MAX(rowHi, fy);
                }
            }
        }
        x = (uint16_t)(x + cw + 1u);
    }
    if (rowLo > rowHi) {
        return;
    }
    // The inked rows (a font pixel OSD_DEMO_DISARM_DOT_LINES lines tall) inside the middle char
    // row, off its first line: in a field the slot engine misses (the first ones after the entry)
    // the chip shows each row's first line — the VBLANK preload — on all its 18 lines, and a text
    // line there turned the row into full-height white stripes.
    const uint16_t h = (uint16_t)(rowHi - rowLo + 1u) * OSD_DEMO_DISARM_DOT_LINES;
    const uint16_t rowMid = (OSD_DEMO_MG_ROWS_MAX * OSD_DEMO_CELL_H / 2u) / OSD_DEMO_CELL_H;
    const uint16_t y0 = (uint16_t)(rowMid * OSD_DEMO_CELL_H
                                   + ((h < OSD_DEMO_CELL_H) ? MAX((OSD_DEMO_CELL_H - h) / 2u, 1u) : 1u));
    for (uint16_t l = 0; l < h; l++) {
        const uint8_t *r = ink[rowLo + l / OSD_DEMO_DISARM_DOT_LINES];
        uint8_t *code = bmCode[buf][y0 + l];
        uint32_t inv = 0;
        for (uint8_t cx = 0; cx < OSD_DEMO_CHARS_PER_LINE; cx++) {
            uint8_t p = 0; // base 3, left column first: 0 black, 2 white
            for (uint8_t k = 0; k < 4u; k++) {
                const uint16_t col = (uint16_t)(cx * 4u + k);
                p = (uint8_t)(p * 3u + (((r[col >> 3] >> (7u - (col & 7u))) & 1u) ? 2u : 0u));
            }
            code[cx] = osdDemo3c[p];
            inv |= (uint32_t)osdDemo3cInv[p] << cx;
        }
        bmInv[buf][y0 + l] = inv;
    }
}

// VBLANK: buffer 0 (the text) or 1 (black) by the blink, switched on a frame start.
static void osdDemoDisarmField(uint8_t cols, uint8_t rows, bool frameStart, osdDemoRasterMotion_t *next)
{
    UNUSED(cols);
    UNUSED(rows);
    if (frameStart) {
        plasmaPlay = (((millis() - fxStartMs) / OSD_DEMO_DISARM_BLINK_MS) & 1u) ? 1u : 0u;
    }
    next->vos = 0;
}

static void osdDemoDisarmTask(void)
{
}

static const osdDemoIrqScene_t osdDemoDisarmScene = {
    .bandLines = 1,
    .bands = OSD_DEMO_CELL_H,
    .armed = &disarmArmed,
    .field = osdDemoDisarmField,
    .band = osdDemoBmBand,
    .task = osdDemoDisarmTask,
};

static void osdDemoDisarmEnter(void)
{
    if (chiptuneIsPlaying()) {
        chiptuneStop(); // the end: silence
        demoMusicMuted = true;
    }
    osdDemoApplyOsdmNow();
    max7456Brightness(0, 3);
    osdDemoBgType(DISPLAY_BACKGROUND_BLACK);
    max7456SetHudMotionOffset(0, 0);
    if (!demoDisplay) {
        return;
    }
    osdDemoFillRowsGlyphFast(demoDisplay->cols, demoDisplay->rows, OSD_DEMO_PIXEL_OFF, OSD_DEMO_PIXEL_OFF);
    memset(mgSram, 0xFF, sizeof(mgSram));
    mgLastEdgeTicks = 0;
    osdDemoDisarmBuild(0, true);
    osdDemoDisarmBuild(1, false);
    plasmaPlay = 0;
    plasmaNextReady = false;
    (void)osdDemoIrqStart(&osdDemoDisarmScene);
}

// --- Scene 12: shaded cube, warp stars and an orbiting scroller on the bitmap engine ----------
// Ported from the cube618 v2 model (2 px × 1 line pixels, black / white per cell line plus one
// grey pixel where a cell line holds no white): a solid cube, its visible faces lit by one point
// light near the camera (an 8×8 ordered dither of the shade, white squares with a black rim in
// the corners of every face), grey streaks of a star tunnel flying past (never in a cell the
// cube touches; a cell line's greys as the grey set has them), and a 5×5 text on an ellipse around the
// cube — white with a black outline on the near side, fading out in grey dither on the far
// side, hidden by the cube where it is behind it. The stars flash white on every beat. The frame is rasterised line by line straight
// into scene 13's code buffers in the build interrupt (no pixel planes: they do not fit in RAM):
// a cell line holds its 6-bit white mask (bit 5 = left) | 0x40 opaque, or 0x80 | its grey mask,
// then becomes a glyph (the 32 black / white ones ± INV, or the grey set's pattern).
// It turns endlessly (no loop); the stars fly at the model's speed (150 frames = its 180 at 30).
// Cheap on purpose: angles from the sine table (interpolated), no libm trig or rounding calls,
// faces filled a cell at a time with the shade in fixed point, streaks by integer steps, the
// scroller's depth test only where a dot can be behind the cube.
#define CUBE_FRAMES        150u
#define CUBE_LW            (OSD_DEMO_CHARS_PER_LINE * 6) // 180 logical px
#define CUBE_SCALE_X       120.0f
#define CUBE_SCALE_Y       240.0f
#define CUBE_CAM           5.0f
#define CUBE_STARS         240u
#define CUBE_STAR_RADIUS   9.5f
#define CUBE_STAR_TRAVEL   24.0f
#define CUBE_STAR_SPEED    (4.0f * CUBE_STAR_TRAVEL / (float)CUBE_FRAMES)
#define CUBE_STAR_WRAP     75u // frames the stars move exactly two tube lengths in: seamless
#define CUBE_SCR_R         4.35f
#define CUBE_SCR_XSQ       0.52f
#define CUBE_SCR_YSQ       0.38f
#define CUBE_SCR_TILT      0.55f
#define CUBE_SCR_PULL      1.45f
#define CUBE_SCR_ARC       (2.0f * 5.0f / CUBE_SCALE_X)
#define CUBE_SCR_CAMPX     (4.0f * 5.0f / CUBE_SCALE_Y)
#define CUBE_SCR_TURNS     0.22f
#define CUBE_SCR_WHITE_T   0.58f

static const char cubeScrollText[] =
    "2X1 PX DOTS * BLACK, GREY AND WHITE * NEW GLYPHS EVERY LINE * ";
// Kerned like scene 1 (osdDemoS1CharInk): each character's first strip column, the strip width
static uint16_t cubeCharStart[sizeof(cubeScrollText)];
static uint16_t cubeStripW;
static bool cubeBgPending;
// On every beat the background stars flash white (set in the field hook, read by the build)
#define OSD_DEMO_CUBE_BEAT_MS   500 // a quarter at 120 BPM (the tracker lines nearest it)
#define OSD_DEMO_CUBE_FLASH_Q8  40  // white for this share of the beat (/256): ~80 ms
static volatile bool cubeStarsWhite;
static uint32_t cubeCells[OSD_DEMO_MG_ROWS_MAX]; // cells the cube touches this frame (no stars)
static uint16_t cubeLines;                       // lines of the frame being built
static float cubeNearDepth;                      // the cube's nearest vertex this frame

// The visible faces as triangles: screen vertices, the shade and 1/depth as screen planes
typedef struct {
    float x[3], y[3];
    float sA, sB, sC;  // shade = sA + sB·x + sC·y
    float zA, zB, zC;  // 1/depth = zA + zB·x + zC·y
} cubeTri_t;
static cubeTri_t cubeTris[6];
static uint8_t cubeTriCount;

// 8×8 ordered dither, thresholds in Q16 ((b + ½) / 64)
#define CB8(b) ((int32_t)(b) * 1024 + 512)
static const int32_t cubeBayer8Q16[64] = {
    CB8(0), CB8(32), CB8(8), CB8(40), CB8(2), CB8(34), CB8(10), CB8(42), CB8(48), CB8(16), CB8(56), CB8(24), CB8(50), CB8(18), CB8(58), CB8(26),
    CB8(12), CB8(44), CB8(4), CB8(36), CB8(14), CB8(46), CB8(6), CB8(38), CB8(60), CB8(28), CB8(52), CB8(20), CB8(62), CB8(30), CB8(54), CB8(22),
    CB8(3), CB8(35), CB8(11), CB8(43), CB8(1), CB8(33), CB8(9), CB8(41), CB8(51), CB8(19), CB8(59), CB8(27), CB8(49), CB8(17), CB8(57), CB8(25),
    CB8(15), CB8(47), CB8(7), CB8(39), CB8(13), CB8(45), CB8(5), CB8(37), CB8(63), CB8(31), CB8(55), CB8(23), CB8(61), CB8(29), CB8(53), CB8(21),
};
#undef CB8
static const uint8_t cubeBayer4[16] = { 0, 8, 2, 10, 12, 4, 14, 6, 3, 11, 1, 9, 15, 7, 13, 5 };

// Fast float → int: round to nearest / floor / ceil for |x| < 16384 (no libm calls)
static inline int16_t cubeRound(float x)
{
    return (int16_t)((int32_t)(x + 16384.5f) - 16384);
}

static inline int16_t cubeFloor(float x)
{
    return (int16_t)((int32_t)(x + 16384.0f) - 16384);
}

static inline int16_t cubeCeil(float x)
{
    return (int16_t)(16384 - (int32_t)(16384.0f - x));
}

// sin / cos of a turn fraction (a: 65536 = a full turn) from the 256-entry table, interpolated
static inline float cubeSin(uint16_t a)
{
    const uint8_t i = (uint8_t)(a >> 8);
    const int32_t f = a & 0xFF;
    const int32_t v = (int32_t)osdDemoSineAt(i) * (256 - f) + (int32_t)osdDemoSineAt((uint8_t)(i + 1u)) * f;
    return (float)v * (1.0f / (127.0f * 256.0f));
}

static inline float cubeCos(uint16_t a)
{
    return cubeSin((uint16_t)(a + 16384u));
}

static inline uint16_t cubeTurn(float rad) // radians → turn fraction
{
    return (uint16_t)(int32_t)(rad * (65536.0f / (2.0f * M_PIf)));
}

static inline uint8_t *osdDemoCubeCell(uint8_t buf, int16_t x, int16_t y)
{
    return &bmCode[buf][y][x / 6];
}

// A pixel white (or black when !white), the cell line opaque
static inline void osdDemoCubePx(uint8_t buf, int16_t x, int16_t y, bool white)
{
    if ((uint16_t)x >= CUBE_LW || (uint16_t)y >= cubeLines) {
        return;
    }
    uint8_t *c = osdDemoCubeCell(buf, x, y);
    const uint8_t bit = (uint8_t)(0x20u >> (x % 6));
    uint8_t v = (*c & 0x40u) ? *c : 0x40u; // a grey there gives way
    *c = white ? (uint8_t)(v | bit) : (uint8_t)(v & (uint8_t)~bit);
}

// A white star pixel (the beat flash): where the cell line is empty or holds white stars only
// (0xC0: opaque, stars); never in a cell of the cube, never over the scroller
static inline void osdDemoCubeStarWhite(uint8_t buf, int16_t x, int16_t y)
{
    if ((uint16_t)x >= CUBE_LW || (uint16_t)y >= cubeLines || (cubeCells[y / OSD_DEMO_CELL_H] & (1u << (x / 6)))) {
        return;
    }
    uint8_t *c = osdDemoCubeCell(buf, x, y);
    if (*c == 0u || (*c & 0xC0u) == 0xC0u) {
        *c = (uint8_t)(0xC0u | (*c & 0x3Fu) | (0x20u >> (x % 6)));
    }
}

// A grey pixel where the cell line has no white (and no cube in the cell): the cell line's greys
// gather as a mask (0x80 | mask, bit 5 = left); the glyph is the grey set's pattern for it, or
// the fullest one of the set inside it (cubeGreyCode) — a star streak shows a half or a whole
// grey cell line where it covers one, a dot elsewhere.
static uint8_t cubeGreyCode[64]; // mask → grey glyph (0: none)

static void osdDemoCubeGreyTable(void)
{
    for (uint8_t m = 0; m < 64u; m++) {
        uint8_t best = 0, bestBits = 0;
        for (uint8_t sub = m; ; sub = (uint8_t)((sub - 1u) & m)) { // every subset of m
            const uint8_t code = sub ? osdDemoGreyCode(sub) : 0u;
            uint8_t bits = 0;
            for (uint8_t k = sub; k; k &= (uint8_t)(k - 1u)) {
                bits++;
            }
            if (code && bits > bestBits) {
                best = code;
                bestBits = bits;
            }
            if (!sub) {
                break;
            }
        }
        cubeGreyCode[m] = best;
    }
}

static inline void osdDemoCubeGrey(uint8_t buf, int16_t x, int16_t y, bool starsRule)
{
    if ((uint16_t)x >= CUBE_LW || (uint16_t)y >= cubeLines) {
        return;
    }
    if (starsRule && (cubeCells[y / OSD_DEMO_CELL_H] & (1u << (x / 6)))) {
        return;
    }
    uint8_t *c = osdDemoCubeCell(buf, x, y);
    if (*c == 0u || (*c & 0xC0u) == 0x80u) {
        *c = (uint8_t)(0x80u | *c | (0x20u >> (x % 6)));
    }
}

// Rotation: v' = v · (Rxᵀ Ryᵀ Rzᵀ). Endless, never looping: each axis turns at its own rate
// (turn / 65536 a frame; X and Z about the model's one turn in 6 s, Y faster), rates with no
// common period, the angles wrapping on their own (16 bits).
#define CUBE_RATE_X 437u
#define CUBE_RATE_Y 683u
#define CUBE_RATE_Z 311u
static void osdDemoCubeRot(uint32_t frame, float m[3][3])
{
    const uint16_t a = (uint16_t)(frame * CUBE_RATE_X);
    const uint16_t b = (uint16_t)(cubeTurn(0.55f) + frame * CUBE_RATE_Y);
    const uint16_t c = (uint16_t)(cubeTurn(0.3f) + frame * CUBE_RATE_Z);
    const float ca = cubeCos(a), sa = cubeSin(a), cb = cubeCos(b), sb = cubeSin(b), cc = cubeCos(c), sc = cubeSin(c);
    // (Rxᵀ Ryᵀ) Rzᵀ written out
    const float t00 = cb, t01 = 0.0f, t02 = -sb;
    const float t10 = sa * sb, t11 = ca, t12 = sa * cb;
    const float t20 = ca * sb, t21 = -sa, t22 = ca * cb;
    m[0][0] = t00 * cc - t01 * sc;  m[0][1] = t00 * sc + t01 * cc;  m[0][2] = t02;
    m[1][0] = t10 * cc - t11 * sc;  m[1][1] = t10 * sc + t11 * cc;  m[1][2] = t12;
    m[2][0] = t20 * cc - t21 * sc;  m[2][1] = t20 * sc + t21 * cc;  m[2][2] = t22;
}

static inline void osdDemoCubeMul(const float v[3], const float m[3][3], float out[3])
{
    out[0] = v[0] * m[0][0] + v[1] * m[1][0] + v[2] * m[2][0];
    out[1] = v[0] * m[0][1] + v[1] * m[1][1] + v[2] * m[2][1];
    out[2] = v[0] * m[0][2] + v[1] * m[1][2] + v[2] * m[2][2];
}

static inline void osdDemoCubeProj(const float v[3], float *x, float *y)
{
    const float r = 1.0f / (v[2] + CUBE_CAM);
    *x = (float)CUBE_LW * 0.5f + CUBE_SCALE_X * v[0] * r;
    *y = (float)cubeLines * 0.5f + CUBE_SCALE_Y * v[1] * r;
}

// The model's shade at a face vertex (point light, in-plane falloff normalised per face)
static const float cubeLight[3] = { 1.2f, -1.8f, -2.8f };

static float osdDemoCubeShade(const float pos[3], const float n[3], float alongN)
{
    const float tl[3] = { cubeLight[0] - pos[0], cubeLight[1] - pos[1], cubeLight[2] - pos[2] };
    const float dist = sqrtf(tl[0] * tl[0] + tl[1] * tl[1] + tl[2] * tl[2]);
    const float ndl = MAX(0.0f, (tl[0] * n[0] + tl[1] * n[1] + tl[2] * n[2]) / MAX(dist, 1e-6f));
    const float atten = 1.0f / (1.0f + 0.10f * dist + 0.02f * dist * dist);
    const float zt = constrainf(1.20f - 0.22f * ((pos[2] + CUBE_CAM - 3.0f) / 2.5f), 0.65f, 1.20f);
    // ndl^0.8 ≈ ndl · (1.2 − 0.2·ndl) on 0..1 (error < 2 %)
    const float lit = ndl * (1.2f - 0.2f * ndl) * (0.60f + 0.40f * alongN);
    return constrainf((0.10f + 0.85f * lit) * atten * zt, 0.05f, 0.76f);
}

static float osdDemoCubeAlong(const float pos[3], const float n[3], const float ctr[3])
{
    float l[3] = { cubeLight[0] - pos[0], cubeLight[1] - pos[1], cubeLight[2] - pos[2] };
    const float ln = l[0] * n[0] + l[1] * n[1] + l[2] * n[2];
    const float lp[3] = { l[0] - n[0] * ln, l[1] - n[1] * ln, l[2] - n[2] * ln };
    const float lpn = sqrtf(lp[0] * lp[0] + lp[1] * lp[1] + lp[2] * lp[2]);
    return ((pos[0] - ctr[0]) * lp[0] + (pos[1] - ctr[1]) * lp[1] + (pos[2] - ctr[2]) * lp[2]) / MAX(lpn, 1e-6f);
}

// A screen plane through three points: v = A + B·x + C·y
static void osdDemoCubePlane(const float *x, const float *y, const float *v, float *a, float *b, float *c)
{
    const float d = (x[1] - x[0]) * (y[2] - y[0]) - (x[2] - x[0]) * (y[1] - y[0]);
    if (fabsf(d) < 1e-6f) {
        *a = v[0];
        *b = *c = 0.0f;
        return;
    }
    const float id = 1.0f / d;
    *b = ((v[1] - v[0]) * (y[2] - y[0]) - (v[2] - v[0]) * (y[1] - y[0])) * id;
    *c = ((x[1] - x[0]) * (v[2] - v[0]) - (x[2] - x[0]) * (v[1] - v[0])) * id;
    *a = v[0] - *b * x[0] - *c * y[0];
}

// The span of a convex polygon on line y (pixel centres): [*l, *r], false if none
static bool osdDemoCubeSpan(const float *px, const float *py, uint8_t n, int16_t y, int16_t *l, int16_t *r)
{
    const float yc = (float)y + 0.5f;
    float xl = 1e9f, xr = -1e9f;
    for (uint8_t i = 0; i < n; i++) {
        const uint8_t j = (uint8_t)((i + 1u == n) ? 0u : i + 1u);
        const float y0 = py[i], y1 = py[j];
        if ((yc < y0 && yc < y1) || (yc > y0 && yc > y1) || y0 == y1) {
            continue;
        }
        const float x = px[i] + (px[j] - px[i]) * (yc - y0) / (y1 - y0);
        xl = MIN(xl, x);
        xr = MAX(xr, x);
    }
    if (xr < xl) {
        return false;
    }
    *l = cubeCeil(xl - 0.5f);
    *r = cubeFloor(xr - 0.5f);
    return *r >= *l;
}

// The cube's depth at a screen point (a big number where it is not): for the scroller
static float osdDemoCubeDepthAt(float x, float y)
{
    for (uint8_t i = 0; i < cubeTriCount; i++) {
        const cubeTri_t *t = &cubeTris[i];
        const float s0 = (t->x[1] - t->x[0]) * (y - t->y[0]) - (t->y[1] - t->y[0]) * (x - t->x[0]);
        const float s1 = (t->x[2] - t->x[1]) * (y - t->y[1]) - (t->y[2] - t->y[1]) * (x - t->x[1]);
        const float s2 = (t->x[0] - t->x[2]) * (y - t->y[2]) - (t->y[0] - t->y[2]) * (x - t->x[2]);
        if ((s0 >= 0 && s1 >= 0 && s2 >= 0) || (s0 <= 0 && s1 <= 0 && s2 <= 0)) {
            const float iz = t->zA + t->zB * x + t->zC * y;
            return (iz > 1e-6f) ? 1.0f / iz : 1e9f;
        }
    }
    return 1e9f;
}

// A triangle's dithered shade, a cell at a time: each covered pixel white where the shade beats
// the 8×8 threshold, black elsewhere; the cells touched are marked (no stars there).
static void osdDemoCubeFillTri(uint8_t buf, const cubeTri_t *t)
{
    const int16_t y0 = MAX((int16_t)0, cubeFloor(MIN(MIN(t->y[0], t->y[1]), t->y[2])));
    const int16_t y1 = MIN((int16_t)(cubeLines - 1u), cubeCeil(MAX(MAX(t->y[0], t->y[1]), t->y[2])));
    const int32_t dS = (int32_t)(t->sB * 65536.0f); // shade step a pixel, Q16
    for (int16_t y = y0; y <= y1; y++) {
        int16_t l, r;
        if (!osdDemoCubeSpan(t->x, t->y, 3, y, &l, &r)) {
            continue;
        }
        l = MAX(l, (int16_t)0);
        r = MIN(r, (int16_t)(CUBE_LW - 1));
        if (r < l) {
            continue;
        }
        const int32_t *thr = &cubeBayer8Q16[(y & 7) << 3];
        int32_t sq = (int32_t)((t->sA + t->sB * ((float)l + 0.5f) + t->sC * ((float)y + 0.5f)) * 65536.0f);
        uint8_t *code = bmCode[buf][y];
        uint32_t cells = 0;
        int16_t x = l;
        while (x <= r) {
            const uint8_t c = (uint8_t)(x / 6);
            const int16_t xe = MIN(r, (int16_t)(c * 6 + 5));
            uint8_t span = 0, white = 0;
            for (; x <= xe; x++, sq += dS) {
                const uint8_t bit = (uint8_t)(0x20u >> (x - c * 6));
                span |= bit;
                if (sq > thr[x & 7]) {
                    white |= bit;
                }
            }
            const uint8_t v = (code[c] & 0x40u) ? code[c] : 0x40u;
            code[c] = (uint8_t)((v & (uint8_t)~span) | white);
            cells |= 1u << c;
        }
        cubeCells[y / OSD_DEMO_CELL_H] |= cells;
    }
}

static void osdDemoCubeFaces(uint8_t buf, const float m[3][3])
{
    static const int8_t fn[6][3] = { { -1, 0, 0 }, { 1, 0, 0 }, { 0, -1, 0 }, { 0, 1, 0 }, { 0, 0, -1 }, { 0, 0, 1 } };
    static const uint8_t fv[6][4] = { { 0, 1, 3, 2 }, { 4, 5, 7, 6 }, { 0, 1, 5, 4 }, { 2, 3, 7, 6 }, { 0, 2, 6, 4 }, { 1, 3, 7, 5 } };
    static const uint8_t tri[2][3] = { { 0, 1, 2 }, { 0, 2, 3 } };
    static const float uv[4][2] = { { 0.0f, 0.0f }, { 0.85f, 0.0f }, { 0.85f, 0.85f }, { 0.0f, 0.85f } };
    float vc[8][3], vx[8], vy[8];
    cubeNearDepth = 1e9f;
    for (uint8_t i = 0; i < 8u; i++) {
        const float v[3] = { (i & 4u) ? 1.0f : -1.0f, (i & 2u) ? 1.0f : -1.0f, (i & 1u) ? 1.0f : -1.0f };
        osdDemoCubeMul(v, m, vc[i]);
        osdDemoCubeProj(vc[i], &vx[i], &vy[i]);
        cubeNearDepth = MIN(cubeNearDepth, vc[i][2] + CUBE_CAM);
    }
    cubeTriCount = 0;
    for (uint8_t f = 0; f < 6u; f++) {
        // A face normal is a matrix row (± by the axis)
        const uint8_t ax = (uint8_t)(f >> 1);
        const float sg = (float)(fn[f][0] + fn[f][1] + fn[f][2]);
        const float n[3] = { sg * m[ax][0], sg * m[ax][1], sg * m[ax][2] };
        float ctr[3] = { 0, 0, 0 };
        for (uint8_t k = 0; k < 4u; k++) {
            for (uint8_t j = 0; j < 3u; j++) {
                ctr[j] += vc[fv[f][k]][j] * 0.25f;
            }
        }
        // Visible: the normal towards the camera at (0, 0, −5)
        if (-n[0] * ctr[0] - n[1] * ctr[1] + n[2] * (-CUBE_CAM - ctr[2]) <= 0.0f) {
            continue;
        }
        float qx[4], qy[4], sh[4], iz[4], al[4];
        float amin = 1e9f, amax = -1e9f;
        for (uint8_t k = 0; k < 4u; k++) {
            const uint8_t v = fv[f][k];
            qx[k] = vx[v];
            qy[k] = vy[v];
            iz[k] = 1.0f / (vc[v][2] + CUBE_CAM);
            al[k] = osdDemoCubeAlong(vc[v], n, ctr);
            amin = MIN(amin, al[k]);
            amax = MAX(amax, al[k]);
        }
        const float ia = 1.0f / MAX(amax - amin, 1e-6f);
        for (uint8_t k = 0; k < 4u; k++) {
            sh[k] = osdDemoCubeShade(vc[fv[f][k]], n, (al[k] - amin) * ia);
        }
        for (uint8_t h = 0; h < 2u && cubeTriCount < ARRAYLEN(cubeTris); h++) {
            cubeTri_t *t = &cubeTris[cubeTriCount++];
            float sv[3], zv[3];
            for (uint8_t k = 0; k < 3u; k++) {
                t->x[k] = qx[tri[h][k]];
                t->y[k] = qy[tri[h][k]];
                sv[k] = sh[tri[h][k]];
                zv[k] = iz[tri[h][k]];
            }
            osdDemoCubePlane(t->x, t->y, sv, &t->sA, &t->sB, &t->sC);
            osdDemoCubePlane(t->x, t->y, zv, &t->zA, &t->zB, &t->zC);
            osdDemoCubeFillTri(buf, t);
        }
        // Corner squares (UV 0 .. 0.15 from each corner): a black rim, then white
        const float *p0 = vc[fv[f][0]], *p1 = vc[fv[f][1]], *p3 = vc[fv[f][3]];
        float mx[4][4], my[4][4];
        int16_t mY0[4], mY1[4];
        for (uint8_t q = 0; q < 4u; q++) {
            float lo = 1e9f, hi = -1e9f;
            for (uint8_t k = 0; k < 4u; k++) {
                const float u = uv[q][0] + ((k == 1u || k == 2u) ? 0.15f : 0.0f);
                const float v = uv[q][1] + ((k >= 2u) ? 0.15f : 0.0f);
                const float pt[3] = { p0[0] + u * (p1[0] - p0[0]) + v * (p3[0] - p0[0]),
                                      p0[1] + u * (p1[1] - p0[1]) + v * (p3[1] - p0[1]),
                                      p0[2] + u * (p1[2] - p0[2]) + v * (p3[2] - p0[2]) };
                osdDemoCubeProj(pt, &mx[q][k], &my[q][k]);
                lo = MIN(lo, my[q][k]);
                hi = MAX(hi, my[q][k]);
            }
            mY0[q] = cubeFloor(lo);
            mY1[q] = cubeCeil(hi);
        }
        for (uint8_t pass = 0; pass < 2u; pass++) {
            for (uint8_t q = 0; q < 4u; q++) {
                for (int16_t y = (int16_t)(mY0[q] - 1); y <= mY1[q] + 1; y++) {
                    int16_t l, r;
                    if (pass == 0u) { // the rim: the square's span one px wider, on its lines ±1
                        int16_t l2, r2;
                        bool any = false;
                        l = 32767;
                        r = -32768;
                        for (int8_t d = -1; d <= 1; d++) {
                            if (osdDemoCubeSpan(mx[q], my[q], 4, (int16_t)(y + d), &l2, &r2)) {
                                l = MIN(l, l2);
                                r = MAX(r, r2);
                                any = true;
                            }
                        }
                        if (any) {
                            for (int16_t x = (int16_t)(l - 1); x <= r + 1; x++) {
                                osdDemoCubePx(buf, x, y, false);
                            }
                        }
                    } else if (osdDemoCubeSpan(mx[q], my[q], 4, y, &l, &r)) {
                        for (int16_t x = l; x <= r; x++) {
                            osdDemoCubePx(buf, x, y, true);
                        }
                    }
                }
            }
        }
    }
}

static void osdDemoCubeTextLayout(void)
{
    uint16_t x = 0;
    for (uint16_t i = 0; i + 1u < sizeof(cubeScrollText); i++) {
        uint8_t c0, w;
        osdDemoS1CharInk(cubeScrollText[i], &c0, &w);
        cubeCharStart[i] = x;
        x = (uint16_t)(x + w + 1u);
    }
    cubeStripW = x;
}

// The kerned strip: font column of strip column u (5 bits, bit v = font row v), 0 in the gaps
static uint8_t osdDemoCubeScrCol(uint16_t u)
{
    uint16_t lo = 0, hi = (uint16_t)(sizeof(cubeScrollText) - 2u); // the character holding u
    while (lo < hi) {
        const uint16_t mid = (uint16_t)((lo + hi + 1u) / 2u);
        if (cubeCharStart[mid] <= u) {
            lo = mid;
        } else {
            hi = (uint16_t)(mid - 1u);
        }
    }
    uint8_t c0, w;
    osdDemoS1CharInk(cubeScrollText[lo], &c0, &w);
    const uint16_t col = (uint16_t)(u - cubeCharStart[lo]);
    if (col >= w) {
        return 0;
    }
    const uint8_t code = (uint8_t)cubeScrollText[lo];
    uint8_t bits = 0;
    for (uint8_t v = 0; v < 5u; v++) {
        if (osdDemoFontBits[code][OSD_DEMO_TUNNEL_INK_Y0 + v] & (0x80u >> (c0 + col))) {
            bits |= (uint8_t)(1u << v);
        }
    }
    return bits;
}

static void osdDemoCubeScroller(uint8_t buf, uint32_t frame)
{
    static uint16_t nSlots, sw;
    if (!nSlots) { // constants of the ring
        const float a = CUBE_SCR_R * CUBE_SCR_XSQ, b = CUBE_SCR_R * CUBE_SCR_YSQ;
        const float h = (a - b) * (a - b) / ((a + b) * (a + b));
        const float perim = M_PIf * (a + b) * (1.0f + 3.0f * h / (10.0f + sqrtf(4.0f - 3.0f * h)));
        nSlots = (uint16_t)MAX(1, cubeRound(perim / CUBE_SCR_ARC));
        osdDemoCubeTextLayout();
        sw = cubeStripW;
    }
    const float a = CUBE_SCR_R * CUBE_SCR_XSQ, b = CUBE_SCR_R * CUBE_SCR_YSQ;
    const uint16_t tilt = cubeTurn(CUBE_SCR_TILT);
    const float ct = cubeCos(tilt), stl = cubeSin(tilt);
    const float zAmp = b * stl;
    const uint16_t shift = (uint16_t)(((uint32_t)((float)frame * (CUBE_SCR_TURNS / (float)CUBE_FRAMES) * (float)sw)) % sw);
    const float ringDepth = CUBE_CAM - CUBE_SCR_PULL;
    const float z0 = ringDepth - CUBE_CAM, dNear = ringDepth - zAmp, iSpan = 1.0f / MAX(2.0f * zAmp, 1e-6f);
    const float midX = (float)CUBE_LW * 0.5f, midY = (float)cubeLines * 0.5f;
    const float yAsp = 1.0f / (CUBE_SCALE_Y / CUBE_SCALE_X);
    // Pass 0: the black outline of the near (white) dots, 1: the white dots, 2: the far grey
    for (uint8_t pass = 0; pass < 3u; pass++) {
        for (uint16_t i = 0; i < nSlots; i++) {
            const uint8_t col = osdDemoCubeScrCol((uint16_t)((i + shift) % sw));
            if (!col) {
                continue;
            }
            const uint16_t ang = (uint16_t)((uint32_t)i * 65536u / nSlots);
            const float c = cubeCos(ang), sn = cubeSin(ang);
            const float yr = b * sn;
            const float depth = z0 + yr * stl + CUBE_CAM;
            const float bright = 1.0f - constrainf((depth - dNear) * iSpan, 0.0f, 1.0f);
            const bool white = bright >= CUBE_SCR_WHITE_T;
            if (depth < 1.05f || bright <= 1e-3f || (pass < 2u) != white) {
                continue;
            }
            const float rd = 1.0f / depth;
            const float yc = yr * ct * yAsp;
            const int16_t dx = cubeRound(midX + CUBE_SCALE_X * a * c * rd - 0.5f);
            const bool test = depth > cubeNearDepth; // nearer than the cube's nearest corner: in front
            for (uint8_t v = 0; v < 5u; v++) {
                if (!(col & (1u << v))) {
                    continue;
                }
                const float qy = yc + ((float)v - 2.0f) * CUBE_SCR_CAMPX;
                const int16_t dy = (int16_t)(cubeRound(midY + CUBE_SCALE_Y * qy * rd - 0.5f) - 1);
                if (test && osdDemoCubeDepthAt((float)dx + 1.0f, (float)dy + 2.0f) <= depth) {
                    continue; // behind the cube
                }
                if (pass == 0u) {
                    for (int16_t y = (int16_t)(dy - 1); y <= dy + 4; y++) {
                        for (int16_t x = (int16_t)(dx - 1); x <= dx + 2; x++) {
                            osdDemoCubePx(buf, x, y, false);
                        }
                    }
                } else if (pass == 1u) {
                    for (int16_t y = dy; y < dy + 4; y++) {
                        osdDemoCubePx(buf, dx, y, true);
                        osdDemoCubePx(buf, (int16_t)(dx + 1), y, true);
                    }
                } else {
                    const float dens16 = bright * (16.0f / CUBE_SCR_WHITE_T); // tonal perspective
                    for (int16_t y = dy; y < dy + 4; y++) {
                        for (int16_t x = dx; x < dx + 2; x++) {
                            if ((float)cubeBayer4[((y & 3) << 2) | (x & 3)] + 0.5f < dens16) {
                                osdDemoCubeGrey(buf, x, y, false);
                            }
                        }
                    }
                }
            }
        }
    }
}

// A star's own numbers, the same every frame (a hash of its index)
static inline uint32_t cubeHash(uint32_t *st)
{
    *st = *st * 1664525u + 1013904223u;
    return *st >> 8; // 24 bits
}

static void osdDemoCubeStars(uint8_t buf, const float m[3][3], uint32_t frame)
{
    // The tube's basis (fixed): forward = the warp direction, right ⟂ forward and y
    static const float fwd[3] = { 0.1759303f, -0.1172869f, 0.9773906f };   // (0.18, −0.12, 1) normalised
    static const float right[3] = { -0.9841833f, 0.0f, 0.1771530f };      // fwd × y, normalised
    static const float up[3] = { 0.0207777f, 0.9930981f, 0.1154318f };     // right × fwd
    float warp[3];
    osdDemoCubeMul(fwd, m, warp);
    const float half = 0.5f * CUBE_STAR_TRAVEL;
    const float move = CUBE_STAR_SPEED * (float)frame;
    const float midX = (float)CUBE_LW * 0.5f, midY = (float)cubeLines * 0.5f;
    const bool white = cubeStarsWhite; // the beat flash
    for (uint16_t i = 0; i < CUBE_STARS; i++) {
        uint32_t st = 0x9E3779B9u * (i + 1u);
        const uint16_t ang = (uint16_t)cubeHash(&st);
        const float rad = CUBE_STAR_RADIUS * sqrtf((float)cubeHash(&st) * (1.0f / 16777216.0f));
        const float along0 = ((float)cubeHash(&st) * (1.0f / 16777216.0f) - 0.5f) * CUBE_STAR_TRAVEL;
        const float scale = 0.70f + 0.50f * (float)cubeHash(&st) * (1.0f / 16777216.0f);
        float along = along0 - move + half;
        along -= CUBE_STAR_TRAVEL * (float)cubeFloor(along * (1.0f / CUBE_STAR_TRAVEL));
        along -= half;
        const float cr = rad * cubeCos(ang), sr = rad * cubeSin(ang);
        const float pos[3] = { cr * right[0] + sr * up[0] + along * fwd[0], cr * right[1] + sr * up[1] + along * fwd[1],
                               cr * right[2] + sr * up[2] + along * fwd[2] };
        float tip[3];
        osdDemoCubeMul(pos, m, tip);
        const float depth = tip[2] + CUBE_CAM;
        if (depth < 1.0f) {
            continue;
        }
        const float wlen = MIN(1.05f * scale * constrainf(2.0f / MAX(depth, 2.0f), 0.50f, 1.15f), 1.7f);
        float tail[3] = { tip[0] + warp[0] * wlen, tip[1] + warp[1] * wlen, tip[2] + warp[2] * wlen };
        if (tail[2] + CUBE_CAM < 1.0f) {
            const float t = (1.0f - depth) / ((tail[2] + CUBE_CAM) - depth);
            for (uint8_t k = 0; k < 3u; k++) {
                tail[k] = tip[k] + (tail[k] - tip[k]) * t;
            }
        }
        const float rt = 1.0f / depth, rl = 1.0f / (tail[2] + CUBE_CAM);
        const float x0 = midX + CUBE_SCALE_X * tip[0] * rt, y0 = midY + CUBE_SCALE_Y * tip[1] * rt;
        float dx = midX + CUBE_SCALE_X * tail[0] * rl - x0, dy = midY + CUBE_SCALE_Y * tail[1] * rl - y0;
        const float adx = fabsf(dx), ady = fabsf(dy), big = MAX(adx, ady);
        if (big < 1.1f) {
            continue; // the model's 1.5 px length, near enough
        }
        if (big > 24.0f) {
            dx *= 24.0f / big;
            dy *= 24.0f / big;
        }
        // The streak, tip to tail, one pixel a step on the long axis (Q16 on the other), one black
        // gap before its last pixel
        const int16_t n = (int16_t)(MIN(big, 24.0f)) + 1;
        const int32_t fx0 = (int32_t)((x0 - 0.5f) * 65536.0f) + 32768, fy0 = (int32_t)((y0 - 0.5f) * 65536.0f) + 32768;
        const int32_t sx = (int32_t)(dx * 65536.0f / (float)n), sy = (int32_t)(dy * 65536.0f / (float)n);
        int32_t fx = fx0, fy = fy0;
        for (int16_t k = 0; k <= n; k++, fx += sx, fy += sy) {
            if (n >= 2 && k == n - 1) {
                continue;
            }
            if (white) {
                osdDemoCubeStarWhite(buf, (int16_t)(fx >> 16), (int16_t)(fy >> 16));
            } else {
                osdDemoCubeGrey(buf, (int16_t)(fx >> 16), (int16_t)(fy >> 16), true);
            }
        }
    }
}

static void osdDemoCubeBuildFrame(uint8_t buf, uint8_t cols, uint8_t rows, uint32_t t)
{
    cubeLines = (uint16_t)(rows * OSD_DEMO_CELL_H);
    float m[3][3];
    osdDemoCubeRot(t, m);
    memset(bmCode[buf], 0, (size_t)cubeLines * OSD_DEMO_CHARS_PER_LINE);
    memset(cubeCells, 0, sizeof(cubeCells));
    osdDemoCubeFaces(buf, m);
    osdDemoCubeScroller(buf, t);
    osdDemoCubeStars(buf, m, t % CUBE_STAR_WRAP);
    // Cell lines → glyphs
    for (uint16_t l = 0; l < cubeLines; l++) {
        uint8_t *code = bmCode[buf][l];
        uint32_t inv = 0;
        for (uint8_t c = 0; c < cols; c++) {
            const uint8_t v = code[c];
            uint8_t g;
            if (v & 0x40u) {
                const uint8_t bits = (uint8_t)(v & 0x3Fu);
                if (bits & 0x20u) {
                    g = (uint8_t)(OSD_DEMO_BM_BASE + (~bits & 0x1Fu));
                    inv |= 1u << c;
                } else {
                    g = (uint8_t)(OSD_DEMO_BM_BASE + bits);
                }
            } else if ((v & 0x80u) && cubeGreyCode[v & 0x3Fu]) {
                g = cubeGreyCode[v & 0x3Fu];
            } else {
                g = OSD_DEMO_BM_BASE; // black
            }
            code[c] = g;
        }
        bmInv[buf][l] = inv;
    }
}

// Frame swap, the grey on once the screen is glyphs; the OSD stays put (no VOS).
static void osdDemoCubeField(uint8_t cols, uint8_t rows, bool frameStart, osdDemoRasterMotion_t *next)
{
    UNUSED(cols);
    UNUSED(rows);
    osdDemoTriFrame(frameStart);
    memset(mgSram, 0xFF, sizeof(mgSram)); // the VBLANK preload writes every first line
    if (cubeBgPending) {
        cubeBgPending = false;
        osdDemoBg(true, 7); // 49 %
    }
    // The background stars white for a moment on every beat (the music's grid; a free clock
    // without music), grey the rest of it
    cubeStarsWhite = osdDemoMusicPhase(OSD_DEMO_CUBE_BEAT_MS, fxStartMs) < OSD_DEMO_CUBE_FLASH_Q8;
    next->vos = 0; // no hop
}

static osdDemoIrqScene_t osdDemoCubeScene = {
    .bandLines = 1,
    .bands = OSD_DEMO_CELL_H,
    .armed = &cubeArmed,
    .field = osdDemoCubeField,
    .band = osdDemoBmBand,
    .task = osdDemoPbTask,
};

static void osdDemoCubeEnter(void)
{
    osdDemoApplyOsdmNow();
    max7456Brightness(0, 3);
    osdDemoBgType(DISPLAY_BACKGROUND_LTGRAY);
    osdDemoBg(true, 0); // black until the screen is glyphs; grey from field 1
    cubeBgPending = true;
    max7456SetHudMotionOffset(0, 0);
    if (!demoDisplay) {
        return;
    }
    osdDemoFillRowsGlyphFast(demoDisplay->cols, demoDisplay->rows, OSD_DEMO_BM_BASE, OSD_DEMO_BM_BASE);
    memset(mgSram, 0xFF, sizeof(mgSram));
    mgLastEdgeTicks = 0;
    plasmaPlay = 0;
    plasmaNextReady = false;
    pbReady = PB_NONE;
    pbBuildFn = osdDemoCubeBuildFrame;
    cubeStarsWhite = false;
    osdDemoCubeGreyTable();
    pbFrameNo = 0;
    pbBuiltT = 0;
    osdDemoCubeBuildFrame(0, MIN(demoDisplay->cols, OSD_DEMO_CHARS_PER_LINE), MIN(demoDisplay->rows, OSD_DEMO_MG_ROWS_MAX), pbBuiltT);
    if (!osdDemoIrqStart(&osdDemoCubeScene)) {
        return;
    }
    osdDemoPbBuildIrqOn(true);
}

// --- Scene 7 on the slot engine: 2×2 plasma, built in the build interrupt, written by DMA ----
// Same picture as before (180 × 144 mega-pixels of 2 px × 2 lines, 4 levels: black, dark grey,
// grey, white as static checkers, the bottom scroller), now through scene 13's bitmap engine:
// the frame is built in the build interrupt for the next frame start (triple buffer, 25 steps
// a second) and every line's 30 codes go out by DMA behind the beam — no OSD-task latency, no
// polled bursts. A band's codes go to both of its lines. Copper bars fly up and down through the
// row brightness, mega-pixel row by row (2 lines; the slot engine rewrites a row's RB register
// between its lines): a chain of four, the near ones thicker and in front, up and down once in
// four beats (music-locked); on the kick they flash (black core darker-lit, wider), the snare
// pushes the chain apart (easing back). Levels only the chip switches cleanly mid-row: the
// plasma white 80 %, a bar's edge white 120 %, its core white 120 % and black lifted to one level
// (20 %, 30 % on the kick) — 90/100 % white or a step between two lifted blacks flicker.
#define OSD_DEMO_P7_COPPER_BARS     4
#define OSD_DEMO_P7_COPPER_MS       2000 // up and down once: four beats (the tracker lines nearest)
#define OSD_DEMO_P7_COPPER_STEP     14   // the chain's bars apart (sine-table steps)
#define OSD_DEMO_P7_COPPER_PUSH     12   // the snare pushes them this much further apart
#define OSD_DEMO_P7_COPPER_PUSH_MS  300  // and back over this
#define OSD_DEMO_P7_COPPER_FLASH_MS 160  // the kick's flash
static timeMs_t p7BeatMs;            // the last kick
static timeMs_t p7SnareMs;           // the last snare
static uint8_t p7KickSeen, p7SnareSeen;
static uint8_t p7RbLine[OSD_DEMO_MG_ROWS_MAX * OSD_DEMO_CELL_H]; // RB a line (the engine's rbLine)

// The copper over mega-pixel rows 0..n−1: level 0 (none) .. 4 per row
static void osdDemoP7Copper(uint8_t *lvl, uint16_t n)
{
    memset(lvl, 0, n);
    const timeMs_t now = millis();
    const bool flash = (timeMs_t)(now - p7BeatMs) < OSD_DEMO_P7_COPPER_FLASH_MS;
    const uint8_t a0 = (uint8_t)osdDemoMusicPhase(OSD_DEMO_P7_COPPER_MS, fxStartMs);
    // the snare's push: at once, easing back (quadratic)
    const uint32_t ps = MIN((uint32_t)(now - p7SnareMs), (uint32_t)OSD_DEMO_P7_COPPER_PUSH_MS);
    const uint32_t rest = OSD_DEMO_P7_COPPER_PUSH_MS - ps;
    const int32_t step = OSD_DEMO_P7_COPPER_STEP
        + (int32_t)(OSD_DEMO_P7_COPPER_PUSH * rest * rest / (OSD_DEMO_P7_COPPER_PUSH_MS * OSD_DEMO_P7_COPPER_PUSH_MS));
    const int32_t mid = (int32_t)n / 2, amp = (int32_t)n / 2 - 6;
    // far ones first (the near ones over them): depth by the swing's other phase
    uint8_t order[OSD_DEMO_P7_COPPER_BARS];
    int8_t depth[OSD_DEMO_P7_COPPER_BARS];
    for (uint8_t i = 0; i < OSD_DEMO_P7_COPPER_BARS; i++) {
        depth[i] = osdDemoSineAt((uint8_t)(a0 - i * step + 64));
        uint8_t k = i;
        while (k > 0 && depth[order[k - 1u]] > depth[i]) {
            order[k] = order[k - 1u];
            k--;
        }
        order[k] = i;
    }
    for (uint8_t o = 0; o < OSD_DEMO_P7_COPPER_BARS; o++) {
        const uint8_t i = order[o];
        const int32_t c = mid + amp * osdDemoSineAt((uint8_t)(a0 - i * step)) / 127;
        const int32_t hh = ((depth[i] > 0) ? 6 : 4) + (flash ? 2 : 0); // half height (rows): near, far
        for (int32_t d = -hh; d <= hh; d++) {
            const int32_t r = c + d;
            if (r < 0 || r >= (int32_t)n) {
                continue;
            }
            const int32_t e = hh + 1 - (d < 0 ? -d : d); // 1 at the edge
            lvl[r] = (uint8_t)MIN(e + (flash ? 1 : 0), 4);
        }
    }
}

// 2-bit level cells of band (y, b) → codes and INV of its two lines.
static void osdDemoP7EmitBand(uint8_t buf, uint8_t y, uint8_t b, uint8_t cols, const uint16_t *cells)
{
    const uint16_t sel = osdDemoMgGraySel(y, b);
    const uint16_t half = ((y * OSD_DEMO_PLASMA2X2_BANDS + b) & 1u) ? 0x2Au : 0x15u;
    const uint16_t selDark = sel & half;
    const uint16_t line = (uint16_t)(y * OSD_DEMO_CELL_H + b * OSD_DEMO_PLASMA2X2_STEP);
    uint8_t *code = bmCode[buf][line];
    uint32_t inv = 0;
    for (uint8_t x = 0; x < cols; x++) {
        const uint16_t lo = mgLevelLut[cells[x] & 0x3Fu];
        const uint16_t hi = mgLevelLut[(cells[x] >> 6) & 0x3Fu];
        const uint16_t w = (uint16_t)((lo & 7u) | ((hi & 7u) << 3));
        const uint16_t g = (uint16_t)(((lo >> 3) & 7u) | (((hi >> 3) & 7u) << 3));
        const uint16_t d = (uint16_t)(((lo >> 6) & 7u) | (((hi >> 6) & 7u) << 3));
        const uint16_t gl = px22Map[(w | (g & sel) | (d & selDark)) & 0x3Fu];
        code[x] = (uint8_t)gl;
        inv |= (uint32_t)(gl >> 8) << x;
    }
    bmInv[buf][line] = inv;
    for (uint8_t l = 1; l < OSD_DEMO_PLASMA2X2_STEP; l++) {
        memcpy(bmCode[buf][line + l], code, cols);
        bmInv[buf][line + l] = inv;
    }
}

static void osdDemoP7BuildFrame(uint8_t buf, uint8_t cols, uint8_t rows, uint32_t t)
{
    osdDemoPlasma2x2BuildLuts(cols, rows, (uint8_t)t);
    uint16_t cells[OSD_DEMO_CHECKER_COLS];
    const uint8_t plasmaRows = (rows > 1u) ? (uint8_t)(rows - 1u) : rows;
    for (uint8_t y = 0; y < plasmaRows; y++) {
        for (uint8_t b = 0; b < OSD_DEMO_PLASMA2X2_BANDS; b++) {
            osdDemoPlasma2x2BuildMasks(y, b, cols, cells);
            osdDemoP7EmitBand(buf, y, b, cols, cells);
        }
    }
    if (plasmaRows < rows) {
        for (uint8_t b = 0; b < OSD_DEMO_PLASMA2X2_BANDS; b++) {
            osdDemoPlasma7TextBand(b, cols, cells);
            osdDemoP7EmitBand(buf, plasmaRows, b, cols, cells);
        }
    }
}

static void osdDemoP7Field(uint8_t cols, uint8_t rows, bool frameStart, osdDemoRasterMotion_t *next)
{
    UNUSED(cols);
    const uint8_t kick = chiptuneGetKickPulse(NULL);
    if (kick != p7KickSeen) {
        p7KickSeen = kick;
        p7BeatMs = millis(); // the copper flashes
    }
    const uint8_t snare = chiptuneGetSnarePulse();
    if (snare != p7SnareSeen) {
        p7SnareSeen = snare;
        p7SnareMs = millis(); // the chain pushed apart
    }
    osdDemoTriFrame(frameStart);
    memset(mgSram, 0xFF, sizeof(mgSram)); // the VBLANK preload writes every first line
    // The copper: RB a line from the bars' level a mega-pixel row (0 none, 1–2 the edge, 3–4
    // the core); the scroller's row at full white. White 80 / 120 %, black 0 / one lifted level.
    const uint8_t core = (uint8_t)((((timeMs_t)(millis() - p7BeatMs) < OSD_DEMO_P7_COPPER_FLASH_MS) ? 3u : 2u) << 2);
    const uint8_t rbOf[5] = { 0x03u, 0x00u, 0x00u, core, core };
    const uint8_t n = MIN(rows, (uint8_t)OSD_DEMO_CHECKER_ROWS);
    const uint8_t plasmaRows = (n > 1u) ? (uint8_t)(n - 1u) : n;
    uint8_t lvl[OSD_DEMO_CHECKER_ROWS * OSD_DEMO_PLASMA2X2_BANDS];
    osdDemoP7Copper(lvl, (uint16_t)plasmaRows * OSD_DEMO_PLASMA2X2_BANDS);
    for (uint16_t l = 0; l < (uint16_t)n * OSD_DEMO_CELL_H; l++) {
        const uint16_t mr = l / OSD_DEMO_PLASMA2X2_STEP;
        p7RbLine[l] = (l / OSD_DEMO_CELL_H < plasmaRows) ? rbOf[lvl[mr]] : (uint8_t)((0u << 2) | (3u - 3u));
    }
    next->rbLine = p7RbLine;
    next->vos = 0;
}

static osdDemoIrqScene_t osdDemoP7Scene = {
    .bandLines = 1,
    .bands = OSD_DEMO_CELL_H,
    .armed = &plasma2x2Armed,
    .field = osdDemoP7Field,
    .band = osdDemoBmBand,
    .task = osdDemoPbTask,
};

static void osdDemoP7Enter(void)
{
    osdDemoApplyOsdmNow();
    max7456Brightness(0, 3);
    osdDemoBgType(DISPLAY_BACKGROUND_BLACK);
    max7456SetHudMotionOffset(0, 0);
    if (!demoDisplay) {
        return;
    }
    osdDemoPx22MapInit();
    osdDemoMgBuildLevelLut();
    osdDemoFillRowsGlyphFast(demoDisplay->cols, demoDisplay->rows, OSD_DEMO_PIXEL_OFF, OSD_DEMO_PIXEL_OFF);
    memset(mgSram, 0xFF, sizeof(mgSram));
    mgLastEdgeTicks = 0;
    memset(plasmaRbWritten, 0xFF, sizeof(plasmaRbWritten));
    plasmaRbNext = 0;
    plasmaPlay = 0;
    plasmaNextReady = false;
    pbReady = PB_NONE;
    pbBuildFn = osdDemoP7BuildFrame;
    p7BeatMs = millis() - OSD_DEMO_P7_COPPER_FLASH_MS;
    p7SnareMs = millis() - OSD_DEMO_P7_COPPER_PUSH_MS;
    p7KickSeen = chiptuneGetKickPulse(NULL);
    p7SnareSeen = chiptuneGetSnarePulse();
    pbFrameNo = 0;
    pbBuiltT = 0;
    osdDemoP7BuildFrame(0, MIN(demoDisplay->cols, OSD_DEMO_CHARS_PER_LINE), MIN(demoDisplay->rows, OSD_DEMO_MG_ROWS_MAX), pbBuiltT);
    if (!osdDemoIrqStart(&osdDemoP7Scene)) {
        return;
    }
    osdDemoPbBuildIrqOn(true);
}


// --- Scene 10 on the engine ---

static const osdDemoIrqScene_t osdDemoShoutScene = {
    .bandLines = OSD_DEMO_SHOUT_BAND_LINES,
    .bands = OSD_DEMO_SHOUT_BANDS,
    .armed = &shoutArmed,
    .field = osdDemoShoutField,
    .band = osdDemoShoutBand,
    .task = NULL,
};

// Per-row brightness from a field hook in VBLANK: every row on the first field, then one
// row per field (~2 us) so the levels survive anything that resets RB0..RB15.
static void osdDemoRbRefresh(const uint8_t *white, uint8_t rows, bool *all, uint8_t *next)
{
    if (*all) {
        *all = false;
        for (uint8_t y = 0; y < rows; y++) {
            max7456BrightnessRow(y, 0, white[y]);
        }
    } else if (rows) {
        if (*next >= rows) {
            *next = 0;
        }
        max7456BrightnessRow(*next, 0, white[*next]);
        (*next)++;
    }
}

// --- Scene 5: Craft tunnel on the engine — kept at its 6×6-px resolution (60×48) ------------
// The demo raises resolution scene by scene (6×6 → 2×2 → 1-px edges → 3×3 text), so the
// tunnel keeps its 6×6 pixels — but now as a mid-glyph raster: 2 columns × 3 bands of 6 lines
// per cell. Each 6×6 pixel is black, 50 % dither or white, so 9 glyphs = 3 levels × 2 columns
// (instead of 64 2×3 pattern glyphs at 0x00..0x3F). Dither = 1-px checker inside the pixel
// (010101/101010…); the scroller background and the tunnel exit are grey.
// Same ray model and numbers as the character version. The OSD task renders a frame into a
// double buffer, the interrupt engine shows it; the 72 Hz animation rates are kept by clock.
#define OSD_DEMO_TUNNEL6_LP_PER_CELL 2
#define OSD_DEMO_TUNNEL6_BAND_LINES  (OSD_DEMO_CELL_H / OSD_DEMO_TUNNEL6_BANDS)   // 6
#define OSD_DEMO_TUNNEL6_LP_COLS     (OSD_DEMO_CHARS_PER_LINE * OSD_DEMO_TUNNEL6_LP_PER_CELL) // 60
#define OSD_DEMO_TUNNEL6_LVL_DITHER  1
#define OSD_DEMO_TUNNEL6_LVL_WHITE   2
#define OSD_DEMO_TUNNEL6_CENTER_UP   2    // tunnel centre raised (6×6 px) onto the scroller
#define OSD_DEMO_TUNNEL6_MAX_STEPS   16   // animation catch-up limit per frame

static bool tunnelArmed;
// Glyph offsets (left*3 + right) per cell and band — the band hook just adds BASE.
// tunnelFb[buf][band row][cell]: in the scene arena.
static volatile uint8_t tunnelPlay;
static volatile bool tunnelNextReady;
static uint32_t tunnelStepsDone; // old 72 Hz paint steps applied so far
// Row brightness (RB white) wanted per char row. Written from the field hook in VBLANK like
// scene 10 — the one-shot writes at scene entry did not stick (RB read back 0x01).
static uint8_t tunnelRowWhite[OSD_DEMO_MG_ROWS_MAX];
static uint8_t tunnelRbWritten[OSD_DEMO_MG_ROWS_MAX];
static uint8_t tunnelRbNext;  // round-robin row refreshed each field
static uint8_t tunnelTextY0;  // scroller rows tunnelTextY0 .. +TEXT_ROWS-1, shadow right below
// Tunnel rows sit at RB white 0 and jump to 3 (the scroller's level) for the first part of
// every beat, then straight back — a hard flash; the shadow row is one step under them.
#define OSD_DEMO_TUNNEL_BEAT_MS   500
#define OSD_DEMO_TUNNEL_FLASH_TOP 3
#define OSD_DEMO_TUNNEL_FLASH_Q8  64    // flash length, share of the beat (/256): ~120 ms

static void osdDemoTunnelStep(void)
{
    // Fly (V), spin (U) and text scroll at different rates — lockstep freezes the wedges.
    if (++tunnelSpinDiv >= OSD_DEMO_TUNNEL_SPIN_DIV) {
        tunnelSpinDiv = 0;
        tunnelRot++;
    }
    if (++tunnelAnimDiv >= OSD_DEMO_TUNNEL_ANIM_DIV) {
        tunnelAnimDiv = 0;
        tunnelAnim++;
    }
    if (++tunnelTextDiv >= OSD_DEMO_TUNNEL_TEXT_DIV) {
        tunnelTextDiv = 0;
        tunnelScrollMx++;
        if (tunnelTextCols && tunnelScrollMx >= tunnelTextCols) {
            tunnelScrollMx = 0;
        }
    }
}

static void osdDemoTunnelBuildFrame(uint8_t buf, uint8_t cols, uint8_t rows)
{
    const timeMs_t elapsed = (timeMs_t)(millis() - fxStartMs);
    const uint32_t due = (uint32_t)(((uint64_t)elapsed * OSD_DEMO_SCROLL_HZ) / 1000u);
    for (uint8_t n = 0; tunnelStepsDone < due && n < OSD_DEMO_TUNNEL6_MAX_STEPS; n++) {
        osdDemoTunnelStep();
        tunnelStepsDone++;
    }
    if (due > tunnelStepsDone + OSD_DEMO_TUNNEL6_MAX_STEPS) {
        tunnelStepsDone = due - OSD_DEMO_TUNNEL6_MAX_STEPS; // after a stall: no fast-forward burst
    }

    const uint8_t a = osdDemoTunnelSceneAngle(elapsed);
    const int16_t sn = osdDemoSineAt(a);
    const int16_t cs = osdDemoSineAt((uint8_t)(a + 64));
    const uint8_t spin = tunnelRot;
    const uint8_t anim = tunnelAnim;

    const uint16_t w = (uint16_t)cols * OSD_DEMO_TUNNEL6_LP_PER_CELL;
    const uint16_t h = (uint16_t)rows * OSD_DEMO_TUNNEL6_BANDS;
    const int16_t midX = (int16_t)(w / 2u);
    const int16_t midY = (int16_t)(h / 2u) - OSD_DEMO_TUNNEL6_CENTER_UP;
    const int16_t den = (midX > 0) ? midX : 1;
    const int16_t z = OSD_DEMO_TUNNEL_Z;

    // Text band: centre 3 character rows (9 pixel rows); the 5 font rows sit in rows 2..6 of it
    // exactly as the character version stacked them.
    const uint8_t textY0 = (rows >= OSD_DEMO_TUNNEL_TEXT_ROWS) ? (uint8_t)((rows - OSD_DEMO_TUNNEL_TEXT_ROWS) / 2u) : 0u;
    const uint16_t bandTop = (uint16_t)textY0 * OSD_DEMO_TUNNEL6_BANDS;
    const uint16_t bandBot = (uint16_t)(bandTop + OSD_DEMO_TUNNEL_TEXT_ROWS * OSD_DEMO_TUNNEL6_BANDS);
    const uint16_t inkTop = (uint16_t)(bandTop + 2u);

    int16_t xv[OSD_DEMO_TUNNEL6_LP_COLS];
    int16_t zv[OSD_DEMO_TUNNEL6_LP_COLS];
    uint16_t xv2[OSD_DEMO_TUNNEL6_LP_COLS];
    for (uint16_t x = 0; x < w; x++) {
        const int16_t xs = (int16_t)(((int32_t)((int16_t)x - midX) * 32) / den);
        xv[x] = (int16_t)((xs * cs - z * sn) >> 7);
        zv[x] = (int16_t)((xs * sn + z * cs) >> 7);
        xv2[x] = (uint16_t)((int32_t)xv[x] * xv[x]);
    }

    // Pixel x adds lvl×3 (left) or lvl (right) to its cell's glyph offset.
#define TUN6_PUT(x, lvl) (line[(x) >> 1] = (uint8_t)(line[(x) >> 1] + (((x) & 1u) ? (lvl) : (lvl) * 3u)))
    memset(tunnelFb[buf], 0, sizeof(tunnelFb[buf]));
    for (uint16_t ly = 0; ly < h; ly++) {
        uint8_t *line = tunnelFb[buf][ly];
        if (ly >= bandTop && ly < bandBot) {
            // Scroller: white ink on a grey (dither) band.
            const bool ink = ly >= inkTop && ly < inkTop + OSD_DEMO_FONT_W;
            const uint8_t fontRow = (uint8_t)(OSD_DEMO_TUNNEL_INK_Y0 + (ly - inkTop));
            for (uint16_t x = 0; x < w; x++) {
                const bool on = ink && osdDemoTunnelTextOn((uint16_t)(tunnelScrollMx + x), fontRow);
                TUN6_PUT(x, on ? OSD_DEMO_TUNNEL6_LVL_WHITE : OSD_DEMO_TUNNEL6_LVL_DITHER);
            }
            continue;
        }
        const int16_t y = (int16_t)(((int32_t)((int16_t)ly - midY) * 32) / den);
        const uint32_t y2 = (uint32_t)((int32_t)y * y);
        for (uint16_t x = 0; x < w; x++) {
            const uint32_t r2 = xv2[x] + y2;
            if (r2 < OSD_DEMO_TUNNEL_HOLE2) {
                TUN6_PUT(x, OSD_DEMO_TUNNEL6_LVL_DITHER); // grey tunnel exit
                continue;
            }
            const uint16_t dist = (uint16_t)sqrtf((float)r2);
            uint16_t invr = (dist > 0u) ? (uint16_t)(512u / dist) : 255u;
            if (invr > 255u) {
                invr = 255u;
            }
            const int8_t dtex = (int8_t)(((int32_t)zv[x] * (int32_t)invr) >> 7);
            const uint8_t u = (uint8_t)(osdDemoAtan2u8(y, xv[x]) + spin);
            const uint8_t v = (uint8_t)(((uint8_t)dtex + anim) * OSD_DEMO_TUNNEL_DEP_MUL);
            if ((((u >> OSD_DEMO_TUNNEL_UV_SHIFT) ^ (v >> OSD_DEMO_TUNNEL_UV_SHIFT)) & 1u) != 0u) {
                TUN6_PUT(x, OSD_DEMO_TUNNEL6_LVL_WHITE);
            }
        }
    }
#undef TUN6_PUT
}

static void osdDemoTunnelTask(void)
{
    if (tunnelNextReady || !demoDisplay) {
        return;
    }
    osdDemoTunnelBuildFrame((uint8_t)(tunnelPlay ^ 1u),
                            MIN(demoDisplay->cols, OSD_DEMO_CHARS_PER_LINE),
                            MIN(demoDisplay->rows, OSD_DEMO_MG_ROWS_MAX));
    __DSB();
    tunnelNextReady = true;
}

static void osdDemoTunnelField(uint8_t cols, uint8_t rows, bool frameStart,
                               osdDemoRasterMotion_t *next)
{
    UNUSED(cols);
    if (frameStart && tunnelNextReady) {
        tunnelPlay ^= 1u;
        tunnelNextReady = false;
    }
    const uint8_t n = MIN(rows, OSD_DEMO_MG_ROWS_MAX);
    const uint32_t frac = osdDemoBeatsQ8(OSD_DEMO_TUNNEL_BEAT_MS, fxStartMs) & 0xFFu;
    const uint8_t flash = (frac < OSD_DEMO_TUNNEL_FLASH_Q8) ? OSD_DEMO_TUNNEL_FLASH_TOP : 0u;
    const uint8_t shadowY = (uint8_t)(tunnelTextY0 + OSD_DEMO_TUNNEL_TEXT_ROWS);
    for (uint8_t y = 0; y < n; y++) {
        const bool text = y >= tunnelTextY0 && y < shadowY;
        tunnelRowWhite[y] = text ? 3u : (y == shadowY) ? (uint8_t)(flash ? flash - 1u : 0u) : flash;
        if (tunnelRowWhite[y] != tunnelRbWritten[y]) {
            tunnelRbWritten[y] = tunnelRowWhite[y];
            max7456BrightnessRow(y, 0, tunnelRowWhite[y]);
        }
    }
    if (n) {
        if (tunnelRbNext >= n) {
            tunnelRbNext = 0;
        }
        max7456BrightnessRow(tunnelRbNext, 0, tunnelRowWhite[tunnelRbNext]);
        tunnelRbNext++;
    }
    next->vos = 0;
}

// Glyphs from the persistent 3c set (left·3 + right → glyph | INV): black / clear (the VM1 grey,
// was a 1-px checker that shimmered) / white.
static uint16_t tunnel3c[9];

static void osdDemoTunnelBand(uint8_t y, uint8_t b, uint8_t cols, uint8_t *glyphs)
{
    const uint8_t *line = tunnelFb[tunnelPlay][(uint16_t)y * OSD_DEMO_TUNNEL6_BANDS + b];
    uint32_t inv = 0;
    for (uint8_t x = 0; x < cols; x++) {
        const uint16_t g = tunnel3c[line[x]];
        glyphs[x] = (uint8_t)g;
        inv |= (uint32_t)(g >> 8) << x;
    }
    mgBandInv = inv;
}

static const osdDemoIrqScene_t osdDemoTunnelScene = {
    .bandLines = OSD_DEMO_TUNNEL6_BAND_LINES,
    .bands = OSD_DEMO_TUNNEL6_BANDS,
    .armed = &tunnelArmed,
    .field = osdDemoTunnelField,
    .band = osdDemoTunnelBand,
    .task = osdDemoTunnelTask,
};

static void osdDemoTunnelEnter(void)
{
    for (uint8_t i = 0; i < 9u; i++) {
        tunnel3c[i] = osdDemo3cHalves((uint8_t)(i / 3u), (uint8_t)(i % 3u)); // levels = 3c digits
    }
    osdDemoBgType(DISPLAY_BACKGROUND_LTGRAY); // clear pixels: the VM1 grey
    if (demoDisplay) {
        osdDemoFillRowsGlyphFast(demoDisplay->cols, demoDisplay->rows, OSD_DEMO_PIXEL_OFF, OSD_DEMO_PIXEL_OFF);
        osdDemoBg(true, 7);
        // Scroller rows full bright; tunnel rows and the shadow by the beat (field hook).
        const uint8_t rows = MIN(demoDisplay->rows, OSD_DEMO_MG_ROWS_MAX);
        tunnelTextY0 = (rows >= OSD_DEMO_TUNNEL_TEXT_ROWS) ? (uint8_t)((rows - OSD_DEMO_TUNNEL_TEXT_ROWS) / 2u) : 0u;
        memset(tunnelRbWritten, 0xFF, sizeof(tunnelRbWritten));
        tunnelRbNext = 0;
    }
    memset(mgSram, 0xFF, sizeof(mgSram));
    mgLastEdgeTicks = 0;
    tunnelStepsDone = 0;
    tunnelPlay = 0;
    tunnelNextReady = false;
    if (demoDisplay) {
        osdDemoTunnelBuildFrame(0, MIN(demoDisplay->cols, OSD_DEMO_CHARS_PER_LINE),
                                MIN(demoDisplay->rows, OSD_DEMO_MG_ROWS_MAX));
    }
    (void)osdDemoIrqStart(&osdDemoTunnelScene);
}



// --- Scene 4: horizontal twister on the engine — 6-px columns, 2-line bands ----------------
// A mid-glyph raster: 60 × 144 logical pixels of 6 px × 2 lines, each black, clear (the VM1
// grey) or white, from 9 glyphs (3 levels × 2 half-cells). Geometry once a frame in the field
// hook; a band is a 60-column classification.
// The square bar, 3/4 of a turn across the screen, shows at most two faces a column, each in its
// own shade (white, grey / white checker, black / grey checker, black — all of them apart from
// the grey background), a dark seam where two meet: the colours run along the twist. (Lit per
// column by its height the shades made vertical stripes across the twist.) Two opposite faces
// carry a scroller (5×5 font, kerned) printed on them: along the face's centre line (a dot
// 6 px of its length; a pixel inks if any font column its column spans along does — on a slope
// a column spans more than one, none dropped), the letters upright across it (5 of 9 rows of the face), so they lean
// and turn with the ribbon — dark on a light face, light on a dark one. The music drives it: the bass spins it faster, the kick
// swells it, the snare bends it (each time the other way). The background: the clear pixels,
// grey, flashing on the kick (28 % → 49 % and back), stars behind the bar on the inside of a
// turning cylinder, the camera on its axis looking at the wall: they come up from under the
// bottom, slow straight ahead and fast at the edges, spread out to the sides above and below
// (the wall nears there) — three shells (radii) turning at their own pace, twinkling; at every
// new section of the track the whole turns a quarter (fast ease); the rows lit by the bar (row
// levels): brightest at its middle, four steps darker towards its edges (not to the darkest),
// between its top and bottom as they are now (the swell, the bend, the slope). (A dark reflection under a water line
// looked bad.)
#define OSD_DEMO_WIPE_LP_PER_CELL  2
#define OSD_DEMO_WIPE_BANDS        9
#define OSD_DEMO_WIPE_BAND_LINES   (OSD_DEMO_CELL_H / OSD_DEMO_WIPE_BANDS)                 // 2
#define OSD_DEMO_WIPE_COLS         (OSD_DEMO_CHARS_PER_LINE * OSD_DEMO_WIPE_LP_PER_CELL)    // 60
#define OSD_DEMO_WIPE_LVL_T        0
#define OSD_DEMO_WIPE_LVL_B        1
#define OSD_DEMO_WIPE_LVL_W        2
#define OSD_DEMO_WIPE_BEAT_MS      500  // no music: a beat clock instead
#define OSD_DEMO_WIPE_PHASE_HZ     24   // twist phase steps/s at rest
#define OSD_DEMO_WIPE_BASS_Q8      640  // extra twist a frame at full bass, steps << 8
#define OSD_DEMO_WIPE_CENTRE_Q8    128  // bar axis at this share of the height (/256)
#define OSD_DEMO_WIPE_BG_LEVEL     4    // the grey at rest (28 %)
#define OSD_DEMO_WIPE_FLASH_HI     7    // at the kick (49 %), back down over KICK_FIELDS
#define OSD_DEMO_WIPE_STARS        60   // 20 a layer
#define OSD_DEMO_WIPE_STAR_F       48   // stars: the lens, line pairs off the centre = F · tan(angle round the axis)
#define OSD_DEMO_WIPE_STAR_PHI     46   // shown from −PHI to +PHI round the axis (sine-table steps; 40 = the edge)
#define OSD_DEMO_WIPE_STAR_AX      72   // along the axis ± this (line pairs, the nearest wall straight ahead)
#define OSD_DEMO_WIPE_STAR_ASPECT  24   // line pairs a logical px is as wide as (×10): the screen 72 × 72
#define OSD_DEMO_WIPE_TURN_MS      300  // the stream's quarter turn at a new section
#define OSD_DEMO_WIPE_KICK_FIELDS  8    // swell / flash after a kick, fields
#define OSD_DEMO_WIPE_TEXT_COLS_S  11   // scroller: font columns a second
#define OSD_DEMO_WIPE_TEXT_DOT     6    // scroller: px of the face's length a font column (a logical px: none skipped flat)
#define OSD_DEMO_WIPE_TEXT_ROWS    9    // scroller: the face across in rows (2 margin, 5 font, 2 margin)

typedef struct osdDemoWipeCol_s {
    int16_t y0[2], y1[2]; // visible faces, screen lines [y0, y1)
    uint8_t shade[2];     // 0 black .. 4 white
    uint8_t text[2];      // the face carries the scroller
    uint8_t n;            // faces shown (0..2)
    uint8_t txt;          // the face (0..1) that carries the scroller (0xFF: none)
    int16_t txtC;         // its centre line here (screen line)
    int16_t txtK;         // font columns × 256 a line down (the letters upright across the face)
    int16_t txtHalf;      // font columns × 256 the column spans along beyond one, halved
    int32_t txtAlong;     // font column × 256 on the centre line here (0 .. text width × 256)
} osdDemoWipeCol_t;

static osdDemoWipeCol_t wipeCol[OSD_DEMO_WIPE_COLS];
static bool wipeArmed;
static uint64_t wipeStarRow[OSD_DEMO_MG_ROWS_MAX * OSD_DEMO_WIPE_BANDS]; // stars a line pair (bit = column)
typedef struct { int16_t phiQ4; int8_t ax; uint8_t layer, tw; } osdDemoWipeStar_t; // round the axis (steps << 4), along it
static osdDemoWipeStar_t wipeStar[OSD_DEMO_WIPE_STARS];
static uint8_t wipeBgLevel;           // VM1 grey written last
static uint8_t wipeDirFrom = 64, wipeDirTo = 64; // the stream's direction (sine-table steps; 64 = up)
static timeMs_t wipeTurnMs;
static int16_t wipeOrder = -1;        // track section seen last
static uint8_t wipeRbReg[OSD_DEMO_MG_ROWS_MAX];
static uint8_t wipeRbNext;
static uint8_t wipeKickT;            // fields since the kick (KICK_FIELDS = none)
static int16_t wipeEnvTop, wipeEnvBot; // the bar's top and bottom now (screen lines, all columns)
static uint16_t wipeBendQ8;          // the snare's bend, decaying
static int8_t wipeBendDir = 1;
static uint32_t wipePhaseQ8;         // twist phase, steps << 8
static uint8_t wipeKickSeen, wipeSnareSeen;
static timeMs_t wipeLastBeatMs;
#define OSD_DEMO_WIPE_TEXT_MAX     (sizeof(osdDemoWipeScrollText) * 6u)
static uint8_t wipeTextCode[OSD_DEMO_WIPE_TEXT_MAX]; // the text by font column: character (0xFF: gap)
static uint8_t wipeTextFcol[OSD_DEMO_WIPE_TEXT_MAX]; // and its font column
static uint16_t wipeTextW;           // font columns

static void osdDemoWipeTextLayout(void)
{
    uint16_t x = 0;
    for (uint16_t i = 0; osdDemoWipeScrollText[i]; i++) {
        uint8_t c0, w;
        osdDemoS1CharInk(osdDemoWipeScrollText[i], &c0, &w);
        for (uint8_t k = 0; k <= w && x < OSD_DEMO_WIPE_TEXT_MAX; k++, x++) {
            const bool ink = k < w && osdDemoWipeScrollText[i] != ' ';
            wipeTextCode[x] = ink ? (uint8_t)osdDemoWipeScrollText[i] : 0xFFu;
            wipeTextFcol[x] = (uint8_t)(c0 + k);
        }
    }
    wipeTextW = x ? x : 1u;
}

// The bass now: the loudest low AY tone (≥ 1000 period) 0..15
static uint8_t osdDemoWipeBass(void)
{
    uint8_t bass = 0;
    uint8_t regs[16];
    if (chiptuneGetAyRegs(regs)) {
        for (uint8_t k = 0; k < 3u; k++) {
            const uint16_t period = (uint16_t)(regs[2u * k] | ((regs[2u * k + 1u] & 0x0Fu) << 8));
            if (((regs[7] >> k) & 1u) == 0u && period >= 1000u) {
                bass = MAX(bass, (uint8_t)((regs[8u + k] & 0x10u) ? 15u : (regs[8u + k] & 0x0Fu)));
            }
        }
    }
    return bass;
}

static void osdDemoWipeGeometry(uint8_t cols, uint8_t rows)
{
    const uint16_t w = (uint16_t)cols * OSD_DEMO_WIPE_LP_PER_CELL;
    const int16_t screenH = (int16_t)((uint16_t)rows * OSD_DEMO_CELL_H);
    const int16_t midY = (int16_t)(screenH * OSD_DEMO_WIPE_CENTRE_Q8 / 256);
    const int16_t den = (w > 1u) ? (int16_t)(w - 1u) : 1;
    const uint8_t phase = (uint8_t)(wipePhaseQ8 >> 8);

    const int8_t slS = osdDemoSineAt(phase);
    const int16_t slope = (int16_t)((screenH / 40) + (((int16_t)(screenH / 64) * (int16_t)slS) / 128));
    const int8_t cvS = osdDemoSineAt((uint8_t)(phase + 64));
    int16_t amp = (int16_t)((screenH / 3) + (((int16_t)(screenH / 16) * (int16_t)cvS) / 128));
    // the kick swells it (a quarter more, back over KICK_FIELDS)
    if (wipeKickT < OSD_DEMO_WIPE_KICK_FIELDS) {
        amp = (int16_t)(amp + amp * (OSD_DEMO_WIPE_KICK_FIELDS - wipeKickT) / (4 * OSD_DEMO_WIPE_KICK_FIELDS));
    }
    const int16_t bend = (int16_t)((int32_t)(screenH / 9) * wipeBendQ8 / 256 * wipeBendDir);

    // the text faces' (0, 2) centre lines, for the scroller printed along them
    static int16_t txtCentre[2][OSD_DEMO_WIPE_COLS];
    static uint8_t txtFace[OSD_DEMO_WIPE_COLS]; // which of them a column shows
    int16_t envTop = screenH, envBot = 0;

    for (uint16_t x = 0; x < w; x++) {
        osdDemoWipeCol_t *col = &wipeCol[x];
        const int16_t t = (int16_t)(((int32_t)x * 2 - den) * slope / den);
        const int16_t arc = (int16_t)((int32_t)bend * osdDemoSineAt((uint8_t)(x * 128u / (uint16_t)den)) / 127);
        const int16_t axis = (int16_t)(midY + t + arc);
        const uint8_t ang = (uint8_t)((uint16_t)((uint32_t)x * 192u / (uint16_t)den) + phase); // 3/4 turn
        int16_t e[4];
        for (uint8_t k = 0; k < 4u; k++) {
            e[k] = (int16_t)(axis + ((amp * (int16_t)osdDemoSineAt((uint8_t)(ang + k * 64u))) / 128));
        }
        envTop = MIN(envTop, MIN(MIN(e[0], e[1]), MIN(e[2], e[3])));
        envBot = MAX(envBot, MAX(MAX(e[0], e[1]), MAX(e[2], e[3])));
        col->n = 0;
        col->txt = 0xFF;
        for (uint8_t i = 0; i < 4u && col->n < 2u; i++) {
            const int16_t y0 = e[i], y1 = e[(i + 1u) & 3u];
            if (y1 > y0) {
                const uint8_t f = col->n++;
                col->y0[f] = y0;
                col->y1[f] = y1;
                static const uint8_t faceShade[4] = { 4, 3, 1, 0 }; // each face its own, none the grey
                col->shade[f] = faceShade[i];
                col->text[f] = (uint8_t)((i & 1u) == 0u && (y1 - y0) >= 18);
                if ((i & 1u) == 0u) {
                    col->txt = f;
                    txtFace[x] = (uint8_t)(i >> 1);
                }
            }
        }
        txtCentre[0][x] = (int16_t)((e[0] + e[1]) / 2);
        txtCentre[1][x] = (int16_t)((e[2] + e[3]) / 2);
    }
    wipeEnvTop = envTop;
    wipeEnvBot = envBot;
    // Along each text face's centre line: its length so far (font columns × 256) from the left,
    // plus the scroll (leftwards); how far along a line down is (the letters upright across it)
    const int32_t wQ8 = (int32_t)wipeTextW * 256;
    const int32_t tposQ8 = (int32_t)((uint32_t)(millis() - fxStartMs) * (OSD_DEMO_WIPE_TEXT_COLS_S * 256u) / 1000u % (uint32_t)wQ8);
    int32_t along[2] = { tposQ8, tposQ8 };
    for (uint16_t x = 0; x < w; x++) {
        osdDemoWipeCol_t *col = &wipeCol[x];
        const uint8_t j = (col->txt != 0xFFu) ? txtFace[x] : 2u;
        for (uint8_t k = 0; k < 2u; k++) {
            const uint16_t xn = (x + 1u < w) ? (uint16_t)(x + 1u) : x, xp = (x + 1u < w) ? x : (uint16_t)(x - 1u);
            const int32_t sl = txtCentre[k][xn] - txtCentre[k][xp];          // lines a column (6 px)
            const int32_t a = sl < 0 ? -sl : sl;
            const int32_t len = MAX(a, 6) + MIN(a, 6) * 3 / 8;                 // px a column (≈ hypot)
            if (k == j) {
                col->txtC = txtCentre[k][x];
                col->txtK = (int16_t)(sl * 256 / (len * OSD_DEMO_WIPE_TEXT_DOT));
                col->txtAlong = along[k];
                col->txtHalf = (int16_t)MAX(len * 256 / OSD_DEMO_WIPE_TEXT_DOT - 256, 0) / 2;
            }
            along[k] += len * 256 / OSD_DEMO_WIPE_TEXT_DOT;
            if (along[k] >= wQ8) {
                along[k] -= wQ8;
            }
        }
    }
}

// Glyphs from the persistent 3c set (left·3 + right in wipe levels → glyph | INV).
static uint16_t wipe3c[9];

// Shade 0..4 → level at logical pixel (lc, line pair lp): the in-betweens a checker
static inline uint8_t osdDemoWipeShadePx(uint8_t shade, uint8_t lc, int16_t lp)
{
    static const uint8_t lo[5] = { OSD_DEMO_WIPE_LVL_B, OSD_DEMO_WIPE_LVL_B, OSD_DEMO_WIPE_LVL_T,
                                   OSD_DEMO_WIPE_LVL_T, OSD_DEMO_WIPE_LVL_W };
    static const uint8_t hi[5] = { OSD_DEMO_WIPE_LVL_B, OSD_DEMO_WIPE_LVL_T, OSD_DEMO_WIPE_LVL_T,
                                   OSD_DEMO_WIPE_LVL_W, OSD_DEMO_WIPE_LVL_W };
    return (((lc + (uint16_t)lp) & 1u) != 0u) ? hi[shade] : lo[shade];
}

static void osdDemoWipeBand(uint8_t y, uint8_t b, uint8_t cols, uint8_t *glyphs)
{
    uint32_t inv = 0;
    const int16_t ly = (int16_t)((uint16_t)y * OSD_DEMO_CELL_H + b * OSD_DEMO_WIPE_BAND_LINES + 1u);
    const int16_t lp = (int16_t)(ly / OSD_DEMO_WIPE_BAND_LINES);
    const uint64_t stars = (lp < (int16_t)ARRAYLEN(wipeStarRow)) ? wipeStarRow[lp] : 0u;
    const osdDemoWipeCol_t *col = wipeCol;
    for (uint8_t x = 0; x < cols; x++) {
        uint8_t g = 0;
        for (uint8_t h = 0; h < 2u; h++, col++) {
            const uint8_t lc = (uint8_t)(x * OSD_DEMO_WIPE_LP_PER_CELL + h);
            // the background: grey (clear), a star white
            uint8_t v = ((stars >> lc) & 1u) ? OSD_DEMO_WIPE_LVL_W : OSD_DEMO_WIPE_LVL_T;
            {
                for (uint8_t f = 0; f < col->n; f++) {
                    if (ly < col->y0[f] || ly >= col->y1[f]) {
                        continue;
                    }
                    uint8_t shade = col->shade[f];
                    if (f == 1u && ly < col->y0[f] + OSD_DEMO_WIPE_BAND_LINES) {
                        v = OSD_DEMO_WIPE_LVL_B; // the seam between the two faces
                        break;
                    }
                    if (col->text[f]) {
                        // across the face: 2 margin rows, the 5 font rows, 2 margin rows; along it
                        // from the centre line, leaning with it
                        const int16_t fr = (int16_t)((ly - col->y0[f]) * OSD_DEMO_WIPE_TEXT_ROWS / (col->y1[f] - col->y0[f])) - 2;
                        if (fr >= 0 && fr < 5) {
                            const int32_t a = col->txtAlong + (int32_t)(ly - col->txtC) * col->txtK;
                            const int32_t tcEnd = (a + col->txtHalf + 128) >> 8;
                            for (int32_t t = (a - col->txtHalf + 128) >> 8; t <= tcEnd; t++) {
                                const int32_t tc = (t < 0) ? t + wipeTextW : ((t >= (int32_t)wipeTextW) ? t - wipeTextW : t);
                                if (tc >= 0 && tc < (int32_t)wipeTextW && wipeTextCode[tc] != 0xFFu
                                    && osdDemoFontPixel(wipeTextCode[tc], wipeTextFcol[tc], (uint8_t)(OSD_DEMO_TUNNEL_INK_Y0 + fr))) {
                                    shade = (shade >= 2u) ? 0u : 4u;
                                    break;
                                }
                            }
                        }
                    }
                    v = osdDemoWipeShadePx(shade, lc, lp);
                    break;
                }
            }
            g = (uint8_t)(g * 3u + v);
        }
        const uint16_t w3 = wipe3c[g];
        glyphs[x] = (uint8_t)w3;
        inv |= (uint32_t)(w3 >> 8) << x;
    }
    mgBandInv = inv;
}

static void osdDemoWipeField(uint8_t cols, uint8_t rows, bool frameStart,
                             osdDemoRasterMotion_t *next)
{
    // the drums
    bool kick = false, snare = false;
    if (chiptuneIsPlaying()) {
        const uint8_t k = osdDemoDrumPulse();
        kick = k != wipeKickSeen;
        wipeKickSeen = k;
        const uint8_t sn = chiptuneGetSnarePulse();
        snare = sn != wipeSnareSeen;
        wipeSnareSeen = sn;
    } else
    {
        const timeMs_t now = millis();
        if ((timeDelta_t)(now - wipeLastBeatMs) >= OSD_DEMO_WIPE_BEAT_MS) {
            wipeLastBeatMs = now;
            kick = true;
        }
    }
    if (kick) {
        wipeKickT = 0;
    }
    if (snare) {
        wipeBendQ8 = 256;
        wipeBendDir = (int8_t)-wipeBendDir;
    }
    if (frameStart) {
        // the bass spins it faster
        wipePhaseQ8 += (uint32_t)(OSD_DEMO_WIPE_PHASE_HZ * 256u / 25u)
            + (uint32_t)osdDemoWipeBass() * OSD_DEMO_WIPE_BASS_Q8 / 15u;
        osdDemoWipeGeometry(cols, rows); // both fields of a frame show the same bar
        wipeBendQ8 = (uint16_t)(wipeBendQ8 * 220u / 256u);
        // the stars: up the wall, the flow a quarter turn at every new section (ease-out: fast,
        // then settling)
        chiptuneClock_t ck;
        if (chiptuneGetClock(&ck) && (int16_t)ck.order != wipeOrder) {
            if (wipeOrder >= 0) {
                wipeDirFrom = wipeDirTo;
                wipeDirTo = (uint8_t)(wipeDirTo - 64u); // up → right → down → left
                wipeTurnMs = millis();
            }
            wipeOrder = (int16_t)ck.order;
        }
        uint32_t tq = (uint32_t)(millis() - wipeTurnMs) * 256u / OSD_DEMO_WIPE_TURN_MS;
        tq = MIN(tq, 256u);
        const uint32_t e = 256u - (256u - tq) * (256u - tq) * (256u - tq) / 65536u; // ease-out (cubic)
        const uint8_t dir = (uint8_t)(wipeDirFrom + (int8_t)(wipeDirTo - wipeDirFrom) * (int32_t)e / 256);
        memset(wipeStarRow, 0, sizeof(wipeStarRow));
        const int32_t hLp = (int32_t)MIN(rows, (uint8_t)OSD_DEMO_MG_ROWS_MAX) * OSD_DEMO_WIPE_BANDS;
        const int32_t w = (int32_t)MIN(cols, (uint8_t)OSD_DEMO_CHARS_PER_LINE) * OSD_DEMO_WIPE_LP_PER_CELL;
        const int32_t cd = osdDemoSineAt((uint8_t)(dir + 64u)), sd = osdDemoSineAt(dir); // the flow
        for (uint8_t i = 0; i < OSD_DEMO_WIPE_STARS; i++) {
            osdDemoWipeStar_t *st = &wipeStar[i];
            static const uint8_t spinQ4[3] = { 14, 20, 28 }; // round the axis a frame (steps << 4)
            static const uint8_t shellQ4[3] = { 9, 12, 16 }; // 16 / the shell's radius: far, mid, near
            st->phiQ4 = (int16_t)(st->phiQ4 + spinQ4[st->layer]);
            if (st->phiQ4 > (OSD_DEMO_WIPE_STAR_PHI << 4)) {
                // over the top: round again from under the bottom, somewhere along the axis
                st->phiQ4 = (int16_t)(-(OSD_DEMO_WIPE_STAR_PHI << 4) - (int16_t)(osdDemoRand() % 128u));
                st->ax = (int8_t)((int32_t)(osdDemoRand() % (2u * OSD_DEMO_WIPE_STAR_AX + 1u)) - OSD_DEMO_WIPE_STAR_AX);
                continue;
            }
            // the camera on the axis looking at the wall: up the screen tan of the angle round
            // the axis (slow ahead, fast at the edges), across it the axis over the depth, cos —
            // the wall nears above and below, the stars there spread out to the sides
            const int8_t phi = (int8_t)(st->phiQ4 / 16);
            const int32_t c = MAX(osdDemoSineAt((uint8_t)(phi + 64)), 24);
            const int32_t u = OSD_DEMO_WIPE_STAR_F * osdDemoSineAt((uint8_t)phi) / c;
            const int32_t v = (int32_t)st->ax * shellQ4[st->layer] * 127 / (16 * c);
            const int32_t px = w / 2 + (u * cd - v * sd) * 10 / (127 * OSD_DEMO_WIPE_STAR_ASPECT);
            const int32_t py = hLp / 2 - (u * sd + v * cd) / 127;
            if (px < 0 || px >= w || py < 0 || py >= hLp) {
                continue; // off the screen for now
            }
            st->tw = (uint8_t)(st->tw + 1u + st->layer);
            if ((st->tw & 0x3Fu) < 6u || py >= (int32_t)ARRAYLEN(wipeStarRow)) {
                continue; // twinkle: off a moment
            }
            wipeStarRow[py] |= (uint64_t)1u << px;
        }
    }
    // The rows lit by the bar: its middle brightest, four steps down to its edges (and beyond)
    const uint8_t n = MIN(rows, (uint8_t)OSD_DEMO_MG_ROWS_MAX);
    const int32_t mid = (wipeEnvTop + wipeEnvBot) / 2, half = MAX((wipeEnvBot - wipeEnvTop) / 2, 1);
    // the grey background flashes on the kick
    const uint8_t bg = (wipeKickT < OSD_DEMO_WIPE_KICK_FIELDS)
        ? (uint8_t)(OSD_DEMO_WIPE_FLASH_HI - (OSD_DEMO_WIPE_FLASH_HI - OSD_DEMO_WIPE_BG_LEVEL) * wipeKickT / OSD_DEMO_WIPE_KICK_FIELDS)
        : (uint8_t)OSD_DEMO_WIPE_BG_LEVEL;
    if (bg != wipeBgLevel) {
        wipeBgLevel = bg;
        osdDemoBg(true, bg);
    }
    for (uint8_t y = 0; y < n; y++) {
        const int32_t yc = (int32_t)y * OSD_DEMO_CELL_H + OSD_DEMO_CELL_H / 2;
        // step 0 (the middle) .. 3 (the edges and out): white 120 / 120 / 100 / 90 %, black
        // lifted 20 / 10 / 0 / 0 %
        static const uint8_t whiteOf[4] = { 3, 3, 2, 1 }, blackOf[4] = { 2, 1, 0, 0 };
        const uint8_t st = (uint8_t)MIN(ABS(yc - mid) * 4 / half, 3);
        const uint8_t white = whiteOf[st];
        const uint8_t black = blackOf[st];
        const uint8_t reg = (uint8_t)((black << 2) | (3u - white));
        if (reg != wipeRbReg[y]) {
            wipeRbReg[y] = reg;
            max7456BrightnessRow(y, black, white);
        }
    }
    if (n) { // one row refreshed a field (a write that did not stick heals)
        wipeRbNext = (uint8_t)((wipeRbNext + 1u) % n);
        max7456BrightnessRow(wipeRbNext, (uint8_t)((wipeRbReg[wipeRbNext] >> 2) & 3u), (uint8_t)(3u - (wipeRbReg[wipeRbNext] & 3u)));
    }
    if (wipeKickT < OSD_DEMO_WIPE_KICK_FIELDS) {
        wipeKickT++;
    }
    next->vos = 0;
}

static const osdDemoIrqScene_t osdDemoWipeScene = {
    .bandLines = OSD_DEMO_WIPE_BAND_LINES,
    .bands = OSD_DEMO_WIPE_BANDS,
    .armed = &wipeArmed,
    .field = osdDemoWipeField,
    .band = osdDemoWipeBand,
    .task = NULL,
};

static void osdDemoWipeEnter(void)
{
    static const uint8_t digit[3] = { 1, 0, 2 }; // wipe T / B / W → 3c clear / black / white
    for (uint8_t i = 0; i < 9u; i++) {
        wipe3c[i] = osdDemo3cHalves(digit[i / 3u], digit[i % 3u]);
    }
    if (!demoDisplay) {
        return;
    }
    const uint8_t cols = MIN(demoDisplay->cols, OSD_DEMO_CHARS_PER_LINE);
    const uint8_t rows = MIN(demoDisplay->rows, OSD_DEMO_MG_ROWS_MAX);
    osdDemoFillRowsGlyphFast(demoDisplay->cols, demoDisplay->rows, OSD_DEMO_PIXEL_OFF, OSD_DEMO_PIXEL_OFF);
    osdDemoWipeTextLayout();
    memset(wipeRbReg, 0xFF, sizeof(wipeRbReg));
    wipeRbNext = 0;
    wipeKickT = OSD_DEMO_WIPE_KICK_FIELDS;
    wipeBendQ8 = 0;
    wipePhaseQ8 = 0;
    wipeBgLevel = OSD_DEMO_WIPE_BG_LEVEL; // written by the enter
    wipeDirFrom = wipeDirTo = 64; // up
    wipeTurnMs = millis() - OSD_DEMO_WIPE_TURN_MS;
    wipeOrder = -1;
    memset(wipeStarRow, 0, sizeof(wipeStarRow));
    for (uint8_t i = 0; i < OSD_DEMO_WIPE_STARS; i++) {
        wipeStar[i].phiQ4 = (int16_t)(((int32_t)(osdDemoRand() % (2u * OSD_DEMO_WIPE_STAR_PHI + 1u)) - OSD_DEMO_WIPE_STAR_PHI) << 4);
        wipeStar[i].ax = (int8_t)((int32_t)(osdDemoRand() % (2u * OSD_DEMO_WIPE_STAR_AX + 1u)) - OSD_DEMO_WIPE_STAR_AX);
        wipeStar[i].layer = (uint8_t)(i % 3u);
        wipeStar[i].tw = (uint8_t)osdDemoRand();
    }
    wipeLastBeatMs = millis();
    wipeKickSeen = osdDemoDrumPulse();
    wipeSnareSeen = chiptuneGetSnarePulse();
    memset(mgSram, 0xFF, sizeof(mgSram));
    mgLastEdgeTicks = 0;
    osdDemoWipeGeometry(cols, rows);
    (void)osdDemoIrqStart(&osdDemoWipeScene);
}

// Scene 8 — classic vertical twister (rotating square cross-section).
// 4 projected edges → front-facing intervals; widths from geometry, not cells.
// Paint faces into a per-px color buffer, then encode cells (two runs at 1-px resolution). The
// middle face is real grey (clear pixels on the VM1 grey) — it was a 1-px checker.

// Cell glyphs: solid black / clear (grey) / white, and two-run cells from the 1-px partial set
// (OSD_DEMO_XFILL_*, in the font since the start) with INV for the white variants.
static void osdDemoTwisterSolidGlyph(uint8_t color, uint8_t *glyph, uint8_t *inv)
{
    *inv = 0;
    if (color == OSD_DEMO_TWISTER_C_W) {
        *glyph = OSD_DEMO_TWISTER_GLYPH_W;
    } else if (color == OSD_DEMO_TWISTER_C_D) {
        *glyph = OSD_DEMO_TWISTER_GLYPH_D;
    } else {
        *glyph = OSD_DEMO_TWISTER_GLYPH_B;
    }
}

// Encode a clean 2-run cell [0,t)=c0, [t,12)=c1 (D = grey = clear pixels).
static void osdDemoTwisterEncodeTwoRun(uint8_t c0, uint8_t c1, uint8_t t,
                                       uint8_t *glyph, uint8_t *inv)
{
    *inv = 0;
    if (t == 0) {
        osdDemoTwisterSolidGlyph(c1, glyph, inv);
        return;
    }
    if (t >= OSD_DEMO_CELL_W) {
        osdDemoTwisterSolidGlyph(c0, glyph, inv);
        return;
    }
    const uint8_t idx = (uint8_t)(t - 1u);
    const uint8_t W = OSD_DEMO_TWISTER_C_W, B = OSD_DEMO_TWISTER_C_B, D = OSD_DEMO_TWISTER_C_D;
    if (c0 == W && c1 == B) {
        *glyph = (uint8_t)(OSD_DEMO_XFILL_W_BASE + idx);
    } else if (c0 == B && c1 == W) {
        *glyph = (uint8_t)(OSD_DEMO_XFILL_W_BASE + idx); // white | black, inverted
        *inv = 1;
    } else if (c0 == D && c1 == B) {
        *glyph = (uint8_t)(OSD_DEMO_XFILL_C_BASE + idx);
    } else if (c0 == D && c1 == W) {
        *glyph = (uint8_t)(OSD_DEMO_XFILL_C_BASE + idx); // clear | black, inverted
        *inv = 1;
    } else if (c0 == B && c1 == D) {
        *glyph = (uint8_t)(OSD_DEMO_XFILL_CT_BASE + idx);
    } else if (c0 == W && c1 == D) {
        *glyph = (uint8_t)(OSD_DEMO_XFILL_CT_BASE + idx); // black | clear, inverted
        *inv = 1;
    } else {
        osdDemoTwisterSolidGlyph(c0, glyph, inv);
    }
}

// Classify one 12px cell. Extreme X (thin faces / 3+ runs) must NEVER expand a color
// past its true span — that picks a wider XFILL and flashes a fat ribbon.
static void osdDemoTwisterEncodeCell(const uint8_t pix[12], uint8_t *glyph, uint8_t *inv)
{
    *inv = 0;

    uint8_t runC[4];
    uint8_t runS[5];
    uint8_t n = 1;
    runS[0] = 0;
    runC[0] = pix[0];
    for (uint8_t i = 1; i < OSD_DEMO_CELL_W; i++) {
        if (pix[i] != runC[n - 1u]) {
            if (n >= 4u) {
                break;
            }
            runS[n] = i;
            runC[n] = pix[i];
            n++;
        }
    }
    runS[n] = OSD_DEMO_CELL_W;

    if (n == 1u) {
        osdDemoTwisterSolidGlyph(runC[0], glyph, inv);
        return;
    }
    if (n == 2u) {
        osdDemoTwisterEncodeTwoRun(runC[0], runC[1], runS[1], glyph, inv);
        return;
    }

    const uint8_t leftC = pix[0];
    const uint8_t rightC = pix[OSD_DEMO_CELL_W - 1u];

    // B|C|B islands: snap to nearest cell edge when close (smooth ribbon), else suppress.
    // Never use B|C-to-end — that flashes a full-cell-wide square.
    if (leftC == OSD_DEMO_TWISTER_C_B && rightC == OSD_DEMO_TWISTER_C_B) {
        if (n >= 3u && runC[1] != OSD_DEMO_TWISTER_C_B) {
            const uint8_t a = runS[1];
            const uint8_t b = runS[2];
            const uint8_t midC = runC[1];
            if (a <= 2u) {
                // Near left: C|B ending at b (at most +2px left expand).
                osdDemoTwisterEncodeTwoRun(midC, OSD_DEMO_TWISTER_C_B, b, glyph, inv);
                return;
            }
            if (b >= (uint8_t)(OSD_DEMO_CELL_W - 2u)) {
                // Near right: B|C starting at a (at most +2px right expand).
                osdDemoTwisterEncodeTwoRun(OSD_DEMO_TWISTER_C_B, midC, a, glyph, inv);
                return;
            }
        }
        osdDemoTwisterSolidGlyph(OSD_DEMO_TWISTER_C_B, glyph, inv);
        return;
    }

    // Silhouette edge + face edge in one cell (B|W|D, B|D|W, W|D|B, D|W|B — 0.5% of cells).
    // No 3-run glyphs exist, so keep the SILHOUETTE edge exact and paint the ribbon part with
    // whichever face covers more of it. The old rule kept the face edge and blacked out the
    // middle face — a black notch "cutting" the ribbon at every B/W/D junction.
    if (leftC == OSD_DEMO_TWISTER_C_B || rightC == OSD_DEMO_TWISTER_C_B) {
        uint8_t a = 0;
        uint8_t b = OSD_DEMO_CELL_W;
        if (leftC == OSD_DEMO_TWISTER_C_B) {
            while (a < OSD_DEMO_CELL_W && pix[a] == OSD_DEMO_TWISTER_C_B) {
                a++;
            }
        } else {
            while (b > 0u && pix[b - 1u] == OSD_DEMO_TWISTER_C_B) {
                b--;
            }
        }
        uint8_t nW = 0;
        uint8_t nD = 0;
        bool solidRibbon = true;
        for (uint8_t i = a; i < b; i++) {
            if (pix[i] == OSD_DEMO_TWISTER_C_W) {
                nW++;
            } else if (pix[i] == OSD_DEMO_TWISTER_C_D) {
                nD++;
            } else {
                solidRibbon = false; // inner black seam: leave it to the rules below
            }
        }
        if (solidRibbon && a < b) {
            const uint8_t face = (nW >= nD) ? OSD_DEMO_TWISTER_C_W : OSD_DEMO_TWISTER_C_D;
            if (leftC == OSD_DEMO_TWISTER_C_B) {
                osdDemoTwisterEncodeTwoRun(OSD_DEMO_TWISTER_C_B, face, a, glyph, inv);
            } else {
                osdDemoTwisterEncodeTwoRun(face, OSD_DEMO_TWISTER_C_B, b, glyph, inv);
            }
            return;
        }
    }

    // B|…|R where R is a pure suffix: encode B|R at suffix start (left under-drawn).
    if (leftC == OSD_DEMO_TWISTER_C_B) {
        uint8_t t = OSD_DEMO_CELL_W;
        while (t > 0u && pix[t - 1u] == rightC) {
            t--;
        }
        osdDemoTwisterEncodeTwoRun(OSD_DEMO_TWISTER_C_B, rightC, t, glyph, inv);
        return;
    }

    // L|…|B where L is a pure prefix: encode L|B (right under-drawn).
    if (rightC == OSD_DEMO_TWISTER_C_B) {
        uint8_t t = 0;
        while (t < OSD_DEMO_CELL_W && pix[t] == leftC) {
            t++;
        }
        osdDemoTwisterEncodeTwoRun(leftC, OSD_DEMO_TWISTER_C_B, t, glyph, inv);
        return;
    }

    // Both edges colored (W|B|D, W|D, …). Right color must stay a suffix.
    {
        uint8_t tR = OSD_DEMO_CELL_W;
        while (tR > 0u && pix[tR - 1u] == rightC) {
            tR--;
        }
        uint8_t tL = 0;
        while (tL < OSD_DEMO_CELL_W && pix[tL] == leftC) {
            tL++;
        }

        bool leftPureToJoin = true;
        for (uint8_t i = 0; i < tR; i++) {
            if (pix[i] != leftC) {
                leftPureToJoin = false;
                break;
            }
        }
        if (leftPureToJoin && leftC != rightC) {
            osdDemoTwisterEncodeTwoRun(leftC, rightC, tR, glyph, inv);
            return;
        }

        // Thin black seam between two faces (W|B|D, ≤2px): absorb into leftC|rightC
        // so the join stays 1px-smooth instead of a square black notch.
        if (leftC != rightC && tL < tR) {
            bool thinBlackSeam = true;
            for (uint8_t i = tL; i < tR; i++) {
                if (pix[i] != OSD_DEMO_TWISTER_C_B) {
                    thinBlackSeam = false;
                    break;
                }
            }
            if (thinBlackSeam && (uint8_t)(tR - tL) <= 2u) {
                osdDemoTwisterEncodeTwoRun(leftC, rightC, tR, glyph, inv);
                return;
            }
        }

        // Mixed middle: keep the wider true edge, under-draw the other.
        const uint8_t leftSpan = tL;
        const uint8_t rightSpan = (uint8_t)(OSD_DEMO_CELL_W - tR);
        if (rightSpan >= leftSpan) {
            osdDemoTwisterEncodeTwoRun(OSD_DEMO_TWISTER_C_B, rightC, tR, glyph, inv);
        } else {
            osdDemoTwisterEncodeTwoRun(leftC, OSD_DEMO_TWISTER_C_B, leftSpan, glyph, inv);
        }
    }
}

// Fast scanline: only the silhouette window, no full-width color buffer / INV.
static void osdDemoTwisterBuildScanline(uint8_t *glyphs, uint8_t cols,
                                        uint8_t phase, int16_t centerX,
                                        uint8_t winL, uint8_t winR)
{
    if (winR > cols) {
        winR = cols;
    }
    if (winL >= winR) {
        return;
    }

    int16_t e[4];
    for (uint8_t k = 0; k < 4; k++) {
        const int8_t s = osdDemoSineAt((uint8_t)(phase + (uint8_t)(k * 64u)));
        e[k] = (int16_t)(centerX + (((int16_t)OSD_DEMO_TWISTER_AMP_PX * (int16_t)s) / 128));
    }

    int16_t fL[2];
    int16_t fR[2];
    uint8_t fC[2];
    uint8_t nFace = 0;
    for (uint8_t i = 0; i < 4; i++) {
        const int16_t x0 = e[i];
        const int16_t x1 = e[(uint8_t)((i + 1u) & 3u)];
        if (x1 > x0 && nFace < 2u) {
            fL[nFace] = x0;
            fR[nFace] = x1;
            fC[nFace] = ((i & 1u) == 0u) ? OSD_DEMO_TWISTER_C_W : OSD_DEMO_TWISTER_C_D;
            nFace++;
        }
    }
    // Left-to-right paint order — stable shared edge, no right-face flash over left.
    if (nFace == 2u && fL[1] < fL[0]) {
        const int16_t sl = fL[0];
        const int16_t sr = fR[0];
        const uint8_t sc = fC[0];
        fL[0] = fL[1];
        fR[0] = fR[1];
        fC[0] = fC[1];
        fL[1] = sl;
        fR[1] = sr;
        fC[1] = sc;
    }

    for (uint8_t x = winL; x < winR; x++) {
        const int16_t cell0 = (int16_t)((uint16_t)x * (uint16_t)OSD_DEMO_CELL_W);
        const int16_t cell1 = (int16_t)(cell0 + (int16_t)OSD_DEMO_CELL_W);

        // Fast path: no face edge strictly inside this cell → the cell is one solid
        // color (same result as the per-pixel path). Only the ≤4 edge cells per band
        // pay for the 12px buffer + run encoder, which keeps the build well under the
        // band period even with audio/USB IRQs stealing time.
        bool hasEdge = false;
        uint8_t solid = OSD_DEMO_TWISTER_C_B;
        for (uint8_t f = 0; f < nFace; f++) {
            if ((fL[f] > cell0 && fL[f] < cell1) || (fR[f] > cell0 && fR[f] < cell1)) {
                hasEdge = true;
                break;
            }
            if (fL[f] <= cell0 && fR[f] >= cell1) {
                solid = fC[f]; // later face wins, as in the paint loop below
            }
        }
        if (!hasEdge) {
            uint8_t inv = 0;
            osdDemoTwisterSolidGlyph(solid, &glyphs[x], &inv); // never INV
            continue;
        }

        uint8_t pix[OSD_DEMO_CELL_W];
        for (uint8_t p = 0; p < OSD_DEMO_CELL_W; p++) {
            pix[p] = OSD_DEMO_TWISTER_C_B;
        }
        for (uint8_t f = 0; f < nFace; f++) {
            int16_t a = (int16_t)(fL[f] - cell0);
            int16_t b = (int16_t)(fR[f] - cell0);
            if (a < 0) {
                a = 0;
            }
            if (b > (int16_t)OSD_DEMO_CELL_W) {
                b = (int16_t)OSD_DEMO_CELL_W;
            }
            for (int16_t p = a; p < b; p++) {
                pix[p] = fC[f];
            }
        }
        uint8_t inv = 0;
        osdDemoTwisterEncodeCell(pix, &glyphs[x], &inv);
        mgBandInv |= (uint32_t)inv << x;
    }
}

static void osdDemoPaintTwister(void)
{
    if (!demoDisplay) {
        return;
    }
    max7456FillScreen(OSD_DEMO_TWISTER_GLYPH_B);
}

// uint32 mul — (uint16)y*256 overflows for y≥256 and warps the lower third of the twist.
static uint8_t osdDemoTwisterTwistPhase(uint8_t rot, uint16_t sampleY, uint16_t span)
{
    if (span == 0) {
        return rot;
    }
    return (uint8_t)(rot + (uint8_t)(((uint32_t)sampleY * (uint32_t)OSD_DEMO_TWISTER_TWIST_NUM) / (uint32_t)span));
}

// Fixed wavelengths → the phase velocity is the same on every line. The old single sine
// with a time-varying integer period moved ∝ y (up to 4–5 px per field at the bottom vs
// 2 px at the top); consecutive interlaced fields with different geometry then read as
// square notches that grow toward the bottom of the screen.
static int8_t osdDemoTwisterBendAt(uint8_t rotA, uint16_t sampleY, uint8_t rotB)
{
    const uint8_t phA = (uint8_t)(rotA
        + (uint8_t)(((uint32_t)sampleY * 256u) / OSD_DEMO_TWISTER_BEND_WAVE_A));
    const uint8_t phB = (uint8_t)(rotB
        - (uint8_t)(((uint32_t)sampleY * 256u) / OSD_DEMO_TWISTER_BEND_WAVE_B));
    return (int8_t)(((int16_t)osdDemoSineAt(phA) * (int16_t)OSD_DEMO_TWISTER_BEND_AMP_A
                     + (int16_t)osdDemoSineAt(phB) * (int16_t)OSD_DEMO_TWISTER_BEND_AMP_B) / 128);
}

// VSYNC→VSYNC is exactly 312.5 PAL lines. Track the real line period in DWT ticks (Q16)
// instead of trusting a 64 us integer: any MCU-vs-video clock error accumulates linearly
// down the field and pushes the bottom rows' bursts into lit lines. Heavy IIR because
// each edge carries STAT-poll jitter (a few us).
static void osdDemoMgTrackLinePeriod(uint32_t edgeTicks, uint32_t nominalLineTicks)
{
    const uint32_t nominalQ16 = nominalLineTicks << 16;
    if (mgLineQ16 == 0) {
        mgLineQ16 = nominalQ16;
    }
    if (hsCapOn && mgCapLineQ16) {
        mgLastEdgeTicks = edgeTicks; // the line period comes from the HSYNC capture
        return;
    }
    if (mgLastEdgeTicks != 0) {
        const uint32_t fieldTicks = edgeTicks - mgLastEdgeTicks;
        const uint32_t nominalField = (nominalLineTicks * (uint32_t)OSD_DEMO_PAL_FIELD_LINES_X2) / 2u;
        // ±5%: still rejects missed VSYNCs (2×) and glitches, but accepts real off-nominal
        // sources — on HAKRCF411D + this camera VSYNC→VSYNC is ~20.43 ms in DWT ticks
        // (line ≈ 65.4 "us"): a ±0.5% gate rejected every field and left the model at
        // 64 us, ~24 us of drift per char row = the growing corners in the lower half.
        const uint32_t tol = nominalField / 20u;
        mgFieldLastTicks = fieldTicks;
        if (fieldTicks > nominalField - tol && fieldTicks < nominalField + tol) {
            const uint32_t measQ16 = (uint32_t)(((uint64_t)fieldTicks << 17)
                                                / (uint64_t)OSD_DEMO_PAL_FIELD_LINES_X2);
            if (mgLineSeeded) {
                mgLineQ16 = (uint32_t)((int32_t)mgLineQ16
                    + (((int32_t)measQ16 - (int32_t)mgLineQ16) >> OSD_DEMO_MG_LINE_IIR_SHIFT));
            } else {
                mgLineQ16 = measQ16; // jump straight to the first real measurement
                mgLineSeeded = true;
            }
        }
    }
    mgLastEdgeTicks = edgeTicks;
}

// Signed offset of `line` from the pivot row (calibrated at nominal 64 us/line).
static inline int32_t osdDemoMgLineOffset(uint16_t line, uint32_t lineQ16)
{
    const int32_t dl = (int32_t)line - (int32_t)OSD_DEMO_MG_PIVOT_LINE;
    return (int32_t)(((int64_t)dl * (int64_t)lineQ16) >> 16);
}

// Diff a built band against what Display SRAM already holds for that row and write only
// the changed cells. Per the datasheet (p.40) every SPI display-memory write can collide
// with the chip's own fetch and break up a character for the field — fewer writes, fewer
// collisions, and a much shorter burst (typically 3–6 cells instead of the whole window).
// checkDeadline: skip (leave SRAM as-is) when the burst cannot finish before finishBy.
// Plan one row burst: cluster changed cells (clean gaps ≤ MERGE_GAP), then per cluster pick
// the cheaper framing — one auto-increment run (DMM, DMAL, 2/cell, END = 6 + 2·span) or
// addressed single writes (DMAL+DMDI = 4/changed cell, + DMM once after an AI run). Greedy
// is within 0.5% of the DP optimum on the plasma; −18% bytes vs "AI runs only", and no
// unchanged cell is ever rewritten by a single-write cluster. Returns the segment count.
// The same plan from a mask of the cells to write (bit c = column c).
static uint8_t osdDemoMgPlanMask(uint32_t chg, uint8_t winL, uint8_t winR, max7456SramSeg_t *seg)
{
#define MG_SAME(c) (!((chg >> (c)) & 1u))
    uint8_t nSeg = 0;
    bool afterAi = true; // DMM state unknown at burst start
    uint8_t x = winL;
    while (x < winR) {
        if (MG_SAME(x)) {
            x++;
            continue;
        }
        const uint8_t first = x;
        uint8_t last = x;
        uint8_t changed = 1;
        x++;
        while (x < winR) {
            if (!MG_SAME(x)) {
                last = x;
                changed++;
                x++;
                continue;
            }
            uint8_t z = x;
            while (z < winR && MG_SAME(z)) {
                z++;
            }
            if (z < winR && (uint8_t)(z - x) <= OSD_DEMO_MG_RUN_MERGE_GAP) {
                x = z;
            } else {
                break;
            }
        }
        const uint8_t span = (uint8_t)(last - first + 1u);
        const uint16_t aiCost = (uint16_t)(6u + 2u * span);
        const uint16_t singleCost = (uint16_t)((afterAi ? 2u : 0u) + 4u * changed);
        if (span > 1u && aiCost < singleCost) {
            seg[nSeg].col = first;
            seg[nSeg].len = span;
            seg[nSeg].autoInc = true;
            nSeg++;
            afterAi = true;
        } else {
            for (uint8_t c = first; c <= last; c++) {
                if (MG_SAME(c)) {
                    continue;
                }
                // Merge adjacent singles into one segment (still one DMAL per cell).
                if (nSeg > 0 && !seg[nSeg - 1u].autoInc
                    && (uint8_t)(seg[nSeg - 1u].col + seg[nSeg - 1u].len) == c) {
                    seg[nSeg - 1u].len++;
                } else {
                    seg[nSeg].col = c;
                    seg[nSeg].len = 1;
                    seg[nSeg].autoInc = false;
                    nSeg++;
                }
            }
            afterAi = false;
        }
        x = (uint8_t)(last + 1u);
    }
    return nSeg;
#undef MG_SAME
}

static uint8_t osdDemoMgPlanRow(uint8_t row, const uint8_t *glyphs, uint32_t inv, uint8_t winL,
                                uint8_t winR, max7456SramSeg_t *seg)
{
    const uint8_t *sram = mgSram[row];
    const uint32_t invDiff = inv ^ mgSramInv[row]; // a cell also changes when its INV does
    uint32_t chg = 0;
    for (uint8_t c = winL; c < winR; c++) {
        if (glyphs[c] != sram[c] || ((invDiff >> c) & 1u)) {
            chg |= 1u << c;
        }
    }
    return osdDemoMgPlanMask(chg, winL, winR, seg);
}

static void osdDemoMgCommit(uint8_t row, const uint8_t *glyphs, uint32_t inv,
                            const max7456SramSeg_t *seg, uint8_t nSeg, uint16_t bytes, uint32_t spent)
{
    for (uint8_t i = 0; i < nSeg; i++) {
        memcpy(&mgSram[row][seg[i].col], &glyphs[seg[i].col], seg[i].len);
        const uint32_t m = (seg[i].len >= 32u) ? 0xFFFFFFFFu
            : (((1u << seg[i].len) - 1u) << seg[i].col);
        mgSramInv[row] = (mgSramInv[row] & ~m) | (inv & m);
    }
    // Learn the real SPI cost per byte (clock divider, polled-loop gaps, CS overhead).
    // A burst stretched by an IRQ (the 50 Hz PT3 tick runs inside the audio IRQ) is not an
    // SPI measurement: one such sample used to inflate the estimate, mis-plan the next 4–5
    // bands (torn 1-px lines) and, when large enough, overflowed the start-time maths into a
    // ~20 s wait (the scene "hang"). Accept faster/near samples; take a slow one only if it
    // keeps repeating.
    if (spent > mgBurstFixedTicks) {
        const uint32_t estTicks = mgBurstFixedTicks + (((uint32_t)bytes * mgByteTicksQ8) >> 8);
        const bool slow = spent > estTicks + estTicks / 2u;
        if (max7456Zone16Active()) {
            // DMA zone session: its words share the wire inside bursts — keep the rate measured
            // without them (a burst's time here is not its own bytes' time).
        } else
        if (!slow || ++mgSlowRun >= 16u) {
            const uint32_t perByteQ8 = ((spent - mgBurstFixedTicks) << 8) / bytes;
            mgByteTicksQ8 = (uint32_t)((int32_t)mgByteTicksQ8
                + (((int32_t)perByteQ8 - (int32_t)mgByteTicksQ8) >> 3));
            mgSlowRun = 0;
        }
    }
}

// Diff a built band against what Display SRAM already holds for that row and write only
// the changed cells. Per the datasheet (p.40) every SPI display-memory write can collide
// with the chip's own fetch and break up a character for the field — fewer writes, fewer
// collisions, and a much shorter burst (typically 3–6 cells instead of the whole window).
// checkDeadline: skip (leave SRAM as-is) when the burst cannot finish before finishBy.
static bool osdDemoMgWriteRow(uint8_t row, const uint8_t *glyphs, uint8_t winL, uint8_t winR,
                              uint32_t finishBy, bool checkDeadline, bool maskIrq)
{
    max7456SramSeg_t seg[OSD_DEMO_CHARS_PER_LINE];
    uint16_t segLast[OSD_DEMO_CHARS_PER_LINE];
    const uint32_t inv = mgBandInv;
    const uint8_t nSeg = osdDemoMgPlanRow(row, glyphs, inv, winL, winR, seg);
    if (nSeg == 0) {
        return true;
    }
    const uint16_t addr = (uint16_t)((uint16_t)row * (uint16_t)OSD_DEMO_CHARS_PER_LINE);
    const uint16_t bytes = max7456EncodeDisplaySramRowAttr(addr, glyphs, inv, seg, nSeg, NULL, segLast);
    if (bytes == 0) {
        return false;
    }
    const uint32_t estTicks = mgBurstFixedTicks + ((uint32_t)bytes * mgByteTicksQ8 >> 8);

    // Deadline check and burst are atomic: the 16 kHz AY IRQ (same top priority as gyro
    // EXTI) or its 50 Hz tracker tick must not land between "fits" and "done" and push
    // the tail of the burst into the lit line. PRIMASK — BASEPRI cannot mask prio 0.
#if defined(__CORTEX_M) // CMSIS core present
    const uint32_t primask = __get_PRIMASK();
    if (maskIrq) {
        __disable_irq();
    }
#else
    UNUSED(maskIrq);
#endif
    const uint32_t t0 = getCycleCounter();
    bool ok = false;
    if (!checkDeadline || (int32_t)(finishBy - t0) >= (int32_t)estTicks) {
        ok = max7456SendEncodedDisplaySram(bytes);
    }
    const uint32_t t1 = getCycleCounter();
#if defined(__CORTEX_M) // CMSIS core present
    __set_PRIMASK(primask);
#endif

    if (!ok) {
        return false;
    }
    osdDemoMgCommit(row, glyphs, inv, seg, nSeg, bytes, t1 - t0);
    return true;
}

// Race the beam: start right behind the beam in line (due−1) and require every cell to land
// before the beam reaches it in line `due`. Per cell that is a whole line of window no matter
// how long the burst is — a full 30-cell row fits, while "finish the burst inside the previous
// line" (≈57 us) never can. Segment landing times come from the exact encoded byte offsets.
// Every cell must land after the beam left it in line due−1 and before the beam reaches it in
// line `due`; within one segment the writer moves at a constant rate (AI ~2 B/cell can be
// faster than the beam, singles ~4 B/cell slower), so its two end cells bound it. Result: a
// start window [startAt, latest] for the burst.

// Once per field, in VBLANK (13 lines before the first OSD line, after the equalizing
// pulses): the absolute HSYNC phase of THIS field and which field it is. PAL field 2's VSYNC
// starts mid-line (datasheet Fig. 6/7), so HSYNC vs VSYNC differs by ½ line (~32 us) between
// fields. The old cross-field IIR averaged the two and left race-the-beam ~±16 us off in
// every field — at the edge of its tolerance: in one field cells landed a line early, and the
// monitor weaving both fields showed a ±1-line comb ("grid"). Parity also lets animations
// advance once per frame on the right field.
static void osdDemoMgMeasureFieldPhase(uint32_t vsyncEdge, uint32_t row0, uint32_t lineTicks)
{
    const uint32_t model = row0 - (uint32_t)OSD_DEMO_MG_FIELD_PHASE_LINES * lineTicks;
    uint32_t edge = 0;
    bool ok = false;
    if (hsCapOn) {
        // HSYNC wired: the phase line's edge from the capture — exact, no STAT polling.
        osdDemoWaitCycles(model + lineTicks / 8u); // past the edge
        edge = osdDemoHsSnap(model, lineTicks);
        ok = edge != model; // else no fresh edge (signal gone): STAT below
    }
    if (!ok) {
        osdDemoWaitCycles(model - lineTicks * 3u / 8u);
        max7456MidGlyphSpiBoost(false); // STAT polling is unreliable at the boosted clock
        ok = max7456WaitHsyncFallingEdge(&edge, 150); // ≥2 chances at a short pulse
        max7456MidGlyphSpiBoost(true);
    }
    if (!ok) {
        mgFieldFirst = !mgFieldFirst; // fields alternate
        if (mgHsyncPhaseByValid[mgFieldFirst]) {
            mgHsyncPhase = mgHsyncPhaseBy[mgFieldFirst];
        }
        return;
    }
    int32_t ph = (int32_t)(edge - model);
    const int32_t half = (int32_t)lineTicks / 2;
    while (ph > half) {
        ph -= (int32_t)lineTicks;
    }
    while (ph < -half) {
        ph += (int32_t)lineTicks;
    }
    const uint32_t pv = (edge - vsyncEdge) % lineTicks;
    mgFieldFirst = (pv < lineTicks / 4u) || (pv > lineTicks - lineTicks / 4u);
    // Field 2's HSYNC sits ~½ line off the VSYNC model: wrapped to ±½ line its phase flipped
    // between +½ and −½ with VSYNC jitter — the whole field a line off, now and then. Keep it
    // next to the same parity's last phase instead.
    if (mgHsyncPhaseByValid[mgFieldFirst]) {
        const int32_t prev = mgHsyncPhaseBy[mgFieldFirst];
        while (ph - prev > half) {
            ph -= (int32_t)lineTicks;
        }
        while (ph - prev < -half) {
            ph += (int32_t)lineTicks;
        }
    }
    // Never more than ¾ line off the model: the phase is a fraction of a line (field 2 near
    // ±½). Kept next to its last value it could walk a whole line on every misread parity —
    // after a long run several lines, past the line-0 correction (±3 lines), and every
    // interrupt scene showed its rows a line or more off until a reboot.
    const int32_t bound = (int32_t)lineTicks * 3 / 4;
    while (ph > bound) {
        ph -= (int32_t)lineTicks;
    }
    while (ph < -bound) {
        ph += (int32_t)lineTicks;
    }
    if (hsCapOn) {
        // Line period from the capture over a whole frame: the same line two fields ago (same
        // parity) is 625 lines back — exact enough to replace the VSYNC-based estimate.
        static uint32_t prevEdge[2];
        static uint32_t prevField[2];
        static bool prevOk[2];
        const uint8_t par = mgFieldFirst ? 1u : 0u;
        if (prevOk[par] && mgStatFields - prevField[par] == 2u) {
            const uint32_t q16 = (uint32_t)(((uint64_t)(edge - prevEdge[par]) << 16) / 625u);
            if (q16 > (lineTicks << 16) * 15u / 16u && q16 < (lineTicks << 16) / 16u * 17u) {
                mgCapLineQ16 = mgCapLineQ16 ? mgCapLineQ16 - (mgCapLineQ16 >> 3) + (q16 >> 3) : q16;
                mgLineQ16 = mgCapLineQ16;
            }
        }
        prevEdge[par] = edge;
        prevField[par] = mgStatFields;
        prevOk[par] = true;
        mgCapEdge = edge;
        mgCapFields++;
    }
    mgHsyncPhase = ph;
    mgHsyncPhaseBy[mgFieldFirst] = ph;
    mgHsyncPhaseByValid[mgFieldFirst] = true;
}

static void osdDemoTwisterBuildBand(uint8_t *glyphs, uint8_t cols, uint16_t line, uint8_t rot,
                                    uint8_t bendRot, uint8_t bendFloat, uint16_t span,
                                    int16_t centerX, uint8_t winL, uint8_t winR)
{
    const uint16_t sampleY = (uint16_t)(line + (OSD_DEMO_TWISTER_BAND_PX / 2u));
    const int16_t cx = (int16_t)(centerX + osdDemoTwisterBendAt(bendRot, sampleY, bendFloat));
    osdDemoTwisterBuildScanline(glyphs, cols, osdDemoTwisterTwistPhase(rot, sampleY, span), cx, winL, winR);
}

// --- Scene 8: twister on the unified engine (bands built in the ISR, a few us each) ---------
static int16_t twCenterX;
static uint8_t twWinL;
static uint8_t twWinR;
static uint16_t twSpan;
static uint8_t twRot;    // this field's phases (field hook → band builder)
static uint8_t twBendA;
static uint8_t twBendB;
static uint32_t twTextPos; // scroller: bands (3 lines) moved since the scene start
// Row brightness: white 80 % / 90 % on the two rows at the top and at the bottom, 120 % between.
static uint8_t twRowWhite[OSD_DEMO_MG_ROWS_MAX];
static bool twRbAll;
static uint8_t twRbNext;
static void osdDemoRbRefresh(const uint8_t *white, uint8_t rows, bool *all, uint8_t *next);

// Stars on circular orbits around the twister's axis, tilted towards the viewer, in 3×3-px
// pixels (white on black, shared 3-colour set). Perspective scales the orbit and the star size
// (near ones 2×2 pixels); behind the ribbon they hide, in front they cover it. Every beat they
// hop off their orbits and land back on the beat. At every new section of the track the stars
// fall out through the bottom (still orbiting, speeding up) while a new set drops in from the top
// and settles onto its orbits.
#define OSD_DEMO_TW_STARS        20
#define OSD_DEMO_TW_STAR_FOCAL   200  // logical px: perspective strength
#define OSD_DEMO_TW_STAR_TILT    64   // orbit tilt, /256 of the radius on screen
#define OSD_DEMO_TW_STAR_NEAR    288  // scale (/256) from which a star is 2×2
#define OSD_DEMO_TW_STAR_SHIFT   (-10) // orbit axis vs the ribbon axis, logical px (left: reach the screen edge)
#define OSD_DEMO_TW_STAR_HOP     5    // logical px: hop off the orbit on every beat (parabola)
#define OSD_DEMO_TW_STAR_BEAT_MS 1000 // one hop per two quarters (the tracker lines nearest it)
#define OSD_DEMO_TW_SWAP_MS      700  // a section change: the old set falls out, the new one in
#define OSD_DEMO_TW_SWAP_DROP    120  // logical px: far enough to clear the screen (96 rows)
typedef struct osdDemoTwStar_s {
    uint8_t angle;      // orbit phase (256 = a turn)
    uint8_t speed;      // phase steps per frame
    uint8_t radius;     // logical px
    uint8_t height;     // orbit centre row, logical px
} osdDemoTwStar_t;
static osdDemoTwStar_t twStars[2][OSD_DEMO_TW_STARS]; // the set shown and the one leaving
static uint8_t twSet;            // the set shown (the other falls out during a swap)
static bool twSwap;
static timeMs_t twSwapMs;
static int16_t twLastOrder = -1; // track section seen last (-1: none yet)
// Projected, both sets (size 0: not drawn)
static int16_t twStarX[2 * OSD_DEMO_TW_STARS];  // top-left, logical px
static int16_t twStarY[2 * OSD_DEMO_TW_STARS];
static uint8_t twStarSize[2 * OSD_DEMO_TW_STARS];
static bool twStarFront[2 * OSD_DEMO_TW_STARS];

static void osdDemoTwisterStarsSet(uint8_t set)
{
    for (uint8_t i = 0; i < OSD_DEMO_TW_STARS; i++) {
        osdDemoTwStar_t *st = &twStars[set][i];
        st->angle = (uint8_t)osdDemoRand();
        st->speed = (uint8_t)(1u + osdDemoRand() % 3u);
        st->radius = (uint8_t)(26u + osdDemoRand() % 22u); // near side reaches the left edge
        st->height = (uint8_t)(8u + (uint32_t)i * 80u / OSD_DEMO_TW_STARS + osdDemoRand() % 4u);
    }
}

static void osdDemoTwisterStarsInit(void)
{
    twSet = 0;
    twSwap = false;
    twLastOrder = -1;
    osdDemoTwisterStarsSet(0);
}

// Once a field: step the orbits (once a frame) and project every star.
static void osdDemoTwisterStarsProject(bool step)
{
    const int32_t cx = twCenterX / 3 + OSD_DEMO_TW_STAR_SHIFT;          // logical px
    const int32_t cy = (OSD_DEMO_CHECKER_ROWS * OSD_DEMO_P3_BANDS) / 2; // 48
    // Beat hop: up and back onto the orbit, landing on the beat (music-locked phase).
    const int32_t p = osdDemoMusicPhase(OSD_DEMO_TW_STAR_BEAT_MS, fxStartMs);
    const int32_t hop = OSD_DEMO_TW_STAR_HOP * 4 * p * (256 - p) / 65536;
    // A new section of the track: the shown set leaves, a new one comes
    chiptuneClock_t ck;
    if (chiptuneGetClock(&ck) && (int16_t)ck.order != twLastOrder) {
        if (twLastOrder >= 0) {
            twSet ^= 1u;
            osdDemoTwisterStarsSet(twSet);
            twSwap = true;
            twSwapMs = millis();
        }
        twLastOrder = (int16_t)ck.order;
    }
    int32_t q = 256; // swap progress /256
    if (twSwap) {
        q = (int32_t)((millis() - twSwapMs) * 256u / OSD_DEMO_TW_SWAP_MS);
        if (q >= 256) {
            q = 256;
            twSwap = false;
        }
    }
    // The leaving set drops at once and ever faster (a start speed, then q²: from rest it hung
    // on its orbit for the first fifth and the swap looked late); the new one slows to its orbit
    const int32_t dropOld = OSD_DEMO_TW_SWAP_DROP * (35 * q * 256 + 65 * q * q) / (100 * 65536);
    const int32_t dropNew = -OSD_DEMO_TW_SWAP_DROP * (256 - q) * (256 - q) / 65536;
    for (uint8_t s = 0; s < 2u; s++) {
        const bool shown = s == twSet;
        const bool on = shown || twSwap;
        const int32_t drop = shown ? dropNew : dropOld;
        for (uint8_t i = 0; i < OSD_DEMO_TW_STARS; i++) {
            osdDemoTwStar_t *st = &twStars[s][i];
            const uint8_t k = (uint8_t)(s * OSD_DEMO_TW_STARS + i);
            if (!on) {
                twStarSize[k] = 0;
                continue;
            }
            if (step) {
                st->angle = (uint8_t)(st->angle + st->speed);
            }
            const int32_t x3 = (int32_t)st->radius * osdDemoSineAt((uint8_t)(st->angle + 64u)) / 127;
            const int32_t z3 = (int32_t)st->radius * osdDemoSineAt(st->angle) / 127; // + = away
            const int32_t y3 = (int32_t)st->height - cy + z3 * OSD_DEMO_TW_STAR_TILT / 256 - hop;
            const int32_t scale = OSD_DEMO_TW_STAR_FOCAL * 256 / (OSD_DEMO_TW_STAR_FOCAL + z3); // /256
            twStarX[k] = (int16_t)(cx + x3 * scale / 256);
            twStarY[k] = (int16_t)(cy + y3 * scale / 256 + drop);
            twStarSize[k] = (scale >= OSD_DEMO_TW_STAR_NEAR) ? 2u : 1u;
            twStarFront[k] = z3 < 0;
        }
    }
}

// Build window = silhouette + geometric bend; the burst is only the changed cells in it.
static void osdDemoTwisterSetupGeometry(uint8_t cols, uint8_t rows)
{
    twCenterX = (int16_t)(((uint16_t)cols * (uint16_t)OSD_DEMO_CELL_W) / 2u);
    int16_t winPx0 = (int16_t)(twCenterX - (int16_t)OSD_DEMO_TWISTER_AMP_PX
                               - (int16_t)OSD_DEMO_TWISTER_WIN_MARGIN_PX);
    int16_t winPx1 = (int16_t)(twCenterX + (int16_t)OSD_DEMO_TWISTER_AMP_PX
                               + (int16_t)OSD_DEMO_TWISTER_WIN_MARGIN_PX);
    if (winPx0 < 0) {
        winPx0 = 0;
    }
    const int16_t screenW = (int16_t)((uint16_t)cols * (uint16_t)OSD_DEMO_CELL_W);
    if (winPx1 > screenW) {
        winPx1 = screenW;
    }
    twWinL = (uint8_t)(winPx0 / (int16_t)OSD_DEMO_CELL_W);
    twWinR = (uint8_t)MIN((winPx1 + (int16_t)OSD_DEMO_CELL_W - 1) / (int16_t)OSD_DEMO_CELL_W, (int16_t)cols);
    if (twWinL >= twWinR) {
        twWinL = 0;
        twWinR = cols;
    }
    twSpan = (uint16_t)MIN((uint16_t)OSD_DEMO_TWISTER_LINE_SPAN, (uint16_t)((uint16_t)rows * OSD_DEMO_CELL_H));
}

static void osdDemoTwisterField(uint8_t cols, uint8_t rows, bool frameStart,
                                osdDemoRasterMotion_t *next)
{
    UNUSED(cols);
    osdDemoRbRefresh(twRowWhite, MIN(rows, OSD_DEMO_MG_ROWS_MAX), &twRbAll, &twRbNext);
    if (frameStart) {
        // One step per frame, doubled: same speed as the old per-field animation.
        twisterPhase = (uint8_t)(twisterPhase + 2u * OSD_DEMO_TWISTER_ROT_STEP);
        twisterBendPhase = (uint8_t)(twisterBendPhase + 2u * OSD_DEMO_TWISTER_BEND_ROT_STEP);
        twisterBendFloat = (uint8_t)(twisterBendFloat + 2u * OSD_DEMO_TWISTER_BEND_FLOAT_STEP);
    }
    twRot = twisterPhase;
    twBendA = twisterBendPhase;
    twBendB = twisterBendFloat;
    twTextPos = (uint32_t)(millis() - fxStartMs) * OSD_DEMO_TWISTER_TEXT_BANDS_PER_S / 1000u;
    osdDemoTwisterStarsProject(frameStart);
    next->vos = 0;
}

// Stars on this band (logical row y·6 + b), no hardware grey: white on black off the ribbon;
// in front of it the ribbon's own white | dither glyphs — a dither (grey) star on a white cell,
// a white star on a dither cell, the run reaching the nearer cell edge. Over the ribbon's
// mixed edge cells a star is not drawn (no glyph for it).
// The scroller columns stay theirs.
static void osdDemoTwisterStars(uint8_t y, uint8_t b, uint8_t cols, uint8_t *glyphs)
{
    const int16_t ly = (int16_t)((uint16_t)y * (OSD_DEMO_CELL_H / OSD_DEMO_TWISTER_BAND_PX) + b);
    const uint8_t xMax = (cols > OSD_DEMO_TWISTER_TEXT_CELLS) ? (uint8_t)(cols - OSD_DEMO_TWISTER_TEXT_CELLS) : 0u;
    uint8_t mask[OSD_DEMO_CHARS_PER_LINE];
    uint32_t touched = 0;
    uint32_t front = 0;
    for (uint8_t i = 0; i < 2u * OSD_DEMO_TW_STARS; i++) {
        if (ly < twStarY[i] || ly >= twStarY[i] + twStarSize[i]) {
            continue;
        }
        for (uint8_t d = 0; d < twStarSize[i]; d++) {
            const int16_t lx = (int16_t)(twStarX[i] + d);
            if (lx < 0) {
                continue;
            }
            const uint8_t cx = (uint8_t)(lx / OSD_DEMO_P3_LP);
            if (cx >= xMax) {
                continue;
            }
            const uint32_t bit = 1u << cx;
            if (!(touched & bit)) {
                touched |= bit;
                mask[cx] = 0;
            }
            mask[cx] |= (uint8_t)(1u << (lx % OSD_DEMO_P3_LP));
            if (twStarFront[i]) {
                front |= bit;
            }
        }
    }
    uint32_t inv = 0;
    for (uint8_t cx = 0; touched; cx++) {
        const uint32_t bit = 1u << cx;
        if (!(touched & bit)) {
            continue;
        }
        touched &= ~bit;
        const bool onRibbon = glyphs[cx] != OSD_DEMO_TWISTER_GLYPH_B;
        if (onRibbon && !(front & bit)) {
            continue; // behind the ribbon
        }
        const uint8_t g = glyphs[cx];
        if (g == OSD_DEMO_TWISTER_GLYPH_D || g == OSD_DEMO_TWISTER_GLYPH_W) {
            // Star colour = the run that is not the cell's own: from the left edge to `last`,
            // or from `first` to the right edge, whichever adds less. 3 px per logical pixel.
            uint8_t first = 0;
            while (!((mask[cx] >> first) & 1u)) {
                first++;
            }
            uint8_t last = OSD_DEMO_P3_LP - 1u;
            while (!((mask[cx] >> last) & 1u)) {
                last--;
            }
            const bool grey = g == OSD_DEMO_TWISTER_GLYPH_D;
            const uint8_t starC = grey ? OSD_DEMO_TWISTER_C_W : OSD_DEMO_TWISTER_C_D;
            const uint8_t cellC = grey ? OSD_DEMO_TWISTER_C_D : OSD_DEMO_TWISTER_C_W;
            uint8_t sInv = 0;
            if (first <= (uint8_t)(OSD_DEMO_P3_LP - 1u - last)) {
                const uint8_t k = (uint8_t)(3u * (last + 1u)); // star run on the left
                osdDemoTwisterEncodeTwoRun(starC, cellC, k, &glyphs[cx], &sInv);
            } else {
                const uint8_t k = (uint8_t)(3u * first);       // cell colour on the left
                osdDemoTwisterEncodeTwoRun(cellC, starC, k, &glyphs[cx], &sInv);
            }
            inv |= (uint32_t)sInv << cx;
            continue;
        }
        if (g != OSD_DEMO_TWISTER_GLYPH_B) {
            continue; // ribbon edge cell, or behind the ribbon
        }
        uint8_t p = 0;
        for (uint8_t k = 0; k < OSD_DEMO_P3_LP; k++) {
            p = (uint8_t)(p * 3u + (((mask[cx] >> k) & 1u) ? 2u : 0u)); // left first: 0 black, 2 white
        }
        glyphs[cx] = osdDemo3c[p];
        inv |= (uint32_t)osdDemo3cInv[p] << cx;
    }
    mgBandInv |= inv;
}

// One band of the right-edge scroller: 12 logical px (3 cells, 6 font pixels of 2) on this band.
static void osdDemoTwisterText(uint8_t y, uint8_t b, uint8_t cols, uint8_t *glyphs)
{
    static const uint16_t len = sizeof(osdDemoTwisterScrollText) - 1u;
    if (cols < OSD_DEMO_TWISTER_TEXT_CELLS) {
        return;
    }
    // Text position under this band, in bands, counting up the screen from the newest: the
    // text moves down and letter k + 1 sits above letter k. Font pixels are 2 bands tall.
    const int32_t u = (int32_t)twTextPos - (int32_t)((uint16_t)y * (OSD_DEMO_CELL_H / OSD_DEMO_TWISTER_BAND_PX) + b);
    // Turned CCW, the letter's right edge is on top: the font column shown here counts up the
    // screen (0 = the letter's left edge, at the bottom); the gap (column 5) is above the letter.
    int8_t fcol = -1;
    uint8_t ch = ' ';
    if (u >= 0) {
        const uint32_t fu = (uint32_t)u / 2u; // font pixels
        fcol = (int8_t)(fu % OSD_DEMO_FONT_ADVANCE);
        if (fcol >= OSD_DEMO_FONT_W) {
            fcol = -1;
        }
        ch = (uint8_t)osdDemoTwisterScrollText[(fu / OSD_DEMO_FONT_ADVANCE) % len];
    }
    const uint8_t x0 = (uint8_t)(cols - OSD_DEMO_TWISTER_TEXT_CELLS);
    uint32_t inv = 0;
    for (uint8_t c = 0; c < OSD_DEMO_TWISTER_TEXT_CELLS; c++) {
        uint8_t p = 0;
        for (uint8_t k = 0; k < OSD_DEMO_P3_LP; k++) {
            // Turned CCW: screen column = font row (top of the letter on the left). Column 5
            // of 6 stays black (right margin).
            const uint8_t col = (uint8_t)((c * OSD_DEMO_P3_LP + k) / 2u);
            const bool on = fcol >= 0 && col < 5u
                && osdDemoFontPixel(ch, (uint8_t)fcol, (uint8_t)(OSD_DEMO_TUNNEL_INK_Y0 + col));
            p = (uint8_t)(p * 3u + (on ? 2u : 0u)); // base 3, left first: 0 black, 2 white
        }
        glyphs[x0 + c] = osdDemo3c[p];
        inv |= (uint32_t)osdDemo3cInv[p] << (x0 + c);
    }
    mgBandInv |= inv;
}

static void osdDemoTwisterBand(uint8_t y, uint8_t b, uint8_t cols, uint8_t *glyphs)
{
    memset(glyphs, OSD_DEMO_TWISTER_GLYPH_B, cols);
    const uint16_t line = (uint16_t)((uint16_t)y * OSD_DEMO_CELL_H + (uint16_t)b * OSD_DEMO_TWISTER_BAND_PX);
    if (line < twSpan) {
        osdDemoTwisterBuildBand(glyphs, cols, line, twRot, twBendA, twBendB, twSpan, twCenterX, twWinL, twWinR);
    }
    osdDemoTwisterStars(y, b, cols, glyphs);
    osdDemoTwisterText(y, b, cols, glyphs);
}

static const osdDemoIrqScene_t osdDemoTwisterScene = {
    .bandLines = OSD_DEMO_TWISTER_BAND_PX,
    .bands = OSD_DEMO_CELL_H / OSD_DEMO_TWISTER_BAND_PX,
    .armed = &twisterArmed,
    .field = osdDemoTwisterField,
    .band = osdDemoTwisterBand,
    .task = NULL,
};

static void osdDemoEnterFx(osdDemoFx_e next) __attribute__((noinline));

// Scene end on the song: start = nearest pattern boundary, end = start + N patterns.
static void osdDemoFxArmEnd(osdDemoFx_e f)
{
    fxEndByRow = false;
    const uint8_t pats = osdDemoFxPatterns(f);
    uint32_t rowQ8;
    uint8_t speed;
    if (pats && osdDemoSongClockQ8(&rowQ8, &speed)) {
        const uint32_t patQ8 = OSD_DEMO_PATTERN_ROWS * 256u;
        fxEndRowQ8 = ((rowQ8 + patQ8 / 2u) / patQ8 + pats) * patQ8;
        fxEndByRow = true;
    }
}

static void osdDemoEnterFx(osdDemoFx_e next)
{
    if (next == OSD_DEMO_FX_PLASMA) { // its gather: the picture on screen now, before the blanking
        max7456CopyLayer(&plasmaGatherCode[0][0], &plasmaGatherInv[0][0], sizeof(plasmaGatherCode));
    }
    // Drop any mid-glyph SPI boost before scene setup (NVM writes / RefreshAll).
    max7456MidGlyphSpiEnd();
    const osdDemoFx_e prevFx = fx;
    fx = next;
    osdDemoFxArmEnd(next);
    if (next != OSD_DEMO_FX_SCROLLER) {
        s1Timeline = false;
    }
    fxStartMs = millis();
    punchY = 0;
    punchVel = 0;
    punchArmed = false;
    plasma2x2Armed = false;
    twisterArmed = false;
    shoutArmed = false;
    tunnelArmed = false;
    wipeArmed = false;
    donutArmed = false;
    megaArmed = false;
    disarmArmed = false;
    rotoArmed = false;
    cubeArmed = false;
    bmArmed = false;
    osdDemoIrqStop();
    // Clear pixels black first, whatever the scene before left in VM1 (grey, a wipe flash,
    // transparent): every scene sets its own background in its enter below, and the switch
    // itself never shows grey or video (scene 1 after scene 10 flashed grey).
    osdDemoBgType(DISPLAY_BACKGROUND_BLACK);
    // Blank: a raster scene leaves each cell with the code of its last line, the same on all
    // 18 lines — vertical stripes over the whole screen (lit by the scene-start flash) until
    // the next scene has written its picture.
    if (demoDisplay) {
        osdDemoFillRowsGlyphFast(demoDisplay->cols, demoDisplay->rows, OSD_DEMO_PIXEL_OFF, OSD_DEMO_PIXEL_OFF);
    }
    if (prevFx == OSD_DEMO_FX_BITMAP) {
        max7456Brightness(0, 3); // a black / white level per row
    }
    memset(prevRowBright, 0xFF, sizeof(prevRowBright));
    max7456Invalidate();
    // Row levels back to normal behind the scene-start flash (the interrupt engine does it in
    // VBLANK for its scenes; the character scenes here, released by the task after two fields).
    if (max7456RbHeld()) {
        max7456RbHold(false, 0);
    }
    max7456RbResetWanted(OSD_DEMO_RB_NORMAL);
    charRbFlashEndMs = 0;
    if (!osdDemoFxIsMidGlyph(next)) {
        max7456RbHold(true, OSD_DEMO_RB_FLASH_REG);
        charRbFlashEndMs = millis() + 2u * 20u;
        if (!charRbFlashEndMs) {
            charRbFlashEndMs = 1;
        }
    }
    if (next == OSD_DEMO_FX_PLASMA) {
        plasmaT = 0;
        plasmaSnakeOn = false;
        plasmaSnakeOrder = -1;
        plasmaSnakeCount = 0;
        // the last picture (taken at the switch's start) gathers into the band, in a flash
        plasmaGather = true;
        plasmaGatherMs = millis();
        charRbFlashEndMs = 0; // the gather holds the rows itself
        max7456Brightness(0, 3);
        // 7-step dither ladder (in the font since the start): no per-cell INV, so a plasma row is
        // one auto-increment run instead of a new run at every INV change.
        osdDemoS1Reset();
        memset(s1RowWhite, 3, sizeof(s1RowWhite));
        max7456FillScreen(OSD_DEMO_PIXEL_OFF);
        osdDemoPaintPlasma();
    } else if (next == OSD_DEMO_FX_FIRE) {
        firePhase = OSD_DEMO_FIRE_COLLAPSE;
        fireStage = 0;
        fireStageOrder = -1;
        fireStageFlash = false;
        fireScrollPos = 0;
        fireScrollDrawn = 0xFFFF;
        fireTextTop = OSD_DEMO_FIRE_TEXT_TOP;
        fireTextTopDrawn = 0xFF;
        fireFall = 0;
        fireTick = 0;
        // Count beats from the nearest beat: the collapse and the burst sit on the grid.
        fireBeat0Q8 = (osdDemoBeatsQ8(OSD_DEMO_FIRE_BEAT_MS, fxStartMs) + 128u) & ~0xFFu;
        memset(fireTipFine, 0, sizeof(fireTipFine));
        memset(fireTipMask, 0xFF, sizeof(fireTipMask));
        memset(fireSparks, 0, sizeof(fireSparks));
        memset(fireFuelAvg, 6, sizeof(fireFuelAvg));
        memset(fireDrawn, 0xFF, sizeof(fireDrawn)); // force dirty first paint
        fireFlameTop = OSD_DEMO_FIRE_ROWS - 1;
        fireFlameTopRaw = fireFlameTop;
        fireTopHold = 0;
        osdDemoApplyOsdmNow();
        max7456Brightness(0, 3);
        osdDemoBgType(DISPLAY_BACKGROUND_BLACK); // the glow's grey comes with the flames
        fireGlowLevel = 0;
        max7456SetHudMotionOffset(0, 0);
        if (demoDisplay) {
            osdDemoSnapshotToFire(demoDisplay->cols, demoDisplay->rows);
        } else {
            memset(fireHeat, 0, sizeof(fireHeat));
        }
        max7456FillScreen(OSD_DEMO_PIXEL_OFF);
        if (demoDisplay) {
            uint8_t cols = demoDisplay->cols;
            uint8_t rows = demoDisplay->rows;
            if (cols > OSD_DEMO_FIRE_COLS) {
                cols = OSD_DEMO_FIRE_COLS;
            }
            if (rows > OSD_DEMO_FIRE_ROWS) {
                rows = OSD_DEMO_FIRE_ROWS;
            }
            osdDemoFireFindTips(cols, rows);
            osdDemoApplyFireRowBright(rows);
            osdDemoFireDrawDirty(cols, rows, true);
        }
    } else if (next == OSD_DEMO_FX_WIPE) {
        osdDemoApplyOsdmNow();
        max7456Brightness(0, 3);
        osdDemoBgType(DISPLAY_BACKGROUND_LTGRAY);
        osdDemoBg(true, OSD_DEMO_WIPE_BG_LEVEL); // the grey background at rest
        max7456SetHudMotionOffset(0, 0);
        osdDemoWipeEnter();
    } else if (next == OSD_DEMO_FX_TUNNEL) {
        tunnelRot = 0;
        tunnelAnim = 0;
        tunnelAnimDiv = 0;
        tunnelSpinDiv = 0;
        tunnelTextDiv = 0;
        tunnelScrollMx = 0;
        osdDemoApplyOsdmNow();
        max7456Brightness(0, 3);
        osdDemoBgType(DISPLAY_BACKGROUND_BLACK);
        max7456SetHudMotionOffset(0, 0);
        osdDemoTunnelEnter();
    } else if (next == OSD_DEMO_FX_PLASMA2X2) {
        osdDemoP7Enter(); // the slot engine (scenes 7 / 13)
    } else if (next == OSD_DEMO_FX_DONUT || next == OSD_DEMO_FX_MEGA || next == OSD_DEMO_FX_ROTO) {
        plasmaPhase = 0;
        // Scene 7: softest OSD insertion (5/5) — no ringing on the white / grey edges of the
        // temporal grey (a thin dark line with the default 3/3).
        osdDemoApplyOsdmNow();
        max7456Brightness(0, 3);
        // Scene 9 (test): real grey = clear pixels on the VM1 grey.
        osdDemoBgType(DISPLAY_BACKGROUND_LTGRAY); // scenes 6 / 9: real grey = clear pixels on the VM1 grey
        max7456SetHudMotionOffset(0, 0);
        osdDemoPx22MapInit(); // the 2-px set is in the font since the start
        if (demoDisplay) {
            osdDemoFillRowsGlyphFast(demoDisplay->cols, demoDisplay->rows,
                                     OSD_DEMO_PIXEL_OFF, OSD_DEMO_PIXEL_OFF);
        } else {
            max7456FillScreen(OSD_DEMO_PIXEL_OFF);
        }
        osdDemoBg(true, 7); // the grey: 49 %
        osdDemoBmTables();  // scene 13's pattern → glyph table (scenes 6, 9)
        osdDemoDonutHalves();
        // 0xFF never appears in a PX22 row (it is the auto-increment escape), so the first
        // field rewrites every cell and the mirror is exact from then on.
        memset(mgSram, 0xFF, sizeof(mgSram));
        mgLastEdgeTicks = 0;
        // First frame into buffer 0, then the interrupt engine takes over.
        plasmaPlay = 0;
        plasmaNextReady = false;
        const bool donut = next == OSD_DEMO_FX_DONUT;
        const bool mega = next == OSD_DEMO_FX_MEGA;
        osdDemoMgBuildLevelLut();
        memset(plasmaRowLevel, 3, sizeof(plasmaRowLevel));
        memset(plasmaRbWritten, 0xFF, sizeof(plasmaRbWritten));
        plasmaRbNext = 0;
        if (donut) {
            memset(donutRbWritten, 0xFF, sizeof(donutRbWritten));
            donutRbNext = 0;
            donutSnareFields = 0;
            donutBassQ8 = 0;
            donutBassPrevQ8 = 0;
            donutHumPhase = 0;
            donutBgBlack = false;
            donutSnareSeen = chiptuneGetSnarePulse(); // no flash for a snare before the scene
        }
        if (demoDisplay) {
            const uint8_t c = MIN(demoDisplay->cols, OSD_DEMO_CHECKER_COLS);
            const uint8_t r = MIN(demoDisplay->rows, OSD_DEMO_CHECKER_ROWS);
            if (donut) {
                osdDemoDonutBuildFrame(0, c, r);
            } else if (next == OSD_DEMO_FX_ROTO) {
                osdDemoRotoBuildFrame(0);
                rotoGreyLevel = 7; // the enter's grey
            } else if (mega) {
                osdDemoMegaBuildFrame(0, 0, 0, 0, OSD_DEMO_MEGA_NO_SPRITE);
                megaBuiltA = megaBuiltB = 0;
                megaBuiltMix = 0;
                megaBuiltX = OSD_DEMO_MEGA_NO_SPRITE;
            } else {
                osdDemoPlasma2x2BuildFrame(0, c, r);
            }
        }
        const osdDemoIrqScene_t *sc = donut ? &osdDemoDonutScene
            : (mega ? &osdDemoMegaScene : ((next == OSD_DEMO_FX_ROTO) ? &osdDemoRotoScene : &osdDemoPlasmaScene));
        (void)osdDemoIrqStart(sc);
    } else if (next == OSD_DEMO_FX_TWISTER) {
        osdDemoApplyOsdmNow();
        max7456Brightness(0, 3);
        osdDemoBgType(DISPLAY_BACKGROUND_LTGRAY); // the grey face: clear pixels
        max7456SetHudMotionOffset(0, 0);
        osdDemoPaintTwister();
        osdDemoBg(true, 7);
        max7456RefreshAll();
        // RefreshAll leaves every twister cell black — seed the SRAM mirror to match.
        memset(mgSram, OSD_DEMO_TWISTER_GLYPH_B, sizeof(mgSram));
        mgLastEdgeTicks = 0;
        twisterPhase = 0;
        twisterBendPhase = 0;
        twisterBendFloat = 0;
        max7456WriteHosSigned(0);
        if (demoDisplay) {
            osdDemoTwisterSetupGeometry(MIN(demoDisplay->cols, OSD_DEMO_CHARS_PER_LINE),
                                        MIN(demoDisplay->rows, OSD_DEMO_MG_ROWS_MAX));
        }
        osdDemoTwisterStarsInit();
        osdDemoTwisterStarsProject(false);
        {
            const uint8_t n = demoDisplay ? MIN(demoDisplay->rows, OSD_DEMO_MG_ROWS_MAX) : OSD_DEMO_MG_ROWS_MAX;
            for (uint8_t y = 0; y < OSD_DEMO_MG_ROWS_MAX; y++) {
                const uint8_t edge = (y < n) ? MIN(y, (uint8_t)(n - 1u - y)) : 0u;
                twRowWhite[y] = (edge >= 2u) ? 3u : edge; // 0 = 80 %, 1 = 90 %, 3 = 120 %
            }
            twRbAll = true;
            twRbNext = 0;
        }
        (void)osdDemoIrqStart(&osdDemoTwisterScene);
    } else if (next == OSD_DEMO_FX_CUBE) {
        osdDemoCubeEnter();
    } else if (next == OSD_DEMO_FX_BITMAP) {
        osdDemoBmEnter();
    } else if (next == OSD_DEMO_FX_DISARM) {
        osdDemoDisarmEnter();
    } else if (next == OSD_DEMO_FX_SHOUT) {
        osdDemoShoutEnter();
        (void)osdDemoIrqStart(&osdDemoShoutScene);
    } else {
        bounceStartMs = millis();
        prevBouncePhase = 0;
        scrollCol = 0;
        osdDemoApplyOsdmNow();
        // Opaque background: transparent pixels never show video through the scene.
        osdDemoBgType(DISPLAY_BACKGROUND_BLACK);
        max7456SetHudMotionOffset(0, 0);
        osdDemoS1Reset();
        // Text waits for the next bounce peak (after the intro on the first run).
        s1TextOn = false;
        s1TextArmed = !s1Timeline;
        s1StarScale = s1Timeline ? 0u : 256u;
        s1StarsShown = s1Timeline ? 0u : 255u;
        s1Shimmer = !s1Timeline;
        s1LeadIn = 0;
        // HOS parked mid-range (offset 0) while there is no text: full-width intro titles
        // stay centred on the glass.
        hosSteps = osdDemoHosSteps();
        scrollFine = (uint8_t)(hosSteps / 2u);
        memset(starAcc, 0, sizeof(starAcc));
        max7456FillScreen(OSD_DEMO_PIXEL_OFF);
        osdDemoPaintScroller();
    }
    // First frame of every scene: same vsync+SPI drain as the steady Update path.
    // Mid-glyph scenes own the chip directly — skip shadow flush.
    if (!osdDemoFxIsMidGlyph(next)) {
        hosWrappedThisStep = false;
        osdDemoSyncFlush(true, true);
    }
}

// Scene 2's snake: at every new section of the track a snake of dither glyphs flies across on a
// helix around the plasma band — in front of it over the plasma, behind it seen only in the star
// rows above and below — its head white, its tail darker down the ladder. The first flies at the
// scene's start, each new section sends one more, each on a helix of its own period and a little
// later than the one before (the same period kept four of them together to the end). (A black channel around it over the plasma made it stand out, but looked worse.)
#define OSD_DEMO_PLASMA_SNAKE_MS    2600 // across the screen
#define OSD_DEMO_PLASMA_SNAKE_LEN   14   // cells
#define OSD_DEMO_PLASMA_SNAKE_TURNS 2.5f // turns of the helix over the pass
#define OSD_DEMO_PLASMA_SNAKE_GAP   0.010f // pass share between two cells of the body
#define OSD_DEMO_PLASMA_SNAKES_MAX  4
#define OSD_DEMO_PLASMA_SNAKE_SPREAD 0.22f // each next one this much more turns (its own period)
#define OSD_DEMO_PLASMA_SNAKE_DELAY_MS 300 // and this much later

static void osdDemoPlasmaSnake(uint8_t cols, uint8_t y0, uint8_t y1)
{
    chiptuneClock_t ck;
    if (chiptuneGetClock(&ck) && (int16_t)ck.order != plasmaSnakeOrder) {
        plasmaSnakeCount = (plasmaSnakeOrder < 0) ? 1u : (uint8_t)MIN(plasmaSnakeCount + 1u, (unsigned)OSD_DEMO_PLASMA_SNAKES_MAX);
        plasmaSnakeOn = true;
        plasmaSnakeMs = millis();
        plasmaSnakeOrder = (int16_t)ck.order;
    }
    if (!plasmaSnakeCount) { // no music: one at the start
        plasmaSnakeCount = 1;
        plasmaSnakeOn = true;
        plasmaSnakeMs = millis();
    }
    if (!plasmaSnakeOn) {
        return;
    }
    const float tau = (float)(millis() - plasmaSnakeMs) / OSD_DEMO_PLASMA_SNAKE_MS;
    // the last one starts (count − 1) delays later
    const float delay = (float)OSD_DEMO_PLASMA_SNAKE_DELAY_MS / OSD_DEMO_PLASMA_SNAKE_MS;
    if (tau > 1.0f + OSD_DEMO_PLASMA_SNAKE_LEN * OSD_DEMO_PLASMA_SNAKE_GAP + delay * (plasmaSnakeCount - 1u)) {
        plasmaSnakeOn = false;
        return;
    }
    // The helix: x from off the left edge to off the right one, y around the band's middle,
    // reaching a row past each edge (the star rows)
    const float cy = (float)(y0 + y1 - 1) * 0.5f;
    const float amp = (float)(y1 - y0) * 0.5f + 1.0f;
    const float x0 = -(float)OSD_DEMO_PLASMA_SNAKE_LEN, span = (float)cols + 2.0f * OSD_DEMO_PLASMA_SNAKE_LEN;
    for (uint8_t j = 0; j < plasmaSnakeCount; j++) {
    // each its own turns (so they part on the way) and start, the turns shifted evenly
    const float turns = OSD_DEMO_PLASMA_SNAKE_TURNS * (1.0f + OSD_DEMO_PLASMA_SNAKE_SPREAD * j);
    int16_t lastX = INT16_MIN, lastY = INT16_MIN;
    for (uint8_t k = 0; k < OSD_DEMO_PLASMA_SNAKE_LEN; k++) {
        const float t = tau - delay * j - (float)k * OSD_DEMO_PLASMA_SNAKE_GAP;
        if (t < 0.0f || t > 1.0f) {
            continue;
        }
        float a = 2.0f * M_PIf * (turns * t + (float)j / (float)plasmaSnakeCount);
        a -= floorf(a / (2.0f * M_PIf) + 0.5f) * 2.0f * M_PIf; // −π..π
        const int16_t x = (int16_t)floorf(x0 + span * t + 0.5f);
        const int16_t y = (int16_t)floorf(cy - amp * sin_approx(a) + 0.5f);
        const bool front = cos_approx(a) > 0.0f;
        if ((x == lastX && y == lastY) || x < 0 || x >= cols || y < 0) {
            continue; // the brighter cell nearer the head keeps it
        }
        lastX = x;
        lastY = y;
        if (!front && y >= y0 && y < y1) {
            continue; // behind the plasma
        }
        // white at the head, down the ladder to its darkest at the tail
        const uint8_t level = (uint8_t)(OSD_DEMO_PLASMA_LEVELS - 1u
                                        - (k * (OSD_DEMO_PLASMA_LEVELS - 2u)) / (OSD_DEMO_PLASMA_SNAKE_LEN - 1u));
        max7456WriteChar((uint8_t)x, (uint8_t)y, osdDemoLadderGlyph(level));
    }
    }
}

// One frame of the gather; false once it is over (the hold released: the rows to their levels)
static bool osdDemoPlasmaGatherFrame(uint8_t cols, uint8_t rows, uint8_t y0, uint8_t y1)
{
    const uint32_t el = millis() - plasmaGatherMs;
    if (el >= OSD_DEMO_PLASMA_GATHER_MS) {
        plasmaGather = false;
        max7456RbHold(false, 0);
        return false;
    }
    const uint32_t t = el * 256u / OSD_DEMO_PLASMA_GATHER_MS;         // 0..255
    const uint32_t e = 256u - (256u - t) * (256u - t) / 256u;         // ease-out: fast at first
    max7456FillScreen(OSD_DEMO_PIXEL_OFF);
    const uint8_t n = MIN(rows, (uint8_t)OSD_DEMO_MG_ROWS_MAX);
    for (uint8_t y = 0; y < n; y++) {
        const int32_t target = (int32_t)y0 + ((int32_t)y * (y1 - y0) + n / 2) / n;
        const int32_t row = (int32_t)y + (((target - (int32_t)y) * (int32_t)e + 128) >> 8);
        for (uint8_t x = 0; x < cols && x < OSD_DEMO_CHARS_PER_LINE; x++) {
            const uint8_t code = plasmaGatherCode[y][x];
            if (code != OSD_DEMO_PIXEL_OFF) {
                max7456WriteCharEx(x, (uint8_t)row, code, plasmaGatherInv[y][x] != 0u);
            }
        }
    }
    // the flash: 120 % white, black 30 % → 0 over the gather
    const uint8_t black = (uint8_t)(3u - MIN(t * 4u / 256u, 3u));
    max7456RbHold(true, (uint8_t)(black << 2));
    return true;
}

static void osdDemoPaintPlasma(void)
{
    if (!demoDisplay) {
        return;
    }
    const uint8_t cols = demoDisplay->cols;
    const uint8_t rows = demoDisplay->rows;
    const uint8_t band = OSD_DEMO_PLASMA_STAR_ROWS;
    // Top stars + ≥1 plasma row + bottom stars.
    if (rows < (uint8_t)(band * 2 + 1)) {
        return;
    }
    const uint8_t plasmaY0 = band;
    const uint8_t plasmaY1 = (uint8_t)(rows - band); // exclusive
    const uint8_t plasmaH = (uint8_t)(plasmaY1 - plasmaY0);
    const uint8_t xDiv = (cols > 1) ? (uint8_t)(cols - 1) : 1;
    const uint8_t yDiv = (plasmaH > 1) ? (uint8_t)(plasmaH - 1) : 1;

    if (plasmaGather && osdDemoPlasmaGatherFrame(cols, rows, plasmaY0, plasmaY1)) {
        return;
    }
    max7456FillScreen(OSD_DEMO_PIXEL_OFF);
    // No HOS on plasma — full-width field (right-bias soft-scroll leaves a left gap).
    max7456SetHudMotionOffset(0, 0);

    // Field fade via RB white; plasma outer rows slightly darker (1st −2, 2nd −1). Written
    // by the field loop right after VSYNC, never mid-field.
    const uint8_t white = osdDemoPlasmaFadeWhite();
    const timeMs_t twinkleT = millis() - fxStartMs;
    for (uint8_t y = 0; y < rows && y < OSD_DEMO_RB_ROWS; y++) {
        uint8_t rowWhite = white;
        if (y >= plasmaY0 && y < plasmaY1) {
            const uint8_t local = (uint8_t)(y - plasmaY0);
            uint8_t drop = 0;
            if (local == 0 || local == (uint8_t)(plasmaH - 1)) {
                drop = 2; // outermost plasma rows
            } else if (plasmaH >= 3 && (local == 1 || local == (uint8_t)(plasmaH - 2))) {
                drop = 1; // next-in rows, still below center
            }
            rowWhite = (white > drop) ? (uint8_t)(white - drop) : 0;
        }
        s1RowWhite[y] = rowWhite;
    }

    // Top / bottom starfields — same rel phase as scroller (starFine keeps moving).
    for (uint8_t layer = 0; layer < OSD_DEMO_STAR_LAYERS; layer++) {
        const uint16_t rel = osdDemoStarRel(layer);
        const uint16_t cellShift = rel / OSD_DEMO_CELL_W;
        const uint8_t frame = (uint8_t)(rel % OSD_DEMO_STAR_FRAMES);
        for (uint8_t i = 0; i < OSD_DEMO_STARS_PER_LAYER; i++) {
            const uint8_t y = stars[layer][i].y;
            if (!((y < band) || (y >= (uint8_t)(rows - band)))) {
                continue;
            }
            const uint8_t tw = (uint8_t)((twinkleT / 50) + (layer * 19) + (i * 37) + y * 5);
            if ((tw & 0x0F) < 3) {
                continue;
            }
            const int sx = (int)stars[layer][i].x - (int)cellShift;
            const uint8_t x = (uint8_t)(((sx % cols) + cols) % cols);
            max7456WriteChar(x, y, osdDemoStarGlyph(stars[layer][i].size, frame));
        }
    }

    // Plasma fills the whole centre band. Same three radial blobs as osdDemoPlasmaLevel, with
    // the per-column and per-row squares hoisted out of the cell loop.
    const uint8_t t = plasmaT;
    uint8_t ax1[OSD_DEMO_CHARS_PER_LINE], ax2[OSD_DEMO_CHARS_PER_LINE], ax3[OSD_DEMO_CHARS_PER_LINE];
    const uint8_t pc = (cols < OSD_DEMO_CHARS_PER_LINE) ? cols : OSD_DEMO_CHARS_PER_LINE;
    for (uint8_t x = 0; x < pc; x++) {
        const int8_t cx = (int8_t)(((uint16_t)x * 59u) / xDiv);
        ax1[x] = osdDemoSqHi((int8_t)(cx - (int8_t)t));
        ax2[x] = osdDemoSqHi((int8_t)(cx - 54));
        ax3[x] = osdDemoSqHi((int8_t)(cx - 97));
    }
    for (uint8_t y = plasmaY0; y < plasmaY1; y++) {
        const uint8_t local = (uint8_t)(y - plasmaY0);
        const int8_t cy = (int8_t)(((uint16_t)local * 31u) / yDiv);
        const uint8_t by1 = osdDemoSqHi((int8_t)(cy - 14));
        const uint8_t by2 = osdDemoSqHi((int8_t)(cy - (int8_t)t));
        const uint8_t by3 = osdDemoSqHi((int8_t)(cy - 16));
        for (uint8_t x = 0; x < pc; x++) {
            int16_t sum = (int16_t)(osdDemoSineAt((uint8_t)((uint8_t)(ax1[x] + by1) << 2)) >> 2);
            sum = (int16_t)(sum + (osdDemoSineAt((uint8_t)((uint8_t)(ax2[x] + by2) << 3)) >> 2));
            sum = (int16_t)(sum + (osdDemoSineAt((uint8_t)((uint8_t)(ax3[x] + by3) << 3)) >> 2));
            const int16_t v = (int16_t)constrain(sum + 96, 0, 189);
            const uint8_t level = (uint8_t)((v * (OSD_DEMO_PLASMA_LEVELS - 1)) / 189);
            max7456WriteChar(x, y, osdDemoLadderGlyph(level));
        }
    }

    osdDemoPlasmaSnake(pc, plasmaY0, plasmaY1);

    UNUSED(white);
}

// Intro title on screen now: index, and 256 = shown / 0 = hidden (the blink phase), or -1.
// white: its rows' white level (0 = 80 % .. 3 = 120 %, the row brightness) — the first half of
// the fade-in blinks up through 80 / 90 / 100 %, the second at 120 %; the fade-out mirrors it
// (no glyph has the letters in the real grey: 39 masks, no room in the font).
static int8_t osdDemoS1Title(uint16_t *fade, uint8_t *white)
{
    *fade = 0;
    *white = 3;
    if (!s1Timeline || s1TextOn) {
        return -1;
    }
    int8_t title = -1;
    for (uint8_t i = 0; i < OSD_DEMO_S1_TITLES; i++) {
        const uint32_t r0 = (uint32_t)s1TitleRow0[i] * 256u;
        if (s1PosQ8 >= r0 && s1PosQ8 < r0 + (uint32_t)s1TitleRows[i] * 256u) {
            title = (int8_t)i;
        }
    }
    if (title < 0) {
        return -1;
    }
    const uint32_t len = s1TitleRows[title];
    const uint32_t local = (s1PosQ8 - (uint32_t)s1TitleRow0[title] * 256u) * 64u / len; // in 64 rows
    const uint32_t in0 = OSD_DEMO_S1_FADE_IN0 * 256u, in1 = OSD_DEMO_S1_FADE_IN1 * 256u;
    const uint32_t out0 = OSD_DEMO_S1_FADE_OUT0 * 256u, out1 = OSD_DEMO_S1_FADE_OUT1 * 256u;
    bool shown;
    if (local < in0 || local >= out1) {
        shown = false;
    } else if (local < in1) {
        // Toggles so far = K·(1 − (1 − f)²): fast at first, slower, then on for good.
        const uint32_t f = (local - in0) * 256u / (in1 - in0);
        const uint32_t g = 256u - f;
        const uint32_t phi = OSD_DEMO_S1_BLINKS * (65536u - g * g) / 65536u;
        shown = (phi & 1u) != 0u;
        *white = (f < 128u) ? (uint8_t)(f * 3u / 128u) : 3u; // 80, 90, 100 %, then 120 %
    } else if (local < out0) {
        shown = true;
    } else {
        // Toggles so far = K·f²: slow at first, faster, then off.
        const uint32_t f = (local - out0) * 256u / (out1 - out0);
        const uint32_t phi = OSD_DEMO_S1_BLINKS * f * f / 65536u;
        shown = (phi & 1u) == 0u;
        *white = (f < 128u) ? 3u : (uint8_t)(2u - MIN((f - 128u) * 3u / 128u, 2u)); // 120 %, then 100, 90, 80 %
    }
    *fade = shown ? 256u : 0u;
    return title;
}

// Lay one centred title line out in mega-pixel columns, kerned (osdDemoS1CharInk): code + font
// column per mp, or 0xFF.
static void osdDemoS1LayoutLine(const char *text, uint8_t mpCols, uint8_t *code, uint8_t *fontCol)
{
    uint8_t width = 0;
    for (const char *t = text; *t; t++) {
        uint8_t c0, w;
        osdDemoS1CharInk(*t, &c0, &w);
        width = (uint8_t)(width + w + (t[1] ? 1u : 0u));
    }
    memset(fontCol, 0xFF, mpCols);
    uint8_t x = (width < mpCols) ? (uint8_t)((mpCols - width) / 2u) : 0u;
    x = (uint8_t)MIN((uint16_t)x + OSD_DEMO_S1_TITLE_SHIFT, (uint16_t)(mpCols - MIN(width, mpCols)));
    for (const char *t = text; *t && x < mpCols; t++) {
        uint8_t c0, w;
        osdDemoS1CharInk(*t, &c0, &w);
        for (uint8_t k = 0; k < w && x < mpCols; k++, x++) {
            code[x] = (uint8_t)*t;
            fontCol[x] = (uint8_t)(c0 + k);
        }
        x++; // 1-mp gap
    }
}

// Title cell (x, row cy of the 4) of the laid-out lines: bit 2r + c = mega-pixel (column c, row r)
static uint8_t osdDemoS1TitleCellMask(uint8_t code[2][OSD_DEMO_CHARS_PER_LINE * 2],
                                      uint8_t fontCol[2][OSD_DEMO_CHARS_PER_LINE * 2], uint8_t x, uint8_t cy)
{
    uint8_t mask = 0;
    for (uint8_t r = 0; r < 3u; r++) {
        const uint8_t mpY = (uint8_t)(cy * 3u + r);
        const int8_t line = (mpY < 5u) ? 0 : ((mpY >= 7u && mpY < 12u) ? 1 : -1);
        if (line < 0) {
            continue;
        }
        const uint8_t fr = (uint8_t)(line ? mpY - 7u : mpY);
        for (uint8_t c = 0; c < 2u; c++) {
            const uint8_t mpX = (uint8_t)(x * 2u + c);
            const uint8_t fc = fontCol[line][mpX];
            if (fc != 0xFFu && osdDemoFontPixel(code[line][mpX], fc, (uint8_t)(OSD_DEMO_TUNNEL_INK_Y0 + fr))) {
                mask |= (uint8_t)(1u << (2u * r + c));
            }
        }
    }
    return mask;
}

// Mask → glyph | INV << 8 (0: none). A whole-height column pair is a 2-px column glyph; the
// other masks the titles use get the slots (the INV of 63 − m for m ≥ 32): 27 of the 64.
static uint16_t s1MaskGlyph[64];

static uint16_t osdDemoS1ColumnsGlyph(uint8_t mask) // the left / right 6 px white where any row is
{
    return osdDemoMegaBw((uint8_t)(((mask & 0x15u) ? 0x38u : 0u) | ((mask & 0x2Au) ? 0x07u : 0u)));
}

static void osdDemoS1AddMask(uint8_t mask)
{
    if (!mask || s1MaskGlyph[mask]) {
        return;
    }
    const uint8_t c0 = mask & 3u;
    if (c0 == ((mask >> 2) & 3u) && c0 == ((mask >> 4) & 3u)) {
        s1MaskGlyph[mask] = osdDemoS1ColumnsGlyph(mask); // the same in all three rows
        return;
    }
    // Whole dot rows from the bottom / the top: the fills (k black rows on top, white below; INV)
    switch (mask) {
    case 0x30u: s1MaskGlyph[mask] = (uint16_t)(OSD_DEMO_FILL_BASE + 12u); return;          // bottom row
    case 0x3Cu: s1MaskGlyph[mask] = (uint16_t)(OSD_DEMO_FILL_BASE + 6u); return;           // bottom two
    case 0x03u: s1MaskGlyph[mask] = (uint16_t)((OSD_DEMO_FILL_BASE + 6u) | 0x100u); return; // top row
    case 0x0Fu: s1MaskGlyph[mask] = (uint16_t)((OSD_DEMO_FILL_BASE + 12u) | 0x100u); return; // top two
    default: break;
    }
    const uint8_t m = (mask & 0x20u) ? (uint8_t)(63u - mask) : mask;
    uint8_t slot = 0;
    while (slot < s1MpCount && s1MpMask[slot] != m) {
        slot++;
    }
    if (slot == s1MpCount) {
        if (s1MpCount < OSD_DEMO_S1_MP_SLOTS) {
            s1MpMask[s1MpCount++] = m;
        }
    }
    if (slot < s1MpCount) {
        s1MaskGlyph[mask] = (uint16_t)((OSD_DEMO_S1_MP_BASE + slot) | ((mask & 0x20u) ? 0x100u : 0u));
    }
}

static uint16_t osdDemoS1MaskCode(uint8_t mask)
{
    return s1MaskGlyph[mask] ? s1MaskGlyph[mask] : osdDemoS1ColumnsGlyph(mask);
}

// The 6×6-dot masks the texts use: scene 1's titles and scene 3's scroller (every step of it)
static void osdDemoS1TitleGlyphs(void)
{
    memset(s1MaskGlyph, 0, sizeof(s1MaskGlyph));
    s1MpCount = 0;
    osdDemoFireTextLayout();
    for (uint8_t top = 0; top < 3u; top++) { // the three dot alignments in a cell (the hop)
        for (uint16_t pos = 0; pos <= OSD_DEMO_FIRE_TEXT_MPS + fireTextW; pos++) {
            for (uint8_t y = 0; y < 3u; y++) {
                for (uint8_t x = 0; x < OSD_DEMO_FIRE_TEXT_MPS / 2u; x++) {
                    osdDemoS1AddMask(osdDemoFireTextMask(x, y, pos, top));
                }
            }
        }
    }
    const uint8_t mpCols = OSD_DEMO_CHARS_PER_LINE * 2u;
    for (uint8_t t = 0; t < OSD_DEMO_S1_TITLES; t++) {
        uint8_t code[2][OSD_DEMO_CHARS_PER_LINE * 2];
        uint8_t fontCol[2][OSD_DEMO_CHARS_PER_LINE * 2];
        for (uint8_t l = 0; l < 2u; l++) {
            osdDemoS1LayoutLine(s1Titles[t][l], mpCols, code[l], fontCol[l]);
        }
        for (uint8_t cy = 0; cy < 4u; cy++) {
            for (uint8_t x = 0; x < OSD_DEMO_CHARS_PER_LINE; x++) {
                osdDemoS1AddMask(osdDemoS1TitleCellMask(code, fontCol, x, cy));
            }
        }
    }
}

static void osdDemoS1PaintTitle(uint8_t cols, uint8_t rows)
{
    uint16_t fade = 0;
    uint8_t white;
    const int8_t title = osdDemoS1Title(&fade, &white);
    if (title < 0 || fade == 0u) {
        return;
    }
    const uint8_t mpCols = (uint8_t)MIN((uint16_t)cols * 2u, (uint16_t)(OSD_DEMO_CHARS_PER_LINE * 2));
    uint8_t code[2][OSD_DEMO_CHARS_PER_LINE * 2];
    uint8_t fontCol[2][OSD_DEMO_CHARS_PER_LINE * 2];
    for (uint8_t l = 0; l < 2u; l++) {
        osdDemoS1LayoutLine(s1Titles[title][l], mpCols, code[l], fontCol[l]);
    }
    for (uint8_t cy = 0; cy < 4u; cy++) {
        const uint8_t y = (uint8_t)(OSD_DEMO_S1_TITLE_ROW0 + cy);
        if (y >= rows) {
            break;
        }
        for (uint8_t x = 0; x < cols && (uint8_t)(x * 2u) < mpCols; x++) {
            const uint8_t mask = osdDemoS1TitleCellMask(code, fontCol, x, cy);
            if (!mask) {
                continue;
            }
            // a mask the install did not see (layout for fewer columns): its columns
            const uint16_t g = osdDemoS1MaskCode(mask);
            if (g >> 8) {
                max7456WriteCharEx(x, y, (uint8_t)g, true);
            } else {
                max7456WriteChar(x, y, (uint8_t)g);
            }
        }
    }
}

static void osdDemoPaintScroller(void)
{
    if (!demoDisplay) {
        return;
    }

    const uint8_t cols = demoDisplay->cols;
    const uint8_t rows = demoDisplay->rows;
    const uint8_t drawH = (rows < OSD_DEMO_FONT_H) ? rows : OSD_DEMO_FONT_H;

    uint8_t floorY = (rows > drawH + 1) ? (uint8_t)(rows - drawH - 1) : 0;
    if (floorY + drawH > rows) {
        floorY = (rows > drawH) ? (uint8_t)(rows - drawH) : 0;
    }
    uint16_t hopCells = OSD_DEMO_HOP_CELLS;
    if (hopCells > floorY) {
        hopCells = floorY;
    }
    const uint16_t hopFine = (uint16_t)(hopCells * OSD_DEMO_CELL_H);
    const uint16_t phase = osdDemoBouncePhase();
    const uint16_t fine = osdDemoBounceFine(hopFine, phase);
    if (s1TextOn) {
        osdDemoUpdatePunch(fine, phase); // no punch before the text lands
    }
    const uint8_t liftRows = (uint8_t)(fine / OSD_DEMO_CELL_H);
    const uint8_t soft = (uint8_t)(fine % OSD_DEMO_CELL_H);
    const uint8_t y0 = (uint8_t)(floorY - liftRows);

    osdDemoApplyHudMotion();
    // Intro: every row is starfield (its own brightness wave).
    osdDemoApplyRowShimmer(y0, s1TextOn ? drawH : 0u, rows);
    if (s1TextOn && !s1Shimmer) {
        for (uint8_t r = y0; r < (uint8_t)(y0 + drawH) && r < OSD_DEMO_RB_ROWS; r++) {
            s1RowWhite[r] = 3; // flat full-bright text until the ridge section
        }
    }
    {
        uint16_t fade = 0;
        uint8_t white;
        const int8_t title = osdDemoS1Title(&fade, &white);
        if (title >= 0) { // the title rows at the blink's level, each line capped at its own
            UNUSED(fade);
            for (uint8_t r = OSD_DEMO_S1_TITLE_ROW0; r < OSD_DEMO_S1_TITLE_ROW0 + 4u && r < OSD_DEMO_RB_ROWS; r++) {
                const uint8_t line = (r < OSD_DEMO_S1_TITLE_ROW0 + 2u) ? 0u : 1u; // rows 0-1 / 2-3
                s1RowWhite[r] = MIN(white, s1TitleLineWhite[title][line]);
            }
        }
    }

    max7456FillScreen(OSD_DEMO_PIXEL_OFF);

    // Stars: HOS carries the shared fine phase; glyph frame + cell hold the rest
    // so total leftward motion = starFine (parallax), continuous across HOS wrap.
    // Twinkle: briefly skip drawing so stars flicker against the row-brightness wave.
    const timeMs_t twinkleT = millis() - bounceStartMs;
    for (uint8_t layer = 0; layer < OSD_DEMO_STAR_LAYERS; layer++) {
        const uint16_t rel = osdDemoStarRel(layer);
        const uint16_t cellShift = rel / OSD_DEMO_CELL_W;
        const uint8_t frame = (uint8_t)(rel % OSD_DEMO_STAR_FRAMES);
        for (uint8_t i = 0; i < OSD_DEMO_STARS_PER_LAYER; i++) {
            const uint8_t y = stars[layer][i].y;
            // Stars also fill the scroller band: the text pass below overwrites only its lit
            // and soft-edge cells, so stars stay in the black cells between the letters.
            // Per-star twinkle phase — mostly on, short off blinks.
            const uint8_t tw = (uint8_t)((twinkleT / 50) + (layer * 19) + (i * 37) + y * 5);
            if ((tw & 0x0F) < 3) {
                continue;
            }
            // Intro: stars appear one at a time (layers interleaved).
            if ((uint16_t)(i * OSD_DEMO_STAR_LAYERS + layer) >= s1StarsShown) {
                continue;
            }
            const int sx = (int)stars[layer][i].x - (int)cellShift;
            const uint8_t x = (uint8_t)(((sx % cols) + cols) % cols);
            if (y < rows) {
                max7456WriteChar(x, y, osdDemoStarGlyph(stars[layer][i].size, frame));
            }
        }
    }

    osdDemoS1PaintTitle(cols, rows);

    for (uint8_t x = 0; x < cols && s1TextOn; x++) {
        const int16_t pc = (int16_t)(scrollCol + x) - (int16_t)s1LeadIn;
        if (pc < 0) {
            continue; // blank lead-in: the text enters from the right edge
        }
        const uint16_t pixelCol = (uint16_t)pc;
        for (uint8_t py = 0; py < drawH; py++) {
            if (!osdDemoTextOn(pixelCol, py)) {
                continue;
            }
            const uint8_t sy = (uint8_t)(y0 + py);
            if (soft == 0) {
                // Full white cell (fill offset 0).
                osdDemoWriteCell(x, sy, rows, OSD_DEMO_FILL_BASE, false);
                continue;
            }
            // Glyph g: white on rows [gOff..17]. INV flips to white on [0..gOff).
            // Shift-UP by `soft`: remainder = INV(g), spill above = g, with gOff = 18-soft.
            const uint8_t g = (uint8_t)(OSD_DEMO_FILL_BASE + (OSD_DEMO_SOFT_STEPS - soft));
            osdDemoWriteCell(x, sy, rows, g, true);
            if (sy > 0) {
                if (py > 0 && osdDemoTextOn(pixelCol, (uint8_t)(py - 1))) {
                    osdDemoWriteCell(x, (uint8_t)(sy - 1), rows, OSD_DEMO_FILL_BASE, false);
                } else {
                    osdDemoWriteCell(x, (uint8_t)(sy - 1), rows, g, false);
                }
            }
        }
    }
}



// --- Scene 1 loop: one step per video field --------------------------------------------------
// Paint just before the predicted VSYNC, wait for the edge (short), then brightness, cells and
// HOS in one fast row-ordered burst inside VBLANK. The OSD task calls this at its own rate:
// return early while the next call still lands before the paint window, otherwise paint now
// and block until the edge (the old behaviour, every call).
#define OSD_DEMO_S1_LEAD_US    2500  // paint + margin before the predicted VSYNC
#define OSD_DEMO_S1_FIELD_US   20000

static bool s1Locked;
static uint32_t s1VsyncTicks;
static uint32_t s1FieldTicks;
static uint32_t s1LastCallTicks;
static uint32_t s1CallGapTicks;  // natural OSD-task call interval (0 = not seen yet)
static bool s1PrevEarly;         // the previous call returned without blocking
static uint64_t s1WaitTicks, s1FlushTicks;
static uint32_t s1WaitMax, s1FlushMax, s1FlushEndMax, s1BytesMax;
static uint32_t s1Bytes, s1Calls, s1Fields, s1Missed, s1Timeouts;

static void osdDemoS1ResetStats(void)
{
    s1WaitTicks = s1FlushTicks = 0;
    s1WaitMax = s1FlushMax = s1FlushEndMax = s1BytesMax = 0;
    s1Bytes = s1Calls = s1Fields = s1Missed = s1Timeouts = 0;
}

static void osdDemoS1Reset(void)
{
    s1Locked = false;
    s1LastCallTicks = 0;
    s1CallGapTicks = 0;
    s1PrevEarly = false;
    memset(s1RowWritten, 0xFF, sizeof(s1RowWritten));
    osdDemoS1ResetStats();
}

static void osdDemoWaitCycles(uint32_t deadlineTicks)
{
    while ((int32_t)(getCycleCounter() - deadlineTicks) < 0) {
    }
}



// --- Intro: Betaflight's own OSD over the camera, eaten by a matrix rain ------------------
// The demo opens on the stock OSD look — horizon (the board's attitude, a slow sway on top),
// crosshairs, side bars, battery, RSSI, GPS with the party's coordinates, timers and a
// blinking DISARMED — in Betaflight's own font over the camera (camera sync). Then rain falls
// in the columns that hold OSD symbols, later in every column: a white head of changing
// symbols that takes the OSD symbol it passes down with it, a trail fading through glyph
// dithers to black. When the screen is black the chip goes to its own sync and the demo's
// font is installed (black meanwhile), and the demo starts.
// The intro's own glyphs (black-backed copies of the symbols, their 50 % / 25 % dithers) go to
// 0xA0..0xFE — Betaflight's logo, not shown here — made from the font itself after its backup,
// the all-black one to PIXEL_OFF; the stop restores the font as always.
#define OSD_DEMO_INTRO_OSD_MS      6000 // the stock OSD alone
#define OSD_DEMO_INTRO_STEP_MS     40   // 25 steps a second
#define OSD_DEMO_INTRO_GLYPH0      0xA0
#define OSD_DEMO_INTRO_GLYPH_END   0xFF // exclusive (0xFF: the auto-increment escape)
#define OSD_DEMO_INTRO_FADE_STEPS  3    // a trail cell's intensity drops a level every this many steps
#define OSD_DEMO_INTRO_ROWS        16
// ZOO party 2026: Oriveden Kampus, Koulutie 5, 35300 Orivesi, Finland (OpenStreetMap)
#define OSD_DEMO_INTRO_LAT         "61.6814178"
#define OSD_DEMO_INTRO_LON         "24.3625163"

static bool introEnabled = true;
static bool introSkip;           // osd_demo sceneN: straight to the scene
static uint8_t introPhase;       // 0 off, 1 stock OSD, 2 rain
static timeMs_t introPhaseMs, introStepMs;
static uint16_t introStep;       // rain steps
static uint8_t introOsd[OSD_DEMO_INTRO_ROWS][OSD_DEMO_CHARS_PER_LINE];
static uint8_t introAge[OSD_DEMO_INTRO_ROWS][OSD_DEMO_CHARS_PER_LINE]; // 0xFF untouched, 3..1 trail, 0 black
static uint8_t introSym[OSD_DEMO_INTRO_ROWS][OSD_DEMO_CHARS_PER_LINE];
static uint8_t introSolid[256];  // code → its black-backed copy (0: none)
static const char introSyms[] = "0123456789Z$#+"; // the rain's symbols
#define OSD_DEMO_INTRO_SYMS (sizeof(introSyms) - 1u)
static uint8_t introDither[OSD_DEMO_INTRO_SYMS][2]; // 50 %, 25 % of the symbol's white
static uint8_t introBlack;
typedef struct {
    int16_t yQ8;     // head row << 8 (negative: not started yet)
    uint8_t speedQ8; // rows a step << 8
    uint8_t carry;   // OSD symbol taken down (0: none)
    uint16_t start;  // step it starts falling
    bool done;
} osdDemoIntroCol_t;
static osdDemoIntroCol_t introCol[OSD_DEMO_CHARS_PER_LINE];

static void osdDemoStartShow(void);

// Song position in rows (Q8), 64-row patterns; no music → the nominal row clock.
static uint32_t osdDemoSongRowQ8(void)
{
    uint32_t rowQ8;
    uint8_t speed;
    if (osdDemoSongClockQ8(&rowQ8, &speed)) {
        return rowQ8;
    }
    return (uint32_t)(millis() - s1TimelineMs) * 256u / OSD_DEMO_S1_ROW_MS;
}

// Scene 1 timeline, once per step before advance / paint.
static void osdDemoS1Timeline(void)
{
    if (s1Timeline) {
        const uint32_t songQ8 = osdDemoSongRowQ8();
        const uint32_t posQ8 = (songQ8 > s1BaseRowQ8) ? songQ8 - s1BaseRowQ8 : 0u;
        const uint32_t bass = OSD_DEMO_S1_BASS_ROW * 256u;
        const uint32_t kick = OSD_DEMO_S1_SCROLL_ROW * 256u;
        s1PosQ8 = posQ8;
        const uint32_t starsEnd = OSD_DEMO_S1_STARS_ROWS * 256u;
        s1StarsShown = (posQ8 >= starsEnd) ? 255u
            : (uint8_t)(1u + posQ8 * (OSD_DEMO_STAR_LAYERS * OSD_DEMO_STARS_PER_LAYER) / starsEnd);
        if (posQ8 < bass) {
            s1StarScale = 0;
        } else if (posQ8 < kick) {
            const uint32_t p = (posQ8 - bass) * 256u / (kick - bass);
            s1StarScale = (uint16_t)((p + p * p / 256u) / 2u); // half linear, half ease-in
        } else {
            s1StarScale = 256;
        }
        s1Shimmer = posQ8 >= OSD_DEMO_S1_SHIMMER_ROW * 256u;
        // Arm the text one bounce period before the kick: it shows at the next peak and its
        // fall ends on the kick (the bounce period starts on the music-locked line grid).
        uint32_t armRows = 16;
        chiptuneClock_t ck;
        if (chiptuneGetClock(&ck)) {
            armRows = osdDemoMusicLines(ck.speed, OSD_DEMO_BOUNCE_PERIOD_MS);
        }
        if (posQ8 + armRows * 256u >= kick) {
            s1TextArmed = true;
        }
    }
    if (!s1TextOn && s1TextArmed) {
        const uint16_t phase = osdDemoBouncePhase();
        if (phase >= 128u && phase < 192u) { // just past the top of the hop, falling
            s1TextOn = true;
            scrollCol = 0;                 // scrollFine kept: HOS (and the stars) do not jump
            s1LeadIn = demoDisplay ? demoDisplay->cols : OSD_DEMO_CHARS_PER_LINE;
            punchY = 0;
            punchVel = 0;
            punchArmed = false;            // re-arms in the air; the first floor hit punches
            prevBouncePhase = phase;
            fxStartMs = millis();          // free-running scene length counts from the text
        }
    }
}

static void osdDemoScrollerUpdate(void)
{
    const uint32_t cyc = clockMicrosToCycles(1);
    const uint32_t now = getCycleCounter();
    s1Calls++;
    // Learn the task's own call interval only after a call that returned at once: after a
    // blocking call the next one comes ~a field later because of our wait, and counting that
    // made the loop believe calls are sparse and block every field (≈ 18 ms busy wait).
    if (s1LastCallTicks && s1PrevEarly) {
        const uint32_t gap = now - s1LastCallTicks;
        s1CallGapTicks = s1CallGapTicks ? (s1CallGapTicks * 7u + gap) / 8u : gap;
    }
    s1LastCallTicks = now;
    s1PrevEarly = false;
    if (s1FieldTicks == 0) {
        s1FieldTicks = OSD_DEMO_S1_FIELD_US * cyc;
    }

    uint32_t timeoutUs = OSD_DEMO_S1_FIELD_US + 5000;
    if (s1Locked) {
        const uint32_t sinceV = now - s1VsyncTicks;
        const uint32_t toNext = s1FieldTicks - (sinceV % s1FieldTicks);
        const uint32_t lead = OSD_DEMO_S1_LEAD_US * cyc;
        if (toNext > lead && s1CallGapTicks + lead < toNext) {
            s1PrevEarly = true;
            return; // the next call still comes before the paint window
        }
        timeoutUs = toNext / cyc + 3000;
    }

    osdDemoAdvanceHosAndStars();
    if (fx == OSD_DEMO_FX_PLASMA) {
        // Phase by the clock (one step per field at 50 Hz), not by how often we get called.
        plasmaT = (uint8_t)(((uint64_t)(millis() - fxStartMs) * OSD_DEMO_PLASMA_T_HZ) / 1000u);
        osdDemoPaintPlasma();
    } else {
        osdDemoS1Timeline();
        osdDemoPaintScroller();
    }

    uint32_t edge = 0;
    const uint32_t w0 = getCycleCounter();
    if (!max7456WaitVsyncFallingEdge(&edge, timeoutUs)) {
        s1Timeouts++;
        s1Locked = false;
        return;
    }
    const uint32_t wait = edge - w0;
    if (s1Locked) {
        const uint32_t period = edge - s1VsyncTicks;
        const uint32_t k = (period + s1FieldTicks / 2u) / s1FieldTicks; // fields since the last step
        if (k > 1u) {
            s1Missed += k - 1u;
        }
        if (k >= 1u) {
            const uint32_t per = period / k;
            const uint32_t nominal = OSD_DEMO_S1_FIELD_US * cyc;
            if (per > nominal - nominal / 20u && per < nominal + nominal / 20u) {
                s1FieldTicks = (s1FieldTicks * 7u + per) / 8u;
            }
        }
    }
    s1VsyncTicks = edge;
    s1Locked = true;

    // VBLANK: brightness, then every changed cell top row first, then HOS (wrap lands with
    // the shifted cells in the same field).
    osdDemoS1WriteRowBrightness(demoDisplay->rows);
    if (osdmApplied != osdDemoOsdmWanted()) {
        osdDemoApplyOsdmNow();
    }
    const uint32_t f0 = getCycleCounter();
    const uint16_t bytes = max7456FlushDirtyBurst();
    const uint32_t f1 = getCycleCounter();

    s1Fields++;
    s1WaitTicks += wait;
    s1FlushTicks += f1 - f0;
    s1Bytes += bytes;
    s1WaitMax = MAX(s1WaitMax, wait);
    s1FlushMax = MAX(s1FlushMax, f1 - f0);
    s1FlushEndMax = MAX(s1FlushEndMax, f1 - edge);
    s1BytesMax = MAX(s1BytesMax, (uint32_t)bytes);
    UNUSED(cyc);
}

// Betaflight's glyph `code` (read from the backup) → a copy on black: transparent → black,
// white kept where a 4×4 ordered dither passes `keep16` of 16.
static bool osdDemoIntroMake(uint8_t addr, uint8_t code, uint8_t keep16)
{
    static const uint8_t bayer[16] = { 0, 8, 2, 10, 12, 4, 14, 6, 3, 11, 1, 9, 15, 7, 13, 5 };
    uint8_t src[OSD_DEMO_GLYPH_BYTES], dst[OSD_DEMO_GLYPH_BYTES];
    if (!max7456NvmRead9((uint16_t)(OSD_DEMO_FONT_BACKUP + code), src)) {
        return false;
    }
    for (uint8_t i = 0; i < OSD_DEMO_GLYPH_BYTES; i++) {
        uint8_t v = 0;
        for (uint8_t k = 0; k < 4u; k++) {
            const uint8_t px = (uint8_t)((src[i] >> (6u - 2u * k)) & 3u);
            const uint8_t r = (uint8_t)(i / 3u), c = (uint8_t)((i % 3u) * 4u + k);
            uint8_t out = OSD_DEMO_PX_B;
            if (px == OSD_DEMO_PX_W && bayer[((r & 3u) << 2) | (c & 3u)] < keep16) {
                out = OSD_DEMO_PX_W;
            }
            v = (uint8_t)((v << 2) | out);
        }
        dst[i] = v;
    }
    return osdDemoNvmPut(addr, dst);
}

// The codes the stock OSD here can show (each gets a black-backed copy for the rain to carry).
static const uint8_t introOsdCodes[] = {
    SYM_RSSI, SYM_BATT_5, SYM_VOLT, SYM_MAH, SYM_AH_CENTER_LINE, SYM_AH_CENTER, SYM_AH_CENTER_LINE_RIGHT,
    SYM_AH_DECORATION, SYM_AH_LEFT, SYM_AH_RIGHT, SYM_SAT_L, SYM_SAT_R, SYM_LAT, SYM_LON, SYM_ALTITUDE,
    SYM_M, SYM_SPEED, SYM_KPH, SYM_ARROW_NORTH, SYM_ON_M, SYM_FLY_M, '.', ':',
    'D', 'I', 'S', 'A', 'R', 'M', 'E',
};

static bool osdDemoIntroGlyphs(void)
{
    memset(introSolid, 0, sizeof(introSolid));
    uint8_t slot = OSD_DEMO_INTRO_GLYPH0;
    bool ok = true;
    // All black at the demo's own black (PIXEL_OFF — Betaflight's full battery, not shown here):
    // the same glyph after the demo font goes in, so the black screen stays black through it
    {
        uint8_t nvm[OSD_DEMO_GLYPH_BYTES];
        memset(nvm, 0, sizeof(nvm)); // 00 = black
        introBlack = OSD_DEMO_PIXEL_OFF;
        ok = osdDemoNvmPut(introBlack, nvm);
    }
    for (uint8_t i = 0; ok && i < OSD_DEMO_INTRO_SYMS; i++) {
        const uint8_t code = (uint8_t)introSyms[i];
        introSolid[code] = slot;
        ok = osdDemoIntroMake(slot++, code, 16);
        introDither[i][0] = slot;
        ok = ok && osdDemoIntroMake(slot++, code, 8);
        introDither[i][1] = slot;
        ok = ok && osdDemoIntroMake(slot++, code, 4);
    }
    for (uint8_t i = 0; ok && i < ARRAYLEN(introOsdCodes) && slot < OSD_DEMO_INTRO_GLYPH_END; i++) {
        const uint8_t code = introOsdCodes[i];
        if (!introSolid[code]) {
            introSolid[code] = slot;
            ok = osdDemoIntroMake(slot++, code, 16);
        }
    }
    for (uint8_t a = 0; ok && a < 9u && slot < OSD_DEMO_INTRO_GLYPH_END; a++) { // the horizon bars
        introSolid[SYM_AH_BAR9_0 + a] = slot;
        ok = osdDemoIntroMake(slot++, (uint8_t)(SYM_AH_BAR9_0 + a), 16);
    }
    return ok;
}

static void osdDemoIntroPut(uint8_t x, uint8_t y, const char *str)
{
    for (; *str && y < OSD_DEMO_INTRO_ROWS && x < OSD_DEMO_CHARS_PER_LINE; str++, x++) {
        introOsd[y][x] = (uint8_t)*str;
    }
}

static void osdDemoIntroPutChar(uint8_t x, uint8_t y, uint8_t code)
{
    if (y < OSD_DEMO_INTRO_ROWS && x < OSD_DEMO_CHARS_PER_LINE) {
        introOsd[y][x] = code;
    }
}

// The stock OSD now (Betaflight's default spots: crosshairs, horizon and side bars around the
// centre (15, 7), warnings below; the rest where pilots usually put them).
static void osdDemoIntroBuildOsd(void)
{
    memset(introOsd, SYM_BLANK, sizeof(introOsd));
    const timeMs_t t = millis() - introPhaseMs + (introPhase == 2u ? OSD_DEMO_INTRO_OSD_MS : 0u);
    char b[16];
    // RSSI, battery, mAh
    osdDemoIntroPutChar(1, 1, SYM_RSSI);
    osdDemoIntroPut(2, 1, "99");
    osdDemoIntroPutChar(22, 1, SYM_BATT_5);
    const uint16_t mv = (uint16_t)(1674u - (t / 1000u) % 3u); // a live-looking 16.74 V
    tfp_sprintf(b, "%u.%02u", (unsigned)(mv / 100u), (unsigned)(mv % 100u));
    osdDemoIntroPut(23, 1, b);
    osdDemoIntroPutChar(28, 1, SYM_VOLT);
    osdDemoIntroPut(25, 2, "0");
    osdDemoIntroPutChar(26, 2, SYM_MAH);
    // Horizon (Betaflight's formula: 20° pitch / 40° roll limits), the board's attitude + sway
    const int sway = (int)osdDemoSineAt((uint8_t)(t / 24u)) * 60 / 127;
    const int bob = (int)osdDemoSineAt((uint8_t)(t / 37u + 40u)) * 30 / 127;
    const int roll = constrain(attitude.values.roll + sway, -400, 400);
    int pitch = constrain(attitude.values.pitch + bob, -200, 200);
    pitch = (pitch * 25) / 200 - 41;
    for (int x = -4; x <= 4; x++) {
        const int y = ((-roll * x) / 64) - pitch;
        if (y >= 0 && y <= 81) {
            osdDemoIntroPutChar((uint8_t)(14 + x), (uint8_t)(2 + y / 9), (uint8_t)(SYM_AH_BAR9_0 + y % 9));
        }
    }
    for (int y = -3; y <= 3; y++) { // side bars
        osdDemoIntroPutChar(7, (uint8_t)(6 + y), SYM_AH_DECORATION);
        osdDemoIntroPutChar(21, (uint8_t)(6 + y), SYM_AH_DECORATION);
    }
    osdDemoIntroPutChar(8, 6, SYM_AH_LEFT);
    osdDemoIntroPutChar(20, 6, SYM_AH_RIGHT);
    osdDemoIntroPutChar(13, 6, SYM_AH_CENTER_LINE);
    osdDemoIntroPutChar(14, 6, SYM_AH_CENTER);
    osdDemoIntroPutChar(15, 6, SYM_AH_CENTER_LINE_RIGHT);
    // Speed, altitude
    osdDemoIntroPutChar(1, 7, SYM_SPEED);
    osdDemoIntroPut(2, 7, "0");
    osdDemoIntroPutChar(3, 7, SYM_KPH);
    osdDemoIntroPutChar(24, 7, SYM_ALTITUDE);
    osdDemoIntroPut(25, 7, "0.0");
    osdDemoIntroPutChar(28, 7, SYM_M);
    // Warning
    if ((t / 400u) & 1u) {
        osdDemoIntroPut(11, 10, "DISARMED");
    }
    // GPS, home
    osdDemoIntroPutChar(1, 12, SYM_SAT_L);
    osdDemoIntroPutChar(2, 12, SYM_SAT_R);
    osdDemoIntroPut(3, 12, "14");
    osdDemoIntroPutChar(14, 12, SYM_ARROW_NORTH);
    osdDemoIntroPutChar(1, 13, SYM_LAT);
    osdDemoIntroPut(2, 13, OSD_DEMO_INTRO_LAT);
    osdDemoIntroPutChar(1, 14, SYM_LON);
    osdDemoIntroPut(2, 14, OSD_DEMO_INTRO_LON);
    // Timers
    const uint32_t on = 42u + t / 1000u;
    osdDemoIntroPutChar(23, 13, SYM_ON_M);
    tfp_sprintf(b, "%02u:%02u", (unsigned)((on / 60u) % 100u), (unsigned)(on % 60u));
    osdDemoIntroPut(24, 13, b);
    osdDemoIntroPutChar(23, 14, SYM_FLY_M);
    osdDemoIntroPut(24, 14, "00:00");
}

static void osdDemoIntroRainStart(void)
{
    memset(introAge, 0xFF, sizeof(introAge));
    memset(introSym, 0, sizeof(introSym));
    const uint8_t cols = MIN(demoDisplay->cols, OSD_DEMO_CHARS_PER_LINE);
    for (uint8_t x = 0; x < cols; x++) {
        bool osd = false;
        for (uint8_t y = 0; y < OSD_DEMO_INTRO_ROWS; y++) {
            osd = osd || introOsd[y][x] != SYM_BLANK;
        }
        osdDemoIntroCol_t *c = &introCol[x];
        c->yQ8 = -256;
        c->speedQ8 = (uint8_t)(110u + osdDemoRand() % 120u); // 0.43 .. 0.9 rows a step
        c->carry = 0;
        c->start = (uint16_t)(osd ? osdDemoRand() % 20u : 12u + osdDemoRand() % 60u); // OSD columns first
        c->done = false;
    }
    introStep = 0;
}

// One rain step; true once the whole screen is black.
static bool osdDemoIntroRainStep(void)
{
    const uint8_t cols = MIN(demoDisplay->cols, OSD_DEMO_CHARS_PER_LINE);
    const uint8_t rows = MIN(demoDisplay->rows, OSD_DEMO_INTRO_ROWS);
    bool busy = false;
    introStep++;
    // Trails fade, now and then a trail symbol changes
    if (introStep % OSD_DEMO_INTRO_FADE_STEPS == 0u) {
        for (uint8_t y = 0; y < rows; y++) {
            for (uint8_t x = 0; x < cols; x++) {
                if (introAge[y][x] != 0xFFu && introAge[y][x] > 0u) {
                    introAge[y][x]--;
                }
            }
        }
    }
    for (uint8_t y = 0; y < rows; y++) {
        for (uint8_t x = 0; x < cols; x++) {
            if (introAge[y][x] != 0xFFu && introAge[y][x] > 0u) {
                busy = true;
                if ((osdDemoRand() & 15u) == 0u) {
                    introSym[y][x] = (uint8_t)(osdDemoRand() % OSD_DEMO_INTRO_SYMS);
                }
            }
        }
    }
    for (uint8_t x = 0; x < cols; x++) {
        osdDemoIntroCol_t *c = &introCol[x];
        if (c->done) {
            continue;
        }
        busy = true;
        if (introStep < c->start) {
            continue;
        }
        const int16_t before = (int16_t)(c->yQ8 >> 8);
        c->yQ8 = (int16_t)(c->yQ8 + c->speedQ8);
        const int16_t now = (int16_t)(c->yQ8 >> 8);
        for (int16_t y = (int16_t)MAX(before + 1, 0); y <= now && y < rows; y++) {
            // The head enters row y: it takes the OSD symbol there, the cell joins the trail
            if (introAge[y][x] == 0xFFu && introOsd[y][x] != SYM_BLANK && introSolid[introOsd[y][x]]) {
                c->carry = introOsd[y][x];
            }
            introAge[y][x] = 3;
            introSym[y][x] = (uint8_t)(osdDemoRand() % OSD_DEMO_INTRO_SYMS);
        }
        if (now >= rows) {
            c->done = true;
        }
    }
    return !busy;
}

// The screen this step: the stock OSD where the rain has not been, the rain over it.
static void osdDemoIntroDraw(void)
{
    const uint8_t cols = MIN(demoDisplay->cols, OSD_DEMO_CHARS_PER_LINE);
    const uint8_t rows = MIN(demoDisplay->rows, OSD_DEMO_INTRO_ROWS);
    for (uint8_t y = 0; y < rows; y++) {
        for (uint8_t x = 0; x < cols; x++) {
            uint8_t code = introOsd[y][x];
            const uint8_t age = (introPhase == 2u) ? introAge[y][x] : 0xFFu;
            if (age != 0xFFu) {
                const uint8_t s = introSym[y][x];
                code = (age >= 3u) ? introSolid[(uint8_t)introSyms[s]]
                     : (age == 2u) ? introDither[s][0]
                     : (age == 1u) ? introDither[s][1] : introBlack;
            }
            max7456WriteChar(x, y, code);
        }
    }
    if (introPhase == 2u) {
        for (uint8_t x = 0; x < cols; x++) { // the heads: white, changing, or the OSD symbol taken
            const osdDemoIntroCol_t *c = &introCol[x];
            const int16_t y = (int16_t)(c->yQ8 >> 8);
            if (c->done || introStep < c->start || y < 0 || y >= rows) {
                continue;
            }
            const uint8_t code = c->carry ? introSolid[c->carry]
                                          : introSolid[(uint8_t)introSyms[osdDemoRand() % OSD_DEMO_INTRO_SYMS]];
            max7456WriteChar(x, (uint8_t)y, code);
        }
    }
    osdDemoSyncFlush(true, false);
}

// Demo start with the intro: the font backed up, the intro's glyphs made, the stock OSD on.
static bool osdDemoIntroBegin(void)
{
    if (!max7456IsAt7456()) {
        return false; // no backup on a MAX7456: the logo slots must not be overwritten
    }
    max7456ResetHudMotionOffset();
    max7456SetSyncMode(0); // the camera's sync: the OSD over its picture
    if (osdDemoFontMark() == OSD_DEMO_FONT_DIRTY) {
        osdDemoFontRestore(); // a demo cut off before: the lower glyphs back to the user's font
    }
    osdDemoFontBackup();
    const bool ok = osdDemoIntroGlyphs();
    osdDemoFontWriteDone();
    if (!ok) {
        return false;
    }
    osdDemoBgType(DISPLAY_BACKGROUND_TRANSPARENT);
    max7456Brightness(displayPortProfileMax7456()->blackBrightness, displayPortProfileMax7456()->whiteBrightness);
    max7456Osdm(OSD_DEMO_OSDM_DEFAULT);
    introPhase = 1;
    introPhaseMs = millis();
    introStepMs = introPhaseMs;
    rngState = micros() ^ ((uint32_t)millis() << 16);
    osdDemoIntroBuildOsd();
    max7456FillScreen(SYM_BLANK);
    osdDemoIntroDraw();
    return true;
}

static void osdDemoIntroUpdate(void)
{
    const timeMs_t now = millis();
    if ((timeDelta_t)(now - introStepMs) < OSD_DEMO_INTRO_STEP_MS) {
        return;
    }
    introStepMs = now;
    osdDemoIntroBuildOsd();
    if (introPhase == 1u) {
        if ((timeDelta_t)(now - introPhaseMs) >= OSD_DEMO_INTRO_OSD_MS) {
            introPhase = 2;
            introPhaseMs = now;
            osdDemoIntroRainStart();
        }
        osdDemoIntroDraw();
        return;
    }
    const bool black = osdDemoIntroRainStep();
    osdDemoIntroDraw();
    if (black) {
        introPhase = 0;
        osdDemoStartShow(); // the chip's own sync, the demo font, the first scene
    }
}

bool osdDemoStart(void)
{
    startLastError = NULL;

    if (active && introPhase) {
        return true; // the intro is on: it starts the demo itself
    }
    if (active) {
        // `osd_demo` / `play` on a running demo: start over from the top (first scene of the
        // order, song from its first row). Resuming the cycle after a held scene had let the
        // song run ahead of the timeline, so every following scene sat off its patterns.
        fxHold = false;
        fxOrderIdx = 0;
        s1BaseRowQ8 = 0;
        const osdDemoFx_e first = osdDemoOrderFx(0);
        s1Timeline = first == OSD_DEMO_FX_SCROLLER;
        if (chiptuneIsPlaying() || demoMusicMuted) {
            demoMusicMuted = false;
            (void)chiptuneStart(chiptuneCurrentTrack());
        }
        s1TimelineMs = millis();
        osdDemoEnterFx(first);
        return true;
    }

    // Recover from a prior mid-glyph session that left SPI hot.
    max7456MidGlyphSpiEnd();

    // Demo needs a bound OSD displayport. Boot only creates one when FEATURE_OSD
    // is on; after a wipe / failed AUTO bind osdDisplayPort stays NULL even if
    // the chip is present. Force-enable + bind MAX7456 for this session.
    if (!featureIsEnabled(FEATURE_OSD)) {
        featureEnableImmediate(FEATURE_OSD);
    }

    osdDisplayPortDevice_e type;
    displayPort_t *dp = osdGetDisplayPort(&type);

    if (!dp) {
        displayPort_t *created = NULL;
        // DisplayPortInit returns false on NOT_FOUND but still hands back a port.
        (void)max7456DisplayPortInit(vcdProfile(), &created);
        if (!created) {
            startLastError = "MAX7456 displayport init failed (CS/SPI not configured?)";
            return false;
        }
        osdInit(created, OSD_DISPLAYPORT_DEVICE_MAX7456);
        dp = created;
    }

    // Boot may leave detected=false (OSDM probe before chip ready). Rescan via
    // the existing port — do NOT call bare max7456Init() again: CS is already
    // owned and a second Init returns NOT_CONFIGURED permanently.
    if (!max7456IsDeviceDetected()) {
        (void)displayCheckReady(dp, true);
    }
    if (!max7456IsDeviceDetected()) {
        startLastError = "MAX7456 not detected (SPI/OSDM) — check wiring / power cycle";
        return false;
    }

    demoDisplay = osdGetDisplayPort(&type);
    if (!demoDisplay) {
        startLastError = "no OSD displayport after bind";
        return false;
    }

    const uint16_t len = osdDemoTextLen();
    if (len == 0) {
        startLastError = "scroll text empty";
        return false;
    }
    textPixelCols = osdDemoS1TextLayout(); // kerned
    const uint16_t tlen = osdDemoTunnelTextLen();
    tunnelTextCols = (uint16_t)(tlen * OSD_DEMO_FONT_ADVANCE);

    if (!osdDemoHsCapDetect()) {
        startLastError = osdDemoHsCapHint();
        return false;
    }
    displayGrab(demoDisplay);
    if (introEnabled && !introSkip && osdDemoIntroBegin()) {
        active = true; // the intro runs from the OSD task; it starts the demo itself
        fxHold = false;
        return true;
    }
    osdDemoStartShow();
    return startLastError == NULL;
}

// The MCU clock against the video: a field pair is 40 ms by the OSD chip's crystal (625 lines of
// 64 us), the CPU ticks over it say how far the MCU clock is off — the sound's sample clock comes
// from the MCU (HAKRCF411D on its HSI: ~2 % fast, pitch and tempo with it), so the synth is told.
// Eight fields (~160 ms) before the music starts; kept as it was when the chip gives none.
#define OSD_DEMO_CLOCK_FIELDS 8
static void osdDemoTrimAudioClock(void)
{
    uint32_t e0, e;
    if (!max7456WaitVsyncFallingEdge(&e0, 30000)) {
        return;
    }
    for (uint8_t n = 0; n < OSD_DEMO_CLOCK_FIELDS; n++) {
        if (!max7456WaitVsyncFallingEdge(&e, 30000)) {
            return;
        }
    }
    const uint32_t ticks = e - e0;
    const uint32_t nominal = clockMicrosToCycles(OSD_DEMO_CLOCK_FIELDS * 20000u);
    if (ticks < nominal - nominal / 20u || ticks > nominal + nominal / 20u) {
        return; // not a clean run of fields
    }
    ayPwmSetClockErrorPpm((int32_t)(((int64_t)ticks - (int64_t)nominal) * 1000000 / (int64_t)nominal));
}

static uint8_t demoSyncMode; // 0 auto: the camera's sync when the chip sees one, else its own

void osdDemoSetSyncMode(uint8_t mode)
{
    demoSyncMode = (uint8_t)MIN(mode, 2u); // from the next demo start (the engines own the bus)
}

uint8_t osdDemoGetSyncMode(void)
{
    return demoSyncMode;
}

// The demo proper: black, the camera's or the chip's own sync, the demo font, the first scene
// and the music.
static void osdDemoStartShow(void)
{
    max7456ResetHudMotionOffset();
    osdDemoBgType(DISPLAY_BACKGROUND_BLACK);
    max7456Brightness(0, 3);
    max7456FillScreen(OSD_DEMO_PIXEL_OFF);
    max7456RefreshAll();

    // Before the font re-inits: they keep it. No camera: external sync would leave no picture.
    max7456SetSyncMode(demoSyncMode ? demoSyncMode : max7456HasVideoIn() ? 2 : 1);
    osdDemoFontBackup(); // the user's font → the upper 256 glyphs (AT7456)
    const bool fontOk = osdDemoInstallFont();
    if (!fontOk) {
        max7456SetBackgroundType(DISPLAY_BACKGROUND_TRANSPARENT);
        demoBgManaged = false;
        max7456Brightness(displayPortProfileMax7456()->blackBrightness,
                          displayPortProfileMax7456()->whiteBrightness);
        displayReleaseAll(demoDisplay);
        demoDisplay = NULL;
        active = false;
        startLastError = "font NVM install failed";
        return;
    }

    osdDemoTrimAudioClock(); // before the music: true pitch and tempo on an off MCU clock
    rngState = micros() ^ ((uint32_t)millis() << 16);
    osdDemoInitStars(demoDisplay->cols, demoDisplay->rows);

    scrollCol = 0;
    scrollFine = 0;
    hosSteps = osdDemoHosSteps();
    lastStepUs = micros();
    bounceStartMs = millis();
    prevBouncePhase = osdDemoBouncePhase();
    punchArmed = false;
    punchY = 0;
    punchVel = 0;
    plasmaT = 0;
    memset(prevRowBright, 0xFF, sizeof(prevRowBright)); // force first shimmer write
    active = true;
    fxHold = false;
    // Explicit local — LTO has reused a stale r0 for EnterFx on this tree before.
    // The demo opens with the first scene of the order (scene 1: its intro) at the song's top.
    fxOrderIdx = 0;
    s1BaseRowQ8 = 0;
    const osdDemoFx_e first = osdDemoOrderFx(0);
    s1Timeline = first == OSD_DEMO_FX_SCROLLER;
    {
        const osdDemoFx_e next = first;
        osdDemoEnterFx(next);
    }

    if (first == OSD_DEMO_FX_SCROLLER) {
        max7456FillScreen(OSD_DEMO_PIXEL_OFF);
        osdDemoPaintScroller();
        max7456RefreshAll();
    }
    chiptuneSchedulerPark();
    // Demo comes with music (scene 10 also takes its quarter notes from the tracker).
    // The timeline follows the song from its first row: (re)start the track.
    const bool wasPlaying = chiptuneIsPlaying();
    const bool started = chiptuneStart(wasPlaying ? chiptuneCurrentTrack() : 0);
    demoStartedChiptune = !wasPlaying && started;
    s1TimelineMs = millis();
    osdDemoFxArmEnd(fx); // first scene's end on the restarted song
}

const char *osdDemoStartLastError(void)
{
    return startLastError ? startLastError : "unknown";
}

void osdDemoStop(void)
{
    if (!active) {
        return;
    }
    active = false;
    introPhase = 0;
    osdDemoHsCapStop();
    plasma2x2Armed = false;
    donutArmed = false;
    megaArmed = false;
    disarmArmed = false;
    rotoArmed = false;
    twisterArmed = false;
    shoutArmed = false;
    osdDemoIrqStop();
    punchY = 0;
    punchVel = 0;
    max7456MidGlyphSpiEnd();
    max7456SetSyncMode(0); // Betaflight's own: auto (camera sync when there is one)
    // The user's font back from the upper 256 glyphs (~1 s of NVM writes): not while armed.
    if (ARMING_FLAG(ARMED)) {
        fontRestorePending = true;
    } else {
        osdDemoFontRestore();
    }
    max7456ResetHudMotionOffset();
    max7456SetBackgroundType(DISPLAY_BACKGROUND_TRANSPARENT); // Betaflight's again
    demoBgManaged = false;
    max7456Osdm(OSD_DEMO_OSDM_DEFAULT); // Betaflight's OSD: the chip default, whatever was tried
    osdmApplied = 0xFF;
    charRbFlashEndMs = 0;
    if (max7456RbHeld()) {
        max7456RbHold(false, 0);
    }
    max7456Brightness(displayPortProfileMax7456()->blackBrightness,
                      displayPortProfileMax7456()->whiteBrightness);
    if (demoDisplay) {
        max7456FillScreen(0x20); // blank in the restored font (0x90 is a battery symbol there)
        max7456RefreshAll();
        displayReleaseAll(demoDisplay);
        displayClearScreen(demoDisplay, DISPLAY_CLEAR_WAIT);
        demoDisplay = NULL;
    }
    if (demoStartedChiptune) {
        chiptuneStop();
        demoStartedChiptune = false;
    }
    demoMusicMuted = false;
    chiptuneSchedulerUnpark();
}

// Scenes that own the chip through a DWT/VSYNC mid-glyph engine.
static bool osdDemoFxIsMidGlyph(osdDemoFx_e f)
{
    return f == OSD_DEMO_FX_WIPE || f == OSD_DEMO_FX_TUNNEL || f == OSD_DEMO_FX_PLASMA2X2
        || f == OSD_DEMO_FX_TWISTER || f == OSD_DEMO_FX_SHOUT || f == OSD_DEMO_FX_DONUT
        || f == OSD_DEMO_FX_MEGA || f == OSD_DEMO_FX_ROTO || f == OSD_DEMO_FX_CUBE
        || f == OSD_DEMO_FX_BITMAP || f == OSD_DEMO_FX_DISARM;
}

static uint8_t osdDemoFxPatterns(osdDemoFx_e f)
{
    return (f < OSD_DEMO_FX_COUNT) ? fxPatterns[f] : 0u;
}

static timeMs_t osdDemoFxDurationMs(osdDemoFx_e f)
{
    // No-music length (and the fallback when the song clock is lost).
    return (timeMs_t)osdDemoFxPatterns(f) * OSD_DEMO_PATTERN_ROWS * OSD_DEMO_S1_ROW_MS;
}

static osdDemoFx_e osdDemoOrderFx(uint8_t idx)
{
    return (osdDemoFx_e)osdDemoSceneFx[demoOrder[idx % demoOrderLen]];
}

// Song line where order entry `idx` starts (the cycle runs from entry 0 at the track's top).
static uint32_t osdDemoOrderRows(uint8_t idx)
{
    uint32_t rows = 0;
    for (uint8_t n = 0; n < idx && n < demoOrderLen; n++) {
        rows += (uint32_t)osdDemoFxPatterns(osdDemoOrderFx(n)) * OSD_DEMO_PATTERN_ROWS;
    }
    return rows;
}

// First order entry showing `scene`, or demoOrderLen when it is not in the cycle.
static uint8_t osdDemoOrderFind(uint8_t scene)
{
    for (uint8_t n = 0; n < demoOrderLen; n++) {
        if (demoOrder[n] == scene) {
            return n;
        }
    }
    return demoOrderLen;
}

static bool osdDemoStartSceneEx(uint8_t scene, bool hold, uint8_t orderIdx);

bool osdDemoStartScene(uint8_t scene)
{
    return osdDemoStartSceneEx(scene, true, UINT8_MAX);
}

// orderIdx: the step of the order it starts (UINT8_MAX: the scene's first step, if any).
static bool osdDemoStartSceneEx(uint8_t scene, bool hold, uint8_t orderIdx)
{
    if (scene == 0u) {
        // Scene 0: the intro — Betaflight's OSD over the camera (its sync), eaten by the rain —
        // then the demo from scene 1 on the chip's own sync, cycling. Not in the cycle itself:
        // every pass would rewrite the whole font twice (~6 s of NVM writes).
        if (ARMING_FLAG(ARMED)) {
            startLastError = "disarm first";
            return false;
        }
        if (active) {
            osdDemoStop(); // the user's font back: the intro shows it
        }
        const bool was = introEnabled;
        introEnabled = true;
        introSkip = false;
        const bool ok = osdDemoStart();
        introEnabled = was;
        if (ok && !introPhase) {
            startLastError = "scene0: no intro on this chip (a MAX7456 has no font backup)";
        }
        return ok;
    }
    if (scene >= ARRAYLEN(osdDemoSceneFx) || osdDemoSceneFx[scene] < 0) {
        startLastError = "unknown scene (0..14)";
        return false;
    }
    if (ARMING_FLAG(ARMED)) {
        startLastError = "disarm first";
        return false;
    }
    if (active && introPhase) {
        introPhase = 0; // a scene asked for during the intro: the demo proper now
        osdDemoStartShow();
        if (!active) {
            return false;
        }
    }
    if (!active) {
        introSkip = true; // a scene by itself: no intro
        const bool ok = osdDemoStart();
        introSkip = false;
        if (!ok) {
            return false;
        }
    }
    fxHold = hold;
    const osdDemoFx_e next = (osdDemoFx_e)osdDemoSceneFx[scene];
    // The music follows the demo timeline: start the track where this scene begins in the
    // cycle order (a scene outside the order keeps the music where it is). Scene 1 replays
    // its intro from its own start.
    const uint8_t idx = (orderIdx < demoOrderLen) ? orderIdx : osdDemoOrderFind(scene);
    const bool inOrder = idx < demoOrderLen;
    if (inOrder) {
        fxOrderIdx = idx;
    }
    if (inOrder && (chiptuneIsPlaying() || demoMusicMuted) && next != OSD_DEMO_FX_DISARM) {
        demoMusicMuted = false;
        (void)chiptuneStartAt(chiptuneCurrentTrack(), osdDemoOrderRows(idx));
    }
    // Already running: the engine goes on, only its end moves to the restarted track (it had
    // kept the music where it was, so the scene sat off its patterns).
    if (fx == next && ((next == OSD_DEMO_FX_TWISTER && twisterArmed)
                       || (next == OSD_DEMO_FX_PLASMA2X2 && plasma2x2Armed)
                       || (next == OSD_DEMO_FX_SHOUT && shoutArmed)
                       || (next == OSD_DEMO_FX_TUNNEL && tunnelArmed)
                       || (next == OSD_DEMO_FX_WIPE && wipeArmed)
                       || (next == OSD_DEMO_FX_DONUT && donutArmed)
                       || (next == OSD_DEMO_FX_MEGA && megaArmed)
                       || (next == OSD_DEMO_FX_ROTO && rotoArmed)
                       || (next == OSD_DEMO_FX_CUBE && cubeArmed)
                       || (next == OSD_DEMO_FX_BITMAP && bmArmed)
                       || (next == OSD_DEMO_FX_DISARM && disarmArmed))) {
        fxStartMs = millis();
        osdDemoFxArmEnd(next);
        return true;
    }
    s1Timeline = (next == OSD_DEMO_FX_SCROLLER) && inOrder;
    s1BaseRowQ8 = inOrder ? osdDemoOrderRows(idx) * 256u : 0u;
    s1TimelineMs = millis();
    osdDemoEnterFx(next);
    return true;
}

bool osdDemoIsActive(void)
{
    return active;
}

static void osdDemoUpdate(timeUs_t currentTimeUs)
{
    if (!active || !demoDisplay) {
        return;
    }
    if (introPhase) {
        UNUSED(currentTimeUs);
        osdDemoIntroUpdate();
        return;
    }
    // Switch effects on wall-clock first (independent of SPI idle / step gates).
    // Keep EnterFx args in locals so LTO cannot reuse a stale r0 (seen passing
    // the heartbeat "reinited" flag instead of OSD_DEMO_FX_SCROLLER).
    const timeMs_t nowMs = millis();
    const timeMs_t elapsed = nowMs - fxStartMs;
    const timeMs_t fxMs = osdDemoFxDurationMs(fx);
    bool fxDue = elapsed >= fxMs;
    if (fxEndByRow) {
        uint32_t rowQ8;
        uint8_t speed;
        fxDue = osdDemoSongClockQ8(&rowQ8, &speed)
            ? (rowQ8 >= fxEndRowQ8 || elapsed >= fxMs + OSD_DEMO_FX_ROW_GRACE_MS)
            : elapsed >= fxMs;
    }
    if (fxDue) {
        if (!fxHold) {
            if (fxOrderIdx + 1u >= demoOrderLen) {
                // The last step is over: the demo ends as `osd_demo stop` ends it (no new lap)
                osdDemoStop();
                return;
            }
            fxOrderIdx++;
            const osdDemoFx_e next = osdDemoOrderFx(fxOrderIdx);
            if (next == OSD_DEMO_FX_SCROLLER) {
                // Scene 1 plays its whole timeline (intro first) from the pattern it starts on.
                uint32_t rowQ8;
                uint8_t speed;
                const uint32_t patQ8 = OSD_DEMO_PATTERN_ROWS * 256u;
                s1BaseRowQ8 = osdDemoSongClockQ8(&rowQ8, &speed) ? (rowQ8 + patQ8 / 2u) / patQ8 * patQ8 : 0u;
                s1TimelineMs = millis();
                s1Timeline = true;
            }
            osdDemoEnterFx(next);
            return;
        }
        // Held: mid-glyph engines run continuously; the scroller keeps scrolling (a restart
        // jumped the text back to its start every 8 s); other time-phased scenes loop.
        if (fx == OSD_DEMO_FX_SCROLLER) {
            s1Timeline = false;  // past its song section: scroll on freely
            fxEndByRow = false;
            fxStartMs = nowMs;
        } else if (!osdDemoFxIsMidGlyph(fx)) {
            const osdDemoFx_e same = fx;
            osdDemoEnterFx(same);
            return;
        }
    }

    if (!osdDemoFxIsMidGlyph(fx) && osdmApplied != osdDemoOsdmWanted()) {
        osdDemoApplyOsdmNow();
    }
    if (!irqRunning) { // the character scenes: their background kept from the task, ~10 Hz
        static timeMs_t bgKeepMs;
        if ((timeDelta_t)(nowMs - bgKeepMs) >= 100) {
            bgKeepMs = nowMs;
            osdDemoBgKeep();
        }
    }
    if (charRbFlashEndMs && (int32_t)(millis() - charRbFlashEndMs) >= 0) {
        charRbFlashEndMs = 0;
        max7456RbHold(false, 0); // the character scene's flash ends: rows to their levels
    }
    // Mid-glyph scenes: the TIM5 interrupt chain draws; the task only does background work.
    if (osdDemoFxIsMidGlyph(fx)) {
        if (ARMING_FLAG(ARMED)) {
            osdDemoStop();
            return;
        }
        osdDemoIrqTask();
        return;
    }

    if (fx == OSD_DEMO_FX_SCROLLER || fx == OSD_DEMO_FX_PLASMA) {
        if (!max7456DmaInProgress()) {
            osdDemoScrollerUpdate(); // field loop: scenes 1 and 2
        }
        return;
    }

    // Same step rate for all effects so HOS/star phase never drifts.
    const timeDelta_t stepUs = 1000000 / OSD_DEMO_SCROLL_HZ;
    if ((int32_t)(currentTimeUs - lastStepUs) < stepUs) {
        return;
    }

    // Do not require buffersSynced — while animating it is almost never true on
    // real hardware, which starved paints and made FX switches look stuck.
    if (max7456DmaInProgress()) {
        return;
    }

    lastStepUs += stepUs;
    if ((int32_t)(currentTimeUs - lastStepUs) > stepUs) {
        lastStepUs = currentTimeUs;
    }

    // 1) Build the next frame in CPU shadow first (tunnel/plasma are heavy).
    // 2) Then VSYNC + SPI immediately — never burn blanking on paint math, or
    //    the beam slices the tunnel text band (strobe / pixel jitter).
    osdDemoAdvanceHosAndStars();

    if (fx == OSD_DEMO_FX_PLASMA) {
        plasmaT++;
        osdDemoPaintPlasma();
    } else if (fx == OSD_DEMO_FX_FIRE) {
        osdDemoPaintFire();
    } else {
        osdDemoPaintScroller();
    }

    // Soft-scroll (no wrap): early HOS. Wrap: HOS stays deferred until pass end.
    osdDemoSyncFlush(true, true);
}

// From the OSD driver's heartbeat: the OSD DEMO mode switch (on: play, off: stop), then the
// demo's tick, or a font restore left over from a demo that was cut off.
void osdDemoPoll(timeUs_t currentTimeUs)
{
    static bool switchWas;
    const bool on = IS_RC_MODE_ACTIVE(BOXOSDDEMO);
    if (on != switchWas) {
        switchWas = on;
        if (on && !active && !ARMING_FLAG(ARMED)) {
            (void)osdDemoStart();
        } else if (!on && active) {
            osdDemoStop();
        }
    }
    if (active) {
        osdDemoUpdate(currentTimeUs);
    } else {
        osdDemoFontRestorePoll();
    }
}

#endif // USE_OSD_DEMO
