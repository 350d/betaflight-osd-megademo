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
 * PT3 → AY. The tracker and the mixer both run from the AY output's render interrupt (a DMA
 * half-buffer): the 50 Hz PT3 tick lands on the exact sample of the frame boundary. The task
 * only starts / stops and watches arming, so OSD demo load cannot stall the music.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_CHIPTUNE

#include "build/debug.h"
#include "common/maths.h"
#include "common/utils.h"
#include "drivers/light_led.h"
#include "drivers/ay_pwm.h"
#include "drivers/time.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "fc/runtime_config.h"
#include "io/chiptune.h"
#include "io/chiptune_tracks.h"
#include "io/pt3.h"
#include "scheduler/scheduler.h"
#include "sensors/gyro.h"

#define CHIPTUNE_TASK_HZ 10 /* ARM/safety poll only — music is IRQ-driven */

static volatile bool playing;
static uint8_t currentTrack;
static pt3Song_t pt3Song;
static int orderIndex;
static int speedLeft;
static int doneLine;
static uint8_t linePulse; // increments on each new PT3 pattern line
// Tracker position snapshot at each 50 Hz tick (seqlock: odd while the ISR writes).
static volatile uint8_t clockSeq;
static volatile uint8_t clockLine;
static volatile uint8_t clockTick;  // frames of clockLine already played
static volatile uint8_t clockOrder; // song position (order index) of clockLine
static volatile uint8_t clockRow;   // row of clockLine inside its pattern
static uint8_t patRow;              // row of the decoded line inside the current pattern
static uint32_t songLines;          // lines since the track started (monotonic across loops)
static volatile uint32_t clockLines; // songLines of clockLine
#define CHIPTUNE_SEEK_MAX_FRAMES 60000u // ~20 min of song: seek guard
static volatile uint8_t clockSpeed;
static volatile timeUs_t clockUs;
// Kick drum from the AY frame: a loud attack whose tone period then grows two frames in a
// row, first by ≥ 30 % (fast pitch drop: 763 → 1401 → 1785 … in Summer Steps). Snares sweep
// slowly (547 → 597 → 647), arpeggio octave jumps (209 → 418 → 418) stop growing, and pure
// noise hits are hats / snares — none of them count. Fires on the 3rd frame (~40 ms late).
static volatile uint8_t kickPulse;
static volatile uint8_t kickLevel;   // attack volume 0..15
static uint8_t kickPrevAmp[3];
static uint16_t kickPrevPer[3];
static uint8_t kickStage[3];         // 0 idle, 1 attack seen, 2 first drop seen
static uint16_t kickPer[3];          // period at the previous stage
static uint8_t kickAmp[3];
#define CHIPTUNE_KICK_MIN_AMP 12
// The frame detector above confirms a kick ~2 frames after it starts. It also learns which
// PT3 sample the kick note uses (two confirmations in a row); from then on a kick is flagged
// when that note is decoded — one tick BEFORE it sounds — so the demo lands on the hit.
#define CHIPTUNE_KICK_LEARN_HITS 2
#define CHIPTUNE_PRELEARN_FRAMES 6000 // the start-up pre-scan: 2 min of song at most
static int8_t kickSample = -1;       // learned kick sample (-1 = not yet), kept for the track
static uint8_t kickSampleTrack;
static uint8_t kickLearnSample = 0xFF;
static uint8_t kickLearnCount;
static bool engineSeeking;           // fast-forward: no events
// Snare: tone + noise attack whose period then grows slowly and steadily (+3..25 % a frame for
// two frames, 547 → 597 → 647 in Summer Steps) at a held volume. Same learn-then-predict.
static volatile uint8_t snarePulse;
static uint8_t snareStage[3];
static uint16_t snarePer[3];
static int8_t snareSample = -1;
static uint8_t snareSampleTrack;
static uint8_t snareLearnSample = 0xFF;
static uint8_t snareLearnCount;
static uint8_t ayFrame[16];

static const taskId_e chiptuneParkList[] = {
    TASK_GYRO,
    TASK_FILTER,
    TASK_PID,
    TASK_ACCEL,
    TASK_ATTITUDE,
    TASK_BATTERY_VOLTAGE,
    TASK_BATTERY_CURRENT,
    TASK_BATTERY_ALERTS,
#ifdef USE_BEEPER
    TASK_BEEPER,
#endif
#ifdef USE_GPS
    TASK_GPS,
#endif
#ifdef USE_GPS_RESCUE
    TASK_GPS_RESCUE,
#endif
#ifdef USE_ALTITUDE_HOLD
    TASK_ALTHOLD,
#endif
#ifdef USE_POSITION_HOLD
    TASK_POSHOLD,
#endif
#ifdef USE_MAG
    TASK_MAGHOLD,
    TASK_COMPASS,
#endif
#ifdef USE_BARO
    TASK_BARO,
#endif
#ifdef USE_RANGEFINDER
    TASK_RANGEFINDER,
#endif
#ifdef USE_OPTICALFLOW
    TASK_OPTICALFLOW,
#endif
#ifdef USE_PITOT
    TASK_PITOT,
#endif
#if defined(USE_BARO) || defined(USE_GPS) || defined(USE_RANGEFINDER) || defined(USE_OPTICALFLOW)
    TASK_POSITION,
#endif
#ifdef USE_DASHBOARD
    TASK_DASHBOARD,
#endif
#ifdef USE_TELEMETRY
    TASK_TELEMETRY,
#endif
#ifdef USE_LED_STRIP
    TASK_LEDSTRIP,
#endif
#ifdef USE_TRANSPONDER
    TASK_TRANSPONDER,
#endif
#ifdef USE_STACK_CHECK
    TASK_STACK_CHECK,
#endif
#ifdef USE_BST
    TASK_BST_MASTER_PROCESS,
#endif
#ifdef USE_ESC_SENSOR
    TASK_ESC_SENSOR,
#endif
#ifdef USE_VTX_CONTROL
    TASK_VTXCTRL,
#endif
#ifdef USE_CAMERA_CONTROL
    TASK_CAMCTRL,
#endif
#ifdef USE_RCDEVICE
    TASK_RCDEVICE,
#endif
#ifdef USE_ADC_INTERNAL
    TASK_ADC_INTERNAL,
#endif
#ifdef USE_PINIOBOX
    TASK_PINIOBOX,
#endif
#ifdef USE_CRSF_V3
    TASK_SPEED_NEGOTIATION,
#endif
#ifdef USE_RC_STATS
    TASK_RC_STATS,
#endif
#ifdef USE_GIMBAL
    TASK_GIMBAL,
#endif
#if ENABLE_OSD_CUSTOM_TEXT
    TASK_OSD_CUSTOM_TEXT,
#endif
#if ENABLE_DRONECAN
    TASK_DRONECAN,
#endif
};

static uint8_t parkDepth;
// The gyro, accelerometer and attitude keep running while the music plays (the Configurator's
// model follows the board).

static bool chiptuneParkKeeps(taskId_e id)
{
    return (id == TASK_GYRO || id == TASK_FILTER || id == TASK_ACCEL || id == TASK_ATTITUDE);
}

static uint8_t parkedWasEnabled[(ARRAYLEN(chiptuneParkList) + 7) / 8];

void chiptuneSchedulerPark(void)
{
    if (parkDepth > 0) {
        parkDepth++;
        return;
    }
    memset(parkedWasEnabled, 0, sizeof(parkedWasEnabled));
    for (unsigned i = 0; i < ARRAYLEN(chiptuneParkList); i++) {
        const taskId_e id = chiptuneParkList[i];
        if (chiptuneParkKeeps(id)) {
            continue;
        }
        taskInfo_t info;
        getTaskInfo(id, &info);
        if (info.isEnabled) {
            setTaskEnabled(id, false);
            parkedWasEnabled[i / 8] |= (uint8_t)(1u << (i % 8));
        }
    }
    parkDepth = 1;
}

void chiptuneSchedulerUnpark(void)
{
    if (parkDepth == 0) {
        return;
    }
    parkDepth--;
    if (parkDepth > 0) {
        return;
    }
    for (unsigned i = 0; i < ARRAYLEN(chiptuneParkList); i++) {
        if ((parkedWasEnabled[i / 8] & (uint8_t)(1u << (i % 8))) == 0) {
            continue;
        }
        setTaskEnabled(chiptuneParkList[i], true);
    }
}

bool chiptuneSchedulerIsParked(void)
{
    return parkDepth > 0;
}

// Board LEDs to the music, from the 50 Hz tracker tick (a GPIO write each, only on change):
// LED0 flashes on the kick, LED1 on the snare.
#define CHIPTUNE_LED_FLASH_FRAMES 3  // 60 ms
static uint8_t ledKickT, ledSnareT;
static uint8_t ledSeenKick, ledSeenSnare;
static uint8_t ledState = 0xFF; // bit 0 LED0, bit 1 LED1 as last written

static void chiptuneLedsOff(void)
{
    LED0_OFF;
    LED1_OFF;
    ledState = 0;
    ledKickT = ledSnareT = 0;
}

static void chiptuneLedsTick(void)
{
    if (kickPulse != ledSeenKick) {
        ledSeenKick = kickPulse;
        ledKickT = CHIPTUNE_LED_FLASH_FRAMES;
    }
    if (snarePulse != ledSeenSnare) {
        ledSeenSnare = snarePulse;
        ledSnareT = CHIPTUNE_LED_FLASH_FRAMES;
    }
    const uint8_t want = (uint8_t)((ledKickT ? 1u : 0u) | (ledSnareT ? 2u : 0u));
    if (want != ledState) {
        if ((want ^ ledState) & 1u) {
            ledSet(0, want & 1u);
        }
        if ((want ^ ledState) & 2u) {
            ledSet(1, (want & 2u) != 0u);
        }
        ledState = want;
    }
    if (ledKickT) {
        ledKickT--;
    }
    if (ledSnareT) {
        ledSnareT--;
    }
}

static bool loadTrack(uint8_t trackIndex)
{
    const chiptuneTrackInfo_t *tr = chiptuneGetTrack(trackIndex);
    if (!tr || tr->len == 0) {
        return false;
    }

    if (!pt3Load(&pt3Song, tr->data, tr->len)) { // played from flash
        return false;
    }
    currentTrack = trackIndex;
    return true;
}

static void engineReset(void)
{
    orderIndex = 0;
    speedLeft = pt3Song.speed > 0 ? pt3Song.speed : 3;
    doneLine = 0;
    linePulse = 0;
    patRow = 0;
    songLines = 0;
    pt3SetPosition(&pt3Song, orderIndex);
    doneLine = pt3DecodeLine(&pt3Song);
    memset(ayFrame, 0, sizeof(ayFrame));
    ayFrame[7] = 0x3f;
    ayFrame[13] = 0xff;
}

static void engineAdvanceLine(void)
{
    speedLeft--;
    if (speedLeft > 0) {
        return;
    }
    speedLeft = pt3Song.speed > 0 ? pt3Song.speed : 3;
    for (;;) {
        if (doneLine) {
            orderIndex++;
            if (orderIndex >= pt3Song.positions) {
                orderIndex = pt3Song.loop;
            }
            pt3SetPosition(&pt3Song, orderIndex);
            patRow = UINT8_MAX; // the next decoded line is row 0
        }
        doneLine = pt3DecodeLine(&pt3Song);
        if (!doneLine) {
            linePulse++;
            patRow++;
            songLines++;
            if (!engineSeeking && (kickSample >= 0 || snareSample >= 0)) {
                const pt3Channel_t *chs = pt3Song.ch;
                bool kick = false;
                bool snare = false;
                for (uint8_t c = 0; c < 3u; c++) {
                    if (!chs[c].newNote) {
                        continue;
                    }
                    const bool isKick = kickSample >= 0 && chs[c].sampleNum == (uint8_t)kickSample;
                    const bool isSnare = snareSample >= 0 && chs[c].sampleNum == (uint8_t)snareSample;
                    kick = kick || isKick;
                    snare = snare || isSnare;
                }
                if (kick) {
                    kickLevel = 15;
                    kickPulse++; // sounds on the next tick
                }
                if (snare) {
                    snarePulse++;
                }
            }
            break;
        }
    }
}

// Tracker position after the frame just applied (seqlock, read by the demo).
static void chiptuneClockSnap(timeUs_t nowUs)
{
    // The frame just applied is the clockTick-th of clockLine (0-based: clockTick - 1).
    const uint8_t speed = pt3Song.speed > 0 ? (uint8_t)pt3Song.speed : 3;
    clockSeq++;
    clockLine = linePulse;
    clockLines = songLines;
    clockOrder = (uint8_t)orderIndex;
    clockRow = patRow;
    clockTick = (uint8_t)(speed - speedLeft);
    clockSpeed = speed;
    clockUs = nowUs;
    clockSeq++;
}

// Kick / snare detectors on the frame just made (ayFrame), and the learning of their samples.
// learnOnly: no pulses (the pre-scan at the track start).
static void chiptuneDetectDrums(bool learnOnly)
{
    uint8_t hit = 0;
    uint8_t hitCh = 0;
    int8_t snareCh = -1;
    for (uint8_t ch = 0; ch < 3u; ch++) {
        const uint8_t a = ayFrame[8 + ch];
        const uint8_t amp = (a & 0x10u) ? 15u : (uint8_t)(a & 0x0Fu); // envelope = loud
        const uint16_t per = (uint16_t)(ayFrame[ch * 2] | ((ayFrame[ch * 2 + 1] & 0x0Fu) << 8));
        const bool tone = (ayFrame[7] & (uint8_t)(1u << ch)) == 0u;
        if (kickStage[ch] == 1u) {
            kickStage[ch] = (tone && amp >= 8u && (uint32_t)per * 10u >= (uint32_t)kickPer[ch] * 13u) ? 2u : 0u;
            kickPer[ch] = per;
        } else if (kickStage[ch] == 2u) {
            if (tone && amp >= 6u && per > kickPer[ch] && kickAmp[ch] > hit) {
                hit = kickAmp[ch];
                hitCh = ch;
            }
            kickStage[ch] = 0;
        }
        if (kickStage[ch] == 0u && amp >= CHIPTUNE_KICK_MIN_AMP && per > 0u
            && (amp >= (uint8_t)(kickPrevAmp[ch] + 3u)
                || (tone && kickPrevPer[ch] && (uint32_t)per * 3u < (uint32_t)kickPrevPer[ch] * 2u))) {
            kickStage[ch] = 1;
            kickPer[ch] = per;
            kickAmp[ch] = amp;
        }
        // Snare detector (same frame data).
        const bool noise = (ayFrame[7] & (uint8_t)(0x08u << ch)) == 0u;
        const bool slowRise = tone && noise && (uint32_t)per * 100u >= (uint32_t)snarePer[ch] * 103u
            && (uint32_t)per * 100u <= (uint32_t)snarePer[ch] * 125u;
        if (snareStage[ch] == 1u) {
            snareStage[ch] = (slowRise && amp >= 11u) ? 2u : 0u;
            snarePer[ch] = per;
        } else if (snareStage[ch] == 2u) {
            if (slowRise && amp >= 10u) {
                snareCh = (int8_t)ch;
            }
            snareStage[ch] = 0;
        }
        if (snareStage[ch] == 0u && amp >= 12u && tone && noise && per > 0u
            && (amp >= (uint8_t)(kickPrevAmp[ch] + 3u) || !kickPrevPer[ch]
                || (uint32_t)per * 3u < (uint32_t)kickPrevPer[ch] * 2u)) {
            snareStage[ch] = 1;
            snarePer[ch] = per;
        }
        kickPrevAmp[ch] = amp;
        kickPrevPer[ch] = per;
    }
    if (snareCh >= 0 && snareSample < 0) {
        const pt3Channel_t *chs = pt3Song.ch;
        const uint8_t smp = chs[snareCh].sampleNum;
        if (!learnOnly) {
            snarePulse++; // not learned yet: the frame detector drives the pulse
        }
        snareLearnCount = (smp == snareLearnSample) ? (uint8_t)(snareLearnCount + 1u) : 1u;
        snareLearnSample = smp;
        if (snareLearnCount >= CHIPTUNE_KICK_LEARN_HITS) {
            snareSample = (int8_t)smp;
            snareSampleTrack = currentTrack;
        }
    }
    if (hit) {
        const pt3Channel_t *chs = pt3Song.ch;
        const uint8_t smp = chs[hitCh].sampleNum;
        if (kickSample < 0) {
            if (!learnOnly) {
                kickLevel = hit;
                kickPulse++; // not learned yet: the (late) frame detector drives the pulse
            }
            kickLearnCount = (smp == kickLearnSample) ? (uint8_t)(kickLearnCount + 1u) : 1u;
            kickLearnSample = smp;
            if (kickLearnCount >= CHIPTUNE_KICK_LEARN_HITS) {
                kickSample = (int8_t)smp;
                kickSampleTrack = currentTrack;
            }
        }
    }
}

/* Render IRQ @50 Hz frame boundary — must stay light and ISR-safe. */
static void chiptuneIrqFrame(void)
{
    if (!playing) {
        return;
    }
    pt3MakeFrame(&pt3Song, ayFrame);
    ayPwmApplyRegsIsr(ayFrame);
    const timeUs_t nowUs = micros();
    chiptuneDetectDrums(false);
    engineAdvanceLine();
    chiptuneClockSnap(nowUs);
    chiptuneLedsTick();
}

bool chiptuneStart(uint8_t trackIndex)
{
    return chiptuneStartAt(trackIndex, 0);
}

// Start the track `lines` pattern lines in: the engine runs silently up to there (same PT3
// state — envelopes, ornaments, volumes — as if it had played), then audio starts.
bool chiptuneStartAt(uint8_t trackIndex, uint32_t lines)
{
    if (ARMING_FLAG(ARMED)) {
        return false;
    }
    if (trackIndex == 0) {
        trackIndex = 1;
    }
    if (trackIndex < 1 || trackIndex > chiptuneTrackCount()) {
        return false;
    }
    if (playing) {
        chiptuneStop();
    }
    if (!loadTrack(trackIndex)) {
        return false;
    }
    chiptuneSchedulerPark();
    if (!ayPwmStart()) {
        chiptuneSchedulerUnpark();
        return false;
    }
    ayPwmResetFrameClock();
    engineReset();
    if (kickSample >= 0 && kickSampleTrack != currentTrack) {
        kickSample = -1; // another track: learn its kick again
    }
    kickLearnCount = 0;
    if (snareSample >= 0 && snareSampleTrack != currentTrack) {
        snareSample = -1;
    }
    snareLearnCount = 0;
    memset(snareStage, 0, sizeof(snareStage));
    // Drums known from the first hit: run the song silently once (up to ~2 min) to learn the
    // kick / snare samples, then start over — without it the first hits came late or not at all.
    if (kickSample < 0 || snareSample < 0) {
        engineSeeking = true;
        for (uint32_t f = 0; f < CHIPTUNE_PRELEARN_FRAMES && (kickSample < 0 || snareSample < 0); f++) {
            pt3MakeFrame(&pt3Song, ayFrame);
            chiptuneDetectDrums(true);
            engineAdvanceLine();
        }
        (void)loadTrack(trackIndex); // the song's state from its start again
        engineReset();
        kickLearnCount = 0;
        snareLearnCount = 0;
        memset(snareStage, 0, sizeof(snareStage));
        memset(kickStage, 0, sizeof(kickStage));
    }
    engineSeeking = true;
    for (uint32_t f = 0; songLines < lines && f < CHIPTUNE_SEEK_MAX_FRAMES; f++) {
        pt3MakeFrame(&pt3Song, ayFrame);
        engineAdvanceLine();
    }
    engineSeeking = false;
    memset(kickStage, 0, sizeof(kickStage));
    pt3MakeFrame(&pt3Song, ayFrame);
    /* Render IRQ is already live — hand the first frame over, don't poke it. */
    ayPwmSetRegs(ayFrame);
    engineAdvanceLine();
    // Fresh clock now: a stale snapshot from the previous run (until the first IRQ frame)
    // made the demo timeline read an old song position and skip scene 1.
    chiptuneClockSnap(micros());
    ayPwmSetFrameCallback(chiptuneIrqFrame);
    chiptuneLedsOff();
    rescheduleTask(TASK_CHIPTUNE, TASK_PERIOD_HZ(CHIPTUNE_TASK_HZ));
    playing = true;
    setTaskEnabled(TASK_CHIPTUNE, true);
    return true;
}

void chiptuneStop(void)
{
    if (!playing) {
        return;
    }
    playing = false;
    setTaskEnabled(TASK_CHIPTUNE, false);
    ayPwmStop();
    chiptuneLedsOff();
    chiptuneSchedulerUnpark();
}

bool chiptuneIsPlaying(void)
{
    return playing;
}

uint8_t chiptuneCurrentTrack(void)
{
    return currentTrack;
}

// Latest AY register frame (written by the 50 Hz tick in the audio IRQ). A torn copy only
// mixes two consecutive frames — fine for visualisation.
bool chiptuneGetAyRegs(uint8_t regs[16])
{
    if (!playing) {
        return false;
    }
    memcpy(regs, ayFrame, 16);
    return true;
}

uint8_t chiptuneGetSpeed(void)
{
    return (playing && pt3Song.speed > 0) ? (uint8_t)pt3Song.speed : 0;
}

bool chiptuneGetClock(chiptuneClock_t *c)
{
    if (!playing) {
        return false;
    }
    // Bounded: a higher-priority reader (OSD raster ISR) can preempt the writer mid-update
    // and would spin forever; after a few tries a torn copy is fine for animation.
    for (uint8_t tries = 0; tries < 4u; tries++) {
        const uint8_t seq = clockSeq;
        c->line = clockLine;
        c->lines = clockLines;
        c->order = clockOrder;
        c->row = clockRow;
        c->tick = clockTick;
        c->speed = clockSpeed;
        c->tickUs = clockUs;
        if (!(seq & 1u) && seq == clockSeq) {
            break;
        }
    }
    return c->speed > 0;
}

uint8_t chiptuneGetSnarePulse(void)
{
    return snarePulse;
}

uint8_t chiptuneGetKickPulse(uint8_t *level)
{
    if (level) {
        *level = kickLevel;
    }
    return kickPulse;
}

uint8_t chiptuneGetLinePulse(void)
{
    return linePulse;
}

void chiptuneUpdate(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
    if (!playing) {
        return;
    }
    if (ARMING_FLAG(ARMED)) {
        chiptuneStop();
    }
    /* Music is entirely TIM-IRQ driven — nothing to fill here. */
}


#endif
