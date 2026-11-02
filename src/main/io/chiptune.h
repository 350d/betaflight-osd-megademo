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

/* PT3 player: a software AY-3-8910 on a PWM-DAC timer pin. */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "common/time.h"

#ifdef USE_CHIPTUNE

/* trackIndex 1..N; 0 = track 1 */
bool chiptuneStart(uint8_t trackIndex);
/* Start `lines` pattern lines into the track (silent fast-forward of the PT3 engine). */
bool chiptuneStartAt(uint8_t trackIndex, uint32_t lines);
void chiptuneStop(void);
bool chiptuneIsPlaying(void);
uint8_t chiptuneCurrentTrack(void);
/* Pattern-line pulse — increments each new PT3 line (≈ beat grid). */
uint8_t chiptuneGetLinePulse(void);
/* Tracker clock at the last 50 Hz tick: line counter (= linePulse), song position and row
 * of that line, frames of it already played after the tick (0 = the tick just ended the
 * previous line), speed, and the tick's micros(). False when stopped. */
typedef struct chiptuneClock_s {
    uint8_t line;
    uint32_t lines;     // lines since the track started, monotonic across song loops
    uint8_t order;
    uint8_t row;
    uint8_t tick;
    uint8_t speed;
    timeUs_t tickUs;
} chiptuneClock_t;
bool chiptuneGetClock(chiptuneClock_t *c);
/* Kick drum counter (loud attack + fast pitch drop in the AY frame), last kick's volume 0..15. */
uint8_t chiptuneGetKickPulse(uint8_t *level);
/* Snare counter (tone + noise attack with a slow pitch drop; learned sample, flagged a tick early). */
uint8_t chiptuneGetSnarePulse(void);
/* Current PT3 speed (50 Hz ticks per line), 0 when not playing. */
uint8_t chiptuneGetSpeed(void);
/* Copy of the AY register frame currently playing; false when stopped. */
bool chiptuneGetAyRegs(uint8_t regs[16]);
/* Park non-essential scheduler tasks while the music plays (refcounted). */
void chiptuneSchedulerPark(void);
void chiptuneSchedulerUnpark(void);
bool chiptuneSchedulerIsParked(void);

void chiptuneUpdate(timeUs_t currentTimeUs);

#endif
