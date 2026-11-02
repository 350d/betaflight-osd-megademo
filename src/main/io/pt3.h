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

#pragma once

#include <stdbool.h>
#include <stdint.h>

// Pro Tracker 3 / Vortex Tracker II module player: AY-3-8910 register frames at 50 Hz.

typedef struct pt3Channel_s {
    uint16_t pattern;           // read position in the pattern data
    uint8_t skip, skipLeft;     // lines a row lasts, and left of the current one
    uint16_t sample;            // sample data (its 4-byte lines)
    uint8_t sampleLoop, sampleLen, samplePos;
    uint16_t ornament;          // ornament data (signed note offsets)
    uint8_t ornamentLoop, ornamentLen, ornamentPos;
    uint8_t sampleNum;          // sample number of the last sample change
    bool newNote;               // a note started in the line just decoded
    uint8_t note, slideToNote, volume;
    bool enabled, envelope, simpleGliss;
    int16_t toneSliding, toneSlideStep, toneDelta;
    uint8_t toneSlideDelay, toneSlideLeft;
    uint16_t toneAcc, tone;
    int8_t ampSliding, noiseSliding, envSliding;
    uint8_t onOffDelay, offOnDelay, onOffLeft;
    uint8_t amplitude;
} pt3Channel_t;

typedef struct pt3Song_s {
    const uint8_t *data;
    uint32_t len;
    uint8_t version;            // 3..9: the x of Pro Tracker 3.x
    uint8_t speed;              // 50 Hz ticks a line
    uint8_t positions, loop;    // order length and the position the song loops to
    uint16_t noteTable[96];
    const uint8_t (*volumeTable)[16];
    pt3Channel_t ch[3];
    uint8_t noiseBase, noiseAdd;
    uint16_t envBase;
    int16_t envSlide, envSlideAdd;
    uint8_t envDelay, envDelayLeft;
    uint8_t envType;            // 0xFF: no new envelope shape since the last frame
} pt3Song_t;

bool pt3Load(pt3Song_t *s, const uint8_t *data, uint32_t len);
void pt3SetPosition(pt3Song_t *s, uint8_t position);
bool pt3DecodeLine(pt3Song_t *s);          // true: the pattern has ended (nothing decoded)
void pt3MakeFrame(pt3Song_t *s, uint8_t regs[16]);
