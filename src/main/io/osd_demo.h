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

#include "common/time.h"

#ifdef USE_OSD_DEMO

// Scenes: 0 intro, 1 scroller, 2 plasma, 3 fire, 4 wipe, 5 tunnel, 6 donut, 7 plasma 2×2,
// 8 twister, 9 megacolor, 10 shoutouts, 11 rotozoom, 12 cube, 13 bitmap plasma, 14 disarm.
#define OSD_DEMO_SCENE_COUNT 15

bool osdDemoStart(void);                 // the whole demo, once
bool osdDemoStartScene(uint8_t scene);   // one scene, held
const char *osdDemoStartLastError(void); // set when a start returns false
void osdDemoSetSyncMode(uint8_t mode);   // 0 auto (camera or own), 1 internal, 2 external
uint8_t osdDemoGetSyncMode(void);
const char *osdDemoSoundPad(void);       // "sound on <pad>" or why not
void osdDemoStop(void);
bool osdDemoIsActive(void);
void osdDemoPoll(timeUs_t currentTimeUs); // OSD heartbeat: mode switch, the demo's tick
void osdDemoFontStatus(bool restoreNow, const char **note, uint8_t *mark, uint16_t *backupWrites, uint16_t *restoreWrites);

#else

static inline bool osdDemoIsActive(void) { return false; }

#endif
