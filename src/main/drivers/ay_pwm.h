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

#include "drivers/io_types.h"

// The music's output: a software AY-3-8910 (drivers/ay_synth.c) as a PWM DAC on a timer pin,
// the LED strip pad by default, the samples streamed into the timer by DMA.

bool ayPwmClaim(void);                          // false: no free pad with a timer and DMA
ioTag_t ayPwmPin(void);                         // the claimed pad, IO_TAG_NONE before
bool ayPwmStart(void);                          // false: no timer / DMA on the pin
void ayPwmStop(void);
bool ayPwmIsRunning(void);
void ayPwmSetRegs(const uint8_t regs[16]);      // task context: latched at the next block
void ayPwmApplyRegsIsr(const uint8_t regs[16]); // render context only (the frame callback)
void ayPwmSetFrameCallback(void (*cb)(void));   // 50 Hz, called from the render interrupt
void ayPwmResetFrameClock(void);
void ayPwmSetClockErrorPpm(int32_t ppm);        // the MCU clock's measured error: true pitch / tempo
