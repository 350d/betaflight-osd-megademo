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
 * OSD demo build: the configuration defaults the demo needs (applied by `defaults` and on a
 * fresh EEPROM). PAL, the MAX7456 displayport, and a light flight loop: the demo's raster
 * engine takes the CPU the PID loop and the filters would use. Bench only — the demo disables
 * the motors anyway.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_OSD_DEMO_DEFAULTS

#include "config/config.h"
#include "config/feature.h"
#include "drivers/motor_types.h"
#include "drivers/osd.h"
#include "flight/pid.h"
#include "osd/osd.h"
#include "pg/motor.h"
#include "pg/rx.h"
#include "pg/vcd.h"
#include "sensors/barometer.h"
#include "sensors/gyro.h"

void targetConfiguration(void)
{
    featureConfigMutable()->enabledFeatures |= FEATURE_OSD;
    vcdProfileMutable()->video_system = VIDEO_SYSTEM_PAL;
    osdConfigMutable()->displayPortDevice = OSD_DISPLAYPORT_DEVICE_MAX7456;
    osdConfigMutable()->framerate_hz = 60;

    pidConfigMutable()->pid_process_denom = 8;
    gyroConfigMutable()->gyro_lpf1_static_hz = 0;
    gyroConfigMutable()->gyro_lpf2_static_hz = 0;
    gyroConfigMutable()->gyro_lpf1_dyn_min_hz = 0;
    gyroConfigMutable()->gyro_lpf1_dyn_max_hz = 0;
    gyroConfigMutable()->simplified_gyro_filter = false;
    for (unsigned i = 0; i < PID_PROFILE_COUNT; i++) {
        pidProfile_t *pid = pidProfilesMutable(i);
        pid->dterm_lpf1_dyn_min_hz = 0;
        pid->dterm_lpf1_dyn_max_hz = 0;
        pid->dterm_lpf1_static_hz = 0;
        pid->dterm_lpf2_static_hz = 0;
        pid->simplified_dterm_filter = false;
    }
#ifdef USE_BARO
    barometerConfigMutable()->baro_hardware = BARO_NONE;
#endif
    rxConfigMutable()->rc_smoothing = 0;
    motorConfigMutable()->dev.motorProtocol = MOTOR_PROTOCOL_DSHOT150;
}

#endif
