/*
 * This file is part of INAV Project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Alternatively, the contents of this file may be used under the terms
 * of the GNU General Public License Version 3, as described below:
 *
 * This file is free software: you may copy, redistribute and/or modify
 * it under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see http://www.gnu.org/licenses/.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include <platform.h>

#include "build/build_config.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/utils.h"
#include "common/time.h"

#include "config/feature.h"
#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/io.h"
#include "drivers/light_led.h"
#include "drivers/time.h"

#include "drivers/vision/vision.h"
#include "drivers/vision/vision_virtual.h"

#include "fc/config.h"
#include "fc/runtime_config.h"
#include "fc/settings.h"

#include "sensors/boardalignment.h"
#include "sensors/gyro.h"
#include "sensors/sensors.h"
#include "sensors/vision.h"

#include "scheduler/scheduler.h"

#include "io/vision.h"

#include "build/debug.h"

vision_t vision;

#ifdef USE_OPFLOW
static bool visionIsCalibrating = false;
static timeMs_t visionCalibrationStartedAt;
static float visionCalibrationBodyAcc;
static float visionCalibrationFlowAcc;

#define OPFLOW_SQUAL_THRESHOLD_HIGH     35      // TBD
#define OPFLOW_SQUAL_THRESHOLD_LOW      10      // TBD
#define OPFLOW_UPDATE_TIMEOUT_US        100000  // At least 5Hz updates required
#define OPFLOW_CALIBRATE_TIME_MS        30000   // 30 second calibration time

PG_REGISTER_WITH_RESET_TEMPLATE(opticalFlowConfig_t, opticalFlowConfig, PG_OPFLOW_CONFIG, 2);

PG_RESET_TEMPLATE(opticalFlowConfig_t, opticalFlowConfig,
    .vision_hardware = SETTING_OPFLOW_HARDWARE_DEFAULT,
    .vision_align = SETTING_ALIGN_OPFLOW_DEFAULT,
    .vision_scale = SETTING_OPFLOW_SCALE_DEFAULT,
);

static bool visionDetect(visionDev_t * dev, uint8_t visionHardwareToUse)
{
    requestedSensors[SENSOR_INDEX_VISION] = visionHardwareToUse;

    switch (visionHardwareToUse) {
        case OPFLOW_FAKE:
#if defined(USE_OPFLOW_FAKE)
            if (fakeVisionDetect(dev)) {
                visionHardware = OPFLOW_FAKE;
            }
#endif
            break;

        case OPFLOW_CXOF:
#if defined(USE_OPFLOW_CXOF)
            if (virtualVisionDetect(dev, &visionCxofVtable)) {
                visionHardware = OPFLOW_CXOF;
            }
#endif
            break;

        case OPFLOW_MSP:
#if defined(USE_OPFLOW_MSP)
            if (virtualVisionDetect(dev, &visionMSPVtable)) {
                visionHardware = OPFLOW_MSP;
            }
#endif
            break;

        case OPFLOW_NONE:
            visionHardware = OPFLOW_NONE;
            break;
    }

    if (visionHardware == OPFLOW_NONE) {
        sensorsClear(SENSOR_OPFLOW);
        return false;
    }

    detectedSensors[SENSOR_INDEX_OPFLOW] = visionHardware;
    sensorsSet(SENSOR_OPFLOW);
    return true;
}

static void visionZeroBodyGyroAcc(void)
{
    vision.gyroBodyRateTimeUs = 0;
    vision.gyroBodyRateAcc[X] = 0;
    vision.gyroBodyRateAcc[Y] = 0;
}

bool visionInit(void)
{
    if (!visionDetect(&vision.dev, opticalFlowConfig()->vision_hardware)) {
        return false;
    }

    if (!vision.dev.initFn(&vision.dev)) {
        sensorsClear(SENSOR_OPFLOW);
        return false;
    }

    visionZeroBodyGyroAcc();

    return true;
}

void visionStartCalibration(void)
{
    visionCalibrationStartedAt = millis();
    visionIsCalibrating = true;
    visionCalibrationBodyAcc = 0;
    visionCalibrationFlowAcc = 0;
}

/*
 * This is called periodically by the scheduler
 */
void visionUpdate(timeUs_t currentTimeUs)
{
    if (!vision.dev.updateFn)
        return;

    if (vision.dev.updateFn(&vision.dev)) {
        // Indicate valid update
        vision.isHwHealty = true;
        vision.lastValidUpdate = currentTimeUs;
        vision.rawQuality = vision.dev.rawData.quality;

        // Handle state switching
        switch (vision.flowQuality) {
            case OPFLOW_QUALITY_INVALID:
                if (vision.dev.rawData.quality >= OPFLOW_SQUAL_THRESHOLD_HIGH) {
                    vision.flowQuality = OPFLOW_QUALITY_VALID;
                }
                break;

            case OPFLOW_QUALITY_VALID:
                if (vision.dev.rawData.quality <= OPFLOW_SQUAL_THRESHOLD_LOW) {
                    vision.flowQuality = OPFLOW_QUALITY_INVALID;
                }
                break;
        }

        // Vision updated. Assume zero valus unless further processing sets otherwise
        vision.flowRate[X] = 0;
        vision.flowRate[Y] = 0;
        vision.bodyRate[X] = 0;
        vision.bodyRate[Y] = 0;

        // In the following code we operate deg/s and do conversion to rad/s in the last step
        // Calculate body rates
        if (vision.gyroBodyRateTimeUs > 0) {
            vision.bodyRate[X] = vision.gyroBodyRateAcc[X] / vision.gyroBodyRateTimeUs;
            vision.bodyRate[Y] = vision.gyroBodyRateAcc[Y] / vision.gyroBodyRateTimeUs;
        }

        // If quality of the flow from the sensor is good - process further
        if (vision.flowQuality == OPFLOW_QUALITY_VALID) {
            const float integralToRateScaler = (opticalFlowConfig()->vision_scale > 0.01f) ? (1.0e6 / vision.dev.rawData.deltaTime) / (float)opticalFlowConfig()->vision_scale : 0.0f;

            // Apply sensor alignment
            applySensorAlignment(vision.dev.rawData.flowRateRaw, vision.dev.rawData.flowRateRaw, opticalFlowConfig()->vision_align);

            // Calculate flow rate and accumulated body rate
            vision.flowRate[X] = vision.dev.rawData.flowRateRaw[X] * integralToRateScaler;
            vision.flowRate[Y] = vision.dev.rawData.flowRateRaw[Y] * integralToRateScaler;

            // Only update DEBUG_FLOW_RAW if flow is good
            DEBUG_SET(DEBUG_FLOW_RAW, 0, (vision.flowRate[X]));
            DEBUG_SET(DEBUG_FLOW_RAW, 1, (vision.flowRate[Y]));
            DEBUG_SET(DEBUG_FLOW_RAW, 2, (vision.bodyRate[X]));
            DEBUG_SET(DEBUG_FLOW_RAW, 3, (vision.bodyRate[Y]));
        }

        // Process calibration
        if (visionIsCalibrating) {
            // Blink LED
            LED0_TOGGLE;

            if ((millis() - visionCalibrationStartedAt) > OPFLOW_CALIBRATE_TIME_MS) {
                // Finish calibration if we accumulated more than 3600 deg of rotation over 30 seconds
                if (visionCalibrationBodyAcc > 3600.0f) {
                    opticalFlowConfigMutable()->vision_scale = visionCalibrationFlowAcc / visionCalibrationBodyAcc;
                    saveConfigAndNotify();
                }

                visionIsCalibrating = 0;
            }
            else if (vision.flowQuality == OPFLOW_QUALITY_VALID) {
                // Ongoing calibration - accumulate body and flow rotation magniture if vision quality is good enough
                const float invDt = 1.0e6 / vision.dev.rawData.deltaTime;
                visionCalibrationBodyAcc += calc_length_pythagorean_2D(vision.bodyRate[X], vision.bodyRate[Y]);
                visionCalibrationFlowAcc += calc_length_pythagorean_2D(vision.dev.rawData.flowRateRaw[X], vision.dev.rawData.flowRateRaw[Y]) * invDt;
            }
        }

        // Convert to radians so NAV doesn't have to do the conversion
        vision.bodyRate[X] = DEGREES_TO_RADIANS(vision.bodyRate[X]);
        vision.bodyRate[Y] = DEGREES_TO_RADIANS(vision.bodyRate[Y]);
        vision.flowRate[X] = DEGREES_TO_RADIANS(vision.flowRate[X]);
        vision.flowRate[Y] = DEGREES_TO_RADIANS(vision.flowRate[Y]);

        // Zero out gyro accumulators to calculate rotation per flow update
        visionZeroBodyGyroAcc();
    }
    else {
        // No new data available
        if (vision.isHwHealty && ((currentTimeUs - vision.lastValidUpdate) > OPFLOW_UPDATE_TIMEOUT_US)) {
            vision.isHwHealty = false;

            vision.flowQuality = OPFLOW_QUALITY_INVALID;
            vision.rawQuality = 0;

            vision.flowRate[X] = 0;
            vision.flowRate[Y] = 0;
            vision.bodyRate[X] = 0;
            vision.bodyRate[Y] = 0;

            visionZeroBodyGyroAcc();
        }
    }
}

/* Run a simple gyro update integrator to estimate average body rate between two optical flow updates */
void visionGyroUpdateCallback(timeUs_t gyroUpdateDeltaUs)
{
    if (!vision.isHwHealty)
        return;

    for (int axis = 0; axis < 2; axis++) {
        vision.gyroBodyRateAcc[axis] += gyro.gyroADCf[axis] * gyroUpdateDeltaUs;
    }

    vision.gyroBodyRateTimeUs += gyroUpdateDeltaUs;
}

bool visionIsHealthy(void)
{
    return vision.isHwHealty;
}
#endif
