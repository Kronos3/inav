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

#pragma once

#include <stdint.h>
#include "config/parameter_group.h"
#include "drivers/vision/vision.h"

typedef enum {
    VISION_QUALITY_INVALID,
    VISION_QUALITY_VALID
} visionQuality_e;

typedef struct vision_s {
    visionDev_t dev;

    visionQuality_e visionQuality;
    timeUs_t        lastValidUpdate;
    bool            isHwHealty;

    float           position[3];    // 3-vector XYZ position
    float           pose[4];        // Quaternion pose

    float           gyroBodyRateAcc[2];
    timeUs_t        gyroBodyRateTimeUs;

    uint8_t         rawQuality;
} vision_t;

extern vision_t vision;

void visionGyroUpdateCallback(timeUs_t gyroUpdateDeltaUs);
bool visionInit(void);
void visionUpdate(timeUs_t currentTimeUs);
bool visionIsHealthy(void);
