/*
 * This file is part of INAV (Vision Extension)
 *
 * INAV is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * INAV is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with INAV.  If not, see <http://www.gnu.org/licenses/>.
 */

// INAV Vision specific IDs start from 0x4500

#define MSP2_INAV_VISION_POSE                   0x4500      // Get current aircraft pose (pos,vel,attitude)
#define MSP2_INAV_VISION_POSE_SET               0x4501      // Feed attitude, position, velocity to controller
#define MSP2_INAV_VISION_PRM_SET                0x4502      // Set weight parameters for vision inputs
