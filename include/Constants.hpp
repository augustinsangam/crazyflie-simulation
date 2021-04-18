#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <bits/stdint-uintn.h>

const float ALTITUDE_STEP = 0.1F;
const uint16_t WALL_DISTANCE_THRESH_FRONT = 20;
const uint16_t WALL_DISTANCE_THRESH_SIDE = 10;
const float DODGE_PRECISION = 0.05F;
const float STABILIZE_POS_PRECISION = 0.005F;
const float STABILIZE_YAW_PRECISION = 0.01F;
const float LANDING_ZONE_PRECISION = 0.2F;
const double BATTERY_THRESHOLD_RTB = 0.3;
const double BATTERY_THRESHOLD_EMERGENCY = 0.05;
const float LANDING_THRESHOLD = 0.01F;
const float FORWARD_UNIT = -0.03F;
const uint16_t DODGE_TIMEOUT_TICKS = 25;

#endif /* CONSTANTS_H */
