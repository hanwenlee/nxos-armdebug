
#ifndef __BBR_AVOID_WALL_H__
#define __BBR_AVOID_WALL_H__
#include "base/lib/bbr/actuator.h"
#include "base/lib/bbr/sensor.h"

void bbr_avoid_wall(SensorReading *heading, SensorReading *distance, ActuatorState behavior_actuations[], ActuatorState availableStates[]);
#endif
