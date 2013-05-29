
#ifndef __NXOS_FOLLOW_LINE_H__
#define __NXOS_FOLLOW_LINE_H__
#include "base/lib/bbr/actuator.h"
#include "base/lib/bbr/sensor.h"

void bbr_follow_line(SensorReading *light_min, SensorReading *light_max, ActuatorState behavior_actuations[], ActuatorState availableStates[]);
#endif
