
#ifndef __BBR_HIKE_MOUNTAIN__
#define __BBR_HIKE_MOUNTAIN__
#include "base/lib/bbr/actuator.h"
#include "base/lib/bbr/sensor.h"

void bbr_hike_mountain(SensorReading *accelValue, ActuatorState behavior_actuations[], ActuatorState availableStates[]);
#endif
