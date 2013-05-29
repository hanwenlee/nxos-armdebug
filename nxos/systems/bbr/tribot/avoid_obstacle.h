
#ifndef __NXOS_AVOID_OBSTACLE_H__
#define __NXOS_AVOID_OBSTACLE_H__
#include "base/lib/bbr/actuator.h"
#include "base/lib/bbr/pid.h"
#include "base/lib/bbr/sensor.h"

void bbr_avoid_obstacle(bool *obstacle_detected, SensorReading *ultrasoundval, U32 *obstacle_tone_timestamp,
		bool *obstacle_tone_active, struct PID_Control *pid, Actuators *actuators,
		ActuatorState behavior_actuations[], ActuatorState availableStates[]);
#endif
