
#ifndef __NXOS_OPEN_CLAWS_H__
#define __NXOS_OPEN_CLAWS_H__
#include "base/lib/bbr/actuator.h"
#include "base/lib/bbr/sensor.h"

void bbr_open_claws(U8 *claw_movement, SensorReading *touched, bool *claw_closed, U32 *claw_timestamp,
		U32 *claw_oldtstamp, U32 *claw_position, U32 *claw_oldpos, ActuatorState *current_state, Actuators *actuators,
		ActuatorState behavior_actuations[], ActuatorState availableStates[]);


#endif
