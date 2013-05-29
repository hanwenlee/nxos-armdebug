
#ifndef __NXOS_MOVE_OBJECT_H__
#define __NXOS_MOVE_OBJECT_H__
#include "base/lib/bbr/actuator.h"
#include "base/lib/bbr/sensor.h"

void bbr_move_object(SensorReading *sound_min, SensorReading *sound_max, SensorReading *touched,
		bool claw_closed, U8 *sound_following_state, U8 *sound_level_trigger,
		ActuatorState behavior_actuations[],
		ActuatorState availableStates[]);
#endif
