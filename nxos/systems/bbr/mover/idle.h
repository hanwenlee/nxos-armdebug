

#ifndef __NXOS_IDLE_H__
#define __NXOS_IDLE_H__
#include "base/lib/bbr/actuator.h"

void bbr_idle(U32 *tone_timestamp, bool *tone_active, ActuatorState behavior_actuations[], ActuatorState availableStates[]);

#endif
