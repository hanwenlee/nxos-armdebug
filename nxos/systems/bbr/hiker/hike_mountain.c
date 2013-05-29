
#include "base/types.h"
#include "base/drivers/systick.h"
#include "base/lib/scaffolding/scaffolding.h"


#include "base/lib/bbr/helper_routines.h"
#include "base/lib/bbr/actuator.h"

#include "hiker.h"
#include "hike_mountain.h"



static S16 uphill_threshold=284;

void bbr_hike_mountain(SensorReading *accelValue, ActuatorState behavior_actuations[], ActuatorState availableStates[])

{

	if (accelValue->reading.values.z   >uphill_threshold)
		copyActuatorState(BBR_HIKE_MOUNTAIN, behavior_actuations, availableStates, BBR_HIKE_MOUNTAIN_UPHILL);
	else
		copyActuatorState(BBR_HIKE_MOUNTAIN, behavior_actuations, availableStates, BBR_HIKE_MOUNTAIN_FLAT);




}





