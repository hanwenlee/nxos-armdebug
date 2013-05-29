
#include "base/types.h"
#include "base/drivers/systick.h"
#include "base/lib/scaffolding/scaffolding.h"


#include "base/lib/bbr/helper_routines.h"
#include "base/lib/bbr/actuator.h"

#include "hiker.h"
#include "avoid_wall.h"


static U8 minDistance=15;
static bool backup=FALSE;
static U16 tmpHeading=0;

void bbr_avoid_wall(SensorReading *heading, SensorReading *distance, ActuatorState behavior_actuations[], ActuatorState availableStates[])

{

	if (backup==FALSE)
	{
		if (distance->reading.analog <minDistance)
		{
			//copyActuatorState(BBR_AVOID_WALL, behavior_actuations, availableStates, BBR_AVOID_WALL_TURN);
			backup=TRUE;
			tmpHeading=heading->reading.heading;
		}
			copyActuatorState(BBR_AVOID_WALL, behavior_actuations, availableStates, BBR_BEHAVIOR_INHIBITED);
	}
	else
	{
		// has decide to backup

		if (heading->reading.heading > tmpHeading)
		{
			if ( (heading->reading.heading-tmpHeading) < 90)
			{
				// not 90 degree yet, turn right
				copyActuatorState(BBR_AVOID_WALL, behavior_actuations, availableStates, BBR_AVOID_WALL_TURN);

			}
			else
			{
				// finish turning 90 degree
				backup=FALSE;
				copyActuatorState(BBR_AVOID_WALL, behavior_actuations, availableStates, BBR_BEHAVIOR_INHIBITED);
			}
		}
		else
		{
			if ( ((heading->reading.heading + 360)-tmpHeading) < 90)
			{
				// not 90 degree yet, turn right
				copyActuatorState(BBR_AVOID_WALL, behavior_actuations, availableStates, BBR_AVOID_WALL_TURN);

			}
			else
			{
				// finish turning 90 degree
				backup=FALSE;
				copyActuatorState(BBR_AVOID_WALL, behavior_actuations, availableStates, BBR_BEHAVIOR_INHIBITED);
			}

		}
	}




}





