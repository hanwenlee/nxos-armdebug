
#include "base/types.h"
#include "base/drivers/systick.h"
#include "base/lib/scaffolding/scaffolding.h"


#include "base/lib/bbr/helper_routines.h"
#include "base/lib/bbr/actuator.h"

#include "linefollower.h"
#include "follow_line.h"

// Light Sensor Color Detection Intensity (0-100) ranges */
// Light Sensor Raw Readings: White: 470-520; Edge: 540-580; Black: 680-700 */
// Changed the range so that it will be a smooth transition from White to Black */
static U8 white_min=43;
static U8 white_max=51;
static U8 edge_min=52;
static U8 edge_max=58;
static U8 black_min=59;
static U8 black_max=68;



void bbr_follow_line(SensorReading *light_min, SensorReading *light_max, ActuatorState behavior_actuations[], ActuatorState availableStates[])

{

		if (light_min->reading.analog >=white_min && light_max->reading.analog <=white_max)
		{
			copyActuatorState(BBR_FOLLOW_LINE, behavior_actuations, availableStates, BBR_FOLLOW_LINE_WHITE);
		}
		else if (light_min->reading.analog >=edge_min && light_max->reading.analog <=edge_max)
		{
			copyActuatorState(BBR_FOLLOW_LINE, behavior_actuations, availableStates, BBR_FOLLOW_LINE_EDGE);
		}
		else if (light_min->reading.analog >=black_min && light_max->reading.analog<=black_max)
		{
			copyActuatorState(BBR_FOLLOW_LINE, behavior_actuations, availableStates, BBR_FOLLOW_LINE_BLACK);
		}
		else
		{
			copyActuatorState(BBR_FOLLOW_LINE, behavior_actuations, availableStates, BBR_BEHAVIOR_INHIBITED);
		}

}





