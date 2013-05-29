
#include "base/types.h"
#include "base/drivers/systick.h"
#include "base/lib/scaffolding/scaffolding.h"


#include "base/lib/bbr/helper_routines.h"
#include "base/lib/bbr/actuator.h"
#include "base/lib/bbr/pid.h"

#include "tribot.h"
#include "avoid_obstacle.h"

static S32 wheel_speed_max=30;
static S32 wheel_speed_min=-30;

static U8 obstacle_mindist=10; 	// FIXME: Set appropriate min distance (in cm)
static U8 obstacle_stopdist=20;		// FIXME: Set appropriate distance to stop avoiding obstacle (in cm)

static U32 idle_tone_interval=500;	// 500 ms,  FIXME: May need to be > TONE_ENABLED_DUR to be safe

void bbr_avoid_obstacle(bool *obstacle_detected, SensorReading *ultrasoundval, U32 *obstacle_tone_timestamp,
		bool *obstacle_tone_active, struct PID_Control *pid, Actuators *actuators,
		ActuatorState behavior_actuations[], ActuatorState availableStates[])
{
	S32 output=0;

	if (ultrasoundval->reading.analog<obstacle_mindist)
	{
		//enable behavior because Obstacle Min Distance breached

		if (*obstacle_detected==FALSE)
		{
			setPIDReferenceVal(pid, obstacle_stopdist);	 // Set PID Reference Value to Obstacle Minimum Distance
		}

		output=PIDController(pid, ultrasoundval->reading.analog);

		if (output!=0 || checkPIDEnd(pid)==FALSE)
		{

			if (time_elapsed(nx_systick_get_ms(), *obstacle_tone_timestamp, idle_tone_interval)==TRUE)
			{
				// Has reached target update time interval yet

				// store current timestamp in tone_timestamp
				*obstacle_tone_timestamp=nx_systick_get_ms();

				// toggle obstacle tone active status
				if (*obstacle_tone_active==FALSE)
					*obstacle_tone_active=TRUE;
				else
					*obstacle_tone_active=FALSE;

			}

			if (*obstacle_tone_active==TRUE)
			{
				// True, enable tone
				copyActuatorState(BBR_AVOID_OBSTACLE,behavior_actuations, availableStates, BBR_AVOID_OBSTACLE_ON);
			}
			else
			{
				// false, disable tone
				copyActuatorState(BBR_AVOID_OBSTACLE,behavior_actuations, availableStates, BBR_AVOID_OBSTACLE_OFF);
			}

			// Convert PID Controller output in R4 into Motor Speed in R1
			output=0-output;				// PID Controller output is positive, need negative motor speed for reverse


			if (output>wheel_speed_max)
				output=wheel_speed_max;
			if (output<wheel_speed_min)
				output=wheel_speed_min;

			if (actuators->type1==ARM)
			{
				behavior_actuations[BBR_AVOID_OBSTACLE].speedB=output;
				behavior_actuations[BBR_AVOID_OBSTACLE].speedC=output;
			}
			else if (actuators->type2==ARM)
			{
				behavior_actuations[BBR_AVOID_OBSTACLE].speedA=output;
				behavior_actuations[BBR_AVOID_OBSTACLE].speedC=output;
			}
			else
			{
				behavior_actuations[BBR_AVOID_OBSTACLE].speedA=output;
				behavior_actuations[BBR_AVOID_OBSTACLE].speedB=output;
			}


			*obstacle_detected=TRUE;
		}
		else
		{
			*obstacle_detected=FALSE;
			// Deactivate Behavior Outputs
			copyActuatorState(BBR_AVOID_OBSTACLE,behavior_actuations, availableStates, BBR_BEHAVIOR_INHIBITED);
		}
	}
	else
	{
		// no obstacle yet

		*obstacle_detected=FALSE;

		// Deactivate Behavior Outputs
		copyActuatorState(BBR_AVOID_OBSTACLE,behavior_actuations, availableStates, BBR_BEHAVIOR_INHIBITED);
	}
}





