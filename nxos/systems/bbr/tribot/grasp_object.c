
#include "base/types.h"
#include "base/drivers/systick.h"
#include "base/lib/scaffolding/scaffolding.h"


#include "base/lib/bbr/helper_routines.h"
#include "base/lib/bbr/actuator.h"

#include "tribot.h"
#include "grasp_object.h"

static S8 stop_speed=0;
static U32 tacho_thresh=0;	// Need value of <= 1 for reliability of detecting limits
static U32 clawpos_check_interval=100;  // 100 ms,  Keep CLAWPOS_CHECK_INTERVAL <= ROBOT_SCHED_DURATION to avoid going out of sync

static bool has_claw_started_moving(Actuators *actuators, ActuatorState *current_state)
{
	S8 motor_inhibited=MOTOR_INHIBITED_BYTE;

	// not moving
	if (actuators->type1==ARM)
	{
		if (current_state->speedA==motor_inhibited || current_state->speedA==stop_speed)
		{
			return FALSE;
		}
		else
			return TRUE;
	}
	else if (actuators->type2==ARM)
	{
		if (current_state->speedB==motor_inhibited || current_state->speedB==stop_speed)
		{
			return FALSE;
		}
		else
			return TRUE;
	}
	else
	{
		if (current_state->speedC==motor_inhibited || current_state->speedC==stop_speed)
		{
			return FALSE;
		}
		else
			return TRUE;
	}

}

static bool has_claw_fully_stopped(U32 *claw_timestamp, U32 *claw_oldtstamp, U32 *claw_position, U32 *claw_oldpos)
{
	U32 tmpDifference=0;

	if (time_elapsed(*claw_timestamp, *claw_oldtstamp, clawpos_check_interval)==TRUE)
	{
		// update previous timestamp = current timestamp
		*claw_oldtstamp=*claw_timestamp;

		if (*claw_position>*claw_oldpos)
			tmpDifference=*claw_position-*claw_oldpos;
		else
			tmpDifference=*claw_oldpos-*claw_position;

		// update previous claw position = current claw position
		*claw_oldpos=*claw_position;

		// (difference > TACHO_THRESH) ? FALSE : TRUE
		if (tmpDifference>tacho_thresh)
			return FALSE;
		else
			return TRUE;


	}

	return FALSE;

}


void bbr_grasp_object(U8 *claw_movement, SensorReading *touched, bool *claw_closed, U32 *claw_timestamp,
		U32 *claw_oldtstamp, U32 *claw_position, U32 *claw_oldpos, ActuatorState *current_state, Actuators *actuators,
		ActuatorState behavior_actuations[], ActuatorState availableStates[])
{


	// if claw stopped
	if (*claw_movement==CLAW_STOPPED)
	{

		//If Not Touched AND Claw Closed
		if(touched->reading.touched==TRUE && *claw_closed==FALSE)
		{
			*claw_movement=CLAW_CLOSING;
			// start opening claw
			copyActuatorState(BBR_GRASP, behavior_actuations, availableStates, BBR_CLAWS_UNLOCK);
		}
		else
		{
			// Not Claw Opening Movement, deactivate Behavior and exit
			copyActuatorState(BBR_GRASP, behavior_actuations, availableStates, BBR_BEHAVIOR_INHIBITED);
		}


	}
	// if claw is closing
	else if (*claw_movement==CLAW_CLOSING)
	{
		// if claw not moving yet
		if (has_claw_started_moving(actuators, current_state)==FALSE)
		{
			copyActuatorState(BBR_GRASP, behavior_actuations, availableStates, BBR_CLAWS_CLOSE);

		}
		else
		{
			// if closing complete
			if (has_claw_fully_stopped(claw_timestamp, claw_oldtstamp, claw_position, claw_oldpos)==TRUE)
			{

				*claw_closed=TRUE;				// update claw closed status
				*claw_movement=CLAW_STOPPED;	// update claw movement status
				copyActuatorState(BBR_GRASP, behavior_actuations, availableStates, BBR_CLAWS_LOCK);

			}
		}

	}
	else
	{
		// deactivate Behavior and exit
		copyActuatorState(BBR_GRASP, behavior_actuations, availableStates, BBR_BEHAVIOR_INHIBITED);
	}



}

