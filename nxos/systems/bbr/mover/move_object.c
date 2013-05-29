
#include "base/types.h"
#include "base/drivers/systick.h"
#include "base/lib/scaffolding/scaffolding.h"


#include "base/lib/bbr/helper_routines.h"
#include "base/lib/bbr/actuator.h"

#include "move_object.h"
#include "mover.h"

// Sound Sensor Threshold (0-100) range */
// Sound Sensor Readings: No Sound: 98+; Loud Sound: towards 0 */
// FIXME: Needs calibration based on ambient background noise, etc. */
static U8 loud_min=1;
static U8 loud_max=69;
static U8 soft_min=70;
static U8 soft_max=84;
static U8 quiet_min=85;
static U8 quiet_max=100;


// Sound Following Constants
//  (Sound Level Trigger is updated every ROBOT_SCHED_DURATION interval)
//  The increments and decrements should have some relationship to the
//  Sound Following Thresholds since the deltas are incremented every main loop.
//
static U8 sound_unknown_decr=1;
static U8 sound_quiet_normal_decr=1;
static U8 sound_quiet_following_decr=5;
static U8 sound_soft_incr=5;
static U8 sound_loud_incr=10;

//	Sound Following Thresholds
//    Hysterisis is implemented
static U8 following_thresh_incr=50;
static U8 following_thresh_decr=40;
static U8 seeking_thresh_incr=30;
static U8 seeking_thresh_decr=10;
static U8 waiting_thresh_incr=5;
static U8 waiting_thresh_decr=1;



static void sound_update_state_output(bool isIncreased, U8 *sound_following_state, U8 *sound_level_trigger,
		ActuatorState behavior_actuations[], ActuatorState availableStates[])
{
	U8 num_sound_states=4;
	if (*sound_following_state>=num_sound_states)
		*sound_following_state=num_sound_states-1;

	// if increased
	if (isIncreased==TRUE)
	{
		if (*sound_following_state==SOUND_IDLE)
		{
			if(*sound_level_trigger>=waiting_thresh_incr)
			{
				*sound_following_state=SOUND_WAITING;
			}
		}
		else if (*sound_following_state==SOUND_WAITING)
		{
			if(*sound_level_trigger>=seeking_thresh_incr)
			{
				*sound_following_state=SOUND_SEEKING;
			}
		}
		else if (*sound_following_state==SOUND_SEEKING)
		{
			if(*sound_level_trigger>=following_thresh_incr)
			{
				*sound_following_state=SOUND_FOLLOWING;
			}
		}
		else if (*sound_following_state==SOUND_FOLLOWING)
		{
			if(*sound_level_trigger>=following_thresh_incr)
			{
				*sound_following_state=SOUND_FOLLOWING;
			}
		}


	}
	else
	{
		// if decreased


		if (*sound_following_state==SOUND_IDLE)
		{
			if(*sound_level_trigger<=waiting_thresh_decr)
			{
				*sound_following_state=SOUND_IDLE;
			}
		}
		else if (*sound_following_state==SOUND_WAITING)
		{
			if(*sound_level_trigger<=waiting_thresh_decr)
			{
				*sound_following_state=SOUND_IDLE;
			}
		}
		else if (*sound_following_state==SOUND_SEEKING)
		{
			if(*sound_level_trigger<=seeking_thresh_decr)
			{
				*sound_following_state=SOUND_WAITING;
			}
		}
		else if (*sound_following_state==SOUND_FOLLOWING)
		{
			if(*sound_level_trigger<=following_thresh_decr)
			{
				*sound_following_state=SOUND_SEEKING;
			}
		}
	}

	// update sound behavior output
	if (*sound_following_state==SOUND_IDLE)
	{
		copyActuatorState(BBR_MOVE_OBJECT, behavior_actuations, availableStates, BBR_MOVE_OBJECT_IDLE);
	}
	else if (*sound_following_state==SOUND_WAITING)
	{
		copyActuatorState(BBR_MOVE_OBJECT, behavior_actuations, availableStates, BBR_MOVE_OBJECT_WAITING);
	}
	else if (*sound_following_state==SOUND_SEEKING)
	{
		copyActuatorState(BBR_MOVE_OBJECT, behavior_actuations, availableStates, BBR_MOVE_OBJECT_SEEKING);
	}
	else if (*sound_following_state==SOUND_FOLLOWING)
	{
		copyActuatorState(BBR_MOVE_OBJECT, behavior_actuations, availableStates, BBR_MOVE_OBJECT_FOLLOWING);
	}

}

void bbr_move_object(SensorReading *sound_min, SensorReading *sound_max, SensorReading *touched,
		bool claw_closed, U8 *sound_following_state, U8 *sound_level_trigger,
		ActuatorState behavior_actuations[],
		ActuatorState availableStates[])
{
	U32 tmp=0;

	// if touched and claw closed
	if(touched->reading.touched==TRUE && claw_closed)
	{
		// check sound level
		if (loud_min <=sound_min->reading.analog && loud_max>=sound_max->reading.analog)
		{
			// sound_loud_handler

			tmp=*sound_level_trigger+sound_loud_incr;

			if (tmp>255)
				*sound_level_trigger=255;	 //Clamp to 255 if exceeded 8-bits
			else
				*sound_level_trigger=tmp;

			sound_update_state_output(TRUE, sound_following_state, sound_level_trigger,behavior_actuations,availableStates);

		}
		else if (soft_min<=sound_min->reading.analog && soft_max>=sound_max->reading.analog)
		{
			// sound_soft_handler
			tmp=*sound_level_trigger+sound_soft_incr;

			if (tmp>255)
				*sound_level_trigger=255;	 //Clamp to 255 if exceeded 8-bits
			else
				*sound_level_trigger=tmp;

			sound_update_state_output(TRUE, sound_following_state, sound_level_trigger,behavior_actuations,availableStates);
		}
		else if (quiet_min<=sound_min->reading.analog && quiet_max>=sound_max->reading.analog)
		{
			// sound_quiet_handler


			// if in sound following state
			if (*sound_following_state==SOUND_FOLLOWING)
			{
				if (*sound_level_trigger-sound_quiet_following_decr<0)
				{
					*sound_level_trigger=0;				// Clamp to 0 if negative
				}
				else
				{
					*sound_level_trigger=*sound_level_trigger-sound_quiet_following_decr;
				}

			}
			else
			{
				if (*sound_level_trigger-sound_quiet_normal_decr<0)
				{
					*sound_level_trigger=0;				// Clamp to 0 if negative
				}
				else
				{
					*sound_level_trigger=*sound_level_trigger-sound_quiet_normal_decr;
				}
			}


			sound_update_state_output(FALSE, sound_following_state, sound_level_trigger,behavior_actuations,availableStates);

		}
		else
		{
			// sound_unknown_handler
			if (*sound_level_trigger-sound_unknown_decr<0)
			{
				*sound_level_trigger=0;				// Clamp to 0 if negative
			}
			else
			{
				*sound_level_trigger=*sound_level_trigger-sound_unknown_decr;
			}
			sound_update_state_output(FALSE, sound_following_state, sound_level_trigger,behavior_actuations,availableStates);
		}

	}
	else
	{
		// If Not Claw Closed, deactivate Behavior Outputs and exit
		copyActuatorState(BBR_MOVE_OBJECT, behavior_actuations, availableStates, BBR_BEHAVIOR_INHIBITED);
	}



}





