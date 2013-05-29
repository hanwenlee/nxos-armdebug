
#include "base/types.h"
#include "base/drivers/systick.h"


#include "base/lib/bbr/helper_routines.h"
#include "base/lib/bbr/actuator.h"

#include "grasper.h"
#include "idle.h"



static U32 idle_tone_interval=500;	// 500 ms,  FIXME: May need to be > TONE_ENABLED_DUR to be safe


void bbr_idle(U32 *tone_timestamp, bool *tone_active, ActuatorState behavior_actuations[], ActuatorState availableStates[])
{
	// if time elapsed
	if (  (time_elapsed(nx_systick_get_ms(),*tone_timestamp,idle_tone_interval)) == TRUE  )
	{
		*tone_timestamp=nx_systick_get_ms();	// update current timestamp

		// Toggle Tone Active status
		if (*tone_active==FALSE)
			*tone_active=TRUE;
		else
			*tone_active=FALSE;

		if (*tone_active==FALSE)
			copyActuatorState(BBR_IDLE, behavior_actuations, availableStates,BBR_IDLE_OFF);
		else
			copyActuatorState(BBR_IDLE, behavior_actuations, availableStates, BBR_IDLE_ON);
	}
}


