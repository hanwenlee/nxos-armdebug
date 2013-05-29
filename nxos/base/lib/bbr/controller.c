#include "base/types.h"
#include "base/drivers/sound.h"
#include "base/drivers/systick.h"
#include "base/drivers/motors.h"
#include "base/lib/scaffolding/scaffolding.h"
#include "base/display.h"

#include "base/lib/bbr/controller.h"
#include "base/lib/bbr/actuator.h"
#include "base/lib/bbr/helper_routines.h"

static U8 stop_speed=0;
static U32 tone_silenced_freq=0;

static void motor_actuator(int portNum, ActuatorState *current_actuator, ActuatorState *previous_actuation)
{


	S8 inhibited_speed=MOTOR_INHIBITED_BYTE;


	if (portNum==2)
	{
		if (current_actuator->speedC!=previous_actuation->speedC || current_actuator->breakC!= previous_actuation->breakC)
		{
			// Copy Current Settings to Previous Settings
			previous_actuation->speedC=current_actuator->speedC;
			previous_actuation->breakC=current_actuator->breakC;


			if (current_actuator->speedC==stop_speed || current_actuator->speedC==inhibited_speed)
			{
				nx_motors_stop(portNum,current_actuator->breakC);
			}
			else
			{
				nx_motors_rotate(portNum,current_actuator->speedC);
			}

		}

	}
	else if (portNum==1)
	{
		if (current_actuator->speedB!=previous_actuation->speedB || current_actuator->breakB!= previous_actuation->breakB)
		{
			// Copy Current Settings to Previous Settings
			previous_actuation->speedB=current_actuator->speedB;
			previous_actuation->breakB=current_actuator->breakB;


			if (current_actuator->speedB==stop_speed || current_actuator->speedB==inhibited_speed)
			{

				nx_motors_stop(portNum,current_actuator->breakB);
			}
			else
			{
				nx_motors_rotate(portNum,current_actuator->speedB);
			}

		}

	}
	else if (portNum==0)
	{
		if (current_actuator->speedA!=previous_actuation->speedA || current_actuator->breakA!= previous_actuation->breakA)
		{
			// Copy Current Settings to Previous Settings
			previous_actuation->speedA=current_actuator->speedA;
			previous_actuation->breakA=current_actuator->breakA;


			if (current_actuator->speedA==stop_speed || current_actuator->speedA==inhibited_speed)
			{

				nx_motors_stop(portNum,current_actuator->breakA);
			}
			else
			{
				nx_motors_rotate(portNum,current_actuator->speedA);
			}

		}

	}


}

static void speaker_controller(Actuators *actuators, ActuatorState *current_actuator, ActuatorState *previous_actuation)
{
	// if speaker input unchanged
	if (current_actuator->tone_freq==previous_actuation->tone_freq && current_actuator->tone_dur==previous_actuation->tone_dur)
	{
		// skip update if already inhibited
		if (current_actuator->tone_freq!=TONE_INHIBITED && current_actuator->tone_dur!=TONE_INHIBITED)
		{
			// if tone duration expired
			if (time_elapsed(nx_systick_get_ms(),actuators->speaker_timestamp, current_actuator->tone_dur)==TRUE)
			{
				previous_actuation->tone_freq=TONE_INHIBITED;
				previous_actuation->tone_dur=TONE_INHIBITED;

				// update the timestamp
				actuators->speaker_timestamp=nx_systick_get_ms();
			}
		}

	}
	else
	{
		// store new tone in previous actuator state
		previous_actuation->tone_freq=current_actuator->tone_freq;
		previous_actuation->tone_dur=current_actuator->tone_dur;



		if (current_actuator->tone_freq==TONE_INHIBITED)
		{
			// update the timestamp
			actuators->speaker_timestamp=nx_systick_get_ms();
		}
		//Activate Speaker if not Tone Inhibited
		else if (current_actuator->tone_freq!=tone_silenced_freq)
		{
			nx_sound_freq_async(current_actuator->tone_freq, current_actuator->tone_dur);

			// update the timestamp
			actuators->speaker_timestamp=nx_systick_get_ms();
		}




	}


}

static void wheels_controller(Actuators *actuators,ActuatorState *current_actuator, ActuatorState *previous_actuation)
{
	if (actuators->type1==LEFT_WHEEL || actuators->type1==RIGHT_WHEEL)
		motor_actuator(actuators->port1, current_actuator, previous_actuation);

	if (actuators->type2==LEFT_WHEEL || actuators->type2==RIGHT_WHEEL)
			motor_actuator(actuators->port2, current_actuator, previous_actuation);

	if (actuators->type3==LEFT_WHEEL || actuators->type3==RIGHT_WHEEL)
			motor_actuator(actuators->port3, current_actuator, previous_actuation);


	// Update Robot State only
	previous_actuation->curr_state=current_actuator->curr_state;


}



static void arm_controller(Actuators *actuators, ActuatorState *current_actuator, ActuatorState *previous_actuator)
{
	if (actuators->type1==ARM)
		motor_actuator(actuators->port1, current_actuator, previous_actuator);

	if (actuators->type2==ARM)
			motor_actuator(actuators->port2, current_actuator, previous_actuator);

	if (actuators->type3==ARM)
			motor_actuator(actuators->port3, current_actuator, previous_actuator);


}

void controllers(Actuators *actuators, ActuatorState *current_actuator_state, ActuatorState *previous_actuator_state)
{
	speaker_controller(actuators, current_actuator_state, previous_actuator_state);
	wheels_controller(actuators, current_actuator_state, previous_actuator_state);
	arm_controller(actuators, current_actuator_state, previous_actuator_state);
}
