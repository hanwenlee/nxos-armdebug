
#include "base/lib/bbr/arbiter.h"
#include "base/lib/bbr/actuator.h"

static void speaker_arbiter(int num_behaviors, ActuatorState *state, ActuatorState behavior_actuations[])
{
	int i;

	for (i=num_behaviors-1;i>=0;i--)
	{
		// Tone is Inhibited if both Tone and Duration are TONE_INHIBITED
		if (behavior_actuations[i].tone_freq!=TONE_INHIBITED || behavior_actuations[i].tone_dur!=TONE_INHIBITED || i==0)
		{
			state->tone_freq=behavior_actuations[i].tone_freq;
			state->tone_dur=behavior_actuations[i].tone_dur;
			break;
		}
	}
}


static void wheels_arbiter(int num_behaviors, Actuators *actuators, ActuatorState *state, ActuatorState behavior_actuations[])
{
	int i=0;


	S8 inhibited_speed=MOTOR_INHIBITED_BYTE;

	for (i=num_behaviors-1;i>=0;i--)
	{
		// Movement is Inhibited if either of the Movement Speed is MOTOR_INHIBITED
		if ( 	( (actuators->type1==LEFT_WHEEL && actuators->type2==RIGHT_WHEEL ) || (actuators->type1==RIGHT_WHEEL && actuators->type2==LEFT_WHEEL ))
				&&
				( (behavior_actuations[i].speedA!=inhibited_speed && behavior_actuations[i].speedB!=inhibited_speed) || i==0 )
			)
		{
			state->speedA=behavior_actuations[i].speedA;
			state->breakA=behavior_actuations[i].breakA;

			state->speedB=behavior_actuations[i].speedB;
			state->breakB=behavior_actuations[i].breakB;

			state->curr_state=behavior_actuations[i].curr_state;


			break;
		}
		else if ( 	( (actuators->type2==LEFT_WHEEL && actuators->type3==RIGHT_WHEEL ) || (actuators->type2==RIGHT_WHEEL && actuators->type3==LEFT_WHEEL ))
					&&
					( (behavior_actuations[i].speedC!=inhibited_speed && behavior_actuations[i].speedB!=inhibited_speed) || i==0 )
				)
		{
			state->speedC=behavior_actuations[i].speedC;
			state->breakC=behavior_actuations[i].breakC;

			state->speedB=behavior_actuations[i].speedB;
			state->breakB=behavior_actuations[i].breakB;

			state->curr_state=behavior_actuations[i].curr_state;


			break;
		}

		else if ( 	( (actuators->type1==LEFT_WHEEL && actuators->type3==RIGHT_WHEEL ) || (actuators->type1==RIGHT_WHEEL && actuators->type3==LEFT_WHEEL ))
					&&
					( (behavior_actuations[i].speedA!=inhibited_speed && behavior_actuations[i].speedC!=inhibited_speed) || i==0 )
				)
		{
			state->speedA=behavior_actuations[i].speedA;
			state->breakA=behavior_actuations[i].breakA;

			state->speedC=behavior_actuations[i].speedC;
			state->breakC=behavior_actuations[i].breakC;

			state->curr_state=behavior_actuations[i].curr_state;


			break;
		}
	}
}

static void arm_arbiter(int num_behaviors, Actuators *actuators, ActuatorState *state, ActuatorState behavior_actuations[])
{
	int i=0;



	S8 inhibited_speed=MOTOR_INHIBITED_BYTE;

	for (i=num_behaviors-1;i>=0;i--)
	{

		// Movement is Inhibited if either of the Movement Speed is MOTOR_INHIBITED

		if ( actuators->type1==ARM && (behavior_actuations[i].speedA!=inhibited_speed || i==0 ))
		{
			state->speedA=behavior_actuations[i].speedA;
			state->breakA=behavior_actuations[i].breakA;


			break;
		}
		if ( actuators->type2==ARM && (behavior_actuations[i].speedB!=inhibited_speed || i==0 ))
		{
			state->speedB=behavior_actuations[i].speedB;
			state->breakB=behavior_actuations[i].breakB;


			break;
		}
		if ( actuators->type3==ARM && (behavior_actuations[i].speedC!=inhibited_speed || i==0 ))
		{
			state->speedC=behavior_actuations[i].speedC;
			state->breakC=behavior_actuations[i].breakC;


			break;
		}
	}
}

void arbiters(int num_behaviors, Actuators *actuators, ActuatorState *current_state, ActuatorState behavior_actuations[])
{
	speaker_arbiter(num_behaviors, current_state, behavior_actuations);
	wheels_arbiter(num_behaviors, actuators, current_state, behavior_actuations);
	arm_arbiter(num_behaviors, actuators, current_state, behavior_actuations);
}
