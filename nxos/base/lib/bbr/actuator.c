

#include "base/lib/bbr/helper_routines.h"
#include "base/lib/bbr/actuator.h"

static void inhibited(U8 behaviorNum, ActuatorState behavior_actuations[])
{
	behavior_actuations[behaviorNum].speedA=MOTOR_INHIBITED_BYTE;
	behavior_actuations[behaviorNum].breakA=FALSE;

	behavior_actuations[behaviorNum].speedB=MOTOR_INHIBITED_BYTE;
	behavior_actuations[behaviorNum].breakB=FALSE;

	behavior_actuations[behaviorNum].speedC=MOTOR_INHIBITED_BYTE;
	behavior_actuations[behaviorNum].breakC=FALSE;

	behavior_actuations[behaviorNum].tone_freq=TONE_INHIBITED;
	behavior_actuations[behaviorNum].tone_dur=TONE_INHIBITED;

}

static void setState(U8 stateNum, ActuatorState actuations[], S8 speedA, bool breakA,
		S8 speedB, bool breakB, S8 speedC, bool breakC, U8 curr_state, U32 tone_freq, U32 tone_dur)
{
	actuations[stateNum].speedA=speedA;
	actuations[stateNum].breakA=breakA;

	actuations[stateNum].speedB=speedB;
	actuations[stateNum].breakB=breakB;

	actuations[stateNum].speedC=speedC;
	actuations[stateNum].breakC=breakC;

	actuations[stateNum].curr_state=curr_state;

	actuations[stateNum].tone_freq=tone_freq;
	actuations[stateNum].tone_dur=tone_dur;

}

void initActuatorStates(ActuatorState behavior_actuations[], U8 behaviorNum)
{
	U8 i=0;
	for (i=0;i< behaviorNum;i++)
	{
		inhibited(i, behavior_actuations);

	}

}
void setActuatorState(U8 stateNum, ActuatorState actuations[], S8 speedA, bool breakA,
		S8 speedB, bool breakB, S8 speedC, bool breakC, U8 curr_state, U32 tone_freq, U32 tone_dur)
{

	setState(stateNum, actuations, speedA, breakA, speedB, breakB, speedC, breakC, curr_state, tone_freq, tone_dur);

}

void setActuators(Actuators *actuators, ActuatorType actuatorAType,ActuatorType actuatorBType, ActuatorType actuatorCType )
{

	actuators->type1=actuatorAType;
	actuators->port1=A;		//a

	actuators->type2=actuatorBType;
	actuators->port2=B;		// b

	actuators->type3=actuatorCType;
	actuators->port3=C;

	actuators->speaker_timestamp=0;
}

void copyActuatorState(int behaviorNum, ActuatorState behavior_actuations[], ActuatorState state[], int stateNum)
{
	behavior_actuations[behaviorNum].speedA=state[stateNum].speedA;
	behavior_actuations[behaviorNum].breakA=state[stateNum].breakA;

	behavior_actuations[behaviorNum].speedB=state[stateNum].speedB;
	behavior_actuations[behaviorNum].breakB=state[stateNum].breakB;

	behavior_actuations[behaviorNum].speedC=state[stateNum].speedC;
	behavior_actuations[behaviorNum].breakC=state[stateNum].breakC;

	behavior_actuations[behaviorNum].curr_state=state[stateNum].curr_state;

	behavior_actuations[behaviorNum].tone_freq=state[stateNum].tone_freq;
	behavior_actuations[behaviorNum].tone_dur=state[stateNum].tone_dur;


}
