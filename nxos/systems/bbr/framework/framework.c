
#include "base/lib/scaffolding/scaffolding.h"
#include "base/drivers/systick.h"
#include "base/types.h"
#include "base/drivers/sensors.h"
#include "base/drivers/radar.h"
#include "base/display.h"



// Behavior-Based Robotics Framework
#include "base/lib/bbr/helper_routines.h"
#include "base/lib/bbr/actuator.h"
#include "base/lib/bbr/controller.h"
#include "base/lib/bbr/arbiter.h"
#include "base/lib/bbr/pid.h"
#include "base/lib/bbr/sensor.h"



#include "framework.h"

// behaviors
#include "idle.h"




void main(void) {


	int num_behaviors=1;			// number of behaviors
	U32 robot_sched_duration=100;  	// minimum time spent for each main loop (in ms)
	char *title="Framework";		// program title to display
	U32 max_iterations=65536;		// maximum number of iterations of main loop
	U32 sched_tick=0;				// store system timer for robot sleep


	// Tone Control Constants
	U32 tone_enabled_freq=2500;		// Hz
	U32 tone_silenced_freq=0;		// Hz
	U32 tone_enabled_dur=100;		// ms



	// Idle Behavior Variables
	bool tone_active=FALSE;
	U32 tone_timestamp=0;



	// Actuator Port Assignments
	Actuators actuators;
	setActuators(&actuators, ARM, RIGHT_WHEEL, LEFT_WHEEL );

	// Sensor Port Assignments and configure



	//current robot actuator state
	ActuatorState current_actuation={MOTOR_INHIBITED_BYTE, FALSE, MOTOR_INHIBITED_BYTE, FALSE, MOTOR_INHIBITED_BYTE, FALSE, ROBOT_STOP, TONE_INHIBITED, TONE_INHIBITED};

	//previous robot actuator state
	ActuatorState previous_actuation={MOTOR_INHIBITED_BYTE, FALSE, MOTOR_INHIBITED_BYTE, FALSE, MOTOR_INHIBITED_BYTE, FALSE, ROBOT_STOP, TONE_INHIBITED, TONE_INHIBITED};

	//behavior_actuations
	ActuatorState behavior_actuations[num_behaviors];

	//array of available states
	ActuatorState availableStates[3]=
	{
		//bbr_behavior_inhibited
		{MOTOR_INHIBITED_BYTE, FALSE, MOTOR_INHIBITED_BYTE, FALSE, MOTOR_INHIBITED_BYTE, FALSE, ROBOT_STOP, TONE_INHIBITED, TONE_INHIBITED},

		//bbr_idle_on
		{MOTOR_INHIBITED_BYTE, FALSE, MOTOR_INHIBITED_BYTE, FALSE, MOTOR_INHIBITED_BYTE, FALSE, ROBOT_STOP, tone_enabled_freq, tone_enabled_dur},

		//bbr_idle_off
		{MOTOR_INHIBITED_BYTE, FALSE, MOTOR_INHIBITED_BYTE, FALSE, MOTOR_INHIBITED_BYTE, FALSE, ROBOT_STOP, tone_silenced_freq, tone_enabled_dur}
	};



	//
	// program initialization
	//
	nx_proginit();
	nx_progtitle(title);
	nx_systick_wait_ms(1000);



	// configure sensors




	// initialize current tick
	sched_tick=nx_systick_get_ms();


	// initialize behavior_actuations
	initActuatorStates(behavior_actuations, num_behaviors);


	while(max_iterations--)
	{


		// dispatcher
		bbr_idle(&tone_timestamp, &tone_active,
				behavior_actuations, availableStates);




		// arbiters
		arbiters(num_behaviors, &actuators, &current_actuation, behavior_actuations);

		// actuators
		controllers(&actuators, &current_actuation, &previous_actuation);


		sleep_robot(&sched_tick, robot_sched_duration);

	}



	nx_progshutdown();
}
