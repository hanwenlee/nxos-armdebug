
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



#include "linefollower.h"

// behaviors
#include "idle.h"
#include "follow_line.h"




void main(void) {


	int num_behaviors=2;			// number of behaviors
	U32 robot_sched_duration=100;  	// minimum time spent for each main loop (in ms)
	char *title="Line Follower";		// program title to display
	U32 max_iterations=65536;		// maximum number of iterations of main loop
	U32 sched_tick=0;				// store system timer for robot sleep


	// Tone Control Constants
	U32 tone_enabled_freq=2500;		// Hz
	U32 tone_silenced_freq=0;		// Hz
	U32 tone_enabled_dur=100;		// ms


	// wheel control constants
	S8 stop_speed=0;
	S8 fwd_speed=30;
	S8 fastrot_speed=40;
	S8 slowrot_speed=20;

	// Idle Behavior Variables
	bool tone_active=FALSE;
	U32 tone_timestamp=0;

	// light input variable
	U8 sampleSize=5;
	SensorReading light_readings[sampleSize];
	SensorReading light_min;
	SensorReading light_max;



	// Actuator Port Assignments
	Actuators actuators;
	setActuators(&actuators, ARM, RIGHT_WHEEL, LEFT_WHEEL );

	// Sensor Port Assignments and configure
	Sensor lightSensor;



	//current robot actuator state
	ActuatorState current_actuation={MOTOR_INHIBITED_BYTE, FALSE, MOTOR_INHIBITED_BYTE, FALSE, MOTOR_INHIBITED_BYTE, FALSE, ROBOT_STOP, TONE_INHIBITED, TONE_INHIBITED};

	//previous robot actuator state
	ActuatorState previous_actuation={MOTOR_INHIBITED_BYTE, FALSE, MOTOR_INHIBITED_BYTE, FALSE, MOTOR_INHIBITED_BYTE, FALSE, ROBOT_STOP, TONE_INHIBITED, TONE_INHIBITED};

	//behavior_actuations
	ActuatorState behavior_actuations[num_behaviors];

	//array of available states
	ActuatorState availableStates[6]=
	{
		//bbr_behavior_inhibited
		{MOTOR_INHIBITED_BYTE, FALSE, MOTOR_INHIBITED_BYTE, FALSE, MOTOR_INHIBITED_BYTE, FALSE, ROBOT_STOP, TONE_INHIBITED, TONE_INHIBITED},

		//bbr_idle_on
		{MOTOR_INHIBITED_BYTE, FALSE, MOTOR_INHIBITED_BYTE, FALSE, MOTOR_INHIBITED_BYTE, FALSE, ROBOT_STOP, tone_enabled_freq, tone_enabled_dur},

		//bbr_idle_off
		{MOTOR_INHIBITED_BYTE, FALSE, MOTOR_INHIBITED_BYTE, FALSE, MOTOR_INHIBITED_BYTE, FALSE, ROBOT_STOP, tone_silenced_freq, tone_enabled_dur},

		//bbr_follow_line_black
		{MOTOR_INHIBITED_BYTE, FALSE, fwd_speed, FALSE, fwd_speed, FALSE, ROBOT_FWD, tone_silenced_freq, robot_sched_duration},

		//bbr_follow_line_edge
		{MOTOR_INHIBITED_BYTE, FALSE, slowrot_speed, FALSE, stop_speed, FALSE, ROBOT_CCW, tone_silenced_freq, robot_sched_duration},

		//bbr_follow_line_white
		{MOTOR_INHIBITED_BYTE, FALSE, stop_speed, FALSE, fastrot_speed, FALSE, ROBOT_CW, tone_silenced_freq, robot_sched_duration}
	};



	//
	// program initialization
	//
	nx_proginit();
	nx_progtitle(title);
	nx_systick_wait_ms(1000);



	// configure sensors
	configureSensor(&lightSensor, LIGHT, FOUR);
	light_led_enable(&lightSensor);



	// initialize current tick
	sched_tick=nx_systick_get_ms();


	// initialize behavior_actuations
	initActuatorStates(behavior_actuations, num_behaviors);


	while(max_iterations--)
	{

		collect_samples(sampleSize, &lightSensor, light_readings, NULL, NULL, NULL, NULL, NULL, NULL);

		calc_min_max(light_readings, sampleSize, &light_min, &light_max);

		// dispatcher
		bbr_idle(&tone_timestamp, &tone_active,
				behavior_actuations, availableStates);

		bbr_follow_line(&light_min, &light_max,
						behavior_actuations, availableStates);




		// arbiters
		arbiters(num_behaviors, &actuators, &current_actuation, behavior_actuations);

		// actuators
		controllers(&actuators, &current_actuation, &previous_actuation);


		sleep_robot(&sched_tick, robot_sched_duration);

	}



	nx_progshutdown();
}
