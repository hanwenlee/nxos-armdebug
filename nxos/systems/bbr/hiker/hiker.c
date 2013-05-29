
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



#include "hiker.h"

// behaviors
#include "idle.h"
#include "hike_mountain.h"
#include "avoid_wall.h"




void main(void) {


	int num_behaviors=3;			// number of behaviors
	U32 robot_sched_duration=100;  	// minimum time spent for each main loop (in ms)
	char *title="Hiker";		// program title to display
	U32 max_iterations=65536;		// maximum number of iterations of main loop
	U32 sched_tick=0;				// store system timer for robot sleep


	// Tone Control Constants
	U32 tone_enabled_freq=2500;		// Hz
	U32 tone_silenced_freq=0;		// Hz
	U32 tone_enabled_dur=100;		// ms


	// wheel control constants
	S8 fwd_speed=25;
	S8 uphill_speed=50;
	S8 left_backup_speed=0;
	S8 right_backup_speed=-20;


	// Idle Behavior Variables
	bool tone_active=FALSE;
	U32 tone_timestamp=0;

	// Hike Mountain Behavior Variables
	SensorReading accel_value;



	// Avoid Wall Behavior Variables
	SensorReading distance;
	SensorReading heading;




	// Actuator Port Assignments
	Actuators actuators;
	setActuators(&actuators, ARM, RIGHT_WHEEL, LEFT_WHEEL );

	// Sensor Port Assignments and configure
	Sensor lightSensor, accelSensor, ulsSensor, compassSensor;



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

		//bbr_hike_mountain_flat
		{MOTOR_INHIBITED_BYTE, FALSE, fwd_speed, FALSE, fwd_speed, FALSE, ROBOT_FWD, tone_silenced_freq, robot_sched_duration},

		//bbr_hike_mountain_uphill
		{MOTOR_INHIBITED_BYTE, FALSE, uphill_speed, FALSE, uphill_speed, FALSE, ROBOT_FWD, tone_silenced_freq, robot_sched_duration},

		//bbr_avoid_wall_turn
		{MOTOR_INHIBITED_BYTE, FALSE, right_backup_speed, FALSE, left_backup_speed, FALSE, ROBOT_FWD, tone_silenced_freq, robot_sched_duration}
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
	configureSensor(&accelSensor, ACCEL, TWO);
	configureSensor(&ulsSensor, RADAR, THREE);
	configureSensor(&compassSensor, COMPASS, ONE);



	// initialize current tick
	sched_tick=nx_systick_get_ms();


	// initialize behavior_actuations
	initActuatorStates(behavior_actuations, num_behaviors);


	while(max_iterations--)
	{



		getSensorReading(&accelSensor, &accel_value);

		getSensorReading(&ulsSensor, &distance);

		getSensorReading(&compassSensor, &heading);

		//displaySensorReadings("range:", &distance, NULL, &accel_value, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, "head:", &heading);

		//if (max_iterations==1)
		//{
		// dispatcher
		bbr_idle(&tone_timestamp, &tone_active,
				behavior_actuations, availableStates);

		bbr_hike_mountain(&accel_value,
						behavior_actuations, availableStates);
		bbr_avoid_wall(&heading, &distance,
						behavior_actuations, availableStates);




		// arbiters
		arbiters(num_behaviors, &actuators, &current_actuation, behavior_actuations);

		// actuators
		controllers(&actuators, &current_actuation, &previous_actuation);

		//}
		sleep_robot(&sched_tick, robot_sched_duration);

	}



	nx_progshutdown();
}
