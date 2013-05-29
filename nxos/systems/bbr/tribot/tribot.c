
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



#include "tribot.h"

// behaviors
#include "idle.h"
#include "follow_line.h"
#include "open_claws.h"
#include "grasp_object.h"
#include "move_object.h"
#include "avoid_obstacle.h"




void main(void) {


	int num_behaviors=6;			// number of behaviors
	U32 robot_sched_duration=100;  	// minimum time spent for each main loop (in ms)
	char *title="Tribot";		// program title to display
	U32 max_iterations=65536;		// maximum number of iterations of main loop
	U32 sched_tick=0;				// store system timer for robot sleep


	// Tone Control Constants
	U32 tone_enabled_freq=2500;		// Hz
	U32 tone_silenced_freq=0;		// Hz
	U32 tone_enabled_dur=100;		// ms
	U32 tone_obstacle_freq=750;		// Hz


	// wheel control constants
	S8 stop_speed=0;
	S8 fwd_speed=30;
	S8 fastrot_speed=40;
	S8 slowrot_speed=20;
	S8 seek_speed=30;

	// Idle Behavior Variables
	bool tone_active=FALSE;
	U32 tone_timestamp=0;

	// Follow Line Behavior Variables
	U8 sampleSize=5;
	SensorReading light_readings[sampleSize];
	SensorReading light_min;
	SensorReading light_max;


	// Open-Claws Behavior and Grasp-Object Behavior Variables
	SensorReading touched;
	U32 claw_timestamp=0;
	U32 claw_oldtstamp=0;
	U32 claw_position=0;
	U32 claw_oldpos=0;
	bool claw_closed;
	U8 claw_movement;

		// Claw Control Constants
		//    Open and Close Speeds are directional (signed)
		// WARNING: Be conservative with the Open and Close Speeds, since we
		//    run the motors until it hits the limiters and the Tachometer stops changing.
		//    Using too high a speed may pop the claw assembly!
	S8 open_speed=10;
	S8 close_speed=-20;

	// Move-Object Behavior Variables
	SensorReading sound_levels[sampleSize];
	SensorReading sound_min;
	SensorReading sound_max;
	U8 sound_following_state=0;
	U8 sound_level_trigger=0;

	// Avoid Object behavior variables
	bool obstacle_detected=FALSE;
	bool obstacle_tone_active=FALSE;
	U32 obstacle_tone_timestamp=0;
	struct PID_Control pid;
	SensorReading distance;
	distance.reading.analog=0;

	// Actuator Port Assignments
	Actuators actuators;
	setActuators(&actuators, ARM, RIGHT_WHEEL, LEFT_WHEEL );

	// Sensor Port Assignments and configure
	Sensor lightSensor, touchSensor, soundSensor, ulsSensor;



	//current robot actuator state
	ActuatorState current_actuation={MOTOR_INHIBITED_BYTE, FALSE, MOTOR_INHIBITED_BYTE, FALSE, MOTOR_INHIBITED_BYTE, FALSE, ROBOT_STOP, TONE_INHIBITED, TONE_INHIBITED};

	//previous robot actuator state
	ActuatorState previous_actuation={MOTOR_INHIBITED_BYTE, FALSE, MOTOR_INHIBITED_BYTE, FALSE, MOTOR_INHIBITED_BYTE, FALSE, ROBOT_STOP, TONE_INHIBITED, TONE_INHIBITED};

	//behavior_actuations
	ActuatorState behavior_actuations[num_behaviors];

	//array of available states
	ActuatorState availableStates[16]=
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
		{MOTOR_INHIBITED_BYTE, FALSE, stop_speed, FALSE, fastrot_speed, FALSE, ROBOT_CW, tone_silenced_freq, robot_sched_duration},

		// Open Claws
		{open_speed, FALSE, stop_speed, TRUE, stop_speed, TRUE, ROBOT_STOP, tone_silenced_freq, robot_sched_duration},

		// Close Claws (Grasp Object)
		{close_speed, FALSE, stop_speed, TRUE, stop_speed, TRUE, ROBOT_STOP, tone_silenced_freq, robot_sched_duration},

		// Unlock Claws
		{stop_speed, FALSE, stop_speed, TRUE, stop_speed, TRUE, ROBOT_STOP, tone_silenced_freq, robot_sched_duration},

		// Lock Claws
		{stop_speed, TRUE, stop_speed, TRUE, stop_speed, TRUE, ROBOT_STOP, tone_silenced_freq, robot_sched_duration},

		// Move-Object Behavior Actuation Outputs
			// idle
		{close_speed, TRUE, stop_speed, TRUE, stop_speed, TRUE, ROBOT_STOP, TONE_INHIBITED, TONE_INHIBITED},

			// waiting
		{close_speed, TRUE, stop_speed, TRUE, stop_speed, TRUE, ROBOT_STOP, tone_silenced_freq, robot_sched_duration},

			// seeking
		{close_speed, TRUE, seek_speed, TRUE, stop_speed, TRUE, ROBOT_SEEK, tone_silenced_freq, robot_sched_duration},

			// follow
		{close_speed, TRUE, fwd_speed, TRUE, fwd_speed, TRUE, ROBOT_FWD, tone_silenced_freq, robot_sched_duration},

		// Avoid-Obstacle Behavior Actuation Outputs
			// avoid obstacle template, tone on
		{MOTOR_INHIBITED_BYTE, TRUE, stop_speed, TRUE, stop_speed, TRUE, ROBOT_AVOID, tone_obstacle_freq, tone_enabled_dur},

			// avoid obstacle template, tone off
		{MOTOR_INHIBITED_BYTE, TRUE, MOTOR_INHIBITED_BYTE, TRUE, stop_speed, TRUE, ROBOT_AVOID, tone_silenced_freq, tone_enabled_dur}
	};



	//
	// program initialization
	//
	nx_proginit();
	nx_progtitle(title);
	nx_systick_wait_ms(1000);



	// configure sensors
	configureSensor(&touchSensor, TOUCH, ONE);
	configureSensor(&soundSensor, SOUND, TWO);
	configureSensor(&ulsSensor, RADAR, THREE);
	configureSensor(&lightSensor, LIGHT, FOUR);
	light_led_enable(&lightSensor);


	// Open-Claws Behavior and Grasp-Object Behavior initialization
	touched.reading.touched=FALSE;
		// force claw opening on init via Open-Claws behavior
	claw_closed=TRUE;		// claw closed boolean
	claw_oldpos=0;			// Initialize Old Position = 0 (default initial position after nx__motors_init)
	claw_oldtstamp=0;		// Initialize Claw Position Old Timestamp = 0
	claw_movement= CLAW_STOPPED;

	// Avoid-Obstacle Behavior initialization
	distance.reading.analog=RADAR_DIST_ERR;	// initialize to unknown distance
	obstacle_tone_active=FALSE;
	obstacle_detected=FALSE;

		// PID Configuration Parameters
		// FIXME: Need to be changed to actual values
		// KP, KI, KD have the range of 16-bit integer values [MIN_SHORT,MAX_SHORT]
		// This is multiplied by alpha to give 32-bit resolution values
		// alphaKP, alphaKI, alphaKD BEFORE initializing the PID Controller
		// The value of alpha is 2^SCALING_SHIFT, where SCALING_SHIFT is 16
		// which gives alpha = 65536
		//

		 // Applying Kc = 3, T = 100 ms, Pc = 1 s, alpha = 65536
		//#define KP 	127795				// alphaKP = alpha x 0.65 x Kc = 127795
		//#define KI 	25559					// alphaKI = alpha x Kp x T / (0.5 x Pc) = 25559
		//#define KD 	153354				// alphaKD = alpha x Kp x 0.12 x Pc / T = 153354
		//#define STEADY_STATE_THRESHOLD 1	    // 0 = disabled, !0 = Stop PID Controller after x iterations
		// init pid
	initPID(&pid, 127795, 25559, 153354, 1);


	// initialize current tick
	sched_tick=nx_systick_get_ms();


	// initialize behavior_actuations
	initActuatorStates(behavior_actuations, num_behaviors);


	while(max_iterations--)
	{

		collect_samples(sampleSize, &lightSensor, light_readings, &soundSensor, sound_levels, NULL, NULL, NULL, NULL);

		calc_min_max(light_readings, sampleSize, &light_min, &light_max);
		calc_min_max(sound_levels, sampleSize, &sound_min, &sound_max);


		getSensorReading(&touchSensor, &touched);					// concerned about touched or not
		getSensorReading(&ulsSensor, &distance);				// concerned about range in cm

		get_claw_status(actuators, &claw_position, &claw_timestamp);


		// turn off light to save energy if follow line is not supposed to work
		if (touched.reading.touched==FALSE)
		{
			light_led_enable(&lightSensor);
		}
		else
		{
			light_led_disable(&lightSensor);
		}

		// dispatcher
		bbr_idle(&tone_timestamp, &tone_active,
				behavior_actuations, availableStates);

		bbr_follow_line(&light_min, &light_max,
						behavior_actuations, availableStates);

		bbr_open_claws(&claw_movement, &touched, &claw_closed, &claw_timestamp, &claw_oldtstamp,
						&claw_position, &claw_oldpos, &current_actuation,&actuators,
						behavior_actuations, availableStates);

		bbr_grasp_object(&claw_movement, &touched, &claw_closed, &claw_timestamp, &claw_oldtstamp,
				&claw_position, &claw_oldpos, &current_actuation, &actuators,
				behavior_actuations, availableStates);

		bbr_move_object(&sound_min, &sound_max, &touched, claw_closed, &sound_following_state,
						&sound_level_trigger,
						behavior_actuations, availableStates);

		bbr_avoid_obstacle(&obstacle_detected, &distance, &obstacle_tone_timestamp, &obstacle_tone_active, &pid, &actuators,
						behavior_actuations, availableStates);

		// arbiters
		arbiters(num_behaviors, &actuators, &current_actuation, behavior_actuations);

		// actuators
		controllers(&actuators, &current_actuation, &previous_actuation);


		sleep_robot(&sched_tick, robot_sched_duration);

	}



	nx_progshutdown();
}
