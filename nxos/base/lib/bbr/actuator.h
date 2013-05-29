/** @file actuator.h
 *  @author Lee Han Wen
 *  @brief Actuators of Behavior-Based Robotics Framework
 *   This header defines the routines common to handle actuators(motors)
 *   and robot's actuator state in Behavior-Based Robotics Framework
 */

#ifndef __BBR_ACTUATOR_H__
#define __BBR_ACTUATOR_H__

#include "base/types.h"
#include "base/lib/bbr/actuator.h"

/** @addtogroup bbr*/
/*@{*/

/** @defgroup actuator Actuator
 *
 * Actuators of Behavior-Based Robotics Framework
 */
/*@{*/


#define MOTOR_INHIBITED 0xFFFFFF80	/**< Speed value(in word) for motor inhibited. */
#define MOTOR_INHIBITED_BYTE (MOTOR_INHIBITED & 0xFF)		/**< Speed value(in byte) for motor inhibited. */
#define TONE_INHIBITED 0x80000000	/**< Tone value for speaker inhibited. */


/**
 *  Type of role by a motor
 */
typedef enum{
	LEFT_WHEEL,		///< the motor is controlling a left wheel
	RIGHT_WHEEL,	///< the motor is controlling a right wheel
	ARM,			///< the motor is controlling an arm
	NO_ACTUATOR		///< no motor connected
}ActuatorType;


/**
 *  Motor port's label
 */
typedef enum{
	A,		///< port A
	B,		///< port B
	C		///< port C
}ActuatorPort;


/**
 * 3 motors with associated roles and port numbers, and a temporary timer value(to be used by controller)
 */
typedef struct{
	ActuatorType type1;		///< Role by first motor
	ActuatorPort port1;		///< Port number of first motor

	ActuatorType type2;		///< Role by second motor
	ActuatorPort port2;		///< Port number of second motor

	ActuatorType type3;		///< Role by third motor
	ActuatorPort port3;		///< Port number of third motor

	U32 speaker_timestamp;	///< Store timer value for speaker controller

}Actuators;

/**
 * A robot actuator state
 */
typedef struct
{
	S8 speedA;		///< Speed for motor A
	bool breakA;	///< For motor A, hard braking if TRUE, coasting stop if FALSE.

	S8 speedB;		///< Speed for motor B
	bool breakB;	///< For motor B, hard braking if TRUE, coasting stop if FALSE.

	S8 speedC;		///< Speed for motor C
	bool breakC;	///< For motor C, hard braking if TRUE, coasting stop if FALSE.

	U8 curr_state;	///< Current state of the robot.

	U32 tone_freq;	///< Frequency of tone to be played by speaker.
	U32 tone_dur;	///< Duration of the tone to be played.

}ActuatorState;

/**
 * Initialize the array of behavior actuator state to be inhibited
 *    @param behavior_actuations   [out] : Array of behavior actuator state
 *    @param behaviorNum 			[in] : Total number of behaviors
 */
void initActuatorStates(ActuatorState behavior_actuations[], U8 behaviorNum);

/**
 * Set stateNum(index number) element of array of actuator state
 *    @param stateNum   	[in]: Element's index number of array of actuator to be set
 *    @param actuations    [out]: array of actuator state
 *    @param speedA 		[in]: Speed for motor A
 *    @param breakA 		[in]: For motor A, hard braking if TRUE, coasting stop if FALSE
 *    @param speedB 		[in]: Speed for motor B
 *    @param breakB  		[in]: For motor B, hard braking if TRUE, coasting stop if FALSE
 *    @param speedC  		[in]: Speed for motor C
 *    @param breakC  		[in]: For motor C, hard braking if TRUE, coasting stop if FALSE
 *    @param curr_state  	[in]: Current state of the robot
 *    @param tone_freq  	[in]: Frequency of tone to be played by speaker
 *    @param tone_dur 		[in]: Duration of the tone to be played
 */
void setActuatorState(U8 stateNum, ActuatorState actuations[], S8 speedA, bool breakA,
		S8 speedB, bool breakB, S8 speedC, bool breakC, U8 curr_state, U32 tone_freq, U32 tone_dur);

/**
 * Set roles of the 3 motors and initialize the speaker timestamp value
 *    @param actuators		[out]: Settings of 3 motors with associated roles and port numbers,
 *    and an initialized temporary timer value(to be used by controller)
 *    @param actuatorAType	[in]: Role of motor A
 *    @param actuatorBType	[in]: Role of motor B
 *    @param actuatorCType	[in]: Role of motor C
 */
void setActuators(Actuators *actuators, ActuatorType actuatorAType,ActuatorType actuatorBType, ActuatorType actuatorCType );

/**
 * Copy index N of array of available actuator state into index N of array of behavior actuator state
 *    @param behaviorNum			 [in]: Total number of behaviors
 *    @param behavior_actuations	[out]: Array of behavior actuator state
 *    @param state					 [in]: Array of available actuator state
 *    @param stateNum				 [in]: Index number of available actuator state to be copied
 */
void copyActuatorState(int behaviorNum, ActuatorState behavior_actuations[], ActuatorState state[], int stateNum);

/*@}*/
/*@}*/

#endif
