/** @file arbiter.h
 *  @author Lee Han Wen
 *  @brief Arbiter of Behavior-Based Robotics Framework
 *   This header defines the routines common to Arbiter Module of Behavior-Based Robotics Framework
 */

#ifndef __BBR_ARBITER_H__
#define __BBR_ARBITER_H__

#include "base/lib/bbr/actuator.h"

/** @addtogroup bbr*/
/*@{*/

/** @defgroup arbiter Arbiter
 *
 * Arbiter of Behavior-Based Robotics Framework
 */
/*@{*/

/**
 * Choose the winner(s) among the behaviors for controlling motors and speaker. Then, copy the winning portion of
 * behavior actuator state into corresponding portion of robot's current actuator state.
 *
 *    @param num_behaviors 			[in]: Total number of behaviors in the program
 *    @param actuators 				[in]: Robot's actuators ( with each motor port set with actuator type)
 *    @param current_state		   [out]: Robot's current actuator state
 *    @param behavior_actuations	[in]: Array of behavior actuator state
 */
void arbiters(int num_behaviors, Actuators *actuators, ActuatorState *current_state, ActuatorState behavior_actuations[]);

/*@}*/
/*@}*/
#endif
