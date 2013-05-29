/** @file controller.h
 *  @author Lee Han Wen
 *  @brief Controller of Behavior-Based Robotics Framework
 *   This header defines the routines common to Controller Module of Behavior-Based Robotics Framework
 */

#ifndef __BBR_CONTROLLER_H__
#define __BBR_CONTROLLER_H__

#include "base/lib/bbr/actuator.h"

/** @addtogroup bbr*/
/*@{*/

/** @defgroup controller Controller
 *
 * Controller of Behavior-Based Robotics Framework
 */
/*@{*/

/**
 * Compare robot's current actuator state with previous actuator state to determine if new actuator command
 * need to be issued to motors and speaker. Update robot's current actuator state and previous actuator state.
 *    @param actuators [in] : Robot's actuators ( with each motor port set with actuator type)
 *    @param current_actuator_state [in/out] : Robot's current actuator state
 *    @param previous_actuator_state [in/out]: Robot's previous actuator state
 */
void controllers(Actuators *actuators, ActuatorState *current_actuator_state, ActuatorState *previous_actuator_state);

/*@}*/
/*@}*/
#endif
