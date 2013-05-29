
#ifndef __NXOS_LINEFOLLOWER_H__
#define __NXOS_LINEFOLLOWER_H__

// behaviors (low to high priority)
#define BBR_IDLE				0
#define BBR_HIKE_MOUNTAIN		1
#define BBR_AVOID_WALL			2

// available actuator states
#define BBR_BEHAVIOR_INHIBITED		0
#define BBR_IDLE_ON					1
#define BBR_IDLE_OFF				2
#define BBR_HIKE_MOUNTAIN_FLAT		3
#define BBR_HIKE_MOUNTAIN_UPHILL	4
#define BBR_AVOID_WALL_TURN			5






//name Robot State Enums
enum robot_state_t{
ROBOT_STOP,   	/**< Initial State. */
ROBOT_FWD,      /**< Robot Moving Forward. */
ROBOT_CW ,      /**< Robot Rotating Clockwise */
ROBOT_CCW,      /**< Robot Rotating Counter-Clockwise */
ROBOT_SEEK,		/**< Robot Seeking Motion (mover, tribot only) */
ROBOT_AVOID,	/**< Robot Avoid Obstacle Motion (tribot only) */
};


#endif
