
#ifndef __NXOS_GRASPER_H__
#define __NXOS_GRASPER_H__

// behaviors (low to high priority)
#define BBR_IDLE			0
#define BBR_FOLLOW_LINE		1
#define BBR_OPEN_CLAWS		2
#define BBR_GRASP			3
#define BBR_MOVE_OBJECT		4
#define BBR_AVOID_OBSTACLE	5

// available actuator states
#define BBR_BEHAVIOR_INHIBITED		0
#define BBR_IDLE_ON					1
#define BBR_IDLE_OFF				2
#define BBR_FOLLOW_LINE_BLACK		3
#define BBR_FOLLOW_LINE_EDGE		4
#define BBR_FOLLOW_LINE_WHITE		5
#define BBR_CLAWS_OPEN				6
#define BBR_CLAWS_CLOSE				7
#define BBR_CLAWS_UNLOCK			8
#define BBR_CLAWS_LOCK				9
#define BBR_MOVE_OBJECT_IDLE		10
#define BBR_MOVE_OBJECT_WAITING		11
#define BBR_MOVE_OBJECT_SEEKING		12
#define BBR_MOVE_OBJECT_FOLLOWING	13
#define BBR_AVOID_OBSTACLE_ON		14
#define BBR_AVOID_OBSTACLE_OFF		15




//name Robot State Enums
enum robot_state_t{
ROBOT_STOP,   	/**< Initial State. */
ROBOT_FWD,      /**< Robot Moving Forward. */
ROBOT_CW ,      /**< Robot Rotating Clockwise */
ROBOT_CCW,      /**< Robot Rotating Counter-Clockwise */
ROBOT_SEEK,		/**< Robot Seeking Motion (mover, tribot only) */
ROBOT_AVOID,	/**< Robot Avoid Obstacle Motion (tribot only) */
};

// Claw Movement State Enums
enum claw_movement_state_t{
	CLAW_STOPPED,
	CLAW_OPENING,
	CLAW_CLOSING
};


// Sound Following State Enums
enum sound_following_state_t{
	SOUND_IDLE,
	SOUND_WAITING,
	SOUND_SEEKING,
	SOUND_FOLLOWING
};
#endif
