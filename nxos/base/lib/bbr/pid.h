/** @file pid.h
 *  @author Lee Han Wen
 *  @brief PID Control of Behavior-Based Robotics Framework
 *   This header defines the routines common to handle PID Controller in Behavior-Based Robotics Framework
 */
#ifndef __BBR_PID_H__
#define __BBR_PID_H__

#include "base/types.h"


/**
 * PID Controller Data Structure
 */
struct PID_Control
{
	U32 alphaKP;		///< proportional factor
	U32 alphaKI;		///< integral factor
	U32 alphaKD;		///< derivative factor
	S32 err_max;		///< limit for err
	S32 errsum_max;		///< limit for err_sum
	U32 steady_state_thresh;		///< threshold for determining steady state
    // state variables
	S32 err_sum;		///< sum[e(k)]|(k=0..n)
	S32 y_prev;		///< y(n-1)
	U32 steady_state_count;		///< count of error within steady_state_thresh
	S32 ref_val;		///< reference (target/set point) value y0
};

/**
 * Initialize PID Controller
 *    @param pid			 	[out]: PID Controller
 *    @param alphaKP			 [in]: Proportional factor
 *    @param alphaKI			 [in]: Integral factor
 *    @param alphaKD			 [in]: Derivative factor
 *    @param steady_state_thresh [in]: Threshold for determining steady state
 */
void initPID(struct PID_Control *pid, U32 alphaKP, U32 alphaKI, U32 alphaKD, S32 steady_state_thresh);

/**
 * Set PID Controller with output reference value
 *    @param pid			 	[out]: PID Controller
 *    @param reference			 [in]: Output reference value
 */
void setPIDReferenceVal(struct PID_Control *pid, S32 reference);

/**
 * Check if PID Controller has ended
 *    @param pid			 	[in]: PID Controller
 *	  @return TRUE(has ended), FALSE(has not ended yet)
 */
bool checkPIDEnd(struct PID_Control *pid);

/**
 * Return PID Controller output value
 *    @param pid			 	[in]: PID Controller
 *    @param systemStatus		[in]: feedback value (such distance measured by ultrasonic sensor)
 *    @return PID Controller output value
 */
S32 PIDController(struct PID_Control *pid, U8 systemStatus);
#endif
