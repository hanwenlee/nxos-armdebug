/** @file helper_routines.h
 *  @author Lee Han Wen
 *  @brief Helper routines of Behavior-Based Robotics Framework
 *   This header defines the common routines used in Behavior-Based Robotics Framework
 */
#ifndef __BBR_HELPER_ROUTINES_H__
#define __BBR_HELPER_ROUTINES_H__

#include "base/types.h"
#include "base/lib/bbr/sensor.h"

/** @addtogroup bbr*/
/*@{*/

/** @defgroup helperRoutines Helper Routines
 *
 * Helper routines of Behavior-Based Robotics Framework
 */
/*@{*/

/**
 * Check if the timer value has expired
 *    @param curr_time    [in] : Current timer value
 *    @param prev_time 	  [in] : Previous timer value
 *    @param interval 	  [in] : Amount of time(ms) for the previous timer value to be expired
 *    @return TRUE(has expired), FALSE(not expired yet)
 */
bool time_elapsed(U32 curr_time,U32 prev_time,U32 interval);

/**
 * Ensure each main loop spent a minimum amount of time
 *    @param sched_tick   [in] : Timer value at the beginning of main loop
 *    @param duration 	  [in] : Minimum time(ms) to spent in the main loop
 */
void sleep_robot(U32 *sched_tick, U32 duration);

/**
 * Set the duration for the next update of display on screen
 *    @param duration   [in] : Duration for the next update of display on screen
 */
void setDisplayDur(U32 duration);

/**
 * Display sensor readings
 *    @param string1    		[in] : Text to be displayed at the begining of row 1
 *    @param reading1 			[in] : reading to be displayed on row 1 based on its sensor type
 *    @param string2    		[in] : Text to be displayed at the begining of row 2
 *    @param reading2 			[in] : reading to be displayed on row 2 based on its sensor type
 *    @param string3    		[in] : Text to be displayed at the begining of row 3
 *    @param reading3 			[in] : reading to be displayed on row 3 based on its sensor type
 *    @param string4    		[in] : Text to be displayed at the begining of row 4
 *    @param reading4 			[in] : reading to be displayed on row 4 based on its sensor type
 *    @param string5    		[in] : Text to be displayed at the begining of row 5
 *    @param reading5 			[in] : reading to be displayed on row 5 based on its sensor type
 *    @param string6    		[in] : Text to be displayed at the begining of row 6
 *    @param reading6 			[in] : reading to be displayed on row 6 based on its sensor type
 *    @param string7    		[in] : Text to be displayed at the begining of row 7
 *    @param reading7 			[in] : reading to be displayed on row 7 based on its sensor type
 *    @param string8    		[in] : Text to be displayed at the begining of row 8
 *    @param reading8 			[in] : reading to be displayed on row 8 based on its sensor type
 */
void displaySensorReadings(char *string1, SensorReading *reading1, char *string2, SensorReading *reading2, char *string3, SensorReading *reading3,
		char *string4, SensorReading *reading4, char *string5, SensorReading *reading5, char *string6, SensorReading *reading6,
		char *string7, SensorReading *reading7, char *string8, SensorReading *reading8);

/*@}*/
/*@}*/
#endif
