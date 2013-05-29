/** @file sensor.h
 *  @author Lee Han Wen
 *  @brief Sensor Readings of Behavior-Based Robotics Framework
 *   This header defines the routines common to Sensor Readings Module of
 *    Behavior-Based Robotics Framework
 */

#ifndef __BBR_SENSOR_H__
#define __BBR_SENSOR_H__

#include "base/types.h"
#include "base/drivers/ht_accel.h"

#include "base/lib/bbr/actuator.h"


/** @addtogroup bbr*/
/*@{*/

/** @defgroup sensorReading Sensor Readings
 *
 *   Sensor Readings of Behavior-Based Robotics Framework
 */
/*@{*/

#define RADAR_DIST_ERR	255	/**< Distance value for error reading by ultrasonic sensor. */
#define RADAR_DIST_NOOBJECT	0	/**< Distance value for no object detected ultrasonic sensor. */

/**
 * Type of sensor
 */
typedef enum{
	TOUCH,		///< touch sensor
	SOUND,		///< sound sensor
	LIGHT,		///< light sensor
	RADAR,		///< ultrasonic sensor
	ACCEL,		///< accelerometer sensor
	COMPASS		///< compass sensor
}SensorType;

/**
 * Sensor port number
 */
typedef enum{
	ONE,		///< Sensor port 1
	TWO,		///< Sensor port 2
	THREE,		///< Sensor port 3
	FOUR		///< Sensor port 4
}SensorPort;

/**
 * Sensor settings with associated sensor type and sensor port number
 */
typedef struct{
	SensorType type;	///< Sensor type
	SensorPort port;	///< Sensor port number

}Sensor;


/**
 * Sensor reading with associated sensor type
 */
typedef struct{
	SensorType type;	///< Sensor type

	/**
	 * reading for sound, light, ultrasonic, touch, compass or accelerometer sensor
	 */
	union{
	U8 analog;		///< reading for sound sensor , light sensor or ultrasonic sensor
	bool touched;	///< reading for touch sensor
	U16 heading;	///< reading for compass sensor
	ht_accel_values values;	 ///< reading for accelerometer sensor
	}reading;

}SensorReading;

/**
 * Set sensor with associated sensor type and port number. Then, initialize or enable the physical sensor based on its type.
 *    @param sensor		[out]: Sensor data structure associated with sensor type and port number
 *    @param type		 [in]: Sensor type
 *    @param port		 [in]: Sensor port number
 */
void configureSensor(Sensor *sensor, SensorType type, SensorPort port);

/**
 * Set the value of ultrasonic sensor's error offset
 *    @param offset		[in]: ultrasonic sensor 's error offset
 */
void setUltrasoundErrorOffset(U8 offset);


/**
 * Capture a reading from the sensor
 *    @param sensor		 		 [in]: Sensor that captures the reading
 *    @param sensorReading		[out]: Reading captured with associated sensor type
 */
void getSensorReading(Sensor *sensor, SensorReading *sensorReading);


/**
 * Capture a number of samples from sensor(s)
 *    @param numberOfReadings		 [in]: Number of samples to collect for each sensor
 *    @param sensor1				 [in]: First sensor
 *    @param readings1				[out]: Array of samples collected from first sensor
 *    @param sensor2				 [in]: Second sensor
 *    @param readings2				[out]: Array of samples collected from second sensor
 *    @param sensor3				 [in]: Third sensor
 *    @param readings3				[out]: Array of samples collected from third sensor
 *    @param sensor4				 [in]: Fourth sensor
 *    @param readings4				[out]: Array of samples collected from fourth sensor
 */
void collect_samples(U8 numberOfReadings, Sensor *sensor1, SensorReading readings1[], Sensor *sensor2, SensorReading readings2[],
					Sensor *sensor3, SensorReading readings3[],Sensor *sensor4, SensorReading readings4[]);

/**
 * Find out the maximum and minimum values of the samples
 *    @param readings		 		 [in]: Array of samples collected
 *    @param numberOfSamples		 [in]: Number of samples collected
 *    @param min		 			 [out]: Minimum values of the samples
 *    @param max					 [out]: Maximum value of the samples
 */
void calc_min_max(SensorReading readings[], U8 numberOfSamples, SensorReading *min, SensorReading *max);

/**
 * Update the claw position with tachnometer counter value and claw_timestamp with current timer value
 *    @param actuators		 		 [in]: Settings of 3 motors with associated roles and port number, and temporary timer value(to be used by controller)
 *    @param claw_position		 	[out]: Tachnometer counter value of the arm (claws)
 *    @param claw_timestamp			[out]: Timer value of the last update of claw_position
 */
void get_claw_status(Actuators actuators, U32 *claw_position, U32 *claw_timestamp);

/**
 * Turn on the light sensor's light
 *    @param sensor   [in] : sensor with associated sensor type and port number
 */
void light_led_enable(Sensor *sensor);

/**
 * Turn off the light sensor's light
 *    @param sensor   [in] : sensor with associated sensor type and port number
 */
void light_led_disable(Sensor *sensor);

/*@}*/
/*@}*/
#endif
