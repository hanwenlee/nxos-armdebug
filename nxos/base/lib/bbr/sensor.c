
#include "base/drivers/sensors.h"
#include "base/drivers/radar.h"
#include "base/drivers/systick.h"
#include "base/display.h"
#include "base/drivers/ht_compass.h"
#include "base/drivers/ht_accel.h"
#include "base/drivers/motors.h"

#include "base/lib/bbr/sensor.h"

// ultrasonic constants
static char *ultrasound_error="ERR: No USnd! ";
//static char *ultrasound_found="              ";


static U32 systick_100ms=300;


// touch sensor threshold
// pressed: 183; released 1023
static U32 touch_thresh=256;


static U8 ultrasound_errcnt=0;


//Ultrasound Sensor Calibration Offset
static U8 ultrasound_err_offset=3;			// FIXME: Min measurable distance is about 3 cm (1 inch), but it always returns 6
static U8 ultrasound_errstate_count=5;		// Used to reset Detected Distances if too many unknown objects /error

static U32 systick_3ms=3;

void configureSensor(Sensor *sensor, SensorType type, SensorPort port)
{

	sensor->type=type;
	sensor->port=port;

	if (sensor->type==TOUCH || sensor->type==SOUND || sensor->type==LIGHT)
	{
		nx_sensors_analog_enable(sensor->port);
	}
	else if (sensor->type==RADAR)
	{
		nx_radar_init(sensor->port);

		nx_systick_wait_ms(systick_100ms);
		while(nx_radar_detect(sensor->port)==FALSE)
		{

			nx_display_cursor_set_pos(0,1);
			nx_display_string(ultrasound_error);
			nx_systick_wait_ms(systick_100ms);
		}


		nx_radar_set_op_mode(sensor->port,RADAR_MODE_CONTINUOUS);
	}
	else if (sensor->type==COMPASS)
	{


		  ht_compass_init(sensor->port);
		  if( ! ht_compass_detect(sensor->port) ) {
		    nx_display_string("No compass!\n");

		    return;
		  }
		  /*
		  while(nx_avr_get_button() != BUTTON_OK) {
		    nx_display_cursor_set_pos(9, 6);
		    nx_display_string("   ");
		    nx_display_cursor_set_pos(9, 6);
		    nx_display_uint( ht_compass_read_heading(sensor) );
		    nx_systick_wait_ms(100);
		  }
		  */
		  //ht_compass_close(sensor);
	}
	else if (sensor->type==ACCEL)
	{
		  ht_accel_init(sensor->port);

		  if( ! ht_accel_detect(sensor->port) ) {
		    nx_display_string("No accel!\n");
		    return;
		  }

		  /*
		  ht_accel_info(sensor);
		  ht_accel_values values;
		  while(nx_avr_get_button() != BUTTON_OK) {
		    nx_display_cursor_set_pos(3, 5);
		    if ( ! ht_accel_read_values(sensor, &values) ) {
		        nx_display_string("Error reading!");
		        break;
		    }
		    nx_display_string("    ");
		    nx_display_cursor_set_pos(3, 5);
		    nx_display_int( values.x );
		    nx_display_cursor_set_pos(3, 6);
		    nx_display_string("    ");
		    nx_display_cursor_set_pos(3, 6);
		    nx_display_int( values.y );
		    nx_display_cursor_set_pos(3, 7);
		    nx_display_string("    ");
		    nx_display_cursor_set_pos(3, 7);
		    nx_display_int( values.z );
		    nx_systick_wait_ms(100);
		  }

		  ht_accel_close(sensor);
		  */
	}
}

void setUltrasoundErrorOffset(U8 offset)
{
	ultrasound_err_offset=offset;
}
void collect_samples(U8 numberOfReadings, Sensor *sensor1, SensorReading readings1[], Sensor *sensor2, SensorReading readings2[],
					Sensor *sensor3, SensorReading readings3[],Sensor *sensor4, SensorReading readings4[])
{
	int i;

	for (i=0;i<numberOfReadings;i++)
	{
		if (sensor1!=NULL && readings1 !=NULL)
			getSensorReading(sensor1, &readings1[i]);
			//readings1[i]=get_reading(sensor1);

		if (sensor2!=NULL && readings2 !=NULL)
			getSensorReading(sensor2, &readings2[i]);

		if (sensor3!=NULL && readings3 !=NULL)
			getSensorReading(sensor3, &readings3[i]);

		if (sensor4!=NULL && readings4 !=NULL)
			getSensorReading(sensor4, &readings4[i]);



		nx_systick_wait_ms(systick_3ms);
	}
}

void calc_min_max(SensorReading readings[], U8 numberOfSamples, SensorReading *min, SensorReading *max)
{

	if (readings[0].type==LIGHT || readings[0].type==SOUND || readings[0].type==RADAR)
	{
		U8 u8Min=255;
		U8 u8Max=0;
		int i;
		for (i=0;i<numberOfSamples;i++)
		{
			if (readings[i].reading.analog<u8Min)
				u8Min=readings[i].reading.analog;

			if (readings[i].reading.analog>u8Max)
				u8Max=readings[i].reading.analog;
		}

		min->reading.analog=u8Min;
		max->reading.analog=u8Max;


	}

	if (readings[0].type==COMPASS)
	{
		U16 u16Min=65535;
		U16 u16Max=0;
		int i;
		for (i=0;i<numberOfSamples;i++)
		{
			if (readings[i].reading.heading<u16Min)
				u16Min=readings[i].reading.heading;

			if (readings[i].reading.heading>u16Max)
				u16Max=readings[i].reading.heading;
		}

		min->reading.heading=u16Min;
		max->reading.heading=u16Max;
	}

	if (readings[0].type==ACCEL)
	{

		S16 s16Minx=32767;
		S16 s16Maxx=-32768;

		S16 s16Miny=32767;
		S16 s16Maxy=-32768;

		S16 s16Minz=32767;
		S16 s16Maxz=-32768;
		int i;
		for (i=0;i<numberOfSamples;i++)
		{
			if (readings[i].reading.values.x<s16Minx)
				s16Minx=readings[i].reading.values.x;

			if (readings[i].reading.values.x>s16Maxx)
				s16Maxx=readings[i].reading.values.x;


			if (readings[i].reading.values.y<s16Miny)
							s16Miny=readings[i].reading.values.y;

			if (readings[i].reading.values.y>s16Maxy)
				s16Maxy=readings[i].reading.values.y;


			if (readings[i].reading.values.z<s16Minz)
							s16Minz=readings[i].reading.values.z;

			if (readings[i].reading.values.z>s16Maxz)
				s16Maxz=readings[i].reading.values.z;
		}

		min->reading.values.x=s16Minx;
		max->reading.values.x=s16Maxx;

		min->reading.values.y=s16Miny;
		max->reading.values.y=s16Maxy;

		min->reading.values.z=s16Minz;
		max->reading.values.z=s16Maxz;
	}

	min->type=readings->type;
	max->type=readings->type;
}










void get_claw_status(Actuators actuators, U32 *claw_position, U32 *claw_timestamp)
{
	if (actuators.type1==ARM)
		*claw_position=nx_motors_get_tach_count(actuators.port1);

	else if (actuators.type2==ARM)
		*claw_position=nx_motors_get_tach_count(actuators.port2);

	else
		*claw_position=nx_motors_get_tach_count(actuators.port3);

	*claw_timestamp=nx_systick_get_ms();


}

void getSensorReading(Sensor *sensor, SensorReading *sensorReading)
{
	sensorReading->type=sensor->type;

	if (sensor->type==TOUCH)
	{
		U32 touchval;

		touchval=nx_sensors_analog_get(sensor->port);

		// if touched
		if (touchval>touch_thresh)
		{
			// only update if previously released
			if (sensorReading->reading.touched!=FALSE)
			{
				sensorReading->reading.touched=FALSE;

			}
		}
		// released
		else
		{
			// only update if previously touched
			if (sensorReading->reading.touched!=TRUE)
			{
				sensorReading->reading.touched=TRUE;

			}
		}
	}
	else if (sensor->type==LIGHT || sensor->type==SOUND)
	{
		sensorReading->reading.analog=nx_sensors_analog_get_normalized(sensor->port);
	}
	else if (sensor->type==RADAR)
	{
			U8 tmp_errcnt=0;
			U8 tmpDistance=0;

			tmpDistance=nx_radar_read_distance(sensor->port,0);

			// if invalid readings (no object or error)
			if (tmpDistance==RADAR_DIST_NOOBJECT || tmpDistance==RADAR_DIST_ERR)
			{
				tmpDistance=sensorReading->reading.analog;
				tmp_errcnt=ultrasound_errcnt+1;

				if (tmp_errcnt>ultrasound_errstate_count)
				{
					tmp_errcnt=0;
					tmpDistance=RADAR_DIST_ERR;
				}
			}
			else
			{

				tmpDistance=tmpDistance-ultrasound_err_offset;
				tmp_errcnt=0;
			}

			sensorReading->reading.analog=tmpDistance;
			ultrasound_errcnt=tmp_errcnt;
	}
	else if (sensor->type==COMPASS)
	{
		sensorReading->reading.heading=ht_compass_read_heading(sensor->port);
	}
	else if (sensor->type==ACCEL)
	{

		if ( ! ht_accel_read_values(sensor->port, &(sensorReading->reading.values) ))
		{
			nx_display_string("Error reading!");
		}
	}
}


void light_led_enable(Sensor *sensor)
{
	if (sensor->type==LIGHT)
		nx_sensors_analog_digi_set(sensor->port,DIGI0);
}


void light_led_disable(Sensor *sensor)
{
	if (sensor->type==LIGHT)
		nx_sensors_analog_digi_clear(sensor->port,DIGI0);

}


