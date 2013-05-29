#include "base/types.h"
#include "base/drivers/systick.h"
#include "base/display.h"

#include "base/lib/bbr/helper_routines.h"
#include "base/lib/bbr/sensor.h"

static U32 dur=1000;

bool time_elapsed(U32 curr_time,U32 prev_time,U32 interval)
{
	if (curr_time<prev_time+interval)
		return FALSE;
	else
		return TRUE;
}


void sleep_robot(U32 *sched_tick, U32 duration)
{

	// keep looping when not time up yet
	while(!time_elapsed(nx_systick_get_ms(), *sched_tick, duration));

	*sched_tick=nx_systick_get_ms();
}


static void printReading(char *string, SensorReading *reading)
{
	if (string!=NULL)
		nx_display_string(string);

	if (reading->type==TOUCH)
	{

		if (reading->reading.touched==TRUE)
			nx_display_string(": pressed");
		else
			nx_display_string(": released");
	}

	else if (reading->type==SOUND || reading->type==RADAR || reading->type==LIGHT)
	{

		nx_display_string(": ");

		nx_display_uint(reading->reading.analog);

		if (reading->type==RADAR)
			nx_display_string("cm");

	}
	else if (reading->type==COMPASS)
	{

		nx_display_string(": ");

		nx_display_uint(reading->reading.heading);

	}

	else if (reading->type==ACCEL)
	{

		nx_display_string("X:");
		nx_display_int(reading->reading.values.x);

		nx_display_end_line();

		nx_display_string("Y:");
		nx_display_int(reading->reading.values.y);

		nx_display_end_line();

		nx_display_string("Z:");
		nx_display_int(reading->reading.values.z);

	}
}
void displaySensorReadings(char *string1, SensorReading *reading1, char *string2, SensorReading *reading2, char *string3, SensorReading *reading3,
		char *string4, SensorReading *reading4, char *string5, SensorReading *reading5, char *string6, SensorReading *reading6,
		char *string7, SensorReading *reading7, char *string8, SensorReading *reading8)
{
	nx_display_clear();

	nx_display_cursor_set_pos(0,0);
	printReading(string1, reading1);

	nx_display_cursor_set_pos(0,1);
	printReading(string2, reading2);

	nx_display_cursor_set_pos(0,2);
	printReading(string3, reading3);

	nx_display_cursor_set_pos(0,3);
	printReading(string4, reading4);


	nx_display_cursor_set_pos(0,4);
	printReading(string5, reading5);

	nx_display_cursor_set_pos(0,5);
	printReading(string6, reading6);

	nx_display_cursor_set_pos(0,6);
	printReading(string7, reading7);

	nx_display_cursor_set_pos(0,7);
	printReading(string8, reading8);


	nx_systick_wait_ms(dur);
}

void setDisplayDur(U32 duration)
{
	dur=duration;
}
