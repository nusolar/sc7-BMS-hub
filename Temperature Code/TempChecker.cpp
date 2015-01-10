/* TempChecker.cpp
 * Last Edited 1/10/2015
 * Wayne Xun and Gregory Leung
 * Email at WayneXun2017@u.northwestern.edu and GregoryLeung2016@u.northwestern.edu
 */

#include "Constants.h"

//global, holds the statuses of the temperatures of the batteries
bool batteryTemperatureStatuses[NUM_BATTERIES];

//global timers used to hold millisecond timers
unsigned long timeElapsed = 0;
unsigned long timeOfLastTemperatureCheck = 0;
unsigned long timeSinceLastTemperatureCheck = 0;

/* Function Signatures */
void setupSPI();
void writeBatteryErrors();
int temperatureCheckLoop();

/* Initializes SPI pins for communication.
 * Call this in order to check the temperature sensor
 *  See Constants.h for more information 
 */
void setupSPI()
{
	//Initialize the bus for the device on pin ARDUINO_TEMP_PIN
	SPI.begin(ARDUINO_TEMP_PIN);
	//Set the clock divider on that pin to ARDUINO_SYSTEM_CLOCK_DIVIDER 
	SPI.setClockDivider(ARDUINO_TEMP_PIN, ARDUINO_SYSTEM_CLOCK_DIVIDER);
}

/*  Writes into boolean array @param batterystatuses[]
 *  Each element of the array corresponds to a single temp. sensor.
 *  Each boolean in the array is 0 for not overheated
 *  or 1 for overheated.
 */
void writeBatteryErrors ()
{
	for(int i = 0; i<NUM_BATTERIES; i++)
	{
		//SPI_CONTINUE to keep the chip selected
		//TODO: figure out how the fudge to get data from an arduino
		unsigned char SPIResponse = SPI.transfer(ARDUINO_TEMP_PIN, i, SPI_CONTINUE);
		//TODO: write into boolean array batteryTemperatureStatuses[];
		if (SPIResponse == BATTERY_TEMP_STATUS_OVERHEAT)
		{
			batteryTemperatureStatuses[i] = BATTERY_TEMP_STATUS_OVERHEAT;
		}
		else
		{
			batteryTemperatureStatuses[i] = BATTERY_TEMP_STATUS_OKAY;
		}
	}
	return;
}

/* runs writeBatteryErros if enough time has elapsed since the last battery check
 * @return 0 if not enough time has passed, 1 if succesfully written
 */
int temperatureCheckLoop ()
{
	//TODO run writeBatteryErrors every WAIT_TIME_MILLISECONDS
	unsigned long timeNow = millis();
	timeElapsed = timeNow - timeOfLastTemperatureCheck;

	if (timeElapsed > WAIT_TIME_MILLISECONDS)
	{
		writeBatteryErrors();
		timeOfLastTemperatureCheck = timeNow;
		//TODO: check battery errors
		timeElapsed = 0ul;
		return 1;
	}
	else
	{
		return 0;
	}
}