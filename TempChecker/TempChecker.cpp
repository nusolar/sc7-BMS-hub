/* TempChecker.cpp
 * Wayne Xun and Gregory Leung
 * Email at WayneXun2017@u.northwestern.edu and GregoryLeung2016@u.northwestern.edu
 */

#include "Constants.h"
#include <SPI.h>


/*
 *  GLOBAL VARIABLES       *************************************
 */

//global, holds the statuses of the temperatures of the batteries
bool batteryTemperatureStatuses[NUM_BATTERIES];

//global timers used to hold millisecond timers
unsigned long timeElapsed = 0;
unsigned long timeNow = 0;
unsigned long timeOfLastTemperatureCheck = 0;
unsigned long timeSinceLastTemperatureCheck = 0;

/*
 *  HELPER FUNCTIONS       *************************************
 */

/* Function Signatures */
void setupSPI();
void writeBatteryErrors();
bool checkBatteryErrors()
bool temperatureCheckLoop();


/*
 * Initializes SPI pins for communication.
 */
void setupSPI()
{
	// Initialize the bus for the device on pin ARDUINO_TEMP_PIN
	SPI.begin(ARDUINO_TEMP_PIN);
	// Set the clock divider on that pin to ARDUINO_SYSTEM_CLOCK_DIVIDER 
	SPI.setClockDivider(ARDUINO_TEMP_PIN, ARDUINO_SYSTEM_CLOCK_DIVIDER);
        // Set order for bits to be read
        setBitOrder(ARDUINO_BIT_ORDER);
        // Set data mode
        setDataMode(ARDUINO_DATA_MODE);
}

/*  Writes into boolean array @param batteryTemperatureStatuses[]
 *  Each element of the array corresponds to a single temp. sensor.
 *  Each boolean in the array is 0 for not overheated
 *  or 1 for overheated.
 */
void writeBatteryErrors ()
{
	for(int i = 0; i < NUM_BATTERIES; i++)
	{
		unsigned char SPIResponse = SPI.transfer(ARDUINO_TEMP_PIN, i);        // some command here to ADC
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

/*  Check boolean array batteryTemperatureStatuses[]
 *  for any true (1) elements. Returns 1 if
 *  there exists any true elements, 0 if none
 */
bool checkBatteryErrors()
{
    for(int i = 0; i < NUM_BATTERIES; i++){
        if (batteryTemperatureStatuses[i]){
            return 1
        }
    }
    return 0
}

/* runs writeBatteryErrors if enough time has elapsed since the last battery check
 * @return 0 if not enough time has passed, 1 if successfully written
 */
bool temperatureCheckLoop ()
{
  	timeNow = millis();
	timeElapsed = timeNow - timeOfLastTemperatureCheck;

	if (timeElapsed > WAIT_TIME_MILLISECONDS)
	{
		writeBatteryErrors();
		timeOfLastTemperatureCheck = millis();
		timeElapsed = 0ul;
		return 1;
	}
	else
	{
		return 0;
	}
}

/*
 *  MAIN FUNCTIONS       *************************************
 */
void setup() {
  void setupSPI();
}

void loop() {
    if (temperatureCheckLoop()){
        checkBatteryErrors();
    }
    void writeBatteryErrors();
    bool checkBatteryErrors();
}
