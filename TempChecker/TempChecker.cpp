/* TempChecker.cpp
 * Wayne Xun
 * Email at WayneXun2017@u.northwestern.edu
 */

#include "Constants.h"
#include <SPI.h>


/*
 *  GLOBAL VARIABLES       *************************************
 */

//global, holds information from the ADC
unsigned int voltage = 0;
unsigned double current = 0;

//global timers used to hold millisecond timers
unsigned long timeOfLastTemperatureCheck = 0;
unsigned long timeNow = 0;
unsigned long timeSinceCheck = 0;
unsigned long timeToDelay = 0;

/*
 *  HELPER FUNCTIONS       *************************************
 */

/* Function Signatures */
void setupSPI();
void writeBatteryErrors();
bool checkBatteryErrors()
int temperatureCheckLoop();


/*
 * Initializes SPI pins for communication.
 */
void setupSPI()
{
	// Initialize the bus for the device on pin ARDUINO_TEMP_PIN
	SPI.begin(ARDUINO_TEMP_PIN);
	// Set the clock divider on that pin to ARDUINO_SYSTEM_CLOCK_DIVIDER 
	// SPI.setClockDivider(ARDUINO_TEMP_PIN, SPI_CLOCK_DIV4); // 4 is default
        // Set order for bits to be read
        setBitOrder(MSBFIRST);
        // Set data mode
        setDataMode(SPI_MODE0);
		
		// send synchronization command here
}
/*
* Sends request through SPI for voltage
*/
int getVoltage(int selection)
{
	// if selection = 1, elseif selection = 2
	// send spi request
	// send 4 bytes full of zeros
	// store into voltage
	// return voltage
}

double getCurrent(double voltage)
{
	// with user defined voltage min, max, and bit positions
	// double current = int * (max - min)/(2^senseBits) + min;
	// return current
}

/* Delays loop dynamically to allow code to run at next user set frequency
 */
void delayLoop()
{	
  	timeNow = millis();
	timeSinceCheck = timeNow - timeOfLastTemperatureCheck;

	if (timeSinceCheck > WAIT_TIME_MILLISECONDS)
	{
		timeToDelay = WAIT_TIME_MILLISECONDS - timeSinceCheck;
	}
	else
	{
		timeToDelay = 0;
	}

	timeOfLastTemperatureCheck = timeNow + timeToDelay;
	delay(timeToDelay);
}

/*
 *  MAIN FUNCTIONS       *************************************
 */
void setup() {
	setupSPI();		// set up and synchronize SPI
}

void loop() {
	if (selection == 1){ selection = 0 };			// alternate ADC to check
	else if(){ selection = 1 };

	voltage = getVoltage(selection);
	current = getCurrent(voltage);
	//send current through CAN packet

	delayLoop();				// wait for next loop
}
