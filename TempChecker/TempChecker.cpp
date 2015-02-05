/* TempChecker.cpp
 * Wayne Xun
 * Email at WayneXun2017@u.northwestern.edu
 *
 * For MAX1142ACAP/MAX1142BCAP ADC made by MAXIM	
 */

#include "Constants.h"
#include <SPI.h>


/*
 *  GLOBAL VARIABLES       *************************************
 */
int numOfPins = 2;						// number of pins to be checking
int pinArray[numOfPins] = [0, 1];		// array containing pin numbers
int selectPin = 0;						// pin to be looked at

//global, holds information from the ADC
int minVoltage = -12;
int maxVoltage = 12;
int bitNum = 8;
unsigned int voltage = 0;
double current = 0;

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
	SPI.begin(ARDUINO_TEMP_PIN);												// Initialize the bus for the device on pin ARDUINO_TEMP_PIN
	// Set the clock divider on that pin to ARDUINO_SYSTEM_CLOCK_DIVIDER 
	SPI.setClockDivider(ARDUINO_TEMP_PIN, 21);									// 21 clock divider. Arduino Due runs at 84 MHz, 84/21 = 4 Mhz. ADC runs 0.1 Mhz min, 4.8 Mhz max
    // Set order for bits to be read
    setBitOrder(MSBFIRST);
    // Set data mode
    setDataMode(SPI_MODE0);
}

/*
* Sends request through SPI for voltage
*/
double getVoltage(int selectPin)
{
	SPI.transfer(pinArray[selectPin], ADC_SPI_PREFIX, SPI_CONTINUE);		// send spi request
	SPI.transfer(pinArray[selectPin], 0x00, SPI_CONTINUE);							// send 4 bytes full of zeros
	SPI.transfer(pinArray[selectPin], 0x00, SPI_CONTINUE);
	SPI.transfer(pinArray[selectPin], 0x00, SPI_CONTINUE);
	SPI.transfer(pinArray[selectPin], 0x00, SPI_CONTINUE);
	voltage = SPI.transfer(pinArray[selectPin], 0x00, SPI_CONTINUE);					// store into voltage
	voltage = voltage << 8;
	voltage += SPI.transfer(pinArray[selectPin], 0x00, SPI_LAST);
	voltage = voltage >> 2;
	
	return voltage;															// return voltage
}

double getCurrent(int voltage)
{
	// with user defined voltage min, max, and bit positions
	current =  (double) voltage * (maxVoltage - minVoltage)/(2^bitNum) + minVoltage;
	// return current
	return current;
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
	selectPin = (selectPin + 1) % numOfPins;

	voltage = getVoltage(selectPin);
	current = getCurrent(voltage);
	//send current through CAN packet

//serial.print

	delayLoop();				// wait for next loop
}
