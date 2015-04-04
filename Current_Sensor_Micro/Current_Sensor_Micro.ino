/* Begin battery and temperature constants */

#define NUM_BATTERIES                			32
#define WAIT_TIME_MILLISECONDS				1000ul
#define BATTERY_TEMP_STATUS_OVERHEAT 			true
#define BATTERY_TEMP_STATUS_OKAY     			false

/* Begin SPI constants */

#define ARDUINO_TEMP_PIN				4
#define ARDUINO_SYSTEM_CLOCK_DIVIDER			21
#define ADC_SPI_PREFIX					0b10011000 // This is unipolar mode, despite what the datasheet says


/* TempChecker.cpp
 * Wayne Xun
 * Email at WayneXun2017@u.northwestern.edu
 *
 * For MAX1142ACAP/MAX1142BCAP ADC made by MAXIM	
 */

#include <SPI.h>
#include <Metro.h>
#include "sc7-can-libinclude.h"


/*
 *  GLOBAL VARIABLES       *************************************
 */
int numOfPins = 1;						// number of pins to be checking
int pinArray[1] = {4};		                        // array containing pin numbers
int selectPin = 0;						// pin to be looked at

//global, holds information from the ADC
double minVoltage = -4.096;
double maxVoltage = 4.096;
int bitNum = 14;

unsigned int voltageFirstByte = 0;
unsigned int voltageSecondByte = 0;


//global timers used to hold millisecond timers
unsigned long timeOfLastTemperatureCheck = 0;
unsigned long timeNow = 0;
unsigned long timeSinceCheck = 0;
unsigned long timeToDelay = 0;

/*
 *  HELPER FUNCTIONS       *************************************
 */

/* Function  Signatures */
void setupSPI();
void writeBatteryErrors();
bool checkBatteryErrors();
int temperatureCheckLoop();


/*
 * Initializes SPI pins for communication.
 */
void setupSPI()
{
	// Initialize the bus for the device on pin ARDUINO_TEMP_PIN
    SPI.begin();
	// Set the clock divider on that pin to ARDUINO_SYSTEM_CLOCK_DIVIDER 
    SPI.setClockDivider(21);									// 21 clock divider. Arduino Due runs at 84 MHz, 84/21 = 4 Mhz. ADC runs 0.1 Mhz min, 4.8 Mhz max
    // Set order for bits to be read
    SPI.setBitOrder(MSBFIRST);
    // Set data mode
    SPI.setDataMode(SPI_MODE0);
	// Set up serial output with 9600 baud rate
    Serial.begin(9600);		
}

/*
* Sends request through SPI for voltage
*/
double getVoltage(int selectPin)
{
        double voltage;
        signed int value = 0;
	SPI.transfer(ADC_SPI_PREFIX);				// send spi request
	SPI.transfer(0x00);							// send 1 byte of waiting full of zeros
	voltageFirstByte = SPI.transfer(0x00);				// store into voltage
	voltageSecondByte = SPI.transfer(0x00);
   
	value = voltageFirstByte << 8;
        value += voltageSecondByte;
        value = value >> 2;
//        value = value << 16;
//        value = value >> 18;
        Serial.print("Value:  "  );
        Serial.println(value, BIN);
        voltage =  (double)value * (maxVoltage - minVoltage)/pow(2,bitNum); //+ minVoltage;

	return voltage; 															// return voltage
}

double getCurrent(double voltage)
{       double volt_offset = 0, current;
	// add offset according to result of calibration
        if (voltage < 0)          volt_offset = -0.01;      
        else                      volt_offset = 0.005;
        current = (voltage+volt_offset)*100/5 - 1.56;
	return current;
}

/* Delays loop dynamically to allow code to run at next user set frequency
 */
void delayLoop()
{	
  	timeNow = millis();
	timeSinceCheck = timeNow - timeOfLastTemperatureCheck;

	if (timeSinceCheck < WAIT_TIME_MILLISECONDS)		timeToDelay = WAIT_TIME_MILLISECONDS - timeSinceCheck;
	else timeToDelay = 0;
	
        delay(timeToDelay);
	timeOfLastTemperatureCheck = millis();	
}

/*
 *  MAIN FUNCTIONS       *************************************
 */
 
void setup() {
	setupSPI();		// set up and synchronize SPI
}

void loop() {
        double voltage = 0, current = 0;
	selectPin = (selectPin + 1) % numOfPins;
	voltage = getVoltage(selectPin);
	current = getCurrent(voltage);

	//send current through serial
	Serial.print("Voltage: ");
	Serial.println(voltage);
	Serial.print("Current: ");
	Serial.println(current);
	//send current through CAN packet

	delayLoop();				// wait for next loop

}

