/* Begin battery and temperature constants */

#define NUM_BATTERIES                			32
#define WAIT_TIME_MILLISECONDS				1000ul
#define BATTERY_TEMP_STATUS_OVERHEAT 			true
#define BATTERY_TEMP_STATUS_OKAY     			false

/* Begin SPI constants */

#define ARDUINO_TEMP_PIN				4
#define ARDUINO_SYSTEM_CLOCK_DIVIDER			21
#define ADC_SPI_PREFIX					0b10011000


/* TempChecker.cpp
 * Wayne Xun
 * Email at WayneXun2017@u.northwestern.edu
 *
 * For MAX1142ACAP/MAX1142BCAP ADC made by MAXIM	
 */

#include <SPI.h>
#include "sc7-can-libinclude.h"


/*
 *  GLOBAL VARIABLES       *************************************
 */
int numOfPins 		= 1;						// number of pins to be checking
int pinArray[1] 	= {4};		                // array containing pin numbers
int selectPin 		= 0;						// pin to be looked at
const byte CS_PIN 	= 4;
const byte INT_PIN 	= 5;

//global, holds information from the ADC
int minVoltage 						= -12;
int maxVoltage 						= 12;
int bitNum 							= 14;
unsigned int voltageFirstByte 		= 0;
unsigned int voltageSecondByte 		= 0;
unsigned int voltage 				= 0;
double current 						= 0;

//global timers used to hold millisecond timers
unsigned long timeOfLastTemperatureCheck = 0;
unsigned long timeNow 				     = 0;
unsigned long timeSinceCheck 		     = 0;
unsigned long timeToDelay 				 = 0;

//can controller parameters
const uint16_t BAUD_RATE = 1000;
const byte     FREQ      = 16;
const uint16_t RXM0      = MASK_NONE;
const uint16_t RXM1      = MASK_NONE;
const uint16_t RXF0      = MASK_NONE;
const uint16_t RXF1      = MASK_NONE;
const uint16_t RXF2      = MASK_NONE;
const uint16_t RXF3      = MASK_NONE;
const uint16_t RXF4      = MASK_NONE;
const uint16_t RXF5      = MASK_NONE;
byte 		   CANErrors = 0

//can controller
CAN_IO CanControl(CS_PIN, INTERRUPT_PIN, BAUD_RATE, FREQ);

/*
 *  HELPER FUNCTIONS       *************************************
 */

/* Function Signatures */
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
	SPI.begin(ARDUINO_TEMP_PIN);
	// Set the clock divider on that pin to ARDUINO_SYSTEM_CLOCK_DIVIDER 
	SPI.setClockDivider(ARDUINO_TEMP_PIN, 21);									// 21 clock divider. Arduino Due runs at 84 MHz, 84/21 = 4 Mhz. ADC runs 0.1 Mhz min, 4.8 Mhz max
    // Set order for bits to be read
    SPI.setBitOrder(ARDUINO_TEMP_PIN, MSBFIRST);
    // Set data mode
    SPI.setDataMode(ARDUINO_TEMP_PIN, SPI_MODE0);

	// Set up serial output with 9600 baud rate
	Serial.begin(9600);

	// Initialize CAN
	CANFilterOpt filters;
  	filters.setRB0(RXM0, RXF0, RXF1);
  	filters.setRB1(RXM1, RXF2, RXF3, RXF4, RXF5);
  	CanControl.Setup(filters, &CANErrors);		
}

/*
* Sends request through SPI for voltage
*/
double getVoltage(int selectPin)
{
	SPI.transfer(pinArray[selectPin], ADC_SPI_PREFIX, SPI_CONTINUE);				// send spi request
	SPI.transfer(pinArray[selectPin], 0x00, SPI_CONTINUE);							// send 1 bytes full of zeros
	voltageFirstByte = SPI.transfer(pinArray[selectPin], 0x00, SPI_CONTINUE);				// store into voltage
	voltageSecondByte = SPI.transfer(pinArray[selectPin], 0x00, SPI_LAST);

	voltageFirstByte = voltageFirstByte << 8;
	voltageFirstByte += voltageSecondByte;
	voltageSecondByte = voltageSecondByte >> 2;
	
	return voltageSecondByte;															// return voltage
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

inline void sendCAN()
{
	CanControl.Send(BMShub_VoltageCurrent(0,current),TXB0);
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

	//send current through serial
	Serial.print('Voltage: ');
	Serial.println(voltage);
	Serial.print('Current: ');
	Serial.println(current);
	//send current through CAN packet
	sendCAN();

//serial.print

	delayLoop();				// wait for next loop
}

