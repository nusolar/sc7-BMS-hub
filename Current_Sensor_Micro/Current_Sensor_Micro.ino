/*  BMS Hub Current Sensor
	Author (A-Z): Francis Chen, Alexander Martin, Wayne Xun
	Date: Mar. 2015

	This is the current sensor code for the BMS Hub. The Arduino communicates via
        SPI to a MAX1142 ADC, which returns the voltage value from a current sensor.
        This voltage corresponds to the current that is being sensed. This code supports
        checking multiple batteries at numerious pins and is set to 1 by default.
	*/

/* Begin battery and temperature constants */

#define NUM_BATTERIES                      32
#define WAIT_TIME_MILLISECONDS            1000ul
#define BATTERY_TEMP_STATUS_OVERHEAT       true
#define BATTERY_TEMP_STATUS_OKAY           false

/* Begin SPI constants */

#define ARDUINO_TEMP_PIN                  4
#define ARDUINO_SYSTEM_CLOCK_DIVIDER      21            // 21 clock divider. Arduino Due runs at 84 MHz, 84/21 = 4 Mhz. ADC runs 0.1 Mhz min, 4.8 Mhz max
#define ADC_SPI_PREFIX                    0b10011000     // This is unipolar mode, despite what the datasheet says


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

// Pin variables
const int    numOfPins    = 1;            // number of pins to be checking
int          pinArray[1]  = {4};          // array containing pin numbers
int          selectPin    = 0;            // pin to be looked at
const byte   CAN_CS_PIN   = 4;
const byte   CAN_INT_PIN  = 5;
const byte   ADC_CS_PIN = 6;
// Global, holds information from the ADC
double  minVoltage = -4.096;
double  maxVoltage =  4.096;
int     bitNum     =     14;

// Data variables
unsigned int   voltageFirstByte  = 0;
unsigned int   voltageSecondByte = 0;
signed   int   voltage           = 0;
double         current           = 0;

// Global timers used to hold millisecond timers
/*unsigned long timeOfLastTemperatureCheck = 0;
unsigned long timeNow                    = 0;
unsigned long timeSinceCheck              = 0;
unsigned long timeToDelay                = 0;*/
Metro cycle_timer(WAIT_TIME_MILLISECONDS);

// CAN controller parameters
const uint16_t BAUD_RATE = 1000;
const byte     FREQ       = 16;

const uint16_t RXM0       = MASK_Sx00;
const uint16_t RXF0       = 0;
const uint16_t RXF1       = 0;

const uint16_t RXM1       = MASK_NONE;
const uint16_t RXF2       = 0;
const uint16_t RXF3       = 0;
const uint16_t RXF4       = 0;
const uint16_t RXF5       = 0;

uint16_t             CANErrors = 0;

// CAN controller object
CAN_IO CanControl(CAN_CS_PIN, CAN_INT_PIN, BAUD_RATE, FREQ);


/*
 *  HELPER FUNCTIONS       *************************************
 */

/* Function Signatures */
void   setupSPI();
void   setupCAN();
void   writeBatteryErrors();
bool   checkBatteryErrors();
int   temperatureCheckLoop();
void  waitForNextCycle();

/*
 * Initializes SPI pins for communication.
 */
inline void setupSPI()
{
  // Initialize the bus for the device on pin ARDUINO_TEMP_PIN
  SPI.begin();
  // Set the clock divider on that pin to ARDUINO_SYSTEM_CLOCK_DIVIDER.
  SPI.setClockDivider(21);                  
  // Set order for bits to be read
  SPI.setBitOrder(MSBFIRST);
  // Set data mode
  SPI.setDataMode(SPI_MODE0);
 pinMode(ADC_CS_PIN,OUTPUT);
 digitalWrite(ADC_CS_PIN,HIGH);
}

inline void setupCAN()
{
  // Initialize CAN
  pinMode(CAN_CS_PIN,OUTPUT);
  pinMode(CAN_INT_PIN,OUTPUT);
 
  CANFilterOpt filters;
  filters.setRB0(RXM0, RXF0, RXF1);
  filters.setRB1(RXM1, RXF2, RXF3, RXF4, RXF5);
 
  CanControl.Setup(filters, &CANErrors);  
}

/*
* Sends request through SPI for voltage
*/
signed int getVoltage(int selectPin)
{
  signed int value = 0;
  
  
 digitalWrite(ADC_CS_PIN,LOW);
  SPI.transfer(ADC_SPI_PREFIX);        // send spi request
  SPI.transfer(0x00);              // send 4 bytes full of zeros
  voltageFirstByte   = SPI.transfer(0x00);        // store into voltage
  voltageSecondByte = SPI.transfer(0x00);
 digitalWrite(ADC_CS_PIN,HIGH);

  value = voltageFirstByte << 8;
  value += voltageSecondByte;
  value = value >> 2; // for Micro
//  value = value << 16;
//  value = value >> 18; for Due
  return value;                               // return voltage
}

/*
 * Converts voltage to current
 */
inline double toCurrent(signed int voltage)
{
  // with user defined voltage min, max, and bit positions
  return (double)voltage * (maxVoltage - minVoltage)/pow(2,bitNum);
}

/* 
 * Delays loop dynamically to allow code to run at next user set frequency
 */
void waitForNextCycle()
{  
  delay(cycle_timer.time_remaining());

  // After the delay, reset timer
  cycle_timer.reset();


}

inline void sendCAN()
{
	 CanControl.Send(BMShub_VoltageCurrent(0,current),TXB0);
}

/*
 *  MAIN FUNCTIONS       *************************************
 */
 
void setup() {
  setupSPI();    // set up and synchronize SPI
  setupCAN();    // set up CAN bus communciation

  // Set up serial output with 9600 baud rate
  Serial.begin(9600);

}

void loop() {
  Serial.println("Sending SPI");
  selectPin = (selectPin + 1) % numOfPins; // Increment selected ADC pin
  voltage = getVoltage(selectPin);
  current = toCurrent(voltage);

  //send current through serial
  Serial.print("Voltage: ");
  Serial.println(voltage,BIN);
  Serial.print("Current: ");
  Serial.println(current);

  //send current through CAN packet
  Serial.println("Sending CAN");
  sendCAN();

  waitForNextCycle();
}
