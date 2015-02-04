/* Constants.h
 * This file holds all relevant constants for TempChecker.cpp.
 * Wayne Xun and Gregory Leung
 * Email at WayneXun2017@u.northwestern.edu and GregoryLeung2016@u.northwestern.edu
 */


/* Begin battery and temperature constants */

#define NUM_BATTERIES                			32
#define WAIT_TIME_MILLISECONDS					1000ul
#define BATTERY_TEMP_STATUS_OVERHEAT 			true
#define BATTERY_TEMP_STATUS_OKAY     			false

/* Begin SPI constants */

#define ARDUINO_TEMP_PIN						4
#define ARDUINO_SYSTEM_CLOCK_DIVIDER			21
#define ADC_SPI_PREFIX							0b10011000

