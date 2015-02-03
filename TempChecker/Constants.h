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

/* Begin ADC constants */
#define ADC_POWER_DOWN                          10010'ARDUINO_TEMP_PIN'
#define ADC_SHORT_ACQUISITION                   10000'ARDUINO_TEMP_PIN'
#define ADC_LONG_ACQUISITION                    10011'ARDUINO_TEMP_PIN'

