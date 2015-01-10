/* Constants.h
 * This file holds all relevant constants for TempChecker.cpp.
 * Last Edited 1/10/2015
 * Wayne Xun and Gregory Leung
 * Email at WayneXun2017@u.northwestern.edu and GregoryLeung2016@u.northwestern.edu
 */


/* Begin battery and temperature constants */

#DEFINE NUM_BATTERIES                		32
#DEFINE WAIT_TIME_MILLISECONDS			    1000ul
#DEFINE BATTERY_TEMP_STATUS_OVERHEAT 		true
#DEFINE BATTERY_TEMP_STATUS_OKAY     		false

/* Begin SPI constants */

#DEFINE ARDUINO_TEMP_PIN					4//TODO: find out which pin the temperature sensor is using
#DEFINE ARDUINO_SYSTEM_CLOCK_DIVIDER		21  
//default value setting clock speed to 4 MHz like other arduino boards