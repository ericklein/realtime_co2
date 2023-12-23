/*
  Project Name:   realtime_co2
  Description:    non-secret configuration data
*/

#ifndef CONFIG_H
#define CONFIG_H

// Configuration Step 1: Create and/or configure secrets.h. Use secrets_template.h as guide to create secrets.h

// Configuration Step 2: Set debug parameters
// comment out to turn off; 1 = summary, 2 = verbose
#define DEBUG 1

// simulate SCD40 sensor operations, returning random but plausible values
// comment out to turn off
// #define SENSOR_SIMULATE
const uint16_t sensorTempMin =      1500; // will be divided by 100.0 to give floats
const uint16_t sensorTempMax =      2500;
const uint16_t sensorHumidityMin =  500; // will be divided by 100.0 to give floats
const uint16_t sensorHumidityMax =  9500;
const uint16_t sensorCO2Min =       400;
const uint16_t sensorCO2Max =       3000;  

// Configuration Step 3: Set network data endpoints
// #define MQTT 		    // log sensor data to M/QTT broker
// #define HASSIO_MQTT  // And, if MQTT enabled, with Home Assistant too?
#define INFLUX	      // Log data to InfluxDB server
// #define DWEET        // Post info to Dweet

// Configuration Step 4: Set battery parameters, if applicable
// If LC709203F detected on i2c, define battery pack based on settings curve from datasheet
// #define BATTERY_APA 0x08 // 100mAH
// #define BATTERY_APA 0x0B // 200mAH
#define BATTERY_APA 0x10 // 500mAH
// #define BATTERY_APA 0x19 // 1000mAH
// #define BATTERY_APA 0x1D // 1200mAH
// #define BATTERY_APA 0x2D // 2000mAH
// #define BATTERY_APA 0x32 // 2500mAH
// #define BATTERY_APA 0x36 // 3000mAH

// Configuration variables that change rarely

// Network
// max connection attempts to network services
const uint8_t networkConnectAttemptLimit = 3;
// seconds between network service connect attempts
const uint8_t networkConnectAttemptInterval = 10;

// Time
// NTP time parameters
// const char* networkNTPAddress = "pool.ntp.org";
#define networkNTPAddress "pool.ntp.org"
const String networkTimeZone = "PST8PDT,M3.2.0,M11.1.0"; // America/Los_Angeles
const String weekDays[7] = { "Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday" };

// Data endpoints
#ifdef INFLUX
  #define INFLUX_ENV_MEASUREMENT "weather"  // Used for environmental sensor data
  #define INFLUX_DEV_MEASUREMENT "device"   // Used for logging AQI device data (e.g. battery)
#endif

#ifdef DWEET
  // Post data to the internet via dweet.io.  Set DWEET_DEVICE to be a
  // unique name you want associated with this reporting device, allowing
  // data to be easily retrieved through the web or Dweet's REST API.
  #define DWEET_HOST "dweet.io"   // Typically dweet.io
  #define DWEET_DEVICE "makerhour-rco2"  // Must be unique across all of dweet.io
#endif

// Display
// Pin config for host board to 1.54" 200x200 EPD
#define EPD_CS      12
#define EPD_DC      27
//#define SRAM_CS     14
#define EPD_RESET   15
#define EPD_BUSY    32

// enable GxEPD2_GFX base class to pass references or pointers to the display instance as parameter, uses ~1.2k more code
#define ENABLE_GxEPD2_GFX 1
// orientation of screen relative to physical housing 
const uint8_t displayRotation = 0; // rotation 0 orients as "top" near flex cable

// CO2 
//sample timing
#ifdef DEBUG
	// number of times SCD40 is read, last read is the sample value
	#define READS_PER_SAMPLE	1
	// time between samples in seconds. Must be >=180 to protect 3 color EPD
	#define SAMPLE_INTERVAL		60
#else
	#define READS_PER_SAMPLE	3
	#define SAMPLE_INTERVAL 	180
#endif
// number of samples stored to generate sparkline
// FIX: nvStorageRead and nvStorageWrite currently don't work if >10
const uint8_t co2MaxStoredSamples = 10;
const String co2Labels[5]={"Good", "OK", "So-So", "Poor", "Bad"};

// Battery
// analog pin used to reading battery voltage
#define VBATPIN A13 // ESP32V2
// number of analog pin reads sampled to average battery voltage
const uint8_t   batteryReadsPerSample = 5;
// battery charge level lookup table
const float batteryVoltageTable[101] = {
  3.200,  3.250,  3.300,  3.350,  3.400,  3.450,
  3.500,  3.550,  3.600,  3.650,  3.700,  3.703,
  3.706,  3.710,  3.713,  3.716,  3.719,  3.723,
  3.726,  3.729,  3.732,  3.735,  3.739,  3.742,
  3.745,  3.748,  3.752,  3.755,  3.758,  3.761,
  3.765,  3.768,  3.771,  3.774,  3.777,  3.781,
  3.784,  3.787,  3.790,  3.794,  3.797,  3.800,
  3.805,  3.811,  3.816,  3.821,  3.826,  3.832,
  3.837,  3.842,  3.847,  3.853,  3.858,  3.863,
  3.868,  3.874,  3.879,  3.884,  3.889,  3.895,
  3.900,  3.906,  3.911,  3.917,  3.922,  3.928,
  3.933,  3.939,  3.944,  3.950,  3.956,  3.961,
  3.967,  3.972,  3.978,  3.983,  3.989,  3.994,
  4.000,  4.008,  4.015,  4.023,  4.031,  4.038,
  4.046,  4.054,  4.062,  4.069,  4.077,  4.085,
  4.092,  4.100,  4.111,  4.122,  4.133,  4.144,
  4.156,  4.167,  4.178,  4.189,  4.200};

// Hardware
// Sleep time in seconds if hardware error occurs
#define HARDWARE_ERROR_INTERVAL 10

#endif // #ifdef CONFIG_H