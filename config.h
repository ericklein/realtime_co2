/*
  Project Name:   realtime_co2
  Description:    Regularly sample and log temperature, humidity, and co2 levels

  See README.md for target information and revision history
*/

// Configuration Step 1: Set debug message output
// 0 = none, 1 = summary, 2 = verbose
#define DEBUG 1

// Configuration Step 2: Set network data endpoints
// #define MQTT 		// log sensor data to M/QTT broker
// #define HASSIO_MQTT  // And, if MQTT enabled, with Home Assistant too?
// #define INFLUX	// Log data to InfluxDB server

// Configuration Step 3: Set battery parameters, if applicable

// If LC709203F detected on i2c, define battery pack based on settings curve from datasheet
// #define BATTERY_APA 0x08 // 100mAH
// #define BATTERY_APA 0x0B // 200mAH
#define BATTERY_APA 0x10 // 500mAH
// #define BATTERY_APA 0x19 // 1000mAH
// #define BATTERY_APA 0x1D // 1200mAH
// #define BATTERY_APA 0x2D // 2000mAH
// #define BATTERY_APA 0x32 // 2500mAH
// #define BATTERY_APA 0x36 // 3000mAH

// used for reading battery voltage from analog PIN on applicable devices
const float batteryMaxVoltage	= 4.2; 	// maximum battery voltage
const float batteryMinVoltage	= 3.2; 	// what we regard as an empty battery
// battery pin for Adafruit ESP32V2 (part#5400)
#define VBATPIN A13

// Configuration Step 4: Set parameters for NTP time configuration

// this will only be used if network data endpoints are defined
#define ntpServer "pool.ntp.org"
// const long  gmtOffset_sec = 0; // UTC
// const long  gmtOffset_sec = 3600; // Ireland
const long  gmtOffset_sec = -28800; // PST
// const int   daylightOffset_sec = 0;
const int   daylightOffset_sec = 3600; // US DT

// Configuration Step 5: Set network data endpoint parameters, if applicable

// set client ID; used by mqtt and wifi
#define CLIENT_ID "RCO2"

#ifdef MQTT
	// structure: site/room/device/data	
	#define MQTT_PUB_TEMPF			"7828/demo/rco2/temperature"
	#define MQTT_PUB_HUMIDITY		"7828/demo/rco2/humidity"
	#define MQTT_PUB_CO2				"7828/demo/rco2/co2"
	#define MQTT_PUB_BATTVOLT		"7828/demo/rco2/battery-voltage"
	#define MQTT_PUB_RSSI				"7828/demo/rco2/rssi"

  // Additional (optional) topics if integrating with Home Assistant
  #ifdef HASSIO_MQTT
    // Home Assistant entity configuration & state (values) topics. NOTE: MUST MATCH value
    // used in Home Assistant MQTT configuration file (configuration.yaml). See 
    // hassio_mqtt.cpp for details.
    #define MQTT_HASSIO_STATE   "homeassistant/sensor/rco2-1/state"
  #endif
#endif

#ifdef INFLUX
  #define INFLUX_ENV_MEASUREMENT "weather"  // Used for environmental sensor data
  #define INFLUX_DEV_MEASUREMENT "device"   // Used for logging AQI device data (e.g. battery)
  
	// Standard set of tag values used for each sensor data point stored to InfluxDB.  Reuses
  // CLIENT_ID as defined anove here in config.h as well as device location (e.g., room in 
  // the house) and site (indoors vs. outdoors, typically).

	// #define DEVICE_LOCATION "test"
	#define DEVICE_LOCATION "RCO2-demo"
	//#define DEVICE_LOCATION "kitchen"
	// #define DEVICE_LOCATION "cellar"
	// #define DEVICE_LOCATION "lab-office"
	// #define DEVICE_LOCATION "master bedroom"
  // #define DEVICE_LOCATION "pocket-office"

	#define DEVICE_SITE "indoor"
	#define DEVICE_TYPE "air quality"
#endif

// Configuration variables that are less likely to require changes

#define CONNECT_ATTEMPT_LIMIT	3 // max connection attempts to internet services
#define CONNECT_ATTEMPT_INTERVAL 10 // seconds between internet service connect attempts

// Pin config for host board
// Adafruit 1.5" mono EPD (part#4196)
#define EPD_CS      12
#define EPD_DC      13
#define SRAM_CS     14 // can set to -1 to not use a pin (uses a lot of RAM!)
#define EPD_RESET   15 // can set to -1 and share with chip Reset (can't deep sleep)
#define EPD_BUSY    32 // can set to -1 to not use a pin (will wait a fixed delay)

// rotation 1 orients the display so the wiring is at the top
// rotation of 3 flips it so the wiring is at the bottom
#define DISPLAY_ROTATION 3

// SCD40 sample timing
#ifdef DEBUG
	// number of times SCD40 is read, last read is the sample value
	#define READS_PER_SAMPLE	1
	// time between samples in seconds
	#define SAMPLE_INTERVAL		60
#else
	#define READS_PER_SAMPLE	5
	#define SAMPLE_INTERVAL 	180
#endif

// nvStorageRead and nvStorageWrite currently don't work if >10
const int co2MaxStoredSamples = 10;

// Sleep time in seconds if hardware error occurs
#define HARDWARE_ERROR_INTERVAL 10

const String co2Labels[5]={"Good", "OK", "So-So", "Poor", "Bad"};
// used in aq_network.cpp
const String weekDays[7] = { "Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday" };