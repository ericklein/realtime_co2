/*
  Project Name:   realtime_co2
  Description:    Regularly sample and log temperature, humidity, and co2 levels

  See README.md for target information and revision history
*/
#ifndef CONFIG_H
#define CONFIG_H

// Step 1: Set conditional compile flags
#define DEBUG 	// Output to serial port
#define WIFI   	// use WiFi
#define MQTT 		// log sensor data to M/QTT broker
//#define HASSIO_MQTT  // And, if MQTT enabled, with Home Assistant too?
#define INFLUX	// Log data to InfluxDB server

// Step 2: Set key device and installation configuration parameters.  These are used
// widely throughout the code to properly identify the device and generate important
// operating elements like MQTT topics, InfluxDB data tags (metadata).  Should be
// customized to match the target installation. Values here are examples.
#define DEVICE           "realtime_co2"
#define DEVICE_SITE      "beachhouse"
#define DEVICE_LOCATION  "outdoor"
#define DEVICE_ROOM      "boatdock"
#define DEVICE_ID        "Unique_device_ID"

// Step 3: Set battery size if applicable
// based on a settings curve in the LC709203F datasheet
// #define BATTERY_APA 0x08 // 100mAH
// #define BATTERY_APA 0x0B // 200mAH
#define BATTERY_APA 0x10 // 500mAH
// #define BATTERY_APA 0x19 // 1000mAH
// #define BATTERY_APA 0x1D // 1200mAH
// #define BATTERY_APA 0x2D // 2000mAH
// #define BATTERY_APA 0x32 // 2500mAH
// #define BATTERY_APA 0x36 // 3000mAH

const float batteryMaxVoltage	= 4.2; 	// maximum battery voltage
const float batteryMinVoltage	= 3.2; 	// what we regard as an empty battery

// Pin config for host board
#if defined (ARDUINO_ADAFRUIT_FEATHER_ESP32_V2)
	// Adafruit 1.5" mono EPD (part#4196)
	#define EPD_CS      12
	#define EPD_DC      13
	#define SRAM_CS     14 // can set to -1 to not use a pin (uses a lot of RAM!)
	#define EPD_RESET   15 // can set to -1 and share with chip Reset (can't deep sleep)
	#define EPD_BUSY    32 // can set to -1 to not use a pin (will wait a fixed delay)

	// battery pin
	#define VBATPIN A13
#endif

// Sleep time in seconds if hardware error occurs
#define HARDWARE_ERROR_INTERVAL 10

#define CONNECT_ATTEMPT_LIMIT	3 // max connection attempts to internet services
#define CONNECT_ATTEMPT_INTERVAL 10 // seconds between internet service connect attempts

// Allow for adjustable screen as needed for physical packaging. 
// Rotation 1 orients the display so the wiring is at the top.
// A rotation of 3 flips it so the wiring is at the bottom
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

const String co2Labels[5]={"Good", "OK", "So-So", "Poor", "Bad"};
// used in aq_network.cpp
const String weekDays[7] = { "Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday" };

// NTP time configuration
//https://cplusplus.com/reference/ctime/tm/

#define ntpServer "pool.ntp.org"
// const long  gmtOffset_sec = 0; // UTC
// const long  gmtOffset_sec = 3600; // Ireland
const long  gmtOffset_sec = -28800; // PST
const int   daylightOffset_sec = 0;
// const int   daylightOffset_sec = 3600; // US DT

#ifdef MQTT
	// Adafruit I/O
	// structure: username/feeds/groupname.feedname or username/feeds/feedname
	// e.g. #define MQTT_PUB_TOPIC1		"sircoolio/feeds/pocket-office.temperature"

	// structure: site/room/device/data	
	// #define MQTT_PUB_TEMPF			"7828/demo/rco2/temperature"
	// #define MQTT_PUB_HUMIDITY		"7828/demo/rco2/humidity"
	// #define MQTT_PUB_CO2				"7828/demo/rco2/co2"
	// #define MQTT_PUB_BATTVOLT		"7828/demo/rco2/battery-voltage"
	// #define MQTT_PUB_RSSI				"7828/demo/rco2/rssi"

  // Additional (optional) topics if integrating with Home Assistant
  #ifdef HASSIO_MQTT
    // Home Assistant entity configuration & state (values) topics. NOTE: MUST MATCH value
    // used in Home Assistant MQTT configuration file (configuration.yaml). See 
    // hassio_mqtt.cpp for details.
    // #define MQTT_HASSIO_STATE   "homeassistant/sensor/rco2-1/state"
  #endif
#endif

#ifdef INFLUX
  #define INFLUX_ENV_MEASUREMENT "weather"  // Used for environmental sensor data
  #define INFLUX_DEV_MEASUREMENT "device"   // Used for logging AQI device data (e.g. battery)
#endif

#endif // #ifdef CONFIG_H