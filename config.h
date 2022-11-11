/*
  Project Name:   realtime_co2
  Description:    Regularly sample and log temperature, humidity, and co2 levels

  See README.md for target information and revision history
*/

// Step 1: Set conditional compile flags
//#define DEBUG 	// Output to serial port
#define WIFI   	// use WiFi
#define MQTT 		// log sensor data to MQTT broker
#define INFLUX	// Log data to remote InfluxDB server

// Step 2: Set battery size if applicable
// based on a settings curve in the LC709203F datasheet
// #define BATTERY_APA 0x08 // 100mAH
// #define BATTERY_APA 0x0B // 200mAH
// #define BATTERY_APA 0x10 // 500mAH
// #define BATTERY_APA 0x19 // 1000mAH
// #define BATTERY_APA 0x1D // 1200mAH
#define BATTERY_APA 0x2D // 2000mAH
// #define BATTERY_APA 0x32 // 2500mAH
// #define BATTERY_APA 0x36 // 3000mAH

// Pin config for e-paper display

#if defined (ARDUINO_ADAFRUIT_FEATHER_ESP32_V2)
	// Adafruit Feather ESP32 V2
	#define EPD_CS      12
	#define EPD_DC      13
	#define SRAM_CS     14 // can set to -1 to not use a pin (uses a lot of RAM!)
	#define EPD_RESET   15 // can set to -1 and share with chip Reset (can't deep sleep)
	#define EPD_BUSY    32 // can set to -1 to not use a pin (will wait a fixed delay)

	// battery pin
	#define VBATPIN A13
#endif

#if defined (ARDUINO_ADAFRUIT_QTPY_ESP32S2)
	#define EPD_CS      8		// A3
	#define EPD_DC      9		// A2
	#define SRAM_CS     17	// A1, can set to -1 to not use a pin (uses a lot of RAM!)
	#define EPD_RESET   -1	// can set to -1 and share with chip Reset (can't deep sleep)
	#define EPD_BUSY    -1	// can set to -1 to not use a pin (will wait a fixed delay)
#endif

// Interval between SCD40 samples in seconds
#ifdef DEBUG
	#define SAMPLE_INTERVAL 60
#else
	#define SAMPLE_INTERVAL 180
#endif

// Sleep time if hardware error occurs in seconds
#define HARDWARE_ERROR_INTERVAL 10

#define WIFI_ATTEMPT_LIMIT	5 // max connection attempts to WiFi AP

const String co2Labels[5]={"Good", "OK", "So-So", "Poor", "Bad"};
// used in aq_network.cpp
const String weekDays[7] = { "Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday" };

// millisecond modifier to seconds for sampling interval (ARM)
// #define SAMPLE_INTERVAL_ARM_MODIFIER 1000
// microsecond modifier to seonds for sampling interval (ESP)
#define SAMPLE_INTERVAL_ESP_MODIFIER 1000000

// select time zone, used by NTPClient
// const int timeZone = 0;  	// UTC
//const int timeZone = 1; // Ireland
//const int timeZone = -5;  // USA EST
//const int timeZone = -4;  // USA EDT
const int timeZone = -7;  // USA PDT
//const int timeZone = -8;  // USA PST

// set client ID; used by mqtt and wifi
#define CLIENT_ID "RCO2"

#ifdef MQTT
	// set MQTT parameters
	#define MQTT_ATTEMPT_LIMIT 	3 	// max connection attempts to MQTT broker

	// Adafruit I/O
	// structure: username/feeds/groupname.feedname or username/feeds/feedname
	// e.g. #define MQTT_PUB_TOPIC1		"sircoolio/feeds/pocket-office.temperature"

	// structure: site/room/device/data	
	#define MQTT_PUB_TOPIC1		"7828/demo/rco2/temperature"
	#define MQTT_PUB_TOPIC2		"7828/demo/rco2/humidity"
	#define MQTT_PUB_TOPIC3		"7828/demo/rco2/co2"
	#define MQTT_PUB_TOPIC5		"7828/demo/rco2/battery-voltage"
	#define MQTT_PUB_TOPIC6		"7828/demo/rco2/rssi"
#endif

#ifdef INFLUX  
  // Name of Measurements expected/used in the Influx DB.
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

	#define INFLUX_ATTEMPT_LIMIT 	3 	// max connection attempts to Influxdb
#endif

// The following parameters are defined in secrets.h.
// 	WiFi credentials (if WiFi enabled)
// 	#define WIFI_SSID
// 	#define WIFI_PASS

// If MQTT enabled
// 	#define MQTT_PORT
// 	#define MQTT_USER
// 	#define MQTT_BROKER
// 	#define MQTT_PASS

// If InfluxDB data storage enabled
// For an InfluxDB v1.X server:
// #define INFLUX_V1
// #define INFLUXDB_URL 
// #define INFLUXDB_DB_NAME
// #define INFLUXDB_USER
// #define INFLUXDB_PASSWORD
//
// For an InfluxDB v2.X server:
// #define INFLUX_V2
// #define INFLUXDB_URL 
// #define INFLUXDB_TOKEN
// #define INFLUXDB_ORG
// #define INFLUXDB_BUCKET