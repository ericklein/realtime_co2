/*
  Project Name:   realtime_co2
  Description:    Regularly sample and log temperature, humidity, and co2 levels

  See README.md for target information and revision history
*/

// hardware and internet configuration parameters
#include "config.h"
// private credentials for network, MQTT, weather provider
#include "secrets.h"

// Generalized network handling
#include "aq_network.h"
AQ_Network aq_network;

// environment sensor data
typedef struct
{
  float internalTempF;
  float internalHumidity;
  uint16_t internalCO2;
} envData;

// global for air characteristics
envData sensorData;

// hardware status data
typedef struct
{
  float batteryPercent;
  float batteryVoltage;
  int rssi;
} hdweData;

hdweData hardwareData;
bool batteryAvailable = false;
bool internetAvailable = false;

// initialize scd40 environment sensor
#include <SensirionI2CScd4x.h>
SensirionI2CScd4x envSensor;

// Battery voltage sensor
#include <Adafruit_LC709203F.h>
Adafruit_LC709203F lc;

// screen support
#include <Adafruit_ThinkInk.h>

#include "Fonts/meteocons16pt7b.h"
//#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSans12pt7b.h>
//#include <Fonts/FreeSans18pt7b.h>
#include <Fonts/FreeSans24pt7b.h>

#define EPD_CS      12
#define EPD_DC      13
#define SRAM_CS     14
#define EPD_RESET   15 // can set to -1 and share with microcontroller Reset!
#define EPD_BUSY    32 // can set to -1 to not use a pin (will wait a fixed delay)

// 1.54" Monochrome displays with 200x200 pixels and SSD1681 chipset
ThinkInk_154_Mono_D67 display(EPD_DC, EPD_RESET, EPD_CS, SRAM_CS, EPD_BUSY);

#ifdef INFLUX
  extern boolean post_influx(uint16_t co2, float tempF, float humidity, float battery_p, float battery_v, int rssi);
#endif

#ifdef MQTTLOG
  extern void mqttConnect();
  extern int mqttDeviceWiFiUpdate(int rssi);
  extern int mqttDeviceBatteryUpdate(float cellPercent, float cellVoltage);
  extern int mqttSensorUpdate(uint16_t co2, float tempF, float humidity);
#endif

void setup()
// One time run of code, then deep sleep
{
  #if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2)
    // turn on the I2C power by setting pin to opposite of 'rest state'
    // Rev B board is LOW to enable
    // Rev C board is HIGH to enable
    pinMode(PIN_I2C_POWER, INPUT);
    delay(1);
    bool polarity = digitalRead(PIN_I2C_POWER);
    pinMode(PIN_I2C_POWER, OUTPUT);
    digitalWrite(PIN_I2C_POWER, !polarity);
  #endif


  display.begin(THINKINK_MONO); // changed from THINKINK_GRAYSCALE4 to eliminate black screen border
  debugMessage("Display ready");

  #ifdef DEBUG
    Serial.begin(115200);
    // wait for serial port connection
    while (!Serial);

    // Confirm key site configuration parameters
    debugMessage("realtime co2 monitor started");
    debugMessage("---------------------------------");
    debugMessage(String(SAMPLE_INTERVAL) + " second sample interval");
    debugMessage("Client ID: " + String(CLIENT_ID));
  #endif

  // Initialize environmental sensor
  if (!initSensor()) 
  {
    debugMessage("Environment sensor failed to initialize, going to sleep");
    screenAlert("SCD40 not detected");
    deepSleep();
  }

  // Environmental sensor available, so fetch values
  if(!readSensor())
  {
    debugMessage("Environment sensor failed to read, going to sleep");
    screenAlert("SCD40 no data");
    deepSleep();
  }

  initBattery();
  readBattery();

  // Setup network connection specified in config.h
  internetAvailable = aq_network.networkBegin();

  String upd_flags = "";  // To indicate whether services succeeded
  if (internetAvailable)
  {
    hardwareData.rssi = aq_network.getWiFiRSSI();

    #ifdef MQTTLOG
      // sensor_pub = mqttSensorUpdate(sensorData.internalCO2, sensorData.internalTempF,sensorData.internalHumidity);
      // device_pub = mqttDeviceWiFiUpdate(hardwareData.rssi);
      // device_pub = mqttDeviceBatteryUpdate(hardwareData.batteryPercent, hardwareData.batteryVoltage);
      if ((mqttSensorUpdate(sensorData.internalCO2, sensorData.internalTempF,sensorData.internalHumidity)) && (mqttDeviceWiFiUpdate(hardwareData.rssi)) && (mqttDeviceBatteryUpdate(hardwareData.batteryPercent, hardwareData.batteryVoltage)))
      {
        upd_flags += "M";
      }
    #endif

    #ifdef INFLUX
      // Returns true if successful
      if (post_influx(sensorData.internalCO2, sensorData.internalTempF, sensorData.internalHumidity, hardwareData.batteryPercent, hardwareData.batteryVoltage, hardwareData.rssi)) {
        upd_flags += "I";
      }
    #endif
  }

  // Update the screen if available
  if (upd_flags == "") {
    // None of the services succeeded
    screenInfo("");
  } else {
    screenInfo("[+" + upd_flags + "] " + aq_network.dateTimeString());
  }
  deepSleep();
}

void loop() {}

void debugMessage(String messageText)
// wraps Serial.println as #define conditional
{
#ifdef DEBUG
  Serial.println(messageText);
  Serial.flush();  // Make sure the message gets output (before any sleeping...)
#endif
}

void deepSleep()
// Powers down hardware in preparation for board deep sleep
{
  debugMessage(String("Going to sleep for ") + SAMPLE_INTERVAL + " second(s)");
  display.powerDown();
  digitalWrite(EPD_RESET, LOW);  // hardware power down mode
  aq_network.networkStop();

  envSensor.stopPeriodicMeasurement();
  envSensor.powerDown();

  #if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2)
    // Rev B board is LOW to enable
    // Rev C board is HIGH to enable
    digitalWrite(PIN_I2C_POWER, HIGH);
  #endif

  esp_sleep_enable_timer_wakeup(SAMPLE_INTERVAL * SAMPLE_INTERVAL_ESP_MODIFIER);
  esp_deep_sleep_start();
}

void screenAlert(String messageText)
// Display critical error message on screen
{
  display.clearBuffer();
  display.setTextColor(EPD_BLACK);
  display.setFont(&FreeSans12pt7b);
  display.setCursor(40,(display.height()/2+6));
  display.print(messageText);

  //update display
  display.display();
}

void screenInfo(String messageText)
// Display environmental information on screen
{
  // screen size cheats
  int xLeftMargin = (display.width()/20);
  // int x_mid_point = (display.width()/2);

  display.clearBuffer();
  display.setTextColor(EPD_BLACK);

  // battery status
  screenBatteryStatus();

  // wifi status
  screenWiFiStatus();

  // Indoor temp
  display.setFont(&FreeSans24pt7b);
  display.setCursor(xLeftMargin,(display.height()/4));
  display.print(String((int)(sensorData.internalTempF+0.5)));
  // display Fahrenheit symbol
  // move the cursor to raise the F indicator
  //display.setCursor(x,y);
  display.setFont(&meteocons16pt7b);
  display.print("+");

  // Indoor humidity
  display.setFont(&FreeSans12pt7b);
  display.setCursor(xLeftMargin,(display.height()*1/2));
  display.print(String((int)(sensorData.internalHumidity+0.5)) + "% humidity");

  // Indoor CO2 level
  String co2Labels[3]={"Good","Poor","Bad"};
  int co2range;

  // default value is 0 for values below 1000
  co2range = 0;
  if (sensorData.internalCO2>1000) co2range = 1;
  if (sensorData.internalCO2>2000) co2range = 2;
  // display.setFont(&FreeSans9pt7b); 
  display.setCursor(xLeftMargin,(display.height()*3/4));
  display.print(String(co2Labels[co2range])+ " CO2, "+ sensorData.internalCO2);

  // status message
  display.setFont();  // resets to system default monospace font
  display.setCursor(5,(display.height()-9));
  display.print(messageText);

  display.display();
  debugMessage("Screen updated");
}

void initBattery() {
  if (lc.begin())
  // Check battery monitoring status
  {
    debugMessage("Battery monitor ready");
    lc.setPackAPA(BATTERY_APA);
    batteryAvailable = true;
  } else {
    debugMessage("Battery monitor not detected");
  }
}

void readBattery()
{
  if (batteryAvailable)
  {
    hardwareData.batteryPercent = lc.cellPercent();
    hardwareData.batteryVoltage = lc.cellVoltage();
    debugMessage("Battery is at " + String(hardwareData.batteryPercent) + " percent capacity");
    debugMessage("Battery voltage: " + String(hardwareData.batteryVoltage) + " v");
  }
}

void screenBatteryStatus()
// Displays remaining battery % as graphic in lower right of screen
// used in XXXScreen() routines
{
  if (batteryAvailable) 
  {
    int barHeight = 10;
    int barWidth = 28;

    // battery nub (3pix wide, 6pix high)
    display.drawRect((display.width()-8),((display.height()*1/8)+7),3,6,EPD_BLACK);
    //battery percentage as rectangle fill
    display.fillRect((display.width()-barWidth-8),((display.height()*1/8)+5),(int((hardwareData.batteryPercent/100)*barWidth)),barHeight,EPD_GRAY);
    // battery border
    display.drawRect((display.width()-barWidth-8),((display.height()*1/8)+5),barWidth,barHeight,EPD_BLACK);
  }
}

void screenWiFiStatus()
{
  if (internetAvailable)
  {
    display.setCursor((display.width()-20),(display.height()-9));
    display.setFont();
    display.print("WiFi");
  }
}

int initSensor() 
{
  uint16_t error;
  char errorMessage[256];

  Wire.begin();
  envSensor.begin(Wire);
  envSensor.wakeUp();
  envSensor.setSensorAltitude(SITE_ALTITUDE); // optimizes CO2 reading

  error = envSensor.startPeriodicMeasurement();
  if (error) 
  {
    // Failed to initialize SCD40
    errorToString(error, errorMessage, 256);
    debugMessage(String(errorMessage) + " executing SCD40 startPeriodicMeasurement()");
    return 0;
  }
  else 
  {
    delay(5000);  // Give SCD40 time to warm up
    return 1; // success
  }
}

int readSensor()
// reads environment sensor and stores data to environment global
{
  uint16_t error;
  char errorMessage[256];

  error = envSensor.readMeasurement(sensorData.internalCO2, sensorData.internalTempF, sensorData.internalHumidity);
  if (error) 
  {
    errorToString(error, errorMessage, 256);
    debugMessage(String(errorMessage) + "executing SCD40 readMeasurement()");
    return 0;
  }
  //convert C to F for temp
  sensorData.internalTempF = (sensorData.internalTempF * 1.8) + 32;

  debugMessage(String("SCD40 values: ") + sensorData.internalTempF + "F, " + sensorData.internalHumidity + "%, " + sensorData.internalCO2 + " ppm");
  return 1;
}