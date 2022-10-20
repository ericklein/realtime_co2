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
#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSans12pt7b.h>
#include <Fonts/FreeSans18pt7b.h>
//#include <Fonts/FreeSans24pt7b.h>

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
// Handle two ESP32 I2C ports
#if defined(ARDUINO_ADAFRUIT_QTPY_ESP32S2) || defined(ARDUINO_ADAFRUIT_QTPY_ESP32S3_NOPSRAM) || defined(ARDUINO_ADAFRUIT_QTPY_ESP32S3) || defined(ARDUINO_ADAFRUIT_QTPY_ESP32_PICO)
  // ESP32 is kinda odd in that secondary ports must be manually
  // assigned their pins with setPins()!
  Wire1.setPins(SDA1, SCL1);
#endif

// Adafruit ESP32 I2C power management
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

#if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2_TFT)
  pinMode(TFT_I2C_POWER, OUTPUT);
  digitalWrite(TFT_I2C_POWER, HIGH);
#endif

#if defined(ADAFRUIT_FEATHER_ESP32_V2)
  // Turn on the I2C power by pulling pin HIGH.
  pinMode(NEOPIXEL_I2C_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_I2C_POWER, HIGH);
#endif

display.begin(THINKINK_MONO);

#ifdef DEBUG
  Serial.begin(115200);
  // wait for serial port connection
  while (!Serial)
    ;

  // Confirm key site configuration parameters
  debugMessage("realtime co2 monitor started");
  debugMessage("---------------------------------");
  debugMessage(String(SAMPLE_INTERVAL) + " second sample interval");
  debugMessage("Client ID: " + String(CLIENT_ID));
#endif

  // Initialize environmental sensor
  if (!initSensor()) {
    debugMessage("Environment sensor failed to initialize, going to sleep");
    screenAlert("SCD40 not detected");
    deepSleep();
  }

  // Environmental sensor available, so fetch values
  if (!readSensor()) {
    debugMessage("Environment sensor failed to read, going to sleep");
    screenAlert("SCD40 no data");
    deepSleep();
  }

  initBattery();
  readBattery();

  // Setup network connection specified in config.h
  internetAvailable = aq_network.networkBegin();

  String upd_flags = "";  // Indicates whether/which external data services were updated
  if (internetAvailable) 
  {
    hardwareData.rssi = abs(aq_network.getWiFiRSSI());

    // Update external data services
    #ifdef MQTTLOG
        if ((mqttSensorUpdate(sensorData.internalCO2, sensorData.internalTempF, sensorData.internalHumidity)) && (mqttDeviceWiFiUpdate(hardwareData.rssi)) && (mqttDeviceBatteryUpdate(hardwareData.batteryPercent, hardwareData.batteryVoltage))) {
          upd_flags += "M";
        }
    #endif

    #ifdef INFLUX
        // Returns true if successful
        if (post_influx(sensorData.internalCO2, sensorData.internalTempF, sensorData.internalHumidity, hardwareData.batteryPercent, hardwareData.batteryVoltage, hardwareData.rssi)) {
          upd_flags += "I";
        }
    #endif

    if (upd_flags == "") 
    {
      // External data services not updated but we have network time
      screenInfo(aq_network.dateTimeString());
    } 
    else 
    {
      // External data services not updated and we have network time
      screenInfo("[+" + upd_flags + "] " + aq_network.dateTimeString());
    }
  }
  else
  {
    // no internet connection, update screen with sensor data only
    screenInfo("");
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

  uint16_t error;
  char errorMessage[256];

  // stop potentially previously started measurement
  error = envSensor.stopPeriodicMeasurement();
  if (error) {
    Serial.print("Error trying to execute stopPeriodicMeasurement(): ");
    errorToString(error, errorMessage, 256);
    debugMessage(errorMessage);
  }
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
  display.setCursor(40, (display.height() / 2 + 6));
  display.print(messageText);

  //update display
  display.display();
}

void screenInfo(String messageText)
// Display environmental information
{
  debugMessage("Starting screen refresh");
  // screen size cheats
  int xLeftMargin = (display.width() / 20);

  display.clearBuffer();
  display.setTextColor(EPD_BLACK);

  // display battery status
  screenBatteryStatus();

  // display wifi status
  screenWiFiStatus();

  // Indoor CO2 level
  // calculate CO2 value range in 400ppm bands
  int co2range = ((sensorData.internalCO2 - 400) / 400);
  display.setFont(&FreeSans18pt7b);
  display.setCursor(xLeftMargin, (display.height() / 4));
  display.print(String(co2Labels[co2range]) + " CO2");
  display.setFont(&FreeSans9pt7b);
  display.setCursor((display.width()-40), (display.height() / 3));
  display.print(sensorData.internalCO2);

  // Indoor temp
  display.setFont(&FreeSans18pt7b);
  display.setCursor(xLeftMargin, (display.height() / 2));
  display.print(String((int)(sensorData.internalTempF + 0.5)));
  // display Fahrenheit symbol
  // move the cursor to raise the F indicator
  //display.setCursor(x,y);
  display.setFont(&meteocons16pt7b);
  display.print("+");

  // Indoor humidity
  display.setFont(&FreeSans12pt7b);
  display.setCursor(xLeftMargin, (display.height() * 3 / 4));
  display.print(String((int)(sensorData.internalHumidity + 0.5)) + "% humidity");

  // status message
  display.setFont();  // resets to system default monospace font
  display.setCursor(5, (display.height() - 9));
  display.print(messageText);

  display.display();
  debugMessage("completed screen update");
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

void readBattery() {
  if (batteryAvailable) {
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
    const int barHeight = 10;
    const int barWidth = 28;

    // battery nub (3pix wide, 6pix high)
    display.drawRect((display.width() - 8), ((display.height() * 1 / 8) + 7), 3, 6, EPD_BLACK);
    //battery percentage as rectangle fill
    display.fillRect((display.width() - barWidth - 8), ((display.height() * 1 / 8) + 5), (int((hardwareData.batteryPercent / 100) * barWidth)), barHeight, EPD_GRAY);
    // battery border
    display.drawRect((display.width() - barWidth - 8), ((display.height() * 1 / 8) + 5), barWidth, barHeight, EPD_BLACK);
    debugMessage("battery status drawn to screen");
  }
}

void screenWiFiStatus() 
{
  if (internetAvailable) 
  {
    const int barWidth = 3;
    int barCount;

    // Convert RSSI values to a 5 bar visual indicator
    // Allowed RSSI values are from 40-90. <40 almost never exist and >90 means no signal
    if (((6-((hardwareData.rssi/10)-3))<6)&((6-((hardwareData.rssi/10)-3))>0))
    {
      // 40-50 rssi value = 5 bars, each +10 rssi value range = one less bar
      barCount = (6-((hardwareData.rssi/10)-3));
      // draw bars to represent WiFi strength
      for (int b = 1; b <= barCount; b++)
      {
        display.fillRect(((display.width() - 35) + (b * 5)), ((display.height()) - (b * 5)), barWidth, b * 5, EPD_BLACK);
      }
      debugMessage(String("WiFi signal strength on screen as ") + barCount +" bars");
    }
    // if (hardwareData.rssi > -50)
    // {
    //   barCount = 5;
    // } 
    // else if (hardwareData.rssi< -50 & hardwareData.rssi > - 60)
    // {
    //   barCount = 4;
    // } 
    // else if (hardwareData.rssi< -60 & hardwareData.rssi > - 70)
    // {
    //   barCount = 3;
    // } 
    // else if (hardwareData.rssi< -70 & hardwareData.rssi > - 80)
    // {
    //   barCount = 2;
    // } 
    // else if (hardwareData.rssi< -80 & hardwareData.rssi > - 90)
    // {
    //   barCount = 1;
    // }
  }
}

int initSensor() {
  uint16_t error;
  char errorMessage[256];

// Handle two ESP32 I2C ports
#if defined(ARDUINO_ADAFRUIT_QTPY_ESP32S2) || defined(ARDUINO_ADAFRUIT_QTPY_ESP32S3_NOPSRAM) || defined(ARDUINO_ADAFRUIT_QTPY_ESP32S3) || defined(ARDUINO_ADAFRUIT_QTPY_ESP32_PICO)
  Wire1.begin();
  envSensor.begin(Wire1);
#else
  Wire.begin();
  envSensor.begin(Wire);
#endif

  envSensor.wakeUp();
  envSensor.setSensorAltitude(SITE_ALTITUDE);  // optimizes CO2 reading

  error = envSensor.startPeriodicMeasurement();
  if (error) {
    // Failed to initialize SCD40
    errorToString(error, errorMessage, 256);
    debugMessage(String(errorMessage) + " executing SCD40 startPeriodicMeasurement()");
    return 0;
  } else {
    debugMessage("SCD40 initialized, waiting 5 sec for first measurement");
    delay(5000);  // Give SCD40 time to warm up
    return 1;     // success
  }
}

int readSensor()
// reads environment sensor and stores data to environment global
{
  uint16_t error;
  char errorMessage[256];

  error = envSensor.readMeasurement(sensorData.internalCO2, sensorData.internalTempF, sensorData.internalHumidity);
  if (error) {
    errorToString(error, errorMessage, 256);
    debugMessage(String(errorMessage) + "executing SCD40 readMeasurement()");
    return 0;
  }
  //convert C to F for temp
  sensorData.internalTempF = (sensorData.internalTempF * 1.8) + 32;

  debugMessage(String("SCD40 values: ") + sensorData.internalTempF + "F, " + sensorData.internalHumidity + "%, " + sensorData.internalCO2 + " ppm");
  return 1;
}