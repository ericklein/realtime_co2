/*
  Project Name:   realtime_co2
  Description:    Regularly sample and log temperature, humidity, and co2 levels

  See README.md for target information and revision history
*/

// hardware and internet configuration parameters
#include "config.h"
// private credentials for network, MQTT
#include "secrets.h"

// Special glyphs for the UI
#include "glyphs.h"

// Generalized network handling
#include "aq_network.h"
AQ_Network aq_network;

// environment sensor data
typedef struct
{
  float ambientTempF;
  float ambientHumidity;
  uint16_t ambientCO2;
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

bool batteryVoltageAvailable = false;
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
  extern boolean post_influx(uint16_t co2, float tempF, float humidity, float battery_v, int rssi);
#endif

#ifdef MQTT
  extern void mqttConnect();
  extern int mqttDeviceWiFiUpdate(int rssi);
  extern int mqttDeviceBatteryUpdate(float cellVoltage);
  extern int mqttSensorUpdate(uint16_t co2, float tempF, float humidity);
#endif

void setup()
// One time run of code, then deep sleep
{
  // handle Serial first so debugMessage() works
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

  enableInternalPower();

  display.begin(THINKINK_MONO);
  display.setRotation(DISPLAY_ROTATION);

  // Initialize environmental sensor
  if (!initSensor()) {
    debugMessage("Environment sensor failed to initialize, going to sleep");
    screenAlert("NO SCD40");
    // This error often occurs right after a firmware flash and reset.
    // Hardware deep sleep typically resolves it, so quickly cycle the hardware
    disableInternalPower(HARDWARE_ERROR_INTERVAL);
  }

  // Environmental sensor available, so fetch values
  if (!readSensor()) {
    debugMessage("SCD40 returned no/bad data, going to sleep");
    screenAlert("SCD40 no/bad data");
    disableInternalPower(HARDWARE_ERROR_INTERVAL);
  }

  batteryReadVoltage();

  // Setup network connection specified in config.h
  internetAvailable = aq_network.networkBegin();

  String upd_flags = "";  // Indicates whether/which external data services were updated
  if (internetAvailable) 
  {
    hardwareData.rssi = abs(aq_network.getWiFiRSSI());

    // Update external data services
    #ifdef MQTT
        if ((mqttSensorUpdate(sensorData.ambientCO2, sensorData.ambientTempF, sensorData.ambientHumidity)) && (mqttDeviceWiFiUpdate(hardwareData.rssi)) && (mqttDeviceBatteryUpdate(hardwareData.batteryVoltage))) {
          upd_flags += "M";
        }
    #endif

    #ifdef INFLUX
        // Returns true if successful
        if (post_influx(sensorData.ambientCO2, sensorData.ambientTempF, sensorData.ambientHumidity, hardwareData.batteryVoltage, hardwareData.rssi)) {
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
  disableInternalPower(SAMPLE_INTERVAL);
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

#ifdef ORIGINAL_LAYOUT
  // Indoor CO2 level
  // calculate CO2 value range in 400ppm bands
  int co2range = ((sensorData.ambientCO2 - 400) / 400);
  co2range = constrain(co2range,0,4); // filter CO2 levels above 2400
  display.setFont(&FreeSans18pt7b);
  display.setCursor(xLeftMargin, (display.height() / 4));
  display.print(String(co2Labels[co2range]) + " CO2");
  display.setFont(&FreeSans9pt7b);
  display.setCursor((display.width()-40), (display.height() / 3));
  display.print(sensorData.ambientCO2);

  // Indoor temp
  display.setFont(&FreeSans18pt7b);
  display.setCursor(xLeftMargin, (display.height() / 2));
  display.print(String((int)(sensorData.ambientTempF + 0.5)));
  // display Fahrenheit symbol
  // move the cursor to raise the F indicator
  //display.setCursor(x,y);
  display.setFont(&meteocons16pt7b);
  display.print("+");

  // Indoor humidity
  display.setFont(&FreeSans12pt7b);
  display.setCursor(xLeftMargin, (display.height() * 3 / 4));
#else
  // Indoor CO2 level
  // calculate CO2 value range in 400ppm bands
  int co2range = ((sensorData.ambientCO2 - 400) / 400);
  co2range = constrain(co2range,0,4); // filter CO2 levels above 2400
  display.setFont(&FreeSans18pt7b);
  display.setCursor(xLeftMargin, 50);
  display.print("CO");
  display.setCursor(xLeftMargin+65,50);
  display.print(": " + String(co2Labels[co2range]));
  display.setFont(&FreeSans12pt7b);
  display.setCursor(xLeftMargin+50,60);
  display.print("2");
  //display.setFont(&FreeSans9pt7b);
  display.setCursor((xLeftMargin+90),75);
  display.print("(" + String(sensorData.ambientCO2) + ")");

  // Indoor temp
  display.setFont(&FreeSans18pt7b);
  int tempF = sensorData.ambientTempF + 0.5;
  if(tempF < 100) {
    display.setCursor(xLeftMargin,130);
    display.print(String(tempF));
    /*
    display.setFont(&FreeSans12pt7b);
    display.setCursor(xLeftMargin+45,130);
    display.print("F");
    */
    display.drawBitmap(xLeftMargin+42,104,epd_bitmap_temperatureF_icon_sm,20,28,EPD_BLACK);
  }
  else {
    display.setCursor(xLeftMargin,130);
    display.print(String(tempF));
    display.setFont(&FreeSans12pt7b);
    display.setCursor(xLeftMargin+65,130);
    display.print("F"); 
  }
  // display Fahrenheit symbol
  // move the cursor to raise the F indicator
  //display.setCursor(x,y);
  // display.setFont(&meteocons16pt7b);
  // display.print("+");

  // Indoor humidity
  display.setFont(&FreeSans18pt7b);
  display.setCursor(display.width()/2, 130);
  display.print(String((int)(sensorData.ambientHumidity + 0.5)));
  /*
  display.setFont(&FreeSans12pt7b);
  display.setCursor(xLeftMargin+45,155);
  display.print("%RH");
  */
  // display.drawLine(xLeftMargin+45,131,xLeftMargin+70,131,EPD_BLACK);
  // display.drawLine(xLeftMargin+45,155,xLeftMargin+70,155,EPD_BLACK);
  display.drawBitmap(display.width()/2+42,104,epd_bitmap_humidity_icon_sm4,20,28,EPD_BLACK);

#endif

  // status message
  display.setFont();  // resets to system default monospace font
  display.setCursor(5, (display.height() - 9));
  display.print(messageText);

  display.display();
  debugMessage("completed screen update");
}

void batteryReadVoltage() 
{
  // check to see if i2C monitor is available
  if (lc.begin())
  // Check battery monitoring status
  {
    lc.setPackAPA(BATTERY_APA);
    hardwareData.batteryPercent = lc.cellPercent();
    hardwareData.batteryVoltage = lc.cellVoltage();
    batteryVoltageAvailable = true;
  } 
  else
  {
  // use supported boards to read voltage
    #if defined (ARDUINO_ADAFRUIT_FEATHER_ESP32_V2)
      pinMode(VBATPIN,INPUT);
      #define BATTV_MAX           4.2     // maximum voltage of battery
      #define BATTV_MIN           3.2     // what we regard as an empty battery

      // assumes default ESP32 analogReadResolution (4095)
      // the 1.05 is a fudge factor original author used to align reading with multimeter
      hardwareData.batteryVoltage = ((float)analogRead(VBATPIN) / 4095) * 3.3 * 2 * 1.05;
      hardwareData.batteryPercent = (uint8_t)(((hardwareData.batteryVoltage - BATTV_MIN) / (BATTV_MAX - BATTV_MIN)) * 100);

      // Adafruit ESP32 V2 power management guide code form https://learn.adafruit.com/adafruit-esp32-feather-v2/power-management-2, which does not work? [logged issue]
      // hardwareData.batteryVoltage = analogReadMilliVolts(VBATPIN);
      // hardwareData.batteryVoltage *= 2;    // we divided by 2, so multiply back
      // hardwareData.batteryVoltage /= 1000; // convert to volts!

      // manual percentage decay map from https://blog.ampow.com/lipo-voltage-chart/
      // hardwareData.batteryPercent = 100;
      // if ((hardwareData.batteryVoltage < 4.2) && (hardwareData.batteryVoltage > 4.15))
      //   hardwareData.batteryPercent = 95;
      // if ((hardwareData.batteryVoltage < 4.16) && (hardwareData.batteryVoltage > 4.10))
      //   hardwareData.batteryPercent = 90;
      // if ((hardwareData.batteryVoltage < 4.11) && (hardwareData.batteryVoltage > 4.07))
      //   hardwareData.batteryPercent = 85;
      // if ((hardwareData.batteryVoltage < 4.08) && (hardwareData.batteryVoltage > 4.01))
      //   hardwareData.batteryPercent = 80;
      // if ((hardwareData.batteryVoltage < 4.02) && (hardwareData.batteryVoltage > 3.97))
      //   hardwareData.batteryPercent = 75;
      // if ((hardwareData.batteryVoltage < 3.98) && (hardwareData.batteryVoltage > 3.94))
      //   hardwareData.batteryPercent = 70;
       // if ((hardwareData.batteryVoltage < 3.95) && (hardwareData.batteryVoltage > 3.90))
      // hardwareData.batteryPercent = 65;
      //  if ((hardwareData.batteryVoltage < 3.91) && (hardwareData.batteryVoltage > 3.87))
      //    hardwareData.batteryPercent = 60;
      //  if ((hardwareData.batteryVoltage < 3.87) && (hardwareData.batteryVoltage > 3.84))
      //    hardwareData.batteryPercent = 55;
      //  if (hardwareData.batteryVoltage = 3.84)
      //    hardwareData.batteryPercent = 50;
      //  if ((hardwareData.batteryVoltage < 3.84) && (hardwareData.batteryVoltage > 3.81))
      //    hardwareData.batteryPercent = 45;
      //  if ((hardwareData.batteryVoltage < 3.82) && (hardwareData.batteryVoltage > 3.79))
      //    hardwareData.batteryPercent = 40;
      //  if (hardwareData.batteryVoltage = 3.79)
      //    hardwareData.batteryPercent = 35;
      //  if ((hardwareData.batteryVoltage < 3.79) && (hardwareData.batteryVoltage > 3.76))
      //    hardwareData.batteryPercent = 30;
      //  if ((hardwareData.batteryVoltage < 3.77) && (hardwareData.batteryVoltage > 3.74))
      //    hardwareData.batteryPercent = 25;
      //  if ((hardwareData.batteryVoltage < 3.75) && (hardwareData.batteryVoltage > 3.72))
      //    hardwareData.batteryPercent = 20;      
      //  if ((hardwareData.batteryVoltage < 3.73) && (hardwareData.batteryVoltage > 3.70))
      //    hardwareData.batteryPercent = 15;
      //  if ((hardwareData.batteryVoltage < 3.73) && (hardwareData.batteryVoltage > 3.70))
      //    hardwareData.batteryPercent = 15;
      //  if ((hardwareData.batteryVoltage < 3.71) && (hardwareData.batteryVoltage > 3.68))
      //    hardwareData.batteryPercent = 10;
      //  if ((hardwareData.batteryVoltage < 3.69) && (hardwareData.batteryVoltage > 3.60))
      //    hardwareData.batteryPercent = 5;
      //  if (hardwareData.batteryVoltage < 3.61)
      //    hardwareData.batteryPercent = 0;
      batteryVoltageAvailable = true;
    #endif
  }
  if (batteryVoltageAvailable) 
  {
    debugMessage("Battery voltage: " + String(hardwareData.batteryVoltage) + " v");
    debugMessage("Battery percentage: " + String(hardwareData.batteryPercent) + " %");
  }
}

void screenBatteryStatus()
// Displays remaining battery % as graphic in lower right of screen
// used in XXXScreen() routines
{
  if (batteryVoltageAvailable) 
  {
    const int barHeight = 10;
    const int barWidth = 28;

    // battery nub (3pix wide, 6pix high)
    display.drawRect((display.width() - 8),7, 3, 6, EPD_BLACK);
    //battery percentage as rectangle fill
    display.fillRect((display.width() - barWidth - 8), 5, (int((hardwareData.batteryPercent / 100) * barWidth)), barHeight, EPD_GRAY);
    // battery border
    display.drawRect((display.width() - barWidth - 8), 5, barWidth, barHeight, EPD_BLACK);
    debugMessage("battery status drawn to screen");
  }
}

void screenWiFiStatus() 
{
  if (internetAvailable) 
  {
    const int barWidth = 3;
    const int barHeightMultiplier = 5;
    const int barSpacingMultipler = 5;
    const int barStartingXModifier = 35;
    int barCount;

    // Convert RSSI values to a 5 bar visual indicator
    // >90 means no signal
    barCount = (6-((hardwareData.rssi/10)-3));
    if (barCount>5) barCount = 5;
    if (barCount>0)
    {
      // <50 rssi value = 5 bars, each +10 rssi value range = one less bar
      // draw bars to represent WiFi strength
      for (int b = 1; b <= barCount; b++)
      {
        display.fillRect(((display.width() - barStartingXModifier) + (b * barSpacingMultipler)), ((display.height()) - (b * barHeightMultiplier)), barWidth, b * barHeightMultiplier, EPD_BLACK);
      }
      debugMessage(String("WiFi signal strength on screen as ") + barCount +" bars");
    }
    else
    {
      debugMessage("RSSI out of expected range");
    }
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
// reads SCD40 READS_PER_SAMPLE times then stores last read
{
  uint16_t error;
  char errorMessage[256];

  screenAlert("CO2 check");
  for (int loop=0; loop<READS_PER_SAMPLE; loop++)
  {
    // minimum time between SCD40 reads
    delay(5000);
    // read and store data if successful
    error = envSensor.readMeasurement(sensorData.ambientCO2, sensorData.ambientTempF, sensorData.ambientHumidity);
    // handle SCD40 errors
    if (error) {
      errorToString(error, errorMessage, 256);
      debugMessage(String(errorMessage) + " error during SCD4X read");
      return 0;
    }
    if (sensorData.ambientCO2<440 || sensorData.ambientCO2>6000)
    {
      debugMessage("SCD40 CO2 reading out of range");
      return 0;
    }
    //convert C to F for temp
    sensorData.ambientTempF = (sensorData.ambientTempF * 1.8) + 32;
    debugMessage(String("SCD40 read ") + loop + "of 5: " + sensorData.ambientTempF + "F, " + sensorData.ambientHumidity + "%, " + sensorData.ambientCO2 + " ppm");
  }
  return 1;
}

void enableInternalPower()
{
  // Handle two ESP32 I2C ports
  #if defined(ARDUINO_ADAFRUIT_QTPY_ESP32S2) || defined(ARDUINO_ADAFRUIT_QTPY_ESP32S3_NOPSRAM) || defined(ARDUINO_ADAFRUIT_QTPY_ESP32S3) || defined(ARDUINO_ADAFRUIT_QTPY_ESP32_PICO)
    // ESP32 is kinda odd in that secondary ports must be manually
    // assigned their pins with setPins()!
    Wire1.setPins(SDA1, SCL1);
    debugMessage("enabled ESP32 hardware with two I2C ports");
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

    // if you need to turn the neopixel on
    // pinMode(NEOPIXEL_POWER, OUTPUT);
    // digitalWrite(NEOPIXEL_POWER, HIGH);
    debugMessage("enabled Adafruit Feather ESP32S2 I2C power");
  #endif

  #if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2_TFT)
    pinMode(TFT_I2C_POWER, OUTPUT);
    digitalWrite(TFT_I2C_POWER, HIGH);
  #endif

  #if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32_V2)
    // Turn on the I2C power
    pinMode(NEOPIXEL_I2C_POWER, OUTPUT);
    digitalWrite(NEOPIXEL_I2C_POWER, HIGH);

    // Turn on neopixel
    // pinMode(NEOPIXEL_POWER, OUTPUT);
    // digitalWrite(NEOPIXEL_POWER, HIGH);
    debugMessage("enabled Adafruit Feather ESP32 V2 I2C power");
  #endif
}

void disableInternalPower(int deepSleepTime)
// Powers down hardware in preparation for board deep sleep
{
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

  #if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32_V2)
    // Turn off the I2C power
    pinMode(NEOPIXEL_I2C_POWER, OUTPUT);
    digitalWrite(NEOPIXEL_I2C_POWER, LOW);

    // if you need to turn the neopixel off
    // pinMode(NEOPIXEL_POWER, OUTPUT);
    // digitalWrite(NEOPIXEL_POWER, LOW);
    debugMessage("disabled Adafruit Feather ESP32 V2 I2C power");
  #endif

  #if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2)
    // Rev B board is LOW to enable
    // Rev C board is HIGH to enable
    digitalWrite(PIN_I2C_POWER, LOW);

    // if you need to turn the neopixel off
    // pinMode(NEOPIXEL_POWER, OUTPUT);
    // digitalWrite(NEOPIXEL_POWER, LOW);
    debugMessage("disabled Adafruit Feather ESP32S2 I2C power");
  #endif

  debugMessage(String("Going to sleep for ") + (deepSleepTime) + " second(s)");
  esp_sleep_enable_timer_wakeup(deepSleepTime*1000000); // ESP microsecond modifier
  esp_deep_sleep_start();
}