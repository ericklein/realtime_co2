/*
  Project Name:   realtime_co2
  Description:    Regularly sample and log temperature, humidity, and co2 levels

  See README.md for target information and revision history
*/

// hardware and internet configuration parameters
#include "config.h"
// private credentials for network, MQTT
#include "secrets.h"

// Generalized network handling
#include "aq_network.h"
AQ_Network aq_network;

// read/write to ESP32 persistent storage
#include <Preferences.h>
Preferences nvStorage;

// environment sensor data
typedef struct
{
  float     ambientTempF;
  float     ambientHumidity;
  uint16_t  ambientCO2;
} envData;
envData sensorData;

uint16_t co2Samples[co2MaxStoredSamples];

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
// Special glyphs for the UI
#include "glyphs.h"
//#include <Fonts/FreeSans24pt7b.h>

// 1.54" Monochrome displays with 200x200 pixels and SSD1681 chipset
ThinkInk_154_Mono_D67 display(EPD_DC, EPD_RESET, EPD_CS, SRAM_CS, EPD_BUSY);

// screen layout assists
const int xLeftMargin = 10;
const int xRightMargin = ((display.width()) - (2 * xLeftMargin));
const int yCO2 = 50;
const int ySparkline = 95;
const int ytemp = 170;
const int yMessage = display.height()- 9;
const int sparklineHeight = 40;

#ifdef INFLUX
  extern boolean post_influx(uint16_t co2, float tempF, float humidity, float battery_v, int rssi);
#endif

#ifdef MQTT
  // extern void mqttConnect();
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

  // retrieve the historical CO2 sample data
  int storedCounter;
  storedCounter = nvStorageRead();
  storedCounter++;
  // store the latest CO2 measurement in sample memory and nvStorage
  co2Samples[storedCounter] = sensorData.ambientCO2;
  nvStorageWrite(storedCounter);

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
    #ifdef DEBUG
      screenInfo("Test messsage");
    #else
      screenInfo("");
    #endif
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
// CO2 @ 50, temp/humid @ 125, sparkline @ 140, message/info @ 191

{
  debugMessage("Starting screen refresh");

  display.clearBuffer();
  display.setTextColor(EPD_BLACK);

  // display battery status
  screenBatteryStatus();

  // display wifi status
  screenWiFiStatus();

  // display sparkline
  screenSparkLines(xLeftMargin,ySparkline,(display.width() - (2* xLeftMargin)),sparklineHeight);

  // Indoor CO2 level
  // calculate CO2 value range in 400ppm bands
  int co2range = ((sensorData.ambientCO2 - 400) / 400);
  co2range = constrain(co2range,0,4); // filter CO2 levels above 2400

  display.setFont(&FreeSans18pt7b);
  display.setCursor(xLeftMargin, yCO2);
  display.print("CO");
  display.setCursor(xLeftMargin+65,yCO2);
  display.print(": " + String(co2Labels[co2range]));
  display.setFont(&FreeSans12pt7b);
  display.setCursor(xLeftMargin+50,yCO2+10);
  display.print("2");
  display.setCursor((xLeftMargin+90),yCO2+25);
  display.print("(" + String(sensorData.ambientCO2) + ")");

  // Indoor temp
  int tempF = sensorData.ambientTempF + 0.5;
  display.setFont(&FreeSans18pt7b);
  if(tempF < 100) {
    display.setCursor(xLeftMargin,ytemp);
    display.print(String(tempF));
    display.drawBitmap(xLeftMargin+42,ytemp-21,epd_bitmap_temperatureF_icon_sm,20,28,EPD_BLACK);
  }
  else {
    display.setCursor(xLeftMargin,ytemp);
    display.print(String(tempF));
    display.setFont(&FreeSans12pt7b);
    display.setCursor(xLeftMargin+65,ytemp);
    display.print("F"); 
  }

  // Indoor humidity
  display.setFont(&FreeSans18pt7b);
  display.setCursor(display.width()/2, ytemp);
  display.print(String((int)(sensorData.ambientHumidity + 0.5)));
  display.drawBitmap(display.width()/2+42,ytemp-21,epd_bitmap_humidity_icon_sm4,20,28,EPD_BLACK);

  // status message
  display.setFont();  // resets to system default monospace font
  display.setCursor(5, yMessage);
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
    debugMessage(String("Battery voltage: ") + hardwareData.batteryVoltage + "v, percent: " + hardwareData.batteryPercent + " %");
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

void screenSparkLines(int xStart, int yStart, int xWidth, int yHeight)
{
  // load test CO2 (if needed)
  //sparkLineTestValues(co2MaxStoredSamples);

  uint16_t co2Min, co2Max = co2Samples[0];
  // # of pixels between each samples x and y coordinates
  int xPixelStep, yPixelStep;

  int sparkLineX[co2MaxStoredSamples], sparkLineY[co2MaxStoredSamples];

  // horizontal distance (pixels) between each displayed co2 value
  xPixelStep = (xWidth / (co2MaxStoredSamples - 1));

  // determine min/max of CO2 samples
  // could use recursive function but co2MaxStoredSamples should always be relatively small
  for(int i=0;i<co2MaxStoredSamples;i++)
  {
    if(co2Samples[i] > co2Max) co2Max = co2Samples[i];
    if(co2Samples[i] < co2Min) co2Min = co2Samples[i];
  }
  debugMessage(String("Max CO2 in stored sample range is ") + co2Max);
  debugMessage(String("Min CO2 in stored sample range is ") + co2Min);
 
  // vertical distance (pixels) between each displayed co2 value
  yPixelStep = ((co2Max - co2Min) / yHeight);

  // sparkline border box (if needed)
  //display.drawRect(xLeftMargin,ySparkline, xRightMargin,sparklineHeight, EPD_BLACK);

  // determine sparkline x,y values
  for(int i=0;i<co2MaxStoredSamples;i++)
  {
    sparkLineX[i] = (xStart + (i * xPixelStep));
    sparkLineY[i] = ((yStart + yHeight) - int((co2Samples[i]-co2Min) / yPixelStep));
    // draw/extend sparkline after first value is generated
    if (i != 0)
      display.drawLine(sparkLineX[i-1],sparkLineY[i-1],sparkLineX[i],sparkLineY[i],EPD_BLACK);  
  }
  for (int i=0;i<co2MaxStoredSamples;i++)
  {
    debugMessage(String("X,Y coordinates for CO2 sample ") + i + " is " + sparkLineX[i] + "," + sparkLineY[i]);
  }
    debugMessage("sparkline drawn to screen");
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
    debugMessage("wifi meter drawn to screen");
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
  for (int loop=1; loop<=READS_PER_SAMPLE; loop++)
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
    debugMessage(String("SCD40 read ") + loop + " of 5: " + sensorData.ambientTempF + "F, " + sensorData.ambientHumidity + "%, " + sensorData.ambientCO2 + " ppm");
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

  debugMessage(String("Going to sleep for ") + (deepSleepTime) + " seconds");
  esp_sleep_enable_timer_wakeup(deepSleepTime*1000000); // ESP microsecond modifier
  esp_deep_sleep_start();
}

void sparkLineTestValues(int sampleSetSize)
// generates test data to exercise the screenSparkLine function
{
    // generate test data
  for(int i=0;i<sampleSetSize;i++)
  {
    // standard range for indoor CO2 values
    co2Samples[i]=random(600,2400);
  }
}

int nvStorageRead()
{
  int storedCounter;

  nvStorage.begin("rco2", false);
  storedCounter = nvStorage.getInt("counter", -1);
  // reset the counter if needed
  if (storedCounter == (co2MaxStoredSamples-1))
  {
    storedCounter = -1;
  }
  debugMessage(String("Retrieved CO2 sample pointer is: ") + storedCounter);

  String nvStoreBaseName;
  for (int i=0; i<co2MaxStoredSamples; i++)
  {
    nvStoreBaseName = "co2Sample" + String(i);
    //debugMessage(nvStoreBaseName);
    // get previously stored values. If they don't exist, create them as 400 (CO2 floor)
    co2Samples[i] = nvStorage.getLong(nvStoreBaseName.c_str(),400);
    debugMessage(String(nvStoreBaseName) + " retrieved from nv storage is " + co2Samples[i]);
  }
  return storedCounter;
}

void nvStorageWrite(int storedCounter)
// Stores current CO2 value into nv storage in a FIFO rotation
// FIX: validate storedCounter with 0 and co2MaxStoredSamples
{
  nvStorage.putInt("counter", storedCounter);
  String nvStoreBaseName = "co2Sample" + String(storedCounter);
  nvStorage.putLong(nvStoreBaseName.c_str(),sensorData.ambientCO2);
  debugMessage(String(nvStoreBaseName) + " stored in nv storage as " + sensorData.ambientCO2);
}