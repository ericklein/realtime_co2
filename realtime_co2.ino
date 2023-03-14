/*
  Project Name:   realtime_co2
  Description:    Regularly sample and log temperature, humidity, and co2 levels

  See README.md for target information and revision history
*/

// hardware and internet configuration parameters
#include "config.h"
// private credentials for network, MQTT
#include "secrets.h"

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

#ifdef WIFI
  #if defined(ESP8266)
    #include <ESP8266WiFi.h>
  #elif defined(ESP32)
    #include <WiFi.h>
  #endif

  // NTP setup
  #include "time.h"
#endif

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

// 1.54" Monochrome display with 200x200 pixels and SSD1681 chipset
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
  extern boolean post_influx(uint16_t co2, float tempF, float humidity, float batteryVoltage, int rssi);
#endif

#ifdef MQTT
  // MQTT uses WiFiClient class to create TCP connections
  WiFiClient client;

  #include <Adafruit_MQTT.h>
  #include <Adafruit_MQTT_Client.h>
  Adafruit_MQTT_Client aq_mqtt(&client, MQTT_BROKER, MQTT_PORT, CLIENT_ID, MQTT_USER, MQTT_PASS);

  extern bool mqttDeviceWiFiUpdate(int rssi);
  extern bool mqttDeviceBatteryUpdate(float batteryVoltage);
  extern bool mqttSensorTempFUpdate(float tempF);
  extern bool mqttSensorHumidityUpdate(float humidity);
  extern bool mqttSensorCO2Update(uint16_t co2);
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

  hardwareData.batteryVoltage = 0;  // 0 = no battery attached
  hardwareData.rssi = 0;            // 0 = no WiFi 

  powerEnable();

  display.begin(THINKINK_MONO);
  display.setRotation(DISPLAY_ROTATION);

  // Initialize environmental sensor
  if (!sensorInit()) {
    debugMessage("Environment sensor failed to initialize");
    screenAlert("NO SCD40");
    // This error often occurs right after a firmware flash and reset.
    // Hardware deep sleep typically resolves it, so quickly cycle the hardware
    powerDisable(HARDWARE_ERROR_INTERVAL);
  }

  // Environmental sensor available, so fetch values
  if (!sensorRead()) {
    debugMessage("SCD40 returned no/bad data");
    screenAlert("SCD40 no/bad data");
    powerDisable(HARDWARE_ERROR_INTERVAL);
  }

  // retrieve the historical CO2 sample data
  int storedCounter;
  storedCounter = nvStorageRead();
  storedCounter++;
  // store the latest CO2 measurement in sample memory and nvStorage
  co2Samples[storedCounter] = sensorData.ambientCO2;
  nvStorageWrite(storedCounter);

  batteryReadVoltage();
  networkConnect();

  String upd_flags = "";  // Indicates whether/which external data services were updated
  if (hardwareData.rssi!=0)
  {
    networkGetTime();
    // Update external data services
    #ifdef MQTT
      if ((mqttSensorTempFUpdate(sensorData.ambientTempF)) && (mqttSensorHumidityUpdate(sensorData.ambientHumidity)) && (mqttSensorCO2Update(sensorData.ambientCO2)) && (mqttDeviceWiFiUpdate(hardwareData.rssi)) && (mqttDeviceBatteryUpdate(hardwareData.batteryVoltage)))
      {
          upd_flags += "M";
      }
    #endif

    #ifdef INFLUX
      // Returns true if successful
      if (post_influx(sensorData.ambientCO2, sensorData.ambientTempF, sensorData.ambientHumidity, hardwareData.batteryVoltage, hardwareData.rssi))
      {
        upd_flags += "I";
      }
    #endif

    if (upd_flags == "") 
    {
      // External data services not updated but we have network time
      screenInfo(dateTimeString());
    } 
    else 
    {
      // External data services not updated and we have network time
      screenInfo("[+" + upd_flags + "] " + dateTimeString());
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
  powerDisable(SAMPLE_INTERVAL);
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
  } 
  else
  {
    // use supported boards to read voltage
    #if defined (ARDUINO_ADAFRUIT_FEATHER_ESP32_V2)
      pinMode(VBATPIN,INPUT);

      // assumes default ESP32 analogReadResolution (4095)
      // the 1.05 is a fudge factor original author used to align reading with multimeter
      hardwareData.batteryVoltage = ((float)analogRead(VBATPIN) / 4095) * 3.3 * 2 * 1.05;
      hardwareData.batteryPercent = (uint8_t)(((hardwareData.batteryVoltage - batteryMinVoltage) / (batteryMaxVoltage - batteryMinVoltage)) * 100);
    #endif
  }
  if (hardwareData.batteryVoltage!=0) 
  {
    debugMessage(String("Battery voltage: ") + hardwareData.batteryVoltage + "v, percent: " + hardwareData.batteryPercent + "%");
  }
}

void screenBatteryStatus()
// Displays remaining battery % as graphic in lower right of screen
// used in XXXScreen() routines
{
  if (hardwareData.batteryVoltage!=0) 
  {
    const int barHeight = 10;
    const int barWidth = 28;

    // battery nub (3pix wide, 6pix high)
    display.drawRect((display.width() - 8),7, 3, 6, EPD_BLACK);
    //battery percentage as rectangle fill
    display.fillRect((display.width() - barWidth - 8), 5, (int((hardwareData.batteryPercent / 100) * barWidth)), barHeight, EPD_GRAY);
    // battery border
    display.drawRect((display.width() - barWidth - 8), 5, barWidth, barHeight, EPD_BLACK);
    debugMessage(String("battery status drawn to screen as ") + hardwareData.batteryPercent + "%" );
  }
}

void screenSparkLines(int xStart, int yStart, int xWidth, int yHeight)
{
  // TEST ONLY: load test CO2 values
  //sparkLineTestValues(co2MaxStoredSamples);

  uint16_t co2Min = co2Samples[0];
  uint16_t co2Max = co2Samples[0];
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
  debugMessage(String("Max CO2 in stored sample range is ") + co2Max +", min is " + co2Min);
 
  // vertical distance (pixels) between each displayed co2 value
  yPixelStep = round(((co2Max - co2Min) / yHeight)+.5);
  debugMessage(String("xPixelStep is ") + xPixelStep + ", yPixelStep is " + yPixelStep);

  // sparkline border box (if needed)
  //display.drawRect(xLeftMargin,ySparkline, xRightMargin,sparklineHeight, EPD_BLACK);

  // determine sparkline x,y values
  for(int i=0;i<co2MaxStoredSamples;i++)
  {
    sparkLineX[i] = (xStart + (i * xPixelStep));
    sparkLineY[i] = ((yStart + yHeight) - (int)((co2Samples[i]-co2Min) / yPixelStep));
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
  if (hardwareData.rssi!=0) 
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
      debugMessage(String("WiFi signal strength drawn to screen as ") + barCount +" bars");
    }
    else
    {
      debugMessage("RSSI out of expected range");
    }
  }
}

bool sensorInit() {
  char errorMessage[256];

  #if defined(ARDUINO_ADAFRUIT_QTPY_ESP32S2) || defined(ARDUINO_ADAFRUIT_QTPY_ESP32S3_NOPSRAM) || defined(ARDUINO_ADAFRUIT_QTPY_ESP32S3) || defined(ARDUINO_ADAFRUIT_QTPY_ESP32_PICO)
    // these boards have two I2C ports so we have to initialize the appropriate port
    Wire1.begin();
    envSensor.begin(Wire1);
  #else
    // only one I2C port
    Wire.begin();
    envSensor.begin(Wire);
  #endif

  envSensor.wakeUp();
  envSensor.setSensorAltitude(SITE_ALTITUDE);  // optimizes CO2 reading

  uint16_t error = envSensor.startPeriodicMeasurement();
  if (error) {
    // Failed to initialize SCD40
    errorToString(error, errorMessage, 256);
    debugMessage(String(errorMessage) + " executing SCD40 startPeriodicMeasurement()");
    return false;
  } 
  else
  {
    debugMessage("SCD40 initialized");
    return true;
  }
}

bool sensorRead()
// reads SCD40 READS_PER_SAMPLE times then stores last read
{
  char errorMessage[256];

  screenAlert("CO2 check");
  for (int loop=1; loop<=READS_PER_SAMPLE; loop++)
  {
    // SCD40 datasheet suggests 5 second delay between SCD40 reads
    delay(5000);
    uint16_t error = envSensor.readMeasurement(sensorData.ambientCO2, sensorData.ambientTempF, sensorData.ambientHumidity);
    // handle SCD40 errors
    if (error) {
      errorToString(error, errorMessage, 256);
      debugMessage(String(errorMessage) + " error during SCD4X read");
      return false;
    }
    if (sensorData.ambientCO2<400 || sensorData.ambientCO2>6000)
    {
      debugMessage("SCD40 CO2 reading out of range");
      return false;
    }
    //convert C to F for temp
    sensorData.ambientTempF = (sensorData.ambientTempF * 1.8) + 32;
    debugMessage(String("SCD40 read ") + loop + " of " + READS_PER_SAMPLE + " : " + sensorData.ambientTempF + "F, " + sensorData.ambientHumidity + "%, " + sensorData.ambientCO2 + " ppm");
  }
  return true;
}

void powerEnable()
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

void powerDisable(int deepSleepTime)
// Powers down hardware in preparation for board deep sleep
{
  char errorMessage[256];

  debugMessage("Starting power down activities");
  // power down epd
  display.powerDown();
  digitalWrite(EPD_RESET, LOW);  // hardware power down mode
  debugMessage("powered down epd");

  networkDisconnect();

  // power down SCD40

  // stops potentially started measurement then powers down SCD40
  uint16_t error = envSensor.stopPeriodicMeasurement();
  if (error) {
    errorToString(error, errorMessage, 256);
    debugMessage(String(errorMessage) + " executing SCD40 stopPeriodicMeasurement()");
  }
  envSensor.powerDown();
  debugMessage("SCD40 powered down");

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

  esp_sleep_enable_timer_wakeup(deepSleepTime*1000000); // ESP microsecond modifier
  debugMessage(String("Going into ESP32 deep sleep for ") + (deepSleepTime) + " seconds");
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

bool networkConnect()
{
  #ifdef WIFI
    // set hostname has to come before WiFi.begin
    WiFi.hostname(CLIENT_ID);

    WiFi.begin(WIFI_SSID, WIFI_PASS);

    for (int tries = 1; tries <= CONNECT_ATTEMPT_LIMIT; tries++)
    // Attempts WiFi connection, and if unsuccessful, re-attempts after CONNECT_ATTEMPT_INTERVAL second delay for CONNECT_ATTEMPT_LIMIT times
    {
      if (WiFi.status() == WL_CONNECTED)
      {
        hardwareData.rssi = abs(WiFi.RSSI());
        debugMessage(String("WiFi IP address lease from ") + WIFI_SSID + " is " + WiFi.localIP().toString());
        debugMessage(String("WiFi RSSI is: ") + hardwareData.rssi + " dBm");
        return true;
      }
      debugMessage(String("Connection attempt ") + tries + " of " + CONNECT_ATTEMPT_LIMIT + " to " + WIFI_SSID + " failed");
      // use of delay() OK as this is initialization code
      delay(CONNECT_ATTEMPT_INTERVAL * 1000); // convered into milliseconds
    }
  #endif
  return false;
}

void networkDisconnect()
{
#ifdef WIFI
  WiFi.disconnect();
  debugMessage("Disconnected from WiFi network");
#endif
}

void networkGetTime()
{
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);    
  debugMessage("NTP time: " + dateTimeString());
}

// Converts system time into human readable strings. Use NTP service
String dateTimeString() {
  String dateTime;

  #ifdef WIFI
    struct tm timeInfo;
    if (getLocalTime(&timeInfo)) {
      int day = timeInfo.tm_wday;
      // int month = timeInfo.tm_mon;
      // int year = timeInfo.tm_year + 1900;
      int hour = timeInfo.tm_hour;
      int minutes = timeInfo.tm_min;
      // int seconds = timeinfo.tm_sec;

      // short human readable format
      dateTime = weekDays[day];
      dateTime += " at ";
      if (hour < 10) dateTime += "0";
      dateTime += hour;
      dateTime += ":";
      if (minutes < 10) dateTime += "0";
      dateTime += minutes;

      // long human readable
      // dateTime = weekDays[day];
      // dateTime += ", ";

      // if (month<10) dateTime += "0";
      // dateTime += month;
      // dateTime += "-";
      // if (day<10) dateTime += "0";
      // dateTime += day;
      // dateTime += " at ";
      // if (hour<10) dateTime += "0";
      // dateTime += hour;
      // dateTime += ":";
      // if (minutes<10) dateTime += "0";
      // dateTime += minutes;

      // zulu format
      // dateTime = year + "-";
      // if (month()<10) dateTime += "0";
      // dateTime += month;
      // dateTime += "-";
      // if (day()<10) dateTime += "0";
      // dateTime += day;
      // dateTime += "T";
      // if (hour<10) dateTime += "0";
      // dateTime += hour;
      // dateTime += ":";
      // if (minutes<10) dateTime += "0";
      // dateTime += minutes;
      // dateTime += ":";
      // if (seconds<10) dateTime += "0";
      // dateTime += seconds;
      // switch (gmtOffset_sec)
      // {
      //   case 0:
      //     dateTime += "Z";
      //     break;
      //   case -28800:
      //     dateTime += "PDT";
      //     break;
      // }
    } else {
      dateTime = "Can't reach time service";
    }
  #else
    // If no network defined
    dateTime = "No network to set time";
  #endif

  return dateTime;
}