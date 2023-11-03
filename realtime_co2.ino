/*
  Project:      realtime_co2
  Description:  Regularly sample and log temperature, humidity, and co2 levels

  See README.md for target information
*/

// hardware and internet configuration parameters
#include "config.h"
// private credentials for network, MQTT
#include "secrets.h"

// read/write to ESP32 persistent storage
#include <Preferences.h>
Preferences nvStorage;

// environment sensor data
typedef struct envData
{
  float     ambientTemperatureF;
  float     ambientHumidity;     // RH [%]  
  uint16_t  ambientCO2;
} envData;
envData sensorData;

uint16_t co2Samples[co2MaxStoredSamples];

// hardware status data
typedef struct hdweData
{
  float batteryPercent;
  float batteryVoltage;
  float batteryTemperatureF;
  int rssi;
} hdweData;
hdweData hardwareData;

// activate only if using network data endpoints
#if defined(MQTT) || defined(INFLUX) || defined(HASSIO_MQTT)  
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
#include <GxEPD2_BW.h>
GxEPD2_BW<GxEPD2_154_D67, GxEPD2_154_D67::HEIGHT> display(GxEPD2_154_D67(EPD_CS, EPD_DC, EPD_RESET, EPD_BUSY)); // GDEH0154D67

#include "Fonts/meteocons16pt7b.h"
#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSans12pt7b.h>
#include <Fonts/FreeSans18pt7b.h>

// Special glyphs for the UI
#include "glyphs.h"

// screen layout assists
const int xMargins = 10;
const int yMargins = 2;
const int yCO2 = 50;
const int ySparkline = 95;
const int yTemperature = 170;
const int sparklineHeight = 40;
const int batteryBarWidth = 28;
const int batteryBarHeight = 10;
const int wifiBarWidth = 3;
const int wifiBarHeightIncrement = 3;
const int wifiBarSpacing = 5;

#ifdef DWEET
  extern void post_dweet(uint16_t co2, float temperatureF, float humidity, float battv, int rssi);
#endif 

#ifdef INFLUX
  extern boolean post_influx(uint16_t co2, float temperatureF, float humidity, float batteryVoltage, int rssi);
#endif

#ifdef MQTT
  // MQTT uses WiFiClient class to create TCP connections
  WiFiClient client;

  #include <Adafruit_MQTT.h>
  #include <Adafruit_MQTT_Client.h>
  Adafruit_MQTT_Client aq_mqtt(&client, MQTT_BROKER, MQTT_PORT, DEVICE_ID, MQTT_USER, MQTT_PASS);

  extern bool mqttDeviceWiFiUpdate(int rssi);
  extern bool mqttDeviceBatteryUpdate(float batteryVoltage);
  extern bool mqttSensorTemperatureFUpdate(float temperatureF);
  extern bool mqttSensorHumidityUpdate(float humidity);
  extern bool mqttSensorCO2Update(uint16_t co2);
  #ifdef HASSIO_MQTT
    extern void hassio_mqtt_publish(uint16_t co2,float temperatureF,float humidity,float batteryVoltage);
  #endif
#endif

void setup()
// One time run of code, then deep sleep
{
  // handle Serial first so debugMessage() works
  #ifdef DEBUG
    Serial.begin(115200);
    // wait for serial port connection
    while (!Serial);
  #endif

  debugMessage("realtime_co2 Device ID: " + String(DEVICE_ID),1);
  debugMessage(String(SAMPLE_INTERVAL) + " second sample interval",2);

  hardwareData.rssi = 0;  // 0 = no WiFi 

  powerI2CEnable();

  // initiate first to display hardware error messages
  //display.init(115200); // default 10ms reset pulse, e.g. for bare panels with DESPI-C02
  display.init(115200, true, 2, false); // USE THIS for Waveshare boards with "clever" reset circuit, 2ms reset pulse

  display.setRotation(DISPLAY_ROTATION);

  // SCD40 stops initializing below battery threshold, so detect that first
  hardwareData.batteryVoltage = 0;  // 0 = no battery attached
  batteryRead(batteryReads);
  if (hardwareData.batteryVoltage < voltageTable[4])
  {
    debugMessage("Battery below required threshold, rebooting",1);
    screenAlert(40, ((display.height()/2)+6), "Low battery");
    // this is a recursive boot sequence
    powerDisable(HARDWARE_ERROR_INTERVAL);
  }

  // Initialize environmental sensor
  if (!sensorCO2Init()) {
    debugMessage("Environment sensor failed to initialize",1);
    screenAlert(40, ((display.height()/2)+6), "No SCD40");
    // This error often occurs right after a firmware flash and reset.
    // Hardware deep sleep typically resolves it, so quickly cycle the hardware
    powerDisable(HARDWARE_ERROR_INTERVAL);
  }

  // Environmental sensor available, so fetch values
  if (!sensorCO2Read()) {
    debugMessage("SCD40 returned no/bad data",1);
    screenAlert(40, ((display.height()/2)+6),"SCD40 read issue");
    powerDisable(HARDWARE_ERROR_INTERVAL);
  }

  // retrieve the historical CO2 sample data
  int storedCounter;
  storedCounter = nvStorageRead();
  storedCounter++;
  // store the latest CO2 measurement in sample memory and nvStorage
  co2Samples[storedCounter] = sensorData.ambientCO2;
  nvStorageWrite(storedCounter);

  if (networkConnect())
  {
    String upd_flags = "";  // Indicates whether/which external data services were updated

    networkGetTime(timeZoneString);
    // Update external data services
    #ifdef MQTT
      if ((mqttSensorTemperatureFUpdate(sensorData.ambientTemperatureF)) && (mqttSensorHumidityUpdate(sensorData.ambientHumidity)) && (mqttSensorCO2Update(sensorData.ambientCO2)) && (mqttDeviceWiFiUpdate(hardwareData.rssi)) && (mqttDeviceBatteryUpdate(hardwareData.batteryVoltage)))
      {
          upd_flags += "M";
      }
      #ifdef HASSIO_MQTT
        debugMessage("Establishing MQTT for Home Assistant",1);
        // Either configure sensors in Home Assistant's configuration.yaml file
        // directly or attempt to do it via MQTT auto-discovery
        // hassio_mqtt_setup();  // Config for MQTT auto-discovery
        hassio_mqtt_publish(sensorData.ambientCO2, sensorData.ambientTemperatureF, sensorData.ambientHumidity, hardwareData.batteryVoltage);
      #endif
    #endif

    #ifdef INFLUX
      // Returns true if successful
      if (post_influx(sensorData.ambientCO2, sensorData.ambientTemperatureF, sensorData.ambientHumidity, hardwareData.batteryVoltage, hardwareData.rssi))
      {
        upd_flags += "I";
      }
    #endif

    #ifdef DWEET
      // Fire and forget posting of device & sensor status via Dweet.io (no update flags)
      post_dweet(sensorData.ambientCO2, sensorData.ambientTemperatureF, sensorData.ambientHumidity, hardwareData.batteryVoltage, hardwareData.rssi);
      upd_flags += "D";
    #endif
    if (upd_flags == "") 
    {
      // External data services not updated but we have network time
      screenInfo(dateTimeString("short"));
    } 
    else 
    {
      // External data services updated and we have network time
      screenInfo("[+" + upd_flags + "] " + dateTimeString("short"));
    }
  }
  else
  {
    // no internet connection, update screen with sensor data only
    screenInfo("");
  }
  powerDisable(SAMPLE_INTERVAL);
}

void loop() {}

void debugMessage(String messageText, int messageLevel)
// wraps Serial.println as #define conditional
{
  #ifdef DEBUG
    if (messageLevel <= DEBUG)
    {
      Serial.println(messageText);
      Serial.flush();  // Make sure the message gets output (before any sleeping...)
    }
  #endif
}

void screenAlert(int initialX, int initialY, String messageText)
// Display critical error message on screen
{
  debugMessage("screenAlert refresh started",1);

  display.setTextColor(GxEPD_BLACK);
  display.setFont(&FreeSans12pt7b);
  display.setCursor(initialX, initialY);
  display.setFullWindow();
  display.firstPage();
  do
  {
    display.fillScreen(GxEPD_WHITE);
    display.print(messageText);
  }
  while (display.nextPage());

  debugMessage("screenAlert refresh complete",1);
}

void screenInfo(String messageText)
// Display environmental information
// CO2 @ 50, temp/humid @ 125, sparkline @ 140, message/info @ 191
{
  debugMessage("screenInfo refresh started",1);
  
  display.setTextColor(GxEPD_BLACK);
  display.setFullWindow();
  display.firstPage();
  do
  {
    display.fillScreen(GxEPD_WHITE);
    // screen helper routines
    // display battery level in the upper right corner, -3 in first parameter accounts for battery nub
    screenHelperBatteryStatus((display.width()-xMargins-batteryBarWidth-3),yMargins,batteryBarWidth, batteryBarHeight);

    // display wifi status left offset to the battery level indicator
    screenHelperWiFiStatus((display.width() - 35), (display.height() - yMargins),wifiBarWidth,wifiBarHeightIncrement,wifiBarSpacing);

    // draws any status message in the lower left corner. -8 in the first parameter accounts for fixed font height
    screenHelperStatusMessage(xMargins, (display.height()-yMargins-8), messageText);

    // display sparkline
    screenHelperSparkLine(xMargins,ySparkline,(display.width() - (2* xMargins)),sparklineHeight);

    // Indoor CO2 level
    // calculate CO2 value range in 400ppm bands
    int co2range = ((sensorData.ambientCO2 - 400) / 400);
    co2range = constrain(co2range,0,4); // filter CO2 levels above 2400

    display.setFont(&FreeSans18pt7b);
    display.setCursor(xMargins, yCO2);
    display.print("CO");
    display.setCursor(xMargins+65,yCO2);
    display.print(": " + String(co2Labels[co2range]));
    display.setFont(&FreeSans12pt7b);
    display.setCursor(xMargins+50,yCO2+10);
    display.print("2");
    display.setCursor((xMargins+90),yCO2+25);
    display.print("(" + String(sensorData.ambientCO2) + ")");

    // Indoor temp
    int temperatureF = sensorData.ambientTemperatureF + 0.5;
    display.setFont(&FreeSans18pt7b);
    if(temperatureF < 100)
    {
      display.setCursor(xMargins,yTemperature);
      display.print(String(temperatureF));
      display.drawBitmap(xMargins+42,yTemperature-21,epd_bitmap_temperatureF_icon_sm,20,28,GxEPD_BLACK);
    }
    else 
    {
      display.setCursor(xMargins,yTemperature);
      display.print(String(temperatureF));
      display.setFont(&FreeSans12pt7b);
      display.setCursor(xMargins+65,yTemperature);
      display.print("F"); 
    }

    // Indoor humidity
    display.setFont(&FreeSans18pt7b);
    display.setCursor(display.width()/2, yTemperature);
    display.print(String((int)(sensorData.ambientHumidity + 0.5)));
    display.drawBitmap(display.width()/2+42,yTemperature-21,epd_bitmap_humidity_icon_sm4,20,28,GxEPD_BLACK);
  }
  while (display.nextPage());

  debugMessage("screenInfo refresh complete",1);
}

void batteryRead(int reads) 
{
  // check to see if i2C monitor is available
  if (lc.begin())
  // Check battery monitoring status
  {
    debugMessage(String("Version: 0x")+lc.getICversion(),2);
    lc.setPackAPA(BATTERY_APA);
    lc.setThermistorB(3950);

    hardwareData.batteryPercent = lc.cellPercent();
    hardwareData.batteryVoltage = lc.cellVoltage();
    hardwareData.batteryTemperatureF = 32 + (1.8* lc.getCellTemperature());
  } 
  else
  {
    // use supported boards to read voltage
    #if defined (ARDUINO_ADAFRUIT_FEATHER_ESP32_V2)
      // modified from the Adafruit power management guide for Adafruit ESP32V2
      float accumulatedVoltage = 0;
      for (int loop = 0; loop < reads; loop++)
      {
        accumulatedVoltage += analogReadMilliVolts(VBATPIN);
      }
      hardwareData.batteryVoltage = accumulatedVoltage/reads; // we now have the average reading
      // convert into volts  
      hardwareData.batteryVoltage *= 2;    // we divided by 2, so multiply back
      hardwareData.batteryVoltage /= 1000; // convert to volts!
      hardwareData.batteryVoltage *= 2;     // we divided by 2, so multiply back
      // ESP32 suggested algo
      // hardwareData.batteryVoltage *= 3.3;   // Multiply by 3.3V, our reference voltage
      // hardwareData.batteryVoltage *= 1.05;  // the 1.05 is a fudge factor original author used to align reading with multimeter
      // hardwareData.batteryVoltage /= 4095;  // assumes default ESP32 analogReadResolution (4095)
      hardwareData.batteryPercent = batteryGetChargeLevel(hardwareData.batteryVoltage);
    #endif
  }
  if (hardwareData.batteryVoltage!=0) 
  {
    debugMessage(String("Battery voltage: ") + hardwareData.batteryVoltage + "v, percent: " + hardwareData.batteryPercent + "%",1);
  }
}

int batteryGetChargeLevel(float volts)
{
  int idx = 50;
  int prev = 0;
  int half = 0;
  if (volts >= 4.2){
    return 100;
  }
  if (volts <= 3.2){
    return 0;
  }
  while(true){
    half = abs(idx - prev) / 2;
    prev = idx;
    if(volts >= voltageTable[idx]){
      idx = idx + half;
    }else{
      idx = idx - half;
    }
    if (prev == idx){
      break;
    }
  }
  return idx;
}

void screenHelperBatteryStatus(int initialX, int initialY, int barWidth, int barHeight)
// helper function for screenXXX() routines that draws battery charge %
{
  // IMPROVEMENT : Screen dimension boundary checks for passed parameters
  // IMPROVEMENT : Check for offscreen drawing based on passed parameters
  if (hardwareData.batteryVoltage>0) 
  {
    // battery nub; width = 3pix, height = 60% of barHeight
    display.fillRect((initialX+barWidth), (initialY+(int(barHeight/5))), 3, (int(barHeight*3/5)), GxEPD_BLACK);
    // battery border
    display.drawRect(initialX, initialY, barWidth, barHeight, GxEPD_BLACK);
    //battery percentage as rectangle fill, 1 pixel inset from the battery border
    display.fillRect((initialX + 2), (initialY + 2), int(0.5+(hardwareData.batteryPercent*((barWidth-4)/100.0))), (barHeight - 4), GxEPD_BLACK);
    debugMessage(String("battery: ") + hardwareData.batteryPercent + "%, " + int(0.5+(hardwareData.batteryPercent*((barWidth-4)/100.0))) + " of " + (barWidth-4) + " pixels",1);
  }
  else
    debugMessage("No battery voltage for screenHelperBatteryStatus to render",1);
}

void screenHelperSparkLine(int xStart, int yStart, int xWidth, int yHeight)
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
  debugMessage(String("Max CO2 in stored sample range: ") + co2Max +", min: " + co2Min,2);
 
  // vertical distance (pixels) between each displayed co2 value
  yPixelStep = round(((co2Max - co2Min) / yHeight)+.5);
  debugMessage(String("xPixelStep: ") + xPixelStep + ", yPixelStep: " + yPixelStep,2);

  // sparkline border box (if needed)
  //display.drawRect(xMargins,ySparkline, ((display.width()) - (2 * xMargins)),sparklineHeight, GxEPD_BLACK);

  // determine sparkline x,y values
  for(int i=0;i<co2MaxStoredSamples;i++)
  {
    sparkLineX[i] = (xStart + (i * xPixelStep));
    sparkLineY[i] = ((yStart + yHeight) - (int)((co2Samples[i]-co2Min) / yPixelStep));
    // draw/extend sparkline after first value is generated
    if (i != 0)
      display.drawLine(sparkLineX[i-1],sparkLineY[i-1],sparkLineX[i],sparkLineY[i],GxEPD_BLACK);  
  }
  for (int i=0;i<co2MaxStoredSamples;i++)
  {
    debugMessage(String("X,Y coordinates for CO2 sample ") + i + ": " + sparkLineX[i] + "," + sparkLineY[i],2);
  }
    debugMessage("sparkline drawn to screen",2);
}

void screenHelperWiFiStatus(int initialX, int initialY, int barWidth, int barHeightMultiplier, int barSpacingMultipler)
// helper function for screenXXX() routines that draws WiFi signal strength
{
  if (hardwareData.rssi!=0) 
  {
    // Convert RSSI values to a 5 bar visual indicator
    // >90 means no signal
    int barCount = constrain((6-((hardwareData.rssi/10)-3)),0,5);
    if (barCount>0)
    {
      // <50 rssi value = 5 bars, each +10 rssi value range = one less bar
      // draw bars to represent WiFi strength
      for (int b = 1; b <= barCount; b++)
      {
        display.fillRect((initialX + (b * barSpacingMultipler)), (initialY - (b * barHeightMultiplier)), barWidth, b * barHeightMultiplier, GxEPD_BLACK);
      }
      debugMessage(String("WiFi signal strength on screen as ") + barCount +" bars",2);
    }
    else
    {
      // you could do a visual representation of no WiFi strength here
      debugMessage("RSSI too low, no display",1);
    }
  }
}

void screenHelperStatusMessage(int initialX, int initialY, String messageText)
// helper function for screenXXX() routines that draws a status message
// uses system default font, so text drawn x+,y+ from initialX,Y
{
  // IMPROVEMENT : Screen dimension boundary checks for function parameters
  display.setFont();  // resets to system default monospace font (6x8 pixels)
  display.setCursor(initialX, initialY);
  display.print(messageText);
}

bool sensorCO2Init() {
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
    debugMessage(String(errorMessage) + " executing SCD40 startPeriodicMeasurement()",1);
    return false;
  } 
  else
  {
    debugMessage("power on: SCD40",1);
    return true;
  }
}

bool sensorCO2Read()
// reads SCD40 READS_PER_SAMPLE times then stores last read
{
  char errorMessage[256];

  screenAlert(40, ((display.height()/2)+6), "CO2 check");
  for (int loop=1; loop<=READS_PER_SAMPLE; loop++)
  {
    // SCD40 datasheet suggests 5 second delay between SCD40 reads
    delay(5000);
    uint16_t error = envSensor.readMeasurement(sensorData.ambientCO2, sensorData.ambientTemperatureF, sensorData.ambientHumidity);
    // handle SCD40 errors
    if (error) {
      errorToString(error, errorMessage, 256);
      debugMessage(String(errorMessage) + " error during SCD4X read",1);
      return false;
    }
    if (sensorData.ambientCO2<400 || sensorData.ambientCO2>6000)
    {
      debugMessage("SCD40 CO2 reading out of range",1);
      return false;
    }
    //convert C to F for temp
    sensorData.ambientTemperatureF = (sensorData.ambientTemperatureF * 1.8) + 32;
    debugMessage(String("SCD40 read ") + loop + " of " + READS_PER_SAMPLE + " : " + sensorData.ambientTemperatureF + "F, " + sensorData.ambientHumidity + "%, " + sensorData.ambientCO2 + " ppm",1);
  }
  return true;
}

void powerI2CEnable()
// enables I2C across multiple Adafruit ESP32 variants
{
  debugMessage("powerEnable started",1);

  // enable I2C on devices with two ports
  #if defined(ARDUINO_ADAFRUIT_QTPY_ESP32S2) || defined(ARDUINO_ADAFRUIT_QTPY_ESP32S3_NOPSRAM) || defined(ARDUINO_ADAFRUIT_QTPY_ESP32S3) || defined(ARDUINO_ADAFRUIT_QTPY_ESP32_PICO)
    // ESP32 is kinda odd in that secondary ports must be manually assigned their pins with setPins()!
    Wire1.setPins(SDA1, SCL1);
    debugMessage("power on: ESP32 variant with two I2C ports",2);
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
    debugMessage("power on: Feather ESP32S2 I2C",1);
  #endif

  #if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2_TFT)
    pinMode(TFT_I2C_POWER, OUTPUT);
    digitalWrite(TFT_I2C_POWER, HIGH);
  #endif

  #if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32_V2)
    // Turn on the I2C power
    pinMode(NEOPIXEL_I2C_POWER, OUTPUT);
    digitalWrite(NEOPIXEL_I2C_POWER, HIGH);
    debugMessage("power on: Feather ESP32V2 I2C",1);
  #endif
}

void powerDisable(int deepSleepTime)
// Powers down hardware in preparation for board deep sleep
{
  debugMessage("powerDisable started",1);
  
  // power down epd
  display.powerOff();
  debugMessage("power off: epd",1);

  networkDisconnect();

  // power down SCD40

  // stops potentially started measurement then powers down SCD40
  uint16_t error = envSensor.stopPeriodicMeasurement();
  if (error) {
    char errorMessage[256];
    errorToString(error, errorMessage, 256);
    debugMessage(String(errorMessage) + " executing SCD40 stopPeriodicMeasurement()",1);
  }
  envSensor.powerDown();
  debugMessage("power off: SCD40",1);

  #if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32_V2)
    // Turn off the I2C power
    pinMode(NEOPIXEL_I2C_POWER, OUTPUT);
    digitalWrite(NEOPIXEL_I2C_POWER, LOW);

    // if you need to turn the neopixel off
    // pinMode(NEOPIXEL_POWER, OUTPUT);
    // digitalWrite(NEOPIXEL_POWER, LOW);
    debugMessage("power off: ESP32V2 I2C",1);
  #endif

  #if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2)
    // Rev B board is LOW to enable
    // Rev C board is HIGH to enable
    digitalWrite(PIN_I2C_POWER, LOW);

    // if you need to turn the neopixel off
    // pinMode(NEOPIXEL_POWER, OUTPUT);
    // digitalWrite(NEOPIXEL_POWER, LOW);
    debugMessage("power off: ESP32S2 I2C",1);
  #endif

  esp_sleep_enable_timer_wakeup(deepSleepTime*1000000); // ESP microsecond modifier
  debugMessage(String("powerDisable complete: ESP32 deep sleep for ") + (deepSleepTime) + " seconds",1);
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
  debugMessage(String("Retrieved CO2 sample pointer is: ") + storedCounter,2);

  String nvStoreBaseName;
  for (int i=0; i<co2MaxStoredSamples; i++)
  {
    nvStoreBaseName = "co2Sample" + String(i);
    // get previously stored values. If they don't exist, create them as 400 (CO2 floor)
    co2Samples[i] = nvStorage.getLong(nvStoreBaseName.c_str(),400);
    debugMessage(String(nvStoreBaseName) + " retrieved from nv storage: " + co2Samples[i],2);
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
  debugMessage(String(nvStoreBaseName) + " stored in nv storage as " + sensorData.ambientCO2,1);
}

bool networkConnect()
{
  // Run only if using network data endpoints
  #if defined(MQTT) || defined(INFLUX) || defined(HASSIO_MQTT) || defined(DWEET)
    // set hostname has to come before WiFi.begin
    WiFi.hostname(DEVICE_ID);

    WiFi.begin(WIFI_SSID, WIFI_PASS);

    for (int tries = 1; tries <= CONNECT_ATTEMPT_LIMIT; tries++)
    // Attempts WiFi connection, and if unsuccessful, re-attempts after CONNECT_ATTEMPT_INTERVAL second delay for CONNECT_ATTEMPT_LIMIT times
    {
      if (WiFi.status() == WL_CONNECTED)
      {
        hardwareData.rssi = abs(WiFi.RSSI());
        debugMessage("power on: WiFi",1);
        debugMessage(String("WiFi IP address from ") + WIFI_SSID + ": " + WiFi.localIP().toString(),1);
        debugMessage(String("WiFi RSSI: ") + hardwareData.rssi + " dBm",1);
        return true;
      }
      debugMessage(String("Connection attempt ") + tries + " of " + CONNECT_ATTEMPT_LIMIT + " to " + WIFI_SSID + " failed",1);
      // use of delay() OK as this is initialization code
      delay(CONNECT_ATTEMPT_INTERVAL * 1000); // convered into milliseconds
    }
  #endif
  return false;
}

void networkDisconnect()
{
  #if defined(MQTT) || defined(INFLUX) || defined(HASSIO_MQTT)
  {
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
    debugMessage("power off: WiFi",1);
  }
  #endif
}

bool networkGetTime(String timezone)
{
  // https://randomnerdtutorials.com/esp32-ntp-timezones-daylight-saving/

  struct tm timeinfo;

  // connect to NTP server with 0 TZ offset
  configTime(0, 0, ntpServer);
  if(!getLocalTime(&timeinfo))
  {
    debugMessage("Failed to obtain time from NTP Server",1);
    return false;
  }
  // set local timezone
  setTimeZone(timezone);
  return true;
}

void setTimeZone(String timezone)
{
  debugMessage(String("setting Timezone to ") + timezone.c_str(),2);
  setenv("TZ",timezone.c_str(),1);
  tzset();
  debugMessage(String("Local time: ") + dateTimeString("short"),1);
}

// Converts time into human readable strings
String dateTimeString(String formatType)
{
  // https://cplusplus.com/reference/ctime/tm/

  String dateTime;
  struct tm timeInfo;

  if (getLocalTime(&timeInfo)) 
  {
    if (formatType == "short")
    {
      // short human readable format
      dateTime = weekDays[timeInfo.tm_wday];
      dateTime += " at ";
      if (timeInfo.tm_hour < 10) dateTime += "0";
      dateTime += timeInfo.tm_hour;
      dateTime += ":";
      if (timeInfo.tm_min < 10) dateTime += "0";
      dateTime += timeInfo.tm_min;
    }
    else if (formatType == "long")
    {
      // long human readable
      dateTime = weekDays[timeInfo.tm_wday];
      dateTime += ", ";
      if (timeInfo.tm_mon<10) dateTime += "0";
      dateTime += timeInfo.tm_mon;
      dateTime += "-";
      if (timeInfo.tm_wday<10) dateTime += "0";
      dateTime += timeInfo.tm_wday;
      dateTime += " at ";
      if (timeInfo.tm_hour<10) dateTime += "0";
      dateTime += timeInfo.tm_hour;
      dateTime += ":";
      if (timeInfo.tm_min<10) dateTime += "0";
      dateTime += timeInfo.tm_min;
    }
  }
  else dateTime = "Can't reach time service";
  return dateTime;
}