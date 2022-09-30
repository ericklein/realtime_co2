#include "Arduino.h"
#include "aq_network.h"

// hardware and internet configuration parameters
#include "config.h"
// private credentials for network, MQTT, weather provider
#include "secrets.h"

// Shared helper function we call here too...
extern void debugMessage(String messageText);
extern bool batteryAvailable;

// Includes and defines specific to WiFi network connectivity
#ifdef WIFI
  #if defined(ARDUINO_SAMD_MKRWIFI1010) || defined(ARDUINO_SAMD_NANO_33_IOT) || defined(ARDUINO_AVR_UNO_WIFI_REV2)
    #include <WiFiNINA.h>
  #elif defined(ARDUINO_SAMD_MKR1000)
    #include <WiFi101.h>
  #elif defined(ARDUINO_ESP8266_ESP12)
    #include <ESP8266WiFi.h>
  #else
    #include <WiFi.h>
#endif

WiFiClient client;
//WiFiClientSecure client; // for SSL

// NTP support
#include <WiFiUdp.h>
WiFiUDP ntpUDP;
#endif

// Includes and defines specific to Ethernet (wired) network connectivity
#ifdef RJ45
  // Set MAC address. If unknown, be careful for duplicate addresses across projects.
  byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
  #include <SPI.h>
  #include <Ethernet.h>
  EthernetClient client;

  // NTP support
  #include <EthernetUdp.h>
  EthernetUDP ntpUDP;
#endif

// Network services independent of physiccal connection
#if defined(WIFI) || defined(RJ45)
  // NTP setup
  #include <NTPClient.h>
  NTPClient timeClient(ntpUDP);

  // Generalized access to HTTP services atop WiFi or Ethernet connections
  #include <HTTPClient.h>
#endif

// MQTT interface depends on the underlying network client object, which is defined and
// managed here (so needs to be defined here).
#ifdef MQTTLOG
  // MQTT setup
  #include <Adafruit_MQTT.h>
  #include <Adafruit_MQTT_Client.h>
  Adafruit_MQTT_Client aq_mqtt(&client, MQTT_BROKER, MQTT_PORT, CLIENT_ID, MQTT_USER, MQTT_PASS);
#endif

//****************************************************************************************************
// AQ_Network Class and Member Functions
//

// Converts system time into human readable strings. Depends on NTP service access
String AQ_Network::dateTimeString() {
  String dateTime;

#if defined(WIFI) || defined(RJ45)
  String weekDays[7] = { "Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday" };

  if (timeClient.update()) {
    // NTPClient doesn't include date information, get it from time structure
    time_t epochTime = timeClient.getEpochTime();
    struct tm* ptm = gmtime((time_t*)&epochTime);
    int day = ptm->tm_mday;
    int month = ptm->tm_mon + 1;
    int year = ptm->tm_year + 1900;

    dateTime = weekDays[timeClient.getDay()];
    dateTime += " at ";
    if (timeClient.getHours() < 10) dateTime += "0";
    dateTime += timeClient.getHours();
    dateTime += ":";
    if (timeClient.getMinutes() < 10) dateTime += "0";
    dateTime += timeClient.getMinutes();

    // long human readable
    // dateTime = weekDays[timeClient.getDay()];
    // dateTime += ", ";

    // if (month<10) dateTime += "0";
    // dateTime += month;
    // dateTime += "-";
    // if (day<10) dateTime += "0";
    // dateTime += day;
    // dateTime += " at ";
    // if (timeClient.getHours()<10) dateTime += "0";
    // dateTime += timeClient.getHours();
    // dateTime += ":";
    // if (timeClient.getMinutes()<10) dateTime += "0";
    // dateTime += timeClient.getMinutes();

    // zulu format
    // dateTime = year + "-";
    // if (month()<10) dateTime += "0";
    // dateTime += month;
    // dateTime += "-";
    // if (day()<10) dateTime += "0";
    // dateTime += day;
    // dateTime += "T";
    // if (timeClient.getHours()<10) dateTime += "0";
    // dateTime += timeClient.getHours();
    // dateTime += ":";
    // if (timeClient.getMinutes()<10) dateTime += "0";
    // dateTime += timeClient.getMinutes();
    // dateTime += ":";
    // if (timeClient.getSeconds()<10) dateTime += "0";
    // dateTime += timeClient.getSeconds();
    // switch (timeZone)
    // {
    //   case 0:
    //     dateTime += "Z";
    //     break;
    //   case -7:
    //     dateTime += "PDT";
    //     break;
    //   case -8:
    //     dateTime += "PST";
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

// Initialize network and connect.  If connection succeeds initialize NTP connection so
// device can report accurate local time.  Returns boolean indicating whether network is
// connected and available.  Depends on configuration #defines in config.h to determine
// what network hardware is attached, and key network settings there as well (e.g. SSID).
bool AQ_Network::networkBegin() {
  bool networkAvailable = false;

#ifdef WIFI
  uint8_t tries;

  // set hostname has to come before WiFi.begin
  WiFi.hostname(CLIENT_ID);
  // WiFi.setHostname(CLIENT_ID); //for WiFiNINA

  // Connect to WiFi.  Prepared to wait a reasonable interval for the connection to
  // succeed, but not forever.  Will check status and, if not connected, delay an
  // increasing amount of time up to a maximum of WIFI_ATTEMPT_LIMIT delay intervals.
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  for (tries = 1; tries <= WIFI_ATTEMPT_LIMIT; tries++) {
    debugMessage(String("Connection attempt ") + tries + " of " + WIFI_ATTEMPT_LIMIT + " to " + WIFI_SSID + " in " + (tries * 10) + " seconds");
    if (WiFi.status() == WL_CONNECTED) {
      // Successful connection!
      networkAvailable = true;
      break;
    }
    // use of delay OK as this is initialization code
    delay(tries * 10000);  // Waiting longer each time we check for status
  }
  if (networkAvailable) {
    debugMessage("WiFi IP address is: " + WiFi.localIP().toString());
    debugMessage("RSSI is: " + String(getWiFiRSSI()) + " dBm");
  } else {
    // Couldn't connect, alas
    debugMessage(String("Can not connect to WFii after ") + WIFI_ATTEMPT_LIMIT + " attempts");
  }
#endif

#ifdef RJ45
  // Configure Ethernet CS pin, not needed if using default D10
  //Ethernet.init(10);  // Most Arduino shields
  //Ethernet.init(5);   // MKR ETH shield
  //Ethernet.init(0);   // Teensy 2.0
  //Ethernet.init(20);  // Teensy++ 2.0
  //Ethernet.init(15);  // ESP8266 with Adafruit Featherwing Ethernet
  //Ethernet.init(33);  // ESP32 with Adafruit Featherwing Ethernet

  // Initialize Ethernet and UDP
  if (Ethernet.begin(mac) == 0) {
    // identified errors
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
      debugMessage("Ethernet hardware not found");
    } else if (Ethernet.linkStatus() == LinkOFF) {
      debugMessage("Ethernet cable not connected");
    } else {
      // generic error
      debugMessage("Failed to configure Ethernet");
    }
  } else {
    debugMessage(String("Ethernet IP address is: ") + Ethernet.localIP().toString());
    networkAvailable = true;
  }
#endif

#if defined(WIFI) || defined(RJ45)
  if (networkAvailable) {
    // Get time from NTP
    timeClient.begin();
    // Set offset time in seconds to adjust for your timezone
    timeClient.setTimeOffset(timeZone * 60 * 60);
    debugMessage("NTP time: " + dateTimeString());
  }
#endif

  return (networkAvailable);
}


String AQ_Network::httpGETRequest(const char* serverName) {
  String payload = "{}";

#if defined(WIFI) || defined(RJ45)
  HTTPClient http;

  // servername is domain name w/URL path or IP address w/URL path
  http.begin(client, serverName);

  // Send HTTP GET request
  int httpResponseCode = http.GET();

  if (httpResponseCode == HTTP_CODE_OK) {
    // HTTP reponse OK code
    payload = http.getString();
  } else {
    debugMessage("HTTP GET error code: " + httpResponseCode);
    payload = "HTTP GET error";
  }
  // free resources
  http.end();
#endif
  return payload;
}

void AQ_Network::networkStop() {
#if defined(WIFI) || defined(RJ45)
  client.stop();
#endif
}

int AQ_Network::httpPOSTRequest(String serverurl, String contenttype, String payload) {
  int httpCode = -1;
#if defined(WIFI) || defined(RJ45)
  HTTPClient http;

  http.begin(client, serverurl);
  http.addHeader("Content-Type", contenttype);

  httpCode = http.POST(payload);

  // httpCode will be negative on error, but HTTP status might indicate failure
  if (httpCode > 0) {
    // HTTP POST complete, print result code
    debugMessage("HTTP POST [" + serverurl + "], result code: " + String(httpCode));

    // If POST succeeded, output response as debug messages
    if (httpCode == HTTP_CODE_OK) {
      const String& payload = http.getString();
      debugMessage("received payload:\n<<");
      debugMessage(payload);
      debugMessage(">>");
    }
  } else {
    debugMessage("HTTP POST [" + serverurl + "] failed, error: " + http.errorToString(httpCode).c_str());
  }

  http.end();
  debugMessage("closing connection for dweeting");
#endif

  return (httpCode);
}

// Utility functions that may be of use

// Return local IP address as a String
String AQ_Network::getLocalIPString() {
#if defined(WIFI) || defined(RJ45)
  return (client.localIP().toString());
#else
  return ("No network");
#endif
}

// Return RSSI for WiFi network, simulate out-of-range value for non-WiFi
int AQ_Network::getWiFiRSSI() {
#ifdef WIFI
  return (WiFi.RSSI());
#else
  return (-255);  //Arbitrary out-of-range value
#endif
}

// Returns true if WIFI defined in config.h, otherwise false
bool AQ_Network::isWireless() {
#ifdef WIFI
  return (true);
#else
  return (false);
#endif
}

// Returns true if RJ45 (Ethernet) defined in config.h, otherwise false
bool AQ_Network::isWired() {
#ifdef RJ45
  return (true);
#else
  return (false);
#endif
}

// Returns connection status (via Client class), or false if no network defined in config.h
bool AQ_Network::isConnected() {
#if defined(WIFI) || defined(RJ45)
  return (client.connected());
#else
  return (false);
#endif
}