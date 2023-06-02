#include "Arduino.h"

// hardware and internet configuration parameters
#include "config.h"
// Overall data and metadata naming scheme
#include "data.h"
// private credentials for network, MQTT, weather provider
#include "secrets.h"


#ifdef DWEET
#include <HTTPClient.h> 

// Shared helper function we call here too...
extern void debugMessage(String messageText);
extern bool batteryVoltageAvailable;
extern bool internetAvailable;

// Post a dweet to report the various sensor readings.  This routine blocks while
// talking to the network, so may take a while to execute.
void post_dweet(uint16_t co2, float tempF, float humidity, float battv, int rssi)
{
  String dweeturl = "http://" + String(DWEET_HOST) + "/dweet/for/" + String(DWEET_DEVICE);

  // If no Internet, return
  if(!internetAvailable) return(void ());

  WiFiClient dweet_client;

  // Transmit Dweet as HTTP post with a data payload as JSON
  String device_info = "{\"rssi\":\""   + String(rssi)               + "\"," +
                        "\"ipaddr\":\"" + WiFi.localIP().toString()  + "\",";
  
  String battery_info;
  if(batteryVoltageAvailable) {
    battery_info = "\"battery_voltage\":\"" + String(battv)   + "\",";
  }
  else {
    battery_info = "";
  }

  String sensor_info;
  sensor_info = "\"co2\":\""         + String(co2)             + "\"," +
                "\"temperature\":\"" + String(tempF, 2)        + "\"," +
                "\"humidity\":\""    + String(humidity, 2)     + "\"}";

  String postdata = device_info + battery_info + sensor_info;

  // Note that the dweet device 'name' gets set here, is needed to fetch values
  dweet_client.println("POST /dweet/for/" + String(DWEET_DEVICE) + " HTTP/1.1");
  dweet_client.println("Host: dweet.io");
  dweet_client.println("User-Agent: ESP32/ESP8266 (orangemoose)/1.0");
  dweet_client.println("Cache-Control: no-cache");
  dweet_client.println("Content-Type: application/json");
  dweet_client.print("Content-Length: ");
  dweet_client.println(postdata.length());
  dweet_client.println();
  dweet_client.println(postdata);
  debugMessage("Dweet POST:");
  debugMessage(postdata);

  delay(1500);  

  // Read all the lines of the reply from server (if any) and print them to Serial Monitor
  #ifdef DEBUG
    debugMessageln("Dweet server response:");
    while(dweet_client.available()){
      String line = dweet_client.readStringUntil('\r');
      debugMessage(line);
    }
    debugMessageln("-----");
  #endif
  
  // Close client connection to dweet server
  dweet_client.stop();
}
#endif