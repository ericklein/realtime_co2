/*
  Project:      realtime_co2
  Description:  write sensor data to DWEET
*/

#include "Arduino.h"

// hardware and internet configuration parameters
#include "config.h"
// private credentials for network, MQTT, weather provider
#include "secrets.h"

#ifdef DWEET
  #include <HTTPClient.h> 

  // Shared helper function
  extern void debugMessage(String messageText, int messageLevel);

  // Post a dweet to report the various sensor readings
  void post_dweet(uint16_t co2, float temperatureF, float humidity, float battv, int rssi)
  {
    WiFiClient dweet_client;

    if(WiFi.status() != WL_CONNECTED) {
      debugMessage("Lost network connection to " + String(WIFI_SSID) + "!",1);
      return;
    }

    // Use our WiFiClient to connect to dweet
    if (!dweet_client.connect(DWEET_HOST, 80)) {
      debugMessage("Dweet connection failed!",1);
      return;
    }

    // Transmit Dweet as HTTP post with a data payload as JSON
    String device_info = "{\"rssi\":\""   + String(rssi)               + "\"," +
                          "\"ipaddr\":\"" + WiFi.localIP().toString()  + "\",";
    
    if(batteryVoltage > 0) {
      battery_info = "\"battery_voltage\":\"" + String(batteryVoltage)   + "\",";
    }
    else {
      battery_info = "";
    }

    String sensor_info;
    sensor_info = "\"co2\":\""         + String(co2)             + "\"," +
                  "\"temperature\":\"" + String(temperatureF, 2)        + "\"," +
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
    debugMessage("Dweet POST:",1);
    debugMessage(postdata,1);

    delay(1500);  

    // Read all the lines of the reply from server (if any) and print them to Serial Monitor
    #ifdef DEBUG
      debugMessage("Dweet server response:",2);
      while(dweet_client.available()){
        String line = dweet_client.readStringUntil('\r');
        debugMessage(line,2);
      }
      debugMessage("-----",2);
    #endif
    
    // Close client connection to dweet server
    dweet_client.stop();
  }
#endif