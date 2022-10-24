#include "Arduino.h"

// hardware and internet configuration parameters
#include "config.h"
// private credentials for network, MQTT, weather provider
#include "secrets.h"

#include "aq_network.h"
extern AQ_Network aq_network;

// Shared helper function
extern void debugMessage(String messageText);

// Status variables shared across various functions
extern bool batteryVoltageAvailable;
extern bool internetAvailable;

#ifdef MQTTLOG
  // MQTT setup
  #include <Adafruit_MQTT.h>
  #include <Adafruit_MQTT_Client.h>
  extern Adafruit_MQTT_Client aq_mqtt;

  void mqttConnect()
  // Connects and reconnects to MQTT broker, call as needed to maintain connection
  {
    int8_t mqttErr;
    int8_t tries = 1;
  
    // exit if already connected
    if (aq_mqtt.connected())
    {
      return;
    }
  
    while ((mqttErr = aq_mqtt.connect() != 0) && (tries<=MQTT_ATTEMPT_LIMIT))
    {
      // generic MQTT error
      // debugMessage(mqtt.connectErrorString(mqttErr));
  
      // Adafruit IO connect errors
      switch (mqttErr)
      {
        case 1: debugMessage("Adafruit MQTT: Wrong protocol"); break;
        case 2: debugMessage("Adafruit MQTT: ID rejected"); break;
        case 3: debugMessage("Adafruit MQTT: Server unavailable"); break;
        case 4: debugMessage("Adafruit MQTT: Incorrect user or password"); break;
        case 5: debugMessage("Adafruit MQTT: Not authorized"); break;
        case 6: debugMessage("Adafruit MQTT: Failed to subscribe"); break;
        default: debugMessage("Adafruit MQTT: GENERIC - Connection failed"); break;
      }
      debugMessage(String(MQTT_BROKER) + " connect attempt " + tries + " of " + MQTT_ATTEMPT_LIMIT + " happens in " + (tries*10) + " seconds");
      aq_mqtt.disconnect();
      delay(tries*10000);
      tries++;
  
      if (tries == MQTT_ATTEMPT_LIMIT)
      {
        debugMessage(String("Connection failed to MQTT broker: ") + MQTT_BROKER);
      }
    }
    if (tries < MQTT_ATTEMPT_LIMIT)
    {
      debugMessage(String("Connected to MQTT broker ") + MQTT_BROKER);
    }
  }

  int mqttDeviceBatteryUpdate(float cellVoltage)
  {
    int result = 0;
    if (batteryVoltageAvailable)
    {
      Adafruit_MQTT_Publish batteryVoltagePub = Adafruit_MQTT_Publish(&aq_mqtt, MQTT_PUB_TOPIC5, MQTT_QOS_1); // if problematic, remove QOS parameter

      mqttConnect();

      // publish battery voltage
      if (batteryVoltagePub.publish(cellVoltage))
      {
        debugMessage("MQTT publish: Battery Voltage succeeded");
        result = 1;
      }
      else
      {
        debugMessage("MQTT publish: Battery Percent failed");
      }
      aq_mqtt.disconnect(); 
    }
    return(result);
  }

  int mqttDeviceWiFiUpdate(int rssi)
  {
    int result = 0;
    if (internetAvailable)
    {
      Adafruit_MQTT_Publish rssiLevelPub = Adafruit_MQTT_Publish(&aq_mqtt, MQTT_PUB_TOPIC6, MQTT_QOS_1); // if problematic, remove QOS parameter

      mqttConnect();

      if (rssiLevelPub.publish(rssi))
      {
        debugMessage("MQTT publish: WiFi RSSI succeeded");
        result = 1;
      }
      else
      {
        debugMessage("MQTT publish: WiFi RSSI failed");
      }
      aq_mqtt.disconnect();
    }
    return(result);
  }
  
  int mqttSensorUpdate(uint16_t co2, float tempF, float humidity)
  // Publishes sensor data to MQTT broker
  {
    Adafruit_MQTT_Publish tempPub = Adafruit_MQTT_Publish(&aq_mqtt, MQTT_PUB_TOPIC1, MQTT_QOS_1);
    Adafruit_MQTT_Publish humidityPub = Adafruit_MQTT_Publish(&aq_mqtt, MQTT_PUB_TOPIC2, MQTT_QOS_1);
    Adafruit_MQTT_Publish co2Pub = Adafruit_MQTT_Publish(&aq_mqtt, MQTT_PUB_TOPIC3, MQTT_QOS_1);
    int result = 1;
    
    mqttConnect();
    // Attempt to publish sensor data
    if(tempPub.publish(tempF))
    {
      debugMessage("MQTT publish: Temperature succeeded");
    }
    else {
      debugMessage("MQTT publish: Temperature failed");
      result = 0;
    }
    
    if(humidityPub.publish(humidity))
    {
      debugMessage("MQTT publish: Humidity succeeded");
    }
    else {
      debugMessage("MQTT publish: Humidity failed");
      result = 0;
    }
    
    if(co2Pub.publish(co2))
    {
      debugMessage("MQTT publish: CO2 succeeded");
    }
    else
    {
      debugMessage("MQTT publish: CO2 failed");
      result = 0;
    }
    aq_mqtt.disconnect();
    return(result);
  }
#endif