#include "Arduino.h"

// hardware and internet configuration parameters
#include "config.h"
// private credentials for network, MQTT, weather provider
#include "secrets.h"

// Shared helper function
extern void debugMessage(String messageText);

#ifdef HASSIO_MQTT
  extern void hassio_mqtt_setup();
  extern void hassio_mqtt_publish(uint16_t co2, float tempF, float humidity);
#endif

#ifdef MQTT
  // MQTT setup
  #include <Adafruit_MQTT.h>
  #include <Adafruit_MQTT_Client.h>
  extern Adafruit_MQTT_Client aq_mqtt;

  void mqttConnect()
  // Connects and reconnects to MQTT broker, call as needed to maintain connection
  {
    // exit if already connected
    if (aq_mqtt.connected())
    {
      debugMessage(String("Already connected to MQTT broker ") + MQTT_BROKER);
      return;
    }

    int8_t mqttErr;

    for(int tries = 1; tries <= CONNECT_ATTEMPT_LIMIT; tries++)
    {
      if ((mqttErr = aq_mqtt.connect()) == 0)
      {
        debugMessage(String("Connected to MQTT broker ") + MQTT_BROKER);
        return;
      }
      // Adafruit IO connect errors
      // switch (mqttErr)
      // {
      //   case 1: debugMessage("Adafruit MQTT: Wrong protocol"); break;
      //   case 2: debugMessage("Adafruit MQTT: ID rejected"); break;
      //   case 3: debugMessage("Adafruit MQTT: Server unavailable"); break;
      //   case 4: debugMessage("Adafruit MQTT: Incorrect user or password"); break;
      //   case 5: debugMessage("Adafruit MQTT: Not authorized"); break;
      //   case 6: debugMessage("Adafruit MQTT: Failed to subscribe"); break;
      //   default: debugMessage("Adafruit MQTT: GENERIC - Connection failed"); break;
      // }
      aq_mqtt.disconnect();
      debugMessage(String("MQTT connection attempt ") + tries + " of " + CONNECT_ATTEMPT_LIMIT + " failed with error msg: " + aq_mqtt.connectErrorString(mqttErr));
      delay(CONNECT_ATTEMPT_INTERVAL*1000);
    }
  } 

  int mqttDeviceBatteryUpdate(float batteryVoltage)
  {
    bool result = false;
    if (batteryVoltage > 0)
    {
      // add ,MQTT_QOS_1); if problematic, remove QOS parameter
      Adafruit_MQTT_Publish batteryVoltagePub = Adafruit_MQTT_Publish(&aq_mqtt, MQTT_PUB_BATTVOLT);
      mqttConnect();

      // publish battery voltage
      if (batteryVoltagePub.publish(batteryVoltage))
      {
        debugMessage("MQTT publish: Battery Voltage succeeded");
        result = true;
      }
      else
      {
        debugMessage("MQTT publish: Battery Voltage failed");
      }
    }
    return(result);
  }

  int mqttDeviceWiFiUpdate(int rssi)
  {
    int result = 0;
    if (rssi!=0)
    {
      // add ,MQTT_QOS_1); if problematic, remove QOS parameter
      Adafruit_MQTT_Publish rssiLevelPub = Adafruit_MQTT_Publish(&aq_mqtt, MQTT_PUB_RSSI);
      
      mqttConnect();

      if (rssiLevelPub.publish(rssi))
      {
        debugMessage("MQTT publish: WiFi RSSI succeeded");
        result = true;
      }
      else
      {
        debugMessage("MQTT publish: WiFi RSSI failed");
      }
    }
    return(result);
  }
  
  bool mqttSensorTempFUpdate(float tempF)
  // Publishes temperature data to MQTT broker
  {
    bool result = false;
    // add ,MQTT_QOS_1); if problematic, remove QOS parameter
    Adafruit_MQTT_Publish tempPub = Adafruit_MQTT_Publish(&aq_mqtt, MQTT_PUB_TEMPF);
    
    mqttConnect();

    // Attempt to publish sensor data
    if(tempPub.publish(tempF))
    {
      debugMessage("MQTT publish: Temperature succeeded");
      result = true;
    }
    else {
      debugMessage("MQTT publish: Temperature failed");
    }
    return(result);
  }

  bool mqttSensorHumidityUpdate(float humidity)
  // Publishes humidity data to MQTT broker
  {
    bool result = false;
    // add ,MQTT_QOS_1); if problematic, remove QOS parameter
    Adafruit_MQTT_Publish humidityPub = Adafruit_MQTT_Publish(&aq_mqtt, MQTT_PUB_HUMIDITY);
    
    mqttConnect();
    
    // Attempt to publish sensor data
    if(humidityPub.publish(humidity))
    {
      debugMessage("MQTT publish: Humidity succeeded");
      result = true;
    }
    else {
      debugMessage("MQTT publish: Humidity failed");
    }
    return(result);
  }

  bool mqttSensorCO2Update(uint16_t co2)
  // Publishes CO2 data to MQTT broker
  {
    bool result = false;
    // add ,MQTT_QOS_1); if problematic, remove QOS parameter
    Adafruit_MQTT_Publish co2Pub = Adafruit_MQTT_Publish(&aq_mqtt, MQTT_PUB_CO2);   
    
    mqttConnect();

    // Attempt to publish sensor data
    if (co2 != 10000)
    {
      if(co2Pub.publish(co2))
      {
        debugMessage("MQTT publish: CO2 succeeded");
        result = true;
      }
      else
      {
        debugMessage("MQTT publish: CO2 failed");
      }
    }
    return(result);
  }
#endif