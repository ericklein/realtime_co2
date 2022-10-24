#include "Arduino.h"

// hardware and internet configuration parameters
#include "config.h"
// private credentials for network, MQTT, weather provider
#include "secrets.h"

// Only compile if InfluxDB enabled
#ifdef INFLUX

// Shared helper function
extern void debugMessage(String messageText);

// Status variables shared across various functions
extern bool batteryVoltageAvailable;
extern bool internetAvailable;

#include <InfluxDbClient.h>

// InfluxDB setup.  See config.h and secrets.h for site-specific settings.  Both InfluxDB v1.X
// and v2.X are supported here depending on configuration settings in secrets.h.  Code here
// reflects a number of presumptions about the data schema and InfluxDB configuration:
//

#ifdef INFLUX_V1
// InfluxDB client instance for InfluxDB 1
InfluxDBClient dbclient(INFLUXDB_URL, INFLUXDB_DB_NAME);
#endif

#ifdef INFLUX_V2
// InfluxDB client instance for InfluxDB 2
InfluxDBClient dbclient(INFLUXDB_URL, INFLUXDB_ORG, INFLUXDB_BUCKET, INFLUXDB_TOKEN);
#endif

// InfluxDB Data point, binds to InfluxDB 'measurement' to use for data. See config.h for value used
Point dbenvdata(INFLUX_ENV_MEASUREMENT);
Point dbdevdata(INFLUX_DEV_MEASUREMENT);

// Post data to Influx DB using the connection established during setup
// Operates over the network, so may take a while to execute.
boolean post_influx(uint16_t co2, float tempF, float humidity, float battery_v, int rssi)
{
  Serial.println("Saving data to Influx");
  #ifdef INFLUX_V1
    // Set InfluxDB v1.X authentication params using values defined in secrets.h.  Not needed as such
    // for InfluxDB v2.X (which uses a token-based scheme via the constructor).
    dbclient.setConnectionParamsV1(INFLUXDB_URL, INFLUXDB_DB_NAME, INFLUXDB_USER, INFLUXDB_PASSWORD);
  #endif
  
  // Add constant Influx data point tags - only do once, will be added to all individual data points
  // Modify if required to reflect your InfluxDB data model (and set values in config.h)
  // First for environmental data
  dbenvdata.addTag("device", DEVICE_TYPE);
  dbenvdata.addTag("location", DEVICE_LOCATION);
  dbenvdata.addTag("site", DEVICE_SITE);
  // And again for device data
  dbdevdata.addTag("device", DEVICE_TYPE);
  dbdevdata.addTag("location", DEVICE_LOCATION);
  dbdevdata.addTag("site", DEVICE_SITE);

  // If confirmed connection to InfluxDB server, store our data values (with retries)
  boolean dbsuccess = false;
  uint8_t dbtries;
  for (dbtries = 1; dbtries <= INFLUX_ATTEMPT_LIMIT; dbtries++) {
    debugMessage(String("InfluxDB connection attempt ") + dbtries + " of "+ INFLUX_ATTEMPT_LIMIT + " in " + (dbtries*10) + " seconds");
    if (dbclient.validateConnection()) {
      debugMessage("Connected to InfluxDB: " + dbclient.getServerUrl());
      dbsuccess = true;
      break;
    }
    delay(dbtries * 10000); // Waiting longer each time we check for status
  }
  if(dbsuccess == false) {
    debugMessage("InfluxDB connection failed: " + dbclient.getLastErrorMessage());
    return(false);  // Failed...
  }
  else 
  {
    // Connected, so store sensor values into timeseries data point
    dbenvdata.clearFields();
    // Report sensor readings
    dbenvdata.addField("temperature", tempF);
    dbenvdata.addField("humidity", humidity);
    dbenvdata.addField("co2", co2);
    debugMessage("Writing: " + dbclient.pointToLineProtocol(dbenvdata));
    // Write point via connection to InfluxDB host
    if (!dbclient.writePoint(dbenvdata)) {
      debugMessage("InfluxDB write failed: " + dbclient.getLastErrorMessage());
      dbsuccess = false;  // So close...
    }

    // Now store device information 
    dbdevdata.clearFields();
    // Report device readings
    if (batteryVoltageAvailable)
      dbdevdata.addField("battery_volts", battery_v);
    if (internetAvailable)
      dbdevdata.addField("rssi", rssi);
    if ((batteryVoltageAvailable) || (internetAvailable))
    {
      // Write point via connection to InfluxDB host
      debugMessage("Writing: " + dbclient.pointToLineProtocol(dbdevdata));
      if (!dbclient.writePoint(dbdevdata))
      {
        debugMessage("InfluxDB write failed: " + dbclient.getLastErrorMessage());
        dbsuccess = false;  // So close...
      }
    }
    dbclient.flushBuffer();  // Clear pending writes (before going to sleep)
  }
  return(dbsuccess);
}
#endif