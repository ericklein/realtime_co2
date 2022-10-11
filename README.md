# Realtime CO2 monitor

### Purpose
Realtime CO2 samples and logs temperature, humidity, and CO2 levels

### Configuring targets
- Set parameters in secrets.h (see config.h for list of required parameters)
- Set parameters in config.h

### External Software Dependencies
    - Sensirion I2C SCD4x library or Adafruit Unified Sensor + appropriate hardware (AHT2x or BME280) library
    - Adafruit LC709203F library (#define BATTERY)
    - Adafruit EPD library (MagTag)
- include all dependencies to these libraries

### known, working BOM
- MCU
    - ESP32 devices

- Ethernet
    - Particle Ethernet Featherwing: https://www.adafruit.com/product/4003
    - Silicognition PoE Featherwing: https://www.crowdsupply.com/silicognition/poe-featherwing
- WiFi
    - esp32 boards
- environment sensor
    - SCD40 True CO2, Temperature and Humidity Sensor: https://www.adafruit.com/product/5187
- battery monitor
    - LC709203F battery voltage monitor: https://www.adafruit.com/product/4712
- screen
    - Adafruit supported epds: https://www.adafruit.com/product/4800
- battery
    - Adafruit battery: https://www.adafruit.com/product/2011

### Information Sources

- NTP
    - https://github.com/PaulStoffregen/Time/tree/master/examples/TimeNTP
- Sensors 
    - https://cdn-learn.adafruit.com/assets/assets/000/104/015/original/Sensirion_CO2_Sensors_SCD4x_Datasheet.pdf?1629489682
    - https://github.com/Sensirion/arduino-i2c-scd4x
    - https://github.com/sparkfun/SparkFun_SCD4x_Arduino_Library
    - https://emariete.com/en/sensor-co2-sensirion-scd40-scd41-2/
- Ethernet
    - https://docs.particle.io/datasheets/accessories/gen3-accessories/
    - https://www.adafruit.com/product/4003#:~:text=Description%2D-,Description,along%20with%20a%20Feather%20accessory
    - https://learn.adafruit.com/adafruit-wiz5500-wiznet-ethernet-featherwing/usage
    - https://www.arduino.cc/en/reference/ethernet
    - https://store.arduino.cc/usa/arduino-ethernet-rev3-without-poe
- Display
    - https://cdn-learn.adafruit.com/downloads/pdf/adafruit-gfx-graphics-library.pdf
- MQTT services
    - https://hackaday.com/2017/10/31/review-iot-data-logging-services-with-mqtt/

### Issues
- See GitHub Issues for project

### Feature Requests
- See GitHub Issues for project

### Questions