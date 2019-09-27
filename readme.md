# Expanded Bluetooth 5 Multi Link Demo

An edited version of [Nordic's firmware for the NRF52 DK](https://github.com/NordicPlayground/nrf52-ble-multi-link-multi-role) to 
include support for reading more of the Thingy node's sensors. Meant to be used in conjunction with a corresponding 
[EPICS controller](https://github.com/epicsNSLS2-sensors/ThingyAggregatorIOC).

## Features
  - Relay sensor readings from Thingy nodes
    - Thingy Envionment service: temperature, pressure, humidity, air quality
    - Thingy Motion service: acceleration, gyroscope, quaternions
    - Button
    - Battery
  - Relay RSSI between node and aggregator
  - Support configuration characteristics
    - Thingy Environment service configuration
    - Thingy Motion service configuration
    - Thingy connection parameters characteristics
  - Commands:
    - Read characteristic
    - Write characteristic
    - Toggle sensors on/off
    - Toggle LED
