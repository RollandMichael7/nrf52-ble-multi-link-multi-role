# Expanded Bluetooth 5 Multi Link Demo

An edited version of [Nordic's firmware for the NRF52 DK](https://github.com/NordicPlayground/nrf52-ble-multi-link-multi-role) to 
include support for reading more of the Thingy node's sensors. Meant to be used in conjunction with a corresponding 
[EPICS controller](https://github.com/epicsNSLS2-sensors/ThingyAggregatorIOC).

## Features
  - Relay sensor readings from Thingy nodes
    - Thingy Envionment service: temperature, pressure, humidity, air quality
    - Thingy Motion service: acceleration, gyroscope, compass, heading, euler, quaternions
    - Button
    - Battery
  - Read/write 4 external digital pins
  - Toggle LED
  - Relay RSSI between node and aggregator
  - Read/write configuration characteristics
    - Thingy Environment service configuration
    - Thingy Motion service configuration
    - Thingy connection parameters characteristics
    
## Commands
Send commands to the aggregator by writing to UUID ```3e520002-1368-b682-4440-d7dd234c45bc```.

The aggregator will respond to commands by pushing notifications to UUID ```3e520003-1368-b682-4440-d7dd234c45bc```. For both reading and writing, the format of the messages comply to the [Thingy documentation](https://nordicsemiconductor.github.io/Nordic-Thingy52-FW/documentation/firmware_architecture.html).

The first byte of every command is the opcode.

**All multi-byte values are transmitted in little-endian format.**

| Opcode        | Command           | Length | Arguments  |
| ------------- |:------------------|:-------|:-----------|
| 1  | Unused | | |
| 2  | Toggle LED | 4 | 3 bytes - Bitmap representation of node IDs to toggle (eg. 00000101 = toggle nodes 0 and 2)|
| 3  | Unused | | |
| 4  | Unused | | |
| 5  | Unused | | |
| 6  | Read environment service config | 2 | 1 byte - Node ID |
| 7  | Write environment service config | 14 | 1 byte - Node ID <br> 2 bytes - Temperature interval in ms (100 - 60k) <br> 2 bytes - Pressure interval in ms (50 - 60k) <br> 2 bytes - Humidity interval in ms (100 - 60k) <br> 2 bytes - Color interval in ms (200 - 60k) <br> 1 byte - Gas mode (1, 2 or 3) <br> 3 bytes - Color sensor LED RGB (0-255)|
| 8  | Read motion service config | 2 | 1 byte - node ID |
| 9  | Write motion service config | 11 | 1 byte - Node ID <br> 2 bytes - Step interval in ms (100 - 5k) <br> 2 bytes - Temperature compensation interval in ms (100 - 5k) <br> 2 bytes - Magnetometer compensation interval in ms (100 - 1k) <br> 2 bytes - Processing frequency in Hz (5 - 200) <br> 1 byte - Wake-on-motion (0 or 1) |
| 10 | Toggle sensor | 4 | 1 byte - Node ID <br> 1 byte - Sensor ID* <br> 1 byte - Value (0 or 1) |
| 11 | Read connection parameters | 2 | 1 byte - Node ID |
| 12 | Write connection parameters | 10 | 1 byte - Node ID <br> 2 bytes - Minimum interval (unit 1.25ms) (6-3200) <br> 2 bytes - Maximum interval (unit 1.25ms) (6-3200) <br> 2 bytes - Slave latency (number of events) (0-499) <br> 2 bytes - Supervision timeout (unit 10ms) (10-3200) <br><br> **Constraint: sup_timeout * 4 > (1 + slave_latency) * max_conn_interval** |
| 13 | Read external pins | 2 | 1 byte - Node ID |
| 14 | Write external pins | 6 | 1 byte - Node ID <br> 4 bytes - Value for each pin (0 or 255) |

&ast; Since this firmware is designed to be used with [this EPICS controller](https://github.com/epicsNSLS2-sensors/ThingyAggregatorIOC), the sensor ID argument matches the IDs used internally by the controller.

| Sensor        | ID  |
| ------------- |:----|
| Temperature   | 5   |
| Humidity      | 6   |
| Pressure      | 7   |
| Gas           | 8   |
| Quaternions   | 41  |
| Raw motion    | 42  |
| Euler         | 43  |
| Heading       | 44  |

## Responses
Notifications pushed to UUID ```3e520003-1368-b682-4440-d7dd234c45bc``` may be sensor readings or responses to a command, such as
config readings. To discern the meaning of the notification, each has an opcode and a node ID. 

The opcode will be the first byte, the second is the MSB of the node ID, and the third is the LSB of the node ID. Following the 
node ID will be the value(s) of the response.

**All multi-byte values (except the node ID) are transmitted in little-endian format.**

| Opcode        | Response         | Length | Values  |
| ------------- |:-----------------|:-------|:--------|
| 1   | Node connected             | 11+ | 1 byte - Device type <br> 1 byte - Button state (0 or 1) <br> 1 byte - LED state <br> 3 bytes - LED RGB (uint8 x 3) <br> 1 byte - RSSI (int8) <br> 1 byte - PHY <br> 0-15 bytes - Bluetooth name of device  |
| 2   | Node disconnected          | 3   | |
| 3   | Button                     | 7   | 1 byte - ??? (hardcoded to 3 by Nordic?) <br> 1 byte - Button state <br> 1 byte - RF PHY <br> 1 byte - RSSI |
| 4   | Battery                    | 4   | 1 byte - Battery level (uint8) |
| 5   | Unused                     |     | |
| 6   | RSSI                       | 4   | 1 byte - RSSI (int8) |
| 7   | Temperature                | 5   | 1 byte - Temperature integer (int8) <br> 1 byte - Temperature decimal (uint8) |
| 8   | Pressure                   | 8   | 4 bytes - Pressure integer (int32) <br> 1 byte - Pressure decimal (uint8) |
| 9   | Humidity                   | 4   | 1 byte - Humidity (uint8) |
| 10  | Gas                        | 7   | 2 bytes - Carbon monoxide ppm (uint16) <br> 2 bytes - TVOC ppb (uint16) |
| 11  | Environment service config | 15  | 2 bytes - Temperature interval in ms (uint16) <br> 2 bytes - Pressure interval in ms (uint16) <br> 2 bytes - Humidity interval in ms (uint16) <br> 2 bytes - Color interval in ms (uint16) <br> 1 byte - Gas mode (uint8) <br> 3 bytes - Color sensor RGB (uint8 x 3)|
| 12  | Quaternions                | 19  | **Values are in 2Q30 fixed point format.** <br> 4 bytes - W (int32) <br> 4 bytes - X (int32) <br> 4 bytes - Y (int32) <br> 4 bytes - Z (int32) |
| 13  | Raw motion                 | 21  | **6Q10 fixed point format** <br> 2 bytes - Accelerometer X in Gs (int16) <br> 2 bytes - Accelerometer Y <br> 2 bytes - Accelerometer Z <br> **11Q5 fixed point format** <br> 2 bytes - Gyroscope X in deg/s (int16) <br> 2 bytes - Gyroscope Y <br> 2 bytes - Gyroscope Z <br> **12Q4 fixed point format** <br> 2 bytes - Compass X in uT (int16) <br> 2 bytes - Compass Y <br> 2 bytes - Compass Z|
| 14  | Euler                      | 15  | **Values are in 16Q16 fixed point format.** <br> 4 bytes - Roll in degrees (int32) <br> 4 bytes - Pitch in degrees (int32) <br> 4 bytes - Yaw in degrees (int32)|
| 15  | Heading                    | 7   | **Values are in 16Q16 fixed point format.** <br> 4 bytes - Heading in degrees (int32)|
| 16  | Motion service config      | 12  | 2 bytes - Step counter interval in ms (uint16) <br> 2 bytes - Temperature compensation interval in ms (uint16) <br> 2 bytes - Magnetometer compensation interval in ms (uint16) <br> 2 bytes - Processing unit frequency in Hz (uint16) <br> 1 byte - Wake-on-motion (0 or 1) |
| 17  | Connection parameters      | 11  | 2 bytes - Minimum connection interval (unit 1.25ms) (uint16) <br> 2 bytes - Maximum connection interval (unit 1.25ms) (uint16) <br> 2 bytes - Slave latency (number of events) (uint16) <br> 2 bytes - Supervision timeout (unit 10ms) (uint16) |
| 18  | External pins              | 7   | Each pin is either 0 or 255. <br><br> 1 byte - MOS_1 <br> 1 byte - MOS_2 <br> 1 byte - MOS_3 <br> 1 byte - MOS_4 |
