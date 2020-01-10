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
  - Read/write 4 external digital pins
  - Toggle LED
  - Relay RSSI between node and aggregator
  - Support configuration characteristics
    - Thingy Environment service configuration
    - Thingy Motion service configuration
    - Thingy connection parameters characteristics
    
## Commands
Send commands to the aggregator by writing to UUID ```3e520002-1368-b682-4440-d7dd234c45bc```.

The first byte of every command is the opcode.

| Opcode        | Command           | Length | Arguments  |
| ------------- |:------------------|:-------|:-----------|
| 1  | Unused | | |
| 2  | Toggle LED | 4 | 3 bytes - Logical OR of node IDs to toggle  |
| 3  | Unused | | |
| 4  | Unused | | |
| 5  | Unused | | |
| 6  | Read environment service config | 2 | 1 byte - Node ID |
| 7  | Write environment service config | 14 | 1 byte - Node ID <br> 2 bytes - Temperature interval <br> 2 bytes - Pressure interval <br> 2 bytes - Humid interval <br> 2 bytes - Color interval <br> 1 byte - Gas mode |
| 8  | Read motion service config | 2 | 1 byte - node ID |
| 9  | Write motion service config | 11 | 1 byte - Node ID <br> 2 bytes - Step interval <br> 2 bytes - Temperature compensation interval <br> 2 bytes - Magnetometer compensation interval <br> 2 bytes - Processing frequency <br> 1 byte - Wake on motion |
| 10 | Toggle sensor | 4 | 1 byte - Node ID <br> 1 byte - Sensor ID <br> 1 byte - Value (0 or 1) |
| 11 | Read connection parameters | 2 | 1 byte - Node ID |
| 12 | Write connection parameters | 10 | 1 byte - Node ID <br> 2 bytes - Minimum interval (unit 1.25ms) <br> 2 bytes - Maximum interval (unit 1.25ms) <br> 2 bytes - Slave latency (number of events) <br> 2 bytes - Supervision timeout (unit 10ms) |
| 13 | Read external pins | 2 | 1 byte - Node ID |
| 14 | Write external pins | 6 | 1 byte - Node ID <br> 4 bytes - Value for each pin (0 or 255) |

The aggregator will respond to commands by pushing notifications to UUID ```3e520003-1368-b682-4440-d7dd234c45bc```. For both reading and writing, the format of the messages comply to the [Thingy documentation](https://nordicsemiconductor.github.io/Nordic-Thingy52-FW/documentation/firmware_architecture.html).
