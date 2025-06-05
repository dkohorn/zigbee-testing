# zigbee-testing

## Contents and Purpose

This repository contains code for 2 ESP32-H2 Devkit N4 boards, as well as code for a RPi5. The purpose of this project is to set up a Zigbee network between the ESP boards where they can be remotely located in a test environment, where the RPi acts as a data log and storage.

## Description of Example Test Setup

This code is configured for one of the ESP32 boards to act as a coordinator for the Zigbee network. It will collect data from an attached vibration sensor, and send it to the router ESP32 via a custom zigbee cluster message every 5 seconds. The coordinator will also be connected to the RPi5 and send its round trip times via an I2C connection. The RPi will store the received data in a local file called rtt_log.csv, so the data can be reviewed at a later moment. On both esp32 boards, an LED can be connected to indicate the status of the network (on = connected, off = disconnected).

## How to Use

Download all contents of the folders "Zigbee_Router" and "Zigbee_Coordinator" (there are many files in these folders, some of which are required and some not, it is unknown what specifically is needed, so recommended to take everything) as well as the offload.py script in the "RPi" folder. Build and flash the Zigbee code onto the ESP32 devices (the main file is gateway.c). All pin connections can be altered at the users discression, however the example layout uses:

**ESP32 Coordinator:**

LED - GPIO5 (10K resistor in series)

Vibration Sensor S - GPIO2 (1M0 in parallel)

Vibration Sensor G - GND

**ESP32 Router:**

LED - GPIO5 (10K resistor in series)

**RPi:**

SCL (5) - GPIO11 on ESP Coordinator

SDA (3) - GPIO10 on ESP Coordinator

GND - ESP GND

The Zigbee connection should be made if the ESP devices are powered. Whenever the user wishes to start logging, they may run the offload.py which will begin adding data to the local rtt_log.csv file.

## Things to note

- The sensor data is sent on 5 second intervals in this example.
- The LED status indicator on both devices turns on when data is received and acknowledgments are sent back. This means that if the connection breaks, it may take extra time for the LED to turn back on due to the data send delay (5 seconds by default). Keep this in mind when changing values associated with timeouts and message intervals.
- Much of the functionality for sending data between devices is handled through timeouts. Ensure there is proper wait time for deciding when a test has failed or succeeded.
- The RPi does not specifically log disconnects, as it would be tedious to implement. However, when looking at the round trip time logs, both the RTT itself and the timestamp associated with it can tell when a connection is broken:
    - My assumption is that a high RTT time indicates that there was an interference when the message was sent, delaying the readings in one or both directions. This may not necessarily mean the connection was broken, but could indicate a quick disconnect, not long enough for the timeout and LED to signal it.
    - The timestamps that differ by more than 5 seconds should indicate a real network failure, where the LED would turn off. From testing these disconnects to range from 6 seconds to a full minute before reconnecting. It is recommended to wait an extended period of time to ensure that the connection has truly failed, if that's what is believed to be the case.

 ## Troubleshooting
- When modifying code to re-flash onto the ESP boards, if they unexpectedly lose power, or during some other odd scenarios, the Zigbee connection will sometimes fail to establish. This will likely require the user to run idf.py erase-flash on both boards and them to be re-flashed to ensure a fresh network state. Especially when making changes to the code, it is recommended to test the network connection at the users setup, before moving the boards to remote locations.
- In addition to erasing and re-flashing, powering up the ESP coordinator before the ESP router can ensure that the network has time to establish before a connection attempts to be made.
- When the coordinator is started, the network opens for 255 seconds. If the router misses the window, the network may need to be reopened through a coordinator restart.

