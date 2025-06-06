# zigbee-testing

## Contents and Purpose

This repository contains code for 2 ESP32-H2 Devkit N4 boards, as well as code for a RPi5. The purpose of this project is to set up a Zigbee network between the ESP boards where they can be remotely located in a test environment, where the RPi acts as a data log and storage.

## Description of Example Test Setup

This code is configured for one of the ESP32 boards to act as a coordinator for the Zigbee network. It will collect data from an attached vibration sensor, and send it to the router ESP32 via a custom zigbee cluster message every 15 seconds. The coordinator will also be connected to the RPi5 and send its round trip times via an I2C connection. The RPi will store the received data in a local file called rtt_log.csv, so the data can be reviewed at a later moment. On both esp32 boards, an LED can be connected to indicate the status of the network (on = connected, off = disconnected). If the coordinator does not receive any sensor data acknowledgements, it will also log a disconnect to the RPi inside "disconnect_log.csv".

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

The Zigbee connection should be made if the ESP devices are powered. Whenever the user wishes to start logging, they may run the offload.py which will begin adding data to the local .csv files.

## Things to note

- The sensor data is sent on 15 second intervals in this example.
- The LED in the router setup will turn off if it detects it no longer is in the network. The LED on coordinator setup will turn off when it does not receive acknowledgements from the router on sent data (17 second timeout = 15 sec send time + 2 sec buffer).
- Much of the functionality for sending data between devices is handled through timeouts. Ensure there is proper wait time for deciding when a test has failed or succeeded and be wary of what is changed and how it can affect syncing of the other timers.

 ## Troubleshooting and Failsafes
- The coordinator will open its network for 255 seconds on a timer that re-opens when that window expires. This is to keep the connection live for development and testing, but is not recommended for deployment.
- If any normal disconnect happens due to some random interference, the router will begin network steering to reconnect. This check happens on a loop every 2 seconds.
- When the coordinator restarts, it creates a new network with different parameters. The router will not realize this has happened and cache data on the previous network state. This will stop data from being sent back and forth, even though the router LED will stay on. There is a timer that will trigger after 30 seconds of not receiving data, which will cause a factory reset on the board, clearing this cache. This will allow the router to restart searching for the new network and not sit stuck.
- It can be helpful to allow the coordinator to fully boot (5 seconds is good enough usually) before starting the router to ensure the network is ready for joining.
- When planning to hook up devices remotely (especially after code modifications), it can be helpful to make the connection at a close range through a computer so the initial setup can be monitored. Then the devices can be plugged into remote batteries and moved when the connection is established.
- For any unforseen error where the Zigbee connection is not made, erasing the flash on one or both devices can help (usually just the router needs this). To do this, open an ESP-IDF terminal and run idf.py erase-flash or specify a port using the -p PORT_NAME flag. This will clear the cache and other items that may be holding up the devices and allow a clean reconnection.


