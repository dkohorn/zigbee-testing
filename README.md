# zigbee-testing

## Contents and Purpose

This repository contains code for 2 ESP32-H2 Devkit N4 boards, as well as code for a RPi5. The purpose of this project is to set up a Zigbee network between the ESP boards where they can be remotely located in a test environment, where the RPi acts as a data log and storage.

## Description of Example Test Setup

This code is configured for one of the ESP32 boards to act as a coordinator for the Zigbee network. It will collect data from an attached vibration sensor, and send it to the router ESP32 via a custom zigbee cluster message every 5 seconds. The coordinator will also be connected to the RPi5 and send its round trip times via an I2C connection. The RPi will store the received data in a local file called rtt_log.csv, so the data can be reviewed at a later moment. On both esp32 boards, an LED can be connected to indicate the status of the network (on = connected, off = disconnected).

## How to Use

Download all contents of the folders "Zigbee_Router" and "Zigbee_Coordinator" as well as the python script in the "RPi" folder.

## Things to note

