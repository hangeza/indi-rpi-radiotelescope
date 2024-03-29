# PiRaTe - The Pi Radio Telescope

## Overview
This is an all-in-one control system for an Az/Alt mount radio telescope originally written for the 3m radio antenna at the astronomical observatory in Radebeul/Dresden (Germany) but may be utilized for similar instruments. 
The main features are:
- Based on Raspberry Pi (tested for model 4) connecting via GPIOs to easy-to-acquire ubiquitous periphery modules for motor control, position sensing, GPS reception, I/O, ADCs etc. The emplyed components and connections are documented here: https://oshwlab.com/antrares/pirate-mainboard
- Abstraction to the hardware is managed through a custom Instrument-Neutral-Device-Interface (INDI) driver, which allows access to the scope through the quasi-standard XML-message-based INDI protocol. INDI is utilized in many remote observatories.
- An observation task manager (Radio Telescope Task Scheduler - RaTSche) which runs as daemon and manages tasks such as measurements, parking, maintenance etc. New tasks are added through a message queue interface e.g. through the included command-line-interface (CLI) client which allows easy integration into web-based interfaces.
- A collection of bash scripts which allow for more complex measurement tasks, i.e. 2d scans in horizontal and equatorial coordinates, transit scans, tracking measurements. The scripts are installed through CMake in the system's binary location together with indi-pirt and ratsche and are invoked by the ratsche daemon when required.
- Four bash MQTT bridges which translate the indi and ratsche messages into an MQTT telemetry stream and vice versa: indi->MQTT, MQTT->indi, ratsche->MQTT and MQTT->ratsche
- systemd service files for indiserver, ratsche daemon and the MQTT bridges to have the complete control chain started at boot time. These are installed into the according system locations by the cmake scripts, too.

## Howto build and install PiRaTe
First, you need all the packages installed, that the driver depends on. Copy&Pasting following command line should do this job:

`sudo apt install libindi-dev libpigpio-dev libpigpiod-if-dev libnova-dev libgsl-dev`

Now, build and install the project:
- checkout the main branch of this git repository: `git clone https://github.com/hangeza/indi-rpi-radiotelescope.git`
- in the project dir `mkdir build && cd build`
- `cmake ../`
- `make`
- `sudo make install`

The binaries, shell scripts and systemd unit files should then be installed in the appropriate system locations.

## Prerequisites
The driver depends on the pigpiod daemon and indiserver which both must be installed and started first:
```
sudo apt install pigpiod indi-bin
sudo systemctl enable --now pigpiod.service
```

## Activate and Start PiRaTe
Now activate the driver and ratsche via systemd services:

    sudo systemctl enable --now indiserver.service
    sudo systemctl enable --now ratsche.service

These steps have to be executed only once. The indi driver as well as the task scheduler should now be running which may be checked by:

    systemctl status indiserver.service
    systemctl status ratsche.service

The output should not contain any critical messages.
