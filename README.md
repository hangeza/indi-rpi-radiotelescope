# PiRaTe - The Pi Radio Telescope

## Overview
This is an all-in-one control system for an Az/Alt mount radio telescope originally written for the 3m radio antenna at the astronomical observatory in Radebeul/Dresden (Germany) but may be utilized for similar instruments. 
The main features are:
- Based on Raspberry Pi (tested for model 4) connecting via GPIOs to easy-to-acquire ubiquitous periphery modules for motor control, position sensing, GPS reception, I/O, ADCs etc.
- Abstraction to the hardware is managed through a custom Instrument-Neutral-Device-Interface (INDI) driver, which allows access to the scope through the quasi-standard XML-message-based INDI protocol. INDI is utilized in many remote observatories.
- An observation task manager (Radio Telescope Task Scheduler - RaTSche) which runs as daemon and manages tasks such as measurements, parking, maintenance etc. New tasks are added through a message queue interface e.g. through the included command-line-interface (CLI) client which allows easy integration into web-based interfaces.
- A collection of bash scripts which allow for more complex measurement tasks, i.e. 2d scans in horizontal and equatorial coordinates, transit scans, tracking measurements. The scripts are installed through CMake in the system's binary location together with indi-pirt and ratsche and are invoked by the ratsche daemon when required.
- systemd service files for indiserver and ratsche daemon to have the complete control chain started at boot time. These are installed into the according system locations by the cmake scripts, too.

## Howto build and install PiRaTe
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
sudo systemctl enable pigpiod.service && sudo systemctl start pigpiod.service
```

## Activate and Start PiRaTe
Now activate the driver and ratsche via systemd services:

    sudo systemctl enable indiserver.service && sudo systemctl start indiserver.service
    sudo systemctl enable ratsche.service && sudo systemctl start ratsche.service

These steps have to be executed only once. The indi driver as well as the task scheduler should now be running which may be checked by:

    systemctl status indiserver.service
    systemctl status ratsche.service

The output should not contain critical messages.
