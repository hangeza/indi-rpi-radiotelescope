# indi-rpi-radiotelescope

This is a Instrument-Neutral-Device-Interface (INDI) driver intended to run on a Raspberry Pi 
and providing all functionality of an Az/Alt mount by interacting with SSI-based rotational
encoders via SPI interface and controlling GPIO pins for motor controls and digital
inputs/outputs. This driver is developed for a dedicated DIY mount of a 3m radio telescope.

Overview of features:
- decode SSI-interface based absolute rotary encoders via SPI buses 0 and 1 (main and auxiliary)
- provide generic GPIO interface class based on the pigpio daemon (pigpiod)
- control motors with PWM, direction and enable signals using the GPIO hardware PWM channels 0 and 1
- PiRT main driver class implements position readout, coordinate conversions, GOTO, Tracking, check for movement limits and others 

Howto build the driver:
- in root dir "mkdir build && cd build"
- "cmake ../pirt"
- "make"
- "sudo make install"

The binary should then be installed in system's bin directory and can be started with "indiserver indi_pirt".
