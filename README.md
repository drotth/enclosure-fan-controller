﻿# enclosure-fan-controller

Fan controller software aimed for 3D-printing enclosures. The design is specifically aimed to be used with a custom-built enclosure, featuring one major exhaust fan. The fan speed should react to several inputs:
- The amount of plastic particles in the air inside the enclosure
- Heat of the air inside the enclosure
- Manual inputs (such as speed override)

The software is designed to be run on an Arduino Uno, and as such is written in C/C++ as an [Arduino sketch](https://docs.arduino.cc/learn/programming/sketches/). Basically, it is still C/C++ but with some domain-specific libraries, and [a few transformations](https://arduino.github.io/arduino-cli/1.0/sketch-build-process/) to the code is made before being compiled and sent to the microcontroller itself.

## Hardware
The hardware setup is comprised of a few things:
- The microcontroller itself, [Arduino Uno](https://docs.arduino.cc/hardware/uno-rev3/), running an ATmega328p processor
- A PWM-controllable fan, [Noctua NF-a12x25 5V PWM](https://noctua.at/en/nf-a12x25-5v-pwm/specification)
	- Noctua PWM specifications white paper: https://noctua.at/pub/media/wysiwyg/Noctua_PWM_specifications_white_paper.pdf
- Sensors:
	- Particle / air quality sensor (tVoc and/or PM2.5) (_not yet implemented_)
  - Temperature sensor (_not yet implemented_)
- A generic 5-pin rotary encoder with a switch button
- An LCD screen to show information and status (_not yet implemented_)

<img src="schematic.png" alt="drawing" width="700"/>

(schematic created in [KiCAD](https://www.kicad.org/))
