## What's it?

This is the code for my bachelor thesis I'm currently working on. It's based on a simple linear moveable table carrying sensors. The result should be the profile of a grid roller.
I dropped leonardo support including support HID support for direct measuring into excel etc. The support for the leonardo is at this point quite bad :(

## What language is it written in?

It's written in arduino language code

## What's its current status?

Currently under devlopment

Roadmap includes the following goals:

* working with arduino leonardo for HID-support
* direct value input into every prgram you want

## How to install?


It's very easy. Just copy the files into your arduino sketchfolder and wire your system. You also need to install this additional libraries:

* i2cmaster by Peter Fleury http://jump.to/fleury
* AFMotor by Adafruit https://github.com/adafruit/Adafruit-Motor-Shield-library

## Wich hardware is used?

Currently are different parts used

* Arduino Uno
* Adafruit motorshield + AFmotor library
* 200 Rev Stepper (Pololu, 10V, 500mA)
* terminal switches - connected to D2/D3
* MLX90614 IR thermometer