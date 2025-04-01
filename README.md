# blinka-firmata

A Python client for the [Firmata Protocol](https://github.com/firmata/protocol/).

This client was written as a layer for [CircuitPython Blinka](https://circuitpython.org/blinka) to allow control of boards running Firmata, such as an Arduino Uno. It can also be used as a standalone module for controlling Firmata devices.

This client supports the StandardFirmata firmware. Other Firmata firmware should work, possibly requiring some modifications to this library. Contributions for additional support are welcome.

## Adding Firmata Support to Blinka

There are several ways to use [Blinka](https://gist.github.com/stonehippo/2393ad06fb1d9524b22568f7110cf0ce) to connect a PC to peripherals like I2C and SPI. I am wondering if Blinka could also be made into a client for the classic [Firmata](https://github.com/firmata/protocol) protocol. This would make it possible to program devices running the [Firmata Arudino](https://github.com/firmata/arduino) using CircuitPython, which could be fun and useful.

If this works, it will be similar to the Blinka support for RP2040 devices](https://learn.adafruit.com/circuitpython-libraries-on-any-computer-with-raspberry-pi-pico). Firmata has support for several common peripherals, like GPIO, ADCs, and PWM, pluse more advanced stuff like I2C and SPI. It *should* be possible to use this as the base layer for at least some of Blinka's APIs.

## Why Do This?

First, for the learning. It's always good to try out new things!

Second, because Firmata has been around for quite a while and it's well understood. It has been the basis for a lot of extensions where controlling microcontroller board functions from a computer is desirable, such as [Johnny Five](https://johnny-five.io/) and [Gobot](https://gobot.io/). Although microcontrollers have gotten a lot more robust since the early days of Arduino, there are still times when you might need more heft than you can squeeze out of that tiny CPU.

Third, CircuitPython is great. It has all the goodness of Python, has a set of portable core APIs across lots of hardware, and tons of drivers for all sorts of sensors and devices. I user CircuitPython whereever I can. I also happen to have a lot of less powerful, 8-bit based development boards around, like the classic ATMEGA328P-based Arduino. I would like to use that hardware with CircuitPython, but it doesn't have the juice. I can and do still write code using Arduino or PlatformIO, but there are use cases where using an Arduino + computer might make sense.

Fourth, it's handy to have several options for using things like GPIO or I2C with a computer. Though I suspect some of the other Blinka options will always be stronger choices than anything sitting on top of Firmata, it doesn't hurt to have another tool available.

Fifth, I have put together a [micro-class on physical computing with CircuitPython](https://github.com/stonehippo/microclass_physical_computing/). I use the Adafruit Proximity Trinkey for that class and it works very well. But I want to look at other ways to make it work. One possibility would be using an Arduino-compatible board for the hardware, with Blinka for the class instruction. This probably not the ideal way to make it work, but I'm going to use it as a test case for now. 

Of course, you could always just run CircuitPython on a single board computer like a Raspberry PI, either directly or via Blinka. Sometimes, though, you just want to crank out code on your laptop and have it effect the real world.

## Working Tasks

I'm going to take my time on this work, so I'll be keeping track of what's done and working (or not) here.

- [ ] Do some research on Firmata and see what's possible
- [ ] Determine what features I went to support
  - [ ] GPIO
  - [ ] ADC
  - [ ] PWM
  - [ ] I2C
  - [ ] SPI
  - [ ] Other?
- [ ] Figure out if this should use an existing Firmata client (e.g. [pymata4](https://github.com/MrYsLab/pymata4)) or if I create something new
- [ ] Get a basic layer working


-----

Copyright (c) 2025 George White

See [LICENSE](./LICENSE) for details.
