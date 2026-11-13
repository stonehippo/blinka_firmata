'''
Copyright (c) 2025-2026 George White

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
'''

'''
a collection on utility functions for common tasks needed to work with 
Firmata messages and data.
'''

from typing import Optional

def pin_port_and_mask(pin:int) -> tuple:
	return (pin // 8, 1 << (pin % 8))

def port_from_data(data:int) -> int:
	return data & 0x0f

def translate_capability_map(cmap:Optional[list]=None) -> Optional[dict]:
	"""
	Create a human-readable version of the capability map sent by the firmaware
	"""
	if cmap is None:
		return
	
	MODES = ["DIGITAL_INPUT","DIGITAL_OUTPUT","ANALOG_INPUT","PWM","SERVO","SHIFT","I2C","ONEWIRE","STEPPER","ENCODER","SERIAL","INPUT_PULLUP","SPI","SONAR","TONE","DHT","FREQUENCY"]
	pin:int = 0
	tmap:dict = {}
	caps:list = []
	cap = True # flip flop betwen the capability and its resolution
	
	for item in cmap:
		if item == 127: # this is the end of a pins capablities
			tmap[f"pin_{pin}"] = caps
			caps = []
			pin = pin + 1
		else:
			if cap:
				item = MODES[item]
				cap = False
			else:
				cap = True
			caps.append(item)

	return tmap

def get_analog_pins(amap:Optional[list]=None) -> Optional[tuple]:
	if amap is None:
		return
	analog_pins = []

	for pin in range(0, len(amap)):
		if amap[pin] != 127:
			analog_pins.append(pin)
	
	return tuple(analog_pins)

def to_7bit(n):
	return [n & 0x7f, (n >> 7) & 0x7f]

def from_7bit(n):
	return n[0] + (n[1] << 7)

def port_pin_values(port:int, data:int, total_pins:int=20) -> list:
        pin_values = []
        pins_per_port = 8
        start_pin = port * pins_per_port
        stop_pin = min(total_pins, port * pins_per_port + pins_per_port)
        for pin in range(start_pin, stop_pin):
                value = data & 0x01
                pin_values.append(value)
                data >>= 1
        return pin_values
