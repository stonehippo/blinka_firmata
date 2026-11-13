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
Event handling for incoming Firmata messages
'''

from blinka_firmata.firmata_constants import FirmataConstants

class FirmataEvents:
	def __init__(self) -> None:
		self._registry = {
			FirmataConstants.ANALOG_MESSAGE: {},
			FirmataConstants.DIGITAL_MESSAGE: {}
		}
		# caches for previous values received
		self._digital_values = {}
		self._analog_values = {}

	# callback management
	def subscribe(self, pin_or_address, event_type=FirmataConstants.DIGITAL_MESSAGE, handler=None) -> None:
		self._registry[event_type][pin_or_address] = handler

	def unsubscribe(self, pin_or_address, event_type=FirmataConstants.DIGITAL_MESSAGE):
		if pin_or_address in self._registry[event_type].keys():
			self._registry[event_type].pop(pin_or_address)

	async def handle_event(self, pin_or_address, event_type=FirmataConstants.DIGITAL_MESSAGE, port=None, value=None):
		if pin_or_address in self._registry[event_type].keys():
			if event_type == FirmataConstants.DIGITAL_MESSAGE:
				# cache the value
				if pin_or_address not in self._digital_values.keys():
					self._digital_values[pin_or_address] = value
					self._registry[event_type][pin_or_address]([pin_or_address, value])  # type: ignore	
				if value != self._digital_values[pin_or_address]:
						self._digital_values[pin_or_address] = value
						self._registry[event_type][pin_or_address]([pin_or_address, value])  # type: ignore	
			elif event_type == FirmataConstants.ANALOG_MESSAGE:
				self._analog_values[pin_or_address] = value
				self._registry[event_type][pin_or_address]([pin_or_address, value])
			elif event_type == FirmataConstants.I2C_REPLY:
				pass