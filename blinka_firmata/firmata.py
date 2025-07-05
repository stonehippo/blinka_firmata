'''
Copyright (c) 2025 George White

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

import asyncio
import aioserial

from serial.tools import list_ports
from serial.serialutil import SerialException

from blinka_firmata.firmata_constants import FirmataConstants

from typing import Optional

# store the state of the digital ports
DIGITAL_PORTS = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

STARTUP_DELAY = 4.0

class Firmata:
	def __init__(
			self,
			port:Optional[str]=None,
			baud:int=115200,
			reset_period:int=4,
			loop:Optional[asyncio.AbstractEventLoop]=None
		):
		self._port:str = port
		self._baudrate:int = baud
		self._device:Optional[aioserial.AioSerial] = None
		self._reset_period:int = reset_period
		# Firmata typically polls its pins every 19ms; this can be set as low as 10ms
		self._sampling_interval_ms:int = 19
		self._loop:asyncio.EventLoop = loop
		self._data_handler_task:Optional[asyncio.Task] = None

		# response data for queries and reports
		self._report_data = {
			FirmataConstants.ANALOG_MAPPING_RESPONSE: None,
			FirmataConstants.CAPABILITY_RESPONSE: None,
			FirmataConstants.PIN_STATE_RESPONSE: None,
			FirmataConstants.REPORT_VERSION: "",
			FirmataConstants.REPORT_FIRMWARE: "",
			FirmataConstants.STRING_DATA: ""
		}


	@property
	def port(self) -> int:
		return self._port
	
	@property
	def baudrate(self) -> int:
		return self._baudrate
	
	@baudrate.setter
	def baudrate(self, value: int):
		self._device.baudrate = value
		self._baudrate = self._device.baudrate

	@property
	def firmata_protocol(self) -> Optional[str]:
		return self._report_data.get(FirmataConstants.REPORT_VERSION)
	
	@property
	def firmata_firmware(self) -> Optional[str]:
		return self._report_data.get(FirmataConstants.REPORT_FIRMWARE)

	
	@property
	def capability_map(self) -> Optional[list]:
		return self._report_data.get(FirmataConstants.CAPABILITY_RESPONSE)
	
	@property
	def readable_capability_map(self) -> Optional[dict]:
		return _translate_capability_map(self.capability_map)
	
	@property
	def analog_map(self) -> Optional[list]:
		return self._report_data.get(FirmataConstants.ANALOG_MAPPING_RESPONSE)	
	@property
	def sampling_interval_ms(self) -> int:
		return self._sampling_interval_ms

	@property 
	def loop(self) -> Optional[asyncio.AbstractEventLoop]:
		if self._loop:
			return self._loop
		else:
			return asyncio.get_running_loop()
	
	@loop.setter
	def loop(self, value: Optional[asyncio.AbstractEventLoop]):
		self._loop = value

	async def connect(self, find_timeout=5):
		if self._port:
			await self._connect_firmata()
		else:
			try:
				async with asyncio.timeout(find_timeout):
					await self._find_firmata()
			except TimeoutError:
				print(f"Timed out trying to detect a device with Firmata after {find_timeout} seconds")

	async def disconnect(self):
		try:
			await self._data_handler_task.cancel()
		except:
			pass
		self._device.close()
		print(f"Disconnected Firmata device at {self._port}")

	async def _connect_firmata(self):
		print(f"Connecting to {self._port}…")
		try:
			self._device = aioserial.AioSerial(
				port=self._port,
				baudrate=self._baudrate,
				timeout=1,
				write_timeout=0
			)

			self._data_handler_task = self.loop.create_task(self._firmata_data_handler())

			# try to get Firmata info
			print("Attempting to get Firmata firmware protocol version…")
			self._firmata_protocol = await self.report_version()
			
			if self._firmata_protocol is not None:
				while self._report_data.get(FirmataConstants.REPORT_VERSION) == "":
					await asyncio.sleep(0)
				print(f"Connected to Firmata device at {self._port} with protocol version {self.firmata_protocol}")
				# Let's query the firmware to see what's available
				await self._capability_query()
				await self._analog_mapping_query()
			else:
				await self.disconnect()
				print(f"Device at {self._port} is not running Firmata")
		except SerialException:
			print(f"Failed to connect to {self._port}")
			pass

	async def _find_firmata(self):
		print("Checking for Firmata devices…")
		ports = list_ports.comports()
		# Find candidate devices
		for port in ports:
			if port.pid is not None:
				# Open the candidate port and see if it could be a Firmata device
				print(f"Trying {port.device}…")
				self._port = port.device
				await self._connect_firmata()
				if self._firmata_protocol is not None:
					# ok, we found a Firmata device, we're going to stop looking
					break

	# basic command sender
	async def _firmata_command(self, command):
		command_bytes = bytes(command)
		try:
			response = await self._device.write_async(command_bytes)
		except SerialException:
			raise RuntimeError('failed to send command')
		return response

	# system exclusive (sysex) command sender
	async def _firmata_sysex_command(self, command, data=[]):
		# create the basic command
		sysex_command = [FirmataConstants.START_SYSEX, command, FirmataConstants.END_SYSEX]
		# splice data into the command
		sysex_command[2:1] = data
		await self._firmata_command(sysex_command)

	# event handler for incoming data from the firmware
	async def _firmata_data_handler(self):
		while True:
			incoming = await self._device.read_async()

			# guard against empty bytes returned
			if incoming:
				incoming = ord(incoming)
			else:
				continue

			print(incoming, flush=True)

			if incoming == FirmataConstants.REPORT_VERSION:
				# read the next two bytes to get the version number
				major, minor = bytearray(await self._device.read_async(size=2))

				self._report_data[FirmataConstants.REPORT_VERSION] = f"{major}.{minor}"

			if incoming == FirmataConstants.START_SYSEX:
				await self._handle_sysex()

			await asyncio.sleep(0)

	async def _handle_sysex(self):
		# what type of sysex is this?
		report_type = None
		incoming = ord(await self._device.read_async())

		if incoming == FirmataConstants.REPORT_FIRMWARE:
			report_type = FirmataConstants.REPORT_FIRMWARE
			# get the firmware version number before the rest
			fw_major, fw_minor = bytearray(await self._device.read_async(size=2))

		if incoming == FirmataConstants.ANALOG_MAPPING_RESPONSE:
			report_type = FirmataConstants.ANALOG_MAPPING_RESPONSE

		if incoming == FirmataConstants.CAPABILITY_RESPONSE:
			report_type = FirmataConstants.CAPABILITY_RESPONSE

		if incoming == FirmataConstants.PIN_STATE_RESPONSE:
			report_type = FirmataConstants.PIN_STATE_RESPONSE

		if incoming == FirmataConstants.STRING_DATA:
			report_type = FirmataConstants.STRING_DATA

		sysex = []
		
		# read the rest of the sysex message
		while incoming is not FirmataConstants.END_SYSEX:
			incoming = ord(await self._device.read_async())
			# print(incoming, flush=True)
			if incoming is not FirmataConstants.END_SYSEX:
				sysex.append(incoming)
			else:
				if report_type is FirmataConstants.REPORT_FIRMWARE or report_type is FirmataConstants.STRING_DATA:
					s = "".join(chr(c) for c in sysex)
					if report_type == FirmataConstants.REPORT_FIRMWARE:
						s = f"{fw_major}.{fw_minor} {s}"
					self._report_data[report_type] = s
				else:
					self._report_data[report_type] = sysex
				report_type = None

	# firmware info and control
	async def report_version(self):
		"""
		Query the Firmata protocol version supported by the firmware
		"""
		return await self._firmata_command(FirmataConstants.REPORT_FIRMWARE)

	async def report_firmware(self):
		"""
		Query the name and version of the firmware
		"""
		return await self._firmata_sysex_command(FirmataConstants.REPORT_FIRMWARE)
	
	async def reset_firmata(self):
		await self._firmata_command(FirmataConstants.SYSTEM_RESET)
		await asyncio.sleep(self._reset_period)

	async def set_sampling_interval(self, interval:int):
		data = [interval & 0x7f, (interval >> 7) & 0x7f]
		await self._firmata_sysex_command(FirmataConstants.SAMPLING_INTERVAL, [data])
		self._sampling_interval_ms = interval

	# queries
	async def _analog_mapping_query(self):
		"""
		Query the mapping of analog pins defined by the firmware
		"""
		await self._firmata_sysex_command(FirmataConstants.ANALOG_MAPPING_QUERY)

	async def _capability_query(self):
		"""
		Query the capabilities (modes) available for the pins defined by the firmware
		"""
		await self._firmata_sysex_command(FirmataConstants.CAPABILITY_QUERY)

	async def pin_state_query(self, pin:int):
		"""
		Query the state of a pin; the response is dependent on the current mode
		"""
		await self._firmata_sysex_command(FirmataConstants.PIN_STATE_QUERY)

	# pin management
	async def set_pin_mode(self, pin:int, mode:int):
		command = (FirmataConstants.SET_PIN_MODE, pin, mode)
		await self._firmata_command(command)

		# give the system a moment to set the new pin mode
		await asyncio.sleep(0.05)
	
	async def get_pin_state(self, pin:int) -> tuple:
		report = None
		await self._firmata_sysex_command(FirmataConstants.PIN_STATE_QUERY, [pin])
		while report is None:
			await asyncio.sleep(0)
		return report

	# digital pin operations
	async def set_digital_pin_reporting(self, pin: int, enable:bool=True):
		port, _ = _pin_port_and_mask(pin)
		if enable:
			command = (FirmataConstants.REPORT_DIGITAL + port, 1)
		else:
			command = (FirmataConstants.REPORT_DIGITAL + port, 0)
		await self._firmata_command(command)

	async def digital_write(self, pin:int, value:bool, use_port=False) -> None:
		"""
		Set a digital pin, either directly or via the port-pin

		:param pin: Arduino pin number to set

		:param value: Value to set the pin (True or False)

		:param use_port: Use port-pin to set the pin value; port is calculated automatically
		"""
		pin_value = 0
		if value:
			pin_value = 1
		
		if use_port:
			port, mask = _pin_port_and_mask(pin)
			if pin_value == 1:
				DIGITAL_PORTS[port] |= mask
			else:
				DIGITAL_PORTS[port] &= ~mask
			command = (FirmataConstants.DIGITAL_MESSAGE + port, DIGITAL_PORTS[port] & 0x7f, (DIGITAL_PORTS[port] >> 7) & 0x7f)
		else:
			command = (FirmataConstants.SET_DIGITAL_PIN_VALUE, pin, pin_value)
		await self._firmata_command(command)


	# analog pin operations	
	async def set_analog_pin_reporting(self, pin:int, enable=True):
		if enable:
			command = (FirmataConstants.REPORT_ANALOG + pin, 1)
		else:
			command = (FirmataConstants.REPORT_ANALOG + pin, 0)
		await self._firmata_command(command)

	async def pwm_write(self, pin, value:int) -> None:
		"""
		Write a analog value to a pin

		:param pin: Arduino pin number to set

		: param value:  Value to set the pin (0x0 - 0x4000)
		"""
		pwm_pin = FirmataConstants.ANALOG_MESSAGE + pin
		if pwm_pin < 0xf0:
			command = (pwm_pin, value & 0x7f, (value >> 7) & 0x7f)
			await self._firmata_command(command)
		else:
			data = [pin, value & 0x7f, (value >> 7) & 0x7f, (value >> 14) & 0x7f]
			await self._firmata_sysex_command(FirmataConstants.EXTENDED_ANALOG, data)

# utility functions
def _pin_port_and_mask(pin:int) -> tuple:
	return (pin // 8, 1 << (pin % 8))

def _translate_capability_map(cmap:list=None) -> dict:
	"""
	Create a human-readable version of the capability map sent by the firmaware
	"""
	if cmap is None:
		return None
	
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

