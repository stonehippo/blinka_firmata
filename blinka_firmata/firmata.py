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
		self._firmata_protocol:Optional[str] = None
		self._firmata_firmware:Optional[str] = None
		self._loop = loop

		# if port is None:
		# 	self._find_firmata()
		# else:
		# 	self._connect_firmata()

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
		return self._firmata_protocol
	
	@property
	def firmata_firmware(self) -> Optional[str]:
		return self._firmata_firmware

	@property 
	def loop(self) -> Optional[asyncio.AbstractEventLoop]:
		if self._loop:
			return self._loop
		else:
			return asyncio.get_running_loop()
	
	@loop.setter
	def loop(self, value: Optional[asyncio.AbstractEventLoop]):
		self._loop = value

	async def connect(self):
		if self._port:
			await self._connect_firmata()
		else:
			await self._find_firmata()

	async def _connect_firmata(self):
		print(f"Connecting to {self._port}…")
		try:
			self._device = aioserial.AioSerial(
				port=self._port,
				baudrate=self._baudrate,
				timeout=1,
				write_timeout=0
			)
			# try to get Firmata info
			self._firmata_protocol = await self.get_firmata_protocol()
			if self._firmata_protocol is not None:
				print(f"Connected to Firmata device at {self._port} with protocol version {self._firmata_protocol}")
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

	async def get_firmata_protocol(self):
		response = await self._firmata_command(FirmataConstants.REPORT_FIRMWARE)
		return response
	
	async def reset_firmata(self):
		self._firmata_command(FirmataConstants.SYSTEM_RESET)
		await asyncio.sleep(self._reset_period)

	async def disconnect(self):
		self._device.close()
		print(f"Disconnected Firmata device at {self._port}")
