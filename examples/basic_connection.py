import asyncio
from blinka_firmata.firmata import Firmata

# Find and connect to the first serial port with a Firmata-compatible device
# This assumes the standard baud rate for StandardFirmata of 115200bps
board = Firmata()

# Optionally select a specific device or at a given data rata
# board = Firmata(port="/dev/ttyUSB0", baud=57600)

async def main():
	await board.connect()
	await board.disconnect()

if __name__ == "__main__":
	asyncio.run(main())

