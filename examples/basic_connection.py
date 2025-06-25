import asyncio
from blinka_firmata.firmata import Firmata

# Find and connect to the first serial port with a Firmata-compatible device
# This assumes the standard baud rate for StandardFirmata of 115200bps
board = Firmata()

# If you're running something like StandardFirmata, it'll be slower
# but we can still find it
# board = Firmata(baud=57600)

# Optionally select a specific device or at a given data rata
# board = Firmata(port="/dev/ttyUSB0", baud=57600)

async def main():
	await board.connect()
	await board.disconnect()

if __name__ == "__main__":
	asyncio.run(main())

