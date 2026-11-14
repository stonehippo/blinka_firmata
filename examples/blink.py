'''
Blink the on-board (pin 13) LED on an Arduino
'''

import asyncio
from blinka_firmata.firmata import Firmata

board = Firmata()

led_pin = 13
interval = 0.5
count = 5

async def blink(count=3, delay=1.0):
	for _ in range(count):
		await board.digital_write(led_pin, True)
		await asyncio.sleep(delay=delay)
		await board.digital_write(led_pin, False)
		await asyncio.sleep(delay=delay)

async def main():
	await board.connect()
	await blink(count=count, delay=interval)

if __name__ == "__main__":
	asyncio.run(main())