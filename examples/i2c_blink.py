'''
Blink the LED on a SparkFun Qwiic Button Breakout
https://www.sparkfun.com/sparkfun-qwiic-button-breakout.html
'''

import asyncio
from blinka_firmata.firmata import Firmata

board = Firmata()

i2c_address = 0x6F
register = 0x19
interval = 0.5
count = 5

async def blink(count=3, delay=1.0):
	for _ in range(count):
		await board.i2c_write(i2c_address, (register, 0x33)) # type: ignore
		await asyncio.sleep(delay=delay)
		await board.i2c_write(i2c_address, (register, 0x00)) # type: ignore
		await asyncio.sleep(delay=delay)

async def main():
	await board.connect()
	await blink(count=count, delay=interval)

if __name__ == "__main__":
	asyncio.run(main())
