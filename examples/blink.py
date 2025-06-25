import asyncio
from blinka_firmata.firmata import Firmata

board = Firmata()

led_pin = 13
interval = 0.5
count = 5

async def blink(count=3, delay=1):
	for _ in range(count):
		await board.digital_write(led_pin, 1)
		await asyncio.sleep(delay)
		await board.digital_write(led_pin, 0)
		await asyncio.sleep(delay=interval)

async def main():
	await board.connect()
	await blink()

if __name__ == "__main__":
	asyncio.run(main())