'''
Blink the LED on a SparkFun Qwiic Button Breakout
https://www.sparkfun.com/sparkfun-qwiic-button-breakout.html
'''

import asyncio
from blinka_firmata.firmata import Firmata
from blinka_firmata.firmata_constants import FirmataConstants
from blinka_firmata.firmata_utility import from_7bit

board = Firmata()

i2c_address = 0x6F
register = 0x19

# A simple callback that just prints the data received from firmata.
def echo(data):
    print(f"I2C reply from {hex(data[0])} (register {hex(data[-1])}): {from_7bit(data[1])}")

async def read(count=3, delay=0.25):
    board.callbacks.subscribe(i2c_address, FirmataConstants.I2C_REPLY, echo)

    # Turn the led on, full brightness
    await board.i2c_write(i2c_address, (register, 0xFF)) # type: ignore
    # Read the value of the LED brightness register (wait for a bit to give Firmata some time to respond)
    await board.i2c_read(i2c_address, register, 1)
    await asyncio.sleep(delay)

    # Wait a little longer, so the blink is noticeable
    await asyncio.sleep(delay=1.75)

    # Turn the led off
    await board.i2c_write(i2c_address, (register, 0x00)) # type: ignore
    # Read the value of the LED brightness register (wait for a bit to give Firmata some time to respond)
    await board.i2c_read(i2c_address, register, 1)
    await asyncio.sleep(delay)

async def main():
	await board.connect()
	await read()

if __name__ == "__main__":
	asyncio.run(main())
