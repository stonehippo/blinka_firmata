'''
Use a sine value to cycle a PWM. Nice for fading an LED up and down.
'''
import asyncio
from blinka_firmata.firmata import Firmata
from blinka_firmata.firmata_constants import FirmataConstants

from math import cos

board = Firmata()

pwm_pin = 11

#
def map_range(num, inMin, inMax, outMin, outMax):
  return outMin + (float(num - inMin) / float(inMax - inMin) * (outMax - outMin))

# generate sine values
levels = [int(map_range(cos(x/10), -1.0, 1.0, 0, 255)) for x in range(0,360)]

async def main():
    await board.connect()
    await board.set_pin_mode(pwm_pin, FirmataConstants.PIN_MODE_PWM)
    a = 0.0
    while True:
        for level in levels:
            await board.pwm_write(pwm_pin, level)
            await asyncio.sleep(0.1)
        

if __name__ == "__main__":
    asyncio.run(main())