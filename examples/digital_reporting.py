'''
Call back registration for digital reporting data.

In this example, momentary switches have been connected to several
pins. Each time the switch state changes, this is reported by
Firmata and handled by the reqisterd callback function.
'''
import asyncio
from blinka_firmata.firmata import Firmata
from blinka_firmata.firmata_constants import FirmataConstants

board = Firmata()

# A simple callback that just prints the data received from firmata.
def echo(data):
    print(data)

async def main():
    await board.connect()
    # register digital reporting callbacks on several pins
    for pin in (2, 7,12, 14, 19):
        await board.set_pin_mode(pin, FirmataConstants.PIN_MODE_PULLUP)
        board.callbacks.subscribe(pin, handler=echo)

    while True:
        await asyncio.sleep(0.001)

if __name__ == "__main__":
    asyncio.run(main())