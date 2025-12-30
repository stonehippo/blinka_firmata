'''
Call back registration for analog reporting data.

In this example, a couple if analog pins are given a callback
function to handle incoming data.
'''
import asyncio
from blinka_firmata.firmata import Firmata
from blinka_firmata.firmata_constants import FirmataConstants

board = Firmata()

# Analog pin to listen on
adc01 = 14 # A0 on Arduino UNO
adc02 = 19 # A5 on Arduino UNO

# A simple callback that just prints the data received from firmata.
def echo(data):
    print(data)

async def main():
    await board.connect()
    # register analog reporting callbacks
    for pin in (adc01, adc02):
        await board.set_pin_mode(pin, FirmataConstants.PIN_MODE_ANALOG)
        board.callbacks.subscribe(pin, event_type=FirmataConstants.ANALOG_MESSAGE, handler=echo)

    while True:
        await asyncio.sleep(0.001)

if __name__ == "__main__":
    asyncio.run(main())