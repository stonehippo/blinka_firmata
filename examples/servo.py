import asyncio
from blinka_firmata.firmata import Firmata
from blinka_firmata.firmata_constants import FirmataConstants

board = Firmata()
servo_pin = 12
# Tune these values for you servo
min_pulse = 544 # servo at ~0 angle
max_pulse = 2400 # servo at ~180 angle

async def main():
    # pass
    await board.connect()
    # set up the servo config for the pin
    await board.servo_config(servo_pin)
    # await asyncio.sleep(0.01)
    await board.set_pin_mode(servo_pin, FirmataConstants.PIN_MODE_SERVO)
    # for l in range(0,180):
    #     print(f"setting servo to {l}")
    #     await board.pwm_write(servo_pin, l)
    #     await asyncio.sleep(0.25)
    # # try moving the servo to the ~0 degree position
    await board.servo_write(servo_pin, 0)
    await asyncio.sleep(3)

    # # try moving the servo to the ~180 degree position
    await board.servo_write(servo_pin, 180)
    await asyncio.sleep(3)

if __name__ == "__main__":
    asyncio.run(main())