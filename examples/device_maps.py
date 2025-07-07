import asyncio
from blinka_firmata.firmata import Firmata
from blinka_firmata.firmata_constants import FirmataConstants

'''
This example assums that you are using Configurable Firmata, at the default baud
of 115,200bps.
'''
# For StandardFirmata, use 57600
# BOARD_SPEED = 57600
BOARD_SPEED = 115200


# Automatically detect and connect to a Firmata device
async def main():
	board = Firmata(baud=BOARD_SPEED)
	await board.connect()

	# pause to give the board time to get set up
	await asyncio.sleep(2)

	# pretty print the capability map of the device
	print("\nBoard Capability Map")
	print("--------------------")
	print("pin # \tcapabilities")
	print("======\t============\n")
	for pin in board.readable_capability_map:
		print(pin, "\t", board.readable_capability_map[pin])
	
	print("")

	# pretty print out the analog map
	print("\nAnalog Pin Map")
	print("--------------")
	print("pin # \tIs Analog?\tAnalog Pin Name (Digital Pin Number)")
	print("======\t==========\t====================================\n")
	for pin_number in range(0, len(board.analog_map)):
		if board.analog_map[pin_number] == 127:
			is_analog = False
			pin_name = "-"
			equivalent_pin = ""
		else:
			is_analog = True
			pin_name = f"A{board.analog_map[pin_number]}"
			equivalent_pin = f"({pin_number})"
		print(f"pin_{pin_number}\t{is_analog}\t\t{pin_name} {equivalent_pin}")

	print("")

	await board.disconnect()


if __name__ == "__main__":
	asyncio.run(main())