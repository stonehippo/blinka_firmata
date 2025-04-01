from blinka_firmata.firmata import Firmata

# Find and connect to the first serial port with a Firmata-compatible device
# This assumes the standard baud rate for StandardFirmata of 57600bps
board = Firmata()

# Select a specific device and at a given data rata
# board = Firmata(port="/dev/ttyUSB0", baud=115200)

# Cleanly close the serial port
board.disconnect()