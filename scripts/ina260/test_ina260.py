import argparse
from ina260.controller import Controller

parser = argparse.ArgumentParser()
addr_help = \
"""
i2c address of your ina260 in hexadecimal. Default 0x40. You can verify that it is detected using 'sudo i2cdetect 1'.
You should see one entry that's not empty in the table. Specify that number as the argument to this script, e.g. 0x41.
"""
parser.add_argument("address", help=addr_help, type=lambda x: int(x, 16), nargs='?', default='0x40')
args = parser.parse_args()

print(f"Attempting to connect INA260 at address {hex(args.address)}...\n")

c = Controller(args.address)
try:
    print("Successfully read values. They should be nonzero:\n"
          f"Current: {round(c.current(), 3)} A\nVoltage: {round(c.voltage(), 3)} V\nPower: {round(c.power(), 2)} W")
except OSError as e:
    print("Unable to connect to your INA260.")
    parser.print_help()
