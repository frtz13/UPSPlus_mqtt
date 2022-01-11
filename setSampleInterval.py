
# adapted from UPSPlus upsPlus.py script by frtz13@github.com

import os
import sys
import smbus2

# Define I2C bus
DEVICE_BUS = 1

# Define device i2c slave address.
DEVICE_ADDR = 0x17

print("-"*60)
print("Modify sample interval in UPS Plus")
print("-"*60)

SAMPLEINTERVAL_MINI_min = 2
SAMPLEINTERVAL_MAX_min = 120

# Raspberry Pi Communicates with MCU via i2c protocol.
bus = smbus2.SMBus(DEVICE_BUS)
current_sample_interval_min = bus.read_byte_data(DEVICE_ADDR, 0x16) << 8 | bus.read_byte_data(DEVICE_ADDR, 0x15)
print(f"Current value:  {current_sample_interval_min} min")

if len(sys.argv) > 1:
    try:
        given_si_min = int(sys.argv[1])
        if given_si_min >= SAMPLEINTERVAL_MINI_min and given_si_min <= SAMPLEINTERVAL_MAX_min :
            bus.write_byte_data(DEVICE_ADDR, 0x15, given_si_min & 0xFF)
            bus.write_byte_data(DEVICE_ADDR, 0x16, 0)
            print(f"Successfully set the sample interval to: {given_si_min} min")
        else:
            errMsg = f"Sample interval should be given between {SAMPLEINTERVAL_MINI_min:.0f} and {SAMPLEINTERVAL_MAX_min:.0f} min"
            print(errMsg)
    except Exception as exc:
        print(f"Incorrect parameter: {sys.argv[1]}. ({str(exc)})")
else:
    print(f"Usage: {sys.argv[0]} <sample_interval_in_min>")
    print(f"<sample_interval_in_min> between {SAMPLEINTERVAL_MINI_min:.0f} and {SAMPLEINTERVAL_MAX_min:.0f}")