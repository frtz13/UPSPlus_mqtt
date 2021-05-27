
# adapted from UPSPlus upsPlus.py script by frtz13@github.com

import os
import sys
import smbus2

# Define I2C bus
DEVICE_BUS = 1

# Define device i2c slave address.
DEVICE_ADDR = 0x17

print("-"*60)
print("Modify battery protection voltage in UPS Plus")
print("-"*60)

PV_Mini_mV = 3000
PV_Maxi_mV = 4000

# Raspberry Pi Communicates with MCU via i2c protocol.
bus = smbus2.SMBus(DEVICE_BUS)
currentProtectionVoltage_mV = bus.read_byte_data(DEVICE_ADDR, 0x12) << 8 | bus.read_byte_data(DEVICE_ADDR, 0x11)
print("Current value:  %d mV" % currentProtectionVoltage_mV)

if len(sys.argv) > 1:
    try:
        givenPV_mV = int(sys.argv[1])
        if givenPV_mV >= PV_Mini_mV and givenPV_mV <= PV_Maxi_mV :
            bus.write_byte_data(DEVICE_ADDR, 0x11, givenPV_mV & 0xFF)
            bus.write_byte_data(DEVICE_ADDR, 0x12, (givenPV_mV >> 8)& 0xFF)
            print("Successfully set the protection voltage to: %d mV" % givenPV_mV)
        else:
            errMsg = "Protection voltage should be given between {:.0f} and {:.0f} mV".format(PV_Mini_mV, PV_Maxi_mV)
            print(errMsg)
    except Exception as exc:
        print("Incorrect parameter: {}. ({})".format(sys.argv[1], str(exc)))
else:
    print("Usage: {} <protection_voltage_in_mV>".format(sys.argv[0]))
    print("<protection_voltage_in_mV> between {:.0f} and {:.0f}".format(PV_Mini_mV,PV_Maxi_mV))