
#!/usr/bin/env python3

#   Copyright (C) 2021  frtz13.github.com

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

# fan control function: thanks to Andreas Spiess https://www.sensorsiot.org/variable-speed-cooling-fan-for-raspberry-pi-using-pwm-video138/
# mqtt publishing : many thanks to http://www.steves-internet-guide.com/into-mqtt-python-client/
# UPSPlus: https://github.com/geeekpi/upsplus

import os
import time
from time import sleep
# import logging
# import signal
import sys
import json
import configparser
# import random
from collections import deque
import syslog
import requests
import itertools as it

import RPi.GPIO as GPIO
import smbus2
from ina219 import INA219,DeviceRangeError
import paho.mqtt.client as mqtt

SCRIPT_VERSION = "2025.04.02"

CONFIG_FILE = "fanShutDownUps.ini"
CONFIGSECTION_FAN = "fan"
CONFIGSECTION_MQTT = "mqtt"
CONFIGSECTION_UPS = "ups"

CMD_NO_TIMER_BIAS = "--notimerbias"
CMD_SHUTDOWN_IMMEDIATELY_WHEN_ON_BATTERY = "--shutdowntest"

DEVICE_BUS = 1 # Define I2C bus
DEVICE_ADDR = 0x17 # Define device i2c slave address.

MQTT_TOPIC_FAN = "/fanspeed"
MQTT_TOPIC_UPS = "/ups"
MQTT_TOPIC_LWT = "/LWT"
MQTT_PAYLOAD_ONLINE = "online"
MQTT_PAYLOAD_OFFLINE = "offline"


def read_config_desired_cpu_temp():
    global DESIRED_CPU_TEMP
    confparser = configparser.RawConfigParser()
    confparser.read(os.path.join(sys.path[0], CONFIG_FILE))
    DESIRED_CPU_TEMP = int(confparser.get(CONFIGSECTION_FAN, "DESIRED_CPU_TEMP_degC"))


def read_config():
    global GPIO_FAN # The GPIO pin ID to control the fan
    global FAN_LOOP_TIME
    global FANSPEED_FILENAME
    global SEND_STATUS_TO_UPSPLUS_IOT_PLATFORM
    global UPSPLUS_IOT_PLATFORM_URL
    global INA219SHUNT_OUT
    global INA219SHUNT_BATT
    global BATT_LOOP_TIME
    global TIMER_BIAS_AT_STARTUP
    global one_hour_delay
    global SHUTDOWN_IMMEDIATELY_WHEN_ON_BATTERY # for testing shutdown only
    global SHUTDOWN_TIMEOUT
    global PROTECTION_VOLTAGE_MARGIN_mV
    global HIGH_BATT_TEMP
    global MQTT_BROKER
    global MQTT_PORT
    global MQTT_USERNAME
    global MQTT_PASSWORD
    global MQTT_TOPIC

    try:
        confparser = configparser.RawConfigParser()
        confparser.read(os.path.join(sys.path[0], CONFIG_FILE))

        GPIO_FAN = int(confparser.get(CONFIGSECTION_FAN, "GPIO_FAN"))
        FAN_LOOP_TIME = int(confparser.get(CONFIGSECTION_FAN, "FAN_LOOP_TIME_s"))
        read_config_desired_cpu_temp()

        try:
            INA219SHUNT_OUT = float(confparser.get(CONFIGSECTION_UPS, "INA219_SHUNT_OUT_Ohm"))
            print(f"INA219 shunt for output current: {INA219SHUNT_OUT} Ohm")
        except Exception as exc:
            INA219SHUNT_OUT = 0.00725 # value provided by Geekpi
        try:
            INA219SHUNT_BATT = float(confparser.get(CONFIGSECTION_UPS, "INA219_SHUNT_BATT_Ohm"))
            print(f"INA219 shunt for battery current: {INA219SHUNT_BATT} Ohm")
        except Exception as exc:
            INA219SHUNT_BATT = 0.005 # value provided by Geekpi

        SEND_STATUS_TO_UPSPLUS_IOT_PLATFORM = 1 == int(confparser.get(CONFIGSECTION_UPS, "SEND_STATUS_TO_UPSPLUS_IOT_PLATFORM"))
        try:
            UPSPLUS_IOT_PLATFORM_URL = confparser.get(CONFIGSECTION_UPS, "UPSPLUS_IOT_PLATFORM_URL")
        except:
            UPSPLUS_IOT_PLATFORM_URL = "https://api.52pi.com/feed"

        BATT_LOOP_TIME = int(confparser.get(CONFIGSECTION_UPS, "BATTERY_CHECK_LOOP_TIME_s"))
        if CMD_NO_TIMER_BIAS in sys.argv:
            TIMER_BIAS_AT_STARTUP = 0
            one_hour_delay = 0
            print("Timer bias at start-up set to 0")
        else:
            try:
                TIMER_BIAS_AT_STARTUP = -int(confparser.get(CONFIGSECTION_UPS, "TIMER_BIAS_AT_STARTUP"))
                if TIMER_BIAS_AT_STARTUP == 0:
                    one_hour_delay = 0
            except:
                TIMER_BIAS_AT_STARTUP = -300 + BATT_LOOP_TIME

        if CMD_SHUTDOWN_IMMEDIATELY_WHEN_ON_BATTERY in sys.argv:
            SHUTDOWN_IMMEDIATELY_WHEN_ON_BATTERY = True
        else:
            try:
                SHUTDOWN_IMMEDIATELY_WHEN_ON_BATTERY = (1 == int(confparser.get(CONFIGSECTION_UPS, "SHUTDOWN_IMMEDIATELY_WHEN_ON_BATTERY")))
            except:
                SHUTDOWN_IMMEDIATELY_WHEN_ON_BATTERY = False
        if SHUTDOWN_IMMEDIATELY_WHEN_ON_BATTERY:
            print("Will immediately shut down when UPS is on battery.")
        SHUTDOWN_TIMEOUT = int(confparser.get(CONFIGSECTION_UPS, "SHUTDOWN_TIMEOUT_s"))

        parm = "PROTECTION_VOLTAGE_MARGIN_mV"
        PROTECTION_VOLTAGE_MARGIN_mV = int(confparser.get(CONFIGSECTION_UPS, parm))
        protVmargin_mini_mV = 100
        protVmargin_maxi_mV = 500
        if PROTECTION_VOLTAGE_MARGIN_mV < protVmargin_mini_mV:
            print("{} set to {:.0f} mV".format(parm, protVmargin_mini_mV))
            PROTECTION_VOLTAGE_MARGIN_mV = protVmargin_mini_mV
        if PROTECTION_VOLTAGE_MARGIN_mV > protVmargin_maxi_mV:
            print("{} set to {:.0f} mV".format(parm, protVmargin_maxi_mV))
            PROTECTION_VOLTAGE_MARGIN_mV = protVmargin_maxi_mV

        try:
            HIGH_BATT_TEMP = int(confparser.get(CONFIGSECTION_UPS, "HIGH_BATT_TEMP"))
        except:
            HIGH_BATT_TEMP = 99 #disables warning about high battery temperature

        MQTT_BROKER = confparser.get(CONFIGSECTION_MQTT, "BROKER")
        MQTT_PORT = int(confparser.get(CONFIGSECTION_MQTT, "TCP_PORT"))
        MQTT_USERNAME = confparser.get(CONFIGSECTION_MQTT, "USERNAME")
        MQTT_PASSWORD = confparser.get(CONFIGSECTION_MQTT, "PASSWORD")
        MQTT_TOPIC = confparser.get(CONFIGSECTION_MQTT, "TOPIC")
        return True
    except Exception as e:
        errmsg = "Error when reading configuration parameters: " + str(e)
        print(errmsg)
        syslog.syslog(syslog.LOG_ERR, errmsg)
        return False


def on_MQTTconnect(client, userdata, flags, rc):
    client.connection_rc = rc
    if rc == 0:
        client.connected_flag = True
        msg = "Connected to MQTT broker"
        print(msg)
        syslog.syslog(syslog.LOG_INFO, msg)
        try:
            client.publish(MQTT_TOPIC + MQTT_TOPIC_LWT, MQTT_PAYLOAD_ONLINE, 0, retain=True)
        except:
            pass
    else:
        errMsg = {
            1: "Connection refused – incorrect protocol version",
            2: "Connection refused – invalid client identifier",
            3: "Connection refused – server unavailable",
            4: "Connection refused – bad username or password",
            5: "connection not autorized"
            }
        errMsgFull = "Connection to MQTT broker failed. " + errMsg.get(rc, f"Unknown error: {str(rc)}.")
        print(errMsgFull)
        syslog.syslog(syslog.LOG_ERR, errMsgFull)

def on_MQTTdisconnect(client, userdata, rc):
#    print("disconnecting reason  "  + str(rc))
    client.connected_flag = False

def MQTT_connect(client):
    # returns True if we started the connection loop, False otherwise
    # the connection loop will take care of reconnections
    client.on_connect = on_MQTTconnect
    client.on_disconnect = on_MQTTdisconnect
    client.will_set(MQTT_TOPIC + MQTT_TOPIC_LWT, MQTT_PAYLOAD_OFFLINE, 0, retain=True)
    if len(MQTT_USERNAME) > 0:
        client.username_pw_set(username=MQTT_USERNAME, password=MQTT_PASSWORD)
#    print("Connecting to broker ",MQTT_BROKER)
    try:
        client.connect(MQTT_BROKER, MQTT_PORT) #connect to broker
        client.loop_start()
    except Exception as e:
        errMsg = f"MQTT connection attempt failed: {e}. Will be retried."
        print(errMsg)
        syslog.syslog(syslog.LOG_WARNING, errMsg)
        return False
    timeout = time.time() + 5
    while client.connection_rc == -1: #wait in loop
        if time.time() > timeout:
            break
        time.sleep(1)
#        print("MQTT wait...")
    return True

def MQTT_terminate(client):
    try:
        if client.connected_flag:
            res = MQTT_client.publish(MQTT_TOPIC + MQTT_TOPIC_LWT, MQTT_PAYLOAD_OFFLINE, 0, retain=True)
#            if res[0] == 0:
#                print("mqtt go offline ok")
            MQTT_client.disconnect()
        sleep(1)
        client.loop_stop()
    except Exception as e:
        print("MQTT client terminated with exception: " + str(e))
        pass

def i2c_bus_read_byte_wait(devaddr, reg, delay_s):
    sleep(delay_s)
    return i2c_bus.read_byte_data(devaddr, reg)

class UPSPlus:
    def __init__(self, upsCurrent, upsHealthCheck):
        self._send_status_data_reply = ""
        # get battery status
        try:
            self._battery_voltage_V = inaBattery.voltage()
            try:
                self._battery_current_mA = -inaBattery.current() # positive: discharge current
            except DeviceRangeError:
                self._battery_current_mA = 16000
        except Exception as exc:
            raise Exception("[UPSPLus.init] Error reading inaBatt registers: " + str(exc))
        self._battery_current_avg_mA = upsCurrent.battery_current_avg_mA
        self._battery_power_avg_mW = upsCurrent.battery_power_avg_mW
        # get output status
        try:
            self._RPi_voltage_V = inaRPi.voltage()
            try:
                self._RPi_current_mA = inaRPi.current()
            except DeviceRangeError:
                self._RPi_current_mA = 16000
        except Exception as exc:
            raise Exception("[UPSPLus.init] Error reading inaRPi registers: " + str(exc))
        self._RPi_current_avg_mA = upsCurrent.out_current_avg_mA
        self._RPi_power_avg_mW = upsCurrent.out_power_avg_mW
        self._RPi_current_peak_mA = upsCurrent.out_current_peak_mA
        self._RPi_voltage_mini_V = upsCurrent.out_voltage_mini_V

        # we only read the full set of registers if we report back to the IOT Platform
        # otherwise just read the register values we actually use
        if SEND_STATUS_TO_UPSPLUS_IOT_PLATFORM:
            it_registers = it.chain(range(0x01,0x2A), range(0xF0,0xFC))
        else:
            it_registers = it.chain(range(0x07,0x0C), range(0x11, 0x15))
        try:
            # pick one of the following lines (with or without delay)
            # self._reg_buff = {i:i2c_bus.read_byte_data(DEVICE_ADDR, i) for i in it_registers}
            self._reg_buff = {i:i2c_bus_read_byte_wait(DEVICE_ADDR, i, 0.02) for i in it_registers}
        except Exception as exc:
            raise Exception("[UPSPLus.init] Error reading UPS registers: " + str(exc))

        self._USB_C_mV = self._reg_buff[0x08] << 8 | self._reg_buff[0x07]
        self._USB_micro_mV = self._reg_buff[0x0A] << 8 | self._reg_buff[0x09]
#        self.battery_temperature_degC = self.reg_buff[12] << 8 | self.reg_buff[11]
#       we very rarely get 0xFF at reg_buff[0x0C]. this value should be 0 anyway for realistic temperatures
        BATT_TEMP_CEILING = 70 # sometimes unrealistic values spoil my graphs
        self._battery_temperature_degC = min(self._reg_buff[0x0B], BATT_TEMP_CEILING)
        upsHealthCheck.check_batt_temperature(self._battery_temperature_degC)
        upsHealthCheck.check_charger(self.on_battery, self._battery_current_avg_mA)
#        self._battery_remaining_capacity_percent = self._reg_buff[0x14] << 8 | self._reg_buff[0x13]
#       remaining capacity always <= 100%
        BATT_CAPACITY_CEILING = 101
        self._battery_remaining_capacity_percent = min(self._reg_buff[0x13], BATT_CAPACITY_CEILING)
        self._healthy = upsHealthCheck.is_healthy

    @property
    def protection_voltage_mV(self):
        prot_voltage_mV = self._reg_buff[0x12] << 8 | self._reg_buff[0x11]
        # protection voltage should be between 3000 and (4000 - margin) mV.
        # the UPS firmware is supposed to shut down RPi power when battery voltage goes lower than the Battery Protection Voltage.
        # so we have to make sure to gracefully shut down the RPi before we reach this level.
        # the script will shut down the RPi at Battery Protection Voltage + _Margin (config. parameter). 
        # Battery Protection Voltage checks:
        # lower bound: make sure that the script will shut down the RPi when battery voltage comes close to the 3000 mV limit.
        # Below such battery voltage the charger circuit (IP5328) will go into low current charge mode,
        # and charging current won't be sufficient to power the RPi.
        # upper bound: make sure the script will not shut down the RPi with a (nearly) full battery
        if prot_voltage_mV >= 3000 and prot_voltage_mV <= (4000 - PROTECTION_VOLTAGE_MARGIN_mV) :
            pass
        else:
            PROTECT_VOLT_DEFAULT_mV = 3500
            errMsg = f"Protection voltage retrieved from UPS seems to have an incorrect value ({prot_voltage_mV:.0f} mV). "\
                f"Assumed to be {PROTECT_VOLT_DEFAULT_mV:.0f} mV"
            print(errMsg)
            syslog.syslog(syslog.LOG_WARNING, errMsg)
            prot_voltage_mV = PROTECT_VOLT_DEFAULT_mV
        return prot_voltage_mV

    @property
    def on_battery(self):
        # with previous firmwares (earlier than v. 10) it seemed more reliable to check discharging current.
        # with firmware >= v.10 we go back to checking the USB ports
        # USB-C and micro-USB voltages are sometimes not reported properly
        # return self._battery_current_avg_mA > 500 # positive current means battery is discharging
        return (self._USB_C_mV < 4000) and (self._USB_micro_mV < 4000)

    @property
    def battery_is_charging(self):
        return self._battery_current_avg_mA < 0

    @property
    def battery_voltage_V(self):
        return self._battery_voltage_V

    def MQTT_publish(self, mqttclient):
            dictPayload = {
                'UsbC_V': self._USB_C_mV / 1000,
                'UsbMicro_V': self._USB_micro_mV / 1000,
                'OnBattery': self.on_battery,
                'BatteryVoltage_V': self._battery_voltage_V,
                'BatteryCurrent_A': int(self._battery_current_mA) / 1000.,
                'BatteryCurrent_avg_A': int(self._battery_current_avg_mA) / 1000.,
                'BatteryPower_avg_W': int(self._battery_power_avg_mW)/1000.,
                'BatteryCharging': self.battery_is_charging,
                'BatteryRemainingCapacity_percent': self._battery_remaining_capacity_percent,
                'BatteryTemperature_degC': self._battery_temperature_degC,
                'OutputVoltage_V': self._RPi_voltage_V,
                'OutputVoltage_mini_V': self._RPi_voltage_mini_V,
                'OutputCurrent_A': int(self._RPi_current_mA) / 1000.,
                'OutputCurrent_avg_A': int(self._RPi_current_avg_mA) / 1000.,
                'OutputPower_avg_W': int(self._RPi_power_avg_mW) / 1000.,
                'OutputCurrent_peak_A': int(self._RPi_current_peak_mA) / 1000.,
                'IsHealthy': self._healthy,
            }
            if SEND_STATUS_TO_UPSPLUS_IOT_PLATFORM:
                dictPayload['UPSPlus_IOT_Platform_Reply'] = self._send_status_data_reply
            payload = json.dumps(dictPayload)
            try:
                res = mqttclient.publish(MQTT_TOPIC + MQTT_TOPIC_UPS, payload, 0, False)
            except:
                pass
    #        if res[0] == 0:
    #            print("publish successful: " + payload)
    #        else:
    #            print("public failed. result: " + str(res.result))

    def send_UPS_status_data(self):
        global send_status_data_warncount
        # time.sleep(random.randint(0, 3))
        tel_data = {
            'PiVccVolt': self._RPi_voltage_V,
            'PiIddAmps': self._RPi_current_mA,
            'BatVccVolt': self._battery_voltage_V,
            'BatIddAmps': -self._battery_current_mA,
            'McuVccVolt': self._reg_buff[2] << 8 | self._reg_buff[1],
            'BatPinCVolt': self._reg_buff[6] << 8 | self._reg_buff[5],
            'ChargeTypeCVolt': self._reg_buff[8] << 8 | self._reg_buff[7],
            'ChargeMicroVolt': self._reg_buff[10] << 8 | self._reg_buff[9],
            'BatTemperature': self._reg_buff[12] << 8 | self._reg_buff[11],
            'BatFullVolt': self._reg_buff[14] << 8 | self._reg_buff[13],
            'BatEmptyVolt': self._reg_buff[16] << 8 | self._reg_buff[15],
            'BatProtectVolt': self._reg_buff[18] << 8 | self._reg_buff[17],
            'SampleTime': self._reg_buff[22] << 8 | self._reg_buff[21],
            'AutoPowerOn': self._reg_buff[25],
            'OnlineTime': (self._reg_buff[31] << 24 | self._reg_buff[30] << 16
                                     | self._reg_buff[29] << 8 | self._reg_buff[28]),
            'FullTime': (self._reg_buff[35] << 24 | self._reg_buff[34] << 16
                                   | self._reg_buff[33] << 8 | self._reg_buff[32]),
            'OneshotTime': (self._reg_buff[39] << 24 | self._reg_buff[38] << 16
                                      | self._reg_buff[37] << 8 | self._reg_buff[36]),
            'Version': self._reg_buff[41] << 8 | self._reg_buff[40],
            'UID0': "%08X" % (self._reg_buff[243] << 24 | self._reg_buff[242] << 16
                                        | self._reg_buff[241] << 8 | self._reg_buff[240]),
            'UID1': "%08X" % (self._reg_buff[247] << 24 | self._reg_buff[246] << 16
                                        | self._reg_buff[245] << 8 | self._reg_buff[244]),
            'UID2': "%08X" % (self._reg_buff[251] << 24 | self._reg_buff[250] << 16
                                        | self._reg_buff[249] << 8 | self._reg_buff[248]),
        }
#        print(tel_data)
        try:
            r = requests.post(UPSPLUS_IOT_PLATFORM_URL, data=tel_data)
            # print(r.text)
            # json data will be formatted with double quotes, so replace them
            self._send_status_data_reply = r.text.replace('"', "'")
            send_status_data_warncount = 0
        except Exception as e:
            # print("sending UPS status data failed: " + str(e))
            warncount = 10
            self._send_status_data_reply = "sending UPS status data failed: " + str(e)
            if send_status_data_warncount == warncount:
                errmsg = f"sending UPS status data failed {warncount} times: " + str(e)
                syslog.syslog(syslog.LOG_ERR, errmsg)
                print(errmsg)
            if send_status_data_warncount <= warncount:
                send_status_data_warncount += 1


class UPSVoltageCurrent:

    def __init__(self):
        self._batt_current = deque([])
        self._batt_power = deque([])
        self._out_current = deque([])
        self._out_power = deque([])
        self._min_RPi_voltage = 6
        self._max_out_current = 0
        self._can_warn = 0
        self._arrLength = 2 * BATT_LOOP_TIME

    def add_value(self):
        had_warning = False
        had_exception = False
        # get measurements and handle i2c bus exceptions
        try:
            battcurr = -inaBattery.current()
            battvolt = inaBattery.voltage()
        except Exception as exc:
            had_exception = True
            if self._can_warn == 0:
                syslog.syslog(syslog.LOG_ERR, "[C_UpsCurrent.add_value] Error reading inaBatt registers: " + str(exc))
                had_warning = True
        try:
            outcurr = inaRPi.current()
            outvolt = inaRPi.voltage()
        except Exception as exc:
            had_exception = True
            if self._can_warn == 0:
                syslog.syslog(syslog.LOG_ERR, "[C_UpsCurrent.add_value] Error reading inaRPi registers: " + str(exc))
                had_warning = True
        # in case we get repeated errors on the i2c bus, we make sure we do not get a warning about this every second
        if had_warning:
            self._can_warn = 60 # send a warning once a minute at most
        else:
            if self._can_warn > 0:
                self._can_warn -= 1
        if had_exception:
            return
        # put measures into arrays
        if len(self._batt_current) >= self._arrLength:
            self._batt_current.popleft()
        self._batt_current.append(battcurr)
        if len(self._batt_power) >= self._arrLength:
            self._batt_power.popleft()
        self._batt_power.append(battcurr * battvolt) # we do not use the ina.power() function as it always returns positive
        if len(self._out_current) >= self._arrLength:
            self._out_current.popleft()
        self._out_current.append(outcurr)
        if self._max_out_current < outcurr:
            self._max_out_current = outcurr
        if len(self._out_power) >= self._arrLength:
            self._out_power.popleft()
        self._out_power.append(outcurr * outvolt)
        if outvolt < self._min_RPi_voltage:
            self._min_RPi_voltage = outvolt

    @property
    def battery_current_avg_mA(self):
        if len(self._batt_current) > 0:
            return sum(self._batt_current) / len(self._batt_current)
        else:
            return 0

    @property
    def battery_power_avg_mW(self):
        if len(self._batt_power) > 0:
            return sum(self._batt_power) / len(self._batt_power)
        else:
            return 0

    @property
    def out_current_avg_mA(self):
        if len(self._out_current) > 0:
            return sum(self._out_current) / len(self._out_current)
        else:
            return 0

    @property
    def out_power_avg_mW(self):
        if len(self._out_power) > 0:
            return sum(self._out_power) / len(self._out_power)
        else:
            return 0

    @property
    # Getting the value will reset max value !!! 
    def out_current_peak_mA(self):
        tmpMax = self._max_out_current
        self._max_out_current = 0
        return tmpMax

    @property
    # Getting the value will reset max value !!! 
    def out_voltage_mini_V(self):
        tmpMin = self._min_RPi_voltage
        self._min_RPi_voltage = 6
        return tmpMin


class UPSHealth:
# the UPS is considered unhealthy if...
# - more than 9 consecutive battery temperature readings are equal.
#   this occurred frequently with firmware versions <= 9) but has not been observed so far for v.10.
#   when this occurred, the UPS would not perform the shutdown procedure at the end of battery life, and would not switch off power to the RPi.
# - at least 3 consecutive battery temperature readings are over threshold
# - charging circuit works properly: the battery is not discharging when not on battery
    def __init__(self, high_batt_temp):
        self._lastTemp = -1
        self._UNHEALTHY_EQUAL_BATT_TEMP_VALUES_COUNT = 20
        self._equalValuesFound = 0
        self._HIGH_BATT_TEMP = high_batt_temp
        self._highBattTempValuesFound = 0
        self._HIGH_BATT_TEMP_MAXCOUNT = 3
        self._charger_works_properly = True
        self._batt_current = deque([])
        self._arrLength = 5
        self._ignore_next_batt_current = False

    def check_batt_temperature(self, batt_temp):
        if batt_temp == self._lastTemp:
            if self._equalValuesFound <= self._UNHEALTHY_EQUAL_BATT_TEMP_VALUES_COUNT:
                self._equalValuesFound += 1
        else:
            self._lastTemp = batt_temp
            self._equalValuesFound = 0
        if batt_temp > self._HIGH_BATT_TEMP:
            if self._highBattTempValuesFound <= self._HIGH_BATT_TEMP_MAXCOUNT:
                self._highBattTempValuesFound += 1
        else:
            self._highBattTempValuesFound = 0
    
    def check_charger(self, ups_on_battery, avg_batt_current_mA):
        # do not try to detect charger malfunction when UPS is on battery
        # do not take first reading after on_battery disappears, because avg current may still be positive at first
        if ups_on_battery or self._ignore_next_batt_current:
            if len(self._batt_current) > 0:
                self._batt_current.clear()
            self._ignore_next_batt_current = False
        else:
            if len(self._batt_current) >= self._arrLength:
                self._batt_current.popleft()
            self._batt_current.append(avg_batt_current_mA)
        if ups_on_battery:
            self._ignore_next_batt_current = True
        # we make another average of avg batt current readings to get rid of current spikes
        # due to battery voltage probings
        avg_avg_batt_current_mA = 0
        if len(self._batt_current) == self._arrLength:
            avg_avg_batt_current_mA = sum(self._batt_current) / len(self._batt_current)
        self._charger_works_properly = not (not ups_on_battery and (avg_avg_batt_current_mA > 100 ))
#        print(f"avg batt current: {avg_batt_current_mA}, avg avg batt current: {avg_avg_batt_current_mA}")
        # we take 0.1A as a limit to stay clear of battery discharge current due to battery voltage probing

    @property
    def is_healthy(self):
        errMsg = ""
        if not (self._equalValuesFound < self._UNHEALTHY_EQUAL_BATT_TEMP_VALUES_COUNT):
            errMsg = "[UPS health] Got too many identical battery temperature values"
        if not (self._highBattTempValuesFound < self._HIGH_BATT_TEMP_MAXCOUNT):
            errMsg = "[UPS health] Battery temperature seems too high"
        if not self._charger_works_properly:
            errMsg = "[UPS health] Charger connected, but battery is discharging"
        if len(errMsg) > 0:
            print(errMsg)
            syslog.syslog(syslog.LOG_ERR, errMsg)

        return (self._equalValuesFound < self._UNHEALTHY_EQUAL_BATT_TEMP_VALUES_COUNT) \
            and (self._highBattTempValuesFound < self._HIGH_BATT_TEMP_MAXCOUNT) \
            and self._charger_works_properly


class Fan:
    def __init__(self, mqttclient):
        self._currentfanspeed = 999
        self._speed = 100
        self._fansum = 0
        self._pTemp = 15
        self._iTemp = 0.4
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(GPIO_FAN, GPIO.OUT)
        self._myPWM = GPIO.PWM(GPIO_FAN, 50)
        self._myPWM.start(50)
        self._mqttclient = mqttclient
        self.switch_off()

    def set_speed(self):
        def get_CPU_temperature():
            res = os.popen('vcgencmd measure_temp').readline()
            temp =(res.replace("temp=","").replace("'C\n",""))
            # print("temp is {0}".format(temp)) #Uncomment here for testing
            return temp
        try:
            actualTemp = float(get_CPU_temperature())
        except Exception as exc:
            print("[get_cpu_temperature] exception: " + str(exc))
            return
        diff = actualTemp - DESIRED_CPU_TEMP
        self._fansum = self._fansum + diff
        pDiff = diff * self._pTemp
        iDiff = self._fansum * self._iTemp
        self._speed = pDiff + iDiff
        if self._speed > 100:
            self._speed = 100
        if self._speed < 15:
            self._speed = 0
        if self._fansum > 100:
            self._fansum = 100
        if self._fansum < -100:
            self._fansum = -100
        self._myPWM.ChangeDutyCycle(self._speed)
        self._publish_speed()

    def switch_off(self):
        self._myPWM.ChangeDutyCycle(0)   # switch fan off
        self._speed = 0
        self._publish_speed()
        return

    def _publish_speed(self):
        if self._mqttclient.connected_flag and (self._speed != self._currentfanspeed):
            try:
                res = self._mqttclient.publish(MQTT_TOPIC + MQTT_TOPIC_FAN, str(int(self._speed)), 0, True)
                if res[0] == 0:
                    self._currentfanspeed = self._speed
            except Exception as e:
                pass
        return

    def cleanup(self):
        self.switch_off()
        GPIO.cleanup() # resets all GPIO ports used by this program

had_upsplus_exception = False

def get_UPS_status_and_check_battery_voltage(mqttclient):
    global UPS_was_on_battery, had_upsplus_exception, one_hour_delay
    try:
        upsplus = UPSPlus(UPS_voltage_current, UPS_health_check)
        had_upsplus_exception = False
    except Exception as exc:
        # only log error if 2 or more exceptions without interruption
        if had_upsplus_exception:
            errMsg = "[get_UPS_status_and_check_battery_voltage] Error getting data from UPS: " + str(exc)
            print(errMsg)
            syslog.syslog(syslog.LOG_ERR, errMsg)
        else:
            had_upsplus_exception = True
        return
    if SEND_STATUS_TO_UPSPLUS_IOT_PLATFORM:
        upsplus.send_UPS_status_data()
    try:
        if mqttclient.connected_flag:
            upsplus.MQTT_publish(mqttclient)
    except Exception as e:
        print("mqttpublish exception: " + str(e))
        pass
 
    if upsplus.on_battery:
        UPS_was_on_battery = True
    else:
        if UPS_was_on_battery:
            syslog.syslog(syslog.LOG_INFO, 'UPS back on AC supply.')
            UPS_was_on_battery = False

    if upsplus.on_battery or (one_hour_delay == 0):
        shutdown_at_battvoltage_V = (upsplus.protection_voltage_mV + PROTECTION_VOLTAGE_MARGIN_mV) / 1000
        if upsplus.on_battery:
            syslog.syslog(syslog.LOG_WARNING, 
                      f"UPS on battery. Battery voltage: {upsplus.battery_voltage_V:.3f} V. "
                      f"Shutdown at {shutdown_at_battvoltage_V:.3f} V")
        if upsplus._battery_voltage_V > 1: # protect against bad battery voltage reading
            if upsplus._battery_voltage_V < shutdown_at_battvoltage_V :
                syslog.syslog(syslog.LOG_WARNING,
                              f"UPS battery voltage below threshold of {shutdown_at_battvoltage_V:.3f} V. Shutting down.")
                if not upsplus.on_battery:
                    syslog.syslog(syslog.LOG_ERR,
                              f"Low battery voltage may be caused by faulty charging circuit. Charger voltage present at USB input.")
                shut_down_RPi(mqttclient)
            else:
                if SHUTDOWN_IMMEDIATELY_WHEN_ON_BATTERY and upsplus.on_battery:
                    syslog.syslog(syslog.LOG_INFO, 'Immediate shutdown.')
                    shut_down_RPi(mqttclient)
    return


def shut_down_RPi(mqttclient):  
    if control_fan:
       fan.switch_off()
    MQTT_terminate(mqttclient)
    # initialize shutdown sequence.
    try:
    	# enable switch on when back on AC 
        i2c_bus.write_byte_data(DEVICE_ADDR, 0x19, 1)
    except Exception as exc:
        syslog.syslog(syslog.LOG_ERR, "[shut_down_RPi] Error writing UPS register (back to AC power up): " + str(exc))
    try:
        i2c_bus.write_byte_data(DEVICE_ADDR, 0x18, SHUTDOWN_TIMEOUT)
    except Exception as exc:
        syslog.syslog(syslog.LOG_ERR, "[shut_down_RPi] Error writing UPS register (shutdown timeout): " + str(exc))
    time.sleep(1)
#    os.system("sudo shutdown -h 1")
    os.system("sudo sync && sudo halt")
    while True:
        time.sleep(100)    


def UPS_is_present():
    """
    check if i2c bus is usable. check if we can read UPS registers.
    initialize i2c bus and both INA sensors
    returns False if we get an exception in any of these operations
    """
    global i2c_bus
    global inaRPi
    global inaBattery

    #   init i2c protocol
    try:
        i2c_bus = smbus2.SMBus(DEVICE_BUS)
    except Exception as e:
        errMsg = 'i2c bus for communication with UPS could not be initialized. Error message: ' + str(e)
        syslog.syslog(syslog.LOG_WARNING, errMsg)
        print(errMsg)
        return False
    # check if we can read UPS registers on the i2c bus
    try:
        void = i2c_bus.read_byte_data(DEVICE_ADDR, 0x12)
    except OSError as e:
        errMsg = 'No reply from UPS on i2c bus. Error message: ' + str(e)
        syslog.syslog(syslog.LOG_WARNING, errMsg)
        print(errMsg)
        return False
    # Raspberry Pi output current and voltage
    try:
        inaRPi = INA219(INA219SHUNT_OUT, busnum=DEVICE_BUS, address=0x40)
        inaRPi.configure(inaRPi.RANGE_32V, inaRPi.GAIN_AUTO, inaRPi.ADC_12BIT, inaRPi.ADC_12BIT)
    except Exception as exc:
        errMsg = 'Cannot initialize communication with INA219 (output). Error message: ' + str(exc)
        syslog.syslog(syslog.LOG_WARNING, errMsg)
        print(errMsg)
        return False
    # Battery current and voltage
    try:
        inaBattery = INA219(INA219SHUNT_BATT, busnum=DEVICE_BUS, address=0x45)
        inaBattery.configure(inaBattery.RANGE_32V, inaBattery.GAIN_AUTO, inaBattery.ADC_12BIT, inaBattery.ADC_12BIT)
    except Exception as exc:
        errMsg = 'Cannot initialize communication with INA219 (battery). Error message: ' + str(exc)
        syslog.syslog(syslog.LOG_WARNING, errMsg)
        print(errMsg)
        return False
    return True


send_status_data_warncount = 0
try:
    print(f"UPS-Plus to MQTT version {SCRIPT_VERSION}")
    print("Copyright (C) 2021 https://github.com/frtz13")
    print("This program comes with ABSOLUTELY NO WARRANTY")
    print("This is free software, and you are welcome to redistribute it")
    print("under conditions of the GPL (see http://www.gnu.org/licenses for details).")
    print()

    if not read_config():
        print("Please check configuration file and parameters")
        syslog.syslog(syslog.LOG_WARNING, "Program stopped. Please check configuration file and parameters.")
        exit()

    print("Type ctrl-C to exit")
    syslog.syslog(syslog.LOG_INFO, f"Version {SCRIPT_VERSION} running...")

#   init MQTT connection
    connect_to_MQTT = MQTT_BROKER != ""
    mqtt.Client.connected_flag = False # create flags in class
    mqtt.Client.connection_rc = -1
    MQTT_client = mqtt.Client("fanShutDownUps")

    control_fan = (GPIO_FAN >= 0)
    if control_fan:
        try:
            fan = Fan(MQTT_client)
        except Exception as exc:
            errMsg = 'Fan class initialization failed. Error message: ' + str(exc)
            syslog.syslog(syslog.LOG_ERR, errMsg)
            print(errMsg)
            control_fan = False
    
    UPS_present = UPS_is_present()
    if UPS_present:
        UPS_voltage_current = UPSVoltageCurrent()
        UPS_health_check = UPSHealth(HIGH_BATT_TEMP)

    UPS_was_on_battery = False
    one_hour_delay = 3600

     # set negative value (-300 + BATT_LOOP_TIME) to force long waiting at startup.
     # so the RPi will be running for some minimum time if power fails again.
     # also, script won't shut down the RPi before elapse of this time, so we have a chance to kill the script should anything malfunction.
     # also allows to wait for the MQTT broker to start if it is running on the RPi, too.
    batterycheck_timer = TIMER_BIAS_AT_STARTUP
    
    fan_timer = 0
    MQTT_connection_loop_running = False
    if connect_to_MQTT:
      MQTT_connection_loop_running = MQTT_connect(MQTT_client)
 
    while True:
        if UPS_present:
            UPS_voltage_current.add_value()
        if control_fan:
            if fan_timer >= FAN_LOOP_TIME:
                fan_timer =  0
                fan.set_speed()
            else:
                fan_timer += 1
        if batterycheck_timer >= BATT_LOOP_TIME:
            batterycheck_timer = 0
            if connect_to_MQTT and not MQTT_connection_loop_running:
                MQTT_connection_loop_running = MQTT_connect(MQTT_client)
            if control_fan:
                read_config_desired_cpu_temp()
            if UPS_present:
                get_UPS_status_and_check_battery_voltage(MQTT_client)
        else:
            batterycheck_timer += 1
        if one_hour_delay > 0:
            one_hour_delay -= 1
        sleep(1)

except KeyboardInterrupt: # trap a CTRL+C keyboard interrupt 
    if control_fan:
        fan.cleanup()
    MQTT_terminate(MQTT_client)
    print()
    syslog.syslog(syslog.LOG_INFO, "Stopped")

