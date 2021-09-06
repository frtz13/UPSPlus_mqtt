
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

SCRIPT_VERSION = "20210906"

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
    global BATT_LOOP_TIME
    global TIMER_BIAS_AT_STARTUP
    global SHUTDOWN_IMMEDIATELY_WHEN_ON_BATTERY # for testing shutdown only
    global SHUTDOWN_TIMEOUT
    global PROTECTION_VOLTAGE_MARGIN_mV
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

        SEND_STATUS_TO_UPSPLUS_IOT_PLATFORM = 1 == int(confparser.get(CONFIGSECTION_UPS, "SEND_STATUS_TO_UPSPLUS_IOT_PLATFORM"))
        try:
            UPSPLUS_IOT_PLATFORM_URL = confparser.get(CONFIGSECTION_UPS, "UPSPLUS_IOT_PLATFORM_URL")
        except:
            UPSPLUS_IOT_PLATFORM_URL = "https://api.52pi.com/feed"

        BATT_LOOP_TIME = int(confparser.get(CONFIGSECTION_UPS, "BATTERY_CHECK_LOOP_TIME_s"))
        if CMD_NO_TIMER_BIAS in sys.argv:
            TIMER_BIAS_AT_STARTUP = 0
            print("Timer bias at start-up set to 0")
        else:
            try:
                TIMER_BIAS_AT_STARTUP = -int(confparser.get(CONFIGSECTION_UPS, "TIMER_BIAS_AT_STARTUP"))
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
#       print("connected OK")
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
#        print("connection failed: " + str(e))
        return False
    timeout = time.time() + 5
    while client.connection_rc == -1: #wait in loop
        if time.time() > timeout:
            break
        time.sleep(1)
    if client.connected_flag:
        return True
    else:
        return False

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


class UPSPlus:
    def __init__(self, upsCurrent):
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

        # we only get the full set of registers if we report back to the IOT Platform
        # otherwise just get the register values we actually use
        if SEND_STATUS_TO_UPSPLUS_IOT_PLATFORM:
            it_registers = it.chain(range(0x01,0x2A), range(0xF0,0xFC))
        else:
            it_registers = it.chain(range(0x07,0x0C), range(0x11, 0x15))
        try:
            self._reg_buff = {i:i2c_bus.read_byte_data(DEVICE_ADDR, i) for i in it_registers}
        except Exception as exc:
            raise Exception("[UPSPLus.init] Error reading UPS registers: " + str(exc))

        self._USB_C_mV = self._reg_buff[0x08] << 8 | self._reg_buff[0x07]
        self._USB_micro_mV = self._reg_buff[0x0A] << 8 | self._reg_buff[0x09]
#        self.battery_temperature_degC = self.reg_buff[12] << 8 | self.reg_buff[11]
#       we very rarely get 0xFF at reg_buff[0x0C]. this value should be 0 anyway for realistic temperatures
        self._battery_temperature_degC = self._reg_buff[0x0B]
        self._battery_remaining_capacity_percent = self._reg_buff[0x14] << 8 | self._reg_buff[0x13]

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
        # with the current firmware (v. 9) it seems more reliable to check discharging current.
        # USB-C and micro-USB voltages are sometimes not reported properly
        return self._battery_current_avg_mA > 500 # positive current means battery is discharging
        # return (self.UsbC_mV < 4000) and (self.UsbMicro_mV < 4000)

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
                'BatteryCurrent_mA': int(self._battery_current_mA),
                'BatteryCurrent_avg_mA': int(self._battery_current_avg_mA),
                'BatteryPower_avg_mW': int(self._battery_power_avg_mW),
                'BatteryCharging': self.battery_is_charging,
                'BatteryRemainingCapacity_percent': self._battery_remaining_capacity_percent,
                'BatteryTemperature_degC': self._battery_temperature_degC,
                'OutputVoltage_V': self._RPi_voltage_V,
                'OutputVoltage_mini_V': self._RPi_voltage_mini_V,
                'OutputCurrent_mA': int(self._RPi_current_mA),
                'OutputCurrent_avg_mA': int(self._RPi_current_avg_mA),
                'OutputPower_avg_mW': int(self._RPi_power_avg_mW),
                'OutputCurrent_peak_mA': int(self._RPi_current_peak_mA),
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
#        print(DATA)
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


def get_UPS_status_and_check_battery_voltage(mqttclient):
    global UPS_was_on_battery
    try:
        upsplus = UPSPlus(UPS_voltage_current)
    except Exception as exc:
        errMsg = "[get_UPS_status_and_check_battery_voltage] Error getting data from UPS: " + str(exc)
        print(errMsg)
        syslog.syslog(syslog.LOG_ERR, errMsg)
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
        shutdown_at_battvoltage_V = (upsplus.protection_voltage_mV + PROTECTION_VOLTAGE_MARGIN_mV) / 1000
        syslog.syslog(syslog.LOG_WARNING, 
                      f"UPS on battery. Battery voltage: {upsplus.battery_voltage_V:.3f} V. "
                      f"Shutdown at {shutdown_at_battvoltage_V:.3f} V")
        if upsplus._battery_voltage_V > 1: # protect against bad battery voltage reading
            if upsplus._battery_voltage_V < shutdown_at_battvoltage_V :
                syslog.syslog(syslog.LOG_WARNING,
                              f"UPS battery voltage below threshold of {shutdown_at_battvoltage_V:.3f} V. Shutting down.")
                shut_down_RPi(mqttclient)
            else:
                if SHUTDOWN_IMMEDIATELY_WHEN_ON_BATTERY :
                    syslog.syslog(syslog.LOG_INFO, 'Immediate shutdown.')
                    shut_down_RPi(mqttclient)
    else:
        if UPS_was_on_battery:
            syslog.syslog(syslog.LOG_INFO, 'UPS back on AC supply.')
            UPS_was_on_battery = False
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
        inaRPi = INA219(0.00725, busnum=DEVICE_BUS, address=0x40)
        inaRPi.configure()
    except Exception as exc:
        errMsg = 'Cannot initialize communication with INA219 (output). Error message: ' + str(exc)
        syslog.syslog(syslog.LOG_WARNING, errMsg)
        print(errMsg)
        return False
    # Battery current and voltage
    try:
        inaBattery = INA219(0.005, busnum=DEVICE_BUS, address=0x45)
        inaBattery.configure()
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
    MQTT_connected = False

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

    UPS_was_on_battery = False

     # set negative value (-300 + BATT_LOOP_TIME) to force long waiting at startup.
     # so the RPi will be running for some minimum time if power fails again.
     # also, script won't shut down the RPi before elapse of this time, so we have a chance to kill the script should anything malfunction.
     # also allows to wait for the MQTT broker to start if it is running on the RPi, too.
    batterycheck_timer = TIMER_BIAS_AT_STARTUP
    
    fan_timer = 0
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
            if control_fan:
                read_config_desired_cpu_temp()
            if not MQTT_connected and connect_to_MQTT:
                MQTT_connected = MQTT_connect(MQTT_client)
            if UPS_present:
                get_UPS_status_and_check_battery_voltage(MQTT_client)
        else:
            batterycheck_timer += 1
        sleep(1)

except KeyboardInterrupt: # trap a CTRL+C keyboard interrupt 
    if control_fan:
        fan.cleanup()
    MQTT_terminate(MQTT_client)
    print()
    syslog.syslog(syslog.LOG_INFO, "Stopped")

