
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
import syslog
import signal
import sys
import RPi.GPIO as GPIO
import smbus2
from ina219 import INA219,DeviceRangeError
import paho.mqtt.client as mqtt
import json
import configparser
import requests
import random
from collections import deque

CONFIG_FILE = "fanShutDownUps.ini"
CONFIGSECTION_FAN = "fan"
CONFIGSECTION_MQTT = "mqtt"
CONFIGSECTION_UPS = "ups"

DEVICE_BUS = 1 # Define I2C bus
DEVICE_ADDR = 0x17 # Define device i2c slave address.
# threshold of UPS automatic power-off default value. the program tries to use the registered protection voltage
RESTART_TIMEOUT = 0

MQTT_TOPIC_FAN = "/fanspeed"
MQTT_TOPIC_UPS = "/ups"
MQTT_TOPIC_LWT = "/LWT"
MQTT_PAYLOAD_ONLINE = "online"
MQTT_PAYLOAD_OFFLINE = "offline"

def ReadConfig():
    global GPIO_FAN # The GPIO pin ID to control the fan
    global FAN_LOOP_TIME
    global DESIRED_CPU_TEMP
    global FANSPEED_FILENAME
    global SEND_STATUS_TO_UPSPLUS_IOT_PLATFORM
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
        DESIRED_CPU_TEMP = int(confparser.get(CONFIGSECTION_FAN, "DESIRED_CPU_TEMP_degC"))

        SEND_STATUS_TO_UPSPLUS_IOT_PLATFORM = 1 == int(confparser.get(CONFIGSECTION_UPS, "SEND_STATUS_TO_UPSPLUS_IOT_PLATFORM"))
        BATT_LOOP_TIME = int(confparser.get(CONFIGSECTION_UPS, "BATTERY_CHECK_LOOP_TIME_s"))
        try:
            TIMER_BIAS_AT_STARTUP = -int(confparser.get(CONFIGSECTION_UPS, "TIMER_BIAS_AT_STARTUP"))
        except:
            TIMER_BIAS_AT_STARTUP = -300 + BATT_LOOP_TIME
        try:
            SHUTDOWN_IMMEDIATELY_WHEN_ON_BATTERY = (1 == int(confparser.get(CONFIGSECTION_UPS, "SHUTDOWN_IMMEDIATELY_WHEN_ON_BATTERY")))
        except:
            SHUTDOWN_IMMEDIATELY_WHEN_ON_BATTERY = False
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
        print("Error when reading configuration parameters: " + str(e))
        syslog.syslog("Error when reading configuration parameters: " + str(e))
        return False

def on_mqttconnect(client, userdata, flags, rc):
    client.connection_rc = rc
    if rc == 0:
        client.connected_flag = True #set flag
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
#        print("Bad connection. " + errMsg.get(rc, "Unknown error: " + str(rc)) + ".")

def on_mqttdisconnect(client, userdata, rc):
#    print("disconnecting reason  "  + str(rc))
    client.connected_flag = False

def MQTT_Connect(client):
    client.on_connect = on_mqttconnect
    client.on_disconnect = on_mqttdisconnect
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
#        print("In wait loop")
        if time.time() > timeout:
            break
        time.sleep(1)
    if client.connected_flag:
#        print("in Main Loop")
        return True
    else:
        return False

def MQTT_Publish(client, ups):
#   make payload
        dictPayload = {}
        dictPayload['UsbC_V'] = '{:.3f}'.format(ups.UsbC_mV / 1000)
        dictPayload['UsbMicro_V'] = '{:.3f}'.format(ups.UsbMicro_mV / 1000)
        dictPayload['OnBattery'] = ups.P_OnBattery
        dictPayload['BatteryVoltage_V'] = ups.BatteryVoltage_V
        dictPayload['BatteryCurrent_mA'] = int(ups.BatteryCurrent_mA)
        dictPayload['BatteryCurrent_avg_mA'] = int(ups.BatteryCurrent_avg_mA)
        dictPayload['BatteryPower_avg_mW'] = int(ups.BatteryPower_avg_mW)
        dictPayload['BatteryCharging'] = ups.P_BatteryIsCharging
        dictPayload['BatteryRemainingCapacity_percent'] = ups.BatteryRemainingCapacity_percent
        dictPayload['BatteryTemperature_degC'] = ups.BatteryTemperature_degC
        dictPayload['OutputVoltage_V'] = ups.RPiVoltage_V
        dictPayload['OutputVoltage_mini_V'] = ups.RPiVoltage_mini_V
        dictPayload['OutputCurrent_mA'] = int(ups.RPiCurrent_mA)
        dictPayload['OutputCurrent_avg_mA'] = int(ups.RPiCurrent_avg_mA)
        dictPayload['OutputPower_avg_mW'] = int(ups.RPiPower_avg_mW)
        dictPayload['OutputCurrent_peak_mA'] = int(ups.RPiCurrent_peak_mA)
        payload = json.dumps(dictPayload)
        try:
            res = client.publish(MQTT_TOPIC + MQTT_TOPIC_UPS, payload, 0, False)
        except:
            pass
#        if res[0] == 0:
#            print("publish successful: " + payload)
#        else:
#            print("public failed. result: " + str(res.result))

def MQTT_Terminate(client):
    try:
        if client.connected_flag:
            res = mqttclient.publish(MQTT_TOPIC + MQTT_TOPIC_LWT, MQTT_PAYLOAD_OFFLINE, 0, retain=True)
#            if res[0] == 0:
#                print("mqtt go offline ok")
            mqttclient.disconnect()
        sleep(1)
        client.loop_stop()
    except Exception as e:
        print("program exit exception: " + str(e))
        pass


class C_UPSPlus:
    def __init__(self, upsCurrent):
        # get battery status
        try:
            self.BatteryVoltage_V = inaBattery.voltage()
            try:
                self.BatteryCurrent_mA = -inaBattery.current() # positive: discharge current
            except DeviceRangeError:
                self.BatteryCurrent_mA = 16000
        except Exception as exc:
            raise Exception("[C_UPSPLus.init] Error reading inaBatt registers: " + str(exc))
        self.BatteryCurrent_avg_mA = upsCurrent.PbattCurrent_avg_mA
        self.BatteryPower_avg_mW = upsCurrent.PbattPower_avg_mW
        # get output status
        try:
            self.RPiVoltage_V = inaRPi.voltage()
            try:
                self.RPiCurrent_mA = inaRPi.current()
            except DeviceRangeError:
                self.RPiCurrent_mA = 16000
        except Exception as exc:
            raise Exception("[C_UPSPLus.init] Error reading inaRPi registers: " + str(exc))
        self.RPiCurrent_avg_mA = upsCurrent.PoutCurrent_avg_mA
        self.RPiPower_avg_mW = upsCurrent.PoutPower_avg_mW
        self.RPiCurrent_peak_mA = upsCurrent.POutCurrent_peak_mA
        # print("avg battery current: {:.0f} mA, avg output current: {:.0f} mA".format(self.BatteryCurent_avg_mA, self.RPiCurrent_avg_mA))
        # print("avg battery power: {:.0f} mW, avg output power {:.0f} mW".format(self.BatteryPower_avg_mW, self.RPiPower_avg_mW))
        self.RPiVoltage_mini_V = upsCurrent.POutVoltage_mini_V

        # get UPS register contents
        self.aReceiveBuf = []
        self.aReceiveBuf.append(0x00)
        try:
            for i in range(1,255):
                self.aReceiveBuf.append(i2c_bus.read_byte_data(DEVICE_ADDR, i))
        except Exception as exc:
            raise Exception("[C_UPSPLus.init] Error reading UPS registers: " + str(exc))

        self.UsbC_mV = self.aReceiveBuf[8] << 8 | self.aReceiveBuf[7]
        self.UsbMicro_mV = self.aReceiveBuf[10] << 8 | self.aReceiveBuf[9]
        self.BatteryTemperature_degC = self.aReceiveBuf[12] << 8 | self.aReceiveBuf[11]
#        self.BatteryFullVoltage_mV = self.aReceiveBuf[14] << 8 | self.aReceiveBuf[13]
#        self.BatteryEmptyVoltage_mV = self.aReceiveBuf[16] << 8 | self.aReceiveBuf[15]
#        self.BatteryProtectionVoltage_mV = self.aReceiveBuf[18] << 8 | self.aReceiveBuf[17] # not used here
        self.BatteryRemainingCapacity_percent = self.aReceiveBuf[20] << 8 | self.aReceiveBuf[19]
#        self.AccumulatedRunTime_s = self.aReceiveBuf[31] << 24 | self.aReceiveBuf[30] << 16 | self.aReceiveBuf[29] << 8 | self.aReceiveBuf[28]
#        self.AccumulatedChargingTime_s = self.aReceiveBuf[35] << 24 | self.aReceiveBuf[34] << 16 | self.aReceiveBuf[33] << 8 | self.aReceiveBuf[32]
#        self.CurrentRunTime_s = self.aReceiveBuf[39] << 24 | self.aReceiveBuf[38] << 16 | self.aReceiveBuf[37] << 8 | self.aReceiveBuf[36]
#        self.FirmwareVersion = self.aReceiveBuf[41] << 8 | self.aReceiveBuf[40]

    @property
    def P_OnBattery(self):
        # with the current firmware (v. 7) it is more reliable to check discharging current.
        # USB-C and micro-USB voltages are sometimes not correctly reported
        return self.BatteryCurrent_avg_mA > 500 # positive current means battery is discharging
        # return (self.UsbC_mV < 4000) and (self.UsbMicro_mV < 4000)

    @property
    def P_BatteryIsCharging(self):
        return self.BatteryCurrent_avg_mA < 0

    def sendUpsStatusData(self):
        FEED_URL = "https://api.thekoziolfoundation.com/feed"
        # time.sleep(random.randint(0, 3))

        DATA = dict()
        DATA['PiVccVolt'] = self.RPiVoltage_V
        DATA['PiIddAmps'] = self.RPiCurrent_mA

        DATA['BatVccVolt'] = self.BatteryVoltage_V
        DATA['BatIddAmps'] = -self.BatteryCurrent_mA

        DATA['McuVccVolt'] = self.aReceiveBuf[2] << 8 | self.aReceiveBuf[1]
        DATA['BatPinCVolt'] = self.aReceiveBuf[6] << 8 | self.aReceiveBuf[5]
        DATA['ChargeTypeCVolt'] = self.aReceiveBuf[8] << 8 | self.aReceiveBuf[7]
        DATA['ChargeMicroVolt'] = self.aReceiveBuf[10] << 8 | self.aReceiveBuf[9]

        DATA['BatTemperature'] = self.aReceiveBuf[12] << 8 | self.aReceiveBuf[11]
        DATA['BatFullVolt'] = self.aReceiveBuf[14] << 8 | self.aReceiveBuf[13]
        DATA['BatEmptyVolt'] = self.aReceiveBuf[16] << 8 | self.aReceiveBuf[15]
        DATA['BatProtectVolt'] = self.aReceiveBuf[18] << 8 | self.aReceiveBuf[17]
        DATA['SampleTime'] = self.aReceiveBuf[22] << 8 | self.aReceiveBuf[21]
        DATA['AutoPowerOn'] = self.aReceiveBuf[25]

        DATA['OnlineTime'] = self.aReceiveBuf[31] << 24 | self.aReceiveBuf[30] << 16 | self.aReceiveBuf[29] << 8 | self.aReceiveBuf[28]
        DATA['FullTime'] = self.aReceiveBuf[35] << 24 | self.aReceiveBuf[34] << 16 | self.aReceiveBuf[33] << 8 | self.aReceiveBuf[32]
        DATA['OneshotTime'] = self.aReceiveBuf[39] << 24 | self.aReceiveBuf[38] << 16 | self.aReceiveBuf[37] << 8 | self.aReceiveBuf[36]
        DATA['Version'] = self.aReceiveBuf[41] << 8 | self.aReceiveBuf[40]

        DATA['UID0'] = "%08X" % (self.aReceiveBuf[243] << 24 | self.aReceiveBuf[242] << 16 | self.aReceiveBuf[241] << 8 | self.aReceiveBuf[240])
        DATA['UID1'] = "%08X" % (self.aReceiveBuf[247] << 24 | self.aReceiveBuf[246] << 16 | self.aReceiveBuf[245] << 8 | self.aReceiveBuf[244])
        DATA['UID2'] = "%08X" % (self.aReceiveBuf[251] << 24 | self.aReceiveBuf[250] << 16 | self.aReceiveBuf[249] << 8 | self.aReceiveBuf[248])

#        print(DATA)
        try:
            r = requests.post(FEED_URL, data=DATA)
#            print(r.text)
        except Exception as e:
#            print("sending UPS status data failed: " + str(e))
            pass

class C_UpsCurrent:
    def __init__(self):
        self.battCurrent = deque([0 for i in range(2 * BATT_LOOP_TIME)]) # we take more samples to flatten out battery sampling of the UPS
        # directly taking the sample period does not seem to make much sense either because it does not precisely correspond to the configured number of minutes
        self.battPower = deque([0 for i in range(BATT_LOOP_TIME)]) # we take more samples to flatten out battery sampling of the UPS
        self.outCurrent = deque([0 for i in range(BATT_LOOP_TIME)])
        self.outPower = deque([0 for i in range(BATT_LOOP_TIME)])
        self.minRpiVoltage = 6
        self.maxOutCurrent = 0
        self.canWarn = 0

    def addValue(self):
        fHadWarning = False
        fHadException = False
        try:
            battCurr = -inaBattery.current()
            battPow = inaBattery.power()
        except Exception as exc:
            fHadException = True
            if self.canWarn == 0:
                syslog.syslog("[C_UpsCurrent.addValue] Error reading inaBatt registers: " + str(exc))
                fHadWarning = True
        try:
            outCurr = inaRPi.current()
            outPow = inaRPi.power()
            rpiVolt = inaRPi.voltage()
        except Exception as exc:
            fHadException = True
            if self.canWarn == 0:
                syslog.syslog("[C_UpsCurrent.addValue] Error reading inaRPi registers: " + str(exc))
                fHadWarning = True
        # we make sure we do not get a warning about timeout etc. every second
        if fHadWarning:
            self.canWarn = 60
        else:
            if self.canWarn > 0:
                self.canWarn = self.canWarn - 1
        if fHadException:
            return
        # put measures into arrays
        self.battCurrent.popleft()
        self.battCurrent.append(battCurr)
        self.battPower.popleft()
        self.battPower.append(battPow)
        self.outCurrent.popleft()
        self.outCurrent.append(outCurr)
        if self.maxOutCurrent < outCurr:
            self.maxOutCurrent = outCurr
        # print("battery current: {:.0f} mA, output current: {:.0f} mA".format(battCurr, outCurr))
        self.outPower.popleft()
        self.outPower.append(outPow)
        if rpiVolt < self.minRpiVoltage:
            self.minRpiVoltage = rpiVolt

    @property
    def PbattCurrent_avg_mA(self):
        # print("Batt current: " + str(self.battCurrent))
        return sum(self.battCurrent) / len(self.battCurrent)

    @property
    def PbattPower_avg_mW(self):
        return sum(self.battPower) / len(self.battPower)

    @property
    def PoutCurrent_avg_mA(self):
        # print("out current: " + str(self.outCurrent))
        return sum(self.outCurrent) / len(self.outCurrent)

    @property
    def PoutPower_avg_mW(self):
        return sum(self.outPower) / len(self.outPower)

    @property
    # Getting the value will reset max value !!! 
    def POutCurrent_peak_mA(self):
        tmpMax = self.maxOutCurrent
        self.maxOutCurrent = 0
        return tmpMax

    @property
    def POutVoltage_mini_V(self):
        tmpMin = self.minRpiVoltage
        self.minRpiVoltage = 6
        return tmpMin

def handleUPS(mqttclient):
    global upsWasOnBattery
    try:
        upsPlus = C_UPSPlus(upsCurrent)
    except Exception as exc:
        errMsg = "[handleUPS] Error getting data from UPS: " + str(exc)
        print(errMsg)
        syslog.syslog(errMsg)
        return
#    print('Battery voltage: %.3f V' % upsPlus.BatteryVoltage_V)
    try:
        if mqttclient.connected_flag:
            MQTT_Publish(mqttclient, upsPlus)
    except Exception as e:
        print("mqttpublish exception: " + str(e))
        pass
    if SEND_STATUS_TO_UPSPLUS_IOT_PLATFORM:
        upsPlus.sendUpsStatusData()
    if upsPlus.P_OnBattery:
        upsWasOnBattery = True
        syslog.syslog('UPS on battery. Battery voltage: {:.3f} V. Shutdown at {:.3f} V'.format(upsPlus.BatteryVoltage_V, (protectionVoltage_mV + PROTECTION_VOLTAGE_MARGIN_mV) / 1000))
        if upsPlus.BatteryVoltage_V > 1: # protect against bad battery voltage reading
            if (upsPlus.BatteryVoltage_V * 1000) < protectionVoltage_mV + PROTECTION_VOLTAGE_MARGIN_mV :
                syslog.syslog('UPS battery voltage below threshold of %.3f V. Shutting down.' % ((protectionVoltage_mV + PROTECTION_VOLTAGE_MARGIN_mV) / 1000))
                Shutdown(mqttclient)
            else:
                if SHUTDOWN_IMMEDIATELY_WHEN_ON_BATTERY :
                    syslog.syslog('Immediate shutdown.')
                    Shutdown(mqttclient)
    else:
        if upsWasOnBattery:
            syslog.syslog('UPS back on AC supply.')
            upsWasOnBattery = False
    return()


def Shutdown(mqttclient):  
    if controlFan:
       fanOFF(mqttclient)
    MQTT_Terminate(mqttclient)
    # initialized shutdown sequence.
    i2c_bus.write_byte_data(DEVICE_ADDR, 24, SHUTDOWN_TIMEOUT)
	# enable switch on when back on AC 
    i2c_bus.write_byte_data(DEVICE_ADDR, 25, 1)
    time.sleep(1)
#    os.system("sudo shutdown -h 1")
    os.system("sudo sync && sudo halt")
    while True:
        time.sleep(100)    


def getCPUtemperature():
    res = os.popen('vcgencmd measure_temp').readline()
    temp =(res.replace("temp=","").replace("'C\n",""))
    # print("temp is {0}".format(temp)) #Uncomment here for testing
    return temp

def fanOFF(mqttclient):
    myPWM.ChangeDutyCycle(0)   # switch fan off
    writefanspeed(0, mqttclient)
    return()

def writefanspeed(fanSpeed, mqttclient):
    global currentfanspeed
    if mqttclient.connected_flag and (fanSpeed != currentfanspeed):
        try:
            res = mqttclient.publish(MQTT_TOPIC + MQTT_TOPIC_FAN, str(int(fanSpeed)), 0, True)
            if res[0] == 0:
                currentfanspeed = fanSpeed
        except Exception as e:
            pass
    return()

def handleFan(mqttclient):
    global fanSpeed,fansum
    try:
        actualTemp = float(getCPUtemperature())
    except Exception as exc:
        print("getCpuTemperature exception: " + str(exc))
        return
    diff=actualTemp-DESIRED_CPU_TEMP
    fansum=fansum+diff
    pDiff=diff*pTemp
    iDiff=fansum*iTemp
    fanSpeed=pDiff +iDiff
    if fanSpeed>100:
        fanSpeed=100
    if fanSpeed<15:
        fanSpeed=0
    if fansum>100:
        fansum=100
    if fansum<-100:
        fansum=-100
#    print("actualTemp %4.2f TempDiff %4.2f pDiff %4.2f iDiff %4.2f fanSpeed %5d" % (actualTemp,diff,pDiff,iDiff,fanSpeed))
    myPWM.ChangeDutyCycle(fanSpeed)
    writefanspeed(fanSpeed, mqttclient)


try:
    print("UPS Plus to MQTT  Copyright (C) 2021  https://github.com/frtz13")
    print("This program comes with ABSOLUTELY NO WARRANTY")
    print("This is free software, and you are welcome to redistribute it")
    print("under conditions of the GPL (see http://www.gnu.org/licenses for details).")
    print("")

    if not ReadConfig():
        print("Please check configuration file and parameters")
        syslog.syslog("Program stopped. Please check configuration file and parameters.")
        exit()

    print("Type ctrl-C to exit")
    controlFan = (GPIO_FAN >= 0)

#   init MQTT connection
    connectToMQTT = not (MQTT_BROKER == "")
    mqtt.Client.connected_flag = False # create flags in class
    mqtt.Client.connection_rc = -1
    mqttclient = mqtt.Client("UPSPlus")
    mqttConnected = False

#   prepare fan settings
    if controlFan:
        currentfanspeed = 999
        fanSpeed=100
        fansum=0
        pTemp=15
        iTemp=0.4
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(GPIO_FAN, GPIO.OUT)
        myPWM=GPIO.PWM(GPIO_FAN,50)
        myPWM.start(50)
        GPIO.setwarnings(False)
        fanOFF(mqttclient)

#   init i2c protocol
    fUpsPresent = True
    try:
        i2c_bus = smbus2.SMBus(DEVICE_BUS)
    except Exception as e:
        fUpsPresent = False
        errMsg = 'i2c bus for communication with UPS could not be initialized. Error message: ' + str(e)
        syslog.syslog(errMsg)
        print(errMsg)

#   check UPS present
    if fUpsPresent:
        try:
            protectVHi = i2c_bus.read_byte_data(DEVICE_ADDR, 0x12)
            protectionVoltage_mV = protectVHi << 8 | i2c_bus.read_byte_data(DEVICE_ADDR, 0x11)
            if protectionVoltage_mV >= 2500 and protectionVoltage_mV <= 4000:
                pass
            else:
                protectionV_default_mV = 3700
                errMsg = "Protection voltage retrieved from UPS seems to have an incorrect value ({:.0f} mV). Assumed to be {:.0f} mV".format(protectionVoltage_mV, protectionV_default_mV)
                print(errMsg)
                syslog.syslog(errMsg)
                protectionVoltage_mV = protectionV_default_mV

        except OSError as e:
            fUpsPresent = False
            errMsg = 'No reply from UPS on i2c bus. Error message: ' + str(e)
            syslog.syslog(errMsg)
            print(errMsg)

    if fUpsPresent:
    #   Raspberry Pi output current and voltage
        inaRPi = INA219(0.00725, address=0x40)
        inaRPi.configure()
    #   Batteries current and voltage
        inaBattery = INA219(0.005, address=0x45)
        inaBattery.configure()
        upsCurrent = C_UpsCurrent()

    upsWasOnBattery = False
     # set negative value (-300 + BATT_LOOP_TIME) to force long waiting at startup if power fails again
     # also, script won't shut down the RPi before elapse of this time so we have a chance to kill the script should anything malfunction
     # also allows to wait for the MQTT broker to start if running on the RPi
    intBatteryCheckTimer = TIMER_BIAS_AT_STARTUP
    intFanTimer = 0
    while True:
        if fUpsPresent:
            upsCurrent.addValue()
        if controlFan:
            if intFanTimer >= FAN_LOOP_TIME:
                intFanTimer =  0
                handleFan(mqttclient)
            else:
                intFanTimer =  intFanTimer + 1
        if intBatteryCheckTimer >= BATT_LOOP_TIME:
            intBatteryCheckTimer = 0
            if not mqttConnected and connectToMQTT:
                mqttConnected = MQTT_Connect(mqttclient)
            if fUpsPresent:
                handleUPS(mqttclient)
        else:
            intBatteryCheckTimer = intBatteryCheckTimer + 1
        sleep(1)

except KeyboardInterrupt: # trap a CTRL+C keyboard interrupt 
    if controlFan:
        fanOFF(mqttclient)
        GPIO.cleanup() # resets all GPIO ports used by this program
    MQTT_Terminate(mqttclient)

