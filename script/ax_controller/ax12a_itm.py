#!/usr/bin/env python
# coding=UTF-8
'''
Author: Wei Luo
Date: 2022-04-06 14:19:19
LastEditors: Wei Luo
LastEditTime: 2022-04-27 17:34:53
Note: self-design class for handle AX-12A
smart servo motor using Raspberry PI Python2/3

Based on Jesse Merritt's script:
https://github.com/jes1510/python_dynamixels
and Josue Alejandro Savage's Arduino library:
http://savageelectronics.blogspot.it/2011/01/arduino-y-dynamixel-ax-12.html

'''

from serial import Serial
import RPi.GPIO as GPIO
from time import sleep
import time


class DynamixelProtocal1(object):
    def __init__(self, ):
        # EEPROM register 0x00 -> 0x17 (23)
        self.AX_MODEL_NUMBER_L = 0  # 0x0C
        self.AX_MODEL_NUMBER_H = 1  # 0x00
        self.AX_VERSION = 2
        self.AX_ID = 3
        self.AX_BAUD_RATE = 4
        self.AX_RETURN_DELAY_TIME = 5
        self.AX_CW_ANGLE_LIMIT_L = 6
        self.AX_CW_ANGLE_LIMIT_H = 7
        self.AX_CCW_ANGLE_LIMIT_L = 8
        self.AX_CCW_ANGLE_LIMIT_H = 9
        self.AX_SYSTEM_DATA2 = 10
        self.AX_LIMIT_TEMPERATURE = 11
        self.AX_DOWN_LIMIT_VOLTAGE = 12
        self.AX_UP_LIMIT_VOLTAGE = 13
        self.AX_MAX_TORQUE_L = 14
        self.AX_MAX_TORQUE_H = 15
        self.AX_RETURN_LEVEL = 16
        self.AX_ALARM_LED = 17
        self.AX_ALARM_SHUTDOWN = 18
        self.AX_OPERATING_MODE = 19
        self.AX_DOWN_CALIBRATION_L = 20
        self.AX_DOWN_CALIBRATION_H = 21
        self.AX_UP_CALIBRATION_L = 22
        self.AX_UP_CALIBRATION_H = 23

        # RAM area 0x18(18) -> 0x31(49)
        self.AX_TORQUE_STATUS = 24
        self.AX_LED_STATUS = 25
        self.AX_CW_COMPLIANCE_MARGIN = 26
        self.AX_CCW_COMPLIANCE_MARGIN = 27
        self.AX_CW_COMPLIANCE_SLOPE = 28
        self.AX_CCW_COMPLIANCE_SLOPE = 29
        self.AX_GOAL_POSITION_L = 30  # 0x1E
        self.AX_GOAL_POSITION_H = 31  # 0x1F
        self.AX_GOAL_SPEED_L = 32  # 0x20
        self.AX_GOAL_SPEED_H = 33  # 0x21
        self.AX_TORQUE_LIMIT_L = 34
        self.AX_TORQUE_LIMIT_H = 35
        self.AX_PRESENT_POSITION_L = 36
        self.AX_PRESENT_POSITION_H = 37
        self.AX_PRESENT_SPEED_L = 38
        self.AX_PRESENT_SPEED_H = 39
        self.AX_PRESENT_LOAD_L = 40
        self.AX_PRESENT_LOAD_H = 41
        self.AX_PRESENT_VOLTAGE = 42
        self.AX_PRESENT_TEMPERATURE = 43
        self.AX_REGISTERED_INSTRUCTION = 44
        self.AX_PAUSE_TIME = 45
        self.AX_MOVING = 46
        self.AX_LOCK = 47
        self.AX_PUNCH_L = 48
        self.AX_PUNCH_H = 49

        # Error info
        # Error lookup dictionary for bit masking

        self.AX_dictErrors = {
            1: "Input Voltage",
            2: "Angle Limit",
            4: "Overheating",
            8: "Range",
            16: "Checksum",
            32: "Overload",
            64: "Instruction"
        }

        # Instruction
        self.AX_PING = 1
        self.AX_READ_DATA = 2
        self.AX_WRITE_DATA = 3
        self.AX_REG_WRITE = 4
        self.AX_ACTION = 5
        self.AX_RESET = 6
        self.AX_SYNC_WRITE = 131  # 0x83

        # length parameters
        self.AX_RESET_LENGTH = 2
        self.AX_ACTION_LENGTH = 2
        self.AX_ID_LENGTH = 4
        self.AX_LR_LENGTH = 4
        self.AX_SRL_LENGTH = 4
        self.AX_RDT_LENGTH = 4
        self.AX_LEDALARM_LENGTH = 4
        self.AX_SHUTDOWNALARM_LENGTH = 4
        self.AX_TL_LENGTH = 4
        self.AX_VL_LENGTH = 6
        self.AX_AL_LENGTH = 7
        self.AX_CM_LENGTH = 6
        self.AX_CS_LENGTH = 5
        self.AX_COMPLIANCE_LENGTH = 7
        self.AX_CCW_CW_LENGTH = 8
        self.AX_BD_LENGTH = 4
        self.AX_TEM_LENGTH = 4
        self.AX_MOVING_LENGTH = 4
        self.AX_RWS_LENGTH = 4
        self.AX_VOLT_LENGTH = 4
        self.AX_LOAD_LENGTH = 4
        self.AX_LED_LENGTH = 4
        self.AX_TORQUE_LENGTH = 4
        self.AX_POS_LENGTH = 4
        self.AX_GOAL_LENGTH = 5
        self.AX_MT_LENGTH = 5
        self.AX_PUNCH_LENGTH = 5
        self.AX_SPEED_LENGTH = 5
        self.AX_GOAL_SP_LENGTH = 7

        # special parameters
        self.AX_BYTE_READ = 1
        self.AX_INT_READ = 2
        self.AX_ACTION_CHECKSUM = 250
        self.AX_BROADCAST_ID = 254
        self.AX_START = 255
        self.AX_CCW_AL_L = 255
        self.AX_CCW_AL_H = 3
        self.AX_LOCK_VALUE = 1
        self.LEFT = 0
        self.RIGHT = 1
        self.RX_TIME_OUT = 10
        self.TX_DELAY_TIME = 0.005


class AX12AMotorController(DynamixelProtocal1):
    def __init__(
        self,
        port_str=None,
        RPI_Direction_PIN=18,
    ):
        super(AX12AMotorController, self).__init__()
        self.RPI_DIRECTION_TX = GPIO.HIGH
        self.RPI_DIRECTION_RX = GPIO.LOW
        self.RPI_DIRECTION_PIN = RPI_Direction_PIN
        self.RPI_DIRECTION_SWITCH_DELAY = 0.0001

        if port_str is None:
            self.port = Serial("/dev/ttyS0", baudrate=1000000, timeout=0.001)
        else:
            self.port = Serial(port_str, baudrate=1000000, timeout=0.001)

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.RPI_DIRECTION_PIN, GPIO.OUT)

    def direction(self, d):
        GPIO.output(self.RPI_DIRECTION_PIN, d)
        sleep(self.RPI_DIRECTION_SWITCH_DELAY)

    def readErrorData(self, id):
        self.direction(self.RPI_DIRECTION_RX)
        reply = self.port.read(5)  # [0xff, 0xff, origin, length, error]
        try:
            assert ord(reply[0]) == 0xFF
        except:
            print("Timeout on servo " + str(id))
        try:
            length = ord(reply[3]) - 2
            error = ord(reply[4])

            if (error != 0):
                print("Error from servo: " + self.dictErrors[error] +
                      ' (code  ' + hex(error) + ')')
                return -error
            # just reading error bit
            elif (length == 0):
                return error
            else:
                if (length > 1):
                    reply = self.port.read(2)
                    returnValue = (ord(reply[1]) << 8) + (ord(reply[0]) << 0)
                else:
                    reply = self.port.read(1)
                    returnValue = ord(reply[0])
                return returnValue
        except Exception as detail:
            pass

    def movePosition(self, id, position, degree=False, rad=True):
        """
            move to position 0->1023 (value) | 0->300 [degree]
        """
        if degree:
            position = int(position / 300.0 * 1023.0)
        elif rad:
            position = int(position * 180 / 3.1415926 / 300 * 1023.0)

        if position > 1023.0:
            position = 1023.0

        self.direction(self.RPI_DIRECTION_TX)
        self.port.reset_input_buffer()
        p = [position & 0xff, position >> 8]
        checksum = (~(id + self.AX_GOAL_LENGTH + self.AX_WRITE_DATA +
                      self.AX_GOAL_POSITION_L + p[0] + p[1])) & 0xff
        outData = bytearray([self.AX_START])
        outData += bytearray([self.AX_START])
        outData += bytearray([id])
        outData += bytearray([self.AX_GOAL_LENGTH])
        outData += bytearray([self.AX_WRITE_DATA])
        outData += bytearray([self.AX_GOAL_POSITION_L])
        outData += bytearray([p[0]])
        outData += bytearray([p[1]])
        outData += bytearray([checksum])
        self.port.write(outData)
        sleep(self.TX_DELAY_TIME)
        return self.readErrorData(id)

    def movePositionSpeed(self,
                          id,
                          position,
                          speed,
                          degree=False,
                          rpm=False,
                          rad=True,
                          rad_s=True):
        if degree:
            position = int(position / 300.0 * 1023.0)
        elif rad:
            position = int(position * 180 / 3.1415926 / 300 * 1023.0)
        if position > 1023.0:
            position = 1023.0

        if rpm:
            speed = int(speed / 114.0 * 1023.0)
        elif rad_s:
            speed = int(speed / 114.0 * 1023.0 / 2 / 3.1415926 * 60)
        if speed < 1:
            # due to speed ==0 ==> max velocity
            speed = 1
        if speed > 1023.0:
            speed = 1023.0

        self.direction(self.RPI_DIRECTION_TX)
        self.port.reset_input_buffer()
        p = [position & 0xff, position >> 8]
        s = [speed & 0xff, speed >> 8]
        checksum = (
            ~(id + self.AX_GOAL_SP_LENGTH + self.AX_WRITE_DATA +
              self.AX_GOAL_POSITION_L + p[0] + p[1] + s[0] + s[1])) & 0xff
        outData = bytearray([self.AX_START])
        outData += bytearray([self.AX_START])
        outData += bytearray([id])
        outData += bytearray([self.AX_GOAL_SP_LENGTH])
        outData += bytearray([self.AX_WRITE_DATA])
        outData += bytearray([self.AX_GOAL_POSITION_L])
        outData += bytearray([p[0]])
        outData += bytearray([p[1]])
        outData += bytearray([s[0]])
        outData += bytearray([s[1]])
        outData += bytearray([checksum])
        self.port.write(outData)
        sleep(self.TX_DELAY_TIME)
        return self.readErrorData(id)

    def movePositionReg(self, id, position, degree=False, rad=True):
        """
            move to position 0->1023 (value) | 0->300 [degree] and save in the register
        """
        if degree:
            position = int(position / 300 * 1023)
        elif rad:
            position = int(position * 180 / 3.1415926 / 300 * 1023.0)
        if position > 1023.0:
            position = 1023.0

        self.direction(self.RPI_DIRECTION_TX)
        self.port.reset_input_buffer()
        p = [position & 0xff, position >> 8]
        checksum = (~(id + self.AX_GOAL_LENGTH + self.AX_REG_WRITE +
                      self.AX_GOAL_POSITION_L + p[0] + p[1])) & 0xff
        outData = bytearray([self.AX_START])
        outData += bytearray([self.AX_START])
        outData += bytearray([id])
        outData += bytearray([self.AX_GOAL_LENGTH])
        outData += bytearray([self.AX_REG_WRITE])
        outData += bytearray([self.AX_GOAL_POSITION_L])
        outData += bytearray([p[0]])
        outData += bytearray([p[1]])
        outData += bytearray([checksum])
        self.port.write(outData)
        sleep(self.TX_DELAY_TIME)
        return self.readErrorData(id)

    def movePositionSpeedReg(self,
                             id,
                             position,
                             speed,
                             degree=False,
                             rpm=False,
                             rad=True,
                             rad_s=True):
        if degree:
            position = int(position / 300.0 * 1023.0)
        elif rad:
            position = int(position * 180 / 3.1415926 / 300 * 1023.0)
        if position > 1023.0:
            position = 1023.0

        if rpm:
            speed = int(speed / 114.0 * 1023.0)
        elif rad_s:
            speed = int(speed / 114.0 * 1023.0 / 2 / 3.1415926 * 60)
        if speed < 1:
            # due to speed ==0 ==> max velocity
            speed = 1
        if speed > 1023.0:
            speed = 1023.0

        self.direction(self.RPI_DIRECTION_TX)
        self.port.reset_input_buffer()
        p = [position & 0xff, position >> 8]
        s = [speed & 0xff, speed >> 8]
        checksum = (
            ~(id + self.AX_GOAL_SP_LENGTH + self.AX_REG_WRITE +
              self.AX_GOAL_POSITION_L + p[0] + p[1] + s[0] + s[1])) & 0xff
        outData = bytearray([self.AX_START])
        outData += bytearray([self.AX_START])
        outData += bytearray([id])
        outData += bytearray([self.AX_GOAL_SP_LENGTH])
        outData += bytearray([self.AX_REG_WRITE])
        outData += bytearray([self.AX_GOAL_POSITION_L])
        outData += bytearray([p[0]])
        outData += bytearray([p[1]])
        outData += bytearray([s[0]])
        outData += bytearray([s[1]])
        outData += bytearray([checksum])
        self.port.write(outData)
        sleep(self.TX_DELAY_TIME)
        return self.readErrorData(id)

    def action(self):
        self.direction(self.RPI_DIRECTION_TX)
        self.port.reset_input_buffer()
        outData = bytearray([self.AX_START])
        outData += bytearray([self.AX_START])
        outData += bytearray([self.AX_BROADCAST_ID])
        outData += bytearray([self.AX_ACTION_LENGTH])
        outData += bytearray([self.AX_ACTION])
        outData += bytearray([self.AX_ACTION_CHECKSUM])
        self.port.write(outData)

    def setAngleLimit(self, id, cwLimit, ccwLimit, degree=False, rad=True):
        """
            set up limits for the angle
            [0, 300] -> [cwLimit, ccwLimit]
            note that, if both limits are set zeros => wheel
        """
        assert cwLimit <= ccwLimit, "cwLimits must be not larger than ccwLimits!!"
        if degree:
            cwLimit = int(cwLimit / 300.0 * 1023.0)
            ccwLimit = int(ccwLimit / 300.0 * 1023.0)
        elif rad:
            cwLimit = int(cwLimit * 180 / 3.1415926 / 300.0 * 1023.0)
            ccwLimit = int(ccwLimit * 180 / 3.1415926 / 300.0 * 1023.0)

        self.direction(self.RPI_DIRECTION_TX)
        self.port.reset_input_buffer()
        cw = [cwLimit & 0xff, cwLimit >> 8]
        ccw = [ccwLimit & 0xff, ccwLimit >> 8]
        checksum = (~(id + self.AX_AL_LENGTH + self.AX_WRITE_DATA +
                      self.AX_CW_ANGLE_LIMIT_L + cw[0] + cw[1] + ccw[0] +
                      ccw[1])) & 0xff
        outData = bytearray([self.AX_START])
        outData += bytearray([self.AX_START])
        outData += bytearray([id])
        outData += bytearray([self.AX_AL_LENGTH])
        outData += bytearray([self.AX_WRITE_DATA])
        outData += bytearray([self.AX_CW_ANGLE_LIMIT_L])
        outData += bytearray([cw[0]])
        outData += bytearray([cw[1]])
        outData += bytearray([ccw[0]])
        outData += bytearray([ccw[1]])
        outData += bytearray([checksum])
        self.port.write(outData)
        sleep(self.TX_DELAY_TIME)
        return self.readData(id)

    def readPosition(self, id):
        self.direction(self.RPI_DIRECTION_TX)
        self.port.reset_input_buffer()
        checksum = (~(id + self.AX_POS_LENGTH + self.AX_READ_DATA +
                      self.AX_PRESENT_POSITION_L + self.AX_INT_READ)) & 0xff
        outData = bytearray([self.AX_START])
        outData += bytearray([self.AX_START])
        outData += bytearray([id])
        outData += bytearray([self.AX_POS_LENGTH])
        outData += bytearray([self.AX_READ_DATA])
        outData += bytearray([self.AX_PRESENT_POSITION_L])
        outData += bytearray([self.AX_INT_READ])
        outData += bytearray([checksum])
        self.port.write(outData)
        sleep(self.TX_DELAY_TIME)
        return self.readErrorData(id)

    def learnServos(self, minValue=1, maxValue=6, verbose=False):
        servoList = []
        for i in range(minValue, maxValue + 1):
            try:
                temp = self.ping(i)
                servoList.append(i)
                if verbose:
                    print("Found servo #" + str(i))
                time.sleep(0.1)

            except Exception as detail:
                if verbose:
                    print("Error pinging servo #" + str(i) + ': ' +
                          str(detail))
                pass
        return servoList

    def ping(self, id):
        self.direction(self.RPI_DIRECTION_TX)
        self.port.reset_input_buffer()
        checksum = (~(id + self.AX_READ_DATA + self.AX_PING)) & 0xff
        outData = bytearray([self.AX_START])
        outData += bytearray([self.AX_START])
        outData += bytearray([id])
        outData += bytearray([self.AX_READ_DATA])
        outData += bytearray([self.AX_PING])
        outData += bytearray([checksum])
        self.port.write(outData)
        sleep(self.TX_DELAY_TIME)
        return self.readErrorData(id)

    def setID(self, id, newId):
        self.direction(self.RPI_DIRECTION_TX)
        self.port.reset_input_buffer()
        checksum = (~(id + self.AX_ID_LENGTH + self.AX_WRITE_DATA +
                      self.AX_ID + newId)) & 0xff
        outData = bytearray([self.AX_START])
        outData += bytearray([self.AX_START])
        outData += bytearray([id])
        outData += bytearray([self.AX_ID_LENGTH])
        outData += bytearray([self.AX_WRITE_DATA])
        outData += bytearray([self.AX_ID])
        outData += bytearray([newId])
        outData += bytearray([checksum])
        self.port.write(outData)
        sleep(self.TX_DELAY_TIME)
        return self.readData(id)
